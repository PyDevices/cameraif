// SPDX-License-Identifier: MIT
//
// A camera sensor described by data rather than code. Camera(sensor=...)
// hands this a dict -- the SCCB address, the chip-ID check, the register
// lists that set it up, its modes, and where its exposure, gain, flip and
// test pattern live -- and it becomes an esp_cam_sensor device like the
// drivers Espressif ships, so the rest of cameraif can't tell the difference.
// Adding a camera is a Python file, not a firmware build.
//
// Everything is copied out of the dict into the C heap: the sensor outlives
// any one Python call, and the soft-reset teardown runs after the Python heap
// is gone.

#include "py/runtime.h"
#include "py/obj.h"

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

#if defined(CONFIG_IDF_TARGET_ESP32P4) && CONFIG_IDF_TARGET_ESP32P4

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_cam_sensor.h"
#include "esp_sccb_intf.h"

#include "sensor_table.h"

#define TABLE_DELAY (0xFFFF)    // a register entry that waits `value` ms instead

typedef struct {
    uint16_t reg;
    uint16_t val;
} table_reg_t;

typedef struct {
    table_reg_t *regs;
    size_t n;
} table_list_t;

typedef struct {
    int reg, bytes, min, max, def; // max <= 0 counts back from the mode's frame length
    bool have;
} table_ctrl_t;

typedef struct {
    esp_cam_sensor_format_t *formats;
    esp_cam_sensor_isp_info_t *isp;
    table_list_t *mode_regs;
    size_t n_modes;
    table_list_t init, stream_on, stream_off;
    table_ctrl_t exposure, gain, pattern;
    int pattern_on, pattern_off;
    int flip_reg, flip_v, flip_h;
    int id_reg, id_value;
    bool reg16;
    // state
    int exp, gain_v, vflip, hmirror;
    char name[16];
} table_t;

// --- parsing ------------------------------------------------------------

static mp_obj_t table_get(mp_obj_t d, qstr key) {
    mp_map_elem_t *e = mp_map_lookup(mp_obj_dict_get_map(d), MP_OBJ_NEW_QSTR(key), MP_MAP_LOOKUP);
    return e ? e->value : MP_OBJ_NULL;
}

// A key the description can't do without.
static mp_obj_t table_need(mp_obj_t d, qstr key) {
    if (!mp_obj_is_dict_or_ordereddict(d)) {
        mp_raise_TypeError(MP_ERROR_TEXT("a sensor description and its modes are dicts"));
    }
    mp_obj_t v = table_get(d, key);
    if (v == MP_OBJ_NULL || v == mp_const_none) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("sensor description needs '%q'"), key);
    }
    return v;
}

static int table_int(mp_obj_t d, qstr key, int dflt) {
    mp_obj_t v = table_get(d, key);
    return v == MP_OBJ_NULL || v == mp_const_none ? dflt : mp_obj_get_int(v);
}

// [(reg, value), (None, ms), ...]
static void table_parse_list(mp_obj_t seq, table_list_t *out) {
    out->regs = NULL;
    out->n = 0;
    if (seq == MP_OBJ_NULL || seq == mp_const_none) {
        return;
    }
    size_t n;
    mp_obj_t *items;
    mp_obj_get_array(seq, &n, &items);
    out->regs = calloc(n ? n : 1, sizeof(table_reg_t));
    if (out->regs == NULL) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("sensor table"));
    }
    for (size_t i = 0; i < n; i++) {
        size_t m;
        mp_obj_t *pair;
        mp_obj_get_array(items[i], &m, &pair);
        if (m != 2) {
            mp_raise_ValueError(MP_ERROR_TEXT("sensor registers are (register, value) pairs"));
        }
        out->regs[i].reg = pair[0] == mp_const_none ? TABLE_DELAY : (uint16_t)mp_obj_get_int(pair[0]);
        out->regs[i].val = (uint16_t)mp_obj_get_int(pair[1]);
    }
    out->n = n;
}

// (register, bytes, minimum, maximum, default)
static void table_parse_ctrl(mp_obj_t v, table_ctrl_t *out) {
    out->have = false;
    if (v == MP_OBJ_NULL || v == mp_const_none) {
        return;
    }
    size_t n;
    mp_obj_t *it;
    mp_obj_get_array(v, &n, &it);
    if (n != 5) {
        mp_raise_ValueError(MP_ERROR_TEXT("a sensor control is (register, bytes, min, max, default)"));
    }
    out->reg = mp_obj_get_int(it[0]);
    out->bytes = mp_obj_get_int(it[1]);
    out->min = mp_obj_get_int(it[2]);
    out->max = mp_obj_get_int(it[3]);
    out->def = it[4] == mp_const_none ? out->max : mp_obj_get_int(it[4]);
    out->have = true;
}

static int table_bayer(const char *s) {
    static const char *names[] = { "RGGB", "GRBG", "GBRG", "BGGR" };
    static const int vals[] = { ESP_CAM_SENSOR_BAYER_RGGB, ESP_CAM_SENSOR_BAYER_GRBG,
                                ESP_CAM_SENSOR_BAYER_GBRG, ESP_CAM_SENSOR_BAYER_BGGR };
    for (int i = 0; i < 4; i++) {
        if (strcmp(s, names[i]) == 0) {
            return vals[i];
        }
    }
    mp_raise_ValueError(MP_ERROR_TEXT("bayer is one of RGGB, GRBG, GBRG, BGGR"));
}

static int table_pixformat(const char *s) {
    if (strcmp(s, "raw8") == 0) {
        return ESP_CAM_SENSOR_PIXFORMAT_RAW8;
    } else if (strcmp(s, "raw10") == 0) {
        return ESP_CAM_SENSOR_PIXFORMAT_RAW10;
    } else if (strcmp(s, "raw12") == 0) {
        return ESP_CAM_SENSOR_PIXFORMAT_RAW12;
    }
    mp_raise_ValueError(MP_ERROR_TEXT("format is raw8, raw10 or raw12"));
}

static void table_free(table_t *t) {
    if (t == NULL) {
        return;
    }
    for (size_t i = 0; i < t->n_modes; i++) {
        if (t->mode_regs) {
            free(t->mode_regs[i].regs);
        }
        if (t->formats) {
            free((void *)t->formats[i].name);
        }
    }
    free(t->formats);
    free(t->isp);
    free(t->mode_regs);
    free(t->init.regs);
    free(t->stream_on.regs);
    free(t->stream_off.regs);
    free(t);
}

void *cameraif_table_parse(mp_obj_t desc, uint8_t *addr) {
    if (!mp_obj_is_dict_or_ordereddict(desc)) {
        mp_raise_TypeError(MP_ERROR_TEXT("sensor= takes a sensor description dict"));
    }
    table_t *t = calloc(1, sizeof(table_t));
    if (t == NULL) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("sensor table"));
    }
    // Free it if anything below raises.
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t name = table_get(desc, MP_QSTR_name);
        strncpy(t->name, name ? mp_obj_str_get_str(name) : "sensor", sizeof(t->name) - 1);
        *addr = (uint8_t)mp_obj_get_int(table_need(desc, MP_QSTR_addr));
        t->reg16 = table_int(desc, MP_QSTR_reg_bits, 16) == 16;
        size_t n;
        mp_obj_t *it;
        mp_obj_get_array(table_need(desc, MP_QSTR_id), &n, &it);
        if (n != 2) {
            mp_raise_ValueError(MP_ERROR_TEXT("id is (register, value)"));
        }
        t->id_reg = mp_obj_get_int(it[0]);
        t->id_value = mp_obj_get_int(it[1]);

        table_parse_list(table_get(desc, MP_QSTR_init), &t->init);
        mp_obj_t stream = table_get(desc, MP_QSTR_stream);
        if (stream != MP_OBJ_NULL && stream != mp_const_none) {
            mp_obj_get_array(stream, &n, &it);
            if (n != 2) {
                mp_raise_ValueError(MP_ERROR_TEXT("stream is (registers on, registers off)"));
            }
            table_parse_list(it[0], &t->stream_on);
            table_parse_list(it[1], &t->stream_off);
        }
        table_parse_ctrl(table_get(desc, MP_QSTR_exposure), &t->exposure);
        table_parse_ctrl(table_get(desc, MP_QSTR_gain), &t->gain);
        mp_obj_t tp = table_get(desc, MP_QSTR_test_pattern);
        if (tp != MP_OBJ_NULL && tp != mp_const_none) {
            mp_obj_get_array(tp, &n, &it);       // (register, bytes, on, off)
            if (n != 4) {
                mp_raise_ValueError(MP_ERROR_TEXT("test_pattern is (register, bytes, on, off)"));
            }
            t->pattern.reg = mp_obj_get_int(it[0]);
            t->pattern.bytes = mp_obj_get_int(it[1]);
            t->pattern_on = mp_obj_get_int(it[2]);
            t->pattern_off = mp_obj_get_int(it[3]);
            t->pattern.have = true;
        }
        t->flip_reg = -1;
        mp_obj_t fl = table_get(desc, MP_QSTR_flip);
        if (fl != MP_OBJ_NULL && fl != mp_const_none) {
            mp_obj_get_array(fl, &n, &it);       // (register, vertical bits, horizontal bits)
            if (n != 3) {
                mp_raise_ValueError(MP_ERROR_TEXT("flip is (register, vertical bits, horizontal bits)"));
            }
            t->flip_reg = mp_obj_get_int(it[0]);
            t->flip_v = mp_obj_get_int(it[1]);
            t->flip_h = mp_obj_get_int(it[2]);
        }

        mp_obj_t *ms;
        size_t n_modes;
        mp_obj_get_array(table_need(desc, MP_QSTR_modes), &n_modes, &ms);
        if (n_modes == 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("a sensor needs at least one mode"));
        }
        t->formats = calloc(n_modes, sizeof(esp_cam_sensor_format_t));
        t->isp = calloc(n_modes, sizeof(esp_cam_sensor_isp_info_t));
        t->mode_regs = calloc(n_modes, sizeof(table_list_t));
        if (!t->formats || !t->isp || !t->mode_regs) {
            mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("sensor table"));
        }
        for (size_t i = 0; i < n_modes; i++) {
            mp_obj_t m = ms[i];
            esp_cam_sensor_format_t *f = &t->formats[i];
            t->n_modes = i + 1;     // so a raise below frees what this mode has
            f->name = strdup(mp_obj_str_get_str(table_need(m, MP_QSTR_name)));
            f->format = table_pixformat(mp_obj_str_get_str(table_need(m, MP_QSTR_format)));
            f->port = ESP_CAM_SENSOR_MIPI_CSI;
            f->xclk = table_int(m, MP_QSTR_xclk, 24000000);
            f->width = mp_obj_get_int(table_need(m, MP_QSTR_width));
            f->height = mp_obj_get_int(table_need(m, MP_QSTR_height));
            f->fps = table_int(m, MP_QSTR_fps, 30);
            f->mipi_info.mipi_clk = mp_obj_get_int(table_need(m, MP_QSTR_lane_hz));
            f->mipi_info.lane_num = table_int(m, MP_QSTR_lanes, 2);
            f->mipi_info.line_sync_en = false;
            // version is const in the struct, so the whole thing is copied in.
            const esp_cam_sensor_isp_info_t isp = { .isp_v1_info = {
                .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
                .pclk = table_int(m, MP_QSTR_pclk, 0),
                .hts = table_int(m, MP_QSTR_hts, 0),
                .vts = mp_obj_get_int(table_need(m, MP_QSTR_vts)),
                .bayer_type = table_bayer(mp_obj_str_get_str(table_need(m, MP_QSTR_bayer))),
            } };
            memcpy(&t->isp[i], &isp, sizeof(isp));
            f->isp_info = &t->isp[i];
            table_parse_list(table_get(m, MP_QSTR_regs), &t->mode_regs[i]);
            f->regs = &t->mode_regs[i];
            f->regs_size = 1;
        }
        nlr_pop();
    } else {
        table_free(t);
        nlr_jump(nlr.ret_val);
    }
    return t;
}

void cameraif_table_free(void *t) {
    table_free((table_t *)t);
}

// --- registers ------------------------------------------------------------

static esp_err_t table_write(esp_cam_sensor_device_t *dev, uint16_t reg, uint8_t val) {
    table_t *t = dev->priv;
    return t->reg16 ? esp_sccb_transmit_reg_a16v8(dev->sccb_handle, reg, val)
                    : esp_sccb_transmit_reg_a8v8(dev->sccb_handle, (uint8_t)reg, val);
}

static esp_err_t table_read(esp_cam_sensor_device_t *dev, uint16_t reg, uint8_t *val) {
    table_t *t = dev->priv;
    return t->reg16 ? esp_sccb_transmit_receive_reg_a16v8(dev->sccb_handle, reg, val)
                    : esp_sccb_transmit_receive_reg_a8v8(dev->sccb_handle, (uint8_t)reg, val);
}

// A value over `bytes` registers, most significant first.
static esp_err_t table_write_n(esp_cam_sensor_device_t *dev, int reg, int bytes, int v) {
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < bytes && ret == ESP_OK; i++) {
        ret = table_write(dev, reg + i, (uint8_t)(v >> (8 * (bytes - 1 - i))));
    }
    return ret;
}

static esp_err_t table_write_list(esp_cam_sensor_device_t *dev, const table_list_t *l) {
    for (size_t i = 0; i < l->n; i++) {
        if (l->regs[i].reg == TABLE_DELAY) {
            vTaskDelay(pdMS_TO_TICKS(l->regs[i].val) + 1);
            continue;
        }
        esp_err_t ret = table_write(dev, l->regs[i].reg, (uint8_t)l->regs[i].val);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

static int table_vts(esp_cam_sensor_device_t *dev) {
    const esp_cam_sensor_format_t *f = dev->cur_format;
    return f && f->isp_info ? (int)f->isp_info->isp_v1_info.vts : 0;
}

static int table_exp_max(esp_cam_sensor_device_t *dev) {
    table_t *t = dev->priv;
    return t->exposure.max > 0 ? t->exposure.max : table_vts(dev) + t->exposure.max;
}

static esp_err_t table_apply_exposure(esp_cam_sensor_device_t *dev) {
    table_t *t = dev->priv;
    if (!t->exposure.have) {
        return ESP_OK;
    }
    int max = table_exp_max(dev);
    if (t->exp > max) {
        t->exp = max;
    }
    if (t->exp < t->exposure.min) {
        t->exp = t->exposure.min;
    }
    return table_write_n(dev, t->exposure.reg, t->exposure.bytes, t->exp);
}

static esp_err_t table_apply_gain(esp_cam_sensor_device_t *dev) {
    table_t *t = dev->priv;
    return t->gain.have ? table_write_n(dev, t->gain.reg, t->gain.bytes, t->gain_v) : ESP_OK;
}

static esp_err_t table_apply_orientation(esp_cam_sensor_device_t *dev) {
    table_t *t = dev->priv;
    if (t->flip_reg < 0) {
        return ESP_OK;
    }
    uint8_t v = 0;
    table_read(dev, t->flip_reg, &v);
    v &= ~(t->flip_v | t->flip_h);
    v |= (t->vflip ? t->flip_v : 0) | (t->hmirror ? t->flip_h : 0);
    return table_write(dev, t->flip_reg, v);
}

static esp_err_t table_read_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id) {
    table_t *t = dev->priv;
    uint8_t hi = 0, lo = 0;
    esp_err_t ret = table_read(dev, t->id_reg, &hi);
    if (ret == ESP_OK) {
        ret = table_read(dev, t->id_reg + 1, &lo);
    }
    if (ret == ESP_OK) {
        id->pid = (uint16_t)(hi << 8 | lo);
    }
    return ret;
}

// --- esp_cam_sensor ops ---------------------------------------------------

static esp_err_t table_set_stream(esp_cam_sensor_device_t *dev, int enable) {
    table_t *t = dev->priv;
    esp_err_t ret = table_write_list(dev, enable ? &t->stream_on : &t->stream_off);
    if (ret == ESP_OK) {
        dev->stream_status = enable;
    }
    return ret;
}

static esp_err_t table_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format) {
    table_t *t = dev->priv;
    if (format == NULL) {
        format = &t->formats[0];
    }
    esp_err_t ret = table_write_list(dev, &t->init);
    if (ret == ESP_OK) {
        ret = table_write_list(dev, (const table_list_t *)format->regs);
    }
    if (ret != ESP_OK) {
        return ret;
    }
    dev->cur_format = format;
    table_apply_exposure(dev);
    table_apply_gain(dev);
    table_apply_orientation(dev);
    return table_set_stream(dev, 0);
}

static esp_err_t table_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format) {
    if (dev->cur_format == NULL) {
        return ESP_FAIL;
    }
    memcpy(format, dev->cur_format, sizeof(*format));
    return ESP_OK;
}

static esp_err_t table_query_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats) {
    table_t *t = dev->priv;
    formats->count = t->n_modes;
    formats->format_array = t->formats;
    return ESP_OK;
}

static esp_err_t table_query_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *cap) {
    cap->fmt_raw = 1;
    return ESP_OK;
}

static esp_err_t table_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *q) {
    table_t *t = dev->priv;
    q->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
    q->number.step = 1;
    switch (q->id) {
        case ESP_CAM_SENSOR_EXPOSURE_VAL:
            if (!t->exposure.have) {
                return ESP_ERR_INVALID_ARG;
            }
            q->number.minimum = t->exposure.min;
            q->number.maximum = table_exp_max(dev);
            q->default_value = q->number.maximum;
            return ESP_OK;
        case ESP_CAM_SENSOR_ANGAIN:
            if (!t->gain.have) {
                return ESP_ERR_INVALID_ARG;
            }
            q->number.minimum = t->gain.min;
            q->number.maximum = t->gain.max;
            q->default_value = t->gain.def;
            return ESP_OK;
        case ESP_CAM_SENSOR_VFLIP:
        case ESP_CAM_SENSOR_HMIRROR:
            if (t->flip_reg < 0) {
                return ESP_ERR_INVALID_ARG;
            }
            q->number.minimum = 0;
            q->number.maximum = 1;
            q->default_value = 0;
            return ESP_OK;
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t table_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size) {
    table_t *t = dev->priv;
    int *v = (int *)arg;
    if (size < sizeof(int)) {
        return ESP_ERR_INVALID_SIZE;
    }
    switch (id) {
        case ESP_CAM_SENSOR_EXPOSURE_VAL:
            *v = t->exp;
            return t->exposure.have ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
        case ESP_CAM_SENSOR_ANGAIN:
            *v = t->gain_v;
            return t->gain.have ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
        case ESP_CAM_SENSOR_VFLIP:
            *v = t->vflip;
            return ESP_OK;
        case ESP_CAM_SENSOR_HMIRROR:
            *v = t->hmirror;
            return ESP_OK;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t table_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size) {
    table_t *t = dev->priv;
    int v = *(const int *)arg;
    switch (id) {
        case ESP_CAM_SENSOR_EXPOSURE_VAL:
            t->exp = v;
            return table_apply_exposure(dev);
        case ESP_CAM_SENSOR_ANGAIN:
            t->gain_v = v < t->gain.min ? t->gain.min : (v > t->gain.max ? t->gain.max : v);
            return table_apply_gain(dev);
        case ESP_CAM_SENSOR_VFLIP:
            t->vflip = v != 0;
            return table_apply_orientation(dev);
        case ESP_CAM_SENSOR_HMIRROR:
            t->hmirror = v != 0;
            return table_apply_orientation(dev);
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t table_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg) {
    table_t *t = dev->priv;
    esp_cam_sensor_reg_val_t *r;
    uint8_t val;
    esp_err_t ret;
    switch (cmd) {
        case ESP_CAM_SENSOR_IOC_HW_RESET:
            if (dev->reset_pin >= 0) {
                gpio_set_level(dev->reset_pin, 0);
                vTaskDelay(pdMS_TO_TICKS(10));
                gpio_set_level(dev->reset_pin, 1);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            return ESP_OK;
        case ESP_CAM_SENSOR_IOC_SW_RESET:
            return ESP_OK;      // the init list starts with the sensor's own reset, if it has one
        case ESP_CAM_SENSOR_IOC_S_STREAM:
            return table_set_stream(dev, *(int *)arg);
        case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
            if (!t->pattern.have) {
                return ESP_ERR_NOT_SUPPORTED;
            }
            return table_write_n(dev, t->pattern.reg, t->pattern.bytes,
                *(int *)arg ? t->pattern_on : t->pattern_off);
        case ESP_CAM_SENSOR_IOC_S_REG:
            r = (esp_cam_sensor_reg_val_t *)arg;
            return table_write(dev, r->regaddr, r->value);
        case ESP_CAM_SENSOR_IOC_G_REG:
            r = (esp_cam_sensor_reg_val_t *)arg;
            ret = table_read(dev, r->regaddr, &val);
            if (ret == ESP_OK) {
                r->value = val;
            }
            return ret;
        case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
            return table_read_id(dev, arg);
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t table_delete(esp_cam_sensor_device_t *dev) {
    if (dev) {
        table_free(dev->priv);
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t table_ops = {
    .query_para_desc = table_query_para_desc,
    .get_para_value = table_get_para_value,
    .set_para_value = table_set_para_value,
    .query_support_formats = table_query_formats,
    .query_support_capability = table_query_capability,
    .set_format = table_set_format,
    .get_format = table_get_format,
    .priv_ioctl = table_priv_ioctl,
    .del = table_delete,
};

// Takes ownership of the table: freed here on failure, by the device's
// delete on success.
esp_cam_sensor_device_t *cameraif_table_detect(void *table, esp_cam_sensor_config_t *config) {
    table_t *t = table;
    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        table_free(t);
        return NULL;
    }
    dev->name = t->name;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    dev->ops = &table_ops;
    dev->priv = t;
    dev->cur_format = &t->formats[0];
    t->exp = t->exposure.def > 0 ? t->exposure.def : table_vts(dev) + t->exposure.def;
    t->gain_v = t->gain.def;

    // A Pi-style camera ties its enable to the connector's GPIO; a board that
    // wires it gives it as pwdn (held low) or reset (held high).
    int pins[2] = { dev->pwdn_pin, dev->reset_pin };
    for (int i = 0; i < 2; i++) {
        if (pins[i] >= 0) {
            gpio_config_t conf = { .pin_bit_mask = 1ULL << pins[i], .mode = GPIO_MODE_OUTPUT };
            gpio_config(&conf);
            gpio_set_level(pins[i], i == 0 ? 0 : 1);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    if (table_read_id(dev, &dev->id) != ESP_OK || dev->id.pid != t->id_value) {
        table_delete(dev);
        return NULL;
    }
    return dev;
}

#endif // CONFIG_IDF_TARGET_ESP32P4
