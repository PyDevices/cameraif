// SPDX-License-Identifier: Apache-2.0
//
// Sony IMX219 (Raspberry Pi Camera Module v2 and its clones) as an
// esp_cam_sensor driver, so cameraif finds it by probing like the sensors
// Espressif ships. It registers with ESP_CAM_SENSOR_DETECT_FN, at SCCB
// address 0x10, and answers chip ID 0x0219.
//
// Registers and PLL settings follow Zephyr's IMX219 driver
// (drivers/video/imx219.c, Apache-2.0, Circuit Valley and tinyVision.ai).
// The module carries its own 24 MHz oscillator, so no XCLK is driven. Two
// CSI-2 lanes at 400 Mbps each; the pixel rate is 120 Mpixel/s, and a frame
// lasts line_length (3448) x frame_length pixel clocks.
//
// The IMX219 has no automatic exposure of its own: exposure (integration
// time, in lines) and analogue gain are set from outside, through cameraif's
// exposure() and gain(). They start at a level that suits a lit room.

#include "py/mpconfig.h"

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
#include "esp_check.h"
#include "esp_log.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_intf.h"

#define IMX219_SCCB_ADDR (0x10)
#define IMX219_PID (0x0219)
#define IMX219_NAME "IMX219"

#define IMX219_FULL_W (3280)
#define IMX219_FULL_H (2464)
#define IMX219_LINE_LENGTH (3448)
#define IMX219_PIXEL_RATE (120000000)
#define IMX219_LANE_RATE (400000000)

#define IMX219_ANGAIN_MAX (232)     // 256 / (256 - code): x1 to x10.7
#define IMX219_ANGAIN_DEFAULT (128) // x2
#define IMX219_REG_END (0xFFFF)
#define IMX219_REG_DELAY (0xFFFE)

static const char *TAG = "imx219";

typedef struct {
    uint16_t reg;
    uint8_t val;
} imx219_reg_t;

// What one mode needs beyond the common setup.
typedef struct {
    uint16_t x0, y0, x1, y1;    // the window on the full array
    uint16_t out_w, out_h;      // what leaves the sensor
    uint8_t binning;            // 0 none, 1 2x2
    uint16_t frame_length;      // lines per frame, blanking included
} imx219_mode_t;

typedef struct {
    int exposure;               // lines
    int angain;                 // gain code
    int hmirror, vflip;
} imx219_state_t;

// Unlock 0x3000-0x5fff, then the vendor's undocumented settings.
static const imx219_reg_t imx219_common_regs[] = {
    {0x0103, 0x01}, {IMX219_REG_DELAY, 6},  // software reset
    {0x30eb, 0x05}, {0x30eb, 0x0c}, {0x300a, 0xff}, {0x300b, 0xff},
    {0x30eb, 0x05}, {0x30eb, 0x09},
    {0x455e, 0x00}, {0x471e, 0x4b}, {0x4767, 0x0f}, {0x4750, 0x14},
    {0x4540, 0x00}, {0x47b4, 0x14}, {0x4713, 0x30}, {0x478b, 0x10},
    {0x478f, 0x10}, {0x4793, 0x10}, {0x4797, 0x0e}, {0x479b, 0x0e},
    {0x0114, 0x01},                         // two lanes
    {0x0128, 0x00},                         // D-PHY timing automatic
    {0x012a, 0x18}, {0x012b, 0x00},         // INCK 24 MHz
    {0x0162, IMX219_LINE_LENGTH >> 8}, {0x0163, IMX219_LINE_LENGTH & 0xff},
    {0x0170, 0x01}, {0x0171, 0x01},         // odd increments
    // PLL: VT 24 / 3 x 30 = 240 MHz, / 4 = 60 MHz (120 Mpixel/s); OP 24 / 3 x
    // 50 = 400 Mbps a lane, / 10 for RAW10.
    {0x0301, 0x04}, {0x0303, 0x01}, {0x0304, 0x03}, {0x0305, 0x03},
    {0x0306, 0x00}, {0x0307, 30}, {0x0309, 10}, {0x030b, 0x01},
    {0x030c, 0x00}, {0x030d, 50},
    {0x018c, 10}, {0x018d, 10},             // RAW10
    {0x0158, 0x01}, {0x0159, 0x00},         // digital gain x1
    {IMX219_REG_END, 0},
};

static const imx219_mode_t imx219_modes[] = {
    // 1280x960, 2x2 binned from the middle 2560x1920: 4:3, about 35 fps
    {360, 272, 360 + 2560 - 1, 272 + 1920 - 1, 1280, 960, 1, 980},
    // 1640x1232, 2x2 binned from the whole array: about 28 fps
    {0, 0, IMX219_FULL_W - 1, IMX219_FULL_H - 1, 1640, 1232, 1, 1252},
    // 1920x1080 from the middle, unbinned: 16:9, about 30 fps
    {680, 692, 680 + 1920 - 1, 692 + 1080 - 1, 1920, 1080, 0, 1100},
};

static const esp_cam_sensor_isp_info_t imx219_isp_info[] = {
    {.isp_v1_info = {.version = SENSOR_ISP_INFO_VERSION_DEFAULT, .pclk = IMX219_PIXEL_RATE,
                     .vts = 980, .hts = IMX219_LINE_LENGTH, .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB}},
    {.isp_v1_info = {.version = SENSOR_ISP_INFO_VERSION_DEFAULT, .pclk = IMX219_PIXEL_RATE,
                     .vts = 1252, .hts = IMX219_LINE_LENGTH, .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB}},
    {.isp_v1_info = {.version = SENSOR_ISP_INFO_VERSION_DEFAULT, .pclk = IMX219_PIXEL_RATE,
                     .vts = 1100, .hts = IMX219_LINE_LENGTH, .bayer_type = ESP_CAM_SENSOR_BAYER_RGGB}},
};

#define IMX219_FORMAT(i, nm, w, h, f) { \
        .name = nm, \
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10, \
        .port = ESP_CAM_SENSOR_MIPI_CSI, \
        .xclk = 24000000, \
        .width = w, \
        .height = h, \
        .regs = &imx219_modes[i], \
        .regs_size = 1, \
        .fps = f, \
        .isp_info = &imx219_isp_info[i], \
        .mipi_info = {.mipi_clk = IMX219_LANE_RATE, .lane_num = 2, .line_sync_en = false}, \
        .reserved = NULL, \
}

static const esp_cam_sensor_format_t imx219_formats[] = {
    IMX219_FORMAT(0, "MIPI_2lane_24Minput_RAW10_1280x960_binning_35fps", 1280, 960, 35),
    IMX219_FORMAT(1, "MIPI_2lane_24Minput_RAW10_1640x1232_binning_28fps", 1640, 1232, 28),
    IMX219_FORMAT(2, "MIPI_2lane_24Minput_RAW10_1920x1080_30fps", 1920, 1080, 30),
};

static esp_err_t imx219_read(esp_sccb_io_handle_t h, uint16_t reg, uint8_t *val) {
    return esp_sccb_transmit_receive_reg_a16v8(h, reg, val);
}

static esp_err_t imx219_write(esp_sccb_io_handle_t h, uint16_t reg, uint8_t val) {
    return esp_sccb_transmit_reg_a16v8(h, reg, val);
}

static esp_err_t imx219_write16(esp_sccb_io_handle_t h, uint16_t reg, uint16_t val) {
    esp_err_t ret = imx219_write(h, reg, val >> 8);
    return ret == ESP_OK ? imx219_write(h, reg + 1, val & 0xff) : ret;
}

static esp_err_t imx219_write_array(esp_sccb_io_handle_t h, const imx219_reg_t *regs) {
    esp_err_t ret = ESP_OK;
    for (int i = 0; ret == ESP_OK && regs[i].reg != IMX219_REG_END; i++) {
        if (regs[i].reg == IMX219_REG_DELAY) {
            vTaskDelay(pdMS_TO_TICKS(regs[i].val) + 1);
        } else {
            ret = imx219_write(h, regs[i].reg, regs[i].val);
        }
    }
    return ret;
}

static imx219_state_t *imx219_state(esp_cam_sensor_device_t *dev) {
    return (imx219_state_t *)dev->priv;
}

static const imx219_mode_t *imx219_mode(esp_cam_sensor_device_t *dev) {
    return dev->cur_format ? (const imx219_mode_t *)dev->cur_format->regs : &imx219_modes[0];
}

static int imx219_exposure_max(esp_cam_sensor_device_t *dev) {
    return imx219_mode(dev)->frame_length - 4;
}

static esp_err_t imx219_apply_exposure(esp_cam_sensor_device_t *dev) {
    imx219_state_t *st = imx219_state(dev);
    int max = imx219_exposure_max(dev);
    if (st->exposure > max) {
        st->exposure = max;
    }
    return imx219_write16(dev->sccb_handle, 0x015a, (uint16_t)st->exposure);
}

static esp_err_t imx219_apply_orientation(esp_cam_sensor_device_t *dev) {
    imx219_state_t *st = imx219_state(dev);
    return imx219_write(dev->sccb_handle, 0x0172, (st->vflip ? 0x02 : 0) | (st->hmirror ? 0x01 : 0));
}

static esp_err_t imx219_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id) {
    uint8_t hi, lo;
    ESP_RETURN_ON_ERROR(imx219_read(dev->sccb_handle, 0x0000, &hi), TAG, "read chip id");
    ESP_RETURN_ON_ERROR(imx219_read(dev->sccb_handle, 0x0001, &lo), TAG, "read chip id");
    id->pid = (uint16_t)(hi << 8 | lo);
    return ESP_OK;
}

static esp_err_t imx219_set_stream(esp_cam_sensor_device_t *dev, int enable) {
    esp_err_t ret = imx219_write(dev->sccb_handle, 0x0100, enable ? 0x01 : 0x00);
    if (ret == ESP_OK) {
        dev->stream_status = enable;
    }
    return ret;
}

static esp_err_t imx219_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format) {
    if (format == NULL) {
        format = &imx219_formats[0];
    }
    const imx219_mode_t *m = (const imx219_mode_t *)format->regs;
    esp_sccb_io_handle_t h = dev->sccb_handle;
    ESP_RETURN_ON_ERROR(imx219_write_array(h, imx219_common_regs), TAG, "common regs");
    esp_err_t ret = imx219_write16(h, 0x0164, m->x0);
    ret |= imx219_write16(h, 0x0166, m->x1);
    ret |= imx219_write16(h, 0x0168, m->y0);
    ret |= imx219_write16(h, 0x016a, m->y1);
    ret |= imx219_write16(h, 0x016c, m->out_w);
    ret |= imx219_write16(h, 0x016e, m->out_h);
    ret |= imx219_write(h, 0x0174, m->binning);
    ret |= imx219_write(h, 0x0175, m->binning);
    ret |= imx219_write16(h, 0x0160, m->frame_length);
    ret |= imx219_write16(h, 0x0624, m->out_w);     // test pattern window
    ret |= imx219_write16(h, 0x0626, m->out_h);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ESP_FAIL, TAG, "mode regs");
    dev->cur_format = format;
    ESP_RETURN_ON_ERROR(imx219_apply_exposure(dev), TAG, "exposure");
    ESP_RETURN_ON_ERROR(imx219_write(h, 0x0157, (uint8_t)imx219_state(dev)->angain), TAG, "gain");
    ESP_RETURN_ON_ERROR(imx219_apply_orientation(dev), TAG, "orientation");
    return imx219_set_stream(dev, 0);
}

static esp_err_t imx219_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format) {
    if (dev->cur_format == NULL) {
        return ESP_FAIL;
    }
    memcpy(format, dev->cur_format, sizeof(*format));
    return ESP_OK;
}

static esp_err_t imx219_query_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats) {
    formats->count = sizeof(imx219_formats) / sizeof(imx219_formats[0]);
    formats->format_array = &imx219_formats[0];
    return ESP_OK;
}

static esp_err_t imx219_query_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *cap) {
    cap->fmt_raw = 1;
    return ESP_OK;
}

static esp_err_t imx219_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *q) {
    q->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
    q->number.step = 1;
    switch (q->id) {
        case ESP_CAM_SENSOR_EXPOSURE_VAL:
            q->number.minimum = 1;
            q->number.maximum = imx219_exposure_max(dev);
            q->default_value = q->number.maximum;
            return ESP_OK;
        case ESP_CAM_SENSOR_ANGAIN:
            q->number.minimum = 0;
            q->number.maximum = IMX219_ANGAIN_MAX;
            q->default_value = IMX219_ANGAIN_DEFAULT;
            return ESP_OK;
        case ESP_CAM_SENSOR_VFLIP:
        case ESP_CAM_SENSOR_HMIRROR:
            q->number.minimum = 0;
            q->number.maximum = 1;
            q->default_value = 0;
            return ESP_OK;
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t imx219_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size) {
    imx219_state_t *st = imx219_state(dev);
    int *v = (int *)arg;
    if (size < sizeof(int)) {
        return ESP_ERR_INVALID_SIZE;
    }
    switch (id) {
        case ESP_CAM_SENSOR_EXPOSURE_VAL:
            *v = st->exposure;
            return ESP_OK;
        case ESP_CAM_SENSOR_ANGAIN:
            *v = st->angain;
            return ESP_OK;
        case ESP_CAM_SENSOR_VFLIP:
            *v = st->vflip;
            return ESP_OK;
        case ESP_CAM_SENSOR_HMIRROR:
            *v = st->hmirror;
            return ESP_OK;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t imx219_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size) {
    imx219_state_t *st = imx219_state(dev);
    int v = *(const int *)arg;
    switch (id) {
        case ESP_CAM_SENSOR_EXPOSURE_VAL:
            st->exposure = v < 1 ? 1 : v;
            return imx219_apply_exposure(dev);
        case ESP_CAM_SENSOR_ANGAIN:
            st->angain = v < 0 ? 0 : v > IMX219_ANGAIN_MAX ? IMX219_ANGAIN_MAX : v;
            return imx219_write(dev->sccb_handle, 0x0157, (uint8_t)st->angain);
        case ESP_CAM_SENSOR_VFLIP:
            st->vflip = v != 0;
            return imx219_apply_orientation(dev);
        case ESP_CAM_SENSOR_HMIRROR:
            st->hmirror = v != 0;
            return imx219_apply_orientation(dev);
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t imx219_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg) {
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
            ret = imx219_write(dev->sccb_handle, 0x0103, 0x01);
            vTaskDelay(pdMS_TO_TICKS(6) + 1);
            return ret;
        case ESP_CAM_SENSOR_IOC_S_STREAM:
            return imx219_set_stream(dev, *(int *)arg);
        case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
            // 0x0600: 0 off, 2 colour bars
            return imx219_write16(dev->sccb_handle, 0x0600, *(int *)arg ? 2 : 0);
        case ESP_CAM_SENSOR_IOC_S_REG:
            r = (esp_cam_sensor_reg_val_t *)arg;
            return imx219_write(dev->sccb_handle, r->regaddr, r->value);
        case ESP_CAM_SENSOR_IOC_G_REG:
            r = (esp_cam_sensor_reg_val_t *)arg;
            ret = imx219_read(dev->sccb_handle, r->regaddr, &val);
            if (ret == ESP_OK) {
                r->value = val;
            }
            return ret;
        case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
            return imx219_get_sensor_id(dev, arg);
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t imx219_delete(esp_cam_sensor_device_t *dev) {
    if (dev) {
        free(dev->priv);
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t imx219_ops = {
    .query_para_desc = imx219_query_para_desc,
    .get_para_value = imx219_get_para_value,
    .set_para_value = imx219_set_para_value,
    .query_support_formats = imx219_query_formats,
    .query_support_capability = imx219_query_capability,
    .set_format = imx219_set_format,
    .get_format = imx219_get_format,
    .priv_ioctl = imx219_priv_ioctl,
    .del = imx219_delete,
};

static void imx219_power_on(esp_cam_sensor_device_t *dev) {
    // Pi camera boards tie their enable to the connector's GPIO line; a board
    // that wires it gives it as pwdn or reset, active-low either way here.
    int pins[2] = {dev->pwdn_pin, dev->reset_pin};
    for (int i = 0; i < 2; i++) {
        if (pins[i] >= 0) {
            gpio_config_t conf = {.pin_bit_mask = 1ULL << pins[i], .mode = GPIO_MODE_OUTPUT};
            gpio_config(&conf);
            gpio_set_level(pins[i], i == 0 ? 0 : 1);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

esp_cam_sensor_device_t *imx219_detect(esp_cam_sensor_config_t *config) {
    if (config == NULL) {
        return NULL;
    }
    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    imx219_state_t *st = calloc(1, sizeof(imx219_state_t));
    if (dev == NULL || st == NULL) {
        free(dev);
        free(st);
        return NULL;
    }
    dev->name = (char *)IMX219_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &imx219_ops;
    dev->priv = st;
    dev->cur_format = &imx219_formats[0];
    st->exposure = imx219_modes[0].frame_length - 4;
    st->angain = IMX219_ANGAIN_DEFAULT;
    imx219_power_on(dev);
    if (imx219_get_sensor_id(dev, &dev->id) != ESP_OK || dev->id.pid != IMX219_PID) {
        ESP_LOGD(TAG, "not an IMX219 (PID 0x%x)", dev->id.pid);
        imx219_delete(dev);
        return NULL;
    }
    ESP_LOGI(TAG, "Detected Camera sensor PID=0x%x", dev->id.pid);
    return dev;
}

ESP_CAM_SENSOR_DETECT_FN(imx219_detect, ESP_CAM_SENSOR_MIPI_CSI, IMX219_SCCB_ADDR) {
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return imx219_detect(config);
}

#endif // CONFIG_IDF_TARGET_ESP32P4
