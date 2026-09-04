// SPDX-License-Identifier: MIT
//
// cameraif: a camera sensor as a MicroPython native module.
//
// ESP-IDF ships the camera *controllers* (esp_driver_cam: CSI, DVP, SPI) and
// no sensor drivers at all. The sensor half comes from the
// espressif/esp_cam_sensor managed component, which puts 37 drivers --
// OmniVision, GalaxyCore, SmartSens, Arducam, Onsemi, Toshiba -- behind one
// interface. Nothing here is written for a particular sensor: resolution,
// frame rate, MIPI lane count and lane bit rate are all read from whatever
// driver answers on the bus. Adding a manufacturer is one sdkconfig line.
//
// The single exception is a quirk workaround for the OV5647, gated on that
// sensor's product ID, and documented where it is applied.
//
// Board facts -- pins, LDO channel, control signals -- are arguments rather
// than constants, because this module has no business knowing which board it
// is on. `pydevices` already records that per board in `board_config` /
// `board_peripherals`, and its camera() passes them here.
//
// Lifecycle follows displayif's idempotent-lifecycle contract: the
// constructor tears down any previous camera before building a new one,
// deinit() is idempotent, __del__ is best-effort, and a soft-reset hook runs
// teardown before the heap is wiped. That last one is not optional -- a soft
// reset clears MicroPython's heap and leaves ESP-IDF's CSI, ISP, LDO and I2C
// allocations live, so without it the second construction fails on resources
// the first still holds. Measured exactly that way before it was fixed:
// "camera already open" after every Ctrl-D.

#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"
#include "py/objarray.h"

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

#if defined(CONFIG_IDF_TARGET_ESP32P4) && CONFIG_IDF_TARGET_ESP32P4

#include <string.h>

#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "esp_private/esp_cache_private.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "driver/jpeg_encode.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// displayif owns the pre-gc-sweep hook (it wraps gc_sweep_all for the whole
// firmware). Declared weak so cameraif still links and runs in a build
// without displayif -- there it simply has no soft-reset teardown, and
// deinit() remains the working path.
extern void displayif_register_soft_reset(void (*fn)(void)) __attribute__((weak));
extern void displayif_unregister_soft_reset(void (*fn)(void)) __attribute__((weak));

#define CAMERAIF_OV5647_PID (0x5647)

typedef struct {
    mp_obj_base_t base;
    bool open;
    bool raw;
    uint16_t width, height;
    size_t frame_bytes;
    uint8_t bytes_per_px;
    uint8_t *fb[2];
    uint8_t *frame;             // the buffer Python may read; never queued
    // Which buffer belongs to whom. The DMA and the Python reader must never
    // hold the same one: at 35 fps the driver refills a buffer every 28 ms
    // while a 1.28 MB read takes ~100 ms, so a single buffer tears three
    // times per frame. A still subject cannot show that -- every pixel it
    // tears to is the same pixel -- which is why one buffer looked correct.
    volatile int8_t fb_filling;  // index the driver is writing
    volatile int8_t fb_held;     // index Python is reading, or -1
    volatile int8_t fb_done;     // index of the most recently completed frame
    bool test_pattern;           // last requested; the ioctl has no read side
    esp_cam_ctlr_handle_t cam;
    isp_proc_handle_t isp;
    jpeg_encoder_handle_t jpeg;
    uint8_t *jpeg_buf;
    size_t jpeg_buf_size;
    esp_ldo_channel_handle_t ldo;
    i2c_master_bus_handle_t i2c;
    esp_sccb_io_handle_t sccb;
    esp_cam_sensor_device_t *sensor;
    SemaphoreHandle_t frame_ready;
    volatile uint32_t frames, dropped, isr_new, isr_done, last_received;
} cameraif_obj_t;

// One camera, mirrored in BSS.
//
// Not a limitation dressed as a design: the P4 has one CSI controller, and
// the soft-reset hook has to reach the hardware after the Python object is
// gone. displayif's mipidsi keeps its handles the same way and for the same
// reason.
static cameraif_obj_t cameraif_singleton;

static void cameraif_teardown(void);

// --- ISR callbacks -------------------------------------------------------

static bool IRAM_ATTR cameraif_on_get_new_trans(esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans, void *user_data) {
    (void)handle;
    (void)user_data;
    cameraif_obj_t *c = &cameraif_singleton;
    // Whatever Python is not holding. With two buffers and one held, the
    // driver keeps refilling the other -- overwriting frames the reader was
    // too slow for, which is what a preview wants anyway.
    c->fb_filling = (c->fb_held == 0) ? 1 : 0;
    trans->buffer = c->fb[c->fb_filling];
    trans->buflen = c->frame_bytes;
    c->isr_new++;
    return false;
}

static bool IRAM_ATTR cameraif_on_trans_finished(esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans, void *user_data) {
    (void)handle;
    (void)user_data;
    cameraif_obj_t *c = &cameraif_singleton;
    BaseType_t woken = pdFALSE;
    c->isr_done++;
    c->last_received = (uint32_t)trans->received_size;
    c->fb_done = (int8_t)c->fb_filling;
    if (c->frame_ready) {
        // Give, never take: a frame arriving while the last is unread is not
        // an error, it is the normal case at 50 fps with a Python reader.
        xSemaphoreGiveFromISR(c->frame_ready, &woken);
    }
    return woken == pdTRUE;
}

// --- helpers -------------------------------------------------------------

static void cameraif_set_para(uint32_t id, int32_t v) {
    cameraif_obj_t *c = &cameraif_singleton;
    if (c->sensor) {
        esp_cam_sensor_set_para_value(c->sensor, id, &v, sizeof(v));
    }
}

// Set one parameter to `percent` of the range the sensor advertises.
//
// Asking the sensor beats hard-coding: units differ per parameter (exposure
// in lines, gain in driver-specific steps) and per format, so a literal that
// suits one mode is wrong in another. Quiet on failure -- a sensor without
// exposure control is not a broken camera.
static void cameraif_set_para_fraction(uint32_t id, int percent) {
    cameraif_obj_t *c = &cameraif_singleton;
    esp_cam_sensor_param_desc_t desc = { .id = id };
    if (!c->sensor || esp_cam_sensor_query_para_desc(c->sensor, &desc) != ESP_OK) {
        return;
    }
    const int32_t lo = desc.number.minimum, hi = desc.number.maximum;
    if (hi <= lo) {
        return;
    }
    int32_t v = lo + (int32_t)(((int64_t)(hi - lo) * percent) / 100);
    if (desc.number.step > 1) {
        v -= (v - lo) % (int32_t)desc.number.step;
    }
    cameraif_set_para(id, v);
}

static bool cameraif_queue_buffer(void) {
    cameraif_obj_t *c = &cameraif_singleton;
    c->fb_filling = (c->fb_held == 0) ? 1 : 0;
    esp_cam_ctlr_trans_t trans = {
        .buffer = c->fb[c->fb_filling], .buflen = c->frame_bytes
    };
    // This timeout is queue space, not pixels, and it is in milliseconds --
    // the driver converts to ticks itself, so pdMS_TO_TICKS() here divides
    // twice.
    return esp_cam_ctlr_receive(c->cam, &trans, 100) == ESP_OK;
}

// Complete host teardown. Safe if already clean. Touches no Python objects,
// because the soft-reset hook runs when the heap is about to be wiped.
static void cameraif_teardown(void) {
    cameraif_obj_t *c = &cameraif_singleton;
    c->open = false;
    if (c->cam) {
        esp_cam_ctlr_stop(c->cam);
        esp_cam_ctlr_disable(c->cam);
        esp_cam_ctlr_del(c->cam);
        c->cam = NULL;
    }
    if (c->isp) {
        esp_isp_disable(c->isp);
        esp_isp_del_processor(c->isp);
        c->isp = NULL;
    }
    if (c->jpeg) {
        jpeg_del_encoder_engine(c->jpeg);
        c->jpeg = NULL;
    }
    if (c->jpeg_buf) {
        heap_caps_free(c->jpeg_buf);
        c->jpeg_buf = NULL;
        c->jpeg_buf_size = 0;
    }
    if (c->sensor) {
        int off = 0;
        esp_cam_sensor_ioctl(c->sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &off);
        esp_cam_sensor_del_dev(c->sensor);
        c->sensor = NULL;
    }
    if (c->sccb) {
        esp_sccb_del_i2c_io(c->sccb);
        c->sccb = NULL;
    }
    if (c->i2c) {
        i2c_del_master_bus(c->i2c);
        c->i2c = NULL;
    }
    if (c->ldo) {
        esp_ldo_release_channel(c->ldo);
        c->ldo = NULL;
    }
    for (int i = 0; i < 2; i++) {
        if (c->fb[i]) {
            heap_caps_free(c->fb[i]);
            c->fb[i] = NULL;
        }
    }
    c->frame = NULL;
    if (c->frame_ready) {
        vSemaphoreDelete(c->frame_ready);
        c->frame_ready = NULL;
    }
}

// Map a sensor's declared output to the ISP's input colour type.
//
// The hardcoded RAW8 this replaced was right for the OV5647 and silently
// wrong for anything emitting RAW10 -- the ISP would misread every frame
// while every call still returned success.
static bool cameraif_isp_input_for(int sensor_format, isp_color_t *out) {
    switch (sensor_format) {
        case ESP_CAM_SENSOR_PIXFORMAT_RAW8:  *out = ISP_COLOR_RAW8;  return true;
        case ESP_CAM_SENSOR_PIXFORMAT_RAW10: *out = ISP_COLOR_RAW10; return true;
        case ESP_CAM_SENSOR_PIXFORMAT_RAW12: *out = ISP_COLOR_RAW12; return true;
        default: return false;
    }
}

// --- constructor ---------------------------------------------------------

static mp_obj_t cameraif_make_new(const mp_obj_type_t *type, size_t n_args,
    size_t n_kw, const mp_obj_t *all_args) {
    enum {
        ARG_sda, ARG_scl, ARG_i2c, ARG_ldo_chan, ARG_ldo_mv,
        ARG_reset, ARG_pwdn, ARG_xclk, ARG_xclk_hz,
        ARG_format, ARG_pixel_format, ARG_raw, ARG_lane_mbps,
        ARG_exposure, ARG_gain, ARG_flip, ARG_mirror, ARG_test_pattern,
    };
    static const mp_arg_t allowed[] = {
        // Board wiring. sda/scl are required: a camera module cannot guess
        // which pins a board put its SCCB bus on, and a wrong default is
        // harder to notice than a missing argument.
        { MP_QSTR_sda,          MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_scl,          MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_i2c,          MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 0} },
        // MIPI PHY power. Channel 3 at 2500 mV is the P4's own arrangement,
        // not a board choice, so it defaults.
        { MP_QSTR_ldo_chan,     MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 3} },
        { MP_QSTR_ldo_mv,       MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 2500} },
        // Sensor control signals. Many modules need one or both toggled to
        // come up at all, and their absence is the likeliest reason a
        // different camera looks dead on a working bus.
        { MP_QSTR_reset,        MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1} },
        { MP_QSTR_pwdn,         MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1} },
        // Some modules carry their own crystal; others expect the host to
        // generate the clock. A sensor with no clock answers nothing.
        { MP_QSTR_xclk,         MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = -1} },
        { MP_QSTR_xclk_hz,      MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 24000000} },
        { MP_QSTR_format,       MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
        { MP_QSTR_pixel_format, MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
        { MP_QSTR_raw,          MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_lane_mbps,    MP_ARG_KW_ONLY | MP_ARG_INT,  {.u_int = 0} },
        { MP_QSTR_exposure,     MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
        { MP_QSTR_gain,         MP_ARG_KW_ONLY | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
        { MP_QSTR_flip,         MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_mirror,       MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_test_pattern, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed),
        allowed, args);

    // Idempotent: tear down whatever a previous camera left before building
    // a new one, rather than refusing. Refusing makes the module unusable
    // after a soft reset until a power cycle, which is the wrong answer to
    // "the interpreter restarted".
    cameraif_teardown();
    cameraif_obj_t *c = &cameraif_singleton;
    memset(c, 0, sizeof(*c));
    c->base.type = type;
    c->raw = args[ARG_raw].u_bool;

    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = args[ARG_ldo_chan].u_int,
        .voltage_mv = args[ARG_ldo_mv].u_int,
    };
    if (esp_ldo_acquire_channel(&ldo_cfg, &c->ldo) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("MIPI PHY LDO unavailable"));
    }

    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = args[ARG_i2c].u_int,
        .scl_io_num = args[ARG_scl].u_int,
        .sda_io_num = args[ARG_sda].u_int,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&i2c_cfg, &c->i2c) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("SCCB bus init failed"));
    }

    // Detection is a linker-section walk, not a call: each driver registers a
    // probe with ESP_CAM_SENSOR_DETECT_FN, and the array between these
    // symbols is every driver linked into this firmware, each with its own
    // SCCB address. The section is only populated for drivers the linker
    // kept, which is why the sdkconfig must enable both CONFIG_CAMERA_<chip>
    // and its AUTO_DETECT option -- without them the array is empty and
    // every camera reports absent, a wiring-shaped error with a build cause.
    esp_cam_sensor_config_t sensor_cfg = {
        .reset_pin = args[ARG_reset].u_int,
        .pwdn_pin = args[ARG_pwdn].u_int,
        .xclk_pin = args[ARG_xclk].u_int,
        .xclk_freq_hz = args[ARG_xclk_hz].u_int,
    };
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
        sccb_i2c_config_t sccb_cfg = {
            .scl_speed_hz = 100000,
            .device_address = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        esp_sccb_io_handle_t io = NULL;
        if (sccb_new_i2c_io(c->i2c, &sccb_cfg, &io) != ESP_OK) {
            continue;
        }
        sensor_cfg.sccb_handle = io;
        sensor_cfg.sensor_port = p->port;
        c->sensor = (*(p->detect))(&sensor_cfg);
        if (c->sensor != NULL) {
            c->sccb = io;
            break;
        }
        esp_sccb_del_i2c_io(io);
    }
    if (c->sensor == NULL) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("no camera sensor answered on SCCB"));
    }

    esp_cam_sensor_format_array_t formats = {0};
    esp_cam_sensor_query_format(c->sensor, &formats);
    if (formats.count == 0) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sensor offers no formats"));
    }
    const esp_cam_sensor_format_t *chosen = &formats.format_array[0];
    if (args[ARG_format].u_obj != mp_const_none) {
        const char *want = mp_obj_str_get_str(args[ARG_format].u_obj);
        const esp_cam_sensor_format_t *found = NULL;
        for (uint32_t i = 0; i < formats.count; i++) {
            if (strcmp(formats.format_array[i].name, want) == 0) {
                found = &formats.format_array[i];
                break;
            }
        }
        if (found == NULL) {
            cameraif_teardown();
            mp_raise_msg_varg(&mp_type_ValueError,
                MP_ERROR_TEXT("sensor has no format '%s'; ask formats()"), want);
        }
        chosen = found;
    }
    if (esp_cam_sensor_set_format(c->sensor, chosen) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sensor rejected format"));
    }

    int stream_on = 1;
    if (esp_cam_sensor_ioctl(c->sensor, ESP_CAM_SENSOR_IOC_S_STREAM,
            &stream_on) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sensor would not stream"));
    }

    // The ISP's input is whatever the sensor says it emits, not an
    // assumption. Its output is RGB565 unless overridden; raw mode takes the
    // ISP out of the path entirely and hands back the sensor's own Bayer,
    // which is the bisect that proves the ISP innocent when frames are black.
    isp_color_t isp_in = ISP_COLOR_RAW8;
    if (!cameraif_isp_input_for(chosen->format, &isp_in)) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_ValueError,
            MP_ERROR_TEXT("sensor emits a format the ISP cannot take"));
    }
    isp_color_t isp_out = ISP_COLOR_RGB565;
    c->bytes_per_px = 2;
    if (args[ARG_pixel_format].u_obj != mp_const_none) {
        const char *pf = mp_obj_str_get_str(args[ARG_pixel_format].u_obj);
        if (strcmp(pf, "rgb565") == 0) {
            isp_out = ISP_COLOR_RGB565; c->bytes_per_px = 2;
        } else if (strcmp(pf, "rgb888") == 0) {
            isp_out = ISP_COLOR_RGB888; c->bytes_per_px = 3;
        } else {
            cameraif_teardown();
            mp_raise_msg(&mp_type_ValueError,
                MP_ERROR_TEXT("pixel_format must be 'rgb565' or 'rgb888'"));
        }
    }
    if (c->raw) {
        c->bytes_per_px = 1;
    }

    c->width = chosen->width;
    c->height = chosen->height;
    c->frame_bytes = (size_t)c->width * c->height * c->bytes_per_px;

    size_t align = 0;
    esp_cache_get_alignment(0, &align);
    for (int i = 0; i < 2; i++) {
        c->fb[i] = heap_caps_aligned_calloc(align ? align : 64, 1, c->frame_bytes,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (c->fb[i] == NULL) {
            cameraif_teardown();
            mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("frame buffer"));
        }
    }
    c->fb_filling = 0;
    c->fb_held = -1;
    c->frame = NULL;
    c->frame_ready = xSemaphoreCreateBinary();

    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id = 0,
        .h_res = c->width,
        .v_res = c->height,
        .lane_bit_rate_mbps = args[ARG_lane_mbps].u_int > 0
            ? (uint32_t)args[ARG_lane_mbps].u_int
            : chosen->mipi_info.mipi_clk / 1000000,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = c->raw ? CAM_CTLR_COLOR_RAW8
                                         : CAM_CTLR_COLOR_RGB565,
        .data_lane_num = chosen->mipi_info.lane_num,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    if (esp_cam_new_csi_ctlr(&csi_cfg, &c->cam) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("CSI controller init failed"));
    }
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = cameraif_on_get_new_trans,
        .on_trans_finished = cameraif_on_trans_finished,
    };
    esp_cam_ctlr_register_event_callbacks(c->cam, &cbs, NULL);
    if (esp_cam_ctlr_enable(c->cam) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("CSI enable failed"));
    }

    if (!c->raw) {
        esp_isp_processor_cfg_t isp_cfg = {
            .clk_hz = 80 * 1000 * 1000,
            .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
            .input_data_color_type = isp_in,
            .output_data_color_type = isp_out,
            .has_line_start_packet = false,
            .has_line_end_packet = false,
            .h_res = c->width,
            .v_res = c->height,
        };
        if (esp_isp_new_processor(&isp_cfg, &c->isp) != ESP_OK) {
            cameraif_teardown();
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ISP init failed"));
        }
        esp_isp_enable(c->isp);
    }

    esp_cam_ctlr_start(c->cam);
    c->open = true;
    // Arm the first transaction. Without a queued buffer the controller
    // streams into nothing and no callback ever fires.
    cameraif_queue_buffer();

    // OV5647 quirk, applied only to an OV5647.
    //
    // esp_cam_sensor 2.5.0's ov5647_set_stream() computes the correct
    // MIPI_CTRL00 value into `val` and then discards it, writing a hardcoded
    // `CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE ? 0x14 : 0x00` instead
    // (ov5647.c:329). With that value the sensor never transmits: I2C works,
    // the register set is accepted, stream-on succeeds, and not one MIPI
    // packet is sent.
    //
    // 0x34 is CLOCK_LANE_GATE | LINE_SYNC | BUS_IDLE -- exactly what the
    // discarded `val` evaluates to for a non-continuous clock, which is what
    // this link is. Measured here, counting real completions in 800 ms: 0x04
    // (the continuous-clock case) gives 0 frames, 0x24 and 0x34 give 28-29.
    // What misleads is that once the sensor *is* transmitting, every later
    // value keeps it transmitting -- so a sweep only shows which value
    // *started* it, and every row after the first reads as success.
    //
    // 0x4800 is an OV5647 register address; on another sensor it names
    // something else, so this is gated on the product ID. Applied here
    // rather than patched into the component, so a component upgrade cannot
    // silently undo it and an upstream fix makes it a harmless rewrite.
    if (c->sensor->id.pid == CAMERAIF_OV5647_PID) {
        esp_cam_sensor_reg_val_t mipi_ctrl00 = { .regaddr = 0x4800, .value = 0x34 };
        if (esp_cam_sensor_ioctl(c->sensor, ESP_CAM_SENSOR_IOC_S_REG,
                &mipi_ctrl00) != ESP_OK) {
            cameraif_teardown();
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("MIPI_CTRL00 write failed"));
        }
    }

    // Nothing in this pipeline does auto-exposure -- the P4's ISP has the
    // hardware but no one drives it -- so a sensor at its defaults gives a
    // recognisable but very dark picture. Fractions of each parameter's own
    // advertised range, so they stay sane across sensors and formats.
    if (args[ARG_exposure].u_obj != mp_const_none) {
        cameraif_set_para(ESP_CAM_SENSOR_EXPOSURE_VAL,
            mp_obj_get_int(args[ARG_exposure].u_obj));
    } else {
        cameraif_set_para_fraction(ESP_CAM_SENSOR_EXPOSURE_VAL, 60);
    }
    if (args[ARG_gain].u_obj != mp_const_none) {
        cameraif_set_para(ESP_CAM_SENSOR_GAIN, mp_obj_get_int(args[ARG_gain].u_obj));
    } else {
        cameraif_set_para_fraction(ESP_CAM_SENSOR_GAIN, 25);
    }
    if (args[ARG_flip].u_bool) {
        cameraif_set_para(ESP_CAM_SENSOR_VFLIP, 1);
    }
    if (args[ARG_mirror].u_bool) {
        cameraif_set_para(ESP_CAM_SENSOR_HMIRROR, 1);
    }
    if (args[ARG_test_pattern].u_bool) {
        int on = 1;
        esp_cam_sensor_ioctl(c->sensor, ESP_CAM_SENSOR_IOC_S_TEST_PATTERN, &on);
    }

    if (displayif_register_soft_reset) {
        displayif_register_soft_reset(cameraif_teardown);
    }
    return MP_OBJ_FROM_PTR(c);
}

// --- capture -------------------------------------------------------------

// Wait for a frame.
//
// esp_cam_ctlr_receive() does NOT wait: it queues a buffer for the driver to
// fill and returns as soon as the queue accepts it. Completion arrives on
// on_trans_finished. Getting this wrong is silent in both directions, and
// this module managed both -- callbacks with nothing queued (nothing could
// complete), then queueing and treating the queue acknowledgement as a frame
// (capture reported a full frame every time while the buffer never changed).
static bool cameraif_wait_frame(cameraif_obj_t *c, mp_int_t timeout_ms) {
    if (!c->open) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("camera is deinited"));
    }
    TickType_t ticks = (timeout_ms <= 0) ? 1 : pdMS_TO_TICKS(timeout_ms);
    if (ticks == 0) {
        ticks = 1;      // never round a real wait down to a poll
    }
    // Release the previous frame first: while Python held it the driver had
    // only one buffer, so let it have both back before we wait.
    c->fb_held = -1;
    c->frame = NULL;
    if (xSemaphoreTake(c->frame_ready, ticks) != pdTRUE) {
        c->dropped++;
        cameraif_queue_buffer();
        return false;
    }
    if (c->last_received == 0) {
        // A completed transaction that carried nothing is not a frame.
        c->dropped++;
        cameraif_queue_buffer();
        return false;
    }
    // Claim the finished buffer before requeueing, so the buffer we hand the
    // caller is the one the DMA is now forbidden to touch.
    c->fb_held = c->fb_done;
    c->frame = c->fb[c->fb_held];
    cameraif_queue_buffer();
    // The frame lands in PSRAM by DMA, so the CPU's view is stale until
    // invalidated. No UNALIGNED flag: the memory-to-cache direction rejects
    // it, the sync then silently does nothing, and the caller reads cache --
    // which for a calloc'd buffer is a perfect, plausible frame of black.
    esp_cache_msync(c->frame, c->frame_bytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    c->frames++;
    return true;
}

// frame(timeout=1000) -> memoryview | None
//
// The frame where it landed, with no copy at all. capture() into a bytearray
// spends ~100 ms moving 1.28 MB that the caller usually only reads once; a
// preview loop wants to blit straight out of DMA memory.
//
// The view stays valid until the next frame()/capture() call on this camera,
// which releases the buffer back to the driver. Reading it after that is
// reading a frame the sensor is overwriting -- so treat it as borrowed, and
// copy anything you need to keep.
static mp_obj_t cameraif_frame(size_t n_args, const mp_obj_t *args) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(args[0]);
    mp_int_t timeout = (n_args > 1) ? mp_obj_get_int(args[1]) : 1000;
    if (!cameraif_wait_frame(c, timeout)) {
        return mp_const_none;
    }
    return mp_obj_new_memoryview('B' | MP_OBJ_ARRAY_TYPECODE_FLAG_RW,
        c->frame_bytes, c->frame);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_frame_obj, 1, 2, cameraif_frame);

// capture(buf=None, timeout=1000)
//
// With a buffer: fills it and returns the byte count -- the zero-allocation
// primitive a streaming loop needs, where allocating a megabyte per frame
// would be absurd. With no buffer: allocates and returns a fresh bytes, for
// the one-shot still where making the caller size a 1.28 MB buffer first is
// a poor first experience. Returns 0 (or None) when no frame arrived, which
// at 50 fps is a timing fact rather than an error.
static mp_obj_t cameraif_capture(size_t n_args, const mp_obj_t *args) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(args[0]);
    mp_obj_t buf_in = (n_args > 1) ? args[1] : mp_const_none;
    mp_int_t timeout = (n_args > 2) ? mp_obj_get_int(args[2]) : 1000;

    if (buf_in == mp_const_none) {
        if (!cameraif_wait_frame(c, timeout)) {
            return mp_const_none;
        }
        return mp_obj_new_bytes(c->frame, c->frame_bytes);
    }
    mp_buffer_info_t buf;
    mp_get_buffer_raise(buf_in, &buf, MP_BUFFER_WRITE);
    if (buf.len < c->frame_bytes) {
        mp_raise_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("buffer must be at least %d bytes"), (int)c->frame_bytes);
    }
    if (!cameraif_wait_frame(c, timeout)) {
        return MP_OBJ_NEW_SMALL_INT(0);
    }
    memcpy(buf.buf, c->frame, c->frame_bytes);
    return MP_OBJ_NEW_SMALL_INT((mp_int_t)c->frame_bytes);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_capture_obj, 1, 3, cameraif_capture);

// capture_yuy2(buf, width, height, timeout=1000)
//
// RGB565 to YUY2, scaled down by integer decimation, in one pass. Both jobs
// together on purpose: a UVC device advertises YUY2 because every host has a
// path for it, and advertises one fixed frame size chosen at build time,
// while a sensor produces what a sensor produces. Two passes over a megabyte
// would be a waste on a board with better uses for the time.
//
// Decimation, not averaging: nearest-neighbour is a few instructions per
// output pixel and needs no line buffer. A resampler is a different module.
static mp_obj_t cameraif_capture_yuy2(size_t n_args, const mp_obj_t *args) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(args[0]);
    mp_buffer_info_t buf;
    mp_get_buffer_raise(args[1], &buf, MP_BUFFER_WRITE);
    mp_int_t out_w = mp_obj_get_int(args[2]);
    mp_int_t out_h = mp_obj_get_int(args[3]);
    mp_int_t timeout = (n_args > 4) ? mp_obj_get_int(args[4]) : 1000;

    if (c->bytes_per_px != 2) {
        mp_raise_msg(&mp_type_ValueError,
            MP_ERROR_TEXT("capture_yuy2 needs an rgb565 camera"));
    }
    if (out_w <= 0 || out_h <= 0 || (out_w & 1)) {
        mp_raise_msg(&mp_type_ValueError,
            MP_ERROR_TEXT("output width must be positive and even"));
    }
    const size_t need = (size_t)out_w * out_h * 2;
    if (buf.len < need) {
        mp_raise_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("buffer must be at least %d bytes"), (int)need);
    }
    if (out_w > c->width || out_h > c->height) {
        mp_raise_msg(&mp_type_ValueError,
            MP_ERROR_TEXT("output larger than the sensor frame"));
    }
    if (!cameraif_wait_frame(c, timeout)) {
        return MP_OBJ_NEW_SMALL_INT(0);
    }

    const uint16_t *src = (const uint16_t *)c->frame;
    uint8_t *dst = (uint8_t *)buf.buf;
    const int xstep = c->width / out_w;
    const int ystep = c->height / out_h;
    size_t o = 0;
    for (int y = 0; y < out_h; y++) {
        const uint16_t *row = src + (size_t)(y * ystep) * c->width;
        for (int x = 0; x < out_w; x += 2) {
            const uint16_t p0 = row[x * xstep];
            const uint16_t p1 = row[(x + 1) * xstep];
            const int r0 = ((p0 >> 11) & 0x1F) << 3, g0 = ((p0 >> 5) & 0x3F) << 2,
                      b0 = (p0 & 0x1F) << 3;
            const int r1 = ((p1 >> 11) & 0x1F) << 3, g1 = ((p1 >> 5) & 0x3F) << 2,
                      b1 = (p1 & 0x1F) << 3;
            // BT.601 in fixed point. Chroma from the pair's average, which is
            // what YUY2 subsampling means.
            const int y0 = (77 * r0 + 150 * g0 + 29 * b0) >> 8;
            const int y1 = (77 * r1 + 150 * g1 + 29 * b1) >> 8;
            const int ra = (r0 + r1) >> 1, ga = (g0 + g1) >> 1, ba = (b0 + b1) >> 1;
            const int ya = (77 * ra + 150 * ga + 29 * ba) >> 8;
            int u = 128 + (((ba - ya) * 144) >> 8);
            int v = 128 + (((ra - ya) * 183) >> 8);
            if (u < 0) { u = 0; } else if (u > 255) { u = 255; }
            if (v < 0) { v = 0; } else if (v > 255) { v = 255; }
            dst[o++] = (uint8_t)(y0 > 255 ? 255 : y0);
            dst[o++] = (uint8_t)u;
            dst[o++] = (uint8_t)(y1 > 255 ? 255 : y1);
            dst[o++] = (uint8_t)v;
        }
    }
    return MP_OBJ_NEW_SMALL_INT((mp_int_t)need);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_capture_yuy2_obj, 4, 5,
    cameraif_capture_yuy2);

// capture_jpeg(quality=80, timeout=1000) -> bytes
//
// Uses the P4's hardware JPEG encoder, which takes RGB565 directly -- the
// same thing the ISP already produces, so a frame goes sensor -> ISP ->
// encoder with no software conversion anywhere.
//
// This is what makes MJPEG practical on this board. A UVC device advertising
// uncompressed YUY2 is limited by bus bandwidth to small frames; the same
// bus carries 800x800 comfortably once each frame is a JPEG a tenth the
// size. It is also what a still is usually wanted as.
//
// The encoder and its output buffer are created on first use and freed by
// teardown, so a program that never asks for JPEG pays nothing for it.
static mp_obj_t cameraif_capture_jpeg(size_t n_args, const mp_obj_t *args) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(args[0]);
    mp_int_t quality = (n_args > 1) ? mp_obj_get_int(args[1]) : 80;
    mp_int_t timeout = (n_args > 2) ? mp_obj_get_int(args[2]) : 1000;

    if (c->bytes_per_px != 2) {
        mp_raise_msg(&mp_type_ValueError,
            MP_ERROR_TEXT("capture_jpeg needs an rgb565 camera"));
    }
    if (quality < 1 || quality > 100) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("quality is 1..100"));
    }
    if (c->jpeg == NULL) {
        jpeg_encode_engine_cfg_t eng = { .timeout_ms = 200 };
        if (jpeg_new_encoder_engine(&eng, &c->jpeg) != ESP_OK) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("no JPEG encoder"));
        }
        // Generous: a JPEG is far smaller than its source, but a
        // high-quality encode of a noisy frame is not as small as one
        // expects, and an output buffer that is merely usually big enough
        // fails on exactly the frames worth keeping.
        jpeg_encode_memory_alloc_cfg_t mcfg = {
            .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
        };
        size_t got = 0;
        c->jpeg_buf = jpeg_alloc_encoder_mem(c->frame_bytes / 2, &mcfg, &got);
        if (c->jpeg_buf == NULL) {
            jpeg_del_encoder_engine(c->jpeg);
            c->jpeg = NULL;
            mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("JPEG buffer"));
        }
        c->jpeg_buf_size = got;
    }
    if (!cameraif_wait_frame(c, timeout)) {
        return mp_const_none;
    }
    jpeg_encode_cfg_t cfg = {
        .width = c->width,
        .height = c->height,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
        .image_quality = (uint32_t)quality,
    };
    uint32_t out_len = 0;
    if (jpeg_encoder_process(c->jpeg, &cfg, c->frame, c->frame_bytes,
            c->jpeg_buf, c->jpeg_buf_size, &out_len) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("JPEG encode failed"));
    }
    return mp_obj_new_bytes(c->jpeg_buf, out_len);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_capture_jpeg_obj, 1, 3,
    cameraif_capture_jpeg);

// --- introspection and controls -----------------------------------------

static mp_obj_t cameraif_size(mp_obj_t self_in) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(self_in);
    mp_obj_t items[3] = {
        MP_OBJ_NEW_SMALL_INT(c->width),
        MP_OBJ_NEW_SMALL_INT(c->height),
        MP_OBJ_NEW_SMALL_INT((mp_int_t)c->frame_bytes),
    };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(cameraif_size_obj, cameraif_size);

static mp_obj_t cameraif_sensor(mp_obj_t self_in) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(self_in);
    if (!c->sensor) {
        return mp_const_none;
    }
    const char *n = c->sensor->name ? c->sensor->name : "unknown";
    mp_obj_t items[2] = {
        mp_obj_new_str(n, strlen(n)),
        MP_OBJ_NEW_SMALL_INT(c->sensor->id.pid),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(cameraif_sensor_obj, cameraif_sensor);

// Every mode this sensor publishes, as (name, width, height). The names are
// the driver's own and are what format= takes.
static mp_obj_t cameraif_formats(mp_obj_t self_in) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(self_in);
    if (!c->sensor) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("camera is deinited"));
    }
    esp_cam_sensor_format_array_t formats = {0};
    esp_cam_sensor_query_format(c->sensor, &formats);
    mp_obj_t list = mp_obj_new_list(0, NULL);
    for (uint32_t i = 0; i < formats.count; i++) {
        const esp_cam_sensor_format_t *f = &formats.format_array[i];
        mp_obj_t row[3] = {
            mp_obj_new_str(f->name, strlen(f->name)),
            MP_OBJ_NEW_SMALL_INT(f->width),
            MP_OBJ_NEW_SMALL_INT(f->height),
        };
        mp_obj_list_append(list, mp_obj_new_tuple(3, row));
    }
    return list;
}
static MP_DEFINE_CONST_FUN_OBJ_1(cameraif_formats_obj, cameraif_formats);

// Read or set one sensor parameter.
//
// Percent of the range the sensor advertises, by default, because the raw
// units are not portable: exposure is in lines, gain is in driver-specific
// steps, and both ranges move when the format does. A literal that suits one
// mode is quietly wrong in another, and a caller who has to know the units
// has to know the sensor. `raw=True` gives the sensor's own numbers to
// anyone who does.
//
// Reading returns None when the driver cannot answer, rather than a number.
// The earlier version ignored esp_cam_sensor_get_para_value()'s return code
// and handed back the zero it had initialised, so a sensor with no read
// support reported a confident, plausible exposure of 0.
static mp_obj_t cameraif_para(cameraif_obj_t *c, uint32_t id, size_t n_args,
    const mp_obj_t *args, mp_map_t *kw) {
    if (!c->sensor) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("camera is deinited"));
    }
    mp_map_elem_t *e = kw ? mp_map_lookup(kw, MP_OBJ_NEW_QSTR(MP_QSTR_raw),
        MP_MAP_LOOKUP) : NULL;
    const bool raw = e && mp_obj_is_true(e->value);

    esp_cam_sensor_param_desc_t desc = { .id = id };
    const bool have_range =
        esp_cam_sensor_query_para_desc(c->sensor, &desc) == ESP_OK
        && desc.number.maximum > desc.number.minimum;

    if (n_args >= 2 && args[1] != mp_const_none) {
        int32_t v = (int32_t)mp_obj_get_int(args[1]);
        if (!raw) {
            if (!have_range) {
                mp_raise_msg(&mp_type_OSError,
                    MP_ERROR_TEXT("sensor advertises no range for this control"));
            }
            if (v < 0 || v > 100) {
                mp_raise_ValueError(MP_ERROR_TEXT("percent must be 0-100"));
            }
            const int32_t lo = desc.number.minimum, hi = desc.number.maximum;
            v = lo + (int32_t)(((int64_t)(hi - lo) * v) / 100);
            if (desc.number.step > 1) {
                v -= (v - lo) % (int32_t)desc.number.step;
            }
        }
        if (esp_cam_sensor_set_para_value(c->sensor, id, &v, sizeof(v)) != ESP_OK) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sensor rejected value"));
        }
    }

    int32_t cur = 0;
    if (esp_cam_sensor_get_para_value(c->sensor, id, &cur, sizeof(cur)) != ESP_OK) {
        return mp_const_none;       // unreadable is not zero
    }
    if (raw) {
        mp_obj_t items[4] = {
            MP_OBJ_NEW_SMALL_INT(cur),
            MP_OBJ_NEW_SMALL_INT(have_range ? desc.number.minimum : 0),
            MP_OBJ_NEW_SMALL_INT(have_range ? desc.number.maximum : 0),
            MP_OBJ_NEW_SMALL_INT(have_range ? desc.number.step : 0),
        };
        return mp_obj_new_tuple(4, items);
    }
    if (!have_range) {
        return mp_const_none;       // no range means no percent to report
    }
    const int32_t lo = desc.number.minimum, hi = desc.number.maximum;
    return MP_OBJ_NEW_SMALL_INT(((int64_t)(cur - lo) * 100) / (hi - lo));
}

// Booleans that live in the sensor. Same parameter mechanism, but a flip is
// on or off -- reporting it as a percentage of a two-value range would be
// technically true and useless.
static mp_obj_t cameraif_para_bool(cameraif_obj_t *c, uint32_t id,
    size_t n_args, const mp_obj_t *args) {
    if (!c->sensor) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("camera is deinited"));
    }
    if (n_args >= 2 && args[1] != mp_const_none) {
        int32_t v = mp_obj_is_true(args[1]) ? 1 : 0;
        if (esp_cam_sensor_set_para_value(c->sensor, id, &v, sizeof(v)) != ESP_OK) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sensor rejected value"));
        }
    }
    int32_t cur = 0;
    if (esp_cam_sensor_get_para_value(c->sensor, id, &cur, sizeof(cur)) != ESP_OK) {
        return mp_const_none;
    }
    return mp_obj_new_bool(cur != 0);
}

// controls() -> {name: (minimum, maximum, step)}
//
// Only what the driver in this firmware actually advertises. Worth its size:
// the sensor drivers vary enormously in how much they implement -- the
// OV5647's, for instance, offers exactly three parameters and cannot read
// any of them back -- and without this a caller discovers that by calling
// things and catching OSError.
static mp_obj_t cameraif_controls(mp_obj_t self_in) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(self_in);
    if (!c->sensor) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("camera is deinited"));
    }
    static const struct { uint16_t q; uint32_t id; } known[] = {
        { MP_QSTR_exposure,   ESP_CAM_SENSOR_EXPOSURE_VAL },
        { MP_QSTR_gain,       ESP_CAM_SENSOR_GAIN },
        { MP_QSTR_gain,       ESP_CAM_SENSOR_ANGAIN },
        { MP_QSTR_flip,       ESP_CAM_SENSOR_VFLIP },
        { MP_QSTR_mirror,     ESP_CAM_SENSOR_HMIRROR },
        { MP_QSTR_brightness, ESP_CAM_SENSOR_BRIGHTNESS },
        { MP_QSTR_contrast,   ESP_CAM_SENSOR_CONTRAST },
        { MP_QSTR_saturation, ESP_CAM_SENSOR_SATURATION },
        { MP_QSTR_sharpness,  ESP_CAM_SENSOR_SHARPNESS },
        { MP_QSTR_denoise,    ESP_CAM_SENSOR_DENOISE },
        { MP_QSTR_awb,        ESP_CAM_SENSOR_AWB },
        { MP_QSTR_agc,        ESP_CAM_SENSOR_AGC },
    };
    mp_obj_t d = mp_obj_new_dict(4);
    for (size_t i = 0; i < MP_ARRAY_SIZE(known); i++) {
        esp_cam_sensor_param_desc_t desc = { .id = known[i].id };
        if (esp_cam_sensor_query_para_desc(c->sensor, &desc) != ESP_OK) {
            continue;
        }
        mp_obj_t range[3] = {
            MP_OBJ_NEW_SMALL_INT(desc.number.minimum),
            MP_OBJ_NEW_SMALL_INT(desc.number.maximum),
            MP_OBJ_NEW_SMALL_INT(desc.number.step),
        };
        mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(known[i].q), mp_obj_new_tuple(3, range));
    }
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_1(cameraif_controls_obj, cameraif_controls);

#define CAMERAIF_PARA_METHOD(pyname, id)                                     \
    static mp_obj_t cameraif_##pyname(size_t n_args, const mp_obj_t *args,   \
        mp_map_t *kw) {                                                      \
        return cameraif_para(MP_OBJ_TO_PTR(args[0]), id, n_args, args, kw);  \
    }                                                                        \
    static MP_DEFINE_CONST_FUN_OBJ_KW(                                       \
        cameraif_##pyname##_obj, 1, cameraif_##pyname);

#define CAMERAIF_PARA_BOOL_METHOD(pyname, id)                                \
    static mp_obj_t cameraif_##pyname(size_t n_args, const mp_obj_t *args) { \
        return cameraif_para_bool(MP_OBJ_TO_PTR(args[0]), id, n_args, args); \
    }                                                                        \
    static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(                              \
        cameraif_##pyname##_obj, 1, 2, cameraif_##pyname);

CAMERAIF_PARA_METHOD(exposure, ESP_CAM_SENSOR_EXPOSURE_VAL)
// gain() is the one control where drivers genuinely disagree about the id:
// some implement absolute GAIN, some only analogue ANGAIN. Try both rather
// than making the caller know which driver they got.
static mp_obj_t cameraif_gain(size_t n_args, const mp_obj_t *args, mp_map_t *kw) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(args[0]);
    esp_cam_sensor_param_desc_t d = { .id = ESP_CAM_SENSOR_GAIN };
    uint32_t id = (c->sensor
        && esp_cam_sensor_query_para_desc(c->sensor, &d) == ESP_OK)
        ? ESP_CAM_SENSOR_GAIN : ESP_CAM_SENSOR_ANGAIN;
    return cameraif_para(c, id, n_args, args, kw);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(cameraif_gain_obj, 1, cameraif_gain);
CAMERAIF_PARA_BOOL_METHOD(flip, ESP_CAM_SENSOR_VFLIP)
CAMERAIF_PARA_BOOL_METHOD(mirror, ESP_CAM_SENSOR_HMIRROR)
// test_pattern(on) is an ioctl rather than a parameter, so it does not fit
// the read/write-with-range shape the others share. Worth having: the
// sensor's own pattern proves the MIPI link independently of lens and
// lighting, which is the evidence that was missing while this camera was
// silent.
static mp_obj_t cameraif_test_pattern(size_t n_args, const mp_obj_t *args) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(args[0]);
    if (!c->sensor) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("camera is deinited"));
    }
    if (n_args < 2) {
        // The ioctl is set-only, so this is what was last asked for rather
        // than what the sensor reports. Said here because a reader who
        // assumes otherwise would trust it during exactly the debugging
        // session it was added for.
        return mp_obj_new_bool(c->test_pattern);
    }
    int on = mp_obj_is_true(args[1]) ? 1 : 0;
    if (esp_cam_sensor_ioctl(c->sensor, ESP_CAM_SENSOR_IOC_S_TEST_PATTERN,
            &on) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("sensor has no test pattern"));
    }
    c->test_pattern = (on != 0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_test_pattern_obj, 1, 2,
    cameraif_test_pattern);

// reg(addr) / reg(addr, value): read or write one sensor register.
//
// Deliberately exposed. A camera that answers I2C and sends no pixels is
// diagnosed by reading registers, and doing that at one firmware rebuild per
// guess is how a night disappears -- this is how the OV5647 quirk above was
// pinned down.
static mp_obj_t cameraif_reg(size_t n_args, const mp_obj_t *args) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(args[0]);
    if (!c->sensor) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("camera is deinited"));
    }
    esp_cam_sensor_reg_val_t rv = {
        .regaddr = (uint32_t)mp_obj_get_int(args[1]),
        .value = 0,
    };
    if (n_args >= 3) {
        rv.value = (uint32_t)mp_obj_get_int(args[2]);
        if (esp_cam_sensor_ioctl(c->sensor, ESP_CAM_SENSOR_IOC_S_REG, &rv) != ESP_OK) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("register write failed"));
        }
        return mp_const_none;
    }
    if (esp_cam_sensor_ioctl(c->sensor, ESP_CAM_SENSOR_IOC_G_REG, &rv) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("register read failed"));
    }
    return MP_OBJ_NEW_SMALL_INT((mp_int_t)rv.value);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_reg_obj, 2, 3, cameraif_reg);

// (frames, dropped, isr_new, isr_done, last_received). The ISR pair says
// whether the hardware is delivering at all, which no amount of staring at
// pixel values can distinguish from delivering black.
static mp_obj_t cameraif_stats(mp_obj_t self_in) {
    cameraif_obj_t *c = MP_OBJ_TO_PTR(self_in);
    mp_obj_t items[5] = {
        mp_obj_new_int_from_uint(c->frames),
        mp_obj_new_int_from_uint(c->dropped),
        mp_obj_new_int_from_uint(c->isr_new),
        mp_obj_new_int_from_uint(c->isr_done),
        mp_obj_new_int_from_uint(c->last_received),
    };
    return mp_obj_new_tuple(5, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(cameraif_stats_obj, cameraif_stats);

// Idempotent: a second call is a no-op, per displayif's lifecycle contract.
static mp_obj_t cameraif_deinit(mp_obj_t self_in) {
    (void)self_in;
    if (displayif_unregister_soft_reset) {
        displayif_unregister_soft_reset(cameraif_teardown);
    }
    cameraif_teardown();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(cameraif_deinit_obj, cameraif_deinit);

static mp_obj_t cameraif_del(mp_obj_t self_in) {
    // Best-effort only: __del__ is unreliable and must never be the sole
    // teardown path. deinit() and the soft-reset hook are the real ones.
    return cameraif_deinit(self_in);
}
static MP_DEFINE_CONST_FUN_OBJ_1(cameraif_del_obj, cameraif_del);

static const mp_rom_map_elem_t cameraif_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_capture), MP_ROM_PTR(&cameraif_capture_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture_yuy2), MP_ROM_PTR(&cameraif_capture_yuy2_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture_jpeg), MP_ROM_PTR(&cameraif_capture_jpeg_obj) },
    { MP_ROM_QSTR(MP_QSTR_size), MP_ROM_PTR(&cameraif_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_sensor), MP_ROM_PTR(&cameraif_sensor_obj) },
    { MP_ROM_QSTR(MP_QSTR_formats), MP_ROM_PTR(&cameraif_formats_obj) },
    { MP_ROM_QSTR(MP_QSTR_controls), MP_ROM_PTR(&cameraif_controls_obj) },
    { MP_ROM_QSTR(MP_QSTR_exposure), MP_ROM_PTR(&cameraif_exposure_obj) },
    { MP_ROM_QSTR(MP_QSTR_gain), MP_ROM_PTR(&cameraif_gain_obj) },
    { MP_ROM_QSTR(MP_QSTR_flip), MP_ROM_PTR(&cameraif_flip_obj) },
    { MP_ROM_QSTR(MP_QSTR_mirror), MP_ROM_PTR(&cameraif_mirror_obj) },
    { MP_ROM_QSTR(MP_QSTR_test_pattern), MP_ROM_PTR(&cameraif_test_pattern_obj) },
    { MP_ROM_QSTR(MP_QSTR_reg), MP_ROM_PTR(&cameraif_reg_obj) },
    { MP_ROM_QSTR(MP_QSTR_frame), MP_ROM_PTR(&cameraif_frame_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats), MP_ROM_PTR(&cameraif_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&cameraif_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&cameraif_del_obj) },
};
static MP_DEFINE_CONST_DICT(cameraif_locals_dict, cameraif_locals_table);

static MP_DEFINE_CONST_OBJ_TYPE(
    cameraif_camera_type,
    MP_QSTR_Camera,
    MP_TYPE_FLAG_NONE,
    make_new, cameraif_make_new,
    locals_dict, &cameraif_locals_dict
);

static mp_obj_t cameraif_available(void) {
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_available_obj, cameraif_available);

#else  // not an ESP32-P4

static mp_obj_t cameraif_unsupported(const mp_obj_type_t *type, size_t n_args,
    size_t n_kw, const mp_obj_t *args) {
    (void)type; (void)n_args; (void)n_kw; (void)args;
    mp_raise_msg(&mp_type_OSError,
        MP_ERROR_TEXT("cameraif needs an ESP32-P4 MIPI-CSI port"));
    return mp_const_none;
}

static MP_DEFINE_CONST_OBJ_TYPE(
    cameraif_camera_type,
    MP_QSTR_Camera,
    MP_TYPE_FLAG_NONE,
    make_new, cameraif_unsupported
);

// available() answers honestly rather than raising, so an application can
// ask what a board can do instead of inferring it from exception types.
static mp_obj_t cameraif_available(void) {
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_available_obj, cameraif_available);

#endif

static const mp_rom_map_elem_t cameraif_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_cameraif) },
    { MP_ROM_QSTR(MP_QSTR_Camera), MP_ROM_PTR(&cameraif_camera_type) },
    { MP_ROM_QSTR(MP_QSTR_available), MP_ROM_PTR(&cameraif_available_obj) },
};
static MP_DEFINE_CONST_DICT(cameraif_globals, cameraif_globals_table);

const mp_obj_module_t cameraif_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&cameraif_globals,
};

MP_REGISTER_MODULE(MP_QSTR_cameraif, cameraif_module);
