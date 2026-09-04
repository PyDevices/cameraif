// SPDX-License-Identifier: MIT
//
// cameraif: a MIPI-CSI camera as a MicroPython native module.
//
// ESP-IDF ships the CSI *controller* driver (esp_driver_cam) and no sensor
// drivers at all; the sensor half comes from the espressif/esp_cam_sensor
// managed component, pulled in for the P4 by
// patches/cameraif-01-micropython-esp32-p4-camera-sensor-component.patch.
// Between them they need four things brought up in order, and the order is
// not obvious from any one header:
//
//   1. An LDO channel, because the P4's MIPI PHY is powered from an internal
//      regulator that is off until something asks for it. Skip this and the
//      CSI controller initialises and then receives nothing, which reads as
//      a dead sensor.
//   2. The sensor itself over SCCB (I2C), which is what actually selects a
//      resolution and frame rate -- from a list the sensor driver publishes,
//      not one the caller invents.
//   3. The CSI controller, told the lane count and bit rate the sensor was
//      configured for. These have to agree with step 2 or the controller
//      locks onto nothing.
//   4. The ISP, which turns the sensor's raw Bayer into pixels. An OV5647
//      emits RAW8; nothing downstream can use that directly.
//
// The frame this hands back is RGB565 because that is the ISP output path
// the reference implementation proves. Callers wanting YUY2 -- which is what
// a UVC device advertises, since every host has a YUY2 path -- get it from
// capture_yuy2(), which converts on the way out rather than making every
// caller write the same loop.

#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"

#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

// The whole module is P4 CSI hardware. Everything else gets the honest
// answers at the bottom of this file rather than an ImportError, so an
// application can ask what the board can do instead of guessing from
// exception types.
#if defined(CONFIG_IDF_TARGET_ESP32P4) && CONFIG_IDF_TARGET_ESP32P4

#include <string.h>

#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_sensor.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "esp_private/esp_cache_private.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// The panel's SCCB bus. Waveshare wires the CSI camera's I2C to GPIO7/8,
// which is also what ESP-IDF's own P4 camera examples use.
#define CAMERAIF_SCCB_SDA   (7)
#define CAMERAIF_SCCB_SCL   (8)
#define CAMERAIF_LDO_CHAN   (3)
#define CAMERAIF_LDO_MV     (2500)

typedef struct {
    bool open;
    bool raw;
    uint16_t width, height;
    size_t frame_bytes;
    uint8_t *frame;                     // RGB565, DMA-capable
    esp_cam_ctlr_handle_t cam;
    isp_proc_handle_t isp;
    esp_ldo_channel_handle_t ldo;
    i2c_master_bus_handle_t i2c;
    esp_sccb_io_handle_t sccb;
    esp_cam_sensor_device_t *sensor;
    SemaphoreHandle_t frame_ready;
    volatile uint32_t frames, dropped;
    volatile uint32_t isr_new, isr_done;
    volatile uint32_t last_received;
} cameraif_t;

static cameraif_t cam;

// Both callbacks run in ISR context. The controller asks for a buffer to
// fill, then reports the fill finished; handing back the same buffer each
// time is what makes this single-buffered, and is why a caller that reads
// slowly sees the newest frame rather than a queue of stale ones.
static bool IRAM_ATTR cameraif_on_get_new_trans(esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans, void *user_data) {
    (void)handle;
    (void)user_data;
    trans->buffer = cam.frame;
    trans->buflen = cam.frame_bytes;
    cam.isr_new++;
    return false;
}

static bool IRAM_ATTR cameraif_on_trans_finished(esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans, void *user_data) {
    (void)handle;
    (void)user_data;
    BaseType_t woken = pdFALSE;
    cam.isr_done++;
    cam.last_received = (uint32_t)trans->received_size;
    if (cam.frame_ready) {
        // Give, never take: a frame arriving while the last is unread is not
        // an error, it is the normal case at 50 fps with a Python reader.
        xSemaphoreGiveFromISR(cam.frame_ready, &woken);
    }
    return woken == pdTRUE;
}

static bool cameraif_queue_buffer(void);

static void cameraif_teardown(void) {
    if (cam.cam) {
        esp_cam_ctlr_stop(cam.cam);
        esp_cam_ctlr_disable(cam.cam);
        esp_cam_ctlr_del(cam.cam);
        cam.cam = NULL;
    }
    if (cam.isp) {
        esp_isp_disable(cam.isp);
        esp_isp_del_processor(cam.isp);
        cam.isp = NULL;
    }
    if (cam.sensor) {
        int stream_off = 0;
        esp_cam_sensor_ioctl(cam.sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_off);
        esp_cam_sensor_del_dev(cam.sensor);
        cam.sensor = NULL;
    }
    if (cam.sccb) {
        esp_sccb_del_i2c_io(cam.sccb);
        cam.sccb = NULL;
    }
    if (cam.i2c) {
        i2c_del_master_bus(cam.i2c);
        cam.i2c = NULL;
    }
    if (cam.ldo) {
        esp_ldo_release_channel(cam.ldo);
        cam.ldo = NULL;
    }
    if (cam.frame) {
        heap_caps_free(cam.frame);
        cam.frame = NULL;
    }
    if (cam.frame_ready) {
        vSemaphoreDelete(cam.frame_ready);
        cam.frame_ready = NULL;
    }
    cam.open = false;
}

// init(format_name=None, raw=False)
//
// `raw` takes the ISP out of the path and hands back the sensor's own RAW8
// Bayer. It exists as a bisect: an all-black frame through the ISP and a
// live one in raw mode says the ISP; black in both says the sensor or the
// CSI link. Guessing between those two costs a build cycle each time.
static mp_obj_t cameraif_init(size_t n_args, const mp_obj_t *args) {
    // Re-open rather than refuse. A soft reset clears MicroPython's heap but
    // not this file's statics, so after Ctrl-D the module still believes it
    // holds a camera it can no longer describe to anyone. Refusing there
    // makes the module unusable until a power cycle, which is the wrong
    // answer to "the interpreter restarted".
    if (cam.open) {
        cameraif_teardown();
    }
    memset(&cam, 0, sizeof(cam));

    // 1. MIPI PHY power. Before anything else: the CSI controller will
    // initialise happily without it and then never see a lane transition.
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = CAMERAIF_LDO_CHAN,
        .voltage_mv = CAMERAIF_LDO_MV,
    };
    if (esp_ldo_acquire_channel(&ldo_cfg, &cam.ldo) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("MIPI PHY LDO unavailable"));
    }

    // 2. The sensor, over SCCB.
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = CAMERAIF_SCCB_SCL,
        .sda_io_num = CAMERAIF_SCCB_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&i2c_cfg, &cam.i2c) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("SCCB bus init failed"));
    }

    // Sensor detection is a linker-section walk, not a call: each driver
    // registers a probe with ESP_CAM_SENSOR_DETECT_FN, and the array between
    // these two symbols is every driver linked into this firmware. Each probe
    // gets its own SCCB address, so the loop is "ask every driver we shipped
    // whether its chip is on this bus".
    //
    // Note the section is only populated for drivers the linker kept, which
    // is why micropython.cmake forces ov5647_detect with -u: nothing in C
    // references it, so without that the array is empty and every camera
    // looks absent.
    esp_cam_sensor_config_t sensor_cfg = {
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,
    };
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
        sccb_i2c_config_t sccb_cfg = {
            .scl_speed_hz = 100000,
            .device_address = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        esp_sccb_io_handle_t io = NULL;
        if (sccb_new_i2c_io(cam.i2c, &sccb_cfg, &io) != ESP_OK) {
            continue;
        }
        sensor_cfg.sccb_handle = io;
        sensor_cfg.sensor_port = p->port;
        cam.sensor = (*(p->detect))(&sensor_cfg);
        if (cam.sensor != NULL) {
            cam.sccb = io;
            break;
        }
        esp_sccb_del_i2c_io(io);
    }
    if (cam.sensor == NULL) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("no CSI sensor answered on SCCB"));
    }

    // Ask the sensor what it can do, and take the format the caller named or
    // the first one offered. Inventing a resolution here is how a caller ends
    // up configuring a controller for a mode the sensor never entered.
    esp_cam_sensor_format_array_t formats = {0};
    esp_cam_sensor_query_format(cam.sensor, &formats);
    if (formats.count == 0) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sensor offers no formats"));
    }
    const esp_cam_sensor_format_t *chosen = &formats.format_array[0];
    if (n_args >= 1 && args[0] != mp_const_none) {
        const char *want = mp_obj_str_get_str(args[0]);
        for (uint32_t i = 0; i < formats.count; i++) {
            if (strcmp(formats.format_array[i].name, want) == 0) {
                chosen = &formats.format_array[i];
                break;
            }
        }
    }
    if (esp_cam_sensor_set_format(cam.sensor, chosen) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sensor rejected format"));
    }
    // Setting a format configures the sensor's registers; it does not turn
    // its output on. Without this the whole pipeline works perfectly and
    // captures black: the CSI controller starts, DMA completes, transfers
    // are reported finished, and every frame is zeros -- which reads as a
    // lens cap, a dark room, or a broken ISP, and is none of them.
    int stream_on = 1;
    if (esp_cam_sensor_ioctl(cam.sensor, ESP_CAM_SENSOR_IOC_S_STREAM,
            &stream_on) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sensor would not stream"));
    }

    const bool raw = (n_args >= 2) && mp_obj_is_true(args[1]);
    // Optional override for the MIPI lane bit rate.
    //
    // The sensor driver's tables name their clock assumption -- the format
    // here is "24Minput" -- and compute the line rate from it. A module with
    // a different crystal (Raspberry-Pi-style OV5647 boards carry 25 MHz)
    // scales that rate, and a PHY told to expect the wrong one does not lock.
    // The failure is silent and total: the controller starts, DMA completes,
    // transfers report finished, and every frame is zeros. Being able to
    // sweep this from Python is the difference between one experiment and
    // one firmware build per guess.
    const mp_int_t lane_override = (n_args >= 3) ? mp_obj_get_int(args[2]) : 0;
    cam.raw = raw;
    cam.width = chosen->width;
    cam.height = chosen->height;
    cam.frame_bytes = (size_t)cam.width * cam.height * (raw ? 1 : 2);

    size_t align = 0;
    esp_cache_get_alignment(0, &align);
    cam.frame = heap_caps_aligned_calloc(align ? align : 64, 1, cam.frame_bytes,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (cam.frame == NULL) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("frame buffer"));
    }
    cam.frame_ready = xSemaphoreCreateBinary();

    // 3. The CSI controller, matched to what the sensor was just set to.
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id = 0,
        .h_res = cam.width,
        .v_res = cam.height,
        .lane_bit_rate_mbps = lane_override > 0 ? (uint32_t)lane_override
                                                : chosen->mipi_info.mipi_clk / 1000000,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = raw ? CAM_CTLR_COLOR_RAW8 : CAM_CTLR_COLOR_RGB565,
        .data_lane_num = chosen->mipi_info.lane_num,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    if (esp_cam_new_csi_ctlr(&csi_cfg, &cam.cam) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("CSI controller init failed"));
    }
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = cameraif_on_get_new_trans,
        .on_trans_finished = cameraif_on_trans_finished,
    };
    esp_cam_ctlr_register_event_callbacks(cam.cam, &cbs, NULL);
    if (esp_cam_ctlr_enable(cam.cam) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("CSI enable failed"));
    }

    // 4. The ISP: RAW8 in, RGB565 out. Skipped entirely in raw mode.
    if (!raw) {
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = cam.width,
        .v_res = cam.height,
    };
    if (esp_isp_new_processor(&isp_cfg, &cam.isp) != ESP_OK) {
        cameraif_teardown();
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ISP init failed"));
    }
    esp_isp_enable(cam.isp);
    }
    esp_cam_ctlr_start(cam.cam);
    cam.open = true;
    // Arm the first transaction. Without a queued buffer the controller
    // streams into nothing and no callback ever fires.
    cameraif_queue_buffer();

    mp_obj_t items[3] = {
        MP_OBJ_NEW_SMALL_INT(cam.width),
        MP_OBJ_NEW_SMALL_INT(cam.height),
        mp_obj_new_str(chosen->name, strlen(chosen->name)),
    };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_init_obj, 0, 3, cameraif_init);

static mp_obj_t cameraif_deinit(void) {
    cameraif_teardown();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_deinit_obj, cameraif_deinit);

// Every list the sensor driver publishes, so a caller can choose rather than
// guess. Names are the driver's own, e.g.
// "MIPI_2lane_24Minput_RAW8_800x640_50fps".
static mp_obj_t cameraif_formats(void) {
    if (!cam.sensor) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("camera not open"));
    }
    esp_cam_sensor_format_array_t formats = {0};
    esp_cam_sensor_query_format(cam.sensor, &formats);
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
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_formats_obj, cameraif_formats);

static mp_obj_t cameraif_size(void) {
    mp_obj_t items[3] = {
        MP_OBJ_NEW_SMALL_INT(cam.width),
        MP_OBJ_NEW_SMALL_INT(cam.height),
        MP_OBJ_NEW_SMALL_INT((mp_int_t)cam.frame_bytes),
    };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_size_obj, cameraif_size);

// Wait for a frame and hand back the RGB565 bytes. `timeout_ms` defaults to
// 1000; 0 means "only if one is already waiting".
// Ask the controller for a frame and wait for it.
//
// esp_cam_ctlr_receive() is the whole mechanism: it queues a transaction and
// blocks until that transaction completes. The registered callbacks are not
// an alternative to it -- on_get_new_trans exists to re-arm the *next*
// frame, so a driver that only registers callbacks and never calls receive
// starts a controller that captures nothing. That reads exactly like a
// sensor producing no output, which is where an hour went.
//
// The cache sync afterwards is not optional either: the frame lands in PSRAM
// by DMA, so the CPU's view of it is stale until invalidated. Skipping it
// returns whatever was in cache -- often the poison a caller pre-filled,
// which then looks like "the capture did nothing".
static bool cameraif_queue_buffer(void) {
    esp_cam_ctlr_trans_t trans = {
        .buffer = cam.frame,
        .buflen = cam.frame_bytes,
    };
    // The timeout here is queue space, not pixels, and it is in
    // milliseconds -- the driver converts to ticks itself. Passing
    // pdMS_TO_TICKS() divides twice.
    return esp_cam_ctlr_receive(cam.cam, &trans, 100) == ESP_OK;
}

// Wait for a frame.
//
// esp_cam_ctlr_receive() does NOT wait: it queues a buffer for the driver to
// fill and returns as soon as the queue accepts it. Completion arrives on
// the on_trans_finished callback. Getting this wrong is silent in both
// directions, and this module managed both in one night -- registering
// callbacks but never queueing a buffer (nothing to fill, so nothing ever
// completed), then queueing and treating the queue acknowledgement as a
// frame (capture reported a full frame every time and the buffer never
// changed).
//
// So: keep one buffer queued, block on the callback's semaphore, and
// re-queue for the next frame.
static bool cameraif_wait_frame(mp_int_t timeout_ms) {
    if (!cam.open) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("camera not open"));
    }
    TickType_t ticks = (timeout_ms <= 0) ? 1 : pdMS_TO_TICKS(timeout_ms);
    if (ticks == 0) {
        ticks = 1;      // never round a real wait down to a poll
    }
    if (xSemaphoreTake(cam.frame_ready, ticks) != pdTRUE) {
        cam.dropped++;
        cameraif_queue_buffer();     // keep the pipeline armed
        return false;
    }
    if (cam.last_received == 0) {
        // A completed transaction that carried nothing is not a frame.
        cam.dropped++;
        cameraif_queue_buffer();
        return false;
    }
    // The frame lands in PSRAM by DMA, so the CPU's view is stale until
    // invalidated. No UNALIGNED flag: the memory-to-cache direction rejects
    // it, the sync then does nothing, and the caller reads cache -- which for
    // a calloc'd buffer is a perfect, plausible frame of black.
    esp_cache_msync(cam.frame, cam.frame_bytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    cam.frames++;
    cameraif_queue_buffer();
    return true;
}

static mp_obj_t cameraif_capture(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t buf;
    mp_get_buffer_raise(args[0], &buf, MP_BUFFER_WRITE);
    mp_int_t timeout = (n_args > 1) ? mp_obj_get_int(args[1]) : 1000;
    if (buf.len < cam.frame_bytes) {
        mp_raise_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("buffer must be at least %d bytes"),
            (int)cam.frame_bytes);
    }
    if (!cameraif_wait_frame(timeout)) {
        return MP_OBJ_NEW_SMALL_INT(0);
    }
    memcpy(buf.buf, cam.frame, cam.frame_bytes);
    return MP_OBJ_NEW_SMALL_INT((mp_int_t)cam.frame_bytes);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_capture_obj, 1, 2, cameraif_capture);

// RGB565 -> YUY2, scaled down by an integer factor, in one pass.
//
// Both jobs together on purpose. A UVC device advertises YUY2 because every
// host has a path for it, and it advertises one fixed frame size chosen at
// build time -- while a sensor produces what a sensor produces. Doing the
// conversion and the decimation separately would mean two passes over a
// megabyte, on a board that has better uses for the time.
//
// Decimation, not averaging: nearest-neighbour is a few instructions per
// output pixel and needs no line buffer. A resampler is a different module.
static mp_obj_t cameraif_capture_yuy2(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t buf;
    mp_get_buffer_raise(args[0], &buf, MP_BUFFER_WRITE);
    mp_int_t out_w = mp_obj_get_int(args[1]);
    mp_int_t out_h = mp_obj_get_int(args[2]);
    mp_int_t timeout = (n_args > 3) ? mp_obj_get_int(args[3]) : 1000;

    if (out_w <= 0 || out_h <= 0 || (out_w & 1)) {
        mp_raise_msg(&mp_type_ValueError,
            MP_ERROR_TEXT("output width must be positive and even"));
    }
    const size_t need = (size_t)out_w * out_h * 2;
    if (buf.len < need) {
        mp_raise_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("buffer must be at least %d bytes"), (int)need);
    }
    if (out_w > cam.width || out_h > cam.height) {
        mp_raise_msg(&mp_type_ValueError,
            MP_ERROR_TEXT("output larger than the sensor frame"));
    }
    if (!cameraif_wait_frame(timeout)) {
        return MP_OBJ_NEW_SMALL_INT(0);
    }

    const uint16_t *src = (const uint16_t *)cam.frame;
    uint8_t *dst = (uint8_t *)buf.buf;
    const int xstep = cam.width / out_w;
    const int ystep = cam.height / out_h;
    size_t o = 0;
    for (int y = 0; y < out_h; y++) {
        const uint16_t *row = src + (size_t)(y * ystep) * cam.width;
        for (int x = 0; x < out_w; x += 2) {
            const uint16_t p0 = row[x * xstep];
            const uint16_t p1 = row[(x + 1) * xstep];
            // RGB565 -> 8-bit components, replicating the high bits into the
            // low ones so full-scale stays full-scale.
            const int r0 = ((p0 >> 11) & 0x1F) << 3, g0 = ((p0 >> 5) & 0x3F) << 2,
                      b0 = (p0 & 0x1F) << 3;
            const int r1 = ((p1 >> 11) & 0x1F) << 3, g1 = ((p1 >> 5) & 0x3F) << 2,
                      b1 = (p1 & 0x1F) << 3;
            // BT.601, fixed point. Chroma is taken from the pair's average,
            // which is what YUY2 subsampling means.
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
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_capture_yuy2_obj, 3, 4,
    cameraif_capture_yuy2);

static mp_obj_t cameraif_stats(void) {
    // (frames, dropped, isr_new, isr_done, last_received). The ISR pair says
    // whether the hardware is delivering at all, which no amount of staring
    // at pixel values can distinguish from delivering black.
    mp_obj_t items[5] = {
        mp_obj_new_int_from_uint(cam.frames),
        mp_obj_new_int_from_uint(cam.dropped),
        mp_obj_new_int_from_uint(cam.isr_new),
        mp_obj_new_int_from_uint(cam.isr_done),
        mp_obj_new_int_from_uint(cam.last_received),
    };
    return mp_obj_new_tuple(5, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_stats_obj, cameraif_stats);

static mp_obj_t cameraif_available(void) {
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_available_obj, cameraif_available);

#else  // not an ESP32-P4

static mp_obj_t cameraif_unsupported(size_t n_args, const mp_obj_t *args) {
    (void)n_args; (void)args;
    mp_raise_msg(&mp_type_OSError,
        MP_ERROR_TEXT("cameraif needs an ESP32-P4 MIPI-CSI port"));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_init_obj, 0, 1, cameraif_unsupported);
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_capture_obj, 1, 2, cameraif_unsupported);
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cameraif_capture_yuy2_obj, 3, 4, cameraif_unsupported);

static mp_obj_t cameraif_none(void) { return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_deinit_obj, cameraif_none);
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_formats_obj, cameraif_none);
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_size_obj, cameraif_none);
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_stats_obj, cameraif_none);

static mp_obj_t cameraif_available(void) { return mp_const_false; }
static MP_DEFINE_CONST_FUN_OBJ_0(cameraif_available_obj, cameraif_available);

#endif

static const mp_rom_map_elem_t cameraif_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_cameraif) },
    { MP_ROM_QSTR(MP_QSTR_available), MP_ROM_PTR(&cameraif_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&cameraif_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&cameraif_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_formats), MP_ROM_PTR(&cameraif_formats_obj) },
    { MP_ROM_QSTR(MP_QSTR_size), MP_ROM_PTR(&cameraif_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture), MP_ROM_PTR(&cameraif_capture_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture_yuy2), MP_ROM_PTR(&cameraif_capture_yuy2_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats), MP_ROM_PTR(&cameraif_stats_obj) },
};
static MP_DEFINE_CONST_DICT(cameraif_globals, cameraif_globals_table);

const mp_obj_module_t cameraif_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&cameraif_globals,
};

MP_REGISTER_MODULE(MP_QSTR_cameraif, cameraif_module);
