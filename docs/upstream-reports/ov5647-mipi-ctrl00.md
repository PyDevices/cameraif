# Draft: OV5647 never streams on ESP32-P4 because `ov5647_set_stream()` discards the value it computes

**Note to the poster — strip everything above the `---`.**

File on **espressif/esp-video-components** (that is where `esp_cam_sensor` lives),
not on esp-idf. Suggested title:

> OV5647 never streams with `CAMERA_OV5647_CSI_LINESYNC_ENABLE=y`: `ov5647_set_stream()`
> writes a hardcoded `MIPI_CTRL00` instead of the value it computes

Cross-reference **espressif/esp-idf#19032** (2026-08-31), which reports this exact
symptom on a Waveshare ESP32-P4-WIFI6 board and is open with no maintainer reply.
Say it *may* be the same bug — that reporter's Kconfig is not in their report, and
the default is what produces the failing value. Do not assert it.

Keep it this short. The maintainers do not want a derivation, and the whole problem
is one sentence: the function computes `val` and then writes something else.

Measured on a Waveshare ESP32-P4-WIFI6-Touch-LCD-4B with a Raspberry Pi Camera v1,
ESP-IDF v5.5.2, esp_cam_sensor 2.5.0, MicroPython native module, 2026-09-04. Every
row of the table below is a separate power-on trial through the same code path.

---

### OV5647 never streams with the default line-sync option

`ov5647_set_stream()` computes the `MIPI_CTRL00` value, then writes a different,
hardcoded one:

```c
uint8_t val = OV5647_MIPI_CTRL00_BUS_IDLE;
if (enable) {
#if CSI2_NONCONTINUOUS_CLOCK
    val |= OV5647_MIPI_CTRL00_CLOCK_LANE_GATE | OV5647_MIPI_CTRL00_LINE_SYNC_ENABLE;
#endif
} else {
    val |= OV5647_MIPI_CTRL00_CLOCK_LANE_GATE | OV5647_MIPI_CTRL00_CLOCK_LANE_DISABLE;
}

ret = ov5647_write(dev->sccb_handle, 0x4800,
                   CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE ? 0x14 : 0x00);
```

`val` is never used. With `CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE=y`, the default,
the sensor gets `0x14` — `LINE_SYNC_ENABLE | BUS_IDLE`, line sync **without** the
clock-lane gate — and then never transmits. SCCB is unaffected: the sensor is
detected, its registers are written and read back, and `S_STREAM` returns `ESP_OK`.

On the CSI host: both data lanes stay in stop state, `PHY_RXCLKACTIVEHS` never
asserts, every error counter stays at zero, and no transaction ever completes.

| `MIPI_CTRL00` | meaning | frames in 800 ms |
|---|---|---|
| `0x14` (current default) | line sync, no clock-lane gate | **0** |
| `0x04` | `BUS_IDLE` — what `val` evaluates to today | 28 |
| `0x24` | `CLOCK_LANE_GATE \| BUS_IDLE` | 28 |
| `0x34` | `CLOCK_LANE_GATE \| LINE_SYNC \| BUS_IDLE` | 28 |

`0x14` is the one combination that fails, and it is the only one Linux's ov5647
driver never produces: there, `LINE_SYNC_ENABLE` is only ever set together with
`CLOCK_LANE_GATE` (`drivers/media/i2c/ov5647.c`, `ov5647_stream_on`).

### Fix

Write the value the function already computes:

```c
ret = ov5647_write(dev->sccb_handle, 0x4800, val);
```

`CSI2_NONCONTINUOUS_CLOCK` is not defined anywhere in the component, so `val` is
`0x04` on enable, which streams. If the line-sync option is meant to be reachable,
it needs the clock-lane gate with it (`0x34`), as the Linux driver does.
