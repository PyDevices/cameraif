# OV5647 never transmits: `ov5647_set_stream()` discards its computed MIPI_CTRL00

**Target:** [espressif/esp-video-components](https://github.com/espressif/esp-video-components) (`esp_cam_sensor` 2.5.0)
**Status:** FILED 2026-09-04 as https://github.com/espressif/esp-video-components/issues/97
(posted as the text below the `---`, plus a one-line environment header).

Kept out of the post on purpose. A one-line mistake does not need a book, and
maintainers have said so in other projects: prune it, or write it the way a
knowledgeable colleague would. What we cut, and where it lives instead:

- The measurement trap that cost two sessions a night — once the sensor
  transmits, any later value keeps it transmitting, so a sweep only shows
  which value *started* it. Deinit between trials, and count DMA completions
  rather than return codes, because every call on this path returns `ESP_OK`
  whether or not a pixel moves. Now in `docs/agent-knowledge/device-debugging.md`
  in the workspace anchor.
- The CSI host PHY read that proved the sensor silent rather than the receiver
  deaf. Same place. Offer it in the thread only if asked how we know.
- Our own two wrong tables. Nobody upstream needs them.

If the maintainers want the fuller picture they will ask, and answering a
question is cheaper for them than skimming a wall.

---

### OV5647 never transmits: `ov5647_set_stream()` discards its computed value

`sensors/ov5647/ov5647.c` computes the `MIPI_CTRL00` value and then writes a
literal instead:

```c
uint8_t val = OV5647_MIPI_CTRL00_BUS_IDLE;
if (enable) {
#if CSI2_NONCONTINUOUS_CLOCK
    val |= OV5647_MIPI_CTRL00_CLOCK_LANE_GATE | OV5647_MIPI_CTRL00_LINE_SYNC_ENABLE;
#endif
} else { ... }

ret = ov5647_write(dev->sccb_handle, 0x4800,
                   CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE ? 0x14 : 0x00);
```

With the default `CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE=y` that literal is
`0x14`, line sync without `CLOCK_LANE_GATE`, and the sensor never sends a MIPI
packet. Detection, `set_format()` and `S_STREAM` all return `ESP_OK`.

Measured on ESP32-P4 with an OV5647, 2-lane RAW8 800x800, counting DMA
completions over 800 ms: `0x14` and `0x04` give 0 frames; `0x24` and `0x34`
give 28-29. `0x34` is what `val` evaluates to.

Fix:

```c
ret = ov5647_write(dev->sccb_handle, 0x4800, val);
```

with `CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE` selecting `LINE_SYNC_ENABLE`
inside that computation rather than replacing the whole value.

Possibly the cause of espressif/esp-idf#19032.
