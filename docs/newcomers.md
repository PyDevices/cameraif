# Newcomer's guide to cameraif

`cameraif` makes a MIPI-CSI camera available as a MicroPython `Camera` object
on the ESP32-P4. It probes the sensor driver on the I2C bus instead of being
hard-coded for one model; the proven configuration is an OV5647 at 800×800.

It is firmware user C code, not a pip or MIP package. Build the required
ESP32-P4 firmware first, then use the Python API from a board application.

## Start with a still or preview

```python
import cameraif

cam = cameraif.Camera(sda=7, scl=8, reset=21)
print(cam.sensor())
open("still.jpg", "wb").write(cam.capture_jpeg(85))
cam.deinit()
```

The named I2C pins are the sensor's SCCB bus. If that bus is shared with a
touch controller, panel, or codec, pass the I2C port that already owns it.

For a zero-copy preview, scale directly into a display framebuffer:

```python
fb = display_drv.framebuffers()[0]
cam.capture_scaled(fb, display_drv.width, display_drv.height)
display_drv.show()
```

`frame()` returns borrowed memory that remains valid only until the next
`frame()` or `capture()` call. Copy a frame you need to retain.

## The data path

```text
sensor probe over SCCB/I2C -> MIPI CSI + DMA into PSRAM
    -> cache invalidation and completed-frame wait
    -> frame / capture / hardware scale / hardware JPEG
    -> display framebuffer, JPEG file, or YUY2 buffer
```

The native module is necessary because the P4 D-PHY power rail, DMA cache
maintenance, and asynchronous CSI receive path are not safely accessible from
ordinary Python. A queued receive is not a completed frame; capture methods
return `None` or zero when no frame arrives by the timeout.

## Firmware integration

`apply_patches.sh` adds the sensor-driver component and required ESP32
sdkconfig change to a MicroPython checkout. Build with this repository's
`micropython.cmake` as `USER_C_MODULES`, then enable the desired sensor driver
in the board sdkconfig. For OV5647, both `CONFIG_CAMERA_OV5647=y` and its
`AUTO_DETECT_MIPI_INTERFACE_SENSOR` setting are required; the latter registers
the probe that discovers the sensor.

The root [README](../README.md#install) is authoritative for the exact patch,
build, and sdkconfig commands.

## Repository map

| Path | Purpose |
|---|---|
| `src/mod_cameraif.c` | Native Camera implementation and capture ownership. |
| `patches/` | MicroPython ESP32 sensor-component and sdkconfig patches. |
| `apply_patches.sh` | Apply, inspect, or revert repository-owned patches. |
| `examples/` | Still, preview, formats, controls, JPEG, transform, and diagnostics. |
| `docs/performance.md` | Measured capture, scaling, and JPEG behavior. |
| `manifest.py`, `micropython.cmake` | Firmware user-module entrypoints. |

## Controls and diagnostics

Use `controls()` before assuming a sensor supports a setting. It reports only
the controls implemented by the detected driver. Numeric exposure and gain
methods use a percentage of the advertised range by default; use `raw=True`
only when sensor units are intentional. `stats()` distinguishes no CSI arrivals
from frames rejected later in the pipeline, while `test_pattern()` helps
separate optics/sensor faults from the data path.

Only one `Camera` may exist because the P4 has one CSI controller. Always call
`deinit()` when finished.

## Safe first contributions

Start with an example, diagnostic improvement, or measured documentation
correction. Sensor support is principally a build-time driver configuration
question, not a reason to fork the generic `Camera` API. Native changes need
hardware validation and must preserve buffer ownership, cache maintenance, and
timeout semantics.
