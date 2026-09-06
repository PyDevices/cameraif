# cameraif

A MIPI-CSI camera as a MicroPython object, on the ESP32-P4.

```python
import cameraif

cam = cameraif.Camera(sda=7, scl=8, reset=21)
print(cam.sensor())                 # ('OV5647', 22087)
print(cam.size())                   # (800, 800, 1280000)

open("still.jpg", "wb").write(cam.capture_jpeg(85))
cam.deinit()
```

The sensor is found by probing, not by being told. `cameraif` asks the
[`esp_cam_sensor`](https://components.espressif.com/components/espressif/esp_cam_sensor)
component which of its drivers answers on the I2C bus you named, then reads
resolution, frame rate, MIPI lane count and lane bit rate out of whatever
driver replied. Adding a different camera is a build-time `CONFIG_CAMERA_*`
away, not a code change here — which is why the class is `Camera` and not
`OV5647`.

Proven on an ESP32-P4 with an OV5647 at 800x800. Every number in this
document was measured on that board.

## Why a C module

Three things in the path are not reachable from Python, and each is the
difference between a working camera and a plausible-looking failure:

- **The MIPI D-PHY needs 2.5 V from LDO channel 3** before the sensor's
  clock lane means anything. Without it the sensor answers on I2C perfectly
  and delivers nothing.
- **Frames arrive by DMA into PSRAM**, so the CPU's cached view of that
  memory is stale until it is invalidated. Skip the invalidate and you read
  the buffer's previous contents — for a freshly allocated buffer, a
  flawless frame of black.
- **`esp_cam_ctlr_receive()` does not wait.** It queues a buffer and returns
  as soon as the queue accepts it. Treat that acknowledgement as a frame and
  the camera appears to work at hundreds of frames a second while the buffer
  never changes.

## Install

`cameraif` is a user C module. Add it to a MicroPython ESP32-P4 build:

```bash
make USER_C_MODULES=/path/to/cameraif/micropython.cmake BOARD=ESP32_GENERIC_P4
```

and enable a sensor driver in the board's sdkconfig — for the OV5647:

```
CONFIG_CAMERA_OV5647=y
CONFIG_CAMERA_OV5647_AUTO_DETECT_MIPI_INTERFACE_SENSOR=y
```

The `AUTO_DETECT` symbol is what registers the driver's probe in the linker
section the component scans. Set only the first and the driver compiles,
links, and is never asked anything.

## API

### `Camera(sda, scl, **kwargs)`

`sda` and `scl` are required and positional; everything else is keyword-only.
Constructing starts the camera streaming. Only one `Camera` can exist at a
time — the P4 has one CSI controller — and constructing a second one raises.

| Argument | Default | Meaning |
|---|---|---|
| `sda`, `scl` | — | I2C pins the sensor answers on (SCCB) |
| `i2c` | `0` | I2C port. **Use the port that already owns these pins** — see below |
| `ldo_chan` | `3` | LDO channel powering the MIPI D-PHY |
| `ldo_mv` | `2500` | LDO voltage, millivolts |
| `reset` | `-1` | Sensor reset pin, `-1` for none |
| `pwdn` | `-1` | Sensor power-down pin, `-1` for none |
| `xclk` | `-1` | Pin to generate the sensor's clock on, `-1` if the board has an oscillator |
| `xclk_hz` | `24000000` | Frequency for that clock |
| `format` | `None` | Sensor format by name or index; `None` picks the driver's default |
| `pixel_format` | `None` | `"rgb565"` (default) or `"yuv422"` |
| `raw` | `False` | Bypass the ISP and hand back the sensor's raw bytes |
| `lane_mbps` | `0` | Override the driver's MIPI lane bit rate; `0` trusts it |
| `exposure` | `None` | Initial exposure, 0–100 percent of the sensor's range |
| `gain` | `None` | Initial gain, 0–100. Not every driver has one — see `controls()` |
| `flip`, `mirror` | `False` | Initial orientation |
| `test_pattern` | `False` | Start with the sensor's internal pattern |

Board facts are arguments rather than build-time constants on purpose: the
same firmware image should drive a camera on any board that has one, and a
pin number is not a reason to recompile.

#### Sharing the bus, which is the one that will bite you

On most boards the camera connector's SCCB lines are the panel's I2C bus,
shared with the touch controller and often the audio codecs. `i2c` must name
the port that bus is already on. Given a port that already has a bus,
`cameraif` attaches to it; given a free port, it opens its own.

Get this wrong and nothing reports an error. Both peripherals reach the pins
through the ESP32's pin matrix, `CONFIG_I2C_SKIP_LEGACY_CONFLICT_CHECK` (set
by MicroPython upstream) suppresses the one check that would have caught it,
and the camera works perfectly — while every read of whatever else lives on
that bus times out. On this board that was the touchscreen, and the symptom
looked like a scheduler problem two layers away.

Two things guard it now. `cameraif` refuses to open a second master on pins
another driver has reserved, naming the port to pass instead. And
`machine.I2C` on the ESP32-P4 is built on esp-idf's new `i2c_master` driver
(`cmods/patches/cameraif-02-…`), because the legacy driver it used before
cannot hand out a bus handle to share and does not reserve its pins, so
neither the sharing nor the check was possible.

### Getting frames

**`frame(timeout=1000)` → `memoryview` | `None`**

The frame where it landed, with no copy. This is what a preview loop wants.

The view is borrowed, not given: it is valid until your next `frame()` or
`capture()` call on this camera, which hands the buffer back to the driver.
Copy anything you need to keep. The module holds two frame buffers and never
gives the DMA the one you are reading, so the picture will not tear under
you — but once released, the sensor will overwrite it within about 30 ms.

**`capture(buf=None, timeout=1000)`**

With a buffer, fills it and returns the byte count — the zero-allocation form
for a loop. With no buffer, returns a fresh `bytes`, for a one-shot still
where making the caller size 1.28 MB first is a poor introduction.

**`capture_scaled(dst, pic_w, pic_h, *, x=0, y=0, w=None, h=None, rotate=0, mirror=False, timeout=1000)`**

Scale the frame into a rectangle of a destination picture, using the P4's
Pixel Processing Accelerator. No pixel passes through the CPU: the PPA reads
the camera buffer and writes the destination by DMA.

```python
fb = display_drv.framebuffers()[0]
cam.capture_scaled(fb, display_drv.width, display_drv.height)
display_drv.show()
```

The destination is described as a whole picture plus the block to write
inside it, because that is exactly what a panel framebuffer is — so a
preview hands this its scanout buffer and gets camera to glass in one
hardware operation. `rotate` takes 0, 90, 180 or 270 and `mirror` flips in x,
both free in the same pass. Returns the `(width, height)` written, or `None`
if no frame arrived.

`dst` must be cache-line aligned, since the PPA writes it by DMA — a panel
framebuffer already is. A misaligned buffer raises rather than being quietly
shifted, because the corruption would surface somewhere else entirely.

**`capture_yuy2(buf, width, height, timeout=1000)`**

Converts to YUY2 and scales to `width` x `height` while copying. This exists
because UVC — what a USB webcam speaks — wants YUY2, and doing that
conversion in Python is not viable.

**`capture_jpeg(quality=80, timeout=1000)` → `bytes` | `None`**

JPEG, encoded by the P4's hardware encoder. The sensor's RGB565 goes to the
encoder untouched, so there is no software colour conversion in the path at
all. Measured on a normally-lit room at 800x800, against 1.28 MB raw:

| quality | size | ratio |
|---|---|---|
| 90 | ~144 KB | 8.9:1 |
| 80 | ~90 KB | 14:1 |
| 70 | ~69 KB | 18.5:1 |
| 50 | ~47 KB | 27:1 |

All four calls return `None` (or `0`) when no frame arrived inside the
timeout. At 35 fps a missed frame is a timing fact, not an error, and a
preview loop should simply ask again.

### Controls

First, ask what there is. Drivers differ enormously in how much of a sensor
they expose, and a program that assumes `gain()` exists works beautifully on
the sensor it was written for and raises `OSError` on the next one.

```python
>>> cam.controls()
{'exposure': (2, 235, 1), 'flip': (0, 1, 1), 'mirror': (0, 1, 1)}
```

`controls()` returns only what the driver in your firmware advertises, as
`{name: (minimum, maximum, step)}`. The example above is the real answer from
the OV5647 — three parameters, and no gain control at all.

Numeric controls take a **percentage of that advertised range**:

```python
cam.exposure(60)              # 60% of whatever this sensor offers
cam.exposure()                # read it back
cam.exposure(raw=True)        # (current, minimum, maximum, step), sensor units
cam.exposure(120, raw=True)   # set in sensor units
```

Percent is the default because the raw units are not portable: exposure is in
lines, gain is in driver-specific steps, and both ranges move when the format
does. A literal that suits one mode is quietly wrong in another. `raw=True`
is there for when you do mean the sensor's own numbers.

| Method | Effect |
|---|---|
| `controls()` | `{name: (min, max, step)}` for what this driver implements |
| `exposure(pct=None, *, raw=False)` | Shutter or AE target, depending on the driver |
| `gain(pct=None, *, raw=False)` | Gain. Tries absolute `GAIN`, falls back to analogue `ANGAIN` |
| `flip(on=None)` | Vertical flip, in the sensor. A bool |
| `mirror(on=None)` | Horizontal mirror, in the sensor. A bool |
| `test_pattern(on=None)` | The sensor's internal pattern |

**Reading a control returns `None` when the driver cannot answer**, rather
than a number. This matters more than it looks: the OV5647 driver's
`get_para_value` returns `NOT_SUPPORTED` unconditionally, and an earlier
version of this module ignored that return code and handed back the zero it
had initialised — reporting a confident, plausible exposure of 0 for a
sensor that cannot report anything. Setting still works; only reading is
unavailable, and `None` is how it says so.

`test_pattern()` reads back what was last *asked for*, not what the sensor
reports — the ioctl behind it has no read side. Said plainly here because a
reader who assumed otherwise would trust it during exactly the debugging
session it exists for.

It earns its place in the API. The pattern is generated inside the sensor,
after the pixel array and before everything else — the MIPI lanes, the ISP,
the DMA, the cache invalidate, your blit. If the pattern is clean and the
picture is not, the fault is in front of the sensor: optics, focus, light, a
lens cap. If the pattern is broken too, everything after the pixel array is
suspect. Either answer halves the search.

#### What the OV5647 driver actually supports

Read out of `esp_cam_sensor`'s driver source and confirmed on the board, in
case it saves someone the same hour:

| | set | read | notes |
|---|---|---|---|
| `exposure` | yes, 2–235 | no | `ov5647_set_AE_target` — an auto-exposure target, not a raw shutter time |
| `flip`, `mirror` | yes | no | |
| `gain` | **no** | no | The driver implements neither `GAIN` nor `ANGAIN` |
| `test_pattern` | yes | — | ioctl, set-only by design |

### Inspection

| Method | Returns |
|---|---|
| `size()` | `(width, height, bytes_per_frame)` |
| `controls()` | `{name: (min, max, step)}`, above |
| `sensor()` | `(name, product_id)` |
| `formats()` | `[(name, width, height), ...]` the driver offers |
| `reg(addr, value=None)` | Read or write one sensor register |
| `stats()` | `(frames, dropped, isr_new, isr_done, last_received)` |

`stats()` is the first thing to look at when there is no picture. If
`isr_done` is not climbing, nothing is arriving and the problem is upstream
of this module — power, clock, lanes, or the sensor not streaming. If it is
climbing and `frames` is not, frames are arriving and being rejected, and
`last_received` says how big they were.

### Teardown

```python
cam.deinit()
```

Idempotent, like the rest of PyDevices: calling it twice is not an error, and
neither is calling it on a camera that never opened. Afterwards every method
raises rather than touching freed hardware. `__del__` does the same on a
best-effort basis, and a soft reset (Ctrl-D) tears the camera down before the
heap is wiped — so an interrupted session leaves the CSI controller, the ISP,
the LDO and the sensor's stream all stopped, not half-running into memory
that no longer exists.

## Examples

| File | What it shows |
|---|---|
| [`camera_preview.py`](examples/camera_preview.py) | The camera on the board's own panel. No PC, no network, no USB |
| [`camera_snapshot.py`](examples/camera_snapshot.py) | A still to the filesystem as hardware-encoded JPEG |
| [`camera_mjpeg.py`](examples/camera_mjpeg.py) | An MJPEG server any browser can open |
| [`camera_still.py`](examples/camera_still.py) | Live view, BOOT button takes the picture. The one that feels like a camera |
| [`camera_controls.py`](examples/camera_controls.py) | Discovering what a sensor supports, then sweeping it on screen |
| [`camera_frame.py`](examples/camera_frame.py) | Zero-copy `frame()` loop with `stats()` |
| [`camera_transform.py`](examples/camera_transform.py) | `capture_scaled` with rotate, mirror, and a cropped inset |
| [`camera_formats.py`](examples/camera_formats.py) | List `formats()`, then construct `Camera(...)` with a named format |
| [`camera_diag.py`](examples/camera_diag.py) | Bring-up: stats, test pattern, optional `reg()` |

The examples take the camera from `board_config`, so they run unchanged on
any board whose config provides one. On a board without one, construct a
`Camera` directly with your own pins -- see `camera_formats.py`.

`camera_still.py` is the one that uses `appdev` as the scheduler, which is
the house idiom for an application with input; the rest are plain scripts
because they have nothing to schedule.

USB webcam (the board presenting as UVC) lives in
[`usbif`](https://github.com/PyDevices/usbif)'s `examples/usbif_webcam.py`,
which sources frames from this module when a camera is present.

Known hole: [`available()` always returns True](https://github.com/PyDevices/cameraif/issues/1).

## Performance, measured

On an ESP32-P4 with an OV5647 at 800x800 RGB565:

| | rate | what dominates |
|---|---|---|
| Sensor delivering frames | 36 fps | the format's own frame rate |
| `capture_jpeg(70)` | 36 fps | nothing — it keeps up with the sensor |
| `frame()` — zero copy | 35 fps | waiting for the next frame |
| `capture_scaled()` to the panel | 34 fps | the PPA, ~1 ms |
| Full preview loop, incl. `show()` | 18.6 fps | the panel's refresh, 24 ms |
| MJPEG over Wi-Fi, quality 40 | 15 fps | the network |
| MJPEG over Wi-Fi, quality 70 | 7 fps | the network |

Everything except the panel refresh now runs at the rate the sensor
delivers. Two things got it there.

**The PPA does the scaling.** `capture_scaled()` hands the frame to the
Pixel Processing Accelerator, which reads the camera buffer and writes the
panel's scanout buffer by DMA. The row-at-a-time version it replaced was 720
blit calls per frame and ran at 4.7 fps.

**And a 97 ms wait that should not have existed.** Every capture was calling
`esp_cam_ctlr_receive()` to hand the driver a buffer. But `esp_cam_ctlr_csi`
reads its transaction queue only in the `else` branch of `if
(ctlr->cbs.on_get_new_trans)` — and this module must register that callback,
because it is how the driver is told which of the two buffers Python is not
holding. So the queue was filled once and never drained again, and every
subsequent `receive()` blocked for its whole timeout. The camera was
delivering a frame every 28 ms and this module was collecting one every
130 ms.

It presented as a slow camera, which is the most expensive kind of bug to
have: everything worked, so there was nothing to debug. It was found by
measuring the parts separately instead of trusting a guess about which part
was slow — the first guess, that the cache invalidate dominated, was wrong
by a factor of fourteen.

`capture_jpeg()` matching the sensor rate is not a rounding artefact: the
hardware encoder genuinely costs less than the 28 ms between frames.

## Known hardware defect

The OV5647's `MIPI_CTRL00` register (0x4800) comes up with clock-lane gating
enabled, and the sensor then produces nothing over MIPI. `cameraif` writes
`0x34` to it for this sensor specifically, gated on its product ID.

The distinguishing bit is `CLOCK_LANE_GATE`, established by measurement
rather than assumption: `0x04` and `0x14` both fail, `0x34` works, and each
trial was run with a full teardown in between so no trial inherited the
previous one's state. Written up in
[`docs/upstream-reports/ov5647-mipi-ctrl00.md`](docs/upstream-reports/ov5647-mipi-ctrl00.md)
and filed upstream as
[esp-video-components#97](https://github.com/espressif/esp-video-components/issues/97).

## Licence

MIT.
