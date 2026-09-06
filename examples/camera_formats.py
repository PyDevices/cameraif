"""List the sensor's formats, then open one by name.

``board_config.camera`` picks the driver's default. This example shows the
board-agnostic constructor: pins from the board, format chosen from
``formats()``. Adding a different camera is a build-time ``CONFIG_CAMERA_*``,
not a code change here -- which is why the class is ``Camera``, not ``OV5647``.

    mpremote run camera_formats.py

Closes the board_config camera first (only one ``Camera`` may exist -- the
P4 has one CSI controller), then constructs a fresh one with an explicit
format.
"""

import time

import board_config
import cameraif
from board_config import display_drv as display


def _pins_from_board():
    """Reuse whatever board_config used to open its camera."""
    # board_config typically keeps constructor kwargs on the helper; fall
    # back to the documented Waveshare P4-WIFI6-Touch-LCD-4B defaults if not.
    cam_helper = getattr(board_config, "camera_pins", None)
    if callable(cam_helper):
        return cam_helper()
    pins = getattr(board_config, "CAMERA_PINS", None)
    if isinstance(pins, dict):
        return pins
    # Waveshare ESP32-P4-WIFI6-Touch-LCD-4B (the reference board).
    return {"sda": 7, "scl": 8, "reset": 21, "i2c": 0}


def main():
    # Peek at formats through the board's already-open camera, then release
    # it so we can construct our own.
    cam = board_config.camera
    print("default sensor %s id 0x%04x" % cam.sensor())
    offered = list(cam.formats())
    print("formats this driver offers:")
    for name, w, h in offered:
        print("   %-48s %dx%d" % (name, w, h))
    cam.deinit()

    if not offered:
        print("no formats; nothing to open")
        return

    # Prefer a format that fits the panel when one exists; else the first.
    choice = offered[0]
    for name, w, h in offered:
        if w <= display.width and h <= display.height:
            choice = (name, w, h)
            break
    name, w, h = choice
    print("opening format %r (%dx%d)" % (name, w, h))

    pins = _pins_from_board()
    cam = cameraif.Camera(pins["sda"], pins["scl"],
                          format=name,
                          i2c=pins.get("i2c", 0),
                          reset=pins.get("reset", -1),
                          pwdn=pins.get("pwdn", -1),
                          xclk=pins.get("xclk", -1))
    print("now", cam.sensor(), cam.size())

    fb = display.framebuffers()[0]
    frames = 0
    t0 = time.ticks_ms()
    try:
        while frames < 60:
            if cam.capture_scaled(fb, display.width, display.height,
                                  timeout=500) is None:
                continue
            display.show()
            frames += 1
        dt = time.ticks_diff(time.ticks_ms(), t0)
        print("%d frames in %d ms (%.1f fps)" % (frames, dt, frames * 1000 / dt))
    finally:
        cam.deinit()


if __name__ == "__main__":
    main()
