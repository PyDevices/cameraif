"""What this camera can actually be told to do -- discovered, not assumed.

    mpremote run camera_controls.py

Sensor drivers vary enormously in how much of the hardware they expose. The
OV5647's driver in `esp_cam_sensor`, for instance, implements exactly three
parameters, offers no gain control at all, and cannot read any of them back:
its `get_para_value` returns NOT_SUPPORTED unconditionally. A different
sensor may offer a dozen.

So this example asks first. `cam.controls()` returns only the parameters the
driver in *your* firmware advertises, with the range it advertises for each,
and everything below is driven from that. Copy this shape rather than
hard-coding a control list -- code that assumes `gain()` exists works
beautifully on the sensor it was written for and raises OSError on the next
one.

The sensor's test pattern gets a section of its own at the end. It is the
most useful debugging tool here.
"""

import time

import board_config
from board_config import display_drv as display

cam = board_config.camera
_w = min(cam.size()[0], display.width)
_h = min(cam.size()[1], display.height)
_sx, _sy = (cam.size()[0] - _w) // 2, (cam.size()[1] - _h) // 2
_dx, _dy = (display.width - _w) // 2, (display.height - _h) // 2
_stride = cam.size()[0] * 2


def show(seconds=2):
    """Paint what the camera sees, for a while."""
    end = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        view = cam.frame(500)
        if view is None:
            continue
        for y in range(_h):
            off = (_sy + y) * _stride + _sx * 2
            display.blit_rect(view[off:off + _w * 2], _dx, _dy + y, _w, 1)
        display.show()


def main():
    print("sensor %s, id 0x%04x" % cam.sensor())
    print("formats:")
    for name, w, h in cam.formats():
        print("   %-48s %dx%d" % (name, w, h))

    controls = cam.controls()
    print("\ncontrols this driver implements:")
    for name in sorted(controls):
        lo, hi, step = controls[name]
        print("   %-12s %d..%d step %d" % (name, lo, hi, step))
    missing = {"exposure", "gain", "flip", "mirror"} - set(controls)
    if missing:
        print("   (not offered by this driver: %s)" % ", ".join(sorted(missing)))

    try:
        # Percent of the advertised range, not raw units. The units are not
        # portable -- exposure is in lines, gain in driver-specific steps --
        # and both ranges move with the format, so a literal that suits one
        # mode is quietly wrong in another. Pass raw=True when you do mean
        # the sensor's own numbers.
        for name in ("exposure", "gain"):
            if name not in controls:
                continue
            print("\n%s sweep" % name)
            for pct in (10, 40, 70, 100):
                getattr(cam, name)(pct)
                print("   %s %d%%" % (name, pct))
                show(1.5)
            getattr(cam, name)(50)

        if "flip" in controls or "mirror" in controls:
            print("\norientation")
            for flip, mirror, label in ((False, False, "normal"),
                                        (True, False, "flipped (upside down)"),
                                        (False, True, "mirrored (as in a mirror)"),
                                        (True, True, "both (180 degrees)")):
                if "flip" in controls:
                    cam.flip(flip)
                if "mirror" in controls:
                    cam.mirror(mirror)
                print("   %s" % label)
                show(2)
            if "flip" in controls:
                cam.flip(False)
            if "mirror" in controls:
                cam.mirror(False)

        # Reading back is a separate question from setting. This driver
        # cannot, and says so by returning None rather than a plausible zero.
        print("\nread-back:")
        for name in sorted(controls):
            print("   %-12s -> %r" % (name, getattr(cam, name)()))

        print("\nsensor test pattern")
        # Generated inside the sensor, after the pixel array and before
        # everything else: the MIPI lanes, the ISP, the DMA, the cache
        # invalidate, your blit. Clean pattern with a bad picture means the
        # fault is in front of the sensor -- optics, focus, light, a lens
        # cap. Broken pattern means everything after the pixel array is
        # suspect. Either answer halves the search.
        try:
            cam.test_pattern(True)
            show(4)
            cam.test_pattern(False)
            show(2)
        except OSError as e:
            print("   not available: %s" % e)

        # Registers, for when the datasheet and the picture disagree.
        # 0x300a/0x300b is the OV5647's product ID.
        print("\nchip id from registers: 0x%02x%02x" % (cam.reg(0x300A), cam.reg(0x300B)))
        print("frames/dropped/isr_new/isr_done/last:", cam.stats())
    finally:
        cam.deinit()


if __name__ == "__main__":
    main()
