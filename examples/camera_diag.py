"""Bring-up diagnostics when the picture is wrong or missing.

``stats()`` is the first thing to look at. If ``isr_done`` is not climbing,
nothing is arriving and the problem is upstream of this module -- power,
clock, lanes, or the sensor not streaming. If it is climbing and ``frames``
is not, frames are arriving and being rejected; ``last_received`` says how
big they were.

The sensor's test pattern is generated inside the sensor, after the pixel
array and before everything else. Pattern clean + picture broken means the
fault is in front of the sensor (optics, light, lens cap). Pattern broken
too means everything after the pixel array is suspect.

    mpremote run camera_diag.py

``reg()`` is there for a deliberate poke when you already know which
register you mean -- not a tour of the sensor map.
"""

import time

import board_config
from board_config import display_drv as display


def main():
    cam = board_config.camera
    print("sensor %s id 0x%04x" % cam.sensor())
    print("size", cam.size())
    print("formats:")
    for name, w, h in cam.formats():
        print("  ", name, w, h)
    print("controls", cam.controls())

    fb = display.framebuffers()[0]

    print("\n-- live scene, 2 s --")
    cam.test_pattern(False)
    end = time.ticks_add(time.ticks_ms(), 2000)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        cam.capture_scaled(fb, display.width, display.height, timeout=500)
        display.show()
    print("stats", cam.stats())

    print("\n-- test pattern, 2 s --")
    cam.test_pattern(True)
    end = time.ticks_add(time.ticks_ms(), 2000)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        cam.capture_scaled(fb, display.width, display.height, timeout=500)
        display.show()
    print("stats", cam.stats())
    print("test_pattern() reads back last ask:", cam.test_pattern())

    # A single register read as a smoke check that SCCB still answers.
    # 0x300A/0x300B are the OV5647 chip id registers; other sensors will
    # return something else or raise -- that is still information.
    try:
        print("reg(0x300A)=", hex(cam.reg(0x300A)),
              "reg(0x300B)=", hex(cam.reg(0x300B)))
    except Exception as exc:
        print("reg() not usable on this sensor/driver:", exc)

    cam.test_pattern(False)
    cam.deinit()
    print("done")


if __name__ == "__main__":
    main()
