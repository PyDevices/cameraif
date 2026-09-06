"""Scale, rotate, mirror, and crop the camera onto the panel via the PPA.

``capture_scaled`` hands the frame to the Pixel Processing Accelerator:
rotate and mirror are free in the same pass as the scale. The destination is
described as a whole picture plus the rectangle to write inside it -- exactly
a panel framebuffer.

    mpremote run camera_transform.py

Cycles through a few orientations so the effect is obvious on the glass.
"""

import time

import board_config
from board_config import display_drv as display


ORIENTATIONS = (
    (0, False, "normal"),
    (90, False, "rotate 90"),
    (180, False, "rotate 180"),
    (270, False, "rotate 270"),
    (0, True, "mirrored"),
    (180, True, "rotate 180 + mirror"),
)


def main(hold_s=2):
    cam = board_config.camera
    fb = display.framebuffers()[0]
    print("sensor %s %dx%d -> panel %dx%d"
          % ((cam.sensor()[0],) + cam.size()[:2] + (display.width, display.height)))

    try:
        for rotate, mirror, label in ORIENTATIONS:
            print(label)
            end = time.ticks_add(time.ticks_ms(), int(hold_s * 1000))
            while time.ticks_diff(end, time.ticks_ms()) > 0:
                # Full panel; crop example: pass x,y,w,h to write into a
                # quadrant instead -- same call, smaller rectangle.
                if cam.capture_scaled(fb, display.width, display.height,
                                      rotate=rotate, mirror=mirror,
                                      timeout=500) is None:
                    continue
                display.show()
            # And a cropped inset: centre quarter of the panel.
            w, h = display.width // 2, display.height // 2
            x, y = (display.width - w) // 2, (display.height - h) // 2
            print("  + cropped inset %dx%d at (%d,%d)" % (w, h, x, y))
            display.fill(0)
            if cam.capture_scaled(fb, display.width, display.height,
                                  x=x, y=y, w=w, h=h,
                                  rotate=rotate, mirror=mirror,
                                  timeout=500) is not None:
                display.show()
            time.sleep_ms(800)
    except KeyboardInterrupt:
        print("stopped")
    finally:
        cam.deinit()


if __name__ == "__main__":
    main()
