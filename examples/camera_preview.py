"""The board watching the room, on its own screen.

A MIPI-CSI camera in one side of the ESP32-P4 and a MIPI-DSI panel out the
other, with nothing in between but the board -- no PC, no network, no USB.

    mpremote run camera_preview.py

**Why it crops instead of scaling.** The sensor gives 800x800 and this panel
is 720x720. Scaling needs a resampler, and in Python that means arithmetic on
every one of half a million pixels -- which measured 0.8 fps, an order of
magnitude below the sensor. Cropping needs no arithmetic at all: each output
row is a slice of an input row. It costs the outer 5% of the picture and buys
a preview you can actually watch.

**Why ``frame()`` and not ``capture()``.** ``capture()`` copies the frame into
a buffer you own, which for 1.28 MB is about 100 ms of pure memory traffic.
``frame()`` hands back a memoryview of the buffer the DMA just finished with,
so a preview reads the pixels where they landed. The borrow is real: that view
is only yours until the next ``frame()`` call, which gives the buffer back to
the driver.
"""

import time

import board_config
from board_config import display_drv as display


def main():
    cam = board_config.camera
    cam_w, cam_h, _ = cam.size()
    print("camera %s %dx%d -> panel %dx%d"
          % (cam.sensor()[0], cam_w, cam_h, display.width, display.height))

    w = min(cam_w, display.width)
    h = min(cam_h, display.height)
    src_x, src_y = (cam_w - w) // 2, (cam_h - h) // 2
    dst_x, dst_y = (display.width - w) // 2, (display.height - h) // 2
    stride = cam_w * 2
    row_bytes = w * 2
    print("centre %dx%d crop" % (w, h))

    display.fill(0)
    frames = 0
    t0 = time.ticks_ms()
    try:
        while True:
            view = cam.frame(500)
            if view is None:
                continue        # a missed frame at 35 fps is timing, not failure
            for y in range(h):
                off = (src_y + y) * stride + src_x * 2
                display.blit_rect(view[off:off + row_bytes], dst_x, dst_y + y, w, 1)
            # Not optional: this panel double-buffers, so blits land in the
            # back buffer and only show() promotes it. Leave it out and the
            # screen never changes, which looks exactly like a dead camera.
            display.show()
            frames += 1
            if frames % 20 == 0:
                dt = time.ticks_diff(time.ticks_ms(), t0)
                print("%d frames, %.1f fps" % (frames, frames * 1000 / dt))
    except KeyboardInterrupt:
        print("stopped after %d frames" % frames)
    finally:
        cam.deinit()


if __name__ == "__main__":
    main()
