"""The board watching the room, on its own screen.

A MIPI-CSI camera in one side of the ESP32-P4 and a MIPI-DSI panel out the
other, with nothing in between but the board -- no PC, no network, no USB.

    mpremote run camera_preview.py

**Nothing here touches a pixel.** `capture_scaled()` hands the frame to the
P4's Pixel Processing Accelerator, which reads the camera's buffer and writes
the panel's scanout buffer by DMA, scaling on the way. The CPU issues one
call per frame and then waits. The whole 800x800 sensor image lands on the
720x720 panel -- scaled, not cropped, so nothing at the edges is lost.

For contrast, the obvious version of this loop -- slice each row out of the
frame and blit it -- is 720 calls per frame and measured 4.7 fps. This is
18.6, and the remaining cost is the panel's own refresh, not the camera.
"""

import time

import board_config
from board_config import display_drv as display


def main():
    cam = board_config.camera
    print("camera %s %dx%d -> panel %dx%d"
          % ((cam.sensor()[0],) + cam.size()[:2] + (display.width, display.height)))

    # The panel's scanout buffer, which is already DMA-aligned because the
    # display controller reads it the same way. capture_scaled() writes here
    # directly; there is no intermediate buffer to allocate.
    framebuffer = display.framebuffers()[0]

    frames = 0
    t0 = time.ticks_ms()
    try:
        while True:
            if cam.capture_scaled(framebuffer, display.width, display.height,
                                  timeout=500) is None:
                continue    # a missed frame at 35 fps is timing, not failure
            # Required: the panel only shows the buffer after show() promotes
            # it. Leave it out and every frame lands and nothing changes,
            # which looks exactly like a dead camera.
            display.show()
            frames += 1
            if frames % 30 == 0:
                dt = time.ticks_diff(time.ticks_ms(), t0)
                print("%d frames, %.1f fps" % (frames, frames * 1000 / dt))
    except KeyboardInterrupt:
        print("stopped after %d frames" % frames)
    finally:
        cam.deinit()


if __name__ == "__main__":
    main()
