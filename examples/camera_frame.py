"""Zero-copy frames from the camera, with stats.

``frame()`` returns a memoryview of the buffer the DMA just filled -- no
copy. Valid until the next ``frame()`` or ``capture()`` on this camera.
Prints ``stats()`` so you can see whether frames are arriving (ISR climbing)
or being dropped.

    mpremote run camera_frame.py

This is the "I have the pixels" use, not the panel use. For the camera on
the board's own screen see ``camera_preview.py``.
"""

import time

import board_config


def main(seconds=5):
    cam = board_config.camera
    print("sensor %s %dx%d" % ((cam.sensor()[0],) + cam.size()[:2]))

    frames = 0
    t0 = time.ticks_ms()
    deadline = time.ticks_add(t0, seconds * 1000)
    try:
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            mv = cam.frame(500)
            if mv is None:
                continue
            frames += 1
            if frames % 30 == 0:
                st = cam.stats()
                dt = time.ticks_diff(time.ticks_ms(), t0)
                print("%d frames, %.1f fps, %d bytes; stats %r"
                      % (frames, frames * 1000 / dt, len(mv), st))
    except KeyboardInterrupt:
        pass
    finally:
        print("final stats", cam.stats())
        cam.deinit()


if __name__ == "__main__":
    main()
