"""Take a still and write it to the filesystem, as a JPEG.

    mpremote run camera_snapshot.py
    mpremote cp :snapshot.jpg .

The ESP32-P4 has a JPEG encoder in silicon, and ``capture_jpeg()`` uses it.
The sensor's RGB565 goes to the encoder untouched -- no software colour
conversion anywhere in the path -- so an 800x800 still costs a few
milliseconds rather than the many seconds a MicroPython JPEG encoder would.

Quality is the usual 1-100 knob. On a normally-lit room this camera gives
roughly:

    quality 90    ~140 KB     9:1
    quality 80     ~90 KB    14:1
    quality 70     ~70 KB    18:1
    quality 50     ~47 KB    27:1

against 1.28 MB of raw RGB565.
"""

import board_config

FILENAME = "snapshot.jpg"
QUALITY = 85


def main():
    cam = board_config.camera
    try:
        # The first frame after startup is whatever the sensor's auto exposure
        # had settled on before you asked -- often the previous scene, or the
        # black it powered up with. Give it a few frames to converge.
        for _ in range(5):
            cam.frame(500)

        img = cam.capture_jpeg(QUALITY)
        if img is None:
            print("no frame")
            return

        # Check the JPEG is a JPEG. A truncated encode still returns bytes,
        # and a file that is the right size but has no end-of-image marker
        # fails much later, on someone else's computer.
        if img[:2] != b"\xff\xd8" or img[-2:] != b"\xff\xd9":
            print("encoder produced %d bytes that are not a JPEG" % len(img))
            return

        with open(FILENAME, "wb") as f:
            f.write(img)
        w, h, raw = cam.size()
        print("%s: %dx%d, %d bytes (%.1f:1)" % (FILENAME, w, h, len(img), raw / len(img)))
    finally:
        cam.deinit()


if __name__ == "__main__":
    main()
