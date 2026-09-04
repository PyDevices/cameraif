"""A camera you actually use: live view on the panel, BOOT button takes the shot.

    mpremote run camera_still.py

Point it at something, press the BOOT button, and a JPEG appears on the
board's filesystem. Press again for another. Fetch them with::

    mpremote ls
    mpremote cp :photo_001.jpg .

**How the button gets here.** `board_config` publishes the BOOT button as
`keypad_read()`, which returns the key codes currently held. Nothing in this
file knows which GPIO it is or that it is active-low, so the same file works
on a board that wires its shutter somewhere else. The press is an edge --
held now, not held last time -- because a button read every 50 ms is held
down across many reads and a level would fire the shutter continuously.

**Why there is no `appdev.App` here, and when you should still use one.**
`appdev` is the right home for most applications: it owns the scheduler,
polls the input devices, dispatches events, and calls `show()`. Use it. This
program is the exception, and the reason is measured rather than assumed --
on this board, the identical preview loop runs at:

    18.7 fps   plain loop, board_config.keypad_read() read directly
     2.0 fps   the same loop with an appdev.App constructed

and the App version then trips the interrupt watchdog and resets the board.
Nine times slower is not a tuning problem, so this example does the simple
thing that works. A camera preview is an unusual load -- it saturates the
memory bus with DMA from the sensor, the scaler and the panel at once -- and
that appears to be what the App's 10 ms service timer collides with. If you
are writing something that is not a continuous full-frame video loop, reach
for `appdev` first.

**Stopping it.** Ctrl-C works, because the loop is ours.
"""

import board_config
from board_config import display_drv as display

QUALITY = 90

cam = board_config.camera

# The panel's scanout buffer. capture_scaled() scales the camera frame into
# it with the PPA, by DMA, so the live view costs the CPU one call per frame.
framebuffer = display.framebuffers()[0]

count = 0


def next_filename():
    """First unused photo_NNN.jpg.

    Deliberately not a counter starting at 1 each run: a board that reboots
    would silently overwrite yesterday's pictures, and that failure only
    surfaces when someone goes looking for one.
    """
    import os

    try:
        existing = set(os.listdir())
    except OSError:
        existing = set()
    n = 1
    while ("photo_%03d.jpg" % n) in existing:
        n += 1
    return "photo_%03d.jpg" % n


def take_photo():
    global count
    # A white frame, before the capture rather than after: it is the only
    # feedback there is on a board with no shutter sound, and it should mark
    # the moment the picture was taken.
    display.fill(0xFFFF)
    display.show()
    try:
        img = cam.capture_jpeg(QUALITY)
        if img is None:
            print("shutter: no frame")
            return
        # A truncated encode still returns bytes. A file that is the right
        # size but has no end-of-image marker fails much later, on somebody
        # else's computer.
        if img[:2] != b"\xff\xd8" or img[-2:] != b"\xff\xd9":
            print("shutter: encoder returned %d bytes that are not a JPEG" % len(img))
            return
        name = next_filename()
        with open(name, "wb") as f:
            f.write(img)
        count += 1
        print("%s  %d bytes  (%d this session)" % (name, len(img), count))
    finally:
        display.fill(0)


def main():
    print("Live view up. Press BOOT to take a picture.")
    held = set()
    try:
        while True:
            now = set(board_config.keypad_read() or ())
            pressed = now - held      # the edge, not the level
            held = now
            if pressed:
                take_photo()
            elif cam.capture_scaled(framebuffer, display.width, display.height,
                                    timeout=200) is not None:
                # Required: the panel only shows a buffer after show()
                # promotes it.
                display.show()
    except KeyboardInterrupt:
        print("stopped after %d picture%s" % (count, "" if count == 1 else "s"))
    finally:
        cam.deinit()


if __name__ == "__main__":
    main()
