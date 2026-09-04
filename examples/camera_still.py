"""A camera you actually use: live view on the panel, BOOT button takes the shot.

    mpremote run camera_still.py

Point it at something, press the BOOT button, and a JPEG appears on the
board's filesystem. Press it again for another. Fetch them with::

    mpremote ls
    mpremote cp :photo_001.jpg .

**How the button gets here.** `board_config` publishes the BOOT button
through `keypad_read`, and `appdev` turns that into ordinary KEYDOWN events
-- the same events a USB keyboard would produce. Nothing in this file knows
which GPIO it is or that it is active-low, which is why the same file works
on a board that wires its shutter somewhere else entirely.

**On the shape of this file.** `appdev` is the scheduler. It polls the input
devices, dispatches the events, and keeps the program alive after the last
line runs -- so there is no `while True` here and no `app.run()` at the
bottom. The preview is on `app.every()` because repainting is genuinely
periodic work of our own; the shutter is on `app.on()` because it happens
when it happens. Anything drawn must be followed by `display_drv.show()`,
which is what actually puts the back buffer on the glass.

**Stopping it.** Ctrl-C will not: `appdev` drives this from a hardware timer,
and interrupting the callback leaves the timer armed to fire the next one.
Press the board's reset button, or::

    python -m esptool --chip esp32p4 --port COMn --after hard_reset chip-id
"""

import appdev
import board_config
from board_config import display_drv as display

QUALITY = 90
PREVIEW_MS = 30

app = appdev.App(board_config)
cam = board_config.camera

# The panel's own scanout buffer. capture_scaled() scales the frame into it
# with the PPA, by DMA, so the live view costs the CPU one call per frame.
framebuffer = display.framebuffers()[0]

count = 0
busy = False


def next_filename():
    """First unused photo_NNN.jpg.

    Deliberately not a counter starting at 1 each run: a board that reboots
    would silently overwrite yesterday's pictures, and the failure would only
    surface when someone went looking for one.
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


def draw_preview(_=None):
    global busy
    if busy:
        return              # never paint over a shot being taken
    if cam.capture_scaled(framebuffer, display.width, display.height,
                          timeout=200) is None:
        return
    display.show()


def shutter(_event):
    global count, busy
    busy = True
    try:
        # A white frame, before the capture rather than after: it is the only
        # feedback there is on a board with no shutter sound, and it should
        # mark the moment the picture was taken.
        display.fill(0xFFFF)
        display.show()

        img = cam.capture_jpeg(QUALITY)
        if img is None:
            print("shutter: no frame")
            return
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
        busy = False


app.on(app.events.KEYDOWN, shutter)
app.every(PREVIEW_MS, draw_preview)

print("Live view up. Press BOOT to take a picture.")
