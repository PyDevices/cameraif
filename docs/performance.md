# Performance, and how it got there

The numbers are on the [README](../README.md#performance-measured). This page is
the part a reader only wants if they came asking: what the two changes were that
took a preview loop from 4.7 fps to the sensor's own rate, and how the second one
was found.

## The PPA does the scaling

`capture_scaled()` hands the frame to the Pixel Processing Accelerator, which
reads the camera buffer and writes the panel's scanout buffer by DMA. The
row-at-a-time version it replaced was 720 blit calls per frame and ran at
**4.7 fps**.

## A 97 ms wait that should not have existed

Every capture was calling `esp_cam_ctlr_receive()` to hand the driver a buffer.
But `esp_cam_ctlr_csi` reads its transaction queue only in the `else` branch of
`if (ctlr->cbs.on_get_new_trans)` — and this module *must* register that
callback, because it is how the driver is told which of the two buffers Python is
not holding.

So the queue was filled once and never drained again, and every subsequent
`receive()` blocked for its whole timeout. The camera was delivering a frame
every 28 ms; this module was collecting one every **130 ms**.

## Why it took so long to see

It presented as a slow camera, which is the most expensive kind of bug to have:
everything worked, so there was nothing to debug. No error, no dropped frame, no
log line — just a number lower than it should have been, with no reason to
suspect any particular part.

It was found by measuring the parts separately instead of trusting a guess about
which part was slow. **The first guess — that the cache invalidate dominated —
was wrong by a factor of fourteen.** That is the transferable lesson: on a path
where every call returns `ESP_OK`, a guess about which stage costs the time is
worth exactly nothing until it is timed on its own.

The same trap in a different costume, on the same sensor, is in
[`upstream-reports/ov5647-mipi-ctrl00.md`](upstream-reports/ov5647-mipi-ctrl00.md)
— count DMA completions rather than return codes, because every call on that path
also reports success whether or not a pixel moves.
