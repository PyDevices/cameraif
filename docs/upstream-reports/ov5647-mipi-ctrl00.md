# OV5647 never transmits: `ov5647_set_stream()` discards its computed MIPI_CTRL00

**Target:** [espressif/esp-video-components](https://github.com/espressif/esp-video-components) (`esp_cam_sensor`)
**Component version:** 2.5.0 · **IDF:** v5.5.2 · **Target:** ESP32-P4
**Status:** ready to file. Rewritten 2026-09-04 after measurement corrected an earlier draft; see *How we got this wrong twice*.
**Likely duplicate/cause of:** [esp-idf#19032](https://github.com/espressif/esp-idf/issues/19032) (same symptom, this board family, unanswered since 2026-08-31)

---

## Symptom

An OV5647 on the P4's MIPI-CSI port is detected, configured and told to
stream — and never sends a single MIPI packet.

Everything on the control path succeeds. The chip ID matches over SCCB, the
whole register set is written and acknowledged, `esp_cam_sensor_set_format()`
returns `ESP_OK`, and `ESP_CAM_SENSOR_IOC_S_STREAM` returns `ESP_OK`. The CSI
controller and ISP initialise. Then nothing arrives: no transaction ever
completes, so every capture times out or returns a buffer the DMA never
wrote.

That combination — perfect I2C, dead data lanes — is what makes this
expensive to diagnose. It looks like wiring, a dark room, a broken ISP, or a
wrong lane rate, and it is none of them.

## Cause

`sensors/ov5647/ov5647.c`, `ov5647_set_stream()`, around line 320:

```c
uint8_t val = OV5647_MIPI_CTRL00_BUS_IDLE;
if (enable) {
#if CSI2_NONCONTINUOUS_CLOCK
    val |= OV5647_MIPI_CTRL00_CLOCK_LANE_GATE | OV5647_MIPI_CTRL00_LINE_SYNC_ENABLE;
#endif
} else {
    val |= OV5647_MIPI_CTRL00_CLOCK_LANE_GATE | OV5647_MIPI_CTRL00_CLOCK_LANE_DISABLE;
}

ret = ov5647_write(dev->sccb_handle, 0x4800,
                   CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE ? 0x14 : 0x00);
```

`val` is computed across four lines and then **discarded**. The write uses a
hardcoded literal instead.

With the default `CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE=y` that literal is
`0x14` — `LINE_SYNC_ENABLE | BUS_IDLE`, line sync **without** the clock-lane
gate. On a non-continuous-clock link the sensor then never transmits.

Note this is not merely an unlucky constant: the function already computes
the correct value for both clock modes and throws it away. Linux's `ov5647`
driver never produces `0x14`, because there line sync is only ever set
together with the clock-lane gate.

## Measurements

ESP32-P4 (Waveshare ESP32-P4-WIFI6-Touch-LCD-4B), OV5647 on the CSI
connector, `MIPI_2lane_24Minput_RAW8_800x800_50fps`, two lanes at 400 Mbps.
Frames counted from the CSI driver's own `on_trans_finished` callback over
800 ms — so these count real DMA completions, not API return codes.

| MIPI_CTRL00 | Bits | Frames in 800 ms |
|---|---|---|
| `0x14` (shipped default) | LINE_SYNC \| BUS_IDLE | **0** |
| `0x04` | BUS_IDLE | **0** |
| `0x24` | CLOCK_LANE_GATE \| BUS_IDLE | 28 |
| `0x34` | CLOCK_LANE_GATE \| LINE_SYNC \| BUS_IDLE | 29 |

`0x34` is exactly what the discarded `val` evaluates to with
`CSI2_NONCONTINUOUS_CLOCK` defined. Writing it after the CSI receiver is
started gives a continuous 28–29 fps stream, a full 1,280,000-byte frame per
capture, 153 distinct pixel values, and a picture that responds to exposure
and gain.

The distinguishing bit is **CLOCK_LANE_GATE**. Both values that set it
stream; neither value without it does.

**One caveat, so a maintainer who sees otherwise does not doubt the rest.**
A second session here measured `0x04` streaming at 28 fps, under a different
protocol: it wrote `0x0100 = 0`, then `0x4800 = 0x04`, then `0x0100 = 1` — a
stop/start of the sensor's standby bit *after* the receiver was already
running. The table above leaves `0x0100` untouched and gets 0 frames for the
same value. Both results are correct under their own conditions.

So a standby restart can apparently also start transmission with a gate-less
value. We did not chase why, and it does not affect the diagnosis or the fix:
with the shipped default and no such restart — which is what every consumer
of this driver actually does — the sensor does not transmit.

## Suggested fix

Write the value the function already computed:

```c
ret = ov5647_write(dev->sccb_handle, 0x4800, val);
```

`CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE` should then select
`LINE_SYNC_ENABLE` within that computation rather than replace the whole
register value.

## How we got this wrong twice, and how to test it

This is worth stating because it will mislead the next person the same way.

**Once a sensor is transmitting, every subsequent MIPI_CTRL00 value keeps it
transmitting.** A sweep therefore only ever shows which value *started* the
stream; every row after the first reads as a success regardless of what it
wrote. Two independent sessions here produced contradictory tables from that
one artefact:

- One concluded the value was irrelevant and only the *timing* mattered,
  having measured a sequence where the first successful row left the sensor
  streaming for all the rest.
- The other concluded `0x14` specifically was the poison value, having
  re-written whatever the driver had already put there — a no-op that was
  recorded as a control.

**So: deinit and re-init the whole pipeline between every candidate value,
and count real DMA completions rather than checking API return codes.** Every
call on this path returns `ESP_OK` whether or not a single pixel moves.

Reading the CSI host PHY directly is the decisive instrument, and worth
having in any bug report on this: at CSI base `0x5009F000`, `PHY_RX` offset
`0x48` bit 17 is `RXCLKACTIVEHS`, and `PHY_STOPSTATE` offset `0x4C` bits 0–1
are the data lanes. With the shipped default the lanes sit parked, there is
no high-speed clock, and every error counter reads zero — which is what
proves the sensor is silent rather than the receiver deaf.

## Workaround

Consumers can write the register themselves after starting the receiver.
`cameraif` does this, gated on `dev->id.pid == 0x5647` so it cannot touch a
different sensor's registers, and it becomes a harmless rewrite of the same
value once this is fixed upstream.
