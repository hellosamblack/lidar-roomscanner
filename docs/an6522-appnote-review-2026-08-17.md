# AN6522 ("Guidelines for tuning ranging profiles with VL53L9CX") review (2026-08-17)

Source: `references/datasheets/NUCLEO-VL53L9CX/an6522-guidelines-for-tuning-ranging-profiles-with-vl53l9cx-stmicroelectronics.pdf`
(Rev 1, July 2026, 9 pages). **Already in the repo** — added `a418131` (2026-08-03) and mined
that same day for the I3C effective-throughput correction (10 Mbps, not the raw 12.5 MHz SDR
clock) and the MIPI-vs-I3C frame-rate-ceiling explanation; see `host/src/roomscan/profiles.py`
module docstring ("I3C ToF bus airtime", "Measured hardware ceiling") and
`docs/vl53l9cx-datasheet-notes.md`. This is a second, goal-directed pass — "did the 2026-08-03
mining get everything, or did later work (the 2026-08-05 preset redesign, `776622d`) leave
something on the table?" — reading the doc section-by-section against the current preset design.

## (A) Should adopt / worth an owner decision

### A1. §3.4 "Power mode" — ST recommends ULP specifically for our transport, and our presets don't use it
> "ST recommends using low power and ultralow power modes with low frame rate or when the
> interframe blanking is long enough. **For example, ST recommends using ultralow power mode when
> the I3C interface is used to readout the data.** This configuration allows a longer readout
> duration, which reduces the power consumption."

I3C is exactly our transport (`host/src/roomscan/profiles.py` `I3C_XFER_MS`/`i3c_bus_utilization_pct`).
But all three current presets (`PRESETS` in `profiles.py`, set by the 2026-08-05 owner-directed
redesign `776622d`) use `PowerMode.REGULAR`. That commit's message gives the exposure/fps
rationale for each preset in detail but no rationale for power mode — it reads as "Regular was
the value before, and the redesign didn't touch it," not a considered rejection of ULP. This is
worth an explicit decision, not a silent carry-over: this is a battery-powered handheld
(`docs/wifi-bridge-filehub.md`), power draw is a real cost, and AN6522 is naming our exact
transport as the case ULP is *for*.

Not a slam dunk either way — two things this review did NOT check, so flag rather than fix:
- The interframe-blanking window at STABILITY/PRECISION (30 fps, 12/15 ms exposure — leaving
  ~15–18 ms of blanking per frame) is comfortably "long enough" for ULP's benefit by AN6522's own
  framing. HIGH_FRAMERATE (46 fps, 4 ms exposure, ~17.7 ms period) is tighter but still has
  meaningful blanking.
- The already-validated power model (`estimate_power_mw`, ProfileTuning.exe-derived, 5/5 within
  0.01%) can answer "how much would ULP actually save at each preset's fps/exposure" cheaply,
  before touching hardware — run `host/tools/profile_tuning.py` (or the `profile_estimate` /
  `profile_tuning` MCP tools) with `--power-config "Ultralow Power"` against each preset's
  fps/exposure and compare to the current Regular estimate. If the saving is small at these duty
  cycles, that alone would explain (and justify) staying on Regular for tracking-loop consistency;
  if it's large, it's a preset change worth putting to the owner.

### A2. Table 2 "Frame exposure recommendations" — STABILITY and PRECISION exceed ST's own indoor exposure ceiling
Table 2 gives ambient-light-conditioned exposure bounds. For indoor (`< 1 W/m²`) Precision mode:
Min 2 ms, Typ 4 ms, **Max 10 ms**. Our two default-use presets are `STABILITY` (12 ms) and
`PRECISION` (15 ms) — both above that indoor max, `PRECISION` by 50%. §1.2.1: "Overexposing may
result in underestimating the target reflectance" — a different failure mode than the plane-RMS
distance-accuracy metric the 2026-08-05 redesign measured ("Depth accuracy at a ~1.5 m wall is
exposure-independent (~2.2 mm plane-RMS 10-16 ms)"). That on-rig result rules out the specific
concern of *distance* accuracy degrading, but doesn't speak to *reflectance* estimation — a
different consumer (the flat-field/FPN reflectance work, `docs/flatfield-calibration.md`;
`docs/reflectance-fpn-flatfield.md`) that this review did not check against.

Not proposing a preset change unilaterally — that's the owner-directed 2026-08-05 design, grounded
in a real on-rig sweep, and it may already have implicitly accounted for this. Flagging because the
two numbers (our defaults vs. ST's own indoor table) disagree and nothing in the repo currently
says why that's fine.

### A3. §1.2.1 — recommended exposure floor (2 ms) is below our UI-allowed floor (1 ms)
"ST recommends also keeping the exposure above 2 ms." `EXPOSURE_MS_MIN = 1` in `profiles.py`. Low
stakes — no preset uses less than 4 ms, this only matters for a hand-typed Manual-mode value below
2 ms — but cheap to tighten (`EXPOSURE_MS_MIN = 2`) or at minimum add a validation warning, matching
the existing pattern (`validate_manual_params` already warns rather than rejects for the
measured-ceiling case).

## (B) Already correctly applied — confirmed, no action

- **I3C effective throughput (10 Mbps, not 12.5 MHz raw)** — Table 5's "54x42 → I3C (10 Mbps):
  11.8 ms" is exactly `I3C_XFER_MS` in `profiles.py`. Mined 2026-08-03, still correct.
- **>100 fps requires MIPI** (§2.2: "streaming above 100 FPS in full resolution requires the
  exposure to be limited to 4.5 ms, using a MIPI interface") — already the explanation on record
  in `docs/roadmap-history.md` for why DS14879's 100 fps "Gaming" anchor never reproduced on our
  I3C-only hardware.
- **Exposure step ratios (Table 3, `NB_SHOT_STEP_n = k_n * x`)** — already the basis for
  `capture_meta`'s exposure-from-metadata inversion (`docs/mcp-server.md`: "exposure (inverted
  from `nb_shot_step` via AN6522 ratios)").
- **`CMD_ACK_FRAME_READ` ordering** (§2.1: "Do not send the acknowledge command before starting to
  read the data... could be overwritten"): checked `vl53l9_get_frame()`
  (`firmware/vendor/53L9A1/Drivers/BSP/Components/vl53l9/vl53l9.c:623`) directly — it reads depth/
  amplitude/ambient, the DSS LUT, and the status line, and only issues `COMMAND_ACK_FRAME_READ`
  last. Vendor reference firmware already complies; nothing to fix (and nothing to fork — this
  path is read-only reference, `firmware/vendor/53L9A1/CLAUDE.md`).
- **Dithering** (§1.2, footnote 1: "It is recommended to keep it") — the vendor driver writes a
  fixed non-zero `VL53L9_REGADDR_STREAM_MAX_DITHERING_STEP` table (`dithering_short`/
  `dithering_long`, `vl53l9.c:134-139`) unconditionally; there's no code path that disables it.
  Already compliant, nothing to change.
- **Table 1 min/max distance and max-range figures** (Precision 50 mm/8.8 m, Ambient 450 mm/8.5 m)
  — already `MIN_DISTANCE_MM` / `MAX_RANGE_M` in `profiles.py`, sourced from DS14879 and
  cross-checked here.

## Not investigated (out of scope for this pass)

Whether A1/A2 actually change any preset is an owner call, gated on the profile_tuning power-model
check A1 names and a flatfield/reflectance check A2 names — neither run here. This review's job was
"does AN6522 say anything we're not using," not "retune the presets."
