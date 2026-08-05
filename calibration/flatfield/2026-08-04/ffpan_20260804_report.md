# 2026-08-04 panned flat-field study (DC-D2 candidate set)

Six captures recorded while **slowly panning** the sensor across the ceiling at
**three operator heights** each — the first flat-field data that can actually test
what the stationary ceiling study (`analysis_20260804_report.md`) could only assume.
Analysis: `analyze_20260804_ffpan.py` → `ffpan_20260804_metrics.json` + per-capture
`ffpan_20260804_*.png`.

## Bottom line

- **5 of 6 are clean, valid panned flat-field candidates.** 0 CRC, ≤0.3% loss, no
  whole-group drops; genuine slow pans (10–13°/s median, no whips); three distinct
  standoffs per capture (~460 / ~680 / ~1130 mm, ~650–720 mm spread). Gains all
  inside **[0.78, 1.20]** (0% outside [0.5, 1.6], 0% at the clip), self-map residual
  ~1.3%, within-family cross-residual ~1.6–2.2%.
- **1 is contaminated — discard and recapture: `ambientRegular8or16msFFpan.bin`.**
  Raw FPN **11.4%** (vs ~6.5% for the others), gain to **1.63**, its map dominated
  by a broad bright-edge/corner **illumination field** + a vertical seam rather than
  the zone grid, cross-application **8.6–11.2%** in both directions, and gain-map
  correlation only **0.40–0.63** with the clean set (vs 0.78–0.99 among the rest).
  This is the "keep an eye on the smoke detector / shelves" risk landing — most
  likely a bright wall corner / soffit held in the FOV and/or an imbalanced height
  distribution (727 / 442 / 299 frames, heavily weighted to the closest height).
- **Mode-family-specificity is confirmed but milder than the static study claimed.**
  Within-family cross-residual ~2%, cross-family (Precision map on Ambient or vice
  versa) ~3.7–4.6% — roughly a 2× penalty, real and worth keying by family, but not
  the 0.34-correlation / ~6% gulf the stationary study reported (that gap was
  inflated by each mode's baked-in static scene).
- **The honest cross-scene floor is ~2%, not the ~1.3–1.5% self-map figure.** The
  self-map/temporal number is near-tautological (metric shares the builder's 2.5σ
  low-pass); the residual that survives a *different* pan of the same family is ~2%.

## Exposure of the two `8or16` captures — unresolved with confidence, and immaterial

Frame rate does not discriminate: all six recorded at a fixed 33.4 ms / 29.9 fps,
consistent with either 8 or 16 ms (16 ms fits inside a 33 ms frame). The physical
discriminators (ambient counts and noise scale with integration time) are ambiguous:

| capture | ambient vs 8 ms sibling | zone σ vs sibling | call |
|---|---:|---:|---|
| `ambientULP8or16` | 1.30× | 1.18× (no drop) | **best guess 8 ms**, low confidence |
| `ambientRegular8or16` | 1.01× | 1.03× | can't trust — capture is contaminated |

A true 16 ms would roughly double ambient and cut noise ~0.71×; neither shows that.
Best guess is **8 ms for both**, but low confidence. **It does not matter for the
calibration:** both are Ambient-family, and the clean `ambientULP8or16` map groups
with the Ambient set (correlation ~0.97, cross-apply ~1.6–2.0%) regardless of its
exposure. The contaminated Regular one needs recapture anyway; confirm its exposure
from operator notes when re-shooting.

## The held-out-distance result (the new, load-bearing test)

Building a map at one height and scoring it at another **fails**: the near-height
map applied to far-height frames leaves 7–8% — *worse than doing nothing* (raw ~6%)
— while a same-distance temporal split "works" (~2–3%). But the cross-**capture**,
same-family transfer (different pan, different scene, overlapping distances) is good
(~2%). Reconciling these: a **single-height** map bakes in that height's ceiling
texture (insufficient angular diversity at one standoff), so dividing another height
by it actively adds inverted texture; a **full multi-height pan** averages the scene
out and transfers. Two consequences:

1. **A production flat-field map must be built from a full multi-height pan** (as
   these are) — never a single standoff. The three-height protocol is necessary, not
   optional.
2. **Validation must be a separate full-pan capture, ideally at a different center
   distance** — an intra-capture height split is confounded by within-band scene.
   This is the real scene-independence control; it is still not fully closed (all six
   share one ceiling), so a second surface / room is the remaining gap.

## Contamination sweep (smoke detector / shelves)

No discrete object blob in the 5 clean captures: depth-residual maps are flat
(scattered ±10–20 mm noise, a few hot pixels, sensor-locked row/col seams), and the
reflectance means show the expected illumination field + zone grid with no localized
intrusion. The per-frame "near-object zone" counts (33–47% of frames with ≥5 zones
reading >100 mm closer than the plane) are an **edge/plane-extrapolation artifact**
— they sit at the frame border in every capture, clean or not, so they are not a
discriminator on their own. The signal that isolated the bad capture was the
combination of high raw FPN + wide gain + low-frequency-dominated gain map + poor
cross-correlation, all of which point at `ambientRegular8or16` alone.

## Recommended next actions

1. **Recapture `ambientRegular8or16`** — same slow multi-height pan, keep bright
   wall corners / soffits / the smoke detector out of the FOV, balance time across
   the three heights, and note the exposure.
2. **Adopt the 5 clean maps as the family candidates** — any of the Precision maps
   for Precision-like modes, any clean Ambient map for Ambient modes.
3. **Implement mode-aware flat-field selection** (Precision vs Ambient at minimum)
   before enabling correction globally — a single `flatfield_path` mis-corrects
   cross-family by ~4%.
4. **Close scene-independence properly**: one full multi-height pan over a *second*
   surface (different room / different center distance) as held-out validation, then
   quote the cross-surface residual (~2% expected) as the shipped floor — not the
   self-map ~1.3%.
