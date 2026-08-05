# Cross-room (held-out-scene) validation — 2026-08-04

Closes the last open question from `ffpan_20260804_report.md`: the six first-room
pans all shared one ceiling, so scene-independence was unproven. Two pans in a
**larger, different room** (`precisionRegular8msFFpanLarge.bin`,
`ambientRegular8msFFpanLarge.bin`) provide the held-out scene.
Analysis: `analyze_20260804_crossroom.py` → `crossroom_20260804_metrics.json` +
`crossroom_20260804_{precision,ambient}.png`. Before/after visualization:
`viz_20260804_before_after.py` → `beforeafter_20260804.png` (held-out FPN maps,
column-ripple profiles, and per-zone residual histograms, per family).

## Wall rejection

The operator flagged that a bit of **wall** drifts into the FOV at the start/end
turnarounds of the Large-room pans (they start and end with holds). A wall in view
puts two planes in one frame, so `analyze_20260804_crossroom.py` rejects any frame
whose per-frame depth RMS about its own best-fit plane exceeds 25 mm, applied
symmetrically to both rooms. Result:

- Room A pans: **0 rejected** (clean, no wall).
- Room B precision: **370 / 1452 rejected (25%)**, 79% of them in the first/last 10%.
- Room B ambient: **340 / 1959 rejected (17%)**, 57% in the ends.
- The precision capture's stray 175 mm plane-residual spike is gone (max now 25 mm).

This confirms the wall was at the turnarounds and removes it. **All numbers below
are wall-rejected.**

## Result — scene-independence PROVEN

Build the map in room A, apply it to room B (unseen), and reverse:

| family | room B raw | self-map | **room-A map (held out)** | reverse B→A | A/B gain corr |
|---|---:|---:|---:|---:|---:|
| Ambient | 5.8% | 1.3% | **2.4%** | 2.5% | **0.945** |
| Precision | 6.2% | 1.3% | **3.0%** | 3.0% | **0.920** |

A map from a room it never saw still flattens the data, symmetrically, with
near-identical gain maps. This is genuine sensor FPN, not a memorised room.

## Honest floor and caveats

- **The shippable cross-scene floor is ~2.4% (Ambient) / ~3.0% (Precision)** —
  the correction removes ~60% / ~50% of the FPN, not the ~78% the self-scored
  stationary study implied. Progression: self-map 1.3% → same-room different-pan
  ~2% → **different-room ~2.4–3.0%** → raw ~6%. Quote the cross-room number.
- **Precision transfers slightly worse than Ambient** (corr 0.92 vs 0.945; 3.0 vs
  2.4%), but the earlier, larger gap (0.86 / 3.6%) was mostly **wall contamination**
  at the turnarounds — after rejection the two rooms' precision gain maps are nearly
  identical, with only a faint residual column term.
- **Mode split reproduces across rooms** (3rd confirmation): matched 2.4/3.0% vs
  cross-family 3.7/4.2%. Precision and Ambient need separate maps.

## Capture quality

Both room-B captures are the cleanest in the set: 0 CRC, **0 RAW frames lost**,
fully-filled frames (no no-return zones), gains inside [0.80, 1.17] / [0.81, 1.17],
low-frequency gain content <0.75% (illumination preserved). The only defect was the
wall at the turnarounds, now rejected. Minor, non-harmful: the precision take is
stop-and-go (33% moving, 14 s of end holds — which is where most wall frames were)
and the ambient take has one 102°/s whip.

## Status / next

- DC-D2 scene-independence gate: **SATISFIED for both families** — Ambient ~2.4%
  (corr 0.945), Precision ~3.0% (corr 0.920), both symmetric across rooms.
- Remaining to enable correction: **mode-aware flat-field selection** (Precision vs
  Ambient), then ship the room-A (or a pooled) map per family. Power/exposure need
  not be keyed (Regular↔ULP corr 0.96–0.98; exposures 0.97–1.00).
- Both candidates are production-ready pending the selection wiring. Precision's
  faint residual column term (3.0 vs 2.4%) is a minor refinement, not a blocker.
- Recording tip for future validation pans: **keep the wall out of the FOV at the
  turnarounds** (or accept that ~20% of an end-hold-heavy capture will be rejected).
