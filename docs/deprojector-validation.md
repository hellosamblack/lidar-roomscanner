# Deprojector validation against ZAPC (Phase 2.5 Task 3)

Validates the host `Deprojector`'s linear-FoV model against ZAPC — the
transform library's own factory-calibrated point cloud. ZAPC runs a true
pinhole deprojection with per-pixel distortion and depth-dependent parallax
correction against the sensor's real optics (`radial_to_perp.c`,
`53L9A1/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/
radial_to_perp.c`, read-only reference) — it is ground truth here, the
`Deprojector` is a datasheet-derived linear approximation.

Reproduce: `host/.venv/Scripts/python host/tests/validate_deprojector_zapc.py`
(3-frame committed golden fixture, ~7 s) or add `--capture
captures/golden_pairs.bin` for the full 731-frame hardware capture (~1.65M
zones, gitignored, ~1 min) — see `host/tests/validate_deprojector_zapc.py`
for the full script.

## Zone survival

Golden fixture (3 frames, 6804 total zones): **6747 valid (99.2%)** after
excluding the `DEPTH_NO_RETURN_MM` (12000.0 mm) no-return sentinel with a
1000 mm margin (28/16/13 sentinel zones per frame — a real room scan,
plenty of returns, some no-return as expected per the task brief).

Full capture (731 frames, 1 657 908 total zones): **1 649 761 valid (99.5%)**.
Every summary statistic below (angular error, best-fit FoV, worst-case
displacement) is essentially identical between the 3-frame fixture and the
731-frame capture (see table below) — the sensor's intrinsics really are
static per the task's premise, and the 3-frame fixture alone was already
sufficient. Ran both anyway rather than assume it, per "numeric honesty."

**Confidence-channel finding:** ZAPC's 4th channel (confidence, nominally
0..1) reports **~1.0 on every zone in this fixture, including the no-return
sentinel zones**. Measured (printed by the script's "ZAPC confidence
channel" section): over all 6804 fixture zones min=1.000000 max=1.000007
mean=1.000000; the [0,1] histogram is single-bin — all 6747 valid zones sit
at exactly 1.000000 — and the 57 sentinel zones read *marginally above* 1.0
(1.000001–1.000007). The full 731-frame capture shows the identical pattern
over 1.66M zones (sentinel min/max 1.000001/1.000007, mean 1.000004). As a
threshold gate it does not discriminate valid from invalid — exclusion had
to rely on the depth value itself (sentinel + finite + positive), not the
confidence gate the task brief suggested as a secondary filter. One nuance
the stats surfaced: the >1.0 values are the library's per-zone filter-status
codes packed into the 1e-6 digits (`radial_to_perp.c:198-203`,
`floor(conf·1e3)·1e-3 + status·1e-6`), and on this data *every* sentinel
zone carries a nonzero status code while every valid zone reads exactly
1.000000 — so the status micro-digits do mark no-return zones, but in the
**wrong direction** for a `conf >= min` gate (sentinels are *higher*, not
lower). Documented as a real finding, not treated as "confidence works, just
didn't need it": on this scene/calib, the 0..1 confidence value itself gave
zero information for zone exclusion. `--conf-min` is still exposed as a
script flag in case a different capture's confidence distribution is more
informative.

**Root cause (library bug — structurally constant, not scene-dependent):**
the uniform ~1.0 is an **uninitialized divisor** in the read-only transform
library. `radial_to_perp.h:61` declares `float_t conf_scaling;` in the algo's
params struct, and `radial_to_perp.c:199` computes the point-cloud confidence
as `minf(confidence[i] / (params->conf_scaling * conf_thr[i]), 1.0f)` — but
`conf_scaling` is **never assigned anywhere in the `53L9A1/` tree** (those
two lines are its only occurrences). The zero-initialized divisor makes the
quotient +inf for every zone with nonzero raw confidence, and the `minf`
clamp collapses +inf to exactly 1.0; the subsequent
`floor(conf·1e3)·1e-3 + status·1e-6` packing (`radial_to_perp.c:202-203`)
then adds only the filter-status micro-digits. So the channel is
**structurally** 1.000000 + status·1e-6 on every zone — no capture, scene,
or calibration can produce a discriminating value. This is a vendor bug in
`vl53l9-transform-c`, not a property of this fixture.

## ZAPC axis/unit conventions (previously unknown — now established)

| aspect | finding |
|---|---|
| x axis | increases monotonically with **column** index (col 0 median tan_x=-0.4996, col 53 median tan_x=+0.5210) |
| y axis | increases monotonically with **row** index (row 0 median tan_y=-0.3750, row 41 median tan_y=+0.3798) |
| radial vs perpendicular z | **perpendicular** — ZAPC's z is **bit-identical** to the ZF32 depth output on every valid zone (max abs diff = 0.000000 mm across both the 3-frame and 731-frame runs; the script **hard-asserts `max_abs_diff == 0.0`** — exact identity, not a tolerance — and exits nonzero if a library/shim/fixture regression ever breaks it). A radial z would grow relative to ZF32 off-axis; there is no such trend, the diff is exactly zero. Traced in the read-only library source: `vl53l9_algo_pointcloud()` (`radial_to_perp.c:195-197`) writes `pointcloud[...+2] = depth[linear_id]` — the *same* r2p-corrected perpendicular depth array that feeds the ZF32 output, not a re-derived radial range. |
| units | millimeters for x, y, and z (matches ZF32's mm depth exactly, and x/y magnitudes — up to ~6300 at the widest corner over ~400–12000 mm depths — are consistent with mm-scale geometry, not meters) |

**Conclusion:** `x = z * tan(angle_x(col))`, `y = z * tan(angle_y(row))`,
`z` perpendicular in mm — exactly the convention `Deprojector` already
assumes. No sign flips, no axis swap, no unit conversion needed to compare
directly against the linear model's `tan_x`/`tan_y` tables.

## Per-zone angular error vs. the linear model (datasheet 55°×42°)

3-frame fixture / 731-frame capture (essentially identical):

| metric | 3-frame fixture | 731-frame capture |
|---|---|---|
| angular error X: median / p99 / worst (deg) | 0.344 / 1.469 / 2.192 | 0.345 / 1.473 / 2.200 |
| angular error Y: median / p99 / worst (deg) | 0.232 / 1.620 / 2.132 | 0.232 / 1.621 / 2.132 |
| displacement @ 2 m: median / p99 / worst (mm) | 19.52 / 81.9 / 126.9 | 19.51 / 82.1 / 127.3 |
| displacement @ 2 m: median / p99 / worst (% of z) | 0.976 / 4.10 / 6.35 | 0.976 / 4.10 / 6.36 |
| worst zone | row 0, col 53 (top-right corner) | row 0, col 53 (same) |
| center region (|Δcol|<10, |Δrow|<8) displacement median / max | 12.2 / 27.6 mm | 12.2 / 27.6 mm |
| edge region displacement median / max | 20.8 / 126.9 mm | 20.8 / 127.3 mm |

The center-vs-edge split is the key diagnostic: median error is small and
roughly uniform near boresight (~12 mm/2 m ≈ 0.6%), but the worst case is
**always at the extreme corner** and **>10x** the center-region max. This is
the signature of genuine lens distortion concentrated at the field edges,
not a globally-wrong FoV constant (which would inflate the *center* error
too, proportionally).

## Best-fit equivalent FoV (least squares over zone centers)

Fit `angle(k) = fov * k` (no intercept — `k` is the known zone-center
weight) via `fov = Σ(k·angle) / Σ(k²)` over all valid zones:

| axis | Deprojector's zone-center convention | zone-center-to-zone-center convention | RMS residual |
|---|---|---|---|
| H | fov = **54.65°** (54.6535; full capture 54.6592) | fov = 53.64° | 0.528° |
| V | fov = **42.50°** (42.4990; full capture 42.5036) | fov = 41.49° | 0.450° |

**Edge-vs-zone-center convention verdict (open question (a)): not a real
choice.** The two candidate conventions —
"55° spans the optical edge, zone centers sit half a zone-pitch inside it"
(current `Deprojector` code, `k_center(i) = (i+0.5)/n - 0.5`) vs.
"55° spans the outermost zone centers directly"
(`k_edge(i) = i/(n-1) - 0.5`) — are related by
`k_edge(i) = [n/(n-1)] * k_center(i)`, a **pure scalar rescaling** with the
same `i`-dependence. A no-intercept least-squares fit against a rescaled
regressor produces an identical residual and a rescaled fit coefficient; the
script asserts this algebraically (`rms_h_center == rms_h_edge` to floating
precision) and it holds on real data. So there is no distinguishing evidence
between the two "conventions" — they're the same underlying linear model
wearing two different fov labels. `Deprojector`'s code already commits to
the zone-center-inside-optical-edge convention (matches typical camera-FoV
semantics: FoV is the full optical field, pixel/zone centers sit inside it),
so its fitted number is the one that matters: **54.65°/42.50°**.

**Reconciliation with the datasheet (55°×42°, DS14879 rev 6):** agrees
closely — H within **0.35°**, V within **0.50°** of the datasheet numbers.
Both directions are well inside what a single global distortion-free fit
can average away; this is a confirmation of the datasheet values under the
existing convention, not a disagreement.

## Decision: does a pure FoV-default change fix it?

**No.** Re-running the displacement calculation with the best-fit FoV
(54.65°/42.50° instead of the datasheet's 55°/42°) leaves the worst case
**unchanged** (6.36% → 6.37% of z at 2 m — actually marginally worse, noise
in the single-scalar fit) because the error is not a scale error, it's a
shape error: real per-zone lens distortion (per `radial_to_perp.c`'s
`compute_distortion()` — a quartic `alpha·r² + …` term, `alpha = -0.00015`
by default in this library — plus a depth-dependent parallax-correction
term) that a single global FoV constant cannot represent, linear or not.

Per the plan's decision gate ("if a pure FoV-defaults change gets worst-case
displacement ≤ 1% of z → update defaults"): **worst-case is 6.35–6.36%,
nowhere near the 1% bar, and does not improve with a fov tweak** — so the
gate is not met and **the datasheet-derived 55°/42° defaults are kept
unchanged** (they're independently confirmed correct to within ~0.5° by the
fit above, so there was nothing to fix there).

Per the plan's fallback ("if the linear model itself is the problem
(distortion), add an optional per-zone tan-table path... only if the
evidence demands it"): the evidence above — corner error 10x the center
error, unmoved by any global FoV choice — is exactly that case. Implemented:

```python
Deprojector(width, height, zone_tan_x=None, zone_tan_y=None, ...)
```

`zone_tan_x`/`zone_tan_y` are optional `(height, width)` per-zone
`tan(angle)` arrays (e.g. seeded from ZAPC's `x/z`, `y/z` per zone) that
bypass the separable linear model entirely when both are supplied; the
linear FoV model remains the default (unchanged behavior for every existing
call site — most callers don't have per-device ZAPC data on hand). No
special-casing was needed in `__call__`: numpy broadcasting already handles
`(1, w)`/`(h, 1)` (linear) and `(h, w)` (per-zone table) uniformly.

The validation script demonstrates the mechanism end-to-end: it builds a
per-zone table from this run's own ZAPC data (median tan per zone across the
available frames; 2256/2268 zones covered, the rest fall back to the linear
model) and self-checks it against the same data (median 0.006 mm, i.e.
essentially exact — not a held-out test, just proof the plumbing works).

**Caveat found while building the demo table:** the self-consistency worst
case is **not** exactly zero (~29 mm) despite averaging the *same* zone
across frames. Traced to `radial_to_perp.c`'s parallax correction
(`vl53l9_algo_radial_to_perp`, `new_x_center` term) being **depth-dependent**
(`∝ 1/depth_perp`, clamped below `parallax_limit=50`) — so a given zone's
true angle shifts slightly with the depth actually measured there, most
noticeably at close range and large off-axis distance (the worst
self-consistency zone). A per-zone table seeded from one scene therefore
carries a small residual floor from scene-to-scene depth variation; it is
still a large improvement over the pure linear model's 127 mm/6.36% worst
case, but is not perfect physics. Not chased further here (would require
also modeling the parallax depth-dependence in the table, which is out of
scope — YAGNI unless a future consumer needs sub-cm corner accuracy).

## Test/verification status

- `host/.venv/Scripts/python -m pytest tests -q` (from `host/`): **51
  passed** (48 baseline + 3 new `test_deproject.py` cases for the
  `zone_tan_x`/`zone_tan_y` constructor path: override behavior, shape
  validation, both-or-neither validation).
- `ruff check` clean on `deproject.py`, `test_deproject.py`,
  `validate_deprojector_zapc.py`.
- `host/tests/validate_deprojector_zapc.py` run against both the 3-frame
  fixture and the full 731-frame capture (`--capture
  captures/golden_pairs.bin`) — consistent results, see table above.

## Summary

| question | answer |
|---|---|
| ZAPC axes | x↔column, y↔row, both monotonic, matches `Deprojector`'s existing assumption |
| ZAPC units | millimeters (x, y, z all) |
| radial vs. perpendicular | perpendicular — bit-identical to ZF32 depth |
| edge-vs-center FoV convention | not distinguishable by fit (algebraically the same model, different label); `Deprojector`'s existing zone-center convention is the one that matters and it fits well |
| best-fit FoV vs. datasheet 55°×42° | 54.65°×42.50° — agrees within 0.35°/0.50° |
| worst-case displacement | 127 mm at 2 m (6.36% of z), always at the extreme corner |
| decision | **keep FoV defaults unchanged** (confirmed correct); **add optional `zone_tan_x`/`zone_tan_y` per-zone table** to `Deprojector` for callers needing corner accuracy — linear stays the default |
