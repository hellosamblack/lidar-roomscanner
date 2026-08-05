# 2026-08-04 stationary ceiling analysis

## Bottom line

At the stated 55 in (1397.0 mm) range, all modes measure the ceiling accurately in Z: the median bias is about **−3.3 to −7.2 mm (−0.24% to −0.51%)**. The best repeatability is in the slower Precision modes, with settled per-zone temporal σ around **0.35–0.43 mm**; 4 ms Precision and HFR are roughly **0.75 mm**.

<!-- Adjusted 2026-08-04 (review): the original "The correction is therefore real"
     overstated the holdout. On a stationary target the half-split proves temporal
     stability, not scene-independence, and the before/after metric shares its
     low-pass with the map builder (partly definitional). Reframed to weight the
     mode-family finding, which the data DOES support strongly, above the reduction
     figure. See the Method-and-limits and Reflectance sections for the full argument. -->
The reflectance plane is the more important finding. Precision-like modes carry a stable **6.6–6.7% sensor-locked spatial pattern**; Ambient modes carry a different, weaker **3.7% pattern**. A flat-field gain map trained on the first half of each stationary capture reduces the held-out residual to **1.4–1.5% for Precision** and **1.7–1.8% for Ambient**. Read that reduction narrowly: on a stationary target it shows the pattern is **temporally stable**, not that the map is **scene-independent**, and the before/after metric shares its low-pass operator with the map builder, so part of the reduction is definitional (see Method and limits). What the data *does* support strongly is that the correction is **mode-family-specific**: applying an Ambient map to Precision (or vice versa) leaves roughly **6.0–6.4%** residual.

The deprojection math is correct about the important convention: the host `z` is perpendicular depth, and it is exactly equal to the native ZAPC `z` output in every run. The remaining host-vs-ZAPC X/Y difference is the already-known linear-FoV approximation to real edge distortion: about **31 mm p95 overall, 44 mm p95 at the field edge, and ~76 mm worst-case** at this 1.397 m range. That is an X/Y model limitation, not a ranging-mode problem.

## Method and limits

- Used the roomscanner MCP capture forensics/profile tools for file integrity, continuity, rates, pairing, motion, and timestamp skew.
- Decoded every RAW_3DMD frame through the same native transform used by the viewer, producing depth, reflectance, confidence, ambient, and ZAPC.
- Used the 54×42 host `Deprojector` and fitted a plane to the temporal-median depth image so a small tripod tilt does not get mistaken for sensor nonuniformity.
- Dropped the first 30 transformed frames of each run for settling. Precision below means temporal variation while the sensor remained stationary; it is not an independent-shot-noise measurement because the transform includes the sensor/library TNR path.
<!-- Adjusted 2026-08-04 (review): spell out exactly what the half-split holdout can
     and cannot prove, and note the shared-low-pass circularity between the metric
     and the builder. The prior wording ("a strong engineering test") let a reader
     take the 6.6->1.5% reduction as generalization proof; it is not. -->
- Flat-field maps were trained on one half and scored on the other half. **What this holdout does and does not prove:** because the target never moved, both halves are nearly the same image, so the split rules out overfitting to temporal/shot noise and confirms the pattern is temporally stable — but it **cannot** separate true sensor-locked FPN from the ceiling’s own static texture/illumination, which is present identically in both halves and gets baked into the map either way. It is also not a fully independent metric: `reflectance_residual` and `build_flatfield` share the same 2.5σ Gaussian low-pass, so the number being scored is deviation-from-blur and the correction is defined to flatten exactly that — part of the 6.6%→1.5% reduction is therefore definitional rather than measured generalization. The load-bearing validation is still outstanding and needs two things a stationary ceiling cannot give: a **slow pan** over a matte wall (smears scene texture, leaving only zone-locked FPN) and a **held-out distance** (separates FPN from the broad illumination field). Treat these maps as candidates whose correction is *temporally stable*, not yet *scene-independent*.
- The tape-measured 55 in distance is treated as approximate. A few millimetres of range bias should not be over-interpreted as a calibration constant without a better reference target.

## Capture integrity and rates

Every file was byte-clean: **0 CRC failures, 0 skipped bytes, and no decoder anomalies**. This does not mean every emitted frame arrived; the MCP continuity check found:

| capture | intended / measured rate | RAW continuity | note |
|---|---:|---:|---|
| `4msExpPrecision` | 30 / 29.93 Hz | 1 missing (0.076%) | good for quality analysis |
| `8msExpPrecision` | 30 / 29.94 Hz | complete | clean baseline |
| `8msExpPrecision60hzEnv` | 30 / 29.94 Hz | complete | ToF RAW complete; extra IMU/ENV cadence is expected |
| `Reg16msExpAmbient` | 30 / 29.86 Hz | complete | clean |
| `Reg16msExpPrecision` | 25 / 24.98 Hz | 1 missing (0.098%) | good for quality analysis |
| `ULP16msExpAmbient` | 15 / 14.98 Hz | complete | clean |
| `ULP16msExpPrecision` | 15 / 14.98 Hz | complete | clean |
| `hfr` | device 45.54 / saved 42.60 Hz | 124 missing (6.47%), 75 whole-group losses | **not a valid transport baseline** for HFR comparison |
| `precision` | 30 / 29.90 Hz | 1 missing (0.082%) | good |

The high-frame-rate run is still useful as a stationary sensor sample, but its saved capture under-represents the device stream. Fix or characterize the Ethernet/recording loss before making a production HFR precision claim.

The MCP motion analysis confirms a stationary tripod hold for the RAW-rate captures: tilt is about 0.4–0.5° with 0° range and no detected motion. The `8msExpPrecision60hzEnv` motion report sees the higher-rate IMU samples as multiple “takes”; its ToF RAW stream and depth maps remain stationary.

## Range accuracy, uniformity, and precision

| mode / capture | median Z error | plane-fit RMS | raw spatial p95−p05 | per-zone temporal σ median / p95 |
|---|---:|---:|---:|---:|
| 4 ms Precision | −4.11 mm (−0.29%) | 2.02 mm | 9.92 mm | 0.766 / 0.861 mm |
| 8 ms Precision | −4.24 mm (−0.30%) | 2.03 mm | 9.92 mm | 0.527 / 0.602 mm |
| 8 ms Precision + 60 Hz env | −4.03 mm (−0.29%) | 2.03 mm | 9.93 mm | 0.512 / 0.587 mm |
| Regular 16 ms Ambient | −7.19 mm (−0.51%) | 2.55 mm | 11.50 mm | 0.432 / 0.505 mm |
| Regular 16 ms Precision | −4.11 mm (−0.29%) | 2.07 mm | 10.33 mm | 0.365 / 0.430 mm |
| ULP 16 ms Ambient | −6.74 mm (−0.48%) | 2.67 mm | 11.79 mm | 0.403 / 0.547 mm |
| ULP 16 ms Precision | −3.32 mm (−0.24%) | 1.94 mm | 9.21 mm | 0.352 / 0.429 mm |
| HFR | −4.20 mm (−0.30%) | 1.97 mm | 9.77 mm | 0.753 / 0.831 mm |
| Precision preset | −3.35 mm (−0.24%) | 2.17 mm | 11.42 mm | 0.451 / 0.528 mm |

What this says:

- **Accuracy is broadly mode-independent and good at this range.** Ambient is consistently about 3 mm farther low than Precision, a small systematic profile difference rather than random noise.
- **Exposure and frame rate dominate precision.** 4 ms is noticeably noisier than 8 ms; HFR is also noisier than the 15/25/30 Hz Precision runs. The slower 16 ms Precision configurations are best.
- **The ceiling is spatially uniform in depth to a few millimetres.** After fitting the small plane tilt, RMS residual is 1.94–2.67 mm and p95 absolute residual is about 3.8–5.3 mm. The raw p95−p05 spread of 9–12 mm includes the tripod’s slight plane tilt and edge/systematic structure.
- The depth residual map has broad tilt/curvature, not the high-frequency block pattern seen in reflectance. Do not use the reflectance flat-field map as a depth correction.

## Reflectance, pixel blocking, and emitter pattern

The saved map image makes the distinction visible: the Precision reflectance image has a block/grid-like internal pattern plus bright edge/corner structure; its gain map contains the matching high-frequency correction. Ambient has a different, mostly broad radial edge/center pattern and a much smaller high-frequency residual.

| family | raw FPN residual | gain range / gain σ | held-out residual after map | reduction |
|---|---:|---:|---:|---:|
| Precision-like runs | 6.59–6.73% | about 0.79–1.21 / 6.5–6.6% | 1.44–1.49% | ~78% |
| Ambient runs | 3.69–3.73% | about 0.85–1.09 / 3.5% | 1.74–1.77% | ~53% |

<!-- Adjusted 2026-08-04 (review): downgraded "the key control" -> "a temporal-stability
     control". On a stationary target the two halves carry the same FPN AND the same
     scene texture/illumination, and the metric shares the builder's low-pass, so this
     cannot certify scene-independence. The panned second-distance capture is the real control. -->
The half-capture holdout is a **temporal-stability** control, not a scene-independence one: it shows the pattern is stable enough that a map built on the first half still flattens the second. But on a stationary target both halves carry the same FPN *and* the same ceiling texture/illumination, and the before/after metric shares the builder’s 2.5σ low-pass — so it cannot certify that the map isn’t partly baking in static scene structure. Establishing that requires the panned matte-wall capture at a second distance (see Method and limits and Recommended next actions).

Map portability is limited but useful:

- Precision maps are highly correlated with one another (roughly 0.97–1.00). The two 8 ms Precision captures are nearly identical (correlation 0.9995), proving that the extra 60 Hz environment polling does not change the ToF flat field.
- The two Ambient maps are also nearly identical (correlation 0.9955).
- Precision↔Ambient map correlation is only about 0.34, and cross-application leaves about 6.0–6.4% residual. This is a real response/profile change, not just a different overall reflectance level.
- The correction currently shipped in `roomscan.flatfield` is a single configured path. Runtime mode switching therefore needs either a mode-keyed map registry or an explicit operator-selected map. A single Precision map must not silently be applied to Ambient.

The builder’s Gaussian smoothing intentionally preserves broad illumination. That is why the Ambient map still shows a smooth edge/center character: it is useful for removing per-zone pixel blocking, but it is not a full correction for emitter/receiver vignetting. If the goal is to remove that broad IR-emitter falloff too, it needs a separate low-frequency illumination normalization and a validation target at more than one distance; do not fold it into the same gain map without deciding whether absolute reflectance should be preserved.

The raw full-image variation is much larger than the FPN number because it includes that broad field: the Precision mean image has about **13.1% CV**, while Ambient is about **24.6% CV**. The 6.6% / 3.7% figures above are the high-frequency residual after the builder’s low-pass illumination estimate. They are therefore not directly comparable to the earlier ~18% headline, which used a different normalization/target; the defensible result here is the held-out before/after reduction, not a claim that the sensor’s physical FPN changed from 18% to 6.6%.

## Deprojection math

The native transform was run with both ZF32 depth and ZAPC output:

- ZAPC `z` minus host depth is **exactly 0.0 mm maximum** in all nine captures. The host and native path agree that the range channel is perpendicular depth, not radial range.
- Host linear-FoV X/Y versus factory-calibrated ZAPC is stable across modes: combined X/Y error is about **31.4 mm p95** at 55 in; the center-region p95 is about **19.4 mm**; the edge-region p95 is **43.8–44.0 mm**, with worst edge samples around **76 mm**.
- Because those errors are almost unchanged between 4 ms, Ambient, ULP, and HFR, they are optical/model systematics, not ranging precision. They match the project’s prior ZAPC validation: the 55°×42° global FoV is right, while real lens distortion dominates at the corners.

This ceiling test validates the Z/depth convention and exposes repeatability, but it cannot independently prove the absolute X/Y angle scale because there are no known lateral landmarks in the ceiling. Keep the current 55°×42° defaults; use the existing optional per-zone ZAPC-derived tan tables only when corner geometry needs better than the linear model.

## Timing and other observations

The MCP skew tool found stream-13 edge timestamps in every capture. Sync RMS is about **14–19.5 µs**, with p95 **27–38 µs**. The older FIFO-derived skew is around 0.9–1.3 ms and has a documented ~601 µs floor; that is a measurement-bound/inference, not the actual frame-edge synchronization quality. At rest, timing is therefore not the limiting error here.

The 60 Hz environment run has approximately two IMU/RAW samples per ToF frame. The profile tool’s same-sequence pairing percentage for IMU_SYNC falls to 93.5% under its older coupled-1:1 assumption; this is a bookkeeping limitation of that probe after decoupled polling, not a missing ToF depth stream.

## Recommended next actions

1. Keep the generated maps as **engineering candidates**, not yet as a universal production map. For immediate viewer testing, use the 8 ms Precision map for Precision-like modes and the Regular/ULP Ambient maps for their corresponding Ambient modes.
2. Make flat-field selection mode-aware before enabling correction globally. At minimum distinguish Precision-like from Ambient; ideally key by ranging mode, power mode, and exposure family.
3. Collect one official slow-pan matte-wall flat-field capture and one held-out flat-wall capture at another distance. That separates sensor-locked blocking from the ceiling’s broad emitter/illumination field.
4. Re-record HFR after transport loss is resolved. The current HFR data supports a provisional sensor-quality observation, not a clean rate/transport result.
5. Do not change the linear FoV constants based on these captures. If corner X/Y accuracy matters, seed/use the existing per-zone tan-table path or add a depth-dependent distortion model; reflectance flat-fielding cannot fix it.

## Generated artifacts

- `analysis_20260804_metrics.json` — all numeric results, map portability matrix, and provenance.
- `analysis_20260804_metrics.csv` — compact per-capture comparison.
- `analysis_20260804_flatfield_maps.png` — visual summary of Precision/Ambient reflectance, gains, depth residual, and temporal σ.
- `flatfield_20260804_*.npz` — one per-capture gain map; these are not enabled automatically.
- `maps_20260804_*.npz` — depth/reflectance summary maps for further inspection.
- `analyze_20260804_flat_field.py` — reproducible analysis script.
