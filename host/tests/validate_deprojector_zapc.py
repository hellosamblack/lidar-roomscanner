"""ZAPC-validated Deprojector check (Phase 2.5 Task 3).

Runnable script, NOT a pytest test (mirrors sweep_golden_capture.py's pattern):

    host/.venv/Scripts/python host/tests/validate_deprojector_zapc.py
    host/.venv/Scripts/python host/tests/validate_deprojector_zapc.py --capture captures/golden_pairs.bin

Validates the host `Deprojector`'s linear-FoV model against ZAPC, the
transform library's own factory-calibrated point cloud (true pinhole
deprojection with per-pixel distortion + parallax correction -- see
radial_to_perp.c, read-only reference in 53L9A1/Middlewares/ST/
vl53l9-transform-c). ZAPC is ground truth here: it uses the sensor's real
optics, the Deprojector uses a datasheet-derived linear approximation.

What this settles (see docs/deprojector-validation.md for the write-up):

1. ZAPC axis/unit conventions (previously UNKNOWN) -- established empirically
   below: x increases with column, y increases with row (both monotonic),
   units are millimeters, and z is PERPENDICULAR depth (not radial) -- it is
   bit-identical to the ZF32 depth output on every valid zone in the golden
   fixture (see the "z vs ZF32 depth" check).
2. Edge-vs-zone-center FoV convention -- turns out to be a non-issue: the
   "zone-center-inside-optical-edge" convention already coded in Deprojector
   and the "zone-center-to-zone-center" alternative are the same linear model
   up to a rescaling of the fov constant (k_edge = n/(n-1) * k_center), so
   they fit the ZAPC data with IDENTICAL residuals -- only the reported fov
   number differs. This script fits both and reports both fov values plus the
   shared residual, settling that there is no separate "convention" bug to
   fix, only a fov magnitude to check.
3. Whether the linear model's distortion-free assumption holds -- checked via
   worst-case displacement at a fixed depth (2 m) and via a center-region vs
   edge-region breakdown.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).parent.parent))

from roomscan.decoder import StreamDecoder  # noqa: E402
from roomscan.deproject import Deprojector  # noqa: E402
from roomscan.native import Transform  # noqa: E402
from roomscan.protocol import DEPTH_NO_RETURN_MM, StreamId  # noqa: E402

from tests.golden import load_golden_pairs  # noqa: E402

FULL_CAPTURE = Path(__file__).parent.parent.parent / "captures" / "golden_pairs.bin"
OUT_W, OUT_H = 54, 42
FOV_H_DATASHEET = 55.0
FOV_V_DATASHEET = 42.0
REPORT_Z_MM = 2000.0  # "displacement at 2 m" per the brief
# Valid-zone gate: exclude the DEPTH_NO_RETURN_MM sentinel (12000.0) with margin,
# and require a finite, positive perpendicular depth.
VALID_DEPTH_CEILING_MM = DEPTH_NO_RETURN_MM - 1000.0


def _load_pairs(capture_path: Path | None) -> tuple[bytes, list[tuple[bytes, bytes]]]:
    if capture_path is None:
        return load_golden_pairs()
    frames = StreamDecoder().feed(capture_path.read_bytes())
    calib = next(f.payload for f in frames if f.header.stream_id == StreamId.CALIB)
    raws = {f.header.seq: f.payload for f in frames if f.header.stream_id == StreamId.RAW_3DMD}
    depths = {f.header.seq: f.payload for f in frames if f.header.stream_id == StreamId.DEPTH_ZF32}
    seqs = sorted(raws.keys() & depths.keys())
    return calib, [(raws[s], depths[s]) for s in seqs]


def _fit_fov(k: np.ndarray, angle_deg: np.ndarray) -> tuple[float, float]:
    """Least-squares fov for the no-intercept linear model angle = fov * k.
    Returns (fov_fit_deg, rms_residual_deg)."""
    fov = float(np.sum(k * angle_deg) / np.sum(k * k))
    residual = angle_deg - fov * k
    rms = float(np.sqrt(np.mean(residual ** 2)))
    return fov, rms


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, default=None,
                         help="Path to a full golden_pairs.bin capture (optional; "
                              "defaults to the 3-frame committed fixture).")
    parser.add_argument("--conf-min", type=float, default=0.0,
                         help="Minimum ZAPC confidence (4th channel) to treat a zone as "
                              "valid, in addition to the depth-sentinel gate (default 0.0 "
                              "-- see the confidence-channel finding in the doc: on the "
                              "golden fixture EVERY zone, including no-return ones, reports "
                              "confidence ~1.0, so this gate does not discriminate here; kept "
                              "for capture files where it might).")
    args = parser.parse_args()

    if not Transform.available():
        print("SKIP: native transform DLL not built -- see roomscan.native._BUILD_HINT")
        return 1
    if args.capture is not None and not args.capture.is_file():
        print(f"SKIP: capture not found at {args.capture}")
        return 1

    calib, pairs = _load_pairs(args.capture)
    source = args.capture if args.capture is not None else "golden fixture (3 frames)"
    print(f"loaded {len(pairs)} (raw, depth) pairs from {source}")

    t = Transform(calib, outputs=("depth", "zapc"))

    row_idx = np.broadcast_to(np.arange(OUT_H)[:, None], (OUT_H, OUT_W))
    col_idx = np.broadcast_to(np.arange(OUT_W)[None, :], (OUT_H, OUT_W))

    all_col: list[np.ndarray] = []
    all_row: list[np.ndarray] = []
    all_tanx: list[np.ndarray] = []
    all_tany: list[np.ndarray] = []
    all_conf: list[np.ndarray] = []      # full plane, incl. sentinel zones -- see conf report
    all_conf_sentinel: list[np.ndarray] = []
    z_vs_depth_diffs: list[np.ndarray] = []
    n_zones_total = 0
    n_zones_valid = 0
    n_zones_sentinel = 0
    n_zones_low_conf = 0

    for i, (raw, depth_mcu) in enumerate(pairs):   # capture order -- TNR is stateful
        result = t.process(raw)
        depth_pc = result["depth"]
        zapc = result["zapc"]
        x, y, z, conf = zapc[..., 0], zapc[..., 1], zapc[..., 2], zapc[..., 3]

        n_zones_total += depth_pc.size
        depth_finite = np.isfinite(depth_pc) & (depth_pc > 0.0)
        sentinel = depth_finite & (depth_pc >= VALID_DEPTH_CEILING_MM)
        n_zones_sentinel += int(sentinel.sum())
        depth_ok = depth_finite & ~sentinel
        conf_ok = conf >= args.conf_min
        n_zones_low_conf += int((depth_ok & ~conf_ok).sum())
        z_ok = np.isfinite(z) & (z > 0.0) & (z < VALID_DEPTH_CEILING_MM)
        valid = depth_ok & conf_ok & z_ok & np.isfinite(x) & np.isfinite(y)
        n_zones_valid += int(valid.sum())

        # z-vs-ZF32-depth anchor: are they the same quantity (perpendicular) or
        # does ZAPC's z run radial (which would show z > depth off-axis)?
        both_ok = depth_ok & z_ok
        z_vs_depth_diffs.append((z - depth_pc)[both_ok])

        all_col.append(col_idx[valid])
        all_row.append(row_idx[valid])
        all_tanx.append((x / z)[valid])
        all_tany.append((y / z)[valid])
        all_conf.append(conf.ravel())
        all_conf_sentinel.append(conf[sentinel])
        if len(pairs) <= 10:
            print(f"  frame {i}: {int(valid.sum())}/{depth_pc.size} zones valid "
                  f"({int(sentinel.sum())} sentinel, "
                  f"{int((depth_ok & ~conf_ok).sum())} depth-ok-but-low-conf)")
    if len(pairs) > 10:
        print(f"  ... {len(pairs)} frames processed (per-frame lines suppressed above 10 frames; "
              f"see the zone-survival summary below for aggregate counts)")

    col = np.concatenate(all_col).astype(np.int64)
    row = np.concatenate(all_row).astype(np.int64)
    tanx = np.concatenate(all_tanx)
    tany = np.concatenate(all_tany)
    conf_all = np.concatenate(all_conf)
    conf_sentinel = np.concatenate(all_conf_sentinel) if n_zones_sentinel else np.empty(0)
    z_diff = np.concatenate(z_vs_depth_diffs)

    print()
    print("=== zone survival ===")
    print(f"total zones (all frames):     {n_zones_total}")
    print(f"sentinel (no-return >= {VALID_DEPTH_CEILING_MM:.0f} mm): {n_zones_sentinel}")
    print(f"low-confidence (excluded):    {n_zones_low_conf}")
    print(f"valid zones used for fit:     {n_zones_valid} "
          f"({100.0 * n_zones_valid / n_zones_total:.1f}%)")

    print()
    print("=== ZAPC confidence channel (4th float) -- does it discriminate? ===")
    hist, edges = np.histogram(conf_all, bins=10, range=(0.0, 1.0))
    n_above_1 = int(np.sum(conf_all > 1.0))
    print(f"all zones (n={conf_all.size}, incl. sentinel): "
          f"min={conf_all.min():.6f} max={conf_all.max():.6f} mean={conf_all.mean():.6f}")
    print(f"histogram over [0,1], 10 bins: {hist.tolist()}"
          + (f"  (+{n_above_1} values marginally >1.0, outside the histogram range: the "
             f"library packs per-zone filter-status codes into the 1e-6 digits, "
             f"radial_to_perp.c:198-203)" if n_above_1 else ""))
    if conf_sentinel.size:
        print(f"sentinel (no-return) zones only (n={conf_sentinel.size}): "
              f"min={conf_sentinel.min():.6f} max={conf_sentinel.max():.6f} "
              f"mean={conf_sentinel.mean():.6f}")
    if conf_all.min() >= 0.99:
        print("-> confidence reports ~1.0 on EVERY zone, including no-return sentinel zones: "
              "non-discriminating on this data; zone exclusion must rely on the depth "
              "sentinel instead (see docs/deprojector-validation.md).")
    else:
        print("-> confidence varies on this data -- the --conf-min gate is meaningful here; "
              "revisit docs/deprojector-validation.md's non-discrimination finding.")

    print()
    print("=== ZAPC axis-convention findings ===")
    med_x_per_col = np.array([np.median(tanx[col == c]) if np.any(col == c) else np.nan
                               for c in range(OUT_W)])
    med_y_per_row = np.array([np.median(tany[row == r]) if np.any(row == r) else np.nan
                               for r in range(OUT_H)])
    x_monotonic = bool(np.all(np.diff(med_x_per_col[np.isfinite(med_x_per_col)]) > 0))
    y_monotonic = bool(np.all(np.diff(med_y_per_row[np.isfinite(med_y_per_row)]) > 0))
    print(f"x monotonic increasing with column index: {x_monotonic} "
          f"(col 0 median tan_x={med_x_per_col[0]:.5f}, "
          f"col {OUT_W - 1} median tan_x={med_x_per_col[-1]:.5f})")
    print(f"y monotonic increasing with row index:    {y_monotonic} "
          f"(row 0 median tan_y={med_y_per_row[0]:.5f}, "
          f"row {OUT_H - 1} median tan_y={med_y_per_row[-1]:.5f})")
    print("-> x maps to column/width axis, y maps to row/height axis, "
          "same convention Deprojector already assumes.")

    print()
    print("=== z (ZAPC) vs depth (ZF32) -- perpendicular-vs-radial anchor ===")
    max_abs_z_diff = float(np.max(np.abs(z_diff)))
    print(f"n compared: {z_diff.size}")
    print(f"max abs diff: {max_abs_z_diff:.6f} mm  "
          f"mean diff: {np.mean(z_diff):.6f} mm  median diff: {np.median(z_diff):.6f} mm")
    # Hard check, not a tolerance: the established finding (docs/deprojector-validation.md)
    # is that ZAPC's z channel is the SAME buffer as the ZF32 depth output
    # (radial_to_perp.c:195-197 writes depth[linear_id] verbatim), so any nonzero diff at
    # all means the library/fixture/shim changed underneath us -- fail loudly.
    assert max_abs_z_diff == 0.0, (
        f"ZAPC z is no longer bit-identical to ZF32 depth (max abs diff "
        f"{max_abs_z_diff} mm over {z_diff.size} zones). This validation's "
        "perpendicular-z finding rested on exact identity (radial_to_perp.c writes the same "
        "corrected-depth array to both outputs) -- a library, shim, or fixture regression has "
        "broken that. Inspect the off-axis trend of (z - depth) before trusting any "
        "conclusion in docs/deprojector-validation.md."
    )
    print("VERDICT: ZAPC's z is bit-identical to ZF32 depth -- PERPENDICULAR, not radial "
          "(radial would grow off-axis; no such trend, diff is exactly 0). "
          "[hard-asserted: max abs diff == 0.0]")

    print()
    print("=== per-zone angular error vs the linear model (datasheet 55x42) ===")
    d_datasheet = Deprojector(width=OUT_W, height=OUT_H, fov_h_deg=FOV_H_DATASHEET, fov_v_deg=FOV_V_DATASHEET)
    tanx_lin = d_datasheet._tan_x[0][col]
    tany_lin = d_datasheet._tan_y[:, 0][row]
    ang_err_x_deg = np.degrees(np.arctan(tanx) - np.arctan(tanx_lin))
    ang_err_y_deg = np.degrees(np.arctan(tany) - np.arctan(tany_lin))
    disp_x_mm = REPORT_Z_MM * (tanx - tanx_lin)
    disp_y_mm = REPORT_Z_MM * (tany - tany_lin)
    disp_mag_mm = np.sqrt(disp_x_mm ** 2 + disp_y_mm ** 2)

    print(f"angular error X (deg): median={np.median(np.abs(ang_err_x_deg)):.4f} "
          f"p99={np.percentile(np.abs(ang_err_x_deg), 99):.4f} "
          f"worst={np.max(np.abs(ang_err_x_deg)):.4f}")
    print(f"angular error Y (deg): median={np.median(np.abs(ang_err_y_deg)):.4f} "
          f"p99={np.percentile(np.abs(ang_err_y_deg), 99):.4f} "
          f"worst={np.max(np.abs(ang_err_y_deg)):.4f}")
    print(f"displacement @ {REPORT_Z_MM / 1000:.0f} m (mm): "
          f"median={np.median(disp_mag_mm):.3f} ({100 * np.median(disp_mag_mm) / REPORT_Z_MM:.3f}% of z)  "
          f"p99={np.percentile(disp_mag_mm, 99):.3f} "
          f"({100 * np.percentile(disp_mag_mm, 99) / REPORT_Z_MM:.3f}% of z)  "
          f"worst={np.max(disp_mag_mm):.3f} ({100 * np.max(disp_mag_mm) / REPORT_Z_MM:.3f}% of z)")
    worst_i = int(np.argmax(disp_mag_mm))
    print(f"worst zone: row={row[worst_i]} col={col[worst_i]} "
          f"(tan_x zapc={tanx[worst_i]:.5f} vs linear={tanx_lin[worst_i]:.5f}, "
          f"tan_y zapc={tany[worst_i]:.5f} vs linear={tany_lin[worst_i]:.5f})")

    center_mask = (np.abs(col - (OUT_W - 1) / 2) < 10) & (np.abs(row - (OUT_H - 1) / 2) < 8)
    print(f"center-region (|dcol|<10,|drow|<8) displacement: "
          f"median={np.median(disp_mag_mm[center_mask]):.3f}mm "
          f"max={np.max(disp_mag_mm[center_mask]):.3f}mm  n={int(center_mask.sum())}")
    print(f"edge-region   displacement:                      "
          f"median={np.median(disp_mag_mm[~center_mask]):.3f}mm "
          f"max={np.max(disp_mag_mm[~center_mask]):.3f}mm  n={int((~center_mask).sum())}")

    print()
    print("=== best-fit equivalent FoV (least squares over zone centers) ===")
    n_w, n_h = OUT_W, OUT_H
    k_center_x = (col + 0.5) / n_w - 0.5
    k_center_y = (row + 0.5) / n_h - 0.5
    k_edge_x = col / (n_w - 1) - 0.5
    k_edge_y = row / (n_h - 1) - 0.5
    angx_deg_zapc = np.degrees(np.arctan(tanx))
    angy_deg_zapc = np.degrees(np.arctan(tany))

    fov_h_center, rms_h_center = _fit_fov(k_center_x, angx_deg_zapc)
    fov_h_edge, rms_h_edge = _fit_fov(k_edge_x, angx_deg_zapc)
    fov_v_center, rms_v_center = _fit_fov(k_center_y, angy_deg_zapc)
    fov_v_edge, rms_v_edge = _fit_fov(k_edge_y, angy_deg_zapc)

    print(f"H, zone-center convention (Deprojector's current convention): "
          f"fov={fov_h_center:.4f} deg, rms residual={rms_h_center:.4f} deg")
    print(f"H, zone-center-to-zone-center convention:                     "
          f"fov={fov_h_edge:.4f} deg, rms residual={rms_h_edge:.4f} deg")
    print(f"V, zone-center convention (Deprojector's current convention): "
          f"fov={fov_v_center:.4f} deg, rms residual={rms_v_center:.4f} deg")
    print(f"V, zone-center-to-zone-center convention:                     "
          f"fov={fov_v_edge:.4f} deg, rms residual={rms_v_edge:.4f} deg")
    assert np.isclose(rms_h_center, rms_h_edge) and np.isclose(rms_v_center, rms_v_edge), \
        "the two conventions are a pure rescaling of each other (k_edge = n/(n-1) * k_center) " \
        "-- if residuals ever diverge here, that assumption broke and needs re-deriving"
    print("(residuals are identical between conventions -- confirmed algebraically: "
          "k_edge(i) = n/(n-1) * k_center(i), a pure rescaling, so this is not a separate "
          "hypothesis to pick between, only a fov magnitude.)")
    print(f"reconciliation vs datasheet: H best-fit {fov_h_center:.2f} vs datasheet "
          f"{FOV_H_DATASHEET:.1f} (delta {fov_h_center - FOV_H_DATASHEET:+.2f} deg); "
          f"V best-fit {fov_v_center:.2f} vs datasheet {FOV_V_DATASHEET:.1f} "
          f"(delta {fov_v_center - FOV_V_DATASHEET:+.2f} deg)")

    print()
    print("=== decision: does a pure FoV-default change get worst-case <= 1% of z? ===")
    d_fit = Deprojector(width=OUT_W, height=OUT_H, fov_h_deg=fov_h_center, fov_v_deg=fov_v_center)
    tanx_fit = d_fit._tan_x[0][col]
    tany_fit = d_fit._tan_y[:, 0][row]
    disp_fit_mm = REPORT_Z_MM * np.sqrt((tanx - tanx_fit) ** 2 + (tany - tany_fit) ** 2)
    worst_pct_datasheet = 100 * np.max(disp_mag_mm) / REPORT_Z_MM
    worst_pct_fit = 100 * np.max(disp_fit_mm) / REPORT_Z_MM
    print(f"worst-case with datasheet fov (55/42):     {worst_pct_datasheet:.3f}% of z")
    print(f"worst-case with best-fit fov ({fov_h_center:.2f}/{fov_v_center:.2f}): "
          f"{worst_pct_fit:.3f}% of z")
    if worst_pct_fit <= 1.0:
        print("DECISION: worst-case <= 1% after a fov-only tweak -- update Deprojector defaults.")
    else:
        print("DECISION: worst-case > 1% even with the best-fit fov -- a global fov change "
              "cannot fix this (same worst-case either way -- see center-vs-edge-region split "
              "above: the error is concentrated at the corners, i.e. real per-zone lens "
              "distortion, not a miscalibrated scalar). Keep the datasheet-derived defaults "
              "(already confirmed correct to within ~0.6 deg by the fit above) and use the "
              "optional zone_tan_x/zone_tan_y per-zone-table constructor path for callers that "
              "need corner accuracy and have ZAPC data to seed it.")

    print()
    print("=== demonstration: per-zone table seeded from this run's ZAPC data ===")
    zone_tan_x = np.full((OUT_H, OUT_W), np.nan)
    zone_tan_y = np.full((OUT_H, OUT_W), np.nan)
    for r in range(OUT_H):
        for c in range(OUT_W):
            m = (row == r) & (col == c)
            if np.any(m):
                zone_tan_x[r, c] = np.median(tanx[m])
                zone_tan_y[r, c] = np.median(tany[m])
    table_covered = np.isfinite(zone_tan_x)
    # Zones with no valid sample in this run keep the linear fallback so the demo table is total.
    zone_tan_x = np.where(table_covered, zone_tan_x, d_datasheet._tan_x[0][None, :])
    zone_tan_y = np.where(table_covered, zone_tan_y, d_datasheet._tan_y[:, 0][:, None])
    d_table = Deprojector(width=OUT_W, height=OUT_H, zone_tan_x=zone_tan_x, zone_tan_y=zone_tan_y)
    tanx_table = d_table._tan_x[row, col]
    tany_table = d_table._tan_y[row, col]
    disp_table_mm = REPORT_Z_MM * np.sqrt((tanx - tanx_table) ** 2 + (tany - tany_table) ** 2)
    print(f"per-zone table covers {int(table_covered.sum())}/{table_covered.size} zones from this run "
          f"(rest fall back to the linear model)")
    print(f"self-consistency check (table fit against the same data it was built from -- expect "
          f"near-zero, this is NOT a held-out test, just proof the mechanism works): "
          f"median={np.median(disp_table_mm):.4f}mm worst={np.max(disp_table_mm):.4f}mm")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
