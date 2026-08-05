"""Cross-room (held-out-SCENE) validation of the panned flat-field maps.

The panned study (`ffpan_20260804_report.md`) showed the maps flatten their OWN
capture to ~1.3% and a *different capture of the same family in the same room* to
~2%, but all six shared one ceiling -- so scene-independence was still unproven.
These two captures were taken in a larger, different room:

  * precisionRegular8msFFpanLarge.bin
  * ambientRegular8msFFpanLarge.bin

The real test: build a map in room A, apply it to room B (and vice versa).  If a
room-A map flattens room-B data as well as room B's own map does, the correction
is genuine sensor behaviour, not a memorised room.  Read-only; reuses the native
transform + `build_flatfield` exactly as the viewer does.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "host" / "src"))
sys.path.insert(0, str(ROOT / "host"))
sys.path.insert(0, str(HERE))

from analyze_20260804_flat_field import load_capture, fit_plane, reflectance_residual  # noqa: E402
from roomscan.deproject import Deprojector  # noqa: E402
from roomscan.flatfield import build_flatfield, _blur  # noqa: E402

WARMUP = 30
# A single flat surface fits ONE plane to a few mm; if a wall drifts into the FOV
# at the start/end turnarounds the frame holds two planes at an angle, so a
# best-fit single plane leaves a large residual.  Reject any frame whose per-frame
# plane RMS exceeds this, or that has too few returns.  (Panning + tilt is removed
# by the plane fit, so tilt alone does not trip it; a clean panned ceiling frame
# sits well under 25 mm.)
WALL_RMS_MM = 25.0
MIN_VALID_FRAC = 0.6

# Both rooms get IDENTICAL treatment: transform the raw capture, reject wall
# frames, build the map from the survivors.  Room A = first-room pans; Room B =
# the larger room.  (Room-A maps are rebuilt here rather than reused from the
# ffpan npz so the wall rejection applies symmetrically to both sides.)
ROOM_A = {
    "precision": "precisionRegular8msFFpan.bin",
    "ambient":   "ambientRegular8msFFpan.bin",
}
ROOM_B = {
    "precision": "precisionRegular8msFFpanLarge.bin",
    "ambient":   "ambientRegular8msFFpanLarge.bin",
}


def _per_frame_plane_rms(z: np.ndarray, deproj: Deprojector) -> np.ndarray:
    """RMS of each frame's depth about its own best-fit plane, in mm.  High = the
    frame is not a single flat surface (a wall has drifted in)."""
    n, h, w = z.shape
    rms = np.full(n, np.inf)
    for i in range(n):
        di = z[i]
        valid = np.isfinite(di) & (di > 0) & (di < 10000)
        if valid.mean() < MIN_VALID_FRAC or valid.sum() < 3:
            continue
        grid, _ = deproj.grid(di)
        xy = grid * 1000.0
        rows, cols = np.where(valid)
        A = np.column_stack((xy[rows, cols, 0], xy[rows, cols, 1], np.ones(rows.size)))
        zz = di[rows, cols]
        coef, *_ = np.linalg.lstsq(A, zz, rcond=None)
        res = zz - A @ coef
        rms[i] = float(np.sqrt(np.mean(res * res)))
    return rms


def settled_reflectance(path: Path):
    arrays, _ = load_capture(path)
    depth = arrays["depth"].astype(np.float64)
    refl = arrays["reflectance"].astype(np.float64)
    n = depth.shape[0]
    sl = slice(min(WARMUP, max(0, n - 1)), n)
    z = depth[sl]
    r = refl[sl]
    deproj = Deprojector(z.shape[2], z.shape[1])
    rms = _per_frame_plane_rms(z, deproj)
    accept = np.isfinite(rms) & (rms <= WALL_RMS_MM)
    # where did the rejects fall?  the wall should be at the start/end turnarounds.
    idx = np.arange(z.shape[0])
    head = idx < 0.1 * z.shape[0]
    tail = idx >= 0.9 * z.shape[0]
    rej = ~accept
    z, r = z[accept], r[accept]
    valid = np.isfinite(z) & (z > 0) & (z < 10000)
    zt = np.where(valid, z, np.nan)
    return {
        "refl_settled": r,
        "refl_mean": np.nanmean(r, axis=0),
        "frame_median_mm": np.nanmedian(zt, axis=(1, 2)),
        "zone_median_mm": np.nanmedian(zt, axis=0),
        "n": int(z.shape[0]),
        "n_before": int(accept.size),
        "rejected": int(rej.sum()),
        "rejected_frac_in_head10_tail10": float(
            (rej & (head | tail)).sum() / max(1, rej.sum())),
        "reject_rms_p95_mm": float(np.nanpercentile(rms[np.isfinite(rms)], 95)),
    }


def _build(name: str, room: str) -> dict:
    s = settled_reflectance(HERE / name)
    ff = build_flatfield(np.nan_to_num(s["refl_settled"], nan=0.0), note=f"{room} {name}")
    gain = ff.gain.astype(float)
    zone_med = s["zone_median_mm"]
    dep_smooth = _blur(np.nan_to_num(zone_med, nan=np.nanmedian(zone_med)), 6.0)
    return {
        "file": name, "n": s["n"], "n_before": s["n_before"], "rejected": s["rejected"],
        "rejected_frac_in_head10_tail10": round(s["rejected_frac_in_head10_tail10"], 3),
        "reject_rms_p95_mm": round(s["reject_rms_p95_mm"], 1),
        "mean": s["refl_mean"], "gain": gain,
        "raw_fpn": reflectance_residual(s["refl_mean"]),
        "self_fpn": reflectance_residual(s["refl_mean"] * gain),
        "gain_min": float(gain.min()), "gain_max": float(gain.max()),
        "gain_frac_outside_0p5_1p6": float(100.0 * np.mean((gain < 0.5) | (gain > 1.6))),
        "gain_lowfreq_std_pct": float(100.0 * _blur(gain, 4.0).std()),
        "dist_spread_mm": float(np.nanpercentile(s["frame_median_mm"], 98)
                                - np.nanpercentile(s["frame_median_mm"], 2)),
        "plane_resid_p99_mm": float(np.nanpercentile(np.abs(zone_med - dep_smooth), 99)),
        "plane_resid_max_mm": float(np.nanmax(np.abs(zone_med - dep_smooth))),
    }


def main() -> int:
    # ---- transform both rooms, wall-reject, build maps (symmetric treatment) ----
    A = {fam: _build(name, "roomA") for fam, name in ROOM_A.items()}
    B = {fam: _build(name, "roomB") for fam, name in ROOM_B.items()}

    # ---- the held-out cross-room numbers ----
    results = {}
    for fam in ("precision", "ambient"):
        a, b = A[fam], B[fam]
        results[fam] = {
            "roomB_file": b["file"], "roomB_frames": b["n"], "roomB_dist_spread_mm": b["dist_spread_mm"],
            "roomA_wall_rejected": f"{a['rejected']}/{a['n_before']}",
            "roomB_wall_rejected": f"{b['rejected']}/{b['n_before']}",
            "roomB_reject_in_ends_frac": b["rejected_frac_in_head10_tail10"],
            # room B on room B (in-scene references)
            "B_raw_fpn_pct": round(b["raw_fpn"], 3),
            "B_selfmap_fpn_pct": round(b["self_fpn"], 3),
            # THE TEST: room-A map applied to room-B mean, and reverse
            "B_after_roomA_map_pct": round(reflectance_residual(b["mean"] * a["gain"]), 3),
            "A_raw_fpn_pct": round(reflectance_residual(a["mean"]), 3),
            "A_after_roomB_map_pct": round(reflectance_residual(a["mean"] * b["gain"]), 3),
            # scene-independence direct: correlation of the two rooms' gain maps
            "AB_gain_correlation": round(float(np.corrcoef(a["gain"].ravel(), b["gain"].ravel())[0, 1]), 4),
            # contamination / validity of the room-B capture
            "B_gain_range": [round(b["gain_min"], 3), round(b["gain_max"], 3)],
            "B_gain_frac_outside_0p5_1p6_pct": round(b["gain_frac_outside_0p5_1p6"], 3),
            "B_gain_lowfreq_std_pct": round(b["gain_lowfreq_std_pct"], 3),
            "B_plane_resid_p99_mm": round(b["plane_resid_p99_mm"], 2),
            "B_plane_resid_max_mm": round(b["plane_resid_max_mm"], 2),
        }

    # ---- cross-FAMILY within room B (does the mode split reproduce across rooms?) ----
    cross_family = {
        "ambientB_after_precisionA_map_pct": round(reflectance_residual(B["ambient"]["mean"] * A["precision"]["gain"]), 3),
        "precisionB_after_ambientA_map_pct": round(reflectance_residual(B["precision"]["mean"] * A["ambient"]["gain"]), 3),
        "ambientB_after_precisionB_map_pct": round(reflectance_residual(B["ambient"]["mean"] * B["precision"]["gain"]), 3),
    }

    out = {"warmup_frames": WARMUP, "per_family": results, "cross_family": cross_family}
    (HERE / "crossroom_20260804_metrics.json").write_text(json.dumps(out, indent=2) + "\n")

    # ---- PNGs: room A gain vs room B gain vs their difference ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for fam in ("precision", "ambient"):
            ga, gb = A[fam]["gain"], B[fam]["gain"]
            fig, ax = plt.subplots(1, 4, figsize=(16, 3.3), constrained_layout=True)
            for a_, img, ttl, cm in (
                (ax[0], B[fam]["mean"], f"room B {fam} reflectance mean", "viridis"),
                (ax[1], ga, "room A gain", "coolwarm"),
                (ax[2], gb, "room B gain", "coolwarm"),
                (ax[3], gb - ga, "room B - room A gain\n(flat = scene-independent)", "coolwarm")):
                im = a_.imshow(img, cmap=cm, interpolation="nearest",
                               **({"vmin": -0.15, "vmax": 0.15} if ttl.startswith("room B - ") else {}))
                a_.set_title(ttl, fontsize=9); fig.colorbar(im, ax=a_, shrink=0.8)
            fig.suptitle(f"cross-room {fam}: corr {results[fam]['AB_gain_correlation']}", fontsize=11)
            fig.savefig(HERE / f"crossroom_20260804_{fam}.png", dpi=140); plt.close(fig)
    except Exception as exc:  # noqa: BLE001
        (HERE / "crossroom_plot_error.txt").write_text(f"{type(exc).__name__}: {exc}\n")

    # ---- console ----
    print("\n=== HELD-OUT CROSS-ROOM (build room A, apply room B) ===")
    for fam in ("precision", "ambient"):
        r = results[fam]
        print(f"\n{fam.upper()}  (room B: {r['roomB_file']}, {r['roomB_frames']} frames, "
              f"dist spread {r['roomB_dist_spread_mm']:.0f} mm)")
        print(f"  wall-reject: room A {r['roomA_wall_rejected']}, room B {r['roomB_wall_rejected']} "
              f"({100*r['roomB_reject_in_ends_frac']:.0f}% of B rejects in first/last 10%)")
        print(f"  room-B validity : gain {r['B_gain_range']} outside[0.5,1.6] "
              f"{r['B_gain_frac_outside_0p5_1p6_pct']}%  lowfreq-gain {r['B_gain_lowfreq_std_pct']}%  "
              f"plane-resid p99 {r['B_plane_resid_p99_mm']}mm max {r['B_plane_resid_max_mm']}mm")
        print(f"  room B: raw {r['B_raw_fpn_pct']}%  ->  self-map {r['B_selfmap_fpn_pct']}%  "
              f"->  ROOM-A map {r['B_after_roomA_map_pct']}%   <-- held out")
        print(f"  reverse: room A raw {r['A_raw_fpn_pct']}%  ->  ROOM-B map {r['A_after_roomB_map_pct']}%")
        print(f"  room A vs room B gain correlation: {r['AB_gain_correlation']}")
    print("\n=== cross-family (mode split reproduces across rooms?) ===")
    for k, v in cross_family.items():
        print(f"  {k}: {v}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
