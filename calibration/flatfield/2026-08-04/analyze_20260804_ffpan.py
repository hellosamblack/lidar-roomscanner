"""Analyze the 2026-08-04 PANNED flat-field captures (DC-D2 candidate set).

Unlike the stationary ceiling study (`analyze_20260804_flat_field.py`), these six
captures were recorded while slowly panning across the ceiling at three different
operator heights.  Panning + multiple heights is what makes a flat-field map valid:
real scene texture (and any smoke detector / shelf that drifts through the FOV)
smears across zones and averages toward the smooth illumination, while the sensor's
fixed per-zone response stays locked.  The three heights additionally give a
held-out *distance* test -- the scene-independence control the static study could
not provide (a static half-split only proves temporal stability).

This is read-only.  It reuses the native transform, the Deprojector, and
`build_flatfield` exactly as the viewer does, and imports the shared helpers from
the stationary study's script rather than reimplementing them.

Outputs beside the captures:
  * ffpan_20260804_metrics.json  -- machine-readable metrics + map-portability matrix
  * ffpan_20260804_<capture>.png -- per-capture depth-residual / reflectance / gain panels
                                     (eyeball these for the smoke detector / shelves)
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

# Reuse the stationary study's transform loader + helpers verbatim -- same native
# path as the viewer, so any difference here is the capture, not the analysis.
from analyze_20260804_flat_field import (  # noqa: E402
    load_capture, fit_plane, reflectance_residual,
)
from roomscan.deproject import Deprojector  # noqa: E402
from roomscan.flatfield import FlatField, _blur, build_flatfield  # noqa: E402

WARMUP = 30
# A "near object" is a zone reading this much closer than the fitted ceiling plane.
# A ceiling smoke detector protrudes ~30-50 mm; a shelf/soffit near the ceiling can
# be far closer.  100 mm is well outside the plane-fit residual (a few mm) and the
# per-frame noise, so a sustained cluster of near-object zones is a real intrusion
# that the pan did NOT average out.
NEAR_OBJECT_MM = 100.0

# family / power / exposure decoded from the operator's filename.  The two "8or16"
# files are the ones whose exposure the operator was unsure of; resolved below.
CAPTURES = {
    "precisionRegular8msFFpan.bin":  {"family": "precision", "power": "regular", "exp_claim": "8"},
    "precisionULP8msFFpan.bin":      {"family": "precision", "power": "ulp",     "exp_claim": "8"},
    "ambientRegular8msFFpan.bin":    {"family": "ambient",   "power": "regular", "exp_claim": "8"},
    "ambientULP8msFFpan.bin":        {"family": "ambient",   "power": "ulp",     "exp_claim": "8"},
    "ambientRegular8or16msFFpan.bin":{"family": "ambient",   "power": "regular", "exp_claim": "8or16"},
    "ambientULP8or16msFFpan.bin":    {"family": "ambient",   "power": "ulp",     "exp_claim": "8or16"},
}


def stats(x):
    x = np.asarray(x, dtype=np.float64)
    x = x[np.isfinite(x)]
    if not x.size:
        return {"n": 0}
    return {"n": int(x.size), "median": float(np.median(x)), "mean": float(np.mean(x)),
            "p05": float(np.percentile(x, 5)), "p95": float(np.percentile(x, 95)),
            "min": float(x.min()), "max": float(x.max())}


def distance_bands(frame_med: np.ndarray, k: int = 3) -> dict:
    """Split settled frames into k distance (=height) bands by 1-D k-means on the
    per-frame median ceiling depth.  Confirms the operator's three heights actually
    produced three distinct standoffs (and enables the held-out-distance test)."""
    x = frame_med[np.isfinite(frame_med)]
    lo, hi = np.percentile(x, 2), np.percentile(x, 98)
    centers = np.linspace(lo, hi, k)
    for _ in range(50):
        d = np.abs(x[:, None] - centers[None, :])
        lab = d.argmin(axis=1)
        new = np.array([x[lab == j].mean() if np.any(lab == j) else centers[j] for j in range(k)])
        if np.allclose(new, centers, atol=0.5):
            centers = new
            break
        centers = new
    order = np.argsort(centers)
    centers = centers[order]
    counts = [int(np.sum(lab == order[j])) for j in range(k)]
    return {"centers_mm": [float(c) for c in centers], "counts": counts,
            "spread_mm": float(centers.max() - centers.min())}


def held_out_distance(refl: np.ndarray, frame_med: np.ndarray) -> dict:
    """Build a flat-field map on the CLOSEST-height frames and score its FPN
    reduction on the FARTHEST-height frames (and vice versa).  Because the ceiling
    texture/illumination projects differently at a different standoff while the
    sensor FPN is locked, a reduction that survives the distance swap is the
    scene-independence evidence the static half-split cannot give.  Compared against
    a within-distance temporal control (train/score inside the closest band)."""
    m = np.isfinite(frame_med)
    idx = np.where(m)[0]
    fv = frame_med[idx]
    q33, q66 = np.percentile(fv, [33, 66])
    near = idx[fv <= q33]           # closest standoff
    far = idx[fv >= q66]            # farthest standoff
    if near.size < 20 or far.size < 20:
        return {"skipped": "insufficient distinct-distance frames"}

    def resid(frames):
        return reflectance_residual(np.nanmean(frames, axis=0))

    def build(frames):
        return build_flatfield(np.nan_to_num(frames, nan=0.0), note="held-out-distance")

    def apply_resid(ff, frames):
        return reflectance_residual(np.mean([ff.apply(f) for f in frames], axis=0))

    near_map = build(refl[near])
    far_map = build(refl[far])
    # temporal control: split the near band in half by time
    h = near.size // 2
    near_a = build(refl[near[:h]])
    return {
        "near_center_mm": float(np.median(fv[fv <= q33])),
        "far_center_mm": float(np.median(fv[fv >= q66])),
        "far_raw_pct": resid(refl[far]),
        "far_after_nearmap_pct": apply_resid(near_map, refl[far]),      # <-- the real control
        "near_raw_pct": resid(refl[near]),
        "near_after_farmap_pct": apply_resid(far_map, refl[near]),      # reverse direction
        "near_after_nearhalfmap_pct": apply_resid(near_a, refl[near[h:]]),  # temporal-only baseline
    }


def contamination(zone_median, plane, depth_settled, refl_mean) -> dict:
    """Look for a smoke detector / shelf the pan did not average out.

    Three independent tells:
      * `plane_residual` -- a localized bump in the temporal-median depth surface
        (a static intrusion that dwelt in some zones survives the median).
      * `near_object` -- per-frame count of zones reading >=NEAR_OBJECT_MM closer
        than the plane; a sustained cluster means an object sat in the FOV.
      * `refl_lowfreq` -- low-frequency (non-grid) structure in the reflectance
        mean beyond the smooth illumination; the FPN is high-frequency, so a broad
        blob that is NOT row/col grid is a candidate object/illumination artifact.
    """
    h, w = zone_median.shape
    rr, cc = np.indices((h, w))
    xy = np.stack([cc, rr], -1).astype(float)
    pred = plane["tilt_x_mm_per_m"] / 1000.0  # not used directly; residual below is exact
    resid_map = zone_median - _blur(np.nan_to_num(zone_median, nan=np.nanmedian(zone_median)), 6.0)
    # per-frame near-object zone counts
    good = np.isfinite(depth_settled) & (depth_settled > 0)
    plane_pred = _blur(np.nan_to_num(zone_median, nan=np.nanmedian(zone_median)), 6.0)
    near = good & (depth_settled < (plane_pred[None] - NEAR_OBJECT_MM))
    near_per_frame = near.reshape(near.shape[0], -1).sum(axis=1)
    # locate the worst persistent near-object zone
    near_frac_zone = near.mean(axis=0)
    worst = np.unravel_index(np.argmax(near_frac_zone), near_frac_zone.shape)
    # reflectance low-frequency (non-grid) content
    refl_low = _blur(np.nan_to_num(refl_mean, nan=np.nanmedian(refl_mean)), 4.0)
    refl_low_cv = float(100.0 * np.nanstd(refl_low) / max(np.nanmean(refl_low), 1e-6))
    return {
        "plane_residual_abs_p99_mm": float(np.nanpercentile(np.abs(resid_map), 99)),
        "plane_residual_abs_max_mm": float(np.nanmax(np.abs(resid_map))),
        "near_object_zones_per_frame": stats(near_per_frame),
        "near_object_worst_zone": {"row": int(worst[0]), "col": int(worst[1]),
                                   "frac_frames": float(near_frac_zone[worst])},
        "frames_with_ge5_near_zones_pct": float(100.0 * np.mean(near_per_frame >= 5)),
        "reflectance_lowfreq_cv_pct": refl_low_cv,
    }


def analyze(name: str, info: dict) -> dict:
    path = HERE / name
    arrays, meta = load_capture(path)
    depth = arrays["depth"].astype(np.float64)
    refl = arrays["reflectance"].astype(np.float64)
    ambient = arrays["ambient"].astype(np.float64)
    conf = arrays["confidence"].astype(np.float64)
    n = depth.shape[0]
    sl = slice(min(WARMUP, max(0, n - 1)), n)
    z, r, a, c = depth[sl], refl[sl], ambient[sl], conf[sl]
    valid = np.isfinite(z) & (z > 0) & (z < 10000)
    zt = np.where(valid, z, np.nan)
    frame_med = np.nanmedian(zt, axis=(1, 2))
    zone_med = np.nanmedian(zt, axis=0)
    zone_good = np.isfinite(zone_med)
    plane = fit_plane(zone_med, zone_good)
    refl_mean = np.nanmean(r, axis=0)

    ff = build_flatfield(np.nan_to_num(r, nan=0.0), note=f"panned {name}")
    ff.save(HERE / f"ffpan_20260804_{path.stem}.npz")
    gain = ff.gain
    gain_low = _blur(gain, 4.0)   # low-freq gain should be ~1 for a clean pan

    m = {
        "file": name, **info,
        "raw_frames": int(n), "settled_frames": int(z.shape[0]),
        "depth": {
            "accuracy_note": "no metrology target on a panned capture; distance varies by design",
            "frame_median_mm": stats(frame_med),
            "distance_bands": distance_bands(frame_med),
            "plane_fit": plane,
        },
        "reflectance": {
            "raw_fpn_pct": reflectance_residual(refl_mean),
            "gain_min": float(gain.min()), "gain_max": float(gain.max()),
            "gain_std_pct": float(100.0 * gain.std()),
            "gain_frac_outside_0p5_1p6_pct": float(100.0 * np.mean((gain < 0.5) | (gain > 1.6))),
            "gain_frac_at_clip_pct": float(100.0 * np.mean((gain <= 0.34) | (gain >= 2.99))),
            "gain_lowfreq_std_pct": float(100.0 * gain_low.std()),  # ~0 = illumination preserved
            "builder_meta": ff.meta,
        },
        "held_out_distance": held_out_distance(r, frame_med),
        "contamination": contamination(zone_med, plane, z, refl_mean),
        "exposure_signal": {
            # integration time scales accumulated photons: longer exposure -> higher
            # ambient counts and lower per-zone temporal noise at the same scene.
            "ambient_mean": float(np.nanmean(a)),
            "ambient_median": float(np.nanmedian(a)),
            "reflectance_mean": float(np.nanmean(r)),
            "confidence_mean": float(np.nanmean(c[np.isfinite(c)])),
            "zone_temporal_std_mm_median": float(np.nanmedian(np.nanstd(zt, axis=0)[zone_good])),
        },
    }
    # persist compact maps for the PNG + cross-capture correlation
    np.savez_compressed(HERE / f"ffpan_maps_{path.stem}.npz",
                        depth_median_mm=zone_med.astype(np.float32),
                        reflectance_mean=refl_mean.astype(np.float32),
                        gain=gain.astype(np.float32))
    return m


def main() -> int:
    results = [analyze(name, info) for name, info in CAPTURES.items()]

    # ---- resolve the 8-vs-16 ms ambiguity against the known-8 ms sibling of the
    # same family+power.  Longer exposure ~doubles ambient counts and cuts noise. ----
    known = {(m["family"], m["power"]): m for m in results if m["exp_claim"] == "8"}
    exposure_calls = {}
    for m in results:
        if m["exp_claim"] != "8or16":
            continue
        ref = known.get((m["family"], m["power"]))
        e = m["exposure_signal"]
        if ref is None:
            exposure_calls[m["file"]] = {"call": "unknown", "reason": "no 8 ms sibling"}
            continue
        re = ref["exposure_signal"]
        amb_ratio = e["ambient_mean"] / max(re["ambient_mean"], 1e-9)
        noise_ratio = e["zone_temporal_std_mm_median"] / max(re["zone_temporal_std_mm_median"], 1e-9)
        # 16 ms vs an 8 ms sibling: ambient ~2x, noise ~1/sqrt(2)=0.71x.
        call = "16" if (amb_ratio > 1.5 or noise_ratio < 0.82) else "8"
        exposure_calls[m["file"]] = {
            "call": call, "vs_sibling": ref["file"],
            "ambient_ratio": round(amb_ratio, 3), "noise_ratio": round(noise_ratio, 3),
            "note": "ambient ~2x AND/OR noise ~0.71x => 16 ms; both ~1x => 8 ms",
        }

    # ---- cross-capture gain-map portability (does the family split reproduce?) ----
    gains = {}
    for m in results:
        with np.load(HERE / f"ffpan_maps_{Path(m['file']).stem}.npz") as d:
            gains[m["file"]] = np.asarray(d["gain"], dtype=np.float64).ravel()
    names = list(gains)
    corr = {}
    for i, li in enumerate(names):
        for rj in names[i + 1:]:
            corr[f"{li} vs {rj}"] = round(float(np.corrcoef(gains[li], gains[rj])[0, 1]), 4)

    # cross-apply each map to every capture's reflectance mean (family mismatch => high residual)
    cross = {}
    means = {}
    for m in results:
        with np.load(HERE / f"ffpan_maps_{Path(m['file']).stem}.npz") as d:
            means[m["file"]] = np.asarray(d["reflectance_mean"], dtype=np.float64)
    for tgt, img in means.items():
        cross[tgt] = {}
        for refn, m in ((mm["file"], mm) for mm in results):
            g = gains[refn].reshape(img.shape)
            cross[tgt][refn] = round(reflectance_residual(img * g), 3)

    out = {"warmup_frames": WARMUP, "near_object_mm": NEAR_OBJECT_MM,
           "captures": results, "exposure_calls": exposure_calls,
           "gain_map_correlation": corr, "cross_applied_fpn_pct": cross}
    (HERE / "ffpan_20260804_metrics.json").write_text(json.dumps(out, indent=2) + "\n")

    # ---- per-capture PNG so the smoke detector / shelves are eyeball-checkable ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for m in results:
            stem = Path(m["file"]).stem
            with np.load(HERE / f"ffpan_maps_{stem}.npz") as d:
                dep = d["depth_median_mm"].astype(float)
                rfl = d["reflectance_mean"].astype(float)
                gn = d["gain"].astype(float)
            dep_res = dep - _blur(np.nan_to_num(dep, nan=np.nanmedian(dep)), 6.0)
            fig, ax = plt.subplots(1, 3, figsize=(13, 3.4), constrained_layout=True)
            for a_, img, ttl, cm in (
                (ax[0], dep_res, "depth residual vs smooth (mm)\n<- dark = closer object", "coolwarm"),
                (ax[1], rfl, "reflectance mean", "viridis"),
                (ax[2], gn, "flat-field gain", "coolwarm")):
                im = a_.imshow(img, cmap=cm, interpolation="nearest")
                a_.set_title(ttl, fontsize=9)
                fig.colorbar(im, ax=a_, shrink=0.8)
            fig.suptitle(f"{m['file']}  ({m['family']}/{m['power']}, exp {m['exp_claim']})", fontsize=11)
            fig.savefig(HERE / f"ffpan_20260804_{stem}.png", dpi=140)
            plt.close(fig)
    except Exception as exc:  # noqa: BLE001
        (HERE / "ffpan_20260804_plot_error.txt").write_text(f"{type(exc).__name__}: {exc}\n")

    # ---- console summary ----
    print("\n=== per-capture ===")
    for m in results:
        r, ho, ct = m["reflectance"], m["held_out_distance"], m["contamination"]
        db = m["depth"]["distance_bands"]
        print(f"\n{m['file']}  [{m['family']}/{m['power']}, exp {m['exp_claim']}]  "
              f"{m['settled_frames']} frames")
        print(f"  distance bands (mm): {[round(c) for c in db['centers_mm']]} "
              f"counts {db['counts']} spread {db['spread_mm']:.0f} mm")
        print(f"  raw FPN {r['raw_fpn_pct']:.2f}%  gain [{r['gain_min']:.3f},{r['gain_max']:.3f}] "
              f"outside[0.5,1.6] {r['gain_frac_outside_0p5_1p6_pct']:.1f}%  "
              f"at-clip {r['gain_frac_at_clip_pct']:.2f}%  lowfreq-gain-std {r['gain_lowfreq_std_pct']:.2f}%")
        if "skipped" not in ho:
            print(f"  held-out DISTANCE: far raw {ho['far_raw_pct']:.2f}% -> near-map "
                  f"{ho['far_after_nearmap_pct']:.2f}%  (temporal-only baseline "
                  f"{ho['near_after_nearhalfmap_pct']:.2f}%)")
        else:
            print(f"  held-out DISTANCE: {ho['skipped']}")
        print(f"  contamination: plane-resid p99 {ct['plane_residual_abs_p99_mm']:.1f}mm "
              f"max {ct['plane_residual_abs_max_mm']:.1f}mm  "
              f">=5 near-zones in {ct['frames_with_ge5_near_zones_pct']:.1f}% of frames  "
              f"worst zone (r{ct['near_object_worst_zone']['row']},c{ct['near_object_worst_zone']['col']}) "
              f"{100*ct['near_object_worst_zone']['frac_frames']:.0f}% of frames")
    print("\n=== exposure calls (8or16 ms) ===")
    for f, v in exposure_calls.items():
        print(f"  {f}: {v}")
    print("\n=== gain-map correlation ===")
    for k, v in corr.items():
        print(f"  {k}: {v}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
