"""Quantify the 2026-08-04 stationary ceiling captures.

This is a reproducible, read-only analysis of the raw captures.  It deliberately
keeps the native transform and the project's Deprojector in the loop, rather than
reimplementing either one here.  Outputs are written beside the captures:

* ``analysis_20260804_metrics.json`` -- machine-readable metrics;
* one ``flatfield_20260804_<capture>.npz`` per capture; and
* ``analysis_20260804_metrics.csv`` -- compact comparison table.

The target distance is 55 in = 1397.0 mm.  That is treated as an approximate
operator measurement, not as a metrology-grade reference.
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "host" / "src"))
sys.path.insert(0, str(ROOT / "host"))

from roomscan.decoder import StreamDecoder  # noqa: E402
from roomscan.deproject import Deprojector  # noqa: E402
from roomscan.flatfield import FlatField, _blur, build_flatfield  # noqa: E402
from roomscan.pipeline import TransformStage  # noqa: E402
from roomscan.protocol import StreamId  # noqa: E402


TARGET_MM = 55.0 * 25.4
WARMUP = 30
CAPTURES = [
    "4msExpPrecision55inchFlatField.bin",
    "8msExpPrecision55inchFlatField.bin",
    "8msExpPrecision60hzEnv55inchFlatField.bin",
    "Reg16msExpAmbient55inchFlatField.bin",
    "Reg16msExpPrecision55inchFlatField.bin",
    "ULP16msExpAmbient55inchFlatField.bin",
    "ULP16msExpPrecision55inchFlatField.bin",
    "hfr55inchFlatField.bin",
    "precision55inchFlatField.bin",
]


def percentile(x: np.ndarray, q: float) -> float:
    x = np.asarray(x, dtype=np.float64)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if x.size else float("nan")


def finite_stats(x: np.ndarray) -> dict[str, float]:
    x = np.asarray(x, dtype=np.float64)
    x = x[np.isfinite(x)]
    if not x.size:
        return {"n": 0, "median": float("nan"), "mean": float("nan"),
                "p05": float("nan"), "p95": float("nan"), "p99": float("nan"),
                "max_abs": float("nan")}
    return {
        "n": int(x.size), "median": float(np.median(x)), "mean": float(np.mean(x)),
        "p05": float(np.percentile(x, 5)), "p95": float(np.percentile(x, 95)),
        "p99": float(np.percentile(x, 99)), "max_abs": float(np.max(np.abs(x))),
    }


def load_capture(path: Path) -> tuple[dict[str, np.ndarray], dict[str, int | float]]:
    """Decode one capture through the same native TransformStage as the viewer."""
    decoder = StreamDecoder()
    stage = TransformStage(outputs=("depth", "reflectance", "confidence", "ambient", "zapc"))
    data = path.read_bytes()
    out: dict[str, list[np.ndarray]] = {k: [] for k in ("depth", "reflectance", "confidence", "ambient", "zapc")}
    for frame in decoder.feed(data):
        result = stage.feed(frame)
        if result is not None and frame.header.stream_id == StreamId.RAW_3DMD:
            for key, value in result[1].items():
                out[key].append(np.array(value, copy=True))
    arrays = {key: np.asarray(values) for key, values in out.items()}
    meta = {
        "bytes": len(data), "frames_decoded": decoder.frames_decoded,
        "crc_failures": decoder.crc_failures, "bytes_skipped": decoder.bytes_skipped,
        "raw_frames": int(arrays["depth"].shape[0]),
    }
    if not arrays["depth"].size:
        raise RuntimeError(f"{path}: no transformed RAW_3DMD frames")
    return arrays, meta


def fit_plane(depth: np.ndarray, valid: np.ndarray) -> dict[str, float]:
    """Fit z = ax + by + c to the temporal-median depth surface."""
    h, w = depth.shape
    deproj = Deprojector(w, h)
    grid, _ = deproj.grid(depth)
    xyz_mm = grid * 1000.0
    rows, cols = np.where(valid)
    if rows.size < 3:
        return {"intercept_mm": float("nan"), "tilt_x_mm_per_m": float("nan"),
                "tilt_y_mm_per_m": float("nan"), "rms_mm": float("nan"),
                "abs_p95_mm": float("nan"), "abs_max_mm": float("nan")}
    x = xyz_mm[rows, cols, 0]
    y = xyz_mm[rows, cols, 1]
    z = depth[rows, cols]
    coef, *_ = np.linalg.lstsq(np.column_stack((x, y, np.ones_like(x))), z, rcond=None)
    residual = z - np.column_stack((x, y, np.ones_like(x))) @ coef
    return {
        "intercept_mm": float(coef[2]),
        "tilt_x_mm_per_m": float(coef[0] * 1000.0),
        "tilt_y_mm_per_m": float(coef[1] * 1000.0),
        "rms_mm": float(np.sqrt(np.mean(residual * residual))),
        "abs_p95_mm": percentile(np.abs(residual), 95),
        "abs_max_mm": float(np.max(np.abs(residual))),
    }


def reflectance_residual(avg: np.ndarray) -> float:
    illum = _blur(avg, 2.5)
    good = np.isfinite(avg) & np.isfinite(illum) & (illum > 1e-6)
    return float(100.0 * np.std(avg[good] / illum[good] - 1.0)) if np.any(good) else float("nan")


def apply_flatfield_residual(ff: FlatField, frames: np.ndarray) -> float:
    corrected = np.asarray([ff.apply(frame) for frame in frames])
    return reflectance_residual(np.mean(corrected, axis=0))


def deprojection_comparison(depth: np.ndarray, zapc: np.ndarray) -> dict[str, object]:
    """Compare host linear deprojection with the transform's factory-calibrated ZAPC."""
    h, w = depth.shape[1:]
    deproj = Deprojector(w, h)
    sample = np.arange(0, depth.shape[0], max(1, depth.shape[0] // 50))
    rows_all: dict[str, list[float]] = {"dx": [], "dy": [], "dz": [], "xy": [], "angle_x": [], "angle_y": []}
    center: dict[str, list[float]] = {"dx": [], "dy": [], "xy": []}
    edge: dict[str, list[float]] = {"dx": [], "dy": [], "xy": []}
    rr, cc = np.indices((h, w))
    is_center = (np.abs(rr - (h - 1) / 2) < 8) & (np.abs(cc - (w - 1) / 2) < 10)
    is_edge = (rr < 5) | (rr >= h - 5) | (cc < 6) | (cc >= w - 6)
    for i in sample:
        host_grid, valid = deproj.grid(depth[i])
        host_mm = host_grid * 1000.0
        zpc = zapc[i]
        good = valid & np.all(np.isfinite(zpc[:, :, :3]), axis=2) & (zpc[:, :, 2] > 0)
        if not np.any(good):
            continue
        delta = zpc[:, :, :3] - host_mm
        dx, dy, dz = (delta[:, :, k][good] for k in range(3))
        rows_all["dx"].extend(dx.tolist()); rows_all["dy"].extend(dy.tolist()); rows_all["dz"].extend(dz.tolist())
        rows_all["xy"].extend(np.hypot(dx, dy).tolist())
        z = zpc[:, :, 2][good]
        rows_all["angle_x"].extend(np.rad2deg(np.arctan2(zpc[:, :, 0][good], z)) .tolist())
        rows_all["angle_y"].extend(np.rad2deg(np.arctan2(zpc[:, :, 1][good], z)) .tolist())
        xy = np.hypot(dx, dy)
        for mask, dst in ((is_center & good, center), (is_edge & good, edge)):
            dst["dx"].extend(delta[:, :, 0][mask].tolist())
            dst["dy"].extend(delta[:, :, 1][mask].tolist())
            dst["xy"].extend(xy[np.isin(np.flatnonzero(good), np.flatnonzero(mask))].tolist())

    # The ``xy`` indexing above is intentionally replaced by a direct masked
    # expression below; keeping it explicit avoids any dependence on flatten order.
    center["xy"] = []
    edge["xy"] = []
    for i in sample:
        host_grid, valid = deproj.grid(depth[i])
        zpc = zapc[i]
        good = valid & np.all(np.isfinite(zpc[:, :, :3]), axis=2) & (zpc[:, :, 2] > 0)
        xy = np.hypot(zpc[:, :, 0] - host_grid[:, :, 0] * 1000.0,
                      zpc[:, :, 1] - host_grid[:, :, 1] * 1000.0)
        center["xy"].extend(xy[good & is_center].tolist())
        edge["xy"].extend(xy[good & is_edge].tolist())

    def group(d: dict[str, list[float]]) -> dict[str, object]:
        return {key: finite_stats(np.asarray(value)) for key, value in d.items()}

    return {
        "sampled_frames": int(sample.size),
        "xyz_delta_mm": group({k: rows_all[k] for k in ("dx", "dy", "dz")} ),
        "xy_delta_all_mm": group({"xy": rows_all["xy"]}),
        "xy_delta_center_mm": group(center),
        "xy_delta_edge_mm": group(edge),
        "z_max_abs_mm": float(np.max(np.abs(rows_all["dz"]))) if rows_all["dz"] else float("nan"),
    }


def analyze(path: Path, out_dir: Path) -> dict[str, object]:
    arrays, decode_meta = load_capture(path)
    depth = arrays["depth"].astype(np.float64)
    refl = arrays["reflectance"].astype(np.float64)
    conf = arrays["confidence"].astype(np.float64)
    ambient = arrays["ambient"].astype(np.float64)
    zapc = arrays["zapc"].astype(np.float64)
    n, h, w = depth.shape
    settled = slice(min(WARMUP, max(0, n - 1)), n)
    z = depth[settled]
    r = refl[settled]
    c = conf[settled]
    a = ambient[settled]
    valid = np.isfinite(z) & (z > 0) & (z < 10000)
    z_valid = np.where(valid, z, np.nan)
    frame_median = np.nanmedian(z_valid, axis=(1, 2))
    zone_median = np.nanmedian(z_valid, axis=0)
    zone_std = np.nanstd(z_valid, axis=0)
    zone_good = np.isfinite(zone_median)
    plane = fit_plane(zone_median, zone_good)
    flat = build_flatfield(np.nan_to_num(r, nan=0.0), note=f"static 55in ceiling; {path.name}")
    ff_path = out_dir / f"flatfield_20260804_{path.stem}.npz"
    flat.save(ff_path)
    mid = max(1, r.shape[0] // 2)
    train_ff = build_flatfield(np.nan_to_num(r[:mid], nan=0.0), note=f"train half of {path.name}")
    raw_fpn = reflectance_residual(np.mean(r, axis=0))
    holdout_fpn_before = reflectance_residual(np.mean(r[mid:], axis=0))
    holdout_fpn_after = apply_flatfield_residual(train_ff, r[mid:])
    refl_mean_zone = np.nanmean(r, axis=0)
    refl_temporal_std = np.nanstd(r, axis=0)
    refl_frame_mean = np.nanmean(r, axis=(1, 2))
    refl_neighbor = np.diff(np.nanmean(r, axis=0), axis=1)
    conf_valid = c[np.isfinite(c)]
    valid_counts = valid.sum(axis=(1, 2))
    metrics: dict[str, object] = {
        "file": path.name,
        "target_distance_mm": TARGET_MM,
        "shape": [h, w],
        "warmup_frames_dropped": int(settled.start or 0),
        **decode_meta,
        "depth": {
            "valid_pct": float(100.0 * np.mean(valid)),
            "frame_median_mm": finite_stats(frame_median),
            "accuracy_global_median_mm": float(np.nanmedian(z_valid)),
            "accuracy_global_error_mm": float(np.nanmedian(z_valid) - TARGET_MM),
            "accuracy_global_error_pct": float(100.0 * (np.nanmedian(z_valid) - TARGET_MM) / TARGET_MM),
            "temporal_precision_zone_std_mm": finite_stats(zone_std[zone_good]),
            "temporal_precision_frame_median_std_mm": float(np.nanstd(frame_median)),
            "temporal_precision_frame_median_p95_abs_delta_mm": percentile(np.abs(np.diff(frame_median)), 95),
            "spatial_uniformity_zone_median_mm": finite_stats(zone_median[zone_good]),
            "spatial_uniformity_raw_p95_minus_p05_mm": float(np.nanpercentile(zone_median, 95) - np.nanpercentile(zone_median, 5)),
            "spatial_uniformity_plane_fit": plane,
            "valid_zones_per_frame": finite_stats(valid_counts),
        },
        "reflectance": {
            "frame_mean": finite_stats(refl_frame_mean),
            "mean_plane": finite_stats(refl_mean_zone),
            "raw_mean_image_cv_pct": float(100.0 * np.nanstd(refl_mean_zone) / np.maximum(np.nanmean(refl_mean_zone), 1e-6)),
            "raw_mean_image_p95_minus_p05_pct": float(100.0 * (np.nanpercentile(refl_mean_zone, 95) - np.nanpercentile(refl_mean_zone, 5)) / np.maximum(np.nanmean(refl_mean_zone), 1e-6)),
            "temporal_zone_std": finite_stats(refl_temporal_std),
            "temporal_zone_cv_pct_median": float(100.0 * np.nanmedian(refl_temporal_std / np.maximum(np.abs(refl_mean_zone), 1e-6))),
            "mean_image_neighbor_x_diff": finite_stats(refl_neighbor),
            "fpn_residual_pct_all_settled": raw_fpn,
            "holdout_fpn_before_pct": holdout_fpn_before,
            "holdout_fpn_after_pct": holdout_fpn_after,
            "holdout_fpn_reduction_pct": float(100.0 * (1.0 - holdout_fpn_after / holdout_fpn_before)) if holdout_fpn_before else float("nan"),
            "flatfield_gain": {"mean": float(flat.gain.mean()), "min": float(flat.gain.min()),
                               "max": float(flat.gain.max()), "std_pct": float(100.0 * flat.gain.std()),
                               "path": ff_path.name, "meta": flat.meta},
        },
        "confidence": {"finite_pct": float(100.0 * np.mean(np.isfinite(c))), **finite_stats(conf_valid)},
        "ambient": {"finite_pct": float(100.0 * np.mean(np.isfinite(a))), **finite_stats(a)},
        "deprojection": deprojection_comparison(z, zapc[settled]),
    }
    # Save compact zone maps for later inspection without retaining all frames.
    np.savez_compressed(out_dir / f"maps_20260804_{path.stem}.npz",
                        depth_median_mm=zone_median.astype(np.float32),
                        depth_std_mm=zone_std.astype(np.float32),
                        reflectance_mean=refl_mean_zone.astype(np.float32),
                        reflectance_temporal_std=refl_temporal_std.astype(np.float32),
                        flatfield_gain=flat.gain)
    return metrics


def main() -> int:
    out_dir = Path(__file__).resolve().parent
    all_metrics = []
    for name in CAPTURES:
        print(f"analyzing {name}", flush=True)
        all_metrics.append(analyze(out_dir / name, out_dir))
    # Compare maps across runs.  A high correlation means the spatial correction
    # is sensor-locked; a low correlation means the map is dominated by the
    # mode's illumination/response and should not be shared across modes.
    maps = {}
    mean_images = {}
    for m in all_metrics:
        with np.load(out_dir / f"maps_20260804_{Path(m['file']).stem}.npz") as d:
            mean_images[m["file"]] = np.asarray(d["reflectance_mean"], dtype=np.float64)
        with np.load(out_dir / m["reflectance"]["flatfield_gain"]["path"]) as d:
            maps[m["file"]] = np.asarray(d["gain"], dtype=np.float64)
    map_stack = np.stack(list(maps.values()))
    map_mean = np.mean(map_stack, axis=0)
    map_comparison = {
        "pairwise": {},
        "cross_corrected_fpn_pct": {},
        "median_map": {"min": float(np.min(map_mean)), "max": float(np.max(map_mean)),
                        "std_pct": float(100.0 * np.std(map_mean))},
    }
    names = list(maps)
    for i, left in enumerate(names):
        for right in names[i + 1:]:
            x, y = maps[left].ravel(), maps[right].ravel()
            corr = float(np.corrcoef(x, y)[0, 1])
            map_comparison["pairwise"][f"{left} vs {right}"] = {
                "corr": corr,
                "rms_gain_difference_pct": float(100.0 * np.sqrt(np.mean((x - y) ** 2))),
            }
    for target, image in mean_images.items():
        map_comparison["cross_corrected_fpn_pct"][target] = {}
        for reference, gain in maps.items():
            map_comparison["cross_corrected_fpn_pct"][target][reference] = reflectance_residual(image * gain)
    (out_dir / "analysis_20260804_metrics.json").write_text(
        json.dumps({"target_distance_mm": TARGET_MM, "warmup_frames": WARMUP,
                    "captures": all_metrics, "flatfield_map_comparison": map_comparison}, indent=2) + "\n")
    fields = ["file", "raw_frames", "depth_accuracy_global_error_mm", "depth_plane_rms_mm",
              "depth_zone_std_median_mm", "depth_zone_std_p95_mm", "depth_valid_pct",
              "refl_fpn_pct", "refl_holdout_after_pct", "refl_temporal_cv_pct",
              "deproj_xy_p95_mm", "deproj_xy_edge_p95_mm"]
    with (out_dir / "analysis_20260804_metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for m in all_metrics:
            d = m["depth"]
            r = m["reflectance"]
            dp = m["deprojection"]
            xyz = dp["xyz_delta_mm"]
            edge = dp["xy_delta_edge_mm"]["xy"]
            writer.writerow({
                "file": m["file"], "raw_frames": m["raw_frames"],
                "depth_accuracy_global_error_mm": d["accuracy_global_error_mm"],
                "depth_plane_rms_mm": d["spatial_uniformity_plane_fit"]["rms_mm"],
                "depth_zone_std_median_mm": d["temporal_precision_zone_std_mm"]["median"],
                "depth_zone_std_p95_mm": d["temporal_precision_zone_std_mm"]["p95"],
                "depth_valid_pct": d["valid_pct"], "refl_fpn_pct": r["fpn_residual_pct_all_settled"],
                "refl_holdout_after_pct": r["holdout_fpn_after_pct"],
                "refl_temporal_cv_pct": r["temporal_zone_cv_pct_median"],
                "deproj_xy_p95_mm": dp["xy_delta_all_mm"]["xy"]["p95"],
                "deproj_xy_edge_p95_mm": edge["p95"],
            })
    # Human-readable companion for the NPZ maps.  The data products remain the
    # NPZ/JSON files; this is only a quick visual index for the fixed pattern.
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        precision_name = "8msExpPrecision55inchFlatField.bin"
        ambient_name = "Reg16msExpAmbient55inchFlatField.bin"
        with (np.load(out_dir / f"maps_20260804_{Path(precision_name).stem}.npz") as pmap,
              np.load(out_dir / f"maps_20260804_{Path(ambient_name).stem}.npz") as amap):
            fig, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
            panels = [
                (pmap["reflectance_mean"], "Precision reflectance mean", "viridis"),
                (pmap["flatfield_gain"], "Precision gain map", "coolwarm"),
                (amap["reflectance_mean"], "Ambient reflectance mean", "viridis"),
                (amap["flatfield_gain"], "Ambient gain map", "coolwarm"),
                (pmap["depth_median_mm"] - np.nanmedian(pmap["depth_median_mm"]), "Depth residual (mm)", "coolwarm"),
                (pmap["depth_std_mm"], "Depth temporal σ (mm)", "magma"),
            ]
            for ax, (image, title, cmap) in zip(axes.ravel(), panels):
                im = ax.imshow(image, cmap=cmap, interpolation="nearest")
                ax.set_title(title, fontsize=10)
                ax.set_xlabel("zone column")
                ax.set_ylabel("zone row")
                fig.colorbar(im, ax=ax, shrink=0.82)
            fig.suptitle("2026-08-04 stationary 55 in ceiling: spatial maps", fontsize=13)
            fig.savefig(out_dir / "analysis_20260804_flatfield_maps.png", dpi=160)
            plt.close(fig)
    except Exception as exc:  # plotting is optional; numeric artifacts must survive
        (out_dir / "analysis_20260804_plot_error.txt").write_text(f"{type(exc).__name__}: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
