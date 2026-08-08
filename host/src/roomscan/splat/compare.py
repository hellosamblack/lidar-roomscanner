"""Compare a lidar SLAM reconstruction against an external ground-truth splat.

The *reverse* of Phase 7's usual direction. Normally the phone-video splat is the
thing we improve using the ToF lidar; here we turn it around and treat a metric
Scaniverse Gaussian splat as **ground truth** to expose where our SLAM got the
ROOM SHAPE wrong -- e.g. BUG-084's map fork on ``officeFullScanAug6``, where the
ceiling drops mid-scan and the reconstruction forks a second, displaced room.

Both inputs are ``.ply``:
  * our scan  -- the Detailed-SLAM triangle mesh ``results/<stem>.ply`` (Open3D,
    metric meters, Open3D-CV world where up = -Y).
  * reference -- an INRIA-3DGS splat (``x,y,z`` + SH + opacity + scale + rot),
    metric scale, arbitrary (here upside-down) frame, with snowglobe floaters.

Because BOTH are metric, we align **rigidly** (no scale): a scale/extent mismatch
is then a *finding*, not something ICP fits away. The final ICP fitness/RMSE are
reported honestly -- a warped scan will not rigidly fit the truth, and that
residual is exactly the signal we are after.

Everything here is torch-free (open3d + plyfile + Pillow), so it is safe to run
without the heavy ``[splat]`` build extra installed.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import open3d as o3d
from plyfile import PlyData

# SH band-0 -> RGB DC term (INRIA 3DGS convention).
_SH_C0 = 0.28209479177387814

# Open3D-CV world: gravity points +Y (down), so "up" is -Y. Height = -y.
_UP_AXIS = 1
_HORIZ_AXES = (0, 2)  # X, Z -- the floor plane

# Tokens that carry no room/scene meaning, so they never count as a name match
# between a capture and a reference splat.
_STOPWORDS = frozenset({"scan", "full", "capture", "splat", "scaniverse", "map",
                        "room", "test", "final", "new", "the", "mnt", "nomnt",
                        "debug", "verify", "aug", "jul", "live", "ref"})


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #
def _finalize_cloud(pts: np.ndarray, colors: np.ndarray | None, voxel: float,
                    *, denoise: bool) -> o3d.geometry.PointCloud:
    """Numpy points (+ optional [0,1] colors) -> a downsampled Open3D cloud."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts, dtype=np.float64))
    if colors is not None and len(colors) == len(pts):
        pcd.colors = o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0).astype(np.float64))
    if voxel > 0:
        pcd = pcd.voxel_down_sample(voxel)
    if denoise and len(pcd.points) > 40:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return pcd


def load_reference_cloud(path: str | Path, *, opacity_min: float = 0.5,
                         voxel: float = 0.03) -> o3d.geometry.PointCloud:
    """INRIA-3DGS splat ``.ply`` -> an opacity-filtered, colored point cloud.

    Gaussians below ``sigmoid(opacity) >= opacity_min`` are dropped -- that is
    what removes the low-opacity "snowglobe" floaters that would otherwise
    dominate the room's apparent extent. Colors come from the band-0 SH DC term.
    """
    v = PlyData.read(str(path))["vertex"].data
    xyz = np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float64)
    keep = np.ones(len(xyz), dtype=bool)
    if "opacity" in v.dtype.names:
        alpha = 1.0 / (1.0 + np.exp(-v["opacity"].astype(np.float64)))
        keep = alpha >= opacity_min
    colors = None
    if all(c in v.dtype.names for c in ("f_dc_0", "f_dc_1", "f_dc_2")):
        f_dc = np.stack([v["f_dc_0"], v["f_dc_1"], v["f_dc_2"]], axis=1).astype(np.float64)
        colors = _SH_C0 * f_dc + 0.5
    return _finalize_cloud(xyz[keep], None if colors is None else colors[keep],
                           voxel, denoise=True)


def load_scan_cloud(path: str | Path, *, voxel: float = 0.03) -> o3d.geometry.PointCloud:
    """Our SLAM triangle-mesh ``.ply`` -> a colored point cloud (its vertices)."""
    mesh = o3d.io.read_triangle_mesh(str(path))
    pts = np.asarray(mesh.vertices, dtype=np.float64)
    if len(pts) == 0:
        raise ValueError(f"scan mesh has no vertices: {path}")
    colors = np.asarray(mesh.vertex_colors) if mesh.has_vertex_colors() else None
    return _finalize_cloud(pts, colors, voxel, denoise=False)


# --------------------------------------------------------------------------- #
# Alignment (rigid; scale mismatch is a finding, not a fit)
# --------------------------------------------------------------------------- #
def _with_normals(pcd: o3d.geometry.PointCloud, voxel: float) -> o3d.geometry.PointCloud:
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 3, max_nn=30))
    return pcd


def _flip_init(source: o3d.geometry.PointCloud,
               target: o3d.geometry.PointCloud) -> np.ndarray:
    """Coarse ``source -> target`` seed: undo the known upside-down orientation
    (180 deg about X, a proper rotation) then match centroids. This is the
    fallback when feature-based global registration fails to lock on."""
    flip = np.eye(4)
    flip[:3, :3] = np.diag([1.0, -1.0, -1.0])
    sc = np.asarray(source.get_center())
    tc = np.asarray(target.get_center())
    flip[:3, 3] = tc - flip[:3, :3] @ sc
    return flip


def _global_init(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud,
                 voxel: float) -> np.ndarray | None:
    """FPFH + RANSAC global registration ``source -> target`` (rotation-invariant,
    so it does not need the flip seed). Returns None if it throws or barely fits."""
    feat_radius = o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100)
    try:
        sf = o3d.pipelines.registration.compute_fpfh_feature(source, feat_radius)
        tf = o3d.pipelines.registration.compute_fpfh_feature(target, feat_radius)
        dist = voxel * 1.5
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source, target, sf, tf, True, dist,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
            [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
             o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist)],
            o3d.pipelines.registration.RANSACConvergenceCriteria(200000, 0.999))
    except (RuntimeError, ValueError):
        return None
    return np.asarray(result.transformation) if result.fitness > 0.05 else None


def _refine_icp(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud,
                init: np.ndarray, voxel: float, *, allow_scale: bool):
    """Multi-scale ICP refine from ``init``. Point-to-plane by default; when
    ``allow_scale`` is set, point-to-point *with scaling* purely as a diagnostic."""
    if allow_scale:
        est = o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=True)
    else:
        est = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    result = None
    T = np.asarray(init, dtype=np.float64)
    for scale in (8.0, 4.0, 2.0):
        result = o3d.pipelines.registration.registration_icp(
            source, target, voxel * scale, T, est,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
        T = np.asarray(result.transformation)
    return result, T


def _scale_of(T: np.ndarray) -> float:
    return float(np.cbrt(max(np.linalg.det(T[:3, :3]), 1e-12)))


def _crop_and_refine(scan: o3d.geometry.PointCloud, ref_aligned: o3d.geometry.PointCloud,
                     voxel: float, *, margin: float = 0.5):
    """Crop the aligned reference to the scan's bounding box (+margin) so far-field
    background does not dominate, then run one more ICP on that room subset for a
    tighter, room-only fit. Returns (cropped+refined ref, {fitness, inlier_rmse})."""
    aabb = scan.get_axis_aligned_bounding_box()
    ext = np.asarray(aabb.get_extent()) / 2 + margin
    c = np.asarray(aabb.get_center())
    room = o3d.geometry.AxisAlignedBoundingBox(c - ext, c + ext)
    ref_room = ref_aligned.crop(room)
    if len(ref_room.points) < 100:
        return ref_room, {"fitness": 0.0, "inlier_rmse": 0.0}
    src = _with_normals(ref_room, voxel)
    tgt = _with_normals(o3d.geometry.PointCloud(scan), voxel)
    result = o3d.pipelines.registration.registration_icp(
        src, tgt, voxel * 4, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    ref_room.transform(result.transformation)
    return ref_room, {"fitness": float(result.fitness), "inlier_rmse": float(result.inlier_rmse)}


def align_reference_to_scan(scan: o3d.geometry.PointCloud, ref: o3d.geometry.PointCloud,
                            *, voxel: float = 0.03, allow_scale: bool = False,
                            use_global: bool = True) -> dict:
    """Register the reference into the scan's frame. Tries global (FPFH+RANSAC)
    and flip-seeded initialisation, refines each with ICP, and keeps whichever
    ICP fits better. Returns the transform + honest fitness/rmse.
    """
    src = _with_normals(o3d.geometry.PointCloud(ref), voxel)
    tgt = _with_normals(o3d.geometry.PointCloud(scan), voxel)

    candidates: list[tuple[str, np.ndarray]] = [("flip", _flip_init(src, tgt))]
    if use_global:
        g = _global_init(src, tgt, voxel)
        if g is not None:
            candidates.insert(0, ("global", g))

    best = None
    for label, init in candidates:
        result, T = _refine_icp(src, tgt, init, voxel, allow_scale=allow_scale)
        if best is None or result.fitness > best["fitness"]:
            best = {"init": label, "transform": T,
                    "fitness": float(result.fitness),
                    "inlier_rmse": float(result.inlier_rmse)}
    best["scale"] = _scale_of(best["transform"]) if allow_scale else 1.0
    return best


# --------------------------------------------------------------------------- #
# Shape-error metrics
# --------------------------------------------------------------------------- #
def _extent(pcd: o3d.geometry.PointCloud) -> dict:
    aabb = np.asarray(pcd.get_axis_aligned_bounding_box().get_extent())
    try:
        obb = np.asarray(pcd.get_oriented_bounding_box().extent)
    except RuntimeError:
        obb = aabb
    return {"aabb": [round(float(x), 3) for x in aabb],
            "obb": [round(float(x), 3) for x in sorted(obb, reverse=True)]}


def _footprint_area_m2(pcd: o3d.geometry.PointCloud, cell: float = 0.05) -> float:
    """Occupied floor area: project onto the horizontal plane, count occupied
    ``cell``-sized bins. A coarse but honest proxy for room footprint."""
    p = np.asarray(pcd.points)
    if len(p) == 0:
        return 0.0
    hz = p[:, list(_HORIZ_AXES)]
    idx = np.floor((hz - hz.min(axis=0)) / cell).astype(np.int64)
    occupied = np.unique(idx, axis=0).shape[0]
    return round(occupied * cell * cell, 3)


def _dist_stats(a: o3d.geometry.PointCloud, b: o3d.geometry.PointCloud) -> tuple[np.ndarray, dict]:
    """Nearest-neighbour distance from every point of ``a`` to ``b`` (meters)."""
    d = np.asarray(a.compute_point_cloud_distance(b))
    if len(d) == 0:
        return d, {}
    return d, {"mean": round(float(d.mean()), 4), "median": round(float(np.median(d)), 4),
               "p95": round(float(np.percentile(d, 95)), 4), "max": round(float(d.max()), 4),
               "frac_over_10cm": round(float((d > 0.10).mean()), 4)}


def _heights(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    return -np.asarray(pcd.points)[:, _UP_AXIS]  # up = -Y, so height = -y


def _count_modes(heights: np.ndarray, *, bin_m: float = 0.05, smooth_m: float = 0.15,
                 min_sep_m: float = 0.5, rel_thresh: float = 0.30) -> list[float]:
    """Dominant horizontal-surface heights: prominent peaks in the (smoothed)
    height histogram. Floor + ceiling give two; BUG-084's dropped ceiling adds a
    third. Smoothing + a meter-scale neighborhood + a 30%-of-peak floor keep real
    surfaces and reject the histogram noise a raw scan produces."""
    if len(heights) < 200:
        return []
    lo, hi = float(np.percentile(heights, 0.5)), float(np.percentile(heights, 99.5))
    if hi - lo < bin_m:
        return [round((lo + hi) / 2, 3)]
    nbins = max(4, int(np.ceil((hi - lo) / bin_m)))
    hist, edges = np.histogram(np.clip(heights, lo, hi), bins=nbins)
    centers = (edges[:-1] + edges[1:]) / 2
    k = max(1, int(round(smooth_m / bin_m)))                 # boxcar smooth
    hist = np.convolve(hist.astype(float), np.ones(2 * k + 1) / (2 * k + 1), mode="same")
    thresh = hist.max() * rel_thresh
    win = max(1, int(round(min_sep_m / bin_m)))              # local-max neighborhood
    peaks: list[tuple[float, float]] = []
    for i in range(nbins):
        a, b = max(0, i - win), min(nbins, i + win + 1)
        if hist[i] >= thresh and hist[i] == hist[a:b].max():
            peaks.append((float(centers[i]), float(hist[i])))
    peaks.sort(key=lambda t: -t[1])                          # merge, keep the taller
    kept: list[float] = []
    for c, _ in peaks:
        if all(abs(c - k2) >= min_sep_m for k2 in kept):
            kept.append(c)
    return [round(c, 3) for c in sorted(kept)]


def _vertical_report(scan: o3d.geometry.PointCloud, ref: o3d.geometry.PointCloud) -> dict:
    """Vertical structure of scan vs truth. A healthy room already has two height
    peaks (floor + ceiling); the BUG-084 fork shows up as the scan carrying MORE
    vertical structure than the truth -- an extra (dropped-ceiling) mode and/or a
    stretched vertical extent -- not merely as "two modes"."""
    hs, hr = _heights(scan), _heights(ref)
    scan_ext = float(hs.max() - hs.min()) if len(hs) else 0.0
    ref_ext = float(hr.max() - hr.min()) if len(hr) else 0.0
    scan_modes = _count_modes(hs)
    ref_modes = _count_modes(hr)
    ratio = round(scan_ext / ref_ext, 3) if ref_ext > 1e-6 else None
    fork = len(scan_modes) > len(ref_modes) or (ratio is not None and ratio > 1.3)
    return {"scan_extent_m": round(scan_ext, 3), "ref_extent_m": round(ref_ext, 3),
            "extent_ratio": ratio,
            "scan_height_modes": scan_modes, "ref_height_modes": ref_modes,
            "fork_suspected": bool(fork)}


# --------------------------------------------------------------------------- #
# Artifacts (headless: PLY + Pillow PNG, never a GPU screenshot)
# --------------------------------------------------------------------------- #
def _turbo(t: np.ndarray) -> np.ndarray:
    """Cheap perceptual-ish blue->cyan->yellow->red ramp for the error heatmap.
    (Not matplotlib's turbo; a 4-stop piecewise lerp -- enough to read a heatmap.)"""
    stops = np.array([[0.19, 0.07, 0.23], [0.10, 0.60, 0.90],
                      [0.95, 0.85, 0.15], [0.85, 0.12, 0.12]])
    t = np.clip(t, 0, 1) * (len(stops) - 1)
    lo = np.floor(t).astype(int)
    hi = np.minimum(lo + 1, len(stops) - 1)
    f = (t - lo)[:, None]
    return stops[lo] * (1 - f) + stops[hi] * f


def _write_overlay_ply(path: Path, ref: o3d.geometry.PointCloud,
                       scan: o3d.geometry.PointCloud) -> None:
    ref = o3d.geometry.PointCloud(ref)
    scan = o3d.geometry.PointCloud(scan)
    ref.paint_uniform_color([0.20, 0.45, 0.95])   # ground truth = blue
    scan.paint_uniform_color([0.95, 0.25, 0.20])  # our scan = red
    o3d.io.write_point_cloud(str(path), ref + scan)


def _write_heatmap_ply(path: Path, scan: o3d.geometry.PointCloud,
                       dist: np.ndarray, vmax: float) -> None:
    scan = o3d.geometry.PointCloud(scan)
    scan.colors = o3d.utility.Vector3dVector(_turbo(dist / max(vmax, 1e-6)))
    o3d.io.write_point_cloud(str(path), scan)


def _scatter_png(path: Path, ref_xy: np.ndarray, scan_xy: np.ndarray,
                 *, px: int = 1100, flip_y: bool = False) -> None:
    """Composite floor-plan / elevation: GT-only = blue, scan-only = red,
    agreement = light. Reveals shape the scan *added* (red) vs *missed* (blue)."""
    from PIL import Image

    both = np.vstack([ref_xy, scan_xy]) if len(ref_xy) and len(scan_xy) else (
        ref_xy if len(ref_xy) else scan_xy)
    lo, hi = both.min(axis=0), both.max(axis=0)
    span = np.maximum(hi - lo, 1e-6)
    pad = 0.04 * span
    lo, hi = lo - pad, hi + pad
    span = hi - lo
    aspect = span[1] / span[0]
    w, h = px, max(64, int(px * aspect))

    def to_px(xy: np.ndarray) -> np.ndarray:
        u = (xy - lo) / span
        col = np.clip((u[:, 0] * (w - 1)).astype(int), 0, w - 1)
        row = np.clip((u[:, 1] * (h - 1)).astype(int), 0, h - 1)
        if flip_y:
            row = (h - 1) - row
        return np.stack([row, col], axis=1)

    def mask(xy: np.ndarray) -> np.ndarray:
        m = np.zeros((h, w), dtype=bool)
        if len(xy):
            rc = to_px(xy)
            m[rc[:, 0], rc[:, 1]] = True
        return m

    mr, ms = mask(ref_xy), mask(scan_xy)
    agree = mr & ms
    canvas = np.full((h, w, 3), (12, 14, 18), dtype=np.uint8)
    canvas[mr & ~agree] = (52, 116, 235)    # GT only -> room the scan missed
    canvas[ms & ~agree] = (235, 64, 52)     # scan only -> shape the scan invented
    canvas[agree] = (228, 230, 235)         # matched
    Image.fromarray(canvas, "RGB").save(str(path))


def _horiz(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    return np.asarray(pcd.points)[:, list(_HORIZ_AXES)]


def _elevation(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    p = np.asarray(pcd.points)
    return np.stack([p[:, _HORIZ_AXES[0]], -p[:, _UP_AXIS]], axis=1)  # (X, height)


# --------------------------------------------------------------------------- #
# Orchestration
# --------------------------------------------------------------------------- #
def _scan_ply_for(capture: str | Path, results_dir: Path) -> Path:
    """The Detailed-SLAM mesh for a capture. Mirrors
    ``roomscan.slam.detailed.sidecar_paths(...)['ply']`` (``results/<stem>.ply``)
    without importing that package -- ``slam/__init__`` pulls the whole Open3D
    Mapper, which this torch-free tool has no need for just to name a file."""
    return results_dir / f"{Path(capture).stem}.ply"


def _tokens(name: str) -> set[str]:
    """Scene tokens of a name: alphabetic words >=3 chars, minus stopwords, with
    camelCase split first so ``officeFullScanAug6`` -> {office} and ``Sam Office``
    -> {sam, office}."""
    import re
    split = re.sub(r"([a-z])([A-Z])", r"\1 \2", str(name))   # camelCase -> spaces
    words = re.findall(r"[a-z]+", split.lower())
    return {w for w in words if len(w) >= 3 and w not in _STOPWORDS}


def choose_reference(capture: str | Path, results_dir: str | Path) -> dict:
    """Pick the best *imported* ground-truth splat to diagnose a capture against.

    Ground truth = imported (external, e.g. Scaniverse) splats only -- our own
    video builds are rough (photometry-only) and are NOT truth. Among the imported
    splats, the one whose display name/slug shares the most scene tokens with the
    capture wins (``officeFullScanAug6`` -> "Sam Office"); with no name overlap the
    most recent import is used as a last resort. Returns
    ``{"ply", "chosen", "reason", "candidates"}`` (``ply`` is None if none exist).
    """
    from .sidecar import _resolve_ply, list_splats, splats_root
    refs = [s for s in list_splats(results_dir) if s.get("imported")]
    if not refs:
        return {"ply": None, "chosen": None,
                "reason": "no imported ground-truth splat under results/splats/ "
                          "(drop a Scaniverse .ply in a subdir to add one)",
                "candidates": []}
    cap = _tokens(capture if isinstance(capture, str) else Path(capture).stem)
    scored = sorted(
        ((len(cap & (_tokens(s["name"]) | _tokens(s["slug"]))), -s.get("mtime", 0), s)
         for s in refs), key=lambda t: (-t[0], t[1]))
    top_score, _, chosen = scored[0]
    ply = _resolve_ply(splats_root(results_dir) / chosen["slug"])
    reason = (f"name match on {sorted(cap & (_tokens(chosen['name']) | _tokens(chosen['slug'])))}"
              if top_score else "no name overlap; newest imported reference")
    return {"ply": ply, "chosen": chosen["slug"], "reason": reason,
            "candidates": [{"slug": s["slug"], "name": s["name"], "score": sc}
                           for sc, _, s in scored]}


def _resolve_reference(reference: str | Path | None, capture: str | Path,
                       results_dir: Path) -> tuple[Path | None, dict]:
    """Resolve an explicit reference, or auto-select the best one for ``capture``.
    Returns (ply, selection)."""
    if not reference:
        sel = choose_reference(capture, results_dir)
        return sel["ply"], sel
    ref = Path(reference)
    if ref.is_file():
        return ref, {"chosen": ref.stem, "reason": "explicit path"}
    from .sidecar import _resolve_ply, splats_root
    cand = _resolve_ply(splats_root(results_dir) / str(reference))
    if cand is not None:
        return cand, {"chosen": str(reference), "reason": "explicit slug"}
    return ref, {"chosen": str(reference), "reason": "explicit (not found)"}


def compare_scan_to_reference(capture: str | Path, reference: str | Path | None = None, *,
                              results_dir: str | Path = "results", opacity_min: float = 0.5,
                              voxel: float = 0.03, allow_scale: bool = False,
                              write_artifacts: bool = True) -> dict:
    """Align our SLAM reconstruction of ``capture`` to a ground-truth splat and
    quantify where the room shape diverges.

    With no ``reference``, the best *imported* ground-truth splat for this capture is
    auto-selected (see ``choose_reference`` -- name match, not hardcoded to one splat);
    pass an explicit ``.ply`` path or a ``results/splats/`` slug to override. Returns a
    report dict (the reference selection, alignment quality, per-axis extents + ratio,
    floor footprint, bidirectional cloud-to-cloud distance, vertical/ceiling-fork
    analysis) and, unless ``write_artifacts`` is False, writes overlay/heatmap PLYs and
    floor-plan + elevation PNGs under ``results/compare/<stem>__vs__<ref>/``.
    """
    results_dir = Path(results_dir)
    scan_ply = _scan_ply_for(capture, results_dir)
    if not scan_ply.is_file():
        return {"ok": False, "error": f"no SLAM reconstruction for {Path(capture).stem} "
                f"(expected {scan_ply}); build it with Detailed SLAM / roomscan-slam first"}
    ref_ply, selection = _resolve_reference(reference, capture, results_dir)
    if ref_ply is None or not ref_ply.is_file():
        return {"ok": False, "error": selection.get("reason", f"reference not found: {ref_ply}"),
                "reference_selection": selection}

    scan = load_scan_cloud(scan_ply, voxel=voxel)
    ref = load_reference_cloud(ref_ply, opacity_min=opacity_min, voxel=voxel)
    if len(scan.points) < 100 or len(ref.points) < 100:
        return {"ok": False, "error": f"too few points after filtering "
                f"(scan={len(scan.points)}, ref={len(ref.points)}); lower opacity_min or voxel"}

    align = align_reference_to_scan(scan, ref, voxel=voxel, allow_scale=allow_scale)
    ref_aligned = o3d.geometry.PointCloud(ref).transform(align["transform"])

    # A room-capture splat (Scaniverse) carries far-field background -- windows, the
    # world outside, stray high-opacity floaters -- tens of meters past the room. That
    # is not shape our lidar was trying to reconstruct, and it would dominate every
    # extent/distance metric, so crop the aligned reference to the room (the scan's
    # neighborhood) and refine the fit there. `frac_kept` reports how much was outside.
    ref_full_n = len(ref_aligned.points)
    ref_aligned, refined = _crop_and_refine(scan, ref_aligned, voxel)
    if len(ref_aligned.points) < 100:
        return {"ok": False, "error": "reference had <100 points inside the scan's "
                "footprint after alignment; the fit likely failed (check the overlay.ply)"}
    align = {"init": align["init"], "coarse_fitness": align["fitness"],
             "fitness": refined["fitness"], "inlier_rmse": refined["inlier_rmse"],
             "scale": align["scale"],
             "note": "rigid fit, cropped to the room; low fitness/high rmse is the shape "
                     "error, not a bug"}

    d_scan, scan_stats = _dist_stats(scan, ref_aligned)   # our error vs truth
    _, ref_stats = _dist_stats(ref_aligned, scan)         # truth the scan missed

    scan_ext, ref_ext = _extent(scan), _extent(ref_aligned)
    ratio = [round(s / r, 3) if r > 1e-6 else None
             for s, r in zip(scan_ext["obb"], ref_ext["obb"])]

    report = {
        "ok": True,
        "capture": Path(capture).stem,
        "scan_ply": str(scan_ply),
        "reference_ply": str(ref_ply),
        "reference_selection": selection,
        "points": {"scan": len(scan.points), "reference_total": ref_full_n,
                   "reference_in_room": len(ref_aligned.points),
                   "reference_frac_in_room": round(len(ref_aligned.points) / max(ref_full_n, 1), 3)},
        "voxel_m": voxel,
        "alignment": align,   # init, coarse_fitness, fitness (room), inlier_rmse, scale, note
        "extent_obb_m": {"scan": scan_ext["obb"], "reference": ref_ext["obb"],
                         "ratio_scan_over_ref": ratio},
        "footprint_m2": {"scan": _footprint_area_m2(scan),
                         "reference": _footprint_area_m2(ref_aligned)},
        "distance_m": {"scan_to_reference": scan_stats, "reference_to_scan": ref_stats},
        "vertical": _vertical_report(scan, ref_aligned),
    }

    if write_artifacts:
        out_dir = results_dir / "compare" / f"{Path(capture).stem}__vs__{ref_ply.parent.name}"
        out_dir.mkdir(parents=True, exist_ok=True)
        vmax = float(np.percentile(d_scan, 95)) if len(d_scan) else 0.1
        _write_overlay_ply(out_dir / "overlay.ply", ref_aligned, scan)
        _write_heatmap_ply(out_dir / "error_heatmap.ply", scan, d_scan, vmax)
        _scatter_png(out_dir / "floorplan.png", _horiz(ref_aligned), _horiz(scan))
        _scatter_png(out_dir / "elevation.png", _elevation(ref_aligned), _elevation(scan),
                     flip_y=True)
        (out_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n",
                                              encoding="utf-8")
        report["artifacts"] = {name: str(out_dir / name) for name in
                               ("overlay.ply", "error_heatmap.ply", "floorplan.png",
                                "elevation.png", "report.json")}
    return report
