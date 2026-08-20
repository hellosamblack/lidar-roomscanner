"""Trajectory / timing metrics, TUM export, and an optional KISS-ICP benchmark
for offline validation. compare_kiss imports kiss_icp lazily so it's optional."""
from __future__ import annotations

import numpy as np

from .frames import baro_height_m, world_up

_BUDGET_MS = 35.0


def trajectory_stats(poses: list[np.ndarray]) -> dict:
    t = np.array([p[:3, 3] for p in poses]) if poses else np.zeros((0, 3))
    if len(t) < 2:
        return {"n": len(t), "path_length_m": 0.0, "start_end_gap_m": 0.0,
                "horizontal_gap_m": 0.0, "vertical_gap_m": 0.0,
                "max_step_m": 0.0}
    steps = np.linalg.norm(np.diff(t, axis=0), axis=1)
    delta = t[-1] - t[0]
    vertical = float(delta @ world_up())
    horizontal = delta - vertical * world_up()
    return {"n": len(t), "path_length_m": float(steps.sum()),
            "start_end_gap_m": float(np.linalg.norm(delta)),
            "horizontal_gap_m": float(np.linalg.norm(horizontal)),
            "vertical_gap_m": vertical,
            "max_step_m": float(steps.max())}


def baro_divergence_stats(
    poses: list[np.ndarray],
    pressures: list[float],
    timestamps: list[float] | np.ndarray,
    *,
    ref_frames: int = 90,
    smooth_s: float = 10.0,
    threshold_m: float = 1.5,
    sustain_s: float = 10.0,
) -> dict:
    """Detect sustained vertical disagreement between barometer and trajectory.

    This is a warning, not a pose correction.  The barometric height is averaged
    over a trailing time window and compared with translation along the rig's
    actual world-up axis.  A run is flagged only when the absolute disagreement
    stays above ``threshold_m`` continuously for ``sustain_s``.

    The 1.5 m / 10 s defaults were selected against the failing AshOffice run
    and six healthy/null captures for issue #187.  They flag AshOffice near
    t=120 s without firing on those controls.  ``first_trigger_s`` is the start
    of the sustained excursion, relative to the first usable sample.
    """
    base = {
        "available": False,
        "diverged": False,
        "samples": 0,
        "threshold_m": float(threshold_m),
        "sustain_s": float(sustain_s),
        "smooth_s": float(smooth_s),
        "peak_abs_m": None,
        "first_trigger_s": None,
        "end_signed_m": None,
    }
    if ref_frames < 1 or smooth_s < 0 or threshold_m <= 0 or sustain_s < 0:
        raise ValueError("invalid barometer-divergence detector parameters")

    n = min(len(poses), len(pressures), len(timestamps))
    if n == 0:
        return base
    try:
        positions = np.asarray([pose[:3, 3] for pose in poses[:n]], dtype=np.float64)
        pressure = np.asarray(pressures[:n], dtype=np.float64)
        time_s = np.asarray(timestamps[:n], dtype=np.float64)
    except (TypeError, ValueError, IndexError):
        return base
    if positions.shape != (n, 3):
        return base

    valid = (np.isfinite(positions).all(axis=1) & np.isfinite(pressure) &
             (pressure > 0.0) & np.isfinite(time_s))
    positions, pressure, time_s = positions[valid], pressure[valid], time_s[valid]
    base["samples"] = int(len(time_s))
    if len(time_s) < ref_frames or np.any(np.diff(time_s) < 0):
        return base

    ref_pa = float(np.mean(pressure[:ref_frames]))
    if not np.isfinite(ref_pa) or ref_pa <= 0:
        return base
    base["available"] = True

    trajectory_height = (positions - positions[0]) @ world_up()
    baro_height = np.asarray(baro_height_m(pressure, ref_pa), dtype=np.float64)
    smoothed = np.empty_like(baro_height)
    left = 0
    rolling_sum = 0.0
    for right, value in enumerate(baro_height):
        rolling_sum += float(value)
        while time_s[left] < time_s[right] - smooth_s:
            rolling_sum -= float(baro_height[left])
            left += 1
        smoothed[right] = rolling_sum / (right - left + 1)

    disagreement = smoothed - trajectory_height
    evaluate = time_s - time_s[0] >= smooth_s
    evaluated = disagreement[evaluate]
    if evaluated.size == 0:
        return base
    base["peak_abs_m"] = float(np.max(np.abs(evaluated)))
    base["end_signed_m"] = float(evaluated[-1])

    above_since = None
    for t, value, ready in zip(time_s, disagreement, evaluate):
        if not ready:
            continue
        if abs(float(value)) > threshold_m:
            if above_since is None:
                above_since = float(t)
            if float(t) - above_since >= sustain_s:
                base["diverged"] = True
                base["first_trigger_s"] = above_since - float(time_s[0])
                break
        else:
            above_since = None
    return base


def footprint_area_m2(points, cell_m: float = 0.1, up_axis: int = 1) -> float:
    """Floor area covered by a reconstruction: the up axis is dropped, the other
    two coordinates are binned onto a `cell_m` grid, and the occupied cells are
    counted x cell area.

    **A floor-projected footprint, not `mesh.get_surface_area()`.** "Area
    covered" is the floor swept. `get_surface_area()` sums walls, ceiling, and
    both faces of every noisy sliver and TSDF speckle -- so a 2 m corridor with
    tall walls would outscore a large open room, and the number would grow with
    mesh density rather than with coverage.

    `up_axis` defaults to 1 because the Open3D CV world is **Y-down**: axis 1 is
    the vertical one (`roomscan.slam.mapper.world_up()` is -Y), so dropping it
    leaves the X/Z ground plane.

    Quantized deliberately: a 0.1 m cell means a single stray vertex 5 m off the
    map adds 0.01 m2, not a convex hull's worth. Returns 0.0 for an empty or
    all-non-finite input rather than raising -- it feeds a UI tile.
    """
    p = np.asarray(points, dtype=np.float64)
    if p.ndim != 2 or p.shape[0] == 0 or p.shape[1] < 3:
        return 0.0
    if not (0 <= up_axis < 3):
        raise ValueError(f"up_axis must be 0, 1 or 2; got {up_axis}")
    if cell_m <= 0:
        raise ValueError(f"cell_m must be > 0; got {cell_m}")
    ground = p[:, [a for a in range(3) if a != up_axis]]
    ground = ground[np.isfinite(ground).all(axis=1)]
    if ground.shape[0] == 0:
        return 0.0
    cells = np.floor(ground / cell_m).astype(np.int64)
    n_cells = len(np.unique(cells, axis=0))
    return float(n_cells * cell_m * cell_m)


def tracking_stats(lost_flags: list[bool]) -> dict:
    """Summarize tracking loss, separating "a few dropped frames" from "the run
    died and never recovered".

    `trailing_lost` is the headline. Once a frame is lost, `predict_pose`
    freezes translation at t_prev and nothing relocalizes, so an unbroken lost
    streak running to the last frame means that whole tail is a frozen,
    fabricated pose -- the reported trajectory and its start/end gap are
    meaningless over it. captures/coffeeRoomCircuitMnt.bin failed exactly this
    way (423 trailing frames, 22% of the capture) while still reporting a
    plausible-looking 2.05 m "drift", which is why the count alone is not enough.
    """
    n = len(lost_flags)
    if n == 0:
        return {"n": 0, "lost": 0, "lost_frac": 0.0, "trailing_lost": 0,
                "longest_lost_run": 0, "died": False}
    trailing = 0
    for f in reversed(lost_flags):
        if not f:
            break
        trailing += 1
    longest = cur = 0
    for f in lost_flags:
        cur = cur + 1 if f else 0
        longest = max(longest, cur)
    return {"n": n, "lost": int(sum(lost_flags)),
            "lost_frac": float(sum(lost_flags)) / n,
            "trailing_lost": trailing, "longest_lost_run": longest,
            # A handful of trailing lost frames is a normal end-of-scan tail;
            # a sustained one means the run never recovered.
            "died": bool(trailing >= 30)}


def timing_stats(ms: list[float]) -> dict:
    a = np.asarray(ms, dtype=np.float64)
    if a.size == 0:
        return {"n": 0, "median_ms": 0.0, "p90_ms": 0.0, "p99_ms": 0.0,
                "max_ms": 0.0, "over_budget_frac": 0.0}
    return {"n": int(a.size), "median_ms": float(np.median(a)),
            "p90_ms": float(np.percentile(a, 90)), "p99_ms": float(np.percentile(a, 99)),
            "max_ms": float(a.max()), "over_budget_frac": float((a > _BUDGET_MS).mean())}


def _mat_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    # returns (qx, qy, qz, qw)
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


def write_tum(path, timestamps: list[float], poses: list[np.ndarray]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        for ts, p in zip(timestamps, poses):
            tx, ty, tz = p[:3, 3]
            qx, qy, qz, qw = _mat_to_quat(p[:3, :3])
            f.write(f"{ts:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")


def compare_kiss(depths, intr, fov_h: float, fov_v: float) -> dict | None:
    """Feed the same depth stream through KISS-ICP (whole-cloud frame-to-map
    odometry, no SFLP/baro priors) as an independent drift benchmark. Returns
    None (with a message) if kiss-icp isn't installed -- this keeps
    --benchmark optional on platforms where it won't build."""
    try:
        from kiss_icp.kiss_icp import KissICP
        from kiss_icp.config import KISSConfig
        from kiss_icp.config.config import DataConfig, MappingConfig
    except ImportError:
        print("[slam] kiss-icp not installed; skipping benchmark "
              "(pip install 'roomscan[slam]')")
        return None

    from ..deproject import Deprojector
    width, height = intr_width(intr), intr_height(intr)
    dep = Deprojector(width, height, fov_h, fov_v)

    # deskew=False: our depth frames are effectively instantaneous snapshots
    # (no per-point timestamps to deskew against); voxel_size=0.05m matches
    # indoor room scale (KISS-ICP's own guidance: ~max_range/100).
    cfg = KISSConfig(data=DataConfig(deskew=False), mapping=MappingConfig(voxel_size=0.05))
    odom = KissICP(cfg)

    translations = [np.zeros(3)]
    for depth_mm in depths:
        pts, valid = dep.grid(depth_mm)
        cloud = pts[valid].astype(np.float64)
        if cloud.shape[0] < 10:
            continue  # too few points for KISS-ICP's own registration to run
        timestamps = np.zeros(cloud.shape[0], dtype=np.float64)
        odom.register_frame(cloud, timestamps)
        translations.append(odom.last_pose[:3, 3].copy())

    t = np.array(translations)
    if len(t) < 2:
        return {"path_length_m": 0.0, "start_end_gap_m": 0.0}
    steps = np.linalg.norm(np.diff(t, axis=0), axis=1)
    return {"path_length_m": float(steps.sum()),
            "start_end_gap_m": float(np.linalg.norm(t[-1] - t[0]))}


def intr_width(intr) -> int:
    # intr may live on a non-CPU compute device (Mapper(device=...)); .cpu()
    # is a no-op when it's already on CPU.
    return int(round(intr.cpu().numpy()[0, 2] * 2))


def intr_height(intr) -> int:
    return int(round(intr.cpu().numpy()[1, 2] * 2))
