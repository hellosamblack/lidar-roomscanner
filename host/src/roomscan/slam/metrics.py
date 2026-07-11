"""Trajectory / timing metrics, TUM export, and an optional KISS-ICP benchmark
for offline validation. compare_kiss imports kiss_icp lazily so it's optional."""
from __future__ import annotations

import numpy as np

_BUDGET_MS = 35.0


def trajectory_stats(poses: list[np.ndarray]) -> dict:
    t = np.array([p[:3, 3] for p in poses]) if poses else np.zeros((0, 3))
    if len(t) < 2:
        return {"n": len(t), "path_length_m": 0.0, "start_end_gap_m": 0.0, "max_step_m": 0.0}
    steps = np.linalg.norm(np.diff(t, axis=0), axis=1)
    return {"n": len(t), "path_length_m": float(steps.sum()),
            "start_end_gap_m": float(np.linalg.norm(t[-1] - t[0])),
            "max_step_m": float(steps.max())}


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
        qw = (m[2, 1] - m[1, 2]) / s; qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s; qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        qw = (m[0, 2] - m[2, 0]) / s; qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s; qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        qw = (m[1, 0] - m[0, 1]) / s; qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s; qz = 0.25 * s
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
