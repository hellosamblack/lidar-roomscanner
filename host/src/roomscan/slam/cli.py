"""roomscan-slam: run frame-to-model SLAM over a recorded capture and report
trajectory + timing, optionally comparing ICP modes / KISS-ICP."""
from __future__ import annotations

import argparse
import sys

import numpy as np

from ..decoder import StreamDecoder
from ..pipeline import TransformStage
from ..protocol import StreamId, FrameType, decode_imu_quat, decode_env
from .config import SlamConfig
from .mapper import Mapper
from . import metrics


def _load_frames(path, max_frames=None):
    """Return (frames, width, height) where frames is a list of
    (depth_mm(h,w), quat(4), pressure_pa|None, t_s). Depth comes from
    TransformStage; quat/pressure are carried forward from the latest 9/10."""
    dec = StreamDecoder()
    stage = TransformStage(outputs=("depth",))
    with open(path, "rb") as f:
        data = f.read()
    frames = []
    last_quat = (1.0, 0.0, 0.0, 0.0)
    last_pa = None
    width = height = None
    for frame in dec.feed(data):
        h = frame.header
        if h.frame_type != FrameType.DATA:
            continue
        if h.stream_id == StreamId.IMU_QUAT:
            last_quat = decode_imu_quat(frame.payload)
            continue
        if h.stream_id == StreamId.ENV:
            last_pa = decode_env(frame.payload)[0]
            continue
        out = stage.feed(frame)
        if out is None:
            continue
        header, arrays = out
        depth = arrays.get("depth")
        if depth is None:
            continue
        width, height = header.width, header.height
        frames.append((depth.astype(np.float32), last_quat, last_pa, header.t_us / 1e6))
        if max_frames and len(frames) >= max_frames:
            break
    return frames, width, height


def _run(frames, width, height, cfg, mode):
    mapper = Mapper(width, height, cfg.fov_h, cfg.fov_v, icp_mode=mode,
                    voxel_size=cfg.voxel_size, baro_weight=cfg.baro_weight,
                    max_dist=cfg.max_dist, min_fitness=cfg.min_fitness, max_rmse=cfg.max_rmse)
    timings, ts = [], []
    for depth, quat, pa, t_s in frames:
        step = mapper.step(depth, quat, pa)
        timings.append(step.slam_ms)
        ts.append(t_s)
    return mapper, timings, ts


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="roomscan-slam")
    ap.add_argument("capture")
    ap.add_argument("--icp-mode", choices=["translation", "6dof"], default=None)
    ap.add_argument("--compare-modes", action="store_true")
    ap.add_argument("--benchmark", action="store_true")
    ap.add_argument("--out-mesh", default="slam_map.ply")
    ap.add_argument("--out-traj", default="slam_traj.tum")
    ap.add_argument("--max-frames", type=int, default=None)
    args = ap.parse_args(argv)

    cfg = SlamConfig.load()
    frames, width, height = _load_frames(args.capture, args.max_frames)
    if not frames:
        print("[slam] no depth frames decoded from capture", file=sys.stderr)
        return 1
    print(f"[slam] {len(frames)} frames, {width}x{height}")

    modes = ["translation", "6dof"] if args.compare_modes else [args.icp_mode or cfg.icp_mode]
    results = {}
    for mode in modes:
        mapper, timings, ts = _run(frames, width, height, cfg, mode)
        tstats = metrics.trajectory_stats(mapper.trajectory)
        mstats = metrics.timing_stats(timings)
        results[mode] = (mapper, tstats, mstats, ts)
        print(f"\n=== mode={mode} ===")
        print(f"  trajectory: n={tstats['n']} path={tstats['path_length_m']:.3f} m "
              f"gap={tstats['start_end_gap_m']:.3f} m max_step={tstats['max_step_m']:.3f} m")
        print(f"  timing: median={mstats['median_ms']:.1f} ms p90={mstats['p90_ms']:.1f} "
              f"p99={mstats['p99_ms']:.1f} max={mstats['max_ms']:.1f} "
              f"over35ms={mstats['over_budget_frac']*100:.1f}% lost={mapper.tracking_lost_count}")

    chosen = modes[0]
    mapper, _, _, ts = results[chosen]
    import open3d as o3d
    o3d.t.io.write_triangle_mesh(args.out_mesh, mapper.mesh())
    metrics.write_tum(args.out_traj, ts, mapper.trajectory)
    print(f"\n[slam] wrote {args.out_mesh} and {args.out_traj} (mode={chosen})")

    if args.benchmark:
        depths = [d for d, _, _, _ in frames]
        kiss = metrics.compare_kiss(depths, mapper._intr, cfg.fov_h, cfg.fov_v)
        if kiss:
            print(f"[slam] KISS-ICP: path={kiss['path_length_m']:.3f} m "
                  f"gap={kiss['start_end_gap_m']:.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
