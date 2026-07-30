"""roomscan-slam: run frame-to-model SLAM over a recorded capture and report
trajectory + timing, optionally comparing ICP modes / KISS-ICP."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from ..decoder import StreamDecoder
from ..flatfield import FlatField
from ..pipeline import TransformStage
from ..protocol import StreamId, FrameType, decode_imu_quat, decode_env
from .config import SlamConfig
from .mapper import Mapper
from . import metrics


def _load_frames(path, max_frames=None):
    """Return (frames, width, height) where frames is a list of
    (depth_mm(h,w), reflectance(h,w)|None, confidence(h,w)|None, quat(4),
    pressure_pa|None, t_s). Depth/reflectance/confidence come from
    TransformStage; quat/pressure are carried forward from the latest 9/10.
    reflectance/confidence are None for sources that don't provide them (the
    on-device DEPTH_ZF32 passthrough path only ever returns "depth")."""
    dec = StreamDecoder()
    stage = TransformStage(outputs=("depth", "reflectance", "confidence"),
                           flatfield=FlatField.load_configured())
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
        reflectance = arrays.get("reflectance")
        confidence = arrays.get("confidence")
        width, height = header.width, header.height
        frames.append((
            depth.astype(np.float32),
            reflectance.astype(np.float32) if reflectance is not None else None,
            confidence.astype(np.float32) if confidence is not None else None,
            last_quat, last_pa, header.t_us / 1e6,
        ))
        if max_frames and len(frames) >= max_frames:
            break
    return frames, width, height


def _run(frames, width, height, cfg, mode, device=None):
    mapper = Mapper(width, height, cfg.fov_h, cfg.fov_v, icp_mode=mode,
                    voxel_size=cfg.voxel_size, baro_authority=cfg.baro_authority,
                    baro_tau_frames=cfg.baro_tau_frames,
                    max_dist=cfg.max_dist, icp_retry_dist=cfg.icp_retry_dist,
                    min_fitness=cfg.min_fitness, max_rmse=cfg.max_rmse,
                    min_confidence=cfg.min_confidence, weight_threshold=cfg.weight_threshold,
                    release_cache_every=cfg.release_cache_every,
                    block_count=cfg.block_count,
                    stationary_hold=cfg.stationary_hold, stationary_window=cfg.stationary_window,
                    stationary_coherence=cfg.stationary_coherence,
                    stationary_step_ceiling=cfg.stationary_step_ceiling,
                    stationary_rot_ceiling=cfg.stationary_rot_ceiling,
                    device=device if device is not None else cfg.device)
    timings, ts = [], []
    for depth, reflectance, confidence, quat, pa, t_s in frames:
        step = mapper.step(depth, quat, pa, reflectance=reflectance, confidence=confidence)
        timings.append(step.slam_ms)
        ts.append(t_s)
    return mapper, timings, ts


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="roomscan-slam")
    ap.add_argument("capture")
    ap.add_argument("--icp-mode", choices=["translation", "6dof"], default=None)
    ap.add_argument("--device", default=None,
                    help='Open3D compute device, e.g. "CPU:0" or "CUDA:0" '
                         "(default: [slam].device in roomscan.toml, else CPU:0). "
                         "Only CPU:0 is testable until a CUDA-enabled Open3D build "
                         "is installed.")
    ap.add_argument("--compare-modes", action="store_true")
    ap.add_argument("--benchmark", action="store_true")
    ap.add_argument("--out-mesh", default="slam_map.ply")
    ap.add_argument("--out-traj", default="slam_traj.tum")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--json", default=None, metavar="PATH",
                    help="also write the run's stats to PATH as JSON. The prose above is for "
                         "humans; this is the machine-readable front end (roomscan-mcp's "
                         "slam_rerender reads it) so nothing has to scrape stdout.")
    ap.add_argument("--voxel-size", type=float, default=None,
                    help="TSDF voxel size in metres, overriding [slam] voxel_size. The live "
                         "scan is only a preview -- the capture holds raw frames, so re-running "
                         "offline at a finer voxel is how you get a high-detail map. Note the "
                         "sensor samples ~36 mm between rays at 2 m, so below ~5 mm the extra "
                         "detail comes only from multi-view fusion, not from a single view.")
    ap.add_argument("--block-count", type=int, default=None,
                    help="TSDF grid capacity, overriding [slam] block_count. Blocks scale as "
                         "1/voxel_size^2, so halving the voxel needs ~4x this. Give it real "
                         # NB "%%": argparse %-expands help strings, and a bare "% o" is a
                         # valid octal conversion -- this line alone made --help crash with
                         # "%o format: an integer is required, not dict" (found 2026-07-30).
                         "headroom: a scan that ran at ~97%% of its capacity stalled and lost "
                         "tracking (BUG-035). Beyond ~6 GiB use --device CPU:0, where system "
                         "RAM rather than VRAM is the limit.")
    ap.add_argument("--baro-authority", type=float, default=None,
                    help="barometer's share of the low-passed height disagreement, overriding "
                         "[slam] baro_authority; 0 turns the height constraint off. Provided so "
                         "the default can be RE-measured rather than argued about -- but note "
                         "single-run comparisons below ~0.3 m are chaos, not signal (a 3 mm "
                         "one-shot height nudge moves the final height error by 146 mm and the "
                         "loop closure by 0.37 m on a real circuit). Sweep it across an ensemble "
                         "of innocuous perturbations, not one run. See BUG-037.")
    args = ap.parse_args(argv)

    cfg = SlamConfig.load()
    if args.baro_authority is not None:
        cfg.baro_authority = args.baro_authority
    if args.voxel_size is not None:
        cfg.voxel_size = args.voxel_size
    if args.block_count is not None:
        cfg.block_count = args.block_count
    frames, width, height = _load_frames(args.capture, args.max_frames)
    if not frames:
        print("[slam] no depth frames decoded from capture", file=sys.stderr)
        return 1
    print(f"[slam] {len(frames)} frames, {width}x{height}, "
          f"voxel {cfg.voxel_size * 1000:g} mm, capacity {cfg.block_count} blocks")

    modes = ["translation", "6dof"] if args.compare_modes else [args.icp_mode or cfg.icp_mode]
    results = {}
    report = {"capture": args.capture, "frames": len(frames),
              "width": width, "height": height,
              "voxel_size": cfg.voxel_size, "block_count": cfg.block_count,
              "device": args.device or cfg.device, "modes": {}}
    for mode in modes:
        mapper, timings, ts = _run(frames, width, height, cfg, mode, device=args.device)
        tstats = metrics.trajectory_stats(mapper.trajectory)
        mstats = metrics.timing_stats(timings)
        results[mode] = (mapper, tstats, mstats, ts)
        print(f"\n=== mode={mode} ===")
        print(f"  trajectory: n={tstats['n']} path={tstats['path_length_m']:.3f} m "
              f"gap={tstats['start_end_gap_m']:.3f} m max_step={tstats['max_step_m']:.3f} m")
        print(f"  timing: median={mstats['median_ms']:.1f} ms p90={mstats['p90_ms']:.1f} "
              f"p99={mstats['p99_ms']:.1f} max={mstats['max_ms']:.1f} "
              f"over35ms={mstats['over_budget_frac']*100:.1f}% lost={mapper.tracking_lost_count}")
        kstats = metrics.tracking_stats(mapper.lost_flags)
        died = ("  <-- THE RUN DIED: the tail is a frozen dead-reckoned pose, "
                "not a measured trajectory" if kstats["died"] else "")
        # BUG-037: say how much of the reported height came from the barometer
        # rather than from ICP, so a run that was pulled by a drifting baro
        # says so instead of quietly reporting it as measured motion.
        print(f"  baro: correction={mapper.baro_correction_m * 1000:+.0f} mm "
              f"(authority={cfg.baro_authority:g}, tau={cfg.baro_tau_frames} frames)")
        print(f"  tracking: lost={kstats['lost']}/{kstats['n']} "
              f"({kstats['lost_frac']*100:.1f}%) longest_run={kstats['longest_lost_run']} "
              f"trailing={kstats['trailing_lost']} "
              f"icp_escalations={mapper.icp_escalations}{died}")
        # BUG-035: report against the CONFIGURED capacity, not the live one --
        # the grid rehashes to grow, so live capacity always looks roomy; what
        # predicted the failure was running near the value it was built with.
        used, live_cap = mapper._tsdf.block_usage()
        cap = cfg.block_count
        saturated = used >= 0.97 * cap
        note = ("  <-- outgrew the configured capacity; give it headroom via --block-count"
                if saturated else "")
        print(f"  map: {used} blocks, {100.0 * used / cap:.0f}% of the configured {cap} "
              f"(live grid capacity {live_cap}){note}")
        report["modes"][mode] = {
            "trajectory": dict(tstats), "timing": dict(mstats),
            "tracking_lost": mapper.tracking_lost_count,
            "tracking": dict(kstats),
            "icp_escalations": mapper.icp_escalations,
            "baro": {"correction_m": mapper.baro_correction_m,
                     "authority": cfg.baro_authority,
                     "tau_frames": cfg.baro_tau_frames},
            "map": {"blocks": used, "capacity": cap, "live_capacity": live_cap,
                    "percent_of_capacity": round(100.0 * used / cap, 1),
                    "saturated": bool(saturated)},
        }

    chosen = modes[0]
    mapper, _, _, ts = results[chosen]
    import open3d as o3d
    # mesh() may live on a non-CPU compute device (--device); write_triangle_mesh
    # is host-side I/O -- .cpu() is a no-op when it's already on CPU.
    o3d.t.io.write_triangle_mesh(args.out_mesh, mapper.mesh().cpu())
    metrics.write_tum(args.out_traj, ts, mapper.trajectory)
    print(f"\n[slam] wrote {args.out_mesh} and {args.out_traj} (mode={chosen})")

    report["chosen_mode"] = chosen
    report["out_mesh"], report["out_traj"] = args.out_mesh, args.out_traj

    if args.benchmark:
        depths = [d for d, _, _, _, _, _ in frames]
        kiss = metrics.compare_kiss(depths, mapper._intr, cfg.fov_h, cfg.fov_v)
        if kiss:
            print(f"[slam] KISS-ICP: path={kiss['path_length_m']:.3f} m "
                  f"gap={kiss['start_end_gap_m']:.3f} m")
            report["kiss_icp"] = dict(kiss)

    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"[slam] wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
