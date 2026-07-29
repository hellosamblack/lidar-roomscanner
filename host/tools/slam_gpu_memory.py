"""Sub-phase 6.G measurement rig: per-frame GPU allocation growth for the SLAM
pipeline on CUDA:0.

The GPU SLAM path OOMs on a long scan (~11.7 GB over a 68 m walk, then a
`ParallelFor` allocation failure). The map itself is NOT the suspect -- the
raycast is frustum-bounded and a ~40k-block VoxelBlockGrid is only ~410 MB --
so this tool exists to separate the two hypotheses:

    H1  memory tracks the map        -> device bytes rise with the ACTIVE BLOCK
                                        COUNT and flatten when the map saturates
    H2  memory tracks the work done  -> device bytes keep rising while the block
                                        count is flat (caching allocator +
                                        temporaries never released)

It therefore logs device bytes, active block count, hashmap capacity, and
per-step wall time for EVERY frame, so the two curves can be compared directly
rather than inferred from a high-water mark.

What it found (RTX 2000 Ada, 8 GiB): H2, and specifically the throttled
`mesh()`/`point_cloud()` EXTRACTION, not the per-frame path. See
`slam/tsdf.py::_release_cache_if_due` for the numbers and the mechanism.

Two frame sources:

  --capture PATH     real recorded frames through the real `Mapper.step`
                     (the live code path, exactly as `roomscan-slam` runs it).
                     Bounded by how much scan the capture contains.

  --synthetic        a generated walk through an analytic box room with pillars
                     (`roomscan.slam.synthscene`), runnable to any length. Depth
                     frames are ray-cast from the scene at ground-truth poses
                     and fed through the same `Mapper.step`; the mapper still
                     runs its own raycast + ICP + integrate, so poses are NOT
                     injected and the measured code path is the production one.

Usage:

    # the per-frame path on its own (no extraction) -- byte-flat
    host/.venv/bin/python host/tools/slam_gpu_memory.py --synthetic --frames 4000

    # the live path: extraction at SlamWorker's cadence. A/B the shipped fix by
    # driving the real Mapper/TsdfMap knob, 0 = off vs 1 = shipped default.
    host/.venv/bin/python host/tools/slam_gpu_memory.py --synthetic --frames 1500 \
        --mesh-every 5 --release-cache-every 0
    host/.venv/bin/python host/tools/slam_gpu_memory.py --synthetic --frames 1500 \
        --mesh-every 5 --release-cache-every 1

    host/.venv/bin/python host/tools/slam_gpu_memory.py --capture captures/x.bin --mesh-every 5

Device memory comes from NVML (`roomscan.slam.gpumem`, ctypes -- no pynvml
dependency) and is DEVICE-WIDE, so run it with nothing else on the GPU; the
baseline is printed and subtracted in the summary. Writes a per-frame CSV to
results/ unless --no-csv.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np

from roomscan.slam.gpumem import Nvml
from roomscan.slam.synthscene import SyntheticWalk


# ------------------------------------------------------------------- the rig


def _load_capture(path: str, max_frames: int | None):
    from roomscan.slam.cli import _load_frames
    frames, width, height = _load_frames(path, max_frames)
    if not frames:
        raise SystemExit(f"[6.G] no depth frames decoded from {path}")
    return frames, width, height


def _percentile(xs, q):
    return float(np.percentile(np.asarray(xs, dtype=np.float64), q)) if xs else 0.0


def _slope_mb_per_frame(idx, mb, lo_frac=0.5):
    """Least-squares MB/frame over the tail of the run (default: last half),
    where start-up transients have settled."""
    if len(idx) < 20:
        return 0.0
    lo = int(len(idx) * lo_frac)
    x = np.asarray(idx[lo:], dtype=np.float64)
    y = np.asarray(mb[lo:], dtype=np.float64)
    return float(np.polyfit(x, y, 1)[0])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="slam_gpu_memory")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--capture", help="recorded .bin replayed through Mapper.step")
    src.add_argument("--synthetic", action="store_true",
                     help="generated walk through an analytic room, runnable to any length")
    ap.add_argument("--frames", type=int, default=2000,
                    help="synthetic frame count, or cap on capture frames (default 2000)")
    ap.add_argument("--loops", type=int, default=1,
                    help="replay a capture this many times back-to-back (default 1)")
    ap.add_argument("--device", default="CUDA:0")
    ap.add_argument("--release-cache-every", type=int, default=None,
                    help="Mapper/TsdfMap release_cache_every: release Open3D's CUDA cache "
                         "every N EXTRACTIONS (0 = off, 1 = shipped default). This drives the "
                         "real shipped knob, so an A/B here measures production behaviour "
                         "(default: [slam] release_cache_every in roomscan.toml).")
    ap.add_argument("--mesh-every", type=int, default=0,
                    help="extract a mesh every N integrated frames, as SlamWorker does "
                         "(slam/worker.py _MESH_EVERY = 5). 0 = never, which measures the "
                         "per-frame path alone.")
    ap.add_argument("--point-cloud", action="store_true",
                    help="also extract a point cloud at --mesh-every (the map_point_cloud path)")
    ap.add_argument("--voxel-size", type=float, default=None,
                    help="override [slam] voxel_size (smaller = more blocks per metre walked)")
    ap.add_argument("--width", type=int, default=54)
    ap.add_argument("--height", type=int, default=42)
    ap.add_argument("--csv", default=None,
                    help="per-frame CSV path (default results/slam_gpu_mem_<tag>.csv)")
    ap.add_argument("--no-csv", action="store_true")
    ap.add_argument("--progress", type=int, default=200, help="print a line every N frames")
    args = ap.parse_args(argv)

    import open3d as o3d
    from roomscan.slam.config import SlamConfig
    from roomscan.slam.mapper import Mapper

    if "CUDA" in args.device.upper() and not o3d.core.cuda.is_available():
        print("[6.G] CUDA not available in this Open3D build", file=sys.stderr)
        return 2

    cfg = SlamConfig.load()
    voxel = args.voxel_size if args.voxel_size is not None else cfg.voxel_size
    release_every = (args.release_cache_every if args.release_cache_every is not None
                     else cfg.release_cache_every)

    nvml = Nvml()
    if not nvml.ok:
        print("[6.G] WARNING: libnvidia-ml unavailable -- memory columns will read 0",
              file=sys.stderr)
    baseline = nvml.used_bytes()

    if args.synthetic:
        walk = SyntheticWalk(args.width, args.height, cfg.fov_h, cfg.fov_v)
        width, height, n_frames = args.width, args.height, args.frames
        source = "synthetic"
        frames = None
    else:
        frames, width, height = _load_capture(args.capture, args.frames)
        n_frames = len(frames) * args.loops
        source = Path(args.capture).name
        walk = None

    mapper = Mapper(width, height, cfg.fov_h, cfg.fov_v, icp_mode=cfg.icp_mode,
                    voxel_size=voxel, baro_weight=cfg.baro_weight,
                    max_dist=cfg.max_dist, min_fitness=cfg.min_fitness,
                    max_rmse=cfg.max_rmse, min_confidence=cfg.min_confidence,
                    weight_threshold=cfg.weight_threshold,
                    stationary_hold=cfg.stationary_hold,
                    release_cache_every=release_every,
                    device=args.device)
    vbg = mapper._tsdf._vbg

    print(f"[6.G] source={source} frames={n_frames} device={args.device} "
          f"voxel={voxel} mesh_every={args.mesh_every} release_cache_every={release_every}")
    print(f"[6.G] GPU baseline {baseline / 2**20:.0f} MiB of {nvml.total_bytes() / 2**20:.0f} MiB")

    rows = []
    integrated = 0
    last_mesh = None            # held exactly as SlamWorker holds it
    t_start = time.perf_counter()
    for i in range(n_frames):
        if walk is not None:
            depth, quat = walk.next_frame()
            refl = conf = None
            pa = None
        else:
            depth, refl, conf, quat, pa, _ts = frames[i % len(frames)]

        step = mapper.step(depth, quat, pa, reflectance=refl, confidence=conf)

        # Mirror SlamWorker.run_once: throttle extraction on INTEGRATED frames
        # (only those changed the TSDF) and HOLD the result, as the worker's
        # `_last_mesh` does -- holding it matters, a mesh dropped immediately
        # would not show a retention leak.
        if args.mesh_every and not step.tracking_lost:
            integrated += 1
            if integrated == 1 or integrated % args.mesh_every == 0:
                last_mesh = mapper.mesh()                   # noqa: F841 -- held on purpose
                if args.point_cloud:
                    last_pc = mapper.map_point_cloud()      # noqa: F841 -- held on purpose

        hm = vbg.hashmap()
        rows.append((i, nvml.used_bytes(), int(hm.size()), int(hm.capacity()),
                     step.slam_ms, step.fitness, int(step.tracking_lost)))
        if args.progress and (i + 1) % args.progress == 0:
            _, used, blocks, cap, ms, fit, _lost = rows[-1]
            print(f"  frame {i+1:6d}  gpu {used / 2**20:8.0f} MiB  blocks {blocks:7d}"
                  f"/{cap:<7d}  {ms:6.1f} ms  fit {fit:.2f}  lost {mapper.tracking_lost_count}")

    wall = time.perf_counter() - t_start
    idx = [r[0] for r in rows]
    mb = [(r[1] - baseline) / 2**20 for r in rows]
    blocks = [r[2] for r in rows]
    ms = [r[4] for r in rows]

    peak_blocks = max(blocks) if blocks else 0
    tail = int(len(blocks) * 0.5)
    block_growth_tail = (blocks[-1] - blocks[tail]) if blocks else 0

    print(f"\n[6.G] === summary ({len(rows)} frames, {wall:.1f} s wall) ===")
    if walk is not None:
        print(f"  synthetic path      : {walk.path_length_m:.1f} m")
    print(f"  GPU (above baseline): first {mb[0]:.0f} MiB  peak {max(mb):.0f} MiB  last {mb[-1]:.0f} MiB")
    print(f"  growth, last half   : {_slope_mb_per_frame(idx, mb):.4f} MiB/frame")
    print(f"  blocks              : peak {peak_blocks}  growth over last half {block_growth_tail}")
    print(f"  tracking lost       : {mapper.tracking_lost_count} frames")
    print(f"  step ms             : p50 {_percentile(ms, 50):.1f}  p90 {_percentile(ms, 90):.1f} "
          f" p99 {_percentile(ms, 99):.1f}  max {max(ms):.1f}")
    print(f"  cache releases      : {mapper._tsdf.cache_releases}")

    if not args.no_csv:
        tag = ("synth" if args.synthetic else Path(args.capture).stem)
        tag += f"_mesh{args.mesh_every}_rel{release_every}"
        out = Path(args.csv) if args.csv else Path("results") / f"slam_gpu_mem_{tag}.csv"
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame", "gpu_used_bytes", "blocks", "capacity",
                        "slam_ms", "fitness", "lost"])
            w.writerows(rows)
        print(f"  csv                 : {out}")

    nvml.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
