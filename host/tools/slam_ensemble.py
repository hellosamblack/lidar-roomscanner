"""Score a capture with an ENSEMBLE of SLAM runs, because one run is not a measurement.

    host/.venv/bin/python host/tools/slam_ensemble.py captures/DebugCapB1.bin -n 10
    host/.venv/bin/python host/tools/slam_ensemble.py <capture> -n 5 --json

WHY THIS EXISTS

Frame-to-model tracking is chaotic at centimetre scale. A deliberate 3 mm one-shot
height nudge moves a real circuit's final height error by 146 mm and its loop
closure by 0.37 m (BUG-037), and identical runs spread 15-20% on path length and
loss count. So `ROADMAP.md` requires that drift figures be "ensemble means +/- sd
over 10 numerically innocuous perturbations, not single runs" -- every number in
the first version of that table was a single run, and all of them were wrong.

Nothing implemented that. `roomscan-slam` runs once; `slam.validation.paired_loop_gate`
consumes matched ensembles keyed on `horizontal_closure_m`, a field no code produced.
This is the missing middle.

THE PERTURBATIONS

Deterministic, and chosen to be numerically innocuous -- they must not change the
physics, only where the optimiser's chaos lands: start one frame later, and nudge
the ICP correspondence radius by +/-1e-4 m (against a 0.05 m radius, i.e. 0.2%).
Run i uses start = i // 3 and the (i % 3)-th radius delta, so a 5-run ensemble is
already spread over two start offsets and all three radii.

Device is deliberately NOT perturbed, though the phase-6 doc lists CPU-vs-CUDA as an
option: mixing them makes half the ensemble ~10x slower for no extra chaos coverage,
and it confounds any timing statistic. Pass `device=` to choose one for the whole run.

WHAT IT REPORTS

`horizontal_closure_m` is the headline and is what the acceptance gate consumes: the
start-to-end gap projected onto the horizontal plane (perpendicular to
`slam.frames.world_up()`, which is -Y on this rig, NOT -Z). It is separated from
`vertical_error_m` because the two have completely different error sources -- the
vertical is barometer-and-ICP-drift, the horizontal is pure odometry drift.

⚠ `horizontal_closure_m` is only DRIFT if the operator actually returned to the start
pose. On a capture that does not close a loop it is just the distance between two
different places, so `closed_loop` is left to the caller to assert.

⚠ Read `tracking` before any of it. Once a frame is lost the pose freezes and nothing
relocalizes, so a dead run still reports a plausible closure that is really just where
the estimate stood when it quit (BUG-036: one circuit reported 2.05 m of "drift" whose
last 22% was fabricated). `runs_died` and `worst_trailing_lost` say how much to distrust.

Only reads the capture; never writes.
"""
from __future__ import annotations

import argparse
import gc
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))

from roomscan.slam import metrics                                    # noqa: E402
from roomscan.slam.cli import _load_frames, _run                     # noqa: E402
from roomscan.slam.config import SlamConfig                          # noqa: E402
from roomscan.slam.frames import world_up                            # noqa: E402
from roomscan.slam.validation import paired_loop_gate                # noqa: E402

# 0.2% of the 0.05 m default correspondence radius: far too small to change which
# correspondences are found on any well-conditioned frame, large enough to move the
# solver onto a different chaotic trajectory. This is the perturbation the phase-6
# validation doc prescribes.
MAX_DIST_DELTAS = (-1e-4, 0.0, 1e-4)
DEFAULT_N = 10


def perturbations(n: int) -> list[dict]:
    """The deterministic perturbation set. Same n always gives the same list."""
    return [{"index": i, "start_frame": i // len(MAX_DIST_DELTAS),
             "max_dist_delta": MAX_DIST_DELTAS[i % len(MAX_DIST_DELTAS)]}
            for i in range(n)]


def split_closure(poses: list[np.ndarray]) -> dict:
    """Start-to-end gap split into horizontal drift and vertical error.

    Uses `slam.frames.world_up()` rather than assuming an axis: up is -Y here, and
    calling the wrong component "height" is a mistake this repo has made before.
    """
    if len(poses) < 2:
        return {"horizontal_closure_m": 0.0, "vertical_error_m": 0.0, "closure_m": 0.0}
    v = np.asarray(poses[-1][:3, 3], dtype=np.float64) - np.asarray(poses[0][:3, 3],
                                                                    dtype=np.float64)
    up = world_up()
    vertical = float(np.dot(v, up))
    horizontal = v - vertical * up
    return {"horizontal_closure_m": float(np.linalg.norm(horizontal)),
            "vertical_error_m": vertical,
            "closure_m": float(np.linalg.norm(v))}


def _mean_sd(values: list[float]) -> dict:
    a = np.asarray(values, dtype=np.float64)
    if a.size == 0:
        return {"mean": None, "sd": None, "min": None, "max": None}
    return {"mean": float(a.mean()),
            # ddof=1: this is a sample of a chaotic process, not the population.
            "sd": float(a.std(ddof=1)) if a.size > 1 else 0.0,
            "min": float(a.min()), "max": float(a.max())}


def run_ensemble(capture, *, n: int = DEFAULT_N, device: str | None = None,
                 voxel_size: float | None = None, block_count: int | None = None,
                 icp_mode: str | None = None, max_frames: int | None = None,
                 baro_authority: float | None = None, max_iter: int | None = None,
                 progress=None) -> dict:
    """Run `n` perturbed SLAM passes over one capture and summarise the spread.

    The capture is decoded ONCE and the frame list reused across every run -- the
    transform pass costs more than the SLAM pass on a big capture, and re-decoding
    per run would triple the wall clock for no added independence.
    """
    cfg = SlamConfig.load()
    if voxel_size is not None:
        cfg.voxel_size = voxel_size
    if block_count is not None:
        cfg.block_count = block_count
    if baro_authority is not None:
        cfg.baro_authority = baro_authority
    if max_iter is not None:
        cfg.max_iter = max(1, int(max_iter))
    mode = icp_mode or cfg.icp_mode
    dev = device or cfg.device

    frames, width, height = _load_frames(str(capture), max_frames)
    if not frames:
        return {"capture": str(capture), "error": "no depth frames decoded from capture"}

    base_max_dist = cfg.max_dist
    runs: list[dict] = []
    for pert in perturbations(n):
        cfg.max_dist = base_max_dist + pert["max_dist_delta"]
        sub = frames[pert["start_frame"]:]
        t0 = time.perf_counter()
        mapper, timings, _ts = _run(sub, width, height, cfg, mode, device=dev)
        wall = time.perf_counter() - t0

        track = metrics.tracking_stats(mapper.lost_flags)
        traj = metrics.trajectory_stats(mapper.trajectory)
        used, live_cap = mapper._tsdf.block_usage()
        row = {
            **pert,
            **split_closure(mapper.trajectory),
            "path_length_m": traj["path_length_m"],
            "start_end_gap_m": traj["start_end_gap_m"],
            "max_step_m": traj["max_step_m"],
            "frames": traj["n"],
            # `lost` and `died` are the field names paired_loop_gate reads.
            "lost": track["lost"],
            "died": track["died"],
            "trailing_lost": track["trailing_lost"],
            "longest_lost_run": track["longest_lost_run"],
            "icp_escalations": mapper.icp_escalations,
            "baro_correction_m": mapper.baro_correction_m,
            "blocks": used,
            "saturated": bool(used >= 0.97 * cfg.block_count),
            "median_ms": metrics.timing_stats(timings)["median_ms"],
            "wall_s": round(wall, 1),
        }
        runs.append(row)
        if progress:
            progress(row, n)
        # A Mapper owns a TSDF grid on the compute device; without dropping it the
        # next run in the ensemble allocates a second one alongside (BUG-032's
        # neighbourhood).
        del mapper
        gc.collect()
    cfg.max_dist = base_max_dist

    summary = {k: _mean_sd([r[k] for r in runs]) for k in
               ("horizontal_closure_m", "vertical_error_m", "closure_m",
                "path_length_m", "median_ms")}
    return {
        "capture": str(capture),
        "n": n, "device": dev, "icp_mode": mode,
        "voxel_size": cfg.voxel_size, "block_count": cfg.block_count,
        "frames_loaded": len(frames), "width": width, "height": height,
        "summary": summary,
        "runs_died": sum(1 for r in runs if r["died"]),
        "worst_trailing_lost": max(r["trailing_lost"] for r in runs),
        # `died` is trailing-only, so a run that freezes mid-scan and then
        # re-registers reports died=False and a plausible closure -- DebugCapB2
        # froze for 628 frames (21.2 s, 15.5% of the run) and still passed it.
        # This is the field that catches that class.
        "worst_longest_lost_run": max(r["longest_lost_run"] for r in runs),
        "worst_lost": max(r["lost"] for r in runs),
        "total_icp_escalations": sum(r["icp_escalations"] for r in runs),
        "any_saturated": any(r["saturated"] for r in runs),
        "runs": runs,
    }


def compare(baseline: dict, closed: dict) -> dict:
    """Apply the pre-registered paired gate to two matched ensembles."""
    return paired_loop_gate(baseline["runs"], closed["runs"])


def format_report(r: dict) -> str:
    if "error" in r:
        return f"=== {r['capture']} ===\n  {r['error']}"
    s = r["summary"]
    out = [f"=== {r['capture']} ===",
           f"  {r['n']} runs, {r['frames_loaded']} frames, {r['width']}x{r['height']}, "
           f"{r['icp_mode']} on {r['device']}, voxel {r['voxel_size'] * 1000:g} mm"]
    hc, ve = s["horizontal_closure_m"], s["vertical_error_m"]
    out.append(f"  horizontal closure: {hc['mean']:.3f} +/- {hc['sd']:.3f} m "
               f"(range {hc['min']:.3f}..{hc['max']:.3f})")
    out.append(f"  vertical error:     {ve['mean'] * 1000:+.0f} +/- {ve['sd'] * 1000:.0f} mm")
    out.append(f"  path length:        {s['path_length_m']['mean']:.2f} +/- "
               f"{s['path_length_m']['sd']:.2f} m")
    out.append(f"  tracking: {r['runs_died']}/{r['n']} runs died, worst trailing "
               f"{r['worst_trailing_lost']}, worst lost {r['worst_lost']}, longest freeze "
               f"{r['worst_longest_lost_run']}, {r['total_icp_escalations']} ICP escalations total")
    if not r["runs_died"] and r["worst_longest_lost_run"] >= 30:
        out.append(f"  !! froze for {r['worst_longest_lost_run']} frames MID-RUN then recovered: "
                   f"`died` is trailing-only and does not catch this (that segment is "
                   f"dead-reckoned, so the path length and closure are contaminated)")
    if r["any_saturated"]:
        out.append("  !! at least one run SATURATED its block grid -- raise block_count (BUG-035)")
    if r["runs_died"]:
        out.append("  !! a run DIED: its closure is where the estimate quit, not drift (BUG-036)")
    out.append(f"  median step {s['median_ms']['mean']:.1f} ms")
    out.append("  runs:")
    for x in r["runs"]:
        out.append(f"    #{x['index']:<2} start+{x['start_frame']} d{x['max_dist_delta']:+.0e}  "
                   f"h={x['horizontal_closure_m']:.3f} m  v={x['vertical_error_m'] * 1000:+5.0f} mm  "
                   f"path={x['path_length_m']:6.2f} m  lost={x['lost']:<4} "
                   f"trail={x['trailing_lost']:<4} esc={x['icp_escalations']:<3} "
                   f"{x['wall_s']}s")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="slam_ensemble",
        description="Ensemble SLAM scoring -- a single run is not a measurement (BUG-037).")
    ap.add_argument("capture")
    ap.add_argument("-n", type=int, default=DEFAULT_N, dest="n")
    ap.add_argument("--device", default=None)
    ap.add_argument("--voxel-size", type=float, default=None)
    ap.add_argument("--block-count", type=int, default=None)
    ap.add_argument("--icp-mode", choices=["translation", "6dof"], default=None)
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--baro-authority", type=float, default=None)
    ap.add_argument("--max-iter", type=int, default=None)
    ap.add_argument("--json", default=None, metavar="PATH")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    def tick(row, total):
        if not args.quiet:
            print(f"[ensemble] run {row['index'] + 1}/{total}: "
                  f"h={row['horizontal_closure_m']:.3f} m lost={row['lost']} "
                  f"({row['wall_s']}s)", flush=True)

    r = run_ensemble(args.capture, n=args.n, device=args.device,
                     voxel_size=args.voxel_size, block_count=args.block_count,
                     icp_mode=args.icp_mode, max_frames=args.max_frames,
                     baro_authority=args.baro_authority, max_iter=args.max_iter,
                     progress=tick)
    print(format_report(r))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(r, indent=2), encoding="utf-8")
        print(f"[ensemble] wrote {args.json}")
    return 1 if "error" in r else 0


if __name__ == "__main__":
    sys.exit(main())
