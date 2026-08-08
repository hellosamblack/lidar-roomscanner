"""roomscan-splat: build a Gaussian splat from a video, or list built splats.

    roomscan-splat build captures/room.mp4 --name "Sam Office"
    roomscan-splat list

The build is minutes-long (COLMAP on CPU + gsplat training on the GPU); progress
prints to stderr so ``--json`` on stdout stays clean for machine callers (the
``splat_build`` MCP tool reads that JSON).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

from .config import SplatPreset
from .sidecar import list_splats

# Every knob the web form / MCP tool can override. `None` => keep the preset value.
_PRESET_OVERRIDES = ("fps", "max_frames", "long_edge", "matcher", "sequential_overlap",
                     "sh_degree", "max_gaussians", "iters", "min_opacity", "opacity_reg",
                     "scale_reg", "cull_opacity", "cull_radius_factor", "depth_lambda",
                     "depth_model")

# Global-progress weights per phase so a single bar advances monotonically across the
# three stages (COLMAP dominates wall time on CPU; training on GPU). base = cumulative.
_PHASE_WEIGHTS = {"frames": (0.0, 0.05), "sfm": (0.05, 0.35), "train": (0.40, 0.60)}


def _default_results_dir() -> str:
    # Match the web server's cwd-relative "results" convention.
    return "results"


def _write_json_atomic(path: Path, obj: dict) -> None:
    """tmp-then-rename write so a reader never sees a half-written progress file
    (same idiom as sidecar.write_manifest_atomic; the SplatRunner polls this ~30 Hz)."""
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj), encoding="utf-8")
    os.replace(tmp, path)


def _cmd_build(args) -> int:
    from . import build_splat   # lazy: pulls torch/gsplat/pycolmap only on build

    preset = SplatPreset.load()
    for k in _PRESET_OVERRIDES:
        v = getattr(args, k, None)
        if v is not None:
            setattr(preset, k, v)

    def log(m):
        print(m, file=sys.stderr, flush=True)

    t0 = time.time()
    progress_path = Path(args.progress_file) if args.progress_file else None

    def progress(phase, frac):
        print(f"[{phase}] {frac * 100:5.1f}%", file=sys.stderr, flush=True)
        if progress_path is not None:
            base, weight = _PHASE_WEIGHTS.get(phase, (0.0, 1.0))
            try:
                _write_json_atomic(progress_path, {
                    "phase": phase, "phase_fraction": round(float(frac), 4),
                    "fraction": round(base + weight * float(frac), 4),
                    "elapsed_s": round(time.time() - t0, 1),
                    "updated_unix": round(time.time(), 3)})
            except OSError:
                pass   # progress reporting is best-effort; never fail a build over it

    report = build_splat(args.video, args.name, args.results_dir, preset=preset,
                         force=args.force, keep_work=args.keep_work,
                         log=log, progress=progress)
    _emit(report, args.json)
    return 0 if report.get("ok") else 2


def _cmd_list(args) -> int:
    report = {"ok": True, "splats": list_splats(args.results_dir)}
    _emit(report, args.json)
    return 0


def _cmd_compare(args) -> int:
    from .compare import compare_scan_to_reference   # pulls open3d, not torch

    report = compare_scan_to_reference(
        args.capture, args.reference or None, results_dir=args.results_dir,
        opacity_min=args.opacity_min, voxel=args.voxel, allow_scale=args.allow_scale)
    _emit(report, args.json)
    return 0 if report.get("ok") else 2


def _emit(report: dict, json_path: str | None) -> None:
    text = json.dumps(report, indent=2)
    if json_path:
        Path(json_path).write_text(text + "\n", encoding="utf-8")
    print(text)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="roomscan-splat", description=__doc__)
    ap.add_argument("--results-dir", default=_default_results_dir(),
                    help="where splats are stored (default: ./results)")
    ap.add_argument("--json", default=None, metavar="PATH",
                    help="also write the JSON report to PATH")
    sub = ap.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="reconstruct a splat from a video")
    b.add_argument("video", help="source video (mp4/mov/…)")
    b.add_argument("--name", required=True, help='display name, e.g. "Sam Office"')
    b.add_argument("--force", action="store_true", help="rebuild even if a current splat exists")
    b.add_argument("--keep-work", action="store_true", help="keep the temp frames/COLMAP dir")
    b.add_argument("--progress-file", dest="progress_file", metavar="PATH",
                   help="write phase/fraction progress JSON here (for the web SplatRunner)")
    b.add_argument("--fps", type=float, help="frames sampled per second (preset default)")
    b.add_argument("--max-frames", type=int, dest="max_frames")
    b.add_argument("--long-edge", type=int, dest="long_edge")
    b.add_argument("--matcher", choices=("sequential", "exhaustive"))
    b.add_argument("--sequential-overlap", type=int, dest="sequential_overlap")
    b.add_argument("--sh-degree", type=int, dest="sh_degree")
    b.add_argument("--max-gaussians", type=int, dest="max_gaussians")
    b.add_argument("--iters", type=int)
    b.add_argument("--min-opacity", type=float, dest="min_opacity")
    b.add_argument("--opacity-reg", type=float, dest="opacity_reg")
    b.add_argument("--scale-reg", type=float, dest="scale_reg")
    b.add_argument("--cull-opacity", type=float, dest="cull_opacity")
    b.add_argument("--cull-radius-factor", type=float, dest="cull_radius_factor")
    b.add_argument("--depth-lambda", type=float, dest="depth_lambda",
                   help="monocular depth-prior weight (0=off; anchors gaussians to surfaces)")
    b.add_argument("--depth-model", dest="depth_model",
                   help="HF depth model id for the depth prior (preset default)")
    b.set_defaults(func=_cmd_build)

    ls = sub.add_parser("list", help="list built splats")
    ls.set_defaults(func=_cmd_list)

    cmp = sub.add_parser("compare", help="diff a SLAM reconstruction against a ground-truth splat")
    cmp.add_argument("capture", help="capture name/stem (uses its results/<stem>.ply SLAM mesh)")
    cmp.add_argument("--reference", default=None,
                     help="ground-truth splat .ply or splats/ slug "
                          "(default: auto-select the best imported reference for this capture)")
    cmp.add_argument("--opacity-min", type=float, default=0.5, dest="opacity_min",
                     help="drop gaussians below this opacity (kills floaters; default 0.5)")
    cmp.add_argument("--voxel", type=float, default=0.03,
                     help="downsample voxel size in meters (default 0.03)")
    cmp.add_argument("--allow-scale", action="store_true", dest="allow_scale",
                     help="let ICP estimate scale (diagnostic only; default rigid)")
    cmp.set_defaults(func=_cmd_compare)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
