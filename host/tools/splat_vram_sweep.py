#!/usr/bin/env python3
"""How dense a splat fits on this 8 GB card? Measure it on the real capture.

    host/.venv/bin/python host/tools/splat_vram_sweep.py captures/room.mp4
    host/.venv/bin/python host/tools/splat_vram_sweep.py --model-dir <colmap> --image-dir <frames> --json

Sweeps MCMC ``cap_max`` on the REAL COLMAP model + REAL frames -- the discredited
synthetic probe under-reported the true worst-frame peak by ~2x -- and reports the
largest gaussian count that fits under a VRAM budget. See roomscan.splat.vram for
the two safety biases (count forced to exactly N; conservative untrained scales)
and the honest caveats it returns.

A low ``registered_ratio`` means the scene is capture-limited: the VRAM cap is not
the binding constraint and a higher cap adds no gaussians -- fix SfM registration
(splat_sfm_probe) instead.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _parse_ladder(s: str) -> list[int] | None:
    if not s:
        return None
    return [int(x) for x in s.replace(" ", "").split(",") if x]


def _print_report(rep: dict) -> None:
    if not rep.get("ok"):
        print(f"error: {rep.get('error', rep)}", file=sys.stderr)
        return
    sc = rep["scene"]
    print(f"device: {rep['device']}   budget: {rep['budget_gib']} GiB "
          f"({rep['budget_source']}; total {rep['total_gib']}, margin {rep['margin_gib']}, "
          f"reserve {rep['reserve_gib']})")
    print(f"scene: {sc['n_views']} views, {sc['n_colmap_points']} pts, "
          f"long_edge={sc['long_edge']}, sh={sc['sh_degree']}, mode={sc['render_mode']}, "
          f"registered_ratio={sc['registered_ratio']}")
    print(f"measured peak x safety_factor {rep['safety_factor']} = estimated real training peak "
          f"(clone-at-N under-measures the trained build)")
    print(f"{'n':>10} {'alloc':>8} {'reserved':>9} {'nvml':>7} {'est_real':>9} {'fit':>4} {'oom':>4} {'s':>6}")
    for r in rep["ladder"]:
        print(f"{r['n']:>10} {_g(r['peak_alloc_gib']):>8} {_g(r['peak_reserved_gib']):>9} "
              f"{_g(r['peak_nvml_gib']):>7} {_g(r.get('estimated_real_peak_gib')):>9} "
              f"{str(r['fit']):>4} {str(r['oom']):>4} {r['seconds']:>6}")
    print(f"\nrecommended_cap: {rep['recommended_cap']} "
          f"(est real peak {rep['recommended_est_real_peak_gib']} GiB); "
          f"effective_ceiling: {rep['effective_ceiling']}; "
          f"capture_limited: {rep['capture_limited']}")
    for w in rep.get("warnings", []):
        print(f"  ! {w}")
    for c in rep.get("caveats", []):
        print(f"  - {c}")


def _g(v):
    return "-" if v is None else f"{v:.2f}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("video", nargs="?", help="source video (runs frames+SfM once)")
    ap.add_argument("--model-dir", help="existing COLMAP model dir (skip SfM)")
    ap.add_argument("--image-dir", help="frames dir matching --model-dir")
    ap.add_argument("--budget-gib", type=float, default=None,
                    help="VRAM budget (default: NVML total - margin - reserve)")
    ap.add_argument("--margin-gib", type=float, default=0.8,
                    help="headroom for the CUDA context + fragmentation (default 0.8)")
    ap.add_argument("--reserve-gib", type=float, default=0.0,
                    help="headroom for a co-resident process (default 0 = isolated)")
    ap.add_argument("--ladder", default="", help="comma-separated counts, e.g. 500000,1000000,2000000")
    ap.add_argument("--sh-degree", type=int, default=3)
    ap.add_argument("--long-edge", type=int, default=0,
                    help="frame long edge for the from-video path (preset default if 0)")
    ap.add_argument("--depth-lambda", type=float, default=0.0,
                    help=">0 measures a depth build (RGB+ED render costs extra VRAM)")
    ap.add_argument("--worst-k", type=int, default=0,
                    help="measure only the k nearest-depth views (optimistic-risk; 0 = all)")
    ap.add_argument("--safety-factor", type=float, default=2.0,
                    help="scale the measured clone peak to estimate the real training peak "
                         "(clone-at-N under-measures ~2x; default 2.0)")
    ap.add_argument("--no-refine", action="store_true", help="skip boundary bisection")
    ap.add_argument("--keep-work", action="store_true", help="keep the temp frames/COLMAP dir")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    from roomscan.splat.vram import sweep_vram, sweep_vram_from_video

    def log(m):
        print(m, file=sys.stderr, flush=True)

    common = dict(budget_gib=args.budget_gib, margin_gib=args.margin_gib,
                  reserve_gib=args.reserve_gib, ladder=_parse_ladder(args.ladder),
                  refine=not args.no_refine, sh_degree=args.sh_degree,
                  depth_lambda=args.depth_lambda, worst_k=args.worst_k,
                  safety_factor=args.safety_factor, log=log)

    if args.model_dir:
        if not args.image_dir:
            print("--model-dir needs --image-dir", file=sys.stderr)
            return 1
        rep = sweep_vram(args.model_dir, args.image_dir, **common)
    elif args.video:
        vid = Path(args.video)
        if not vid.is_file():
            print(f"no such video: {vid}", file=sys.stderr)
            return 1
        from roomscan.splat.config import SplatPreset
        preset = SplatPreset.load()
        if args.long_edge:
            preset.long_edge = args.long_edge
        if args.depth_lambda:
            preset.depth_lambda = args.depth_lambda
        rep = sweep_vram_from_video(vid, keep_work=args.keep_work, preset=preset, **common)
    else:
        print("give a video, or --model-dir + --image-dir", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        _print_report(rep)
    return 0 if rep.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
