#!/usr/bin/env python3
"""Why is a splat sparse? Sweep SfM configs and report what actually registers.

    host/.venv/bin/python host/tools/splat_sfm_probe.py captures/room.mp4
    host/.venv/bin/python host/tools/splat_sfm_probe.py <video> --configs seq,exhaustive --json

Density has two ceilings -- VRAM (splat_vram_sweep) and REGISTRATION -- and for a
room walkthrough the binding one is usually registration. The default sequential
matcher keeps only the largest connected sub-model; a long video thinned to 300
frames gets large inter-frame baselines, sequential overlap breaks, the
reconstruction fragments, and most frames are discarded (the new Sam Office 2
video registered 16%).

This extracts frames ONCE and runs SfM under several configs on the SAME frames,
reporting per config: registered_ratio, largest_ratio, total_placed_ratio,
n_submodels, points3D, track length, reprojection error, seconds. It recommends
the config maximizing single-connected registration (largest_ratio x points). Each
config calls the real `run_sfm`, so a winning config behaves identically in a build.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
import time
from pathlib import Path

# The built-in configs, each a kwargs dict for roomscan.splat.sfm.run_sfm. Named so
# --configs can pick a subset. "seq" is the current default; the rest are the levers
# most likely to reconnect a fragmented walkthrough on a long capture.
_CONFIGS: dict[str, dict] = {
    "seq":              dict(matcher="sequential", sequential_overlap=10),
    "seq_overlap20":    dict(matcher="sequential", sequential_overlap=20),
    "seq_loop":         dict(matcher="sequential", sequential_overlap=10, loop_detection=True),
    "exhaustive":       dict(matcher="exhaustive"),
    "exhaustive_dsp":   dict(matcher="exhaustive", max_num_features=16384,
                             estimate_affine_shape=True, domain_size_pooling=True),
}


def probe_sfm(video, *, configs=None, fps=0.0, max_frames=0, long_edge=0,
              min_registered=8, keep_work=False, log=lambda m: None) -> dict:
    """Extract frames once, then run each SfM config on them; return a report dict.

    Pure apart from reading the video and writing a temp work dir. Reuses the
    build's `extract_frames` + `run_sfm`, so the measured registration is the real
    pipeline's, not an approximation.
    """
    from roomscan.splat.config import SplatPreset
    from roomscan.splat.frames import extract_frames
    from roomscan.splat.sfm import SfmError, run_sfm

    video = Path(video)
    if not video.is_file():
        return {"ok": False, "error": f"video not found: {video}"}

    names = [c.strip() for c in configs.split(",")] if configs else list(_CONFIGS)
    unknown = [n for n in names if n not in _CONFIGS]
    if unknown:
        return {"ok": False, "error": f"unknown config(s): {unknown}; "
                f"choose from {list(_CONFIGS)}"}

    preset = SplatPreset.load()
    fps = fps or preset.fps
    max_frames = max_frames or preset.max_frames
    long_edge = long_edge or preset.long_edge

    root = Path(tempfile.mkdtemp(prefix="splat-sfmprobe-"))
    frames_dir = root / "frames"
    try:
        frame_paths = extract_frames(video, frames_dir, fps=fps, long_edge=long_edge,
                                     max_frames=max_frames, log=log)
        n_frames = len(frame_paths)
        log(f"[probe] {n_frames} frames extracted; sweeping {names}")

        results = []
        for name in names:
            kw = _CONFIGS[name]
            work = root / name
            work.mkdir(parents=True, exist_ok=True)
            # Feed run_sfm the frames from a private copy so each config's database
            # is isolated (COLMAP writes database.db next to the model in work_dir).
            t0 = time.monotonic()
            row = {"config": name, "params": kw}
            try:
                stats = run_sfm(frames_dir, work, min_registered=min_registered,
                                log=log, **kw)
                row.update({k: stats[k] for k in (
                    "registered_ratio", "largest_ratio", "total_placed_ratio",
                    "n_submodels", "submodel_sizes", "points3D",
                    "mean_track_length", "mean_reprojection_error")})
                row["ok"] = True
            except SfmError as e:
                row.update({"ok": False, "error": str(e)})
            except Exception as e:   # a config the installed pycolmap can't run (e.g.
                row.update({"ok": False, "error": f"{type(e).__name__}: {e}"})  # no vocab tree)
            row["seconds"] = round(time.monotonic() - t0, 1)
            log(f"[probe] {name}: {'ok' if row['ok'] else 'FAIL'} "
                f"largest_ratio={row.get('largest_ratio')} points={row.get('points3D')} "
                f"({row['seconds']}s)")
            results.append(row)

        best = _pick_best(results)
        baseline = next((r for r in results if r["config"] == "seq"), None)
        rep = {
            "ok": True, "video": str(video), "n_frames": n_frames,
            "fps": fps, "max_frames": max_frames, "long_edge": long_edge,
            "configs": results,
            "recommended": best["config"] if best else None,
            "recommendation_note": _reco_note(best, baseline),
        }
        return rep
    finally:
        if not keep_work:
            shutil.rmtree(root, ignore_errors=True)
        else:
            log(f"[probe] work kept at {root}")


def _pick_best(results):
    """The config that registers the most single-connected structure: the ok row
    with the largest largest_ratio x points3D. Pure over the per-config rows so the
    reduce logic is testable without COLMAP. None if nothing registered."""
    ok_rows = [r for r in results if r.get("ok")]
    return max(ok_rows, key=lambda r: r["largest_ratio"] * r["points3D"], default=None)


def _reco_note(best, baseline):
    if best is None:
        return "no config registered a usable model."
    if baseline is None or not baseline.get("ok"):
        return (f"{best['config']}: largest_ratio {best['largest_ratio']}, "
                f"{best['points3D']} points.")
    lift = best["largest_ratio"] - baseline["largest_ratio"]
    frag = (f" baseline fragmented into {baseline['n_submodels']} sub-models "
            f"({baseline['submodel_sizes']})" if baseline["n_submodels"] > 1 else "")
    return (f"{best['config']} registers {best['largest_ratio']:.0%} vs the current "
            f"sequential default's {baseline['largest_ratio']:.0%} "
            f"({lift:+.0%}); {best['points3D']} vs {baseline['points3D']} points.{frag}")


def _print_report(rep: dict) -> None:
    if not rep.get("ok"):
        print(f"error: {rep.get('error', rep)}", file=sys.stderr)
        return
    print(f"video: {Path(rep['video']).name}   {rep['n_frames']} frames "
          f"(fps={rep['fps']}, long_edge={rep['long_edge']})")
    print(f"{'config':>16} {'reg':>6} {'largest':>8} {'placed':>7} {'subs':>5} "
          f"{'points':>8} {'track':>6} {'reproj':>7} {'s':>6}")
    for r in rep["configs"]:
        if r["ok"]:
            print(f"{r['config']:>16} {r['registered_ratio']:>6} {r['largest_ratio']:>8} "
                  f"{r['total_placed_ratio']:>7} {r['n_submodels']:>5} {r['points3D']:>8} "
                  f"{r['mean_track_length']:>6} {r['mean_reprojection_error']:>7} {r['seconds']:>6}")
        else:
            print(f"{r['config']:>16}  FAIL: {r['error'][:60]} ({r['seconds']}s)")
    print(f"\nrecommended: {rep['recommended']}")
    print(f"  {rep['recommendation_note']}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("video", help="source video (mp4/mov/…)")
    ap.add_argument("--configs", default="", help=f"comma-separated subset of {list(_CONFIGS)}")
    ap.add_argument("--fps", type=float, default=0.0, help="frames/sec (preset default if 0)")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--long-edge", type=int, default=0)
    ap.add_argument("--keep-work", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    def log(m):
        print(m, file=sys.stderr, flush=True)

    rep = probe_sfm(args.video, configs=args.configs, fps=args.fps,
                    max_frames=args.max_frames, long_edge=args.long_edge,
                    keep_work=args.keep_work, log=log)
    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        _print_report(rep)
    return 0 if rep.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
