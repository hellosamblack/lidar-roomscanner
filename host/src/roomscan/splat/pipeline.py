"""End-to-end splat build: video -> frames -> COLMAP -> 3DGS -> results/splats/<slug>.

One synchronous ``build_splat`` that returns a JSON-able report dict.  It is the
single implementation behind both the ``roomscan-splat`` CLI and the
``splat_build`` MCP tool.  Long stages (COLMAP, gsplat training) log progress
through the ``log`` / ``progress`` callbacks; the caller decides whether that
goes to a terminal, a file, or nowhere.

Intermediate frames/COLMAP live in a temp work dir and are discarded on success
unless ``keep_work=True`` -- only the ``.ply`` + ``manifest.json`` are kept.
"""
from __future__ import annotations

import shutil
import tempfile
import time
from pathlib import Path

from .config import SplatPreset
from .frames import extract_frames
from .level import estimate_upright_transform
from .sfm import run_sfm
from .sidecar import (build_manifest, sidecar_paths, sidecar_status, slugify,
                      write_manifest_atomic)


def build_splat(video: str | Path, name: str, results_dir: str | Path, *,
                preset: SplatPreset | None = None, force: bool = False,
                keep_work: bool = False, work_dir: str | Path | None = None,
                log=lambda m: None, progress=lambda phase, f: None) -> dict:
    """Build (or skip if current) the splat for ``video`` named ``name``.

    Returns ``{ok, slug, name, built, reason?, paths, stats, transform, elapsed_s}``.
    ``built`` is False (with ``ok=True``) when a current sidecar already existed
    and ``force`` was not set.
    """
    video = Path(video)
    if not video.is_file():
        return {"ok": False, "reason": f"video not found: {video}"}
    preset = preset or SplatPreset.load()
    slug = slugify(name)
    paths = sidecar_paths(slug, results_dir)
    t0 = time.time()

    status = sidecar_status(slug, results_dir, video=video, preset=preset)
    if status["current"] and not force:
        return {"ok": True, "built": False, "slug": slug, "name": name,
                "reason": "current splat already exists (use --force to rebuild)",
                "paths": {k: str(v) for k, v in paths.items()},
                "stats": (status["manifest"] or {}).get("stats"),
                "transform": (status["manifest"] or {}).get("transform"),
                "elapsed_s": 0.0}

    owns_work = work_dir is None
    work_dir = Path(work_dir) if work_dir else Path(tempfile.mkdtemp(prefix=f"splat-{slug}-"))
    frames_dir = work_dir / "frames"
    try:
        progress("frames", 0.0)
        log(f"[splat] '{name}' <- {video.name}  (work: {work_dir})")
        extract_frames(video, frames_dir, fps=preset.fps, long_edge=preset.long_edge,
                       max_frames=preset.max_frames, log=log)
        progress("frames", 1.0)

        progress("sfm", 0.0)
        sfm_stats = run_sfm(frames_dir, work_dir, matcher=preset.matcher,
                            sequential_overlap=preset.sequential_overlap, log=log)
        progress("sfm", 1.0)
        model_dir = sfm_stats["model_dir"]

        transform = estimate_upright_transform(model_dir)

        train_stats = train_splat_lazy(
            model_dir, frames_dir, paths["ply"], preset=preset,
            log=log, progress=lambda f: progress("train", f))

        stats = {**sfm_stats, **train_stats}
        manifest = build_manifest(name, slug, video, preset, stats=stats, transform=transform)
        write_manifest_atomic(paths["manifest"], manifest)   # commit marker, written LAST
        elapsed = round(time.time() - t0, 1)
        log(f"[splat] done in {elapsed}s: {stats['gaussians']} gaussians")
        return {"ok": True, "built": True, "slug": slug, "name": name,
                "paths": {k: str(v) for k, v in paths.items()},
                "stats": stats, "transform": transform, "elapsed_s": elapsed}
    finally:
        if owns_work and not keep_work:
            shutil.rmtree(work_dir, ignore_errors=True)


def train_splat_lazy(model_dir, image_dir, out_ply, *, preset: SplatPreset, log, progress) -> dict:
    """Import + run the torch/gsplat trainer (kept lazy so the module imports light)."""
    from .train import train_splat
    return train_splat(model_dir, image_dir, out_ply, sh_degree=preset.sh_degree,
                       max_gaussians=preset.max_gaussians, iters=preset.iters,
                       min_opacity=preset.min_opacity, opacity_reg=preset.opacity_reg,
                       scale_reg=preset.scale_reg, cull_opacity=preset.cull_opacity,
                       cull_radius_factor=preset.cull_radius_factor,
                       depth_lambda=preset.depth_lambda, depth_model=preset.depth_model,
                       log=log, progress=progress)
