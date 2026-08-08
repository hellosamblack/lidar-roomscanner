"""Offline Gaussian-splat reconstruction (standalone Phase 7 subset).

Video -> ffmpeg frames -> pycolmap SfM -> gsplat 3DGS -> results/splats/<slug>/.
A *rough* room reconstruction from phone video alone (no ToF fusion / hand-eye
calibration -- that is the blocked full Phase 7 / DC-I).  Rendered navigably in
roomscan-web as the third Live/View/Splat source.

Filesystem/manifest helpers (``sidecar``, ``config``) are torch-free and safe to
import anywhere; ``pipeline.build_splat`` pulls the heavy training stack lazily.
"""
from __future__ import annotations

from .config import SplatPreset
from .sidecar import (delete_splat, list_source_videos, list_splats, sidecar_paths,
                      sidecar_status, slugify, splat_defaults, write_import_manifest)

__all__ = ["SplatPreset", "list_splats", "list_source_videos", "splat_defaults",
           "delete_splat", "sidecar_paths", "sidecar_status", "slugify", "build_splat",
           "compare_scan_to_reference", "choose_reference", "write_import_manifest"]


def build_splat(*args, **kwargs):
    """Lazy proxy to :func:`roomscan.splat.pipeline.build_splat`.

    Kept out of module import so ``import roomscan.splat`` (used by the web server
    and tests for the light helpers) never imports torch/gsplat/pycolmap.
    """
    from .pipeline import build_splat as _impl
    return _impl(*args, **kwargs)


def compare_scan_to_reference(*args, **kwargs):
    """Lazy proxy to :func:`roomscan.splat.compare.compare_scan_to_reference`.

    Lazy so ``import roomscan.splat`` stays cheap -- ``compare`` pulls open3d.
    """
    from .compare import compare_scan_to_reference as _impl
    return _impl(*args, **kwargs)


def choose_reference(*args, **kwargs):
    """Lazy proxy to :func:`roomscan.splat.compare.choose_reference`."""
    from .compare import choose_reference as _impl
    return _impl(*args, **kwargs)
