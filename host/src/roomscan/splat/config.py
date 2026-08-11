"""Resolved Gaussian-splat reconstruction preset.

Mirrors ``slam/config.py``'s ``DetailedSlamPreset``: the values that decide what
a splat *contains* are fingerprinted so a stale reconstruction can be detected;
a purely cosmetic ``note`` is excluded so editing it never marks every existing
splat stale.  Loaded from the shared ``roomscan.toml`` ``[splat]`` table.
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from ..config import config_path


@dataclass
class SplatPreset:
    """Knobs for the video -> COLMAP -> 3DGS pipeline.

    Defaults are tuned for a handheld room walkthrough on an 8 GB GPU: enough
    frames for COLMAP to register a textured indoor scene, a gaussian cap that
    fits VRAM, and an iteration count that resolves a *rough* room in minutes
    rather than the 30 k the papers use for benchmark PSNR.
    """
    fps: float = 3.0                 # frames sampled per second of video
    max_frames: int = 300            # hard cap after fps sampling (COLMAP is O(n^2)-ish)
    long_edge: int = 1600            # downscale so the long image edge is this many px
    # "exhaustive" (all frame pairs) | "sequential" (adjacent-only, O(n) but fragile).
    # Measured 2026-08-08 (splat_sfm_probe on the new Sam Office 2 video): a long
    # walkthrough thinned to 300 frames has large inter-frame baselines, so sequential
    # overlap breaks and COLMAP fragments the scene into disconnected sub-models,
    # keeping only the largest -- 18% of frames registered. Exhaustive reconnects it:
    # 70% registered, 4.6x the seed points. At the 300-frame cap the O(n^2) cost is
    # ~8.6 min (vs 3.6), trivial against a 30-45 min build; drop to "sequential" only
    # if max_frames is raised far past 300.
    matcher: str = "exhaustive"
    sequential_overlap: int = 10     # sequential matcher: match each frame to the next N
    sh_degree: int = 3               # spherical-harmonic bands (view-dependent colour)
    # MCMC cap; the VRAM governor on this 8 GB box. Lowered 2M->1M (2026-08-08): 2M
    # OOMs the real trainer (splat-vram-ceiling), and splat_vram_sweep's clone-at-N
    # measurement is a ~2x-2.8x LOWER BOUND on the real training peak (worse with the
    # depth prior's RGB+ED channel -- a 1.3M depth build OOM'd isolated at 6.59 GiB).
    # 1M non-depth fits with margin; depth or a larger room want an even lower cap.
    max_gaussians: int = 1_000_000
    iters: int = 15_000              # training steps
    # Anti-"snowglobe" controls (BUG-094 follow-up). Photometry-only 3DGS from a
    # single video leaves most gaussians as faint, oversized floaters (measured:
    # 74% below 0.1 opacity, 16x larger than a LiDAR-fused Scaniverse capture).
    min_opacity: float = 0.05        # MCMC prunes below this every refine step (was 0.005)
    opacity_reg: float = 0.01        # MCMC opacity regulariser (fewer, more decisive splats)
    scale_reg: float = 0.02          # MCMC scale regulariser (smaller, surface-hugging)
    cull_opacity: float = 0.12       # post-train: drop gaussians fainter than this
    cull_radius_factor: float = 3.0  # post-train: drop gaussians beyond this x the camera-path radius
    # Monocular depth prior (Depth-Anything-V2): the "poor man's LiDAR". >0 turns
    # on a scale/shift-invariant depth-regularisation loss that anchors gaussians
    # to a plausible surface, collapsing the translucent floater volume. 0 = off.
    depth_lambda: float = 0.0
    depth_model: str = "depth-anything/Depth-Anything-V2-Small-hf"
    note: str = "rough room reconstruction from phone video (no ToF fusion)"

    # `note` documents intent, it does not change the geometry -- excluded from
    # the fingerprint for the same reason DetailedSlamPreset excludes its timing
    # fields: a staleness flag that fires on a comment edit is one users learn to
    # ignore, and then it cannot warn about a real change.
    _NON_RECONSTRUCTION_FIELDS = ("note",)

    @classmethod
    def load(cls, path: Optional[Path] = None) -> "SplatPreset":
        path = Path(path) if path is not None else config_path()
        try:
            raw = tomllib.loads(path.read_text(encoding="utf-8"))
            table = raw.get("splat", {})
        except (OSError, tomllib.TOMLDecodeError, AttributeError):
            return cls()
        if not isinstance(table, dict):
            return cls()
        known = {f.name for f in dataclasses.fields(cls)}
        try:
            return cls(**{k: v for k, v in table.items() if k in known})
        except (TypeError, ValueError):
            return cls()

    def fingerprint(self) -> str:
        payload = {k: v for k, v in dataclasses.asdict(self).items()
                   if k not in self._NON_RECONSTRUCTION_FIELDS}
        return hashlib.sha256(json.dumps(payload, sort_keys=True,
                                         separators=(",", ":")).encode()).hexdigest()[:16]
