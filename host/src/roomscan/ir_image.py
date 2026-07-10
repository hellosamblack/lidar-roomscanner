"""IR reflectance panel renderer. Pure array->image math: percentile-based auto-range,
normalization, and colormap lookup. No GUI, no I/O, no Open3D dependency here — the panel
wraps the returned uint8 array in o3d.geometry.Image itself.
"""
from __future__ import annotations

import numpy as np

from .colors import percentile_range, turbo


def ir_range(refl: np.ndarray, lo_pct: float = 2.0, hi_pct: float = 98.0) -> tuple[float, float]:
    """Return (vmin, vmax) as the lo_pct/hi_pct percentiles of the finite values in `refl`.

    Thin wrapper over `colors.percentile_range` (the shared auto-range also used by the
    point-cloud coloring in `shading`) so the IR monitor and the cloud stay in lockstep.
    Degenerate cases return a safe unit-wide window so downstream normalization never
    divides by zero: (0.0, 1.0) when nothing is finite, (v, v + 1.0) when all-equal.
    """
    return percentile_range(refl, lo_pct, hi_pct)


def reflectance_to_rgb(
    refl: np.ndarray,
    *,
    colormap: str = "gray",
    vmin: float | None = None,
    vmax: float | None = None,
    upscale: int = 1,
    lo_pct: float = 2.0,
    hi_pct: float = 98.0,
) -> np.ndarray:
    """Render a (H, W) reflectance array to a (H*upscale, W*upscale, 3) uint8 RGB image.

    Normalization range: if both vmin and vmax are given (panel "freeze range" mode), use
    them as-is; otherwise compute (vmin, vmax) per-call via `ir_range(refl, lo_pct, hi_pct)`.
    Values are clipped to [0, 1] after normalization. Non-finite input values (NaN/inf) map
    to 0.0 (the darkest end), regardless of range. `upscale` does nearest-neighbor block
    replication (np.repeat on both axes) so zones stay crisp — no interpolation.
    """
    if colormap not in ("gray", "turbo"):
        raise ValueError(f"unknown colormap: {colormap!r}")

    arr = np.asarray(refl, dtype=np.float64)
    finite_mask = np.isfinite(arr)

    if vmin is None or vmax is None:
        lo, hi = ir_range(arr, lo_pct, hi_pct)
    else:
        lo, hi = float(vmin), float(vmax)
        if lo == hi:
            hi = lo + 1.0

    norm = np.zeros_like(arr, dtype=np.float64)
    norm[finite_mask] = (arr[finite_mask] - lo) / (hi - lo)
    norm = np.clip(norm, 0.0, 1.0)
    norm[~finite_mask] = 0.0

    if colormap == "gray":
        rgb = np.repeat(norm[..., None], 3, axis=-1)
    else:
        rgb = turbo(norm)

    img = np.clip(np.round(rgb * 255.0), 0, 255).astype(np.uint8)

    upscale = int(upscale)
    if upscale > 1:
        img = np.repeat(img, upscale, axis=0)
        img = np.repeat(img, upscale, axis=1)

    return img
