"""Cloud coloring with an optional near-target contrast boost.

The default point-cloud coloring normalizes the chosen value (z-depth or an aux
plane) linearly across the WHOLE frame's range. When the scene is a person in
front of a wall, the person occupies a thin slice of that range, so facial relief
(a few cm of depth) maps to almost no color change. `cloud_colors` adds three
opt-in ways to spend more of the colormap on close targets (see the panel's
"Near contrast" control):

- ``window``   (default when enabled): color only the near points (z <= cutoff)
  and spread the full colormap across *their* range; render everything beyond the
  cutoff (the wall) flat grey so the subject pops out. Works for any color value.
- ``emphasis``: a gamma that expands the near (low) end of the normalized value
  across more of the colormap while the far end stays compressed-but-colored.
- ``equalize``: per-frame histogram equalization (CDF rank) — dense surfaces (the
  person's face, many points over a small depth span) auto-stretch; flat regions
  compress. No tuning.
- ``off``: the original linear normalize.

The value being colored (``vals``) and the per-point depth (``z_m``, metres, used
for the window cutoff) are passed separately so windowing works even when
coloring by reflectance/confidence rather than depth.
"""
from __future__ import annotations

import numpy as np

from .colors import turbo

# Muted grey for beyond-cutoff points in window mode -- reads as "background",
# lets the colored near target stand out.
FAR_GREY = (0.30, 0.30, 0.34)

MODES = ("off", "window", "emphasis", "equalize")


def _norm(vals: np.ndarray) -> np.ndarray:
    """Linear normalize to [0, 1] with a flat-field divide guard."""
    return (vals - vals.min()) / max(float(np.ptp(vals)), 1e-6)


def cloud_colors(vals, z_m, *, mode: str = "off", cutoff_m: float = 1.5,
                 emphasis: float = 0.5) -> np.ndarray:
    """Map per-point ``vals`` to (N, 3) turbo RGB in [0, 1], per ``mode``.

    ``vals``: the (N,) values to colorize (z-depth for depth mode, else the aux
    plane, already filtered to the valid/deprojected points).
    ``z_m``: (N,) per-point depth in metres, aligned to ``vals`` (used only by
    ``window`` for the cutoff). For depth-mode coloring ``vals`` and ``z_m`` are
    the same array.
    ``cutoff_m``: window-mode near/far boundary (metres).
    ``emphasis``: emphasis-mode strength in [0, 1]; 0 = linear, 1 = strongest
    near boost.
    """
    vals = np.asarray(vals, dtype=np.float64)
    n = vals.shape[0]
    if n == 0:
        return np.zeros((0, 3))

    if mode == "window":
        z = np.asarray(z_m, dtype=np.float64)
        near = z <= float(cutoff_m)
        colors = np.empty((n, 3), dtype=np.float64)
        if near.any():
            colors[near] = turbo(_norm(vals[near]))   # full colormap over the near target only
        colors[~near] = FAR_GREY
        return colors

    if mode == "emphasis":
        # p < 1 lifts small (near) normalized values, spreading them across more
        # of the colormap; the far end stays near 1.0. s=0 -> p=1 (linear).
        p = 1.0 - 0.7 * float(np.clip(emphasis, 0.0, 1.0))
        return turbo(np.power(_norm(vals), p))

    if mode == "equalize":
        # CDF rank: fraction of points <= each value. Ties share a rank (a flat
        # wall stays one color); dense gradients get stretched.
        order = np.sort(vals)
        vn = np.searchsorted(order, vals, side="right").astype(np.float64) / n
        return turbo(vn)

    # off / unknown -> original linear behavior
    return turbo(_norm(vals))
