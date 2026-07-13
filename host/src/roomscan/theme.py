"""The 'depth-scope' instrument palette + pure stage helpers for the 3D scene
and the SLAM/Showcase experience.

Single source of truth for the colors and geometry that make the scene read as
a precision optical instrument rather than dots in a void:

  * a graded background (`vertical_gradient`) instead of a flat fill,
  * a grounded floor grid (`floor_grid_lines`) that establishes scale + horizon,
  * a trajectory ramp (`trajectory_ramp`) that fades old->new so motion reads.

Colors are RGB floats in [0, 1] (Open3D geometry/material convention). Every
helper here is pure and unit-tested (test_theme.py) -- no Open3D, no I/O -- so
the panel can stay presentation-only and the look is verifiable without a window.
"""
from __future__ import annotations

import numpy as np

# ---- palette (RGB floats, Open3D convention) -------------------------------
# Chrome is monochrome cyan (the "live ToF" signal); data stays chromatic.
ACCENT = (0.18, 0.88, 0.82)            # live cyan -- capture beam, trajectory head
FLOOR_GRID = (0.12, 0.35, 0.42)        # faint cyan-teal ground grid
TRAJ_NEW = (0.18, 0.88, 0.82)          # trajectory: most-recent segment (bright)
TRAJ_OLD = (0.05, 0.15, 0.18)          # trajectory: oldest segment (dim)

# Graded background stops (top -> bottom). Dark is the default "stage"; light is
# the toggle. A vertical gradient reads as a horizon and gives the scene depth.
STAGE_TOP_DARK = (0.15, 0.19, 0.29)
STAGE_BOTTOM_DARK = (0.02, 0.03, 0.06)
STAGE_TOP_LIGHT = (0.92, 0.93, 0.96)
STAGE_BOTTOM_LIGHT = (0.82, 0.84, 0.88)

# Background clear color (RGBA) paired with each gradient image -- Open3D wants a
# solid color too; match the gradient's midpoint so any unpainted pixel blends in.
BG_CLEAR_DARK = [0.05, 0.07, 0.11, 1.0]
BG_CLEAR_LIGHT = [0.87, 0.88, 0.92, 1.0]


def vertical_gradient(width: int, height: int, top, bottom) -> np.ndarray:
    """(height, width, 3) uint8 RGB image: a smooth vertical gradient from
    `top` at row 0 to `bottom` at the last row. `top`/`bottom` are RGB floats
    in [0, 1]. Used as the scene's background image so the void becomes a graded
    horizon. Pure -- unit-tested."""
    width = max(1, int(width))
    height = max(1, int(height))
    top = np.asarray(top, dtype=np.float64)
    bottom = np.asarray(bottom, dtype=np.float64)
    t = np.linspace(0.0, 1.0, height)[:, None]           # (H, 1)
    rows = (1.0 - t) * top[None, :] + t * bottom[None, :]  # (H, 3)
    img = np.broadcast_to(rows[:, None, :], (height, width, 3))
    return np.clip(np.rint(img * 255.0), 0, 255).astype(np.uint8)


def _vertical_axis(up: np.ndarray) -> tuple[int, float]:
    """(axis_index, sign) of the dominant component of `up`. sign is +1/-1 so
    that `sign * coord` increases in the physical-up direction."""
    up = np.asarray(up, dtype=np.float64)
    axis = int(np.argmax(np.abs(up)))
    sign = -1.0 if up[axis] < 0 else 1.0   # up=[0,-1,0] (y-down) -> smaller y is higher
    return axis, sign


def floor_grid_lines(min_bound, max_bound, up=None, spacing: float = 0.5,
                     pad: float = 0.5):
    """(points (N,3) float64, lines (M,2) int64) for a floor grid spanning the
    horizontal extent of an axis-aligned box, placed at the box's FLOOR (the
    physically-lowest plane along `up`).

    `up` defaults to Open3D CV world-up `[0,-1,0]` (y-down), so the floor sits
    at the box's MAX vertical coordinate. `spacing` is the grid pitch (metres);
    `pad` extends the grid past the box so the room doesn't sit on the very edge
    of the plane. Returns empty arrays for a degenerate (zero-extent) box. Pure
    -- unit-tested; feeds the panel's floor-grid LineSet."""
    if up is None:
        up = np.array([0.0, -1.0, 0.0])
    lo = np.asarray(min_bound, dtype=np.float64).copy()
    hi = np.asarray(max_bound, dtype=np.float64).copy()
    axis, sign = _vertical_axis(up)
    # floor = the extreme that is lowest physically (max coord when sign<0)
    floor = hi[axis] if sign < 0 else lo[axis]
    ax = [i for i in range(3) if i != axis]          # the two horizontal axes
    # Degeneracy is judged on the UNPADDED box: a zero-extent box has no room to
    # ground and shouldn't be rescued into a grid by `pad`.
    if spacing <= 0 or hi[ax[0]] - lo[ax[0]] <= 0 or hi[ax[1]] - lo[ax[1]] <= 0:
        return np.zeros((0, 3)), np.zeros((0, 2), dtype=np.int64)
    a0lo, a0hi = lo[ax[0]] - pad, hi[ax[0]] + pad
    a1lo, a1hi = lo[ax[1]] - pad, hi[ax[1]] + pad
    us = np.arange(a0lo, a0hi + spacing, spacing)
    vs = np.arange(a1lo, a1hi + spacing, spacing)
    pts: list[list[float]] = []
    lines: list[list[int]] = []

    def _pt(u, v):
        p = [0.0, 0.0, 0.0]
        p[axis] = floor
        p[ax[0]] = u
        p[ax[1]] = v
        return p

    for u in us:                                     # lines parallel to axis ax[1]
        i = len(pts)
        pts.append(_pt(u, a1lo)); pts.append(_pt(u, a1hi))
        lines.append([i, i + 1])
    for v in vs:                                     # lines parallel to axis ax[0]
        i = len(pts)
        pts.append(_pt(a0lo, v)); pts.append(_pt(a0hi, v))
        lines.append([i, i + 1])
    return np.asarray(pts, dtype=np.float64), np.asarray(lines, dtype=np.int64)


def trajectory_ramp(n: int, old=TRAJ_OLD, new=TRAJ_NEW,
                    floor: float = 0.2) -> np.ndarray:
    """(n, 3) float64 per-segment colors fading `old` (dimmest, at `floor`
    brightness of the ramp) up to `new` (segment n-1, the most recent). `n` is
    the number of segments (== trajectory points - 1). Empty for n <= 0. Pure
    -- unit-tested; replaces the flat lime debug line."""
    if n <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    old = np.asarray(old, dtype=np.float64)
    new = np.asarray(new, dtype=np.float64)
    f = np.linspace(floor, 1.0, n)[:, None]          # (n, 1)
    return f * new[None, :] + (1.0 - f) * old[None, :]
