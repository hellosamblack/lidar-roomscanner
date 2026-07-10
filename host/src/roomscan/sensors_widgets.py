"""Numpy-drawn panel widgets for the LSM6DSV16X sensors: a compass dial and a sparkline.
Pure functions producing (H, W, 3) uint8 RGB images, fed to Open3D gui.ImageWidget --
same role as ir_image.reflectance_to_rgb for the IR monitor."""
from __future__ import annotations

import numpy as np

_BG = (24, 24, 28)
_FG = (220, 220, 230)
_ACCENT = (240, 120, 90)


def _blank(h: int, w: int, color: tuple[int, int, int]) -> np.ndarray:
    img = np.empty((h, w, 3), dtype=np.uint8)
    img[:, :] = color
    return img


def _line(img: np.ndarray, x0: float, y0: float, x1: float, y1: float,
          color: tuple[int, int, int]) -> None:
    """Draw an anti-alias-free line (Bresenham-ish via sampling) into img in place."""
    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    xs = np.linspace(x0, x1, n).round().astype(int)
    ys = np.linspace(y0, y1, n).round().astype(int)
    h, w = img.shape[:2]
    ok = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
    img[ys[ok], xs[ok]] = color


def render_compass(heading_deg: float, size: int = 180) -> np.ndarray:
    img = _blank(size, size, _BG)
    cx = cy = size / 2.0
    r = size * 0.42
    # dial ring
    theta = np.linspace(0, 2 * np.pi, 180)
    xs = (cx + r * np.cos(theta)).round().astype(int)
    ys = (cy + r * np.sin(theta)).round().astype(int)
    img[np.clip(ys, 0, size - 1), np.clip(xs, 0, size - 1)] = _FG
    # needle: heading 0 = up (+screen -Y = north), clockwise
    a = np.radians(heading_deg)
    tipx = cx + r * 0.9 * np.sin(a)
    tipy = cy - r * 0.9 * np.cos(a)
    _line(img, cx, cy, tipx, tipy, _ACCENT)
    return img


def render_sparkline(values: np.ndarray, width: int = 220, height: int = 60, *,
                     label: str = "", unit: str = "") -> np.ndarray:
    img = _blank(height, width, _BG)
    v = np.asarray(values, dtype=np.float64)
    if v.size < 2:
        img[height // 2, :] = _FG  # flat baseline
        return img
    lo, hi = float(v.min()), float(v.max())
    span = hi - lo if hi > lo else 1.0
    xs = np.linspace(2, width - 3, v.size)
    ys = height - 3 - (v - lo) / span * (height - 6)
    for i in range(v.size - 1):
        _line(img, xs[i], ys[i], xs[i + 1], ys[i + 1], _ACCENT)
    return img
