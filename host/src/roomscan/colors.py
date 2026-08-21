"""Depth colormap. Turbo polynomial approximation (Google AI blog, 2019) — perceptually
ordered blue->green->yellow->red, better shape cues than a naive RGB lerp."""
from __future__ import annotations

import numpy as np

_R = (0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943)
_G = (0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604)
_B = (0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973)


def _poly(x: np.ndarray, c) -> np.ndarray:
    return c[0] + x * (c[1] + x * (c[2] + x * (c[3] + x * (c[4] + x * c[5]))))


def turbo(zn: np.ndarray) -> np.ndarray:
    """Map values in [0, 1] to (N, 3) RGB in [0, 1]. Input is clipped."""
    x = np.clip(np.asarray(zn, dtype=np.float64), 0.0, 1.0)
    rgb = np.stack([_poly(x, _R), _poly(x, _G), _poly(x, _B)], axis=-1)
    return np.clip(rgb, 0.0, 1.0)


def gray(zn: np.ndarray) -> np.ndarray:
    """Map values in [0, 1] to (N, 3) grayscale RGB in [0, 1]."""
    x = np.clip(np.asarray(zn, dtype=np.float64), 0.0, 1.0)
    return np.stack([x, x, x], axis=-1)


#: The CrowPanel firmware's IR-feed thermal ramp, as an exact 256-entry LUT.
#: This is a byte-for-byte port of `ir_thermal_rgb565`'s pre-565 RGB
#: (`CrowPanelProp/main/prop_ui.c`): black -> red -> yellow -> white, with the
#: two knees at 96 and 192 and the same integer divisors (95, 95, 63). We index
#: this LUT rather than evaluate a continuous ramp so the thin voxel view the
#: host renders matches the CrowPanel's own IR tone at *every* level, not just
#: the endpoints. (The panel further quantises to RGB565 on its own display; we
#: keep 8-bit here since the thin frame carries full RGB.)
_AMBER_LUT: np.ndarray | None = None


def _amber_lut() -> np.ndarray:
    global _AMBER_LUT
    if _AMBER_LUT is None:
        v = np.arange(256, dtype=np.int64)
        r = np.where(v < 96, v * 255 // 95, 255)
        g = np.where(v < 96, 0, np.where(v < 192, (v - 96) * 255 // 95, 255))
        b = np.where(v < 192, 0, (v - 192) * 255 // 63)
        _AMBER_LUT = np.clip(np.stack([r, g, b], axis=-1), 0, 255).astype(np.uint8)
    return _AMBER_LUT


def amber(zn: np.ndarray) -> np.ndarray:
    """Map values in [0, 1] to (N, 3) RGB in [0, 1] via the CrowPanel IR ramp.

    Black -> red -> yellow -> white, matching the CrowPanel's IR feed so the
    thin voxel view reads as the same instrument. Input is clipped; the scalar
    is quantised to the 8-bit ramp index (`_amber_lut`) so a given normalised
    depth lands on the exact colour the panel would show for that intensity."""
    x = np.clip(np.asarray(zn, dtype=np.float64), 0.0, 1.0)
    idx = np.clip(np.rint(x * 255.0).astype(np.int64), 0, 255)
    return _amber_lut()[idx].astype(np.float64) / 255.0


def percentile_range(vals: np.ndarray, lo_pct: float = 2.0,
                     hi_pct: float = 98.0) -> tuple[float, float]:
    """Return (vmin, vmax) as the lo_pct/hi_pct percentiles of the finite values.

    Outlier-robust: a few extreme values (specular reflectance returns, stray far
    points) sit outside the window instead of stretching it. Degenerate cases return
    a safe unit-wide window so downstream normalization never divides by zero:
    (0.0, 1.0) when nothing is finite, or (v, v + 1.0) when every finite value is
    identical. This is the single source of truth for auto-ranging shared by the IR
    monitor (`ir_image`) and the point-cloud coloring (`shading`)."""
    arr = np.asarray(vals, dtype=np.float64)
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        return (0.0, 1.0)
    vmin, vmax = np.percentile(finite, [lo_pct, hi_pct])
    vmin = float(vmin)
    vmax = float(vmax)
    if vmin == vmax:
        return (vmin, vmin + 1.0)
    return (vmin, vmax)


def normalize(vals: np.ndarray, lo_pct: float = 2.0,
              hi_pct: float = 98.0) -> np.ndarray:
    """Percentile-clipped linear stretch of `vals` to [0, 1].

    The lo_pct/hi_pct percentiles of the finite values map to 0 and 1; values beyond
    saturate. Non-finite inputs map to 0.0. Unlike a raw min/max normalize this keeps
    a few outliers from crushing the mid-range contrast — the reason the IR monitor
    shows facial/picture-frame detail. Use this, not min/max, for any value→colormap."""
    arr = np.asarray(vals, dtype=np.float64)
    lo, hi = percentile_range(arr, lo_pct, hi_pct)
    out = np.zeros_like(arr, dtype=np.float64)
    finite = np.isfinite(arr)
    out[finite] = (arr[finite] - lo) / (hi - lo)
    return np.clip(out, 0.0, 1.0)
