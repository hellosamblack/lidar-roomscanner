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
