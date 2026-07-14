"""Rendered 'instrument' cards for the Showcase reveal -- (H, W, 3) uint8 RGB
images shown over the 3D scene via an Open3D gui.ImageWidget, same technique as
metrics_hud.render_hud and sensors_widgets.

Why an image, not a gui.Label: Open3D's gui font can't set tracked eyebrows,
tabular mono figures, or the corner-tick frame that makes the whole scanner
read as one instrument -- so the FINAL "scan complete" moment is drawn with
Pillow instead of printed as a system-font debug string.

The accent cyan is sourced from `theme.ACCENT` so the card, the trajectory
ribbon, the capture beam, and the floor grid are all the same signal color.
Pure -- unit-tested (test_cards.py); no Open3D, no I/O.
"""
from __future__ import annotations

import numpy as np

from . import theme
from .instrument import (
    ACCENT_U8 as _ACCENT,
    HAIR as _HAIR,
    MUTED as _MUTED,
    PANEL as _PANEL,
    TEXT as _TEXT,
    corner_ticks as _corner_ticks,
    fmt_count,
    load_font as _font,
    tracked_text as _tracked,
    u8 as _u8,
)


def render_scan_complete_card(frames: int, drift_m: float, verts: int,
                              elapsed_s: float, *, width: int = 540) -> np.ndarray:
    """(H, W, 3) uint8 RGB 'SCAN COMPLETE' card: a cyan-eyebrow lower-third with
    a row of labeled tabular-mono stats (frames, drift, verts, time) and the
    corner-tick instrument frame. Pure -- unit-tested."""
    from PIL import Image, ImageDraw

    height = 96
    img = Image.new("RGB", (width, height), _PANEL)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, width - 1, height - 1], outline=_HAIR)
    _corner_ticks(d, 0, 0, width - 1, height - 1, _ACCENT)

    pad = 22
    _tracked(d, (pad, 16), "SCAN COMPLETE", _font(13, bold=True), _ACCENT)

    stats = [
        ("FRAMES", str(int(frames))),
        ("DRIFT", f"{drift_m:.2f} m"),
        ("VERTS", fmt_count(verts)),
        ("TIME", f"{elapsed_s:.1f} s"),
    ]
    lbl_font = _font(11)
    val_font = _font(18, mono=True, bold=True)
    col_w = (width - 2 * pad) / len(stats)
    for i, (lab, val) in enumerate(stats):
        cx = pad + i * col_w
        d.text((cx, 46), lab, font=lbl_font, fill=_MUTED)
        d.text((cx, 60), val, font=val_font, fill=_TEXT)
    return np.asarray(img, dtype=np.uint8)
