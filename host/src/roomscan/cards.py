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

# card chrome (RGB 0-255) -- the 2D counterpart to theme's 3D scene palette
_PANEL = (16, 19, 26)
_HAIR = (46, 52, 64)
_TEXT = (232, 234, 242)
_MUTED = (126, 132, 148)


def _u8(rgb_float) -> tuple[int, int, int]:
    return tuple(int(round(c * 255)) for c in rgb_float)


_ACCENT = _u8(theme.ACCENT)


def _font(size: int, *, mono: bool = False, bold: bool = False):
    from PIL import ImageFont
    if mono:
        cands = ["C:/Windows/Fonts/consolab.ttf" if bold else "C:/Windows/Fonts/consola.ttf",
                 "DejaVuSansMono.ttf"]
    else:
        cands = ["C:/Windows/Fonts/seguisb.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf",
                 "DejaVuSans.ttf"]
    for c in cands:
        try:
            return ImageFont.truetype(c, size)
        except Exception:
            continue
    try:
        return ImageFont.load_default(size)
    except Exception:
        return ImageFont.load_default()


def _tracked(draw, xy, text, font, fill, spacing=3):
    """Draw `text` with extra inter-letter spacing (an 'eyebrow' look Pillow
    can't do via a font feature). Returns the x cursor after the last glyph."""
    x, y = xy
    for ch in text:
        draw.text((x, y), ch, font=font, fill=fill)
        x += draw.textlength(ch, font=font) + spacing
    return x


def fmt_count(n: int) -> str:
    """Compact integer: 273789 -> '273k', 1_200_000 -> '1.2M', <1000 -> as-is.
    Pure -- unit-tested."""
    n = int(n)
    if abs(n) >= 1_000_000:
        return f"{n / 1_000_000:.1f}M"
    if abs(n) >= 1_000:
        return f"{n // 1000}k"
    return str(n)


def _corner_ticks(d, x0, y0, x1, y1, color, ln=11, inset=6, width=2):
    x0 += inset; y0 += inset; x1 -= inset; y1 -= inset
    for cx, cy, dx, dy in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
        d.line([cx, cy, cx + dx * ln, cy], fill=color, width=width)
        d.line([cx, cy, cx, cy + dy * ln], fill=color, width=width)


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
