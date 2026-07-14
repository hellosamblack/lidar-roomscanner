"""Shared "depth-scope" instrument-drawing primitives (pure Pillow/numpy).

Lifted out of cards.py so both cards.py and hud.py draw in one visual
language from one source. No Open3D, no IO. Accent cyan == theme.ACCENT.
"""
from __future__ import annotations

from . import theme

# instrument chrome (RGB 0-255) -- the 2D counterpart to theme's 3D scene palette
PANEL = (16, 19, 26)
HAIR = (46, 52, 64)
TEXT = (232, 234, 242)
MUTED = (126, 132, 148)


def u8(rgb_float) -> tuple[int, int, int]:
    return tuple(int(round(c * 255)) for c in rgb_float)


ACCENT_U8 = u8(theme.ACCENT)


def load_font(size: int, *, mono: bool = False, bold: bool = False):
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


def tracked_text(draw, xy, text, font, fill, spacing=3) -> float:
    """Draw `text` with extra inter-letter spacing (an 'eyebrow' look Pillow
    can't do via a font feature). Returns the x cursor after the last glyph."""
    x, y = xy
    for ch in text:
        draw.text((x, y), ch, font=font, fill=fill)
        x += draw.textlength(ch, font=font) + spacing
    return x


def corner_ticks(d, x0, y0, x1, y1, color, ln=11, inset=6, width=2) -> None:
    x0 += inset; y0 += inset; x1 -= inset; y1 -= inset
    for cx, cy, dx, dy in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
        d.line([cx, cy, cx + dx * ln, cy], fill=color, width=width)
        d.line([cx, cy, cx, cy + dy * ln], fill=color, width=width)


def fmt_count(n: int) -> str:
    """Compact integer: 273789 -> '273k', 1_200_000 -> '1.2M', <1000 -> as-is.
    Pure -- unit-tested."""
    n = int(n)
    if abs(n) >= 1_000_000:
        return f"{n / 1_000_000:.1f}M"
    if abs(n) >= 1_000:
        return f"{n // 1000}k"
    return str(n)
