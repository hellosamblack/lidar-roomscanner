"""Pure-function tests for the shared instrument-drawing primitives."""
import numpy as np
from PIL import Image, ImageDraw

from roomscan import instrument, theme


def test_u8_rounds_accent():
    assert instrument.u8(theme.ACCENT) == (46, 224, 209)


def test_fmt_count_thresholds():
    assert instrument.fmt_count(42) == "42"
    assert instrument.fmt_count(273789) == "273k"
    assert instrument.fmt_count(1_200_000) == "1.2M"
    assert instrument.fmt_count(0) == "0"


def test_accent_u8_constant():
    assert instrument.ACCENT_U8 == (46, 224, 209)


def test_corner_ticks_draws_accent_pixels():
    img = Image.new("RGB", (60, 40), instrument.PANEL)
    d = ImageDraw.Draw(img)
    instrument.corner_ticks(d, 0, 0, 59, 39, instrument.ACCENT_U8)
    arr = np.asarray(img)
    matches = np.all(arr.reshape(-1, 3) == np.array(instrument.ACCENT_U8), axis=1)
    assert matches.sum() > 8          # 4 corners * 2 strokes, several px each


def test_tracked_text_advances_cursor():
    img = Image.new("RGB", (200, 40), instrument.PANEL)
    d = ImageDraw.Draw(img)
    end = instrument.tracked_text(d, (5, 5), "ABC", instrument.load_font(13), instrument.TEXT)
    assert end > 5                     # cursor moved right of the start x


def test_load_font_returns_font():
    f = instrument.load_font(12, mono=True, bold=True)
    assert f is not None
