"""Pure-function tests for the Showcase reveal card (cards.py)."""
import numpy as np

from roomscan import cards, theme


def test_fmt_count():
    assert cards.fmt_count(42) == "42"
    assert cards.fmt_count(999) == "999"
    assert cards.fmt_count(273789) == "273k"
    assert cards.fmt_count(1_200_000) == "1.2M"
    assert cards.fmt_count(0) == "0"


def test_render_scan_complete_card_shape_and_dtype():
    img = cards.render_scan_complete_card(312, 0.14, 273789, 11.4, width=540)
    assert img.shape == (96, 540, 3)
    assert img.dtype == np.uint8


def test_render_scan_complete_card_uses_accent_cyan():
    # The corner ticks + eyebrow are drawn in theme.ACCENT -- assert those exact
    # pixels appear, so the card shares the scene's one signal color.
    img = cards.render_scan_complete_card(312, 0.14, 273789, 11.4)
    accent = np.array([int(round(c * 255)) for c in theme.ACCENT])
    matches = np.all(img.reshape(-1, 3) == accent, axis=1)
    assert matches.sum() > 20          # ticks + eyebrow glyph pixels


def test_render_scan_complete_card_is_deterministic():
    a = cards.render_scan_complete_card(10, 0.5, 1000, 2.0)
    b = cards.render_scan_complete_card(10, 0.5, 1000, 2.0)
    np.testing.assert_array_equal(a, b)


def test_render_scan_complete_card_width_respected():
    img = cards.render_scan_complete_card(1, 0.0, 5, 0.1, width=400)
    assert img.shape[1] == 400
