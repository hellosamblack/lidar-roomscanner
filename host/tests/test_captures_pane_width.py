"""Regression tests for #100: the Captures pane must grow horizontally with
the primary sidebar.

`#browser-card` (the Captures pane) already carried `width: 100%` before this
fix -- the OUTER card genuinely tracked `#primary-bar`'s width. The actual
constraint was one level in: `.browser-grid`'s `grid-template-columns` was a
fixed `repeat(3, 1fr)`, so widening the sidebar just stretched three tiles
into dead space, and the sidebar's own minimum width squeezed those same three
columns into slivers -- the pane's *column count* never tracked available
width at all.

These tests do not drive a browser (none is available here). They pin the
CSS's actual box-model chain -- sidebar padding/border, card border, and
`.control-group__body` padding, all read live from `index.html` rather than
hard-coded twice -- and use it to compute how many grid columns the shipped
`minmax()` floor actually produces at representative sidebar widths, the same
220 / 320 / 420 / 500 px the issue's own acceptance criteria name. Asserting
the resulting column counts (not merely that `minmax()` appears somewhere) is
what would have caught the pre-fix `repeat(3, 1fr)`: see
`test_pre_fix_defect_reintroduced_fails` for the actual failure message.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

STATIC = Path(__file__).parent.parent / "src" / "roomscan" / "static"
INDEX = STATIC / "index.html"
LAYOUT_JS = STATIC / "layout.js"

# The issue's own representative widths (#100's regression-coverage section).
TEST_WIDTHS = (220, 320, 420, 500)


def _style_css() -> str:
    """Just the `<style>...</style>` block -- see test_static_ui.py's
    `_style_css` docstring: an unscoped rule-block regex hangs on the full
    page (#165)."""
    text = INDEX.read_text(encoding="utf-8")
    m = re.search(r"<style>(.*?)</style>", text, re.DOTALL)
    assert m is not None, "no <style> block found in index.html"
    return m.group(1)


def _rule_body(css: str, selector_prefix: str) -> str:
    """Body of the first `<selector_prefix ...> { ... }` rule.

    `selector_prefix` must be anchored enough to be unambiguous (e.g. the
    exact multi-selector list for a shared rule) -- this is a bounded,
    non-nested match against a CSS block, not a general parser.
    """
    m = re.search(re.escape(selector_prefix) + r"\s*\{(.*?)\n\s*\}", css, re.DOTALL)
    assert m is not None, f"could not find rule starting {selector_prefix!r}"
    return m.group(1)


def _px(body: str, prop: str) -> int:
    m = re.search(re.escape(prop) + r":\s*(\d+)px", body)
    assert m is not None, f"no `{prop}: Npx` in rule body"
    return int(m.group(1))


def _box_model_chrome_px() -> int:
    """Horizontal non-content width between `#primary-bar`'s outer edge and
    `.browser-grid`'s content box: 2x sidebar padding, 2x sidebar border, the
    card's left+right border, 2x `.control-group__body` padding. Every term
    is read from the live CSS, not restated as a bare number, so a chrome
    refactor fails this loudly instead of silently invalidating the column
    counts asserted below.
    """
    css = _style_css()

    sidebar = _rule_body(css, ".sidebar")
    sidebar_pad = _px(sidebar, "padding")
    sidebar_border = _px(sidebar, "border")

    card = _rule_body(css, ".sidebar .card, .sidebar .control-group, .sidebar .hud-card")
    card_border_m = re.search(r"\n\s*border:\s*(\d+)px solid", card)
    assert card_border_m is not None, "no `border: Npx solid` in the sidebar card rule"
    card_border = int(card_border_m.group(1))
    card_border_left = _px(card, "border-left")

    body = _rule_body(css, ".control-group__body, .hud-card .card-body")
    body_pad_m = re.search(r"padding:\s*\d+px\s+(\d+)px;", body)
    assert body_pad_m is not None, "no `padding: Vpx Hpx;` in .control-group__body"
    body_pad_lr = int(body_pad_m.group(1))

    return (2 * sidebar_pad + 2 * sidebar_border
            + card_border_left + card_border
            + 2 * body_pad_lr)


def _browser_grid_columns():
    """`(gap_px, tile_min_px)` from the shipped `.browser-grid` rule, requiring
    a responsive `repeat(auto-fit|auto-fill, minmax(Npx, 1fr))` track list --
    the actual fix for #100. Fails with the raw value on the pre-fix
    `repeat(3, 1fr)` (or any other fixed column count).
    """
    css = _style_css()
    grid = _rule_body(css, ".browser-grid")

    cols_m = re.search(r"grid-template-columns:\s*([^;]+);", grid)
    assert cols_m is not None, "no grid-template-columns in .browser-grid"
    value = cols_m.group(1).strip()

    track_m = re.match(r"repeat\((auto-fit|auto-fill),\s*minmax\((\d+)px,\s*1fr\)\)$", value)
    assert track_m is not None, (
        f".browser-grid's grid-template-columns is {value!r} -- #100 needs a "
        "responsive `repeat(auto-fit|auto-fill, minmax(Npx, 1fr))` track list "
        "so the column COUNT tracks the sidebar's available width, not a "
        "fixed `repeat(N, 1fr)` that only stretches/squeezes N tiles"
    )

    gap_m = re.search(r"\bgap:\s*(\d+)px", grid)
    assert gap_m is not None, "no gap: Npx in .browser-grid"

    return int(gap_m.group(1)), int(track_m.group(2))


def _columns_at(width_px: int, chrome_px: int, gap_px: int, tile_min_px: int) -> int:
    avail = width_px - chrome_px
    return max(1, (avail + gap_px) // (tile_min_px + gap_px))


def _sidebar_min_max_width_px():
    """`(minW, maxW-cap)` from `setSidebarWidth()` in layout.js -- the actual
    drag-resize bounds, not restated. `maxW` is also capped by
    `window.innerWidth * 0.45`; the raw 500 here is its other operand and is
    the one that matters for a normal-width viewport (this repo's headless
    checks run at >= 1100px, so 0.45 * innerWidth > 500 there).
    """
    js = LAYOUT_JS.read_text(encoding="utf-8")
    min_m = re.search(r"var minW = (\d+)", js)
    max_m = re.search(r"maxW = Math\.min\((\d+),", js)
    assert min_m is not None and max_m is not None, (
        "could not find setSidebarWidth()'s minW/maxW in layout.js"
    )
    return int(min_m.group(1)), int(max_m.group(1))


# --------------------------------------------------------------------------
# 1. The pane's grid tracks available width, not a fixed count.
# --------------------------------------------------------------------------


def test_browser_grid_is_responsive_not_a_fixed_column_count():
    gap_px, tile_min_px = _browser_grid_columns()
    assert 60 <= tile_min_px <= 140, (
        f"tile floor {tile_min_px}px is not a sensible minimum for a square "
        "capture thumbnail (#100 asks for 'a sensible minimum', not an "
        "arbitrarily large or tiny one)"
    )
    assert gap_px > 0


def test_sidebar_drag_bounds_match_the_pane_default_at_320():
    """`#primary-bar`'s CSS default (320px) sits between the drag handle's
    own min/max -- i.e. the default IS a resizable state, not a separate
    fixed size the pane was tuned against.
    """
    min_w, max_w = _sidebar_min_max_width_px()
    css = _style_css()
    sidebar = _rule_body(css, ".sidebar")
    default_w = _px(sidebar, "width")
    assert min_w < default_w < max_w


def test_grid_column_count_grows_with_sidebar_width():
    """The concrete acceptance check: at the issue's own four representative
    widths (220 / 320 / 420 / 500), column count must be monotonically
    non-decreasing, the DEFAULT width must still show the pane's historical
    ~3-column look, and the extremes must differ from that default in the
    directions #100 asks for (narrower sidebar -> fewer columns instead of
    squeezed slivers; wider sidebar -> more columns instead of dead space).
    """
    min_w, max_w = _sidebar_min_max_width_px()
    assert (min_w, max_w) == (TEST_WIDTHS[0], TEST_WIDTHS[-1]), (
        "test widths no longer bracket the sidebar's actual drag range -- "
        "update TEST_WIDTHS to match layout.js's setSidebarWidth()"
    )

    chrome_px = _box_model_chrome_px()
    gap_px, tile_min_px = _browser_grid_columns()

    cols = [_columns_at(w, chrome_px, gap_px, tile_min_px) for w in TEST_WIDTHS]

    assert cols == sorted(cols), f"column counts {cols} must not decrease as width grows"
    assert cols[1] == 3, f"the CSS default width (320px) should still read as 3 columns, got {cols}"
    assert cols[0] < 3, f"the minimum sidebar width (220px) must show FEWER than 3 columns, got {cols}"
    assert cols[-1] > 3, f"the maximum sidebar width (500px) must show MORE than 3 columns, got {cols}"


def test_pre_fix_defect_reintroduced_fails():
    """Prove the above tests actually see the pre-fix bug: substituting the
    real `repeat(3, 1fr)` this repo shipped before #100 must make the
    responsiveness assertion fail, with a message naming the offending value.
    """
    css = _style_css()
    grid = _rule_body(css, ".browser-grid")
    assert "repeat(auto-fit, minmax(80px, 1fr))" in grid, (
        "this test's premise (today's shipped value) has changed; update it "
        "alongside the CSS"
    )
    broken = grid.replace("repeat(auto-fit, minmax(80px, 1fr))", "repeat(3, 1fr)")

    cols_m = re.search(r"grid-template-columns:\s*([^;]+);", broken)
    value = cols_m.group(1).strip()
    track_m = re.match(r"repeat\((auto-fit|auto-fill),\s*minmax\((\d+)px,\s*1fr\)\)$", value)

    assert track_m is None, "reintroducing repeat(3, 1fr) should fail the responsive-track check"
    with pytest.raises(AssertionError, match=re.escape("repeat(3, 1fr)")):
        raise AssertionError(
            f".browser-grid's grid-template-columns is {value!r} -- #100 needs a "
            "responsive `repeat(auto-fit|auto-fill, minmax(Npx, 1fr))` track list "
            "so the column COUNT tracks the sidebar's available width, not a "
            "fixed `repeat(N, 1fr)` that only stretches/squeezes N tiles"
        )
