"""Markup hygiene for the web UI's single static page.

These are cheap structural checks over `static/index.html` -- the kind of thing
that is obvious in review and invisible at runtime until someone hits it. They
live in their own module rather than in `test_web.py` because they assert on
markup, not on the `/ws` protocol, and because `test_web.py` is already 3000
lines of behavioural tests.

What is deliberately NOT here: anything that would break on a CSS reformat or a
whitespace change. Each check parses for a specific attribute on a specific tag.
"""

import re
from pathlib import Path

import pytest

STATIC = Path(__file__).parent.parent / "src" / "roomscan" / "static"
INDEX = STATIC / "index.html"


def _index() -> str:
    return INDEX.read_text(encoding="utf-8")


# Collapse headers are exempt from the tooltip rule: the header's entire visible
# content IS the panel name, so a `title` could only repeat it. Every other
# button is a verb whose effect is not fully carried by its label.
_HEADER_CLASSES = ("control-group__header", "diag__header", "log-console__header",
                   "ir-card__header")


def _is_exempt(tag: str) -> bool:
    return any(cls in tag for cls in _HEADER_CLASSES)


def test_every_button_has_a_tooltip():
    """docs/engineering-practices.md: every interactive control carries a `title`.

    The rule predates this test by months and was enforced only by review, which
    is why nine controls had drifted past it (the magcal Start/Stop pair, the
    colour-by segmented control, both modal button rows). A control whose effect
    is not obvious from a one-word label is exactly where a tooltip earns its
    keep, and those are the ones review forgets.
    """
    missing = [
        tag for tag in re.findall(r"<button\b[^>]*>", _index())
        if "title=" not in tag and not _is_exempt(tag)
    ]
    assert not missing, (
        "buttons without a title= tooltip (docs/engineering-practices.md "
        f"'Web UI'):\n  " + "\n  ".join(missing)
    )


def test_every_toggle_label_has_a_tooltip():
    """The `title` goes on the <label class="toggle">, not the inner <input>.

    A native tooltip only fires over the element the pointer is actually on, and
    the label's text is the wide part of a toggle -- putting it on the checkbox
    means it appears over a 14 px square and nowhere else.
    """
    missing = [
        tag for tag in re.findall(r"<label\b[^>]*>", _index())
        if "toggle" in tag and "title=" not in tag
    ]
    assert not missing, (
        "toggle labels without a title= tooltip:\n  " + "\n  ".join(missing)
    )


def test_no_duplicate_element_ids():
    """A duplicate id silently rewires whichever module looks it up second.

    BUG-047: `btn-restart` named both the top bar's "Restart Server" and the
    playback "Restart". `getElementById` returns the first in document order, so
    the playback button was dead AND its transport handler was attached to
    Restart Server -- pressing that fired a transport restart *and* restarted the
    server process. Neither symptom points at a duplicate id, which is why this
    is a whole-file structural check rather than a test of either feature.
    """
    # Whitespace-anchored, not \b: `\bid="` also matches inside `data-card-id="`,
    # because \b fires between the hyphen and the `i`.
    ids = re.findall(r'\sid="([^"]+)"', _index())
    dupes = sorted({i for i in ids if ids.count(i) > 1})
    assert not dupes, f"duplicate id= attributes in index.html: {dupes}"


@pytest.mark.parametrize("selector", [
    "#diag-log",
    ".log-console__body",
    ".sensor-block-val",
    ".jitter-table",
])
def test_readonly_text_sinks_opt_back_into_selection(selector):
    """`body { user-select: none }` made every diagnostic uncopyable.

    That rule exists so dragging on the 3D canvas doesn't paint a text selection
    across the overlay, and it should stay -- but it also meant you could read a
    quaternion or a stack trace on screen and have no way to get it into a bug
    report. These four are read-only text with nothing draggable in them.
    """
    css = _index()
    assert "user-select: none" in css, "the body rule this test is about is gone"
    rule = re.search(
        r"([^{}]*\buser-select:\s*text\b[^{}]*)|"
        r"([^{}]*)\{[^{}]*user-select:\s*text", css)
    assert rule is not None, "no user-select: text rule at all"
    # The selector must appear in a rule block that grants text selection.
    granting = [
        m.group(1) for m in re.finditer(r"([^{}]+)\{([^{}]*)\}", css)
        if "user-select: text" in m.group(2)
    ]
    assert any(selector in sel for sel in granting), (
        f"{selector} is not covered by a `user-select: text` rule; "
        f"granting selectors were: {granting}"
    )
