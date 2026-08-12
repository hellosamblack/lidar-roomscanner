"""Regression tests for #103: selecting a capture must surface ITS action pane.

Two independent failure modes were measured in the live UI and are pinned here.

1. **Layout** — `#browser-selected-detail` used to sit *after* `#cap-grid`. The
   grid is the flex-growing member that also scrolls itself, so the drawer was
   pushed past the card's `overflow: hidden` clip: measured at bottom 1018 in a
   card clipped at 946, which put `#btn-preview-load` and `#btn-preview-build`
   outside the card AND (for Build) outside the viewport. The only scrollable
   ancestor was the card body, and a wheel over the grid scrolls the *grid*, so
   no gesture brought them back. The fix is structural -- the drawer belongs to
   the card's fixed chrome, before the grid -- so a DOM-order check is what
   actually guards it.

2. **Selection lifecycle** — the drawer's buttons act on `previewed`, so the
   drawer must never point at a capture the user did not pick. Three ways it did:
   a rename left `previewed` naming a file that no longer existed and the client
   fell back to the *playing* capture; deleting an explicit pick did the same;
   and `state.selected_capture` (which is the server's LOADED capture, not the
   user's highlight) was echoed into `previewed` on every `state` broadcast,
   snapping the drawer back ~3 ms after any rename.

The lifecycle tests EXECUTE the shipped `hub.on('captures', ...)` handler under
Node rather than reimplementing it -- the same pattern as
`test_capture_browser_tooltip.py` and `test_protocol_c_crosscheck.py`. A Python
reimplementation could pass while the shipped code regressed.
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

STATIC = Path(__file__).parent.parent / "src" / "roomscan" / "static"
INDEX = STATIC / "index.html"
BROWSER_JS = STATIC / "browser.js"


# --------------------------------------------------------------------------
# 1. layout: the drawer is fixed chrome ahead of the scrolling grid
# --------------------------------------------------------------------------


def test_action_drawer_precedes_the_scrolling_capture_grid():
    """#103: after the grid, the drawer is clipped out of the card entirely."""
    html = INDEX.read_text(encoding="utf-8")
    drawer = html.index('id="browser-selected-detail"')
    grid = html.index('id="cap-grid"')
    assert drawer < grid, (
        "#browser-selected-detail must come BEFORE #cap-grid: the grid is the "
        "flex-growing, self-scrolling member, so anything after it is pushed "
        "past the card's `overflow: hidden` clip and Load/Build become "
        "unclickable (#103)"
    )


def test_action_drawer_cannot_be_squeezed_or_clipped():
    """`flex: 0 0 auto` is what keeps the drawer at its natural height."""
    style = re.search(r"<style>(.*?)</style>", INDEX.read_text(encoding="utf-8"), re.DOTALL)
    assert style is not None, "no <style> block in index.html"
    rule = re.search(
        r"#browser-selected-detail\s*\{([^}]*)\}", style.group(1), re.DOTALL
    )
    assert rule is not None, "no #browser-selected-detail rule found"
    body = rule.group(1)
    assert re.search(r"flex:\s*0\s+0\s+auto", body), (
        "#browser-selected-detail must be `flex: 0 0 auto` so the capture grid "
        "is the only member that flexes; a shrinkable drawer gets clipped (#103)"
    )


def test_grid_is_the_flexing_member():
    """The other half of the contract: the grid absorbs the leftover height."""
    style = re.search(r"<style>(.*?)</style>", INDEX.read_text(encoding="utf-8"), re.DOTALL)
    rule = re.search(r"\.browser-grid\s*\{([^}]*)\}", style.group(1), re.DOTALL)
    assert rule is not None, "no .browser-grid rule found"
    assert re.search(r"flex:\s*1\s+1\s+auto", rule.group(1)), (
        ".browser-grid must stay `flex: 1 1 auto` -- it is the member that "
        "yields height to the action drawer (#103)"
    )


# --------------------------------------------------------------------------
# 2. selection lifecycle: execute the shipped `captures` handler under Node
# --------------------------------------------------------------------------


def _captures_handler_source() -> str:
    """The body of `hub.on('captures', (msg) => { ... })`, verbatim."""
    text = BROWSER_JS.read_text(encoding="utf-8")
    marker = "hub.on('captures', (msg) => {"
    start = text.index(marker) + len(marker)
    depth = 1
    i = start
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start:i]
        i += 1
    raise AssertionError("could not find the end of the `captures` handler")


def _run_handler(*, items, previewed, preview_explicit, pending_rename_to, playing):
    """Run the real handler over one `captures` message and report the state."""
    node = shutil.which("node")
    if node is None:
        pytest.skip("no node on PATH")

    harness = f"""
    let captures = [];
    let selected = new Set();
    let previewed = {json.dumps(previewed)};
    let previewExplicit = {json.dumps(preview_explicit)};
    let pendingRenameTo = {json.dumps(pending_rename_to)};
    let playing = {json.dumps(playing)};
    const render = () => {{}};

    const handler = (msg) => {{{_captures_handler_source()}}};
    handler({{ items: {json.dumps(items)} }});

    console.log(JSON.stringify({{ previewed, pendingRenameTo }}));
    """
    proc = subprocess.run(
        [node, "-e", harness], capture_output=True, text=True, timeout=30
    )
    assert proc.returncode == 0, f"node harness failed:\n{proc.stderr}"
    return json.loads(proc.stdout)


def _lib(*names):
    return [{"name": n} for n in names]


def test_rename_carries_the_selection_to_the_new_name():
    """The drawer follows the same file across a rename."""
    out = _run_handler(
        items=_lib("new.bin", "other.bin"),
        previewed="old.bin",
        preview_explicit=True,
        pending_rename_to="new.bin",
        playing="other.bin",
    )
    assert out["previewed"] == "new.bin", (
        "after a rename the drawer must follow the renamed capture, not fall "
        "back to the playing one (#103)"
    )
    assert out["pendingRenameTo"] is None


def test_a_refused_rename_keeps_the_original_selection():
    """The server can refuse (collision/invalid); the old name is still there."""
    out = _run_handler(
        items=_lib("old.bin", "other.bin"),
        previewed="old.bin",
        preview_explicit=True,
        pending_rename_to="taken.bin",
        playing="other.bin",
    )
    assert out["previewed"] == "old.bin"
    assert out["pendingRenameTo"] is None, "a refused rename must not stay pending"


def test_deleting_an_explicit_pick_clears_the_drawer():
    """It must NOT retarget at the playing capture -- Build would hit the wrong file."""
    out = _run_handler(
        items=_lib("other.bin"),
        previewed="gone.bin",
        preview_explicit=True,
        pending_rename_to=None,
        playing="other.bin",
    )
    assert out["previewed"] is None, (
        "deleting the selected capture must clear the drawer rather than point "
        "Load/Build at whatever happens to be playing (#103)"
    )


def test_the_playing_capture_is_still_adopted_as_an_initial_default():
    """Pre-existing convenience, preserved: no pick yet -> show what is playing."""
    out = _run_handler(
        items=_lib("playing.bin", "other.bin"),
        previewed=None,
        preview_explicit=False,
        pending_rename_to=None,
        playing="playing.bin",
    )
    assert out["previewed"] == "playing.bin"


def test_an_explicit_pick_survives_an_unrelated_library_echo():
    """A `captures` refresh must not move the user's selection."""
    out = _run_handler(
        items=_lib("mine.bin", "playing.bin"),
        previewed="mine.bin",
        preview_explicit=True,
        pending_rename_to=None,
        playing="playing.bin",
    )
    assert out["previewed"] == "mine.bin"


# --------------------------------------------------------------------------
# 3. the state echo must not clobber the client-local selection
# --------------------------------------------------------------------------


def test_selected_capture_only_seeds_the_selection():
    """`state` re-broadcasts on every settings change (see BUG-062's lesson).

    `selected_capture` is the server's LOADED capture, so echoing it into
    `previewed` unconditionally snapped the drawer back to it ~3 ms after any
    rename. It may only seed a selection the user has not made yet.
    """
    js = BROWSER_JS.read_text(encoding="utf-8")
    assignments = re.findall(r"^.*previewed\s*=\s*msg\.selected_capture.*$", js, re.M)
    assert assignments, "no `previewed = msg.selected_capture` assignment found"
    for line in assignments:
        assert "previewExplicit" in line, (
            "assigning msg.selected_capture into `previewed` must be guarded by "
            f"!previewExplicit, or the state echo clobbers the user's pick: {line.strip()!r}"
        )


def test_clicking_a_tile_marks_the_selection_explicit():
    """Without this flag every guard above degrades to the old behaviour."""
    js = BROWSER_JS.read_text(encoding="utf-8")
    click_handler = js[js.index("grid?.addEventListener('click'"):]
    click_handler = click_handler[: click_handler.index("\n    segSort")]
    assert "previewExplicit = true" in click_handler, (
        "the tile click handler must set previewExplicit so a later delete or "
        "state echo cannot silently retarget the drawer (#103)"
    )


def test_tiles_expose_selection_to_assistive_tech():
    """Selection was communicated by border colour alone."""
    js = BROWSER_JS.read_text(encoding="utf-8")
    assert "aria-selected" in js, (
        "cap tiles must carry aria-selected alongside the is-selected class (#103)"
    )
