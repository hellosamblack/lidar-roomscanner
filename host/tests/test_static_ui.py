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
LAYOUT_JS = STATIC / "layout.js"


def _index() -> str:
    return INDEX.read_text(encoding="utf-8")


def _js_object_keys(js: str, var_name: str) -> set:
    """Quoted top-level keys of `var <var_name> = { 'k': ..., ... };` in layout.js.

    Cheap enough for a classic-script object literal (no nested braces before a
    key): scoped to the text between the var's opening `{` and its closing `};`
    so a same-named key elsewhere in the file can't leak in.
    """
    m = re.search(re.escape(var_name) + r"\s*=\s*\{(.*?)\n\s*\};", js, re.DOTALL)
    assert m is not None, f"could not find `var {var_name} = {{...}};` in layout.js"
    return set(re.findall(r"'([a-zA-Z0-9_-]+)'\s*:", m.group(1)))


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


def test_every_card_id_has_a_squircle_icon_and_title():
    """Every `data-card-id` in index.html must appear in BOTH `CARD_ICONS` and
    `CARD_TITLES` in layout.js.

    This is the real invariant behind the squircle rail: `updateSquircles()`
    does `if (!cardId || !CARD_ICONS[cardId]) continue;`, so a card with no
    `CARD_ICONS` entry gets no squircle button at all -- and once that card is
    collapsed there is no way back to it in the UI (short of clearing
    localStorage). A missing `CARD_TITLES` entry degrades more quietly (the
    button falls back to the raw id as its `title`), but it's the same class of
    drift, so both are checked together.
    """
    card_ids = sorted(set(re.findall(r'data-card-id="([^"]+)"', _index())))
    assert card_ids, "no data-card-id attributes found in index.html -- test is broken"

    js = LAYOUT_JS.read_text(encoding="utf-8")
    icon_keys = _js_object_keys(js, "CARD_ICONS")
    title_keys = _js_object_keys(js, "CARD_TITLES")

    missing_icons = [c for c in card_ids if c not in icon_keys]
    missing_titles = [c for c in card_ids if c not in title_keys]
    assert not missing_icons, (
        f"data-card-id values with no CARD_ICONS entry in layout.js (their "
        f"squircle button silently never gets created): {missing_icons}"
    )
    assert not missing_titles, (
        f"data-card-id values with no CARD_TITLES entry in layout.js (their "
        f"squircle button's title falls back to the raw id): {missing_titles}"
    )


def test_view_modes_include_preview_and_build_uses_a_confirmation_modal():
    """The thumbnail is a real display mode, and Detailed build cannot start
    invisibly from the capture card."""
    html = _index()
    assert 'data-display="preview"' in html
    for element_id in ("build-modal", "build-intro", "build-facts", "build-confirm"):
        assert f'id="{element_id}"' in html


def test_detailed_build_has_an_always_visible_progress_status():
    """An offline build must never look like a dead click.

    The SLAM HUD is intentionally collapsed by default, so its frame counter is
    not an adequate progress affordance.  The viewport overlay is the durable
    status surface: it stays visible while the progressive mesh refines below.
    """
    html = _index()
    for element_id in (
        "detailed-build-status", "detailed-build-title", "detailed-build-progress",
        "detailed-build-bar", "detailed-build-time", "detailed-build-detail",
        "detailed-resource-gpu", "detailed-resource-cpu", "detailed-resource-ram",
        "detailed-resource-vram",
    ):
        assert f'id="{element_id}"' in html
    assert 'role="progressbar"' in html
    assert 'detailed-build__resources' in html


# ---------------------------------------------------------------------------
# devicemodel.js -- the shared 3D device (owner ask, 2026-07-31)
# ---------------------------------------------------------------------------
# The block is drawn by two renderers that share nothing else: magcal3d.js
# (WebGL, mag-cal modal) and sensors.js (2D canvas, orientation gizmo). These
# checks pin the two constants that carry real physical meaning, because both
# are the kind of thing that is invisibly wrong: a mount rotation that flips the
# model upside down still renders a plausible box, and a frame permutation that
# is silently transposed still renders a plausible box that mirrors.

DEVICE_JS = STATIC / "devicemodel.js"


def _js_number_array(js: str, name: str) -> list:
    """Numbers of `export const <name> = [ ... ];` in devicemodel.js."""
    m = re.search(r"export const " + re.escape(name) + r"\s*=\s*\[(.*?)\];", js, re.DOTALL)
    assert m is not None, f"could not find `export const {name} = [...]` in devicemodel.js"
    return [float(x) for x in re.findall(r"-?\d+(?:\.\d+)?", m.group(1))]


def _js_dims(js: str) -> dict:
    m = re.search(r"export const DEVICE_DIMS\s*=\s*\{(.*?)\};", js, re.DOTALL)
    assert m is not None, "could not find `export const DEVICE_DIMS = {...}`"
    return {k: float(v) for k, v in re.findall(r"([xyz])\s*:\s*(-?[\d.]+)", m.group(1))}


def test_device_dims_carry_the_owners_5_5_by_3_by_2_5_block():
    """The scanner is a 5.5" x 3" x 2.5" block, not the flat PCB silhouette the
    mag-cal view used to draw. The ratios are what matter (the absolute scale is
    magcal3d's shell units), and the DEPTH ratio especially: at 8x the old
    board's thickness the model sits across the middle of the coverage shell,
    which is the whole reason every caller ghosts its fill."""
    d = _js_dims(DEVICE_JS.read_text(encoding="utf-8"))
    assert d["y"] / d["x"] == pytest.approx(3.0 / 5.5, abs=1e-3)
    assert d["z"] / d["x"] == pytest.approx(2.5 / 5.5, abs=1e-3)


def test_device_dims_m_are_the_real_block_in_metres():
    """The metric twin of DEVICE_DIMS: 5.5" x 3" x 2.5" at 0.0254 m/in."""
    js = DEVICE_JS.read_text(encoding="utf-8")
    m = re.search(r"export const DEVICE_DIMS_M\s*=\s*\{(.*?)\};", js, re.DOTALL)
    assert m is not None, "could not find `export const DEVICE_DIMS_M = {...}`"
    vals = {k: eval(v) for k, v in                      # noqa: S307 - literal arithmetic
            re.findall(r"([xyz])\s*:\s*([-\d.*\s]+?)\s*[,}]", m.group(1) + "}")}
    assert vals["x"] == pytest.approx(0.1397, abs=1e-4)
    assert vals["y"] == pytest.approx(0.0762, abs=1e-4)
    assert vals["z"] == pytest.approx(0.0635, abs=1e-4)
    # Same block, two scales: the shell-unit set must stay proportional to it,
    # so a future tweak to one cannot silently reshape the other.
    d = _js_dims(js)
    ratios = [d[k] / vals[k] for k in "xyz"]
    assert max(ratios) - min(ratios) < 1e-2, f"DEVICE_DIMS/DEVICE_DIMS_M not uniform: {ratios}"


def test_the_map_marker_is_drawn_at_the_devices_real_metric_size():
    """slam.js's scene is in METRES, next to real walls, so the pose marker must
    pass DEVICE_DIMS_M. It used to pass no `dims` at all and take the shell-unit
    default, drawing a 62 cm slab of scanner in a room with 2 m doorways
    (reported 2026-08-01). Nothing catches this in mag-cal, where the block sits
    in a unitless shell and 4.44x means nothing."""
    slam = (STATIC / "slam.js").read_text(encoding="utf-8")
    assert "DEVICE_DIMS_M" in slam, "slam.js must ask for the metric block"
    call = re.search(r"createDeviceMesh\(THREE,\s*\{(.*?)\}\)", slam, re.DOTALL)
    assert call is not None
    assert "dims: DEVICE_DIMS_M" in " ".join(call.group(1).split())


def test_aperture_scales_with_the_block_it_is_drawn_on():
    """The aperture is a FRACTION of the long axis, not an absolute. As an
    absolute 0.048 it is 7.7% of the shell-unit block but 34% of the metric one,
    so the real-size scanner would have rendered as mostly lens."""
    js = DEVICE_JS.read_text(encoding="utf-8")
    assert re.search(r"export const APERTURE_R_FRAC\s*=", js)
    body = js[js.index("export function createDeviceMesh"):]
    body = body[:body.index("\nexport function", 1)] if "\nexport function" in body[1:] else body
    assert "APERTURE_R_FRAC * dims.x" in body
    assert "CircleGeometry(APERTURE_R," not in body, "aperture still hard-coded to shell units"


def test_device_bands_are_half_grey_quarter_white_quarter_blue():
    """Through the depth, from the face toward the user to the camera face."""
    js = DEVICE_JS.read_text(encoding="utf-8")
    m = re.search(r"export const DEVICE_BANDS\s*=\s*\[(.*?)\];", js, re.DOTALL)
    assert m is not None
    fracs = [float(x) for x in re.findall(r"frac:\s*([\d.]+)", m.group(1))]
    assert fracs == [0.50, 0.25, 0.25]
    assert sum(fracs) == pytest.approx(1.0)
    # Ordered -Z (user) -> +Z (boresight); the camera is on the last one.
    names = re.findall(r"name:\s*'([a-z]+)'", m.group(1))
    assert names[-1] == "camera"


def test_mount_rotation_stands_the_device_upright_at_the_owners_attitude():
    """MOUNT_ROTATION maps the DESIGN frame (+X = the device's own top) onto the
    SFLP BODY frame, and it is a 180 deg turn about the boresight.

    Held normally the instrument reads **World pitch 0 deg, roll 180 deg**, and
    the owner confirms those readings are correct. World roll is
    `sensors.triad_roll_deg`: the roll of body **+X** about the boresight against
    true vertical -- so 180 deg means body +X points DOWN while the device is the
    right way up. Body +X is therefore the device's bottom, and the old model
    (which called +X "Up, USB down") drew it upside down.

    This asserts the consequence rather than the matrix: at exactly that
    attitude, the device's top must point at the ceiling and its camera face
    must be horizontal. An identity MOUNT_ROTATION passes every "is it a box"
    check and fails this one, which is the point -- a 180 deg error is invisible
    to symmetry checks (the block's silhouette is symmetric about its centre).
    """
    import numpy as np

    from roomscan import sensors

    mount = np.array(_js_number_array(DEVICE_JS.read_text(encoding="utf-8"),
                                      "MOUNT_ROTATION")).reshape(3, 3)

    # Body -> SFLP world (X=North, Y=West, Z=Up) for "held normally":
    # boresight (body +Z) aimed North and level; body +X pointing at the floor.
    r = np.array([[0.0, 0.0, 1.0],
                  [0.0, 1.0, 0.0],
                  [-1.0, 0.0, 0.0]])
    assert np.linalg.det(r) == pytest.approx(1.0)

    # It really is the owner's reported attitude, by the server's own functions.
    down_body = r.T @ np.array([0.0, 0.0, -1.0])
    assert sensors.tilt_from_down_deg(down_body) == pytest.approx(0.0, abs=1e-9)
    assert abs(sensors.triad_roll_deg(down_body)) == pytest.approx(180.0, abs=1e-9)

    design_to_world = r @ mount
    top = design_to_world @ np.array([1.0, 0.0, 0.0])       # the device's top
    camera = design_to_world @ np.array([0.0, 0.0, 1.0])    # the blue/camera face
    assert top == pytest.approx([0.0, 0.0, 1.0], abs=1e-9), (
        "the device's top must point at the ceiling when it is held the right "
        f"way up; MOUNT_ROTATION put it at {top}")
    assert camera[2] == pytest.approx(0.0, abs=1e-9), (
        "pitch 0 means the camera face is horizontal; got a Z component of "
        f"{camera[2]}")
    # ...and the rotation must be about the boresight, or the camera face moves.
    assert mount[2, 2] == pytest.approx(1.0)


def test_body_to_cv_is_the_transpose_of_the_servers_t_cv_to_body():
    """`drawDeviceBox2D` is handed the `sensor` message's `rot`
    (`T_WORLD_TO_CV @ R @ T_CV_TO_BODY`), which rotates CV-frame vectors, while
    the block's geometry is body-framed. `BODY_TO_CV` is the one permutation the
    client owns to bridge those. Pinning it here means a server-side convention
    change fails a test rather than silently mirroring the gizmo -- and a
    mirrored box is exactly the defect nobody spots, because a box looks like a
    box either way."""
    import numpy as np

    from roomscan import sensors

    body_to_cv = np.array(_js_number_array(DEVICE_JS.read_text(encoding="utf-8"),
                                           "BODY_TO_CV")).reshape(3, 3)
    assert body_to_cv == pytest.approx(sensors.T_CV_TO_BODY.T)


def test_the_orientation_gizmo_draws_the_device_not_an_axis_triad():
    """sensors.js must source its widget from the shared module, so the gizmo
    and the mag-cal modal cannot drift into two different devices."""
    js = (STATIC / "sensors.js").read_text(encoding="utf-8")
    assert "from './devicemodel.js'" in js
    assert "drawDeviceBox2D" in js
    assert "AXIS_COLORS" not in js, "the RGB axis triad is still being drawn"


def test_camera_views_cover_every_display_and_slam_uses_the_shared_scanner_model():
    """World / FPV / Mirror must not turn off when SLAM is selected.

    The live cloud has a server-side frame transform, whereas maps are
    world-fixed: their first-person/mirror behaviour belongs in the shared scene
    camera.  The pose marker is the same physical scanner model as Sensors, not
    another anonymous green sphere.
    """
    controls = (STATIC / "controls.js").read_text(encoding="utf-8")
    scene = (STATIC / "scene.js").read_text(encoding="utf-8")
    slam = (STATIC / "slam.js").read_text(encoding="utf-8")
    browser = (STATIC / "browser.js").read_text(encoding="utf-8")
    assert "b.disabled = (msg.mode === 'slam')" not in controls
    assert "setSlamPose" in scene and "setViewportMirror" in scene
    assert "from './devicemodel.js'" in slam and "createDeviceMesh" in slam
    assert "SphereGeometry(0.03" not in slam
    assert "viewMode === 'mirror' ? 'scaleX(-1)'" in browser


def test_every_fusion_status_has_a_label_and_a_remedy():
    """Every `YawFusion.status` string the server can emit must have a HUD label
    (`web._FUSION_LABELS`) and a remedy line (`sensors.js` `FUSION_HELP`).

    A status with neither renders as the bare key -- "gated:no-field" in the
    Sensors card -- with nothing telling the operator what to do about it.
    Enforced rather than remembered: `gated:no-field` (BUG-058) is the fourth
    gate added to this filter, and the convention was documented nowhere.
    """
    import ast

    from roomscan import web

    src = (Path(__file__).parent.parent / "src" / "roomscan" / "sensors.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    cls = next(n for n in ast.walk(tree)
               if isinstance(n, ast.ClassDef) and n.name == "YawFusion")
    statuses = {n.value for n in ast.walk(cls)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)
                and (n.value in ("init", "active") or n.value.startswith("gated:"))}
    assert "gated:no-field" in statuses, "AST scan found no statuses -- the check is broken"

    help_keys = set(re.findall(r"^\s*'?([a-z:-]+)'?\s*:", 
                               re.search(r"const FUSION_HELP = \{(.*?)\n\};",
                                         (STATIC / "sensors.js").read_text(encoding="utf-8"),
                                         re.DOTALL).group(1), re.MULTILINE))
    missing_label = statuses - set(web._FUSION_LABELS)
    # "active" is the working state and deliberately has no remedy line.
    missing_help = statuses - help_keys - {"active"}
    assert not missing_label, f"YawFusion statuses with no HUD label: {sorted(missing_label)}"
    assert not missing_help, f"YawFusion statuses with no remedy line: {sorted(missing_help)}"
