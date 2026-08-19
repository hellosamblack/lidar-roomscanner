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
from html.parser import HTMLParser
from pathlib import Path

import pytest

STATIC = Path(__file__).parent.parent / "src" / "roomscan" / "static"
INDEX = STATIC / "index.html"
LAYOUT_JS = STATIC / "layout.js"


def _index() -> str:
    return INDEX.read_text(encoding="utf-8")


def _style_css() -> str:
    """Just the `<style>...</style>` block, not the whole page.

    #165: a rule-block regex (`[^{}]+\\{[^{}]*\\}`) run against the FULL
    `index.html` -- inline `<script>` JS and all -- hits pathological
    backtracking against that shape of input (deeply nested object/template
    braces) and never returns; the same regex against just the CSS finishes
    in low single-digit milliseconds. Scoping to the `<style>` block is also
    the actually-correct behaviour: a test about CSS selectors should not be
    matching JS object literals that happen to contain the same braces.
    """
    m = re.search(r"<style>(.*?)</style>", _index(), re.DOTALL)
    assert m is not None, "no <style> block found in index.html"
    return m.group(1)


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
_HEADER_CLASSES = ("control-group__header", "log-console__header",
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
        "'Web UI'):\n  " + "\n  ".join(missing)
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
    css = _style_css()
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


# ---------------------------------------------------------------------------
# Sidepane hierarchy weight (BUG-093 / #104): a card header, a subgroup
# summary and a leaf field label must read as three visually DISTINCT levels,
# not two. The pre-fix CSS had the subgroup summary (the parent) reading
# *lighter* than the field label it directly contains (the child) --
# 0.68rem/600/opacity .9 vs 0.66rem/600/opacity 1 implicit -- which is the
# hierarchy inverted, not merely flattened.
# ---------------------------------------------------------------------------

_HEADER_SEL = ".control-group__header"
_SUBGROUP_SEL = ".control-subgroup > summary, .debug-section > summary"
_FIELD_SEL = ".field-label"
_SUBFIELD_SEL = ".field-label--sub"


def _rule_decl(css: str, selector: str, prop: str) -> str:
    """The declared value of `prop` inside the rule whose selector text is
    exactly `selector` (as authored, including any comma-joined group).

    Anchored to the start of a line (past leading whitespace): an unanchored
    search for `.control-group__header` also matches inside the unrelated,
    earlier `.hud-card .control-group__header { pointer-events: ... }` rule
    (it is a literal substring of that descendant selector) and reads that
    rule's declarations instead of the real one.
    """
    m = re.search(r"(?m)^[ \t]*" + re.escape(selector) + r"\s*\{([^}]*)\}", css)
    assert m is not None, f"no CSS rule found for selector {selector!r}"
    pm = re.search(r"(?<![-\w])" + re.escape(prop) + r":\s*([^;]+);", m.group(1))
    assert pm is not None, f"`{prop}` not declared in rule {selector!r}"
    return pm.group(1).strip()


def test_sidepane_three_levels_are_defined_by_distinct_css_rules():
    """`.control-group__header`, the subgroup `> summary` and `.field-label`
    must each be their own rule in the stylesheet -- the acceptance criterion
    from the issue body, checked structurally rather than just by effect."""
    css = _style_css()
    for selector in (_HEADER_SEL, _SUBGROUP_SEL, _FIELD_SEL, _SUBFIELD_SEL):
        assert re.search(re.escape(selector) + r"\s*\{", css), (
            f"expected a dedicated CSS rule for {selector!r}"
        )


def test_sidepane_hierarchy_font_size_strictly_decreases_by_level():
    """Card header > subgroup summary > field label > nested sub-field label,
    strictly, in font-size -- the most immediately scannable cue."""
    css = _style_css()
    sizes = {
        sel: float(_rule_decl(css, sel, "font-size").removesuffix("rem"))
        for sel in (_HEADER_SEL, _SUBGROUP_SEL, _FIELD_SEL, _SUBFIELD_SEL)
    }
    assert sizes[_HEADER_SEL] > sizes[_SUBGROUP_SEL] > sizes[_FIELD_SEL] > sizes[_SUBFIELD_SEL], (
        f"sidepane level font-sizes are not strictly decreasing: {sizes}"
    )


def test_sidepane_subgroup_summary_outweighs_the_field_label_it_contains():
    """Regression guard for the actual reported defect: a subgroup summary
    (a DIRECT child of the card, e.g. "Camera & Pose") must carry more
    font-weight than a `.field-label` nested inside it (a GRANDCHILD of the
    card, e.g. "View Mode") -- not the same or less.

    Reintroducing the pre-fix values (both declared `font-weight: 600`) makes
    this fail: `header_weight >= subgroup_weight > field_weight` requires a
    strict `>` on the second comparison, which two equal 600s cannot satisfy.
    """
    css = _style_css()
    header_weight = int(_rule_decl(css, _HEADER_SEL, "font-weight"))
    subgroup_weight = int(_rule_decl(css, _SUBGROUP_SEL, "font-weight"))
    field_weight = int(_rule_decl(css, _FIELD_SEL, "font-weight"))
    sub_field_weight = int(_rule_decl(css, _SUBFIELD_SEL, "font-weight"))
    assert header_weight >= subgroup_weight > field_weight > sub_field_weight, (
        f"font-weight must strictly separate levels below the header: "
        f"header={header_weight} subgroup={subgroup_weight} "
        f"field={field_weight} sub-field={sub_field_weight}"
    )

    # And opacity must not invert the hierarchy either: the subgroup summary
    # (parent) must be at least as opaque/prominent as the field label
    # (child) it contains, and a field-label--sub (grandchild-of-subgroup)
    # must be dimmer still than its own field-label parent.
    subgroup_opacity = float(_rule_decl(css, _SUBGROUP_SEL, "opacity"))
    field_opacity = float(_rule_decl(css, _FIELD_SEL, "opacity"))
    sub_field_opacity = float(_rule_decl(css, _SUBFIELD_SEL, "opacity"))
    assert subgroup_opacity > field_opacity > sub_field_opacity, (
        f"opacity must strictly separate levels: subgroup={subgroup_opacity} "
        f"field={field_opacity} sub-field={sub_field_opacity}"
    )


@pytest.mark.parametrize("label", ["Distance", "Height", "Rotation", "Orbit Speed"])
def test_camera_pose_nested_labels_carry_the_subordinate_sub_class(label):
    """Inside the View card's "Camera & Pose" subgroup, Distance/Height/
    Rotation/Orbit Speed nest one level deeper than their own group label
    ("Camera") -- they must carry `field-label--sub` in addition to
    `field-label`, or they read at the same weight as "Camera", "View Mode"
    and "See-Through" (the BUG-093 complaint, restated one level down)."""
    html = _index()
    view_camera = re.search(
        r'data-subgroup-id="view-camera".*?</details>', html, re.DOTALL)
    assert view_camera is not None, "could not locate the Camera & Pose subgroup"
    body = view_camera.group(0)
    m = re.search(r'<div class="([^"]*)">' + re.escape(label), body)
    assert m is not None, f"{label!r} field-label not found in Camera & Pose"
    classes = m.group(1).split()
    assert "field-label" in classes and "field-label--sub" in classes, (
        f"{label!r} must carry both `field-label` and `field-label--sub`, got {classes}"
    )


def test_camera_pose_group_labels_do_not_carry_the_sub_class():
    """The direct children of the subgroup ("View Mode", "Camera",
    "See-Through") are one level UP from Distance/Height/Rotation and must
    stay at the plain `.field-label` weight -- otherwise every label in the
    section collapses back to one indistinguishable level."""
    html = _index()
    view_camera = re.search(
        r'data-subgroup-id="view-camera".*?</details>', html, re.DOTALL)
    assert view_camera is not None
    body = view_camera.group(0)
    for label in ("View Mode", "See-Through"):
        m = re.search(r'<div class="([^"]*)">' + re.escape(label), body)
        assert m is not None, f"{label!r} field-label not found in Camera & Pose"
        classes = m.group(1).split()
        assert "field-label" in classes
        assert "field-label--sub" not in classes, (
            f"{label!r} is a direct child of the subgroup, not a nested "
            f"sub-field -- it must not carry field-label--sub"
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


# ---------------------------------------------------------------------------
# BUG-061 -- credit-based /ws-mesh transport, World-follow "track, keep my
# framing", and scanner-model visibility keyed on view mode.
# ---------------------------------------------------------------------------

SLAM_JS = STATIC / "slam.js"
SCENE_JS = STATIC / "scene.js"
WS_JS = STATIC / "ws.js"
BROWSER_JS = STATIC / "browser.js"


def test_ws_js_opens_a_dedicated_mesh_socket():
    """MESH (tag 3) moved off `/ws` onto `/ws-mesh` so a whole-map re-send can
    never sit in front of the 30 Hz pose JSON (BUG-061 Part A). The client
    needs its own socket + ack path, not just a demux branch."""
    js = WS_JS.read_text(encoding="utf-8")
    assert "/ws-mesh" in js
    assert "ackMesh" in js
    assert "mesh_ack" in js


def test_slam_js_reuses_geometry_and_does_not_frustum_cull():
    """Two BUG-061 client-side requirements, both load-bearing for the same
    failure mode (a live-updating buffer silently going stale/invisible):

    - `frustumCulled = false` on the live-updating SLAM objects -- otherwise
      three.js caches `boundingSphere` from the first (possibly empty) packet
      and never recomputes it (see scene.js's own comment on `points`).
    - `mesh_ack` proves the client only frees the server's one-mesh-in-flight
      credit after the mesh is actually applied, not merely received.
    """
    js = SLAM_JS.read_text(encoding="utf-8")
    assert "frustumCulled = false" in js
    assert "mesh_ack" in js or "ackMesh" in js


def test_scanner_model_visible_only_in_world():
    """BUG-061 Part D: the pose marker used to be forced visible in every view
    mode, including FPV/Mirror where the camera sits AT the scanner and the
    model rendered as a giant box wrapped around the viewpoint. A single
    helper is the source of truth so the three call sites (both pose handlers
    plus the state-driven `applyState`) can't drift from each other."""
    js = SLAM_JS.read_text(encoding="utf-8")
    assert re.search(r"scanner\.visible\s*=.*view_mode\s*===\s*'world'", js), (
        "scanner visibility must be keyed on `state.view_mode === 'world'`"
    )
    # Regression guard: the historical unconditional `scanner.visible = true`
    # inside a pose handler must be gone from both sites.
    assert js.count("scanner.visible = true") == 0, (
        "a pose handler still force-shows the scanner outside World"
    )


def test_world_follow_tracks_target_without_snapping_camera():
    """Owner choice #1 for World + Follow ON, "track, keep my framing": the
    camera's zoom/orbit angle must survive a follow update, which rules out
    the FPV/Mirror-style eye/center snap (`setFollowTarget`). scene.js must
    expose a distinct API for it, and slam.js's World branch must call that
    API rather than the snap one."""
    scene = SCENE_JS.read_text(encoding="utf-8")
    slam = SLAM_JS.read_text(encoding="utf-8")
    assert "trackTarget" in scene, "scene.js must export a trackTarget API"
    assert "trackTarget" in slam, "slam.js must drive World-follow via trackTarget"
    # The World+follow branch in slam.js must not hand the server's nose-camera
    # eye/center to setFollowTarget -- that framing belongs to FPV/Mirror only.
    world_follow_block = re.search(
        r"view_mode\s*===\s*'world'[^;]*\{\s*\n\s*sceneApi\.(\w+)\(", slam)
    assert world_follow_block is not None, (
        "could not locate the World+follow branch in slam.js's `slam` handler"
    )
    assert world_follow_block.group(1) == "trackTarget"


def test_follow_toggle_lives_in_the_view_card():
    """Follow used to be buried in the SLAM card, which ships `hidden
    collapsed` -- the owner had never seen it. It must now sit in the View
    card's Camera & Pose sub-area, next to the World/FPV/Mirror segmented
    control, as a straight move (same id, so BUG-047's duplicate-id class
    can't recur) rather than a second mirrored control."""
    html = _index()
    ids = re.findall(r'\sid="([^"]+)"', html)
    assert ids.count("chk-slam-follow") == 1, (
        "chk-slam-follow must appear exactly once (a mirrored control needs a "
        "distinct id wired to the same slam_opt message, per BUG-047)"
    )
    view_card = re.search(
        r'data-card-id="view">.*?data-subgroup-id="view-camera"(.*?)</details>',
        html, re.DOTALL)
    assert view_card is not None, "could not locate the View card's Camera & Pose sub-area"
    assert 'id="chk-slam-follow"' in view_card.group(1), (
        "chk-slam-follow must be inside the View card's Camera & Pose sub-area"
    )
    assert 'id="chk-slam-follow"' not in html.split('data-card-id="view"')[0], (
        "chk-slam-follow must not still be inside the SLAM card"
    )


def test_slam_hud_shows_the_compute_device():
    """Part B (GPU visibility, client half): the `slam` message's new `device`
    field (e.g. "CUDA:0") must reach the HUD, with a tooltip like every other
    HUD readout."""
    html = _index()
    assert 'id="slam-device"' in html
    assert re.search(r'<div class="hud-row"[^>]*title="[^"]+"[^>]*>\s*'
                      r'<span>Device</span><span class="hud-val" id="slam-device">', html), (
        "the Device HUD row must carry a title tooltip"
    )
    slam = (STATIC / "slam.js").read_text(encoding="utf-8")
    assert "slam-device" in slam and "m.device" in slam


# ---------------------------------------------------------------------------
# SLAM HUD counters and stage timing (plan item 2, 2026-08-02:
# docs/superpowers/plans/2026-08-02-slam-compute-and-transport-followups.md).
# ---------------------------------------------------------------------------

_SLAM_STAGE_TIMING_IDS = (
    "slam-backend", "slam-throughput", "slam-overwritten",
    "slam-gpu-util", "slam-gpu-vram",
    "slam-raycast-ms", "slam-icp-ms", "slam-integrate-ms",
    "slam-mesh-extract-ms", "slam-mesh-prep-ms", "slam-mesh-pack-ms",
    "slam-mesh-bytes",
)


def _row_title(html: str, element_id: str) -> str:
    """The `title=` on the `.hud-row` that contains `id="<element_id>"`, or
    None. Keyed on id, not label -- several of these ids share a label
    ("VRAM", "Backend") with an unrelated row elsewhere on the page."""
    m = re.search(
        r'<div class="hud-row" title="([^"]+)"[^>]*>\s*<span>[^<]*</span>'
        r'<span class="hud-val" id="' + re.escape(element_id) + r'"', html)
    return m.group(1) if m else None


def test_slam_hud_new_counters_and_stage_timing_have_tooltips():
    """Every new plan-item-2 readout carries a `title=`, same convention as
    the rest of the SLAM HUD (`slam-device`) and the per-stream tooltip rule
    in hud.js. None of these are `<button>`/toggle `<label>` elements, so
    `test_every_button_has_a_tooltip` cannot see them -- this is the
    equivalent check for the SLAM HUD's plain `.hud-row`s."""
    html = _index()
    missing = [i for i in _SLAM_STAGE_TIMING_IDS if not _row_title(html, i)]
    assert not missing, f"SLAM HUD rows with no title= tooltip: {missing}"

    # The collapsed diagnostics drawer's own affordance needs one too.
    assert re.search(r'<summary title="[^"]+"[^>]*>Stage timing</summary>', html), (
        "the 'Stage timing' <details><summary> must carry a title tooltip"
    )


def test_slam_hud_new_ids_are_unique_and_present_in_slam_js():
    """Sanity companion to `test_no_duplicate_element_ids`: every id this
    change introduces actually exists (once) and is wired up client-side."""
    html = _index()
    slam = (STATIC / "slam.js").read_text(encoding="utf-8")
    for element_id in _SLAM_STAGE_TIMING_IDS:
        assert html.count(f'id="{element_id}"') == 1, f"{element_id} missing or duplicated"
        assert element_id in slam, f"{element_id} is not referenced from slam.js"


def test_overwritten_frame_readout_is_distinct_from_tracking_lost():
    """Plan item 2 is explicit: an overwritten frame (never reached
    `Mapper.step`) and a tracking-lost frame (reached the mapper, failed to
    register) are different failures and must not share a label, a tile, or
    a colour. Checked three ways: distinct element ids (so distinct DOM
    tiles), distinct CSS classes driving colour (`lost` vs `warn`, styled
    differently in index.html's `<style>`), and the Overwritten tooltip must
    itself name the contrast rather than presenting as a plain counter.
    """
    html = _index()
    assert 'id="slam-track"' in html and 'id="slam-overwritten"' in html
    assert "slam-track" != "slam-overwritten"   # trivially true; documents the intent

    over_title = _row_title(html, "slam-overwritten")
    assert over_title is not None
    assert "never reached the mapper" in over_title.lower()
    assert "tracking" in over_title.lower() and "lost" in over_title.lower()

    slam = (STATIC / "slam.js").read_text(encoding="utf-8")
    assert "classList.toggle('lost'" in slam, "Tracking row must still drive the 'lost' class"
    assert "classList.toggle('warn'" in slam, "Overwritten row must drive its own 'warn' class"

    # The two classes must resolve to different colours, not just different names.
    lost_rule = re.search(r"\.hud-val\.lost\s*\{([^}]*)\}", html)
    warn_rule = re.search(r"\.hud-val\.warn\s*\{([^}]*)\}", html)
    assert lost_rule is not None and warn_rule is not None
    assert lost_rule.group(1).strip() != warn_rule.group(1).strip()


def test_integrate_ms_tooltip_warns_it_is_a_dispatch_time_lower_bound():
    """`integrate_ms` is, on CUDA, usually a dispatch-time lower bound (no
    forced `cuda.synchronize()`), unlike `raycast_ms`/`icp_ms` which already
    sync internally. Presenting it as GPU work time would be misleading, so
    the tooltip must say so -- this pins the exact caveat, not just presence
    of a title."""
    html = _index()
    title = _row_title(html, "slam-integrate-ms")
    assert title is not None
    assert "DISPATCH" in title
    assert "not kernel-completion time" in title
    assert "cuda.synchronize()" in title.lower() or "sync" in title.lower()


def test_gpu_scope_is_never_presented_as_plain_device_wide_decoration():
    """`gpu_util_scope`/`vram_scope` are not decoration (plan item 2): a
    device-wide reading proves *some* CUDA kernel ran, not that SLAM's did.
    The tooltip must carry that caveat, and the scope must be visible in the
    label (not just the tooltip) per the task's "prefer visible in the label
    when device-wide" instruction."""
    html = _index()
    util_title = _row_title(html, "slam-gpu-util")
    assert util_title is not None
    assert "device" in util_title.lower() and "process" in util_title.lower()
    assert "not necessarily slam" in util_title.lower()

    vram_title = _row_title(html, "slam-gpu-vram")
    assert vram_title is not None
    assert "device-wide" in vram_title.lower()

    slam = (STATIC / "slam.js").read_text(encoding="utf-8")
    assert "scopeLabel" in slam, "the scope must be rendered into the visible label, not just the tooltip"


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


# ---------------------------------------------------------------------------
# Ranging profiles / manual sensor control / IMU-env poll rate (Task 10,
# docs/superpowers/plans/completed/2026-07-31-high-framerate-and-manual-ranging-modes.md).
# ---------------------------------------------------------------------------
#
# Numeric bounds are imported from `roomscan.profiles` (the single host-side
# owner of every range/step) rather than copied, so a future coefficient or
# range change in that module fails THIS test instead of silently desyncing
# the markup from the model that validates the values it sends.

from roomscan import profiles as _profiles  # noqa: E402


def test_ranging_profile_selector_replaces_the_old_usecase_control():
    """The old two-button Usecase segmented control is gone; the four-way
    Stability / Precision / High Frame Rate / Manual selector replaces it
    entirely (plan Task 10 item 4)."""
    html = _index()
    assert 'id="seg-usecase"' not in html
    assert 'data-uc=' not in html
    seg = re.search(r'<div class="segmented" id="seg-ranging-profile">(.*?)</div>', html, re.DOTALL)
    assert seg is not None, "seg-ranging-profile not found"
    profiles_present = set(re.findall(r'data-profile="([^"]+)"', seg.group(1)))
    assert profiles_present == {"stability", "precision", "high_framerate", "manual"}


def test_every_new_ranging_input_has_a_tooltip():
    """`test_every_button_has_a_tooltip` only scans `<button>`; the manual/IMU
    panels add `<input type="range">`/`<input type="number">` controls that
    need the same coverage (plan Task 10 item 9: "a `title` on EVERY new
    control")."""
    html = _index()
    ids = ("sl-manual-fps", "num-manual-fps", "sl-manual-exposure", "num-manual-exposure",
          "sl-imu-env-rate", "num-imu-env-rate")
    missing = []
    for element_id in ids:
        m = re.search(r'<input\b[^>]*\bid="' + re.escape(element_id) + r'"[^>]*>', html)
        assert m is not None, f"input id={element_id} not found"
        if "title=" not in m.group(0):
            missing.append(element_id)
    assert not missing, f"ranging inputs without a title= tooltip: {missing}"


def _input_attrs(html: str, element_id: str) -> dict:
    m = re.search(r'<input\b[^>]*\bid="' + re.escape(element_id) + r'"[^>]*>', html)
    assert m is not None, f"input id={element_id} not found"
    tag = m.group(0)
    attrs = {}
    for name in ("min", "max", "step", "value"):
        am = re.search(name + r'="([^"]*)"', tag)
        if am:
            attrs[name] = am.group(1)
    return attrs


@pytest.mark.parametrize("slider_id, number_id", [
    ("sl-manual-fps", "num-manual-fps"),
    ("sl-manual-exposure", "num-manual-exposure"),
    ("sl-imu-env-rate", "num-imu-env-rate"),
])
def test_manual_paired_inputs_agree_with_each_other(slider_id, number_id):
    """The range and number half of each paired input must share the same
    min/max/step, or the two widgets would silently accept different values
    for the identical field."""
    html = _index()
    s = _input_attrs(html, slider_id)
    n = _input_attrs(html, number_id)
    assert s["min"] == n["min"]
    assert s["max"] == n["max"]
    assert s["step"] == n["step"]


def test_manual_fps_bounds_match_profiles_py():
    html = _index()
    a = _input_attrs(html, "sl-manual-fps")
    assert int(a["min"]) == _profiles.FPS_MIN
    assert int(a["max"]) == _profiles.FPS_MAX


def test_manual_exposure_bounds_match_profiles_py():
    html = _index()
    a = _input_attrs(html, "sl-manual-exposure")
    assert int(a["min"]) == _profiles.EXPOSURE_MS_MIN
    assert int(a["max"]) == _profiles.EXPOSURE_MS_MAX
    assert int(a["step"]) == _profiles.EXPOSURE_STEP_MS


def test_imu_env_rate_bounds_match_profiles_py():
    html = _index()
    a = _input_attrs(html, "sl-imu-env-rate")
    assert int(a["max"]) == _profiles.IMU_ENV_RATE_MAX_HZ


def test_preset_button_tooltips_carry_no_hardcoded_consequence_numbers():
    """The Stability/Precision/High Frame Rate tooltips are STATIC markup
    that cannot track a live `profiles.py` coefficient/preset change -- a
    concurrent hardware-investigation session amending
    `PRESETS[ProfileId.HIGH_FRAMERATE]`'s fps mid-review is exactly the case
    that silently staled a fps/range/% figure baked into this text before this
    test existed. Consequence NUMBERS belong only in the live `ranging`-echo-
    driven "Applied"/estimate readouts (`ranging-applied-val`/
    `ranging-range-val`/`ranging-power-val`/`ranging-i3c-caption`), which are
    always current because they come from the server on every change --
    never in a preset button's own `title=`."""
    html = _index()
    for profile in ("stability", "precision", "high_framerate"):
        m = re.search(r'data-profile="' + profile + r'"[^>]*title="([^"]+)"', html)
        assert m is not None, f"data-profile={profile!r} button not found"
        tooltip = m.group(1)
        assert not re.search(r"\d", tooltip), (
            f"{profile} tooltip contains a digit (a hardcoded consequence number "
            f"that WILL go stale): {tooltip!r}"
        )


def test_ranging_i3c_bar_uses_the_documented_thresholds():
    """The spec's own thresholds (green <70%, yellow 70-85%, red >85%) --
    controls.js must classify the bar with exactly these cutoffs, matching
    the `.hud-bar__fill`/`is-warn`/`is-crit` convention every other bar in
    this app already uses (Resources card, TSDF blocks gauge)."""
    js = (STATIC / "controls.js").read_text(encoding="utf-8")
    assert "ranging-i3c-fill" in js
    assert re.search(r"frac\s*>=\s*0\.[67]0\s*&&\s*frac\s*<\s*0\.85", js), (
        "expected an is-warn band gated on [0.70, 0.85)"
    )
    assert re.search(r"frac\s*>=\s*0\.85", js), "expected an is-crit cutoff at 0.85"


def test_ranging_cdc_warning_element_exists_and_is_driven_by_server_estimate():
    """The CDC warning must come from the server's `estimate.transport_warning`
    field, never a client-side fps>60 comparison -- the global constraint is
    "never guess from URL, browser location, or link rate"."""
    html = _index()
    assert 'id="ranging-cdc-warning"' in html
    js = (STATIC / "controls.js").read_text(encoding="utf-8")
    # Driven straight from the server's field -- no client-side fps>60 literal
    # comparison anywhere near it (the global constraint: never guess from
    # URL, browser location, or link rate).
    assert "rangingCdcWarning.hidden = !est.transport_warning" in js
    assert "> 60" not in js and ">= 60" not in js and "> 60.0" not in js


def test_device_card_reachable_from_squircle_rail():
    """Task 10 item 9's "Device card reachability": the card that now hosts
    the ranging profile/manual/IMU-env controls must still resolve through
    the squircle rail's permanent map of every panel (it starts collapsed,
    same as before this change)."""
    html = _index()
    assert re.search(r'<div class="control-group card collapsed" data-card-id="device">', html), (
        "the Device card must still exist, collapsed by default"
    )
    js = LAYOUT_JS.read_text(encoding="utf-8")
    icon_keys = _js_object_keys(js, "CARD_ICONS")
    title_keys = _js_object_keys(js, "CARD_TITLES")
    assert "device" in icon_keys and "device" in title_keys


def test_ranging_and_imu_env_pending_disable_their_own_controls_only():
    """Task 10 item 8: IMU/env is "a second, independent pending command, not
    a fifth field bolted onto Manual ranging" -- the two `pending` flags must
    gate DISJOINT sets of controls in controls.js, not one shared flag."""
    js = (STATIC / "controls.js").read_text(encoding="utf-8")
    assert "rangingPending" in js and "imuEnvPending" in js
    # The IMU/env controls must never be gated on the ranging pending flag.
    imu_block = js[js.index("segImuEnvMode?.addEventListener"):js.index("hub.on('ranging'")]
    assert "rangingPending" not in imu_block


# --- 2026-08-05: use-case presets, fps grey-out, number-input focus guard ------

def _controls() -> str:
    return (STATIC / "controls.js").read_text(encoding="utf-8")


def test_fps_ceiling_table_in_js_matches_the_python_measured_table():
    """controls.js mirrors profiles._MEASURED_CEILING_FPS so the fps grey-out can
    track the exposure live while editing. The two must not drift."""
    import sys
    sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
    from roomscan import profiles
    js = _controls()
    m = re.search(r"CEILING_FPS_TABLE\s*=\s*\[(.*?)\]\s*;", js, re.DOTALL)
    assert m is not None, "CEILING_FPS_TABLE not found in controls.js"
    pairs = [(int(a), int(b)) for a, b in re.findall(r"\[\s*(\d+)\s*,\s*(\d+)\s*\]", m.group(1))]
    assert pairs == list(profiles._MEASURED_CEILING_FPS), (
        "controls.js CEILING_FPS_TABLE has drifted from profiles._MEASURED_CEILING_FPS")


def test_manual_number_fields_have_a_focus_edit_guard():
    """Typing in the fps/exposure number box must not be clobbered by the ~4 Hz
    `ranging` re-seed (BUG-081): the re-seed is suppressed while a field has focus."""
    js = _controls()
    assert "manualEditing" in js
    assert "!manualEditing" in js  # folded into the re-seed guard
    assert re.search(r"numManualFps\?\.addEventListener\('focus'", js)
    assert re.search(r"numManualExposure\?\.addEventListener\('focus'", js)


def test_fps_cap_and_blurb_elements_present():
    html = _index()
    for el_id in ("ranging-blurb", "fps-cap", "fps-cap-ok", "fps-cap-no", "fps-cap-note"):
        assert f'id="{el_id}"' in html, f"missing #{el_id} in index.html"


def test_profile_blurbs_cover_the_preset_slugs():
    js = _controls()
    m = re.search(r"PROFILE_BLURBS\s*=\s*\{(.*?)\n\s*\};", js, re.DOTALL)
    assert m is not None
    keys = set(re.findall(r"(\w+)\s*:", m.group(1)))
    assert {"stability", "precision", "high_framerate", "manual"} <= keys


def test_oscillate_amplitude_slider_disabled_unless_oscillate_is_the_active_mode():
    """Issue #107 claim 2: the amplitude slider has zero effect under
    Continuous (scene.js `updateOscillate`'s wave logic only runs when
    `orbitMode === 'oscillate'`), so it must be disabled -- not just
    world-view-gated like Orbit Speed/mode, which both stay meaningful
    (rate/direction) under Continuous too.

    Reintroducing the defect (`!worldOnly` alone, the pre-fix binding) makes
    this fail: that expression is satisfied in World with Continuous
    selected, which is exactly the state the issue reports as "stays
    interactive when oscillate is off"."""
    js = _controls()
    m = re.search(r"const worldOnly = .+?slOrbitAmplitude\.disabled\s*=\s*(.+?);", js, re.DOTALL)
    assert m is not None, "no disabled binding found for slOrbitAmplitude"
    block, expr = m.group(0), m.group(1)
    # Must not be the bare worldOnly expression the pre-#107 code used --
    # that is satisfied in World with Continuous selected, exactly the state
    # the issue reports as "stays interactive when oscillate is off".
    assert expr.strip() != "!worldOnly", (
        "amplitude slider is still gated on worldOnly alone (issue #107 regression)")
    # Must reference orbit_mode (the only signal that distinguishes
    # Continuous from Oscillate), directly or via a variable defined in the
    # same block, and must still require World -- a locked view has nothing
    # to swing either.
    assert "orbit_mode" in block, f"amplitude disabled-binding does not gate on orbit_mode: {expr!r}"
    assert "worldOnly" in expr or re.search(r"=\s*worldOnly\b", block), (
        f"amplitude disabled-binding dropped the World-only gate: {expr!r}")


def test_oscillate_reversal_maps_each_threshold_to_the_opposite_travel_direction():
    """Issue #107 claim 1: Oscillate orbited forever instead of swinging.

    `updateOscillate` was running the whole time -- forcing `autoRotateSpeed`
    negative on the live rig saw it re-asserted positive within 500 ms -- but it
    could never reverse, because the two threshold branches were swapped.

    The convention is measured, not derived: OrbitControls' autoRotate calls
    `rotateLeft()`, so a POSITIVE `autoRotateSpeed` makes `getAzimuthalAngle()`
    DECREASE. Positive `oscDir` therefore drives `oscUnwrappedDeg` NEGATIVE, and
    the `<= -n` branch must select -1 while the `>= n` branch selects +1. With
    them swapped each branch re-asserted the direction the wave was already
    travelling, so it latched: measured live at an 18 deg amplitude, 80.1 deg of
    travel in 9 s with 0 reversals and `autoRotateSpeed` pinned at one sign.
    After the swap, the same 9 s window gives 1 reversal bounded to
    -20.4..+21.2 deg.

    This is a guard against a silent re-swap, not a behavioural test -- the
    behaviour is only observable with a real OrbitControls in a browser, and was
    verified there. Reintroducing the swap makes this fail.
    """
    js = (STATIC / "scene.js").read_text(encoding="utf-8")
    pos = re.search(r"if\s*\(\s*oscUnwrappedDeg\s*>=\s*n\s*\)\s*oscDir\s*=\s*(-?1)\s*;", js)
    neg = re.search(r"if\s*\(\s*oscUnwrappedDeg\s*<=\s*-\s*n\s*\)\s*oscDir\s*=\s*(-?1)\s*;", js)
    assert pos is not None, "no `oscUnwrappedDeg >= n` reversal branch found in scene.js"
    assert neg is not None, "no `oscUnwrappedDeg <= -n` reversal branch found in scene.js"
    assert pos.group(1) != neg.group(1), (
        "both oscillate thresholds select the same direction, so the wave can never "
        f"turn around: >=n -> {pos.group(1)}, <=-n -> {neg.group(1)}")
    assert pos.group(1) == "1", (
        "crossing +n must select oscDir=+1 (positive autoRotateSpeed decreases the "
        f"azimuth, sending the wave back down); got {pos.group(1)}")
    assert neg.group(1) == "-1", (
        "crossing -n must select oscDir=-1 (negative autoRotateSpeed increases the "
        f"azimuth, sending the wave back up); got {neg.group(1)}")


def test_profile_blurbs_carry_no_hardcoded_consequence_numbers():
    """Like the button tooltips, the static benefit blurbs must not bake in
    fps/exposure numbers that would stale on a preset retune -- the live config is
    in the Applied/estimate readouts."""
    js = _controls()
    m = re.search(r"PROFILE_BLURBS\s*=\s*\{(.*?)\n\s*\};", js, re.DOTALL)
    assert m is not None
    assert not re.search(r"\d", m.group(1)), "PROFILE_BLURBS contains a digit (stale-prone)"


# ---------------------------------------------------------------------------
# Record control relocation (issue #118) -- moved out of its own sidebar card
# ("#capture-card") into the top bar, as one prominent, always-visible
# control. A plain substring check ("id=\"btn-record\"" appears somewhere in
# the file) cannot tell a top-bar button from a sidebar one -- both would
# contain that exact string -- so this actually parses the document into a
# tag stack and asks what ELEMENT the button's id attribute opened inside.
# ---------------------------------------------------------------------------

_VOID_TAGS = {
    "area", "base", "br", "col", "embed", "hr", "img", "input",
    "link", "meta", "param", "source", "track", "wbr",
}


class _AncestryParser(HTMLParser):
    """Records, for each `id=` seen, the full open-tag stack (root -> self)
    at the moment that element was opened, plus how many times each id
    occurred at all -- so both "exactly once" and "who is its parent" can be
    asked from one real parse instead of independent regexes."""

    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.stack = []            # [(tag, {attr: value}), ...], root first
        self.first_stack = {}      # id -> stack snapshot (incl. itself) at first sight
        self.id_counts = {}        # id -> number of elements carrying it

    def _seen(self, tag, attrs_dict):
        eid = attrs_dict.get("id")
        if eid:
            self.id_counts[eid] = self.id_counts.get(eid, 0) + 1
            self.first_stack.setdefault(eid, list(self.stack) + [(tag, attrs_dict)])

    def handle_starttag(self, tag, attrs):
        attrs_dict = dict(attrs)
        self._seen(tag, attrs_dict)
        if tag not in _VOID_TAGS:
            self.stack.append((tag, attrs_dict))

    def handle_startendtag(self, tag, attrs):
        # Self-closed (`<path ... />`) elements never have children, so there
        # is nothing to push -- just record it if it carries an id.
        self._seen(tag, dict(attrs))

    def handle_endtag(self, tag):
        for i in range(len(self.stack) - 1, -1, -1):
            if self.stack[i][0] == tag:
                del self.stack[i:]
                return
        # Unmatched close tag (stray markup) -- ignore rather than raise; the
        # other structural tests in this file already guard well-formedness
        # of the parts they care about.


def _parsed_index() -> _AncestryParser:
    parser = _AncestryParser()
    parser.feed(_index())
    return parser


def test_record_control_relocated_out_of_the_capture_card_into_the_top_bar():
    """#btn-record must exist exactly once, must NOT be nested inside any
    `[data-card-id]` card (that was the old, now-removed `#capture-card`
    sidebar section), and must be nested inside the top bar (`#topbar`)."""
    parser = _parsed_index()

    assert parser.id_counts.get("btn-record") == 1, (
        f"expected exactly one #btn-record, found "
        f"{parser.id_counts.get('btn-record', 0)}"
    )

    stack = parser.first_stack["btn-record"]
    ancestors = stack[:-1]  # exclude the button itself

    card_ancestors = [tag for tag, attrs in ancestors if "data-card-id" in attrs]
    assert not card_ancestors, (
        "#btn-record is still nested inside a [data-card-id] card "
        f"({card_ancestors}); it must live in the top bar, not a sidebar card"
    )

    topbar_ancestor_ids = [attrs.get("id") for _tag, attrs in ancestors]
    assert "topbar" in topbar_ancestor_ids, (
        "#btn-record is not nested inside #topbar; ancestor ids were "
        f"{topbar_ancestor_ids}"
    )


def test_record_status_is_also_relocated_alongside_the_button():
    """#record-status (elapsed time / bytes readout) must move with the
    button, not linger behind in a stale sidebar card."""
    parser = _parsed_index()

    assert parser.id_counts.get("record-status") == 1
    stack = parser.first_stack["record-status"]
    ancestors = stack[:-1]

    assert not any("data-card-id" in attrs for _tag, attrs in ancestors), (
        "#record-status is still nested inside a [data-card-id] card"
    )
    assert "topbar" in [attrs.get("id") for _tag, attrs in ancestors]


def test_the_old_capture_card_id_is_gone():
    """`#capture-card` / `data-card-id="capture"` must not linger as dead
    markup once the Record control has moved out of it."""
    html = _index()
    assert 'id="capture-card"' not in html
    assert 'data-card-id="capture"' not in html


# ---------------------------------------------------------------------------
# Playback panel relocation (issue #123) -- moved out of its own sidebar card
# ("#transport-card") into a floating panel docked at the bottom of the 3D
# viewport. Same rationale as the #118 checks above: a plain substring check
# cannot tell a floating panel's button from a sidebar one, so this parses
# the tag stack and asks what ELEMENT each control's id attribute opened
# inside.
# ---------------------------------------------------------------------------

_TRANSPORT_CONTROL_IDS = (
    "btn-golive", "btn-playpause", "btn-transport-restart",
    "seg-speed", "chk-loop", "seek", "pos-status",
)


def test_transport_controls_appear_exactly_once():
    """Sanity companion to `test_no_duplicate_element_ids`: the move must not
    have left a stale second copy of any transport control id behind."""
    parser = _parsed_index()
    dupes = {i: parser.id_counts.get(i, 0) for i in _TRANSPORT_CONTROL_IDS
              if parser.id_counts.get(i, 0) != 1}
    assert not dupes, f"transport control ids not appearing exactly once: {dupes}"


def test_transport_controls_are_not_nested_in_a_sidebar_or_card():
    """None of the preserved transport control ids may be nested inside a
    `[data-card-id]` card (the old `#transport-card`) or inside either
    `.sidebar` (`#primary-bar`/`#secondary-bar`) -- that was exactly the
    coupling this issue removes (sidebar hide/resize/collapse must no longer
    affect playback)."""
    parser = _parsed_index()
    bad = {}
    for element_id in _TRANSPORT_CONTROL_IDS:
        stack = parser.first_stack[element_id]
        ancestors = stack[:-1]
        card_ancestors = [tag for tag, attrs in ancestors if "data-card-id" in attrs]
        sidebar_ids = [attrs.get("id") for _tag, attrs in ancestors
                       if attrs.get("id") in ("primary-bar", "secondary-bar")]
        if card_ancestors or sidebar_ids:
            bad[element_id] = {"card_ancestors": card_ancestors, "sidebar_ids": sidebar_ids}
    assert not bad, f"transport controls still coupled to sidebar/card chrome: {bad}"


def test_transport_controls_are_all_inside_the_new_playback_panel():
    """The successor container is `#playback-panel`: every preserved control
    id must resolve inside it, and the panel itself must be a floating
    element (not inside any sidebar, and outside the viewport render target
    `#canvas-container` -- it overlays it via CSS, not DOM nesting, same as
    `#toast-layer`/`#detailed-build-status`)."""
    parser = _parsed_index()

    assert parser.id_counts.get("playback-panel") == 1, (
        f"expected exactly one #playback-panel, found "
        f"{parser.id_counts.get('playback-panel', 0)}"
    )
    panel_ancestors = parser.first_stack["playback-panel"][:-1]
    panel_ancestor_tags = [tag for tag, _attrs in panel_ancestors]
    assert "aside" not in panel_ancestor_tags, (
        "#playback-panel is nested inside an <aside> sidebar; it must float "
        f"outside both bars, ancestor tags were {panel_ancestor_tags}"
    )
    assert not any("data-card-id" in attrs for _tag, attrs in panel_ancestors), (
        "#playback-panel is nested inside a [data-card-id] card"
    )

    for element_id in _TRANSPORT_CONTROL_IDS:
        stack = parser.first_stack[element_id]
        ancestor_ids = [attrs.get("id") for _tag, attrs in stack[:-1]]
        assert "playback-panel" in ancestor_ids, (
            f"#{element_id} is not nested inside #playback-panel; ancestor "
            f"ids were {ancestor_ids}"
        )


def test_playback_panel_has_no_data_card_id():
    """The panel is transient replay chrome, not a sidebar/squircle-rail
    card -- it must not carry `data-card-id`, or layout.js's collapse/
    persistence bookkeeping (`roomscan.card.<id>.collapsed`) would apply to
    it, and it would need a CARD_ICONS/CARD_TITLES entry it has no use for
    (`test_every_card_id_has_a_squircle_icon_and_title` would otherwise
    demand one)."""
    html = _index()
    m = re.search(r'<div\b[^>]*\bid="playback-panel"[^>]*>', html)
    assert m is not None, "#playback-panel not found"
    assert "data-card-id" not in m.group(0)


def test_playback_panel_floats_fixed_and_anchors_off_dock_bottom():
    """The panel must be positioned `fixed` (so it floats over the 3D scene
    rather than taking up flow layout) and anchored using `--dock-bottom`
    (the same variable the Event Log console height feeds into), so
    expanding the console can never slide it underneath/behind the panel."""
    css = _style_css()
    rule = re.search(r"\.playback-panel\s*\{([^}]*)\}", css)
    assert rule is not None, "no .playback-panel rule found"
    body = rule.group(1)
    assert "position: fixed" in body
    assert "--dock-bottom" in body


def test_pos_status_can_shrink_instead_of_overflowing_into_the_speed_group():
    """Regression guard (#123 follow-up, browser-measured at 1600x1000):
    `#pos-status` used to be `flex: none` inside `.playback-panel__group--seek`,
    a group whose own `min-width: 140px` is smaller than "#seek's min-width +
    gap + pos-status's natural content width" can ever be. Once the group's
    box was squeezed toward that floor in the single-row (>=1100px) layout,
    the unshrinkable pos-status painted past its own group's right edge and
    into the 18px inter-group gap, overlapping the Speed group's
    `.field-label` by a few px ("...4103 / 4103SPEED").

    The fix is `min-width: 0` (an item can only ever overflow its container
    if its min-width floor exceeds the space it is given) plus
    `overflow: hidden; text-overflow: ellipsis` so a genuinely too-small
    width truncates instead of visually overflowing. `flex: none` is
    EXACTLY the pre-fix binding (flex-shrink: 0 -- the item that could never
    give ground), so this fails against the pre-change markup.
    """
    css = _style_css()
    rule = re.search(r"\.playback-panel #pos-status\s*\{([^}]*)\}", css)
    assert rule is not None, "no `.playback-panel #pos-status` rule found"
    body = rule.group(1)
    assert "min-width: 0" in body, (
        "#pos-status must set min-width: 0 so it can shrink to fit whatever "
        "space its (also-shrinkable) parent group is actually given"
    )
    assert re.search(r"flex:\s*none\b", body) is None, (
        "#pos-status must not be flex: none (flex-shrink: 0) -- that is the "
        "exact pre-fix binding that let it overflow past its shrunk parent"
    )
    assert "overflow: hidden" in body and "text-overflow: ellipsis" in body, (
        "#pos-status must truncate rather than visually overflow when its "
        "available width is genuinely too small"
    )


def test_the_old_transport_card_id_is_gone():
    """`#transport-card` / `data-card-id="transport"` must not linger as dead
    markup once Playback has moved into the floating panel, and the
    'transport' CARD_ICONS/CARD_TITLES entries in layout.js (which existed
    only to give the old sidebar card a squircle button) must be gone too --
    the panel is not part of that system at all."""
    html = _index()
    assert 'id="transport-card"' not in html
    assert 'data-card-id="transport"' not in html

    js = LAYOUT_JS.read_text(encoding="utf-8")
    icon_keys = _js_object_keys(js, "CARD_ICONS")
    title_keys = _js_object_keys(js, "CARD_TITLES")
    assert "transport" not in icon_keys
    assert "transport" not in title_keys


# ---------------------------------------------------------------------------
# Issue #122 -- WASD free-camera navigation.
# ---------------------------------------------------------------------------


def _scene_js() -> str:
    return SCENE_JS.read_text(encoding="utf-8")


def test_scene_js_registers_keydown_keyup_and_blur_handlers():
    """WASD nav needs a keydown/keyup pair, plus a `blur` handler so a key
    held during a focus loss (alt-tab, DevTools) can't stick -- the matching
    keyup never arrives in that case."""
    js = _scene_js()
    assert re.search(r"addEventListener\(\s*['\"]keydown['\"]", js), (
        "scene.js must register a keydown listener for WASD nav"
    )
    assert re.search(r"addEventListener\(\s*['\"]keyup['\"]", js), (
        "scene.js must register a keyup listener for WASD nav"
    )
    assert re.search(r"addEventListener\(\s*['\"]blur['\"]", js), (
        "scene.js must clear held nav keys on window blur"
    )


def test_wasd_nav_guards_against_typing_context():
    """A WASD keypress must not fire while the user is typing in a text
    field -- the same event fires whether the input has focus or not, so the
    handler has to check the target itself."""
    js = _scene_js()
    assert re.search(r"INPUT['\"]?\s*\|\|.*TEXTAREA", js) or (
        "'INPUT'" in js and "'TEXTAREA'" in js and "'SELECT'" in js
    ), "no INPUT/TEXTAREA/SELECT guard found for the nav key handler"
    assert "isContentEditable" in js, (
        "no isContentEditable guard found for the nav key handler "
        "(a contenteditable region is also a typing context)"
    )


def test_wasd_nav_guards_against_modifier_keys():
    """Ctrl/Alt/Meta held means the keystroke belongs to the browser or OS
    (Ctrl+W closes the tab, Alt+D jumps to the address bar, etc.) -- the nav
    handler must back off rather than eating it."""
    js = _scene_js()
    assert re.search(r"e\.ctrlKey\s*\|\|\s*e\.altKey\s*\|\|\s*e\.metaKey", js), (
        "no ctrlKey/altKey/metaKey guard found before the nav keys are read"
    )


def test_wasd_nav_derives_movement_from_the_camera_world_matrix():
    """KNOWN TRAP (issue #107, verified live): three.js sign conventions are
    NOT what static reading suggests -- a positive `autoRotateSpeed`
    DECREASES azimuth. Movement vectors here must come from the camera's own
    world matrix (`extractBasis`/`matrixWorld`), never from an angle assumed
    by inspection."""
    js = _scene_js()
    assert "matrixWorld" in js and "extractBasis" in js, (
        "WASD movement must be derived from camera.matrixWorld.extractBasis, "
        "not an assumed angle convention (issue #107's known trap)"
    )
    # Regression guard: no camera-relative-angle helper (getAzimuthalAngle is
    # OrbitControls' own API, fine for oscillate; it must not also be reused
    # to derive WASD's forward/right, which is a different, camera-space
    # question extractBasis answers directly).
    nav_block = js[js.index("WASD free-camera"):js.index("--- real-time view mode")]
    assert "getAzimuthalAngle" not in nav_block


def test_wasd_movement_is_scaled_by_dt_and_shift_boosts_speed():
    """Frame-rate independence: movement must be multiplied by `dt`, not a
    fixed per-frame constant (this box renders nearer 13 fps than 60 -- see
    the `frameDelta` comment above `animate`). Shift is the idiomatic "move
    faster" modifier."""
    js = _scene_js()
    assert re.search(r"navStep\.copy\(navMove\)\.multiplyScalar\(speed\s*\*\s*dt\)", js), (
        "WASD movement must be multiplied by dt for frame-rate independence"
    )
    assert re.search(r"shiftDown\s*\?\s*MOVE_SPEED_FAST_MULT", js), (
        "held Shift must boost the base fly speed"
    )


def test_wasd_nav_coexists_with_world_follow_by_gating_trackTarget():
    """WASD must take the camera away from World+Follow's `trackTarget`
    (called every SLAM pose) rather than fighting it every frame -- the least
    surprising pairing is for a keypress to suspend the follow-pan and for
    releasing every key to hand it straight back on the next pose."""
    js = _scene_js()
    m = re.search(r"function trackTarget\(pos\)\s*\{\s*\n(?:.*\n)*?\s*if\s*\(([^)]*)\)\s*return;",
                  js)
    assert m is not None, "could not find trackTarget's early-return guard"
    assert "manualFlightActive" in m.group(1), (
        "trackTarget must gate on manualFlightActive so a WASD keypress "
        f"takes the camera; guard was: {m.group(1)!r}"
    )


def test_wasd_nav_restricted_to_world_view_mode():
    """FPV/Mirror are locked, scanner-relative views with no orbit to fly
    around (see `applyViewMode`) -- WASD must not fight that lock."""
    js = _scene_js()
    assert re.search(
        r"function applyManualFlight\(dt\)\s*\{\s*\n\s*if\s*\(\s*viewMode\s*!==\s*'world'",
        js,
    ), "applyManualFlight must early-return outside World view mode"


def test_wasd_nav_wired_into_the_render_loop():
    """The handler computing movement each frame must actually be called from
    `animate()`'s World branch, ahead of `controls.update`, so OrbitControls
    picks up the new camera+target pair in the same tick (see the
    `trackTarget`-style comment on `applyManualFlight`)."""
    js = _scene_js()
    assert re.search(
        r"applyManualFlight\(dt\);\s*\n\s*controls\.update\(dt\);", js
    ), "applyManualFlight must be called immediately before controls.update(dt) in animate()"


# ---------------------------------------------------------------------------
# #121 -- View mode is capture-focused: entering View makes the Captures
# browser the focal (expanded) sidebar card and collapses its neighbours.
# ---------------------------------------------------------------------------


def test_layout_js_defines_the_sidebar_focus_mechanism():
    """layout.js owns the reusable mechanism (it is the single place that
    knows the sidebar card set and the collapse-persistence key format), and
    exposes it on `window` so a module-scoped file like browser.js -- which
    cannot import a classic script -- can call it."""
    js = LAYOUT_JS.read_text(encoding="utf-8")
    assert "function focusSidebarCard" in js
    assert "window.__focusSidebarCard" in js
    # It must reuse the SAME localStorage key format a manual header click
    # uses (`getCardKey`), not a second, drifting persistence scheme.
    assert "getCardKey(id)" in js


def test_browser_js_drives_the_focus_policy_off_the_state_echo_not_a_click():
    """The policy must be wired through the `state` handler (one-way flow,
    §5), not a local click handler -- `state` is the only authority on which
    page ("source") the client is on."""
    js = BROWSER_JS.read_text(encoding="utf-8")
    assert "focusOnEnteringView" in js
    assert "window.__focusSidebarCard" in js

    # The call must live textually inside the `hub.on('state', ...)` handler,
    # not off some other event (a click, a `captures`/`session` echo).
    m = re.search(r"hub\.on\('state',\s*\(msg\)\s*=>\s*\{(.*?)\n    \}\);", js, re.DOTALL)
    assert m is not None, "could not find browser.js's hub.on('state', ...) handler"
    state_handler_body = m.group(1)
    assert "focusOnEnteringView" in state_handler_body, (
        "focusOnEnteringView must be invoked from the state handler, not "
        "wired to a click or a different hub event"
    )


def test_focus_on_entering_view_is_edge_triggered_not_reasserted_every_echo():
    """`state` re-broadcasts on every unrelated setting change (BUG-060's
    class of bug lives right here). The policy must gate on an actual
    Live->View TRANSITION -- comparing the new source against a REMEMBERED
    previous one -- never fire unconditionally just because `source ===
    'view'` is currently true, or it would refight a user who re-expanded a
    card a moment after arriving."""
    js = BROWSER_JS.read_text(encoding="utf-8")

    m = re.search(r"function focusOnEnteringView\(([^)]*)\)\s*\{(.*?)\n    \}", js, re.DOTALL)
    assert m is not None, "focusOnEnteringView() not found in browser.js"
    params, body = m.group(1), m.group(2)

    # It must take the previous source as an argument (the caller -- the
    # state handler -- is the one place that still has the OLD value before
    # overwriting its `source` variable).
    assert "prevSource" in params, (
        "focusOnEnteringView must accept the previous source, not read a "
        "module-level variable that has already been overwritten by the "
        "time it runs"
    )
    # And it must actually gate on that argument, not just have it in scope.
    assert "prevSource" in body
    assert "'view'" in body


def test_focus_policy_scopes_to_the_two_sidebars_not_the_bottom_console():
    """The event-log console (`#log-console`, `data-card-id="log"`) is a
    bottom console outside `#primary-bar`/`#secondary-bar`, not a sidebar
    card competing for the same attention as the Captures browser -- the
    mechanism must not reach for it."""
    js = LAYOUT_JS.read_text(encoding="utf-8")
    m = re.search(r"function sidebarCardIds\(\)\s*\{(.*?)\n    \}", js, re.DOTALL)
    assert m is not None, "sidebarCardIds() not found in layout.js"
    body = m.group(1)
    assert "primary-bar" in body and "secondary-bar" in body
    assert "log-console" not in body


def test_captures_browser_data_card_id_still_matches_the_focus_target():
    """The mechanism focuses the card literally named 'browser' -- pin that
    id against the real markup so a future rename of the Captures browser's
    `data-card-id` (it is not `#capture-card` any more, see #118) is caught
    here rather than silently focusing nothing."""
    assert 'data-card-id="browser"' in _index()
    js = BROWSER_JS.read_text(encoding="utf-8")
    assert "focusOnEnteringView" in js
    assert "'browser'" in js
