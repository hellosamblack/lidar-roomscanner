"""Regression tests for #108: splat/detailed default camera must be at human
eye level, centred on the room, sized to actually show the whole room.

Before this fix, `splat.js`'s `frameCamera()` hard-coded `cam.position.set(0,
-2.5, 7)` on every load -- a number tuned once against a single preset's own
~3-unit-radius normalisation. It ignored the loaded content's actual position
and scale entirely, so an imported splat or a differently-cropped build landed
anywhere from "inside a wall" to "a speck in the void", and 2.5 m off the
floor is a stepladder, not eye level. Detailed (SLAM mesh) mode had no
content-aware framing at all -- World mode's `DEFAULT_VIEW_CAM.world` offset
is relative to the SENSOR'S OWN ORIGIN, meaningless once the reconstructed
mesh has grown away from where the scan started.

The fix is `frameRoomBBox` in `scene.js`: a pure function (plain numbers in,
plain numbers out, no THREE/WebGL runtime) that turns a room's bounding box
into an eye/target pose. `splat.js` and `slam.js`'s Detailed path both feed it
their own geometry's bbox via `scene.js`'s `frameCameraToBBox`.

These tests EXECUTE the shipped function under Node (the same cross-check
pattern as `test_capture_browser_tooltip.py`/`test_protocol_c_crosscheck.py`)
rather than reimplementing its arithmetic in Python -- a Python port could
stay green while the shipped code regressed. Skips (not fails) if Node is
unavailable, matching that fixture's handling of a missing toolchain.
"""

from __future__ import annotations

import json
import math
import shutil
import subprocess
from pathlib import Path

import pytest

SCENE_JS = Path(__file__).parent.parent / "src" / "roomscan" / "static" / "scene.js"

EYE_HEIGHT_M = 1.83  # ~6 ft, the issue's own acceptance number


def _extract_frame_room_bbox_source() -> str:
    """Pull `export const ROOM_EYE_HEIGHT_M` through the end of
    `frameRoomBBox`'s matching closing brace -- its own dependencies
    (`ROOM_FIT_MARGIN`, `ROOM_MIN_DISTANCE_M`) are declared right above it in
    the same block, so this slice is self-contained once `export` is stripped.
    """
    text = SCENE_JS.read_text(encoding="utf-8")
    start = text.index("export const ROOM_EYE_HEIGHT_M")
    fn_marker = "export function frameRoomBBox(bbox, fovDeg, opts = {}) {"
    fn_start = text.index(fn_marker, start)
    # Start brace-counting AT the function body's own opening brace (the last
    # char of the marker) -- starting any earlier double-counts the `{}`
    # default-parameter object literal in `opts = {}` and matches its close
    # as if it were the function's own, truncating the extraction mid-body.
    body_open = fn_start + len(fn_marker) - 1
    assert text[body_open] == "{"

    depth = 1
    i = body_open + 1
    end = None
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
        i += 1
    assert end is not None, "could not find matching closing brace for frameRoomBBox()"
    return text[start:end].replace("export const", "const").replace("export function", "function")


def _node() -> str | None:
    return shutil.which("node")


def _frame(bbox, fov_deg=60, **opts):
    node = _node()
    if node is None:
        pytest.skip("no node on PATH")
    if not SCENE_JS.exists():
        pytest.skip(f"scene.js not found at {SCENE_JS}")

    src = _extract_frame_room_bbox_source()
    harness = f"""
    {src}
    const result = frameRoomBBox({json.dumps(bbox)}, {json.dumps(fov_deg)}, {json.dumps(opts)});
    console.log(JSON.stringify(result));
    """
    proc = subprocess.run([node, "-e", harness], capture_output=True, text=True, timeout=30)
    assert proc.returncode == 0, f"node harness failed:\n{proc.stderr}"
    return json.loads(proc.stdout)


# A realistic room: 6 m wide (X), 8 m deep (Z), 2.4 m of vertical extent (Y).
# Y-DOWN world (gravity +Y): the sensor started near Y=0, so the ceiling sits
# at Y=-1.2 (above) and the floor at Y=+1.2 (below).
ROOM_BBOX = {"minX": -3.0, "maxX": 3.0, "minY": -1.2, "maxY": 1.2, "minZ": -4.0, "maxZ": 4.0}


# --------------------------------------------------------------------------
# 1. Eye height: ~6 ft off the FLOOR, not off the world origin.
# --------------------------------------------------------------------------


def test_eye_is_at_human_eye_height_above_the_floor():
    pose = _frame(ROOM_BBOX)
    assert pose is not None
    assert pose["eye_height_m"] == pytest.approx(EYE_HEIGHT_M, abs=0.01), (
        "camera must sit ~1.83 m (6 ft) above the room's OWN floor (#108)"
    )
    floor_y = ROOM_BBOX["maxY"]  # Y-down: floor is the larger Y
    assert pose["eye"][1] == pytest.approx(floor_y - EYE_HEIGHT_M, abs=0.01)


def test_eye_height_tracks_the_rooms_own_floor_not_a_world_constant():
    """The regression this guards against: a room translated 5 m down the Y
    axis must move the computed eye position with it -- a hard-coded world
    height (the pre-fix `-2.5`) would stay put and silently drift to the
    wrong height relative to the NEW floor. Reintroducing that defect (fixing
    `eye[1]` to a constant regardless of `bbox`) makes this fail."""
    shifted = dict(ROOM_BBOX)
    shifted["minY"] += 5.0
    shifted["maxY"] += 5.0
    pose = _frame(shifted)
    assert pose["eye_height_m"] == pytest.approx(EYE_HEIGHT_M, abs=0.01)
    assert pose["eye"][1] == pytest.approx(shifted["maxY"] - EYE_HEIGHT_M, abs=0.01)


def test_eye_height_clamps_inside_a_very_low_room():
    """A room shorter than 1.83 m (e.g. a crawlspace, or a noisy/partial
    capture) must not push the eye through the ceiling."""
    low = {"minX": -1, "maxX": 1, "minY": -0.3, "maxY": 0.3, "minZ": -1, "maxZ": 1}
    pose = _frame(low)
    assert low["minY"] < pose["eye"][1] < low["maxY"], (
        "the eye must stay within the room's own vertical extent (#108)"
    )


# --------------------------------------------------------------------------
# 2. Horizontal centring: the camera looks at the room's OWN centroid.
# --------------------------------------------------------------------------


def test_target_is_the_rooms_horizontal_centroid():
    pose = _frame(ROOM_BBOX)
    expected_x = (ROOM_BBOX["minX"] + ROOM_BBOX["maxX"]) / 2
    expected_z = (ROOM_BBOX["minZ"] + ROOM_BBOX["maxZ"]) / 2
    assert pose["target"][0] == pytest.approx(expected_x, abs=1e-6)
    assert pose["target"][2] == pytest.approx(expected_z, abs=1e-6)
    # The eye is directly behind the target on the same horizontal axis (X),
    # so the room's centroid is centred in frame, not off to one side.
    assert pose["eye"][0] == pytest.approx(expected_x, abs=1e-6)


def test_centroid_tracks_a_room_not_centred_on_the_origin():
    """The other half of the same regression: an off-origin room (a scan
    that wandered far from where it started) must still centre the shot on
    ITS OWN middle, not on world (0, 0)."""
    offset = {"minX": 20, "maxX": 26, "minY": -1.2, "maxY": 1.2, "minZ": 100, "maxZ": 108}
    pose = _frame(offset)
    assert pose["target"][0] == pytest.approx(23.0, abs=1e-6)
    assert pose["target"][2] == pytest.approx(104.0, abs=1e-6)


# --------------------------------------------------------------------------
# 3. Distance: tuned to the room's OWN size, and provably shows all of it.
# --------------------------------------------------------------------------


def test_distance_grows_with_room_size():
    small = {"minX": -1, "maxX": 1, "minY": -1, "maxY": 1, "minZ": -1, "maxZ": 1}
    big = {"minX": -5, "maxX": 5, "minY": -1, "maxY": 1, "minZ": -5, "maxZ": 5}
    d_small = _frame(small)["distance_m"]
    d_big = _frame(big)["distance_m"]
    assert d_big > d_small * 2, (
        "a bigger room must back the camera off further, or it either "
        "clips (too close) or wastes screen space (too far) -- #108 rule 3"
    )


def test_distance_actually_fits_the_whole_room_in_the_vertical_fov():
    """Quantitative form of 'show the full room without excessive zoom': the
    bounding SPHERE (radius = half the box's 3D diagonal, so every corner is
    within `radius` of the target) must subtend an angle no larger than the
    camera's own half-vertical-FOV as seen from the computed eye. That is
    exactly `distance_m * sin(halfFov) >= radius` -- reproduced here
    independently of `frameRoomBBox`'s own internals as a real geometric
    check, not a recomputation of its formula."""
    fov_deg = 60
    for bbox in (ROOM_BBOX, {"minX": -0.5, "maxX": 0.5, "minY": -0.4, "maxY": 0.4,
                             "minZ": -0.5, "maxZ": 0.5}):
        pose = _frame(bbox, fov_deg=fov_deg)
        dx = bbox["maxX"] - bbox["minX"]
        dy = bbox["maxY"] - bbox["minY"]
        dz = bbox["maxZ"] - bbox["minZ"]
        radius = 0.5 * math.sqrt(dx * dx + dy * dy + dz * dz)
        half_fov = math.radians(fov_deg) / 2
        assert pose["distance_m"] * math.sin(half_fov) >= radius - 1e-6, (
            "every corner of the room's bounding box must fall within the "
            "camera's vertical FOV cone from the computed eye position"
        )


def test_distance_is_not_wastefully_far():
    """The other half of 'without underutilisation of screen space': the fit
    must be tight, not an arbitrarily generous pull-back. A tiny margin over
    the theoretical minimum distance, not 10x it."""
    fov_deg = 60
    pose = _frame(ROOM_BBOX, fov_deg=fov_deg)
    dx = ROOM_BBOX["maxX"] - ROOM_BBOX["minX"]
    dy = ROOM_BBOX["maxY"] - ROOM_BBOX["minY"]
    dz = ROOM_BBOX["maxZ"] - ROOM_BBOX["minZ"]
    radius = 0.5 * math.sqrt(dx * dx + dy * dy + dz * dz)
    half_fov = math.radians(fov_deg) / 2
    min_distance = radius / math.sin(half_fov)
    assert pose["distance_m"] <= min_distance * 1.3


# --------------------------------------------------------------------------
# 4. Degenerate input.
# --------------------------------------------------------------------------


def test_degenerate_single_point_box_returns_null():
    point = {"minX": 1, "maxX": 1, "minY": 1, "maxY": 1, "minZ": 1, "maxZ": 1}
    assert _frame(point) is None


def test_non_finite_box_returns_null():
    broken = {"minX": None, "maxX": 1, "minY": -1, "maxY": 1, "minZ": -1, "maxZ": 1}
    assert _frame(broken) is None
