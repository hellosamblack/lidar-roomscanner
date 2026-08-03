"""Phase 1 web-instrument backend tests (spec §10.1).

Strategy: the protocol/coloring/classification logic in ``roomscan.web`` is
factored into pure, socket-free module-level helpers, so the bulk of these
tests exercise those helpers directly (no server, no event loop). Only the
broadcaster fan-out regression (§5.3) needs a live server -- that one spins a
real ``uvicorn.Server`` on a background thread and connects two ``websockets``
clients, because it is specifically about the WebSocket transport (one
broadcast task feeding every client, no frame-stealing).
"""
from __future__ import annotations

import asyncio
import json
import math
import socket
import struct
import threading
import time
from dataclasses import fields
from pathlib import Path
import pathlib

import numpy as np
import pytest
from types import SimpleNamespace

from roomscan import panel, web
from roomscan.control import CommandDispatcher
from roomscan.deproject import Deprojector
from roomscan.ir_image import reflectance_to_rgb
from roomscan.logbus import LogBus
from roomscan.metrics import MetricsRegistry, MetricsSnapshot, StreamRate
from roomscan.pipeline import TransformStage
from roomscan.protocol import (
    CommandCode,
    Frame,
    FrameHeader,
    FrameType,
    StreamId,
    pack_frame,
)
from roomscan.sensors import (
    SensorState,
    T_CV_TO_BODY,
    T_WORLD_TO_CV,
    boresight_view_deg,
    quat_to_matrix,
)
from roomscan.sources import FileSource
from roomscan.viewer import Stats


# =============================================================================
# 1. Protocol framing (pure) -- pack_point_cloud / pack_ir_image
# =============================================================================

def test_pack_point_cloud_tag_and_length():
    n = 5
    pts = np.arange(3 * n, dtype=np.float32).reshape(n, 3)
    colors = (np.arange(3 * n, dtype=np.float32).reshape(n, 3) + 100.0)
    blob = web.pack_point_cloud(pts, colors)

    # leading 4-byte little-endian tag == 1
    (tag,) = struct.unpack_from("<I", blob, 0)
    assert tag == web.TAG_POINT_CLOUD == 1
    # length == 4 + 24*N (tag + f32[3N] pos + f32[3N] col)
    assert len(blob) == 4 + 24 * n


def test_pack_point_cloud_roundtrip():
    n = 4
    pts = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9], [-1, -2, -3]], dtype=np.float32)
    colors = np.array([[0.0, 0.1, 0.2], [0.3, 0.4, 0.5],
                       [0.6, 0.7, 0.8], [0.9, 1.0, 0.05]], dtype=np.float32)
    blob = web.pack_point_cloud(pts, colors)

    body = np.frombuffer(blob[4:], dtype="<f4")
    got_pos = body[: 3 * n].reshape(n, 3)
    got_col = body[3 * n:].reshape(n, 3)
    np.testing.assert_array_equal(got_pos, pts)
    np.testing.assert_allclose(got_col, colors, rtol=0, atol=0)


def test_pack_ir_image_tag_dims_length_and_roundtrip():
    h, w = 6, 8
    rgb = np.arange(h * w * 3, dtype=np.uint8).reshape(h, w, 3)
    blob = web.pack_ir_image(rgb)

    tag, width, height = struct.unpack_from("<IHH", blob, 0)
    assert tag == web.TAG_IR_IMAGE == 2
    # width/height come from rgb.shape == (H, W, 3)
    assert (width, height) == (w, h)
    # length == 4 (tag) + 2 (w) + 2 (h) + W*H*3
    assert len(blob) == 4 + 2 + 2 + w * h * 3

    got = np.frombuffer(blob[8:], dtype=np.uint8).reshape(h, w, 3)
    np.testing.assert_array_equal(got, rgb)


# =============================================================================
# 2. JSON shapes (pure) -- build_metrics_message
# =============================================================================

def _snapshot_with_nones() -> MetricsSnapshot:
    """Two streams; the second carries device_hz=None / jitter_ms=None to prove
    None -> JSON null survives the round-trip."""
    return MetricsSnapshot(
        render_fps=27.8,
        streams=[
            StreamRate(stream_id=StreamId.DEPTH_ZF32, label="ToF",
                       device_hz=28.0, host_hz=27.5, bytes_per_s=123456.0, jitter_ms=1.5),
            StreamRate(stream_id=StreamId.IMU_QUAT, label="IMU",
                       device_hz=None, host_hz=479.0, bytes_per_s=7680.0, jitter_ms=None),
        ],
        link_bytes_per_s=131136.0,
        resources=None,
        drops=3,
        gaps=1,
    )


def test_metrics_message_field_set_matches_snapshot_schema():
    msg = web.build_metrics_message(_snapshot_with_nones())

    # Top-level keys == MetricsSnapshot fields (+ the "type" discriminator).
    snap_fields = {f.name for f in fields(MetricsSnapshot)}
    assert set(msg) - {"type"} == snap_fields
    assert msg["type"] == "metrics"

    # Each stream's keys == StreamRate fields exactly.
    stream_fields = {f.name for f in fields(StreamRate)}
    for s in msg["streams"]:
        assert set(s) == stream_fields

    # resources is null in Phase 1.
    assert msg["resources"] is None


def test_metrics_message_none_survives_json_roundtrip():
    msg = web.build_metrics_message(_snapshot_with_nones())
    reloaded = json.loads(json.dumps(msg))

    assert reloaded["render_fps"] == pytest.approx(27.8)
    assert reloaded["link_bytes_per_s"] == pytest.approx(131136.0)
    assert reloaded["drops"] == 3
    assert reloaded["gaps"] == 1
    assert reloaded["resources"] is None

    tof, imu = reloaded["streams"]
    assert tof["label"] == "ToF" and tof["device_hz"] == pytest.approx(28.0)
    # None -> null -> None across the wire.
    assert imu["device_hz"] is None
    assert imu["jitter_ms"] is None
    assert imu["host_hz"] == pytest.approx(479.0)


# =============================================================================
# 3. Color-mode selection (pure) -- select_colors
# =============================================================================

def _synthetic_outputs(h=6, w=8):
    """Three KNOWN, distinct (H,W) planes so coloring differences are provable.

    depth varies left->right, reflectance varies top->bottom, confidence is a
    third independent gradient -- so a min-max normalize of each yields a
    different per-point ordering and hence different turbo colors.
    """
    col = np.arange(w, dtype=np.float32)[None, :]
    row = np.arange(h, dtype=np.float32)[:, None]
    depth = (1000.0 + 200.0 * np.broadcast_to(col, (h, w))).astype(np.float32)  # all valid mm
    reflectance = (10.0 + 5.0 * np.broadcast_to(row, (h, w))).astype(np.float32)
    confidence = (np.broadcast_to(col + 2.0 * row, (h, w))).astype(np.float32)
    return {"depth": depth, "reflectance": reflectance, "confidence": confidence}, h, w


def test_select_colors_shapes_dtypes_and_range():
    outputs, h, w = _synthetic_outputs()
    deproj = Deprojector(w, h)
    pts, colors, fell_back = web.select_colors(outputs, deproj, "depth")

    assert not fell_back
    assert pts.dtype == np.float32 and colors.dtype == np.float32
    assert pts.ndim == 2 and pts.shape[1] == 3
    assert colors.ndim == 2 and colors.shape[1] == 3
    assert pts.shape[0] == colors.shape[0] == h * w   # all cells valid
    assert colors.min() >= 0.0 and colors.max() <= 1.0


def test_select_colors_track_selected_plane():
    outputs, h, w = _synthetic_outputs()
    deproj = Deprojector(w, h)

    _, c_depth, fb_d = web.select_colors(outputs, deproj, "depth")
    _, c_refl, fb_r = web.select_colors(outputs, deproj, "reflectance")
    _, c_conf, fb_c = web.select_colors(outputs, deproj, "confidence")

    assert not (fb_d or fb_r or fb_c)
    # Distinct planes -> distinct colors (coloring tracks the SELECTED plane).
    assert not np.array_equal(c_depth, c_refl)
    assert not np.array_equal(c_depth, c_conf)
    assert not np.array_equal(c_refl, c_conf)


def test_select_colors_falls_back_to_depth_when_plane_missing():
    outputs, h, w = _synthetic_outputs()
    deproj = Deprojector(w, h)

    depth_only = {"depth": outputs["depth"]}       # reflectance/confidence absent
    _, c_depth, _ = web.select_colors(outputs, deproj, "depth")
    pts_fb, c_fb, fell_back = web.select_colors(depth_only, deproj, "reflectance")

    assert fell_back is True
    # The fallback result is exactly the depth-colored result.
    np.testing.assert_array_equal(c_fb, c_depth)


# =============================================================================
# 4. IR encoding (pure) -- reflectance_to_rgb feeding pack_ir_image
# =============================================================================

def test_ir_rgb_shape_dtype_matches_pack():
    h, w = 5, 7
    refl = np.linspace(0.0, 100.0, h * w, dtype=np.float32).reshape(h, w)
    rgb = reflectance_to_rgb(refl, colormap="gray", upscale=1)

    assert rgb.shape == (h, w, 3)
    assert rgb.dtype == np.uint8

    blob = web.pack_ir_image(rgb)
    _, width, height = struct.unpack_from("<IHH", blob, 0)
    assert (width, height) == (w, h)
    assert len(blob) == 8 + w * h * 3


def test_ir_gray_vs_turbo_bytes_differ():
    h, w = 4, 6
    refl = np.linspace(0.0, 50.0, h * w, dtype=np.float32).reshape(h, w)
    gray = web.pack_ir_image(reflectance_to_rgb(refl, colormap="gray"))
    turbo = web.pack_ir_image(reflectance_to_rgb(refl, colormap="turbo"))
    assert gray != turbo


def test_ir_frozen_range_holds_across_frames_while_auto_differs():
    h, w = 4, 6
    # Two frames with DIFFERENT dynamic ranges.
    frame_a = np.linspace(0.0, 50.0, h * w, dtype=np.float32).reshape(h, w)
    frame_b = np.linspace(100.0, 300.0, h * w, dtype=np.float32).reshape(h, w)

    # A frozen (vmin, vmax) applies the SAME normalization mapping to both
    # frames -> identical relative structure -> after subtracting the per-frame
    # difference the mapping is deterministic; concretely, a linearly-scaled
    # copy of a frame under a fixed range reproduces exactly.
    vmin, vmax = 0.0, 300.0
    froz_a = reflectance_to_rgb(frame_a, colormap="gray", vmin=vmin, vmax=vmax)
    froz_a2 = reflectance_to_rgb(frame_a, colormap="gray", vmin=vmin, vmax=vmax)
    # Same input + same frozen range == byte-identical (mapping is fixed).
    np.testing.assert_array_equal(froz_a, froz_a2)

    # Auto-range (vmin=vmax=None) rescales EACH frame to its own span, so two
    # differently-ranged frames that share the same *shape* of gradient collapse
    # to the same normalized image -- whereas the frozen range keeps them
    # distinct. Assert the frozen mapping distinguishes the two frames while
    # auto-range does not.
    auto_a = reflectance_to_rgb(frame_a, colormap="gray", vmin=None, vmax=None)
    auto_b = reflectance_to_rgb(frame_b, colormap="gray", vmin=None, vmax=None)
    froz_b = reflectance_to_rgb(frame_b, colormap="gray", vmin=vmin, vmax=vmax)

    # Under a fixed range, a higher-valued frame maps brighter -> different bytes.
    assert not np.array_equal(froz_a, froz_b)
    # Under per-frame auto-range, both frames' identical gradient shape normalizes
    # to the same image.
    np.testing.assert_array_equal(auto_a, auto_b)


# =============================================================================
# 5. Command dispatch -> bus classification (pure) -- classify_bus_line
# =============================================================================

@pytest.mark.parametrize("tail,expected_status", [
    ("OK applied=1", "ok"),
    ("REJECTED_BINNING applied=0", "ok"),          # any "<ResultCode> applied=<n>" is a success shape
    ("busy, command already in flight", "busy"),
    ("TIMEOUT no ACK for cmd=1 token=42 within 2.0s", "timeout"),
    ("ERROR SerialException('port gone')", "error"),
    ("not available in replay", "error"),
])
def test_classify_command_result_status(tail, expected_status):
    line = f"ping -> {tail}"
    msg = web.classify_bus_line(line)
    assert msg == {"type": "cmd", "label": "ping", "status": expected_status, "detail": tail}


def test_classify_event_line():
    msg = web.classify_bus_line("[event] code=2 detail=0 trigger timeout")
    assert msg == {"type": "event", "code": 2, "detail": 0, "msg": "trigger timeout"}


def test_classify_undecodable_event_is_log():
    line = "[event] undecodable payload (12 B)"
    assert web.classify_bus_line(line) == {"type": "log", "line": line}


def test_classify_plain_line_is_log():
    line = "reader stopped: RuntimeError('boom')"
    assert web.classify_bus_line(line) == {"type": "log", "line": line}


def test_classify_command_labels_gate():
    # With a command_labels set that does NOT include the label, a "->" line is
    # NOT classified as a command (belt-and-suspenders), it falls back to log.
    line = "unrelated -> OK applied=1"
    assert web.classify_bus_line(line, command_labels={"ping"}) == {"type": "log", "line": line}
    # When the label IS known, it classifies as a command.
    line2 = "ping -> OK applied=1"
    assert web.classify_bus_line(line2, command_labels={"ping"})["type"] == "cmd"


def test_classify_empty_returns_none():
    assert web.classify_bus_line("") is None


def test_dispatch_none_client_reaches_bus_classified_as_error():
    """Integration angle: a replay-mode dispatch (client=None) emits its result
    on the bus; draining + classifying reproduces §7.1's replay-unavailable
    mapping end to end."""
    bus = LogBus()
    handle = bus.subscribe()
    disp = CommandDispatcher(client=None, on_message=bus.publish)
    disp.dispatch(CommandCode.PING, 0, "ping")

    lines = list(bus.drain(handle))
    assert lines == ["ping -> not available in replay"]
    msg = web.classify_bus_line(lines[0])
    assert msg["type"] == "cmd" and msg["status"] == "error"
    assert msg["detail"] == "not available in replay"


# =============================================================================
# resolve_command (pure) -- inbound name -> (code, param, label)
# =============================================================================

def test_resolve_command_usecase_carries_id():
    assert web.resolve_command("usecase", 3) == (CommandCode.SET_USECASE, 3, "usecase 3")


def test_resolve_command_ping_and_unknown():
    assert web.resolve_command("ping", 0) == (CommandCode.PING, 0, "ping")
    assert web.resolve_command("bogus", 0) is None


# =============================================================================
# 6. Broadcaster fan-out, no frame-stealing (LIVE socket, §5.3 regression)
# =============================================================================

# =============================================================================
# Sensors (streams 9/10) -- build_sensor_message + reader integration (Phase 2)
# =============================================================================

def _sframe(sid, payload: bytes, t_us: int = 1000) -> Frame:
    return Frame(FrameHeader(FrameType.DATA, sid, 0, 1, t_us, 0, 0, len(payload)), payload)


# ---------------------------------------------------------------------------
# Gravity alignment of the live display (desktop-panel orbit parity).
# ---------------------------------------------------------------------------

def test_display_rotation_none_without_orientation():
    # ToF-only session: no quat => no rotation => raw sensor frame.
    assert web.display_rotation(None) is None


def test_display_rotation_is_the_canonical_sandwich():
    q = (0.92388, 0.38268, 0.0, 0.0)   # ~45 deg about x
    expect = T_WORLD_TO_CV @ quat_to_matrix(*q) @ T_CV_TO_BODY
    assert np.allclose(web.display_rotation(q), expect)


def test_rotate_points_noop_without_rotation():
    pts = np.arange(9, dtype=np.float32).reshape(3, 3)
    assert web.rotate_points(pts, None) is pts       # same array, no allocation


def test_rotate_points_matches_matrix_product_and_is_float32():
    pts = np.array([[1.0, 2.0, 3.0], [-4.0, 0.5, 2.0]], dtype=np.float32)
    rot = web.display_rotation((0.92388, 0.0, 0.38268, 0.0))
    out = web.rotate_points(pts, rot)
    assert out.dtype == np.float32
    assert np.allclose(out, (rot @ pts.T).T, atol=1e-5)


def test_rotate_points_handles_the_surface_grid_shape():
    # select_surface hands back (h, w, 3); the rotation must broadcast over it.
    grid = np.random.default_rng(0).normal(size=(4, 5, 3)).astype(np.float32)
    rot = web.display_rotation((0.70711, 0.70711, 0.0, 0.0))
    out = web.rotate_points(grid, rot)
    assert out.shape == (4, 5, 3)
    assert np.allclose(out.reshape(-1, 3), (rot @ grid.reshape(-1, 3).T).T, atol=1e-5)


# Board held upright the reference way (vertical, USB down, facing North):
# body X=Up -> world Z, body Y=Right -> world -Y, body Z=Forward -> world X. That
# matrix is a 180 deg turn about (1,0,1)/sqrt(2), hence this quaternion.
_Q_UPRIGHT = (0.0, 1.0 / np.sqrt(2.0), 0.0, 1.0 / np.sqrt(2.0))
_Q_ROLL_180 = (0.0, 0.0, 0.0, 1.0)     # 180 deg about body Z (the sensor's boresight)


def test_display_rotation_is_identity_when_held_upright():
    # Sanity-check the frame algebra end to end: held the reference way, the
    # gravity-aligned display frame IS the raw sensor frame, so nothing moves.
    assert np.allclose(web.display_rotation(_Q_UPRIGHT), np.eye(3), atol=1e-6)


def test_rotate_points_upside_down_board_is_flipped_upright():
    from roomscan.sensors import graft_yaw, quat_mul, quat_yaw_deg

    up_cv = np.array([[0.0, -1.0, 0.0]], dtype=np.float32)   # CV +Y is down
    upside_down = quat_mul(_Q_UPRIGHT, _Q_ROLL_180)

    # Held upright, a point above the sensor renders above it.
    assert web.rotate_points(up_cv, web.display_rotation(_Q_UPRIGHT))[0][1] < -0.9
    # Held upside down, that same raw pixel is physically below the sensor, and
    # alignment must render it below -- this is the reported bug.
    assert web.rotate_points(up_cv, web.display_rotation(upside_down))[0][1] > 0.9
    # Yaw alone can never undo the flip -- this is why "Reset Heading" (a yaw-only
    # fusion reset) is structurally incapable of fixing an upside-down view.
    yaw_stripped = graft_yaw(upside_down, -quat_yaw_deg(upside_down))
    assert web.rotate_points(up_cv, web.display_rotation(yaw_stripped))[0][1] > 0.9


# --- real-time view modes: World / FPV / Mirror (owner ask, 2026-07-29) -----

def test_view_rotation_world_is_the_gravity_alignment_unchanged():
    """World must be byte-for-byte the pre-existing behaviour: the same matrix
    object the broadcaster already computed, not a re-derivation."""
    grav = web.display_rotation((0.92388, 0.38268, 0.0, 0.0))
    assert web.view_rotation(grav, "world") is grav
    # ToF-only session (no orientation) stays None, as it always did.
    assert web.view_rotation(None, "world") is None


def test_view_rotation_fpv_keeps_the_boresight_dead_ahead():
    """FPV looks along the sensor's optical axis, so whatever the sensor has
    centred stays centred: CV +Z in, CV +Z out, however the board is held."""
    for quat in (_Q_UPRIGHT, (0.92388, 0.38268, 0.0, 0.0), (0.7, 0.1, -0.3, 0.64)):
        rot = web.view_rotation(web.display_rotation(quat), "fpv")
        ahead = web.rotate_points(np.array([[0.0, 0.0, 2.0]], dtype=np.float32), rot)
        assert np.allclose(ahead, [[0.0, 0.0, 2.0]], atol=1e-5)


def test_view_rotation_fpv_is_a_no_op_when_held_the_reference_way():
    """Held upright (the pose where `display_rotation` is the identity), the
    boresight view frame IS the raw sensor frame -- nothing moves. Anchors the
    frame algebra: if this drifts, every other FPV assertion is measuring the
    wrong thing."""
    rot = web.view_rotation(web.display_rotation(_Q_UPRIGHT), "fpv")
    assert np.allclose(rot, np.eye(3), atol=1e-6)


def test_view_rotation_fpv_levels_the_horizon_under_roll():
    """The owner requirement: FPV must "respect gravity the same way world
    does". Roll the board about its own boresight and the physically-up
    direction has to keep rendering straight UP on screen (CV -Y, no sideways
    lean) — the scene must not spin with the board."""
    world_up = np.array([0.0, -1.0, 0.0])       # Open3D CV world is Y-down
    for deg in (0.0, 30.0, 90.0, 180.0, -75.0):
        grav = web.display_rotation(_perturb(_Q_UPRIGHT, deg, (0.0, 0.0, 1.0)))
        up_cv = (grav.T @ world_up).astype(np.float32)[None, :]   # same ray, sensor frame

        out = web.rotate_points(up_cv, web.view_rotation(grav, "fpv"))[0]
        assert out[1] < -0.999, (deg, out)          # straight up...
        assert abs(out[0]) < 1e-4, (deg, out)       # ...and not leaning

        # Contrast: shipping the RAW sensor frame (the naive "FPV") tilts it,
        # which is the thing this frame exists to prevent.
        raw = up_cv[0]
        assert deg == 0.0 or not (raw[1] < -0.999 and abs(raw[0]) < 1e-4), deg


def test_view_rotation_fpv_survives_a_vertical_boresight():
    """Aimed straight at the ceiling, "level" is undefined about the boresight.
    A handheld scanner does that constantly, so the fallback must produce a
    finite, still-orthonormal frame rather than a NaN cloud."""
    # Rotate the reference pose about body Y (Right) so the boresight (body Z)
    # swings onto the world vertical.
    grav = web.display_rotation(_perturb(_Q_UPRIGHT, 90.0, (0.0, 1.0, 0.0)))
    assert abs(abs((grav @ np.array([0.0, 0.0, 1.0]))[1]) - 1.0) < 1e-6   # really vertical
    rot = web.view_rotation(grav, "fpv")
    assert np.all(np.isfinite(rot))
    assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-6)


def test_view_rotation_mirror_is_fpv_with_x_negated():
    """Mirror is a left-right flip about the vertical axis of the FPV view: in
    the CV frame (X=Right, Y=Down, Z=Forward) that is X alone -- up/down and
    range must be untouched, or the view would be upside down or inside out."""
    grav = web.display_rotation((0.92388, 0.38268, 0.0, 0.0))
    fpv = web.view_rotation(grav, "fpv")
    mirror = web.view_rotation(grav, "mirror")
    assert np.allclose(mirror, np.diag([-1.0, 1.0, 1.0]) @ fpv)

    pts = np.array([[0.4, -0.2, 1.5], [-0.9, 0.3, 2.0]], dtype=np.float32)
    a, b = web.rotate_points(pts, fpv), web.rotate_points(pts, mirror)
    assert np.allclose(b[:, 0], -a[:, 0], atol=1e-5)     # X flipped
    assert np.allclose(b[:, 1:], a[:, 1:], atol=1e-5)    # Y/Z untouched


def test_view_rotation_without_orientation_falls_back_to_the_sensor_frame():
    """ToF-only session: there is no gravity to level against, and the cloud is
    already the sensor's own view -- so FPV is a no-op and Mirror is the bare
    flip. It must not crash or silently blank the cloud."""
    assert web.view_rotation(None, "fpv") is None
    assert np.allclose(web.view_rotation(None, "mirror"), np.diag([-1.0, 1.0, 1.0]))


def test_view_rotation_keys_differ_so_the_cloud_cache_invalidates_on_a_switch():
    """The broadcaster's packed-bytes cache is keyed on `rotation_key(view_rot)`
    (plus seq/colour). If two modes shared a key, switching mode on a paused or
    stationary frame would keep serving the previous mode's bytes."""
    grav = web.display_rotation((0.92388, 0.38268, 0.0, 0.0))
    keys = [web.rotation_key(web.view_rotation(grav, m))
            for m in ("world", "fpv", "mirror")]
    assert len(set(keys)) == 3


def _perturb(quat, deg, axis=(1.0, 0.0, 0.0)):
    """Rotate `quat` by `deg` about a body axis — a stand-in for IMU noise."""
    from roomscan.sensors import quat_mul
    a = np.radians(deg) / 2.0
    n = np.array(axis, dtype=float) / np.linalg.norm(axis)
    return quat_mul(quat, (np.cos(a), *(np.sin(a) * n)))


def test_quat_slerp_endpoints_and_midpoint():
    a, b = _Q_UPRIGHT, _perturb(_Q_UPRIGHT, 40.0)
    assert web.quat_angle_deg(web.quat_slerp(a, b, 0.0), a) == pytest.approx(0.0, abs=1e-6)
    assert web.quat_angle_deg(web.quat_slerp(a, b, 1.0), b) == pytest.approx(0.0, abs=1e-4)
    assert web.quat_angle_deg(a, web.quat_slerp(a, b, 0.5)) == pytest.approx(20.0, abs=1e-3)


def test_quat_slerp_takes_the_short_way_round():
    a = _Q_UPRIGHT
    b = tuple(-v for v in _perturb(_Q_UPRIGHT, 10.0))   # same rotation, negated
    assert web.quat_angle_deg(a, web.quat_slerp(a, b, 1.0)) == pytest.approx(10.0, abs=1e-3)


def test_smoother_adopts_the_first_sample_without_ramping():
    # Must not ease in from identity on connect -- that would swing the whole
    # scene through a large arc on the first frame.
    sm = web.OrientationSmoother()
    assert sm.update(_Q_UPRIGHT) == pytest.approx(_Q_UPRIGHT)
    assert sm.update(None) is None


def test_smoother_damps_stationary_noise_to_sub_millimetre():
    # Replay the measured stationary profile (2026-07-28 live rig): zero-mean
    # ~0.14 deg/update wobble. What the eye reads as shimmer is the per-update
    # CHANGE, so that -- not excursion from truth -- is what must collapse.
    rng = np.random.default_rng(7)
    sm = web.OrientationSmoother()
    sm.update(_Q_UPRIGHT)
    prev_raw = prev_held = _Q_UPRIGHT
    raw_step, held_step = [], []
    for _ in range(200):
        noisy = _perturb(_Q_UPRIGHT, rng.normal(0.0, 0.14), axis=rng.normal(size=3))
        held = sm.update(noisy)
        raw_step.append(web.quat_angle_deg(prev_raw, noisy))
        held_step.append(web.quat_angle_deg(prev_held, held))
        prev_raw, prev_held = noisy, held
    # Frame-to-frame shimmer drops by an order of magnitude...
    assert np.mean(held_step) < np.mean(raw_step) / 10.0
    # ...to well under 0.02 deg/update, i.e. sub-millimetre at a 3 m lever arm.
    assert np.degrees(np.radians(np.mean(held_step))) < 0.02
    assert np.radians(np.mean(held_step)) * 3000.0 < 1.0        # mm at 3 m
    # And it must not wander off the true attitude while damping.
    assert web.quat_angle_deg(_Q_UPRIGHT, prev_held) < 0.1


def test_smoother_tracks_real_motion_one_to_one():
    # A deliberate sweep must not lag: past snap_deg the blend weight is 1.0.
    sm = web.OrientationSmoother()
    q = _Q_UPRIGHT
    sm.update(q)
    for _ in range(10):
        q = _perturb(q, 5.0)          # 5 deg/update, well past snap_deg=2.0
        held = sm.update(q)
    assert web.quat_angle_deg(held, q) == pytest.approx(0.0, abs=1e-4)


def test_smoother_does_not_suppress_before_the_window_fills():
    # No evidence of jitter yet => never damp. Motion right after connect must
    # pass straight through (same rule as StationarityGate's full-window guard).
    sm = web.OrientationSmoother()
    q = _Q_UPRIGHT
    sm.update(q)
    for _ in range(sm.window - 1):
        q = _perturb(q, 0.1)
        held = sm.update(q)
    assert web.quat_angle_deg(held, q) == pytest.approx(0.0, abs=1e-4)


def test_smoother_separates_a_slow_pan_from_jitter_of_the_same_magnitude():
    """The whole reason for coherence gating: identical per-update magnitude,
    opposite treatment. A magnitude deadband cannot tell these apart."""
    step = 0.15            # deg per update -- same for both cases
    rng = np.random.default_rng(11)

    # (a) incoherent: random directions => held should barely move.
    jit = web.OrientationSmoother()
    jit.update(_Q_UPRIGHT)
    for _ in range(60):
        noisy = _perturb(_Q_UPRIGHT, rng.normal(0.0, step), axis=rng.normal(size=3))
        held_jit = jit.update(noisy)
    lag_jitter = web.quat_angle_deg(_Q_UPRIGHT, held_jit)

    # (b) coherent: every increment the same way => held should keep up.
    pan = web.OrientationSmoother()
    q = _Q_UPRIGHT
    pan.update(q)
    for _ in range(60):
        q = _perturb(q, step)          # steady pan about a fixed axis
        held_pan = pan.update(q)
    lag_pan = web.quat_angle_deg(held_pan, q)

    assert lag_jitter < 0.1, "incoherent wobble should be held, not followed"
    assert lag_pan < 0.1, "a coherent pan must track without lagging"
    # The pan genuinely travelled while the jitter went nowhere.
    assert web.quat_angle_deg(_Q_UPRIGHT, held_pan) > 5.0


def test_smoother_snaps_on_a_single_large_step_even_while_gated():
    # Drive the gate closed with jitter, then flick: the flick must not be eaten.
    rng = np.random.default_rng(3)
    sm = web.OrientationSmoother()
    sm.update(_Q_UPRIGHT)
    for _ in range(30):
        sm.update(_perturb(_Q_UPRIGHT, rng.normal(0.0, 0.15), axis=rng.normal(size=3)))
    flick = _perturb(_Q_UPRIGHT, 20.0, axis=(0.0, 1.0, 0.0))
    assert web.quat_angle_deg(sm.update(flick), flick) == pytest.approx(0.0, abs=1e-4)


def test_smoother_converges_so_a_slow_pan_does_not_lag_forever():
    # Holding a new attitude must settle there, not park at a fixed offset.
    sm = web.OrientationSmoother()
    sm.update(_Q_UPRIGHT)
    target = _perturb(_Q_UPRIGHT, 0.2)      # inside the deadband: slowest path
    for _ in range(400):
        held = sm.update(target)
    assert web.quat_angle_deg(held, target) < 0.01


def test_ir_gravity_rot_turns_the_pane_when_board_is_upside_down():
    from roomscan.sensors import ir_gravity_rot, quat_mul

    assert ir_gravity_rot(_Q_UPRIGHT) == 0
    assert ir_gravity_rot(quat_mul(_Q_UPRIGHT, _Q_ROLL_180)) == 2   # two 90 deg turns


# ---------------------------------------------------------------------------
# Continuous IR gravity roll: the 90-deg snap plus a <=45-deg client residual.
# ---------------------------------------------------------------------------

def _rolled(deg):
    """_Q_UPRIGHT rolled `deg` about the sensor boresight (body Z)."""
    from roomscan.sensors import quat_mul
    a = np.radians(deg) / 2.0
    return quat_mul(_Q_UPRIGHT, (np.cos(a), 0.0, 0.0, np.sin(a)))


@pytest.mark.parametrize("roll", [0.0, 12.0, 40.0, 50.0, 90.0, 137.0, 180.0, -33.0, -95.0])
def test_snap_plus_residual_reconstructs_the_continuous_roll(roll):
    """The whole point of the split: snap + residual must equal the continuous
    angle, so the pane ends up exactly as level as the point cloud."""
    from roomscan.sensors import (ir_gravity_angle_deg, ir_gravity_residual_deg,
                                  ir_gravity_rot, wrap180)
    q = _rolled(roll)
    total = 90.0 * ir_gravity_rot(q) + ir_gravity_residual_deg(q)
    assert wrap180(total - ir_gravity_angle_deg(q)) == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("roll", [0.0, 12.0, 40.0, 50.0, 90.0, 137.0, 180.0, -33.0, -95.0])
def test_residual_never_exceeds_a_quarter_turn(roll):
    # Bounds the CSS rotation, which is what bounds the empty corners in the
    # square frame. Anything larger means the snap picked the wrong quarter.
    from roomscan.sensors import ir_gravity_residual_deg
    assert abs(ir_gravity_residual_deg(_rolled(roll))) <= 45.0 + 1e-9


def test_residual_is_what_the_snap_alone_would_miss():
    # A 40 deg roll snaps to ZERO turns -- the pane would not move at all while
    # the cloud tilts the full 40. This is the reported bug, in one assertion.
    from roomscan.sensors import ir_gravity_residual_deg, ir_gravity_rot
    q = _rolled(40.0)
    assert ir_gravity_rot(q) == 0
    assert ir_gravity_residual_deg(q) == pytest.approx(-40.0, abs=1e-3)


@pytest.mark.parametrize("roll", [0.0, 15.0, 30.0, 45.0, 90.0, 120.0, -30.0, -90.0])
def test_ir_gravity_angle_matches_the_point_cloud_rotation(roll):
    """THE sign guard. Derives the expected image rotation from the *verified*
    cloud path instead of restating the formula, so an inverted convention fails
    here even though it survives the obvious checks (a 180 deg flip is its own
    inverse; a 90 deg turn swaps width/height either way).

    Ground truth: the IR image and the cloud come off the same 54x42 grid, so
    wherever the aligned cloud puts the image's +u axis on screen is where the
    rotated image must put it too.
    """
    from roomscan.deproject import Deprojector
    from roomscan.sensors import ir_gravity_angle_deg

    q = _rolled(roll)
    r_disp = web.display_rotation(q)
    grid, _ = Deprojector(54, 42, 60.0, 45.0).grid(
        np.full((42, 54), 1000.0, dtype=np.float32))       # flat wall at 1 m
    a0 = r_disp @ grid[21, 20]
    a1 = r_disp @ grid[21, 33]                              # same row, +13 px in u
    # Screen angle of the image's +u axis, CCW-positive (screen Y points down).
    expect_ccw = np.degrees(np.arctan2(-(a1[1] - a0[1]), a1[0] - a0[0]))

    from roomscan.sensors import wrap180
    assert wrap180(ir_gravity_angle_deg(q) - expect_ccw) == pytest.approx(0.0, abs=0.05)


@pytest.mark.parametrize("roll", [15.0, 30.0, 45.0, 90.0, 120.0, -30.0, -90.0])
def test_applied_rotation_stabilises_rather_than_doubling(roll):
    """The user-visible failure mode, asserted directly: with the sign inverted
    the content does not hold still, it counter-rotates at 2x the board rate.
    Total applied (rot90 turns + CSS residual) must cancel the roll, not double it.
    """
    from roomscan.sensors import ir_gravity_residual_deg, ir_gravity_rot, wrap180
    q = _rolled(roll)
    applied_ccw = 90.0 * ir_gravity_rot(q) + ir_gravity_residual_deg(q)
    # Rolling the board +roll turns the content -roll on screen, so a correct
    # correction is -roll; the inverted one would be +roll (a 2*roll error).
    assert wrap180(applied_ccw + roll) == pytest.approx(0.0, abs=0.05)
    assert abs(wrap180(applied_ccw - roll)) > 1.0 or abs(roll) % 180.0 < 1e-9


def test_ir_roll_rides_the_sensor_message_from_the_display_quat():
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *_Q_UPRIGHT)))
    # No display quat supplied (e.g. before the first smoothed sample) -> null,
    # and the client falls back to no residual rather than guessing.
    assert web.build_sensor_message(ss, None)["ir_roll_deg"] is None
    # Supplied -> the residual for THAT quat, not for the raw fused one, so it
    # agrees with the snap the IR pane was rendered with.
    msg = web.build_sensor_message(ss, None, ir_display_quat=_rolled(40.0))
    # Negative: a +40 deg board roll needs a 40 deg CLOCKWISE content correction.
    assert msg["ir_roll_deg"] == pytest.approx(-40.0, abs=0.01)
    json.dumps(msg)


def test_smoother_exposes_its_held_quat_for_the_ir_roll():
    sm = web.OrientationSmoother()
    assert sm.held is None                      # before any sample
    sm.update(_Q_UPRIGHT)
    assert sm.held == pytest.approx(_Q_UPRIGHT)


def test_rotation_key_quantizes_and_survives_json_free_comparison():
    assert web.rotation_key(None) is None
    r = web.display_rotation((0.92388, 0.38268, 0.0, 0.0))
    assert web.rotation_key(r) == web.rotation_key(r.copy())
    # Sub-milliradian noise must not invalidate the cached point-cloud bytes.
    assert web.rotation_key(r) == web.rotation_key(r + 1e-5)
    # Real motion must.
    assert web.rotation_key(r) != web.rotation_key(web.display_rotation((1.0, 0.0, 0.0, 0.0)))


def test_build_sensor_message_none_when_empty():
    # A ToF-only session (no 9/10 frames ever) must produce no sensor traffic.
    assert web.build_sensor_message(SensorState(), None) is None


def test_build_sensor_message_orientation_raw_null_without_quat():
    # Env-only (pressure/temp before the first stream-9 sample): orientation_raw
    # fields are all None, not missing/raising.
    ss = SensorState()
    ss.feed(_sframe(StreamId.ENV, struct.pack("<5f", 101000.0, 1.0, 2.0, 3.0, 22.0)))
    msg = web.build_sensor_message(ss, None)
    assert msg is not None
    assert msg["orientation_raw"] == {
        "quat": None, "roll_deg": None, "pitch_deg": None, "yaw_deg": None, "heading_deg": None,
    }
    json.dumps(msg)


def test_build_sensor_message_orientation_raw_full_precision():
    # rot/heading are rounded (5dp/1dp) for the wire; orientation_raw must NOT
    # be -- it exists precisely so sub-rounding-threshold changes are visible.
    from roomscan.sensors import quat_pitch_deg, quat_roll_deg, quat_yaw_deg

    ss = SensorState()
    q = (0.92387953, 0.38268343, 0.0001234, 0.0)   # deliberately not a "clean" 5dp value
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *q)))
    msg = web.build_sensor_message(ss, None)
    got = msg["orientation_raw"]
    assert got["quat"] == pytest.approx(list(q), abs=1e-6)
    assert got["roll_deg"] == pytest.approx(quat_roll_deg(q), abs=1e-4)
    assert got["pitch_deg"] == pytest.approx(quat_pitch_deg(q), abs=1e-4)
    assert got["yaw_deg"] == pytest.approx(quat_yaw_deg(q), abs=1e-4)
    assert got["heading_deg"] is None   # no env/mag yet
    json.dumps(msg)


def test_build_sensor_message_jitter_defaults_to_empty_when_no_tracker():
    # No jitter tracker passed (default None) -> every signal reports "not
    # enough samples yet", not zero and not a crash.
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    msg = web.build_sensor_message(ss, None)
    j = msg["jitter"]
    assert j["window_s"] == pytest.approx(web.SENSOR_JITTER_WINDOW_S)
    for sig in ("roll", "pitch", "yaw", "heading", "orientation"):
        assert j[sig] == {"mean_deg": None, "p95_deg": None, "n": 0}
    json.dumps(msg)


def test_build_sensor_message_wires_a_jitter_tracker():
    # With a real tracker, consecutive calls accumulate history and jitter
    # reflects the actual frame-to-frame change.
    ss = SensorState()
    jit = web.OrientationJitter()
    q0 = (1.0, 0.0, 0.0, 0.0)
    q1 = _perturb(q0, 1.0, axis=(0.0, 0.0, 1.0))   # 1 deg yaw step
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *q0)))
    web.build_sensor_message(ss, None, jit)
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *q1)))
    msg = web.build_sensor_message(ss, None, jit)
    j = msg["jitter"]
    assert j["yaw"]["n"] == 1
    assert j["yaw"]["mean_deg"] == pytest.approx(1.0, abs=0.05)
    assert j["yaw"]["p95_deg"] == pytest.approx(1.0, abs=0.05)
    assert j["orientation"]["n"] == 1
    assert j["orientation"]["mean_deg"] == pytest.approx(1.0, abs=0.05)
    json.dumps(msg)


# ---------------------------------------------------------------------------
# OrientationJitter -- rolling-window frame-to-frame noise stats (pure, no server).
# ---------------------------------------------------------------------------

def test_jitter_reports_none_before_two_samples():
    jit = web.OrientationJitter()
    out = jit.update(_Q_UPRIGHT, heading_deg=10.0, now=0.0)
    for sig in ("roll", "pitch", "yaw", "heading", "orientation"):
        assert out[sig] == {"mean_deg": None, "p95_deg": None, "n": 0}


def test_jitter_yaw_step_is_measured():
    # Identity baseline so a body-Z perturbation IS a pure Euler-yaw change
    # (with a non-trivial base quat like _Q_UPRIGHT, body Z isn't world Z).
    jit = web.OrientationJitter()
    q = (1.0, 0.0, 0.0, 0.0)
    jit.update(q, heading_deg=None, now=0.0)
    q = _perturb(q, 2.0, axis=(0.0, 0.0, 1.0))
    out = jit.update(q, heading_deg=None, now=0.1)
    assert out["yaw"]["n"] == 1
    assert out["yaw"]["mean_deg"] == pytest.approx(2.0, abs=0.05)
    assert out["yaw"]["p95_deg"] == pytest.approx(2.0, abs=0.05)


def test_jitter_heading_wraps_at_360():
    # 359 deg -> 1 deg is a 2 deg step through the wrap, not a 358 deg jump.
    jit = web.OrientationJitter()
    jit.update(None, heading_deg=359.0, now=0.0)
    out = jit.update(None, heading_deg=1.0, now=0.1)
    assert out["heading"]["n"] == 1
    assert out["heading"]["mean_deg"] == pytest.approx(2.0, abs=1e-6)


def test_jitter_orientation_uses_normalized_dot_product():
    # A near-unit but not-exactly-unit float32 quat pair with a genuinely
    # small angle between them must NOT report zero (the clip-to-1.0 bug).
    jit = web.OrientationJitter()
    q0 = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    tiny = math.radians(0.05) / 2.0
    q1 = np.array([math.cos(tiny), 0.0, 0.0, math.sin(tiny)], dtype=np.float32)
    # Perturb both off exact unit norm, as a real float32 stream would.
    q0 = q0 * np.float32(1.00003)
    q1 = q1 * np.float32(0.99997)
    jit.update(tuple(q0), heading_deg=None, now=0.0)
    out = jit.update(tuple(q1), heading_deg=None, now=0.1)
    assert out["orientation"]["n"] == 1
    assert out["orientation"]["mean_deg"] == pytest.approx(0.05, abs=0.01)
    assert out["orientation"]["mean_deg"] > 0.0   # NOT clipped to zero


def test_jitter_p95_and_mean_over_several_steps():
    jit = web.OrientationJitter()
    q = (1.0, 0.0, 0.0, 0.0)
    jit.update(q, heading_deg=None, now=0.0)
    steps = [0.1, 0.1, 0.1, 0.1, 1.0]   # one outlier -> p95 should catch it, mean should be pulled less
    t = 0.0
    for s in steps:
        t += 0.1
        q = _perturb(q, s, axis=(0.0, 0.0, 1.0))
        out = jit.update(q, heading_deg=None, now=t)
    assert out["yaw"]["n"] == 5
    assert out["yaw"]["mean_deg"] == pytest.approx(np.mean(steps), abs=0.05)
    assert out["yaw"]["p95_deg"] == pytest.approx(np.percentile(steps, 95), abs=0.05)
    assert out["yaw"]["p95_deg"] > out["yaw"]["mean_deg"]   # the outlier shows up in p95


def test_jitter_window_expires_old_samples():
    jit = web.OrientationJitter(window_s=1.0)
    q = (1.0, 0.0, 0.0, 0.0)
    jit.update(q, heading_deg=None, now=0.0)
    q = _perturb(q, 5.0, axis=(0.0, 0.0, 1.0))
    jit.update(q, heading_deg=None, now=0.1)   # one sample, inside the window
    q = _perturb(q, 0.2, axis=(0.0, 0.0, 1.0))
    out = jit.update(q, heading_deg=None, now=5.0)   # far past window_s=1.0
    # The 5 deg step from t=0.1 has aged out; only the fresh 0.2 deg step remains.
    assert out["yaw"]["n"] == 1
    assert out["yaw"]["mean_deg"] == pytest.approx(0.2, abs=0.05)


def test_build_sensor_message_rot_is_display_transform():
    ss = SensorState()
    q = (0.92388, 0.38268, 0.0, 0.0)   # ~45 deg about x
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *q)))
    msg = web.build_sensor_message(ss, None)
    assert msg is not None and msg["type"] == "sensor" and msg["have_quat"] is True
    # rot is exactly the gizmo_pose display rotation, row-major (sensors.py:183-192).
    expect = (T_WORLD_TO_CV @ quat_to_matrix(*q) @ T_CV_TO_BODY).reshape(-1)
    assert len(msg["rot"]) == 9
    assert np.allclose(np.array(msg["rot"], dtype=float), expect, atol=1e-4)
    # env-derived fields stay null with no ENV frame yet.
    assert msg["heading"] is None
    assert msg["pressure_pa"] is None and msg["temp_c"] is None and msg["mag_ut"] is None
    json.dumps(msg)   # fully JSON-serialisable


def test_build_sensor_message_env_fields_and_history():
    ss = SensorState()
    # A real grip (boresight horizontal), not the identity quat: identity aims
    # straight up, where no compass bearing exists and `heading` is correctly
    # None (BUG-058).
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *_Q_86_PITCH)))
    for i in range(3):
        ss.feed(_sframe(StreamId.ENV, struct.pack("<5f", 101000.0 + i, 1.0, 2.0, 3.0, 22.0 + i), t_us=1000 + i * 3_000_000))
    msg = web.build_sensor_message(ss, None)
    assert msg is not None
    assert msg["pressure_pa"] == pytest.approx(101002.0, abs=0.5)   # latest wins
    assert msg["temp_c"] == pytest.approx(24.0, abs=0.05)
    assert msg["mag_ut"] == [1.0, 2.0, 3.0]                          # raw (no mag_cal)
    assert msg["heading"] is not None                               # quat+env present
    assert len(msg["pressure_hist"]) == 3 and len(msg["temp_hist"]) == 3
    assert msg["pressure_hist"][0] == pytest.approx(101000.0, abs=0.5)
    json.dumps(msg)


# ---------------------------------------------------------------------------
# Orientation decomposition modes + labels + singularity warning (owner ask,
# 2026-07-28), and the world-referenced gravity+mag mode (owner follow-up).
# ---------------------------------------------------------------------------

_Q_86_PITCH = None  # set below: a quat at ~86 deg ZYX pitch, the live-rig scenario


def _zyx_pitch_quat(pitch_deg: float):
    """A quat with the given ZYX pitch and zero roll/yaw (pure rotation about
    the ZYX pitch axis), for exercising near-singularity behavior."""
    from roomscan.sensors import quat_pitch_deg as _qp
    a = math.radians(pitch_deg) / 2.0
    q = (math.cos(a), 0.0, math.sin(a), 0.0)
    assert _qp(q) == pytest.approx(pitch_deg, abs=1e-6)
    return q


_Q_86_PITCH = _zyx_pitch_quat(86.0)


def test_orientation_view_none_without_quat():
    v = web.orientation_view("zyx", None)
    assert v == {"roll_deg": None, "pitch_deg": None, "yaw_deg": None,
                 "singularity_margin_deg": None, "near_singularity": False,
                 "valid": False, "reason": "no orientation yet"}


def test_orientation_view_zyx_matches_existing_helpers():
    from roomscan.sensors import quat_pitch_deg, quat_roll_deg, quat_yaw_deg
    q = (0.92388, 0.38268, 0.0, 0.0)
    v = web.orientation_view("zyx", q)
    assert v["roll_deg"] == pytest.approx(quat_roll_deg(q))
    assert v["pitch_deg"] == pytest.approx(quat_pitch_deg(q))
    assert v["yaw_deg"] == pytest.approx(quat_yaw_deg(q))


def test_orientation_view_zyx_fires_near_singularity_at_86_deg_pitch():
    # The exact scenario that motivated this feature: rig at ~86 deg pitch.
    v = web.orientation_view("zyx", _Q_86_PITCH)
    assert v["singularity_margin_deg"] == pytest.approx(4.0, abs=1e-6)
    assert v["near_singularity"] is True


def test_orientation_view_zyx_not_near_singularity_at_45_deg():
    q = _zyx_pitch_quat(45.0)
    v = web.orientation_view("zyx", q)
    assert v["near_singularity"] is False


def test_orientation_view_zxy_disagrees_with_zyx_singularity():
    # At the ZYX mode's singularity, the ZXY mode must be clear -- that's the
    # entire point of offering it as an alternative.
    v_zyx = web.orientation_view("zyx", _Q_86_PITCH)
    v_zxy = web.orientation_view("zxy", _Q_86_PITCH)
    assert v_zyx["near_singularity"] is True
    assert v_zxy["near_singularity"] is False


def test_orientation_view_boresight_mode():
    az, el, roll = boresight_view_deg(_Q_86_PITCH)
    v = web.orientation_view("boresight", _Q_86_PITCH)
    assert v["yaw_deg"] == pytest.approx(az)
    assert v["pitch_deg"] == pytest.approx(el)
    assert v["roll_deg"] == pytest.approx(roll)
    assert v["near_singularity"] == (90.0 - abs(el) < web.ORIENTATION_SINGULARITY_MARGIN_DEG)


def test_orientation_view_world_mode_falls_back_to_quat_gravity_without_imu_raw():
    q = (1.0, 0.0, 0.0, 0.0)
    v = web.orientation_view("world", q, mag_ut_raw=None, heading_full=None,
                              mag_cal=None, imu_raw_batch=None)
    assert v["gravity_source"] == "quat"
    assert v["pitch_deg"] == pytest.approx(90.0, abs=1e-4)   # tilt, matches boresight elevation at identity
    assert v["valid"] is False   # no mag_cal at all
    assert v["reason"] == "no magnetometer calibration"


def test_orientation_view_world_roll_is_near_zero_in_the_normal_grip():
    """World's Roll slot must read ~0 deg when the instrument is held normally,
    not sit on the +-180 wrap (BUG-051).

    Body +X is the instrument's BOTTOM (`web._DEVICE_TOP_BODY`, and
    `devicemodel.js`'s MOUNT_ROTATION for the same reason), so referencing
    `triad_roll_deg` to its default body +X put the operating pose at ~178 deg:
    a few degrees of real roll swung the readout across the branch cut, +178 to
    -178, and read like a fault. The quat below is the owner's, read off /ws in
    the normal handheld grip. Reintroducing the default reference fails this.
    """
    grip = (0.604421, 0.35965, 0.593567, -0.391159)
    v = web.orientation_view("world", grip)
    assert abs(v["roll_deg"]) < 15.0, (
        f"World roll {v['roll_deg']:.2f} deg -- upright grip should be near 0, "
        "not near the +-180 wrap")
    assert v["pitch_deg"] == pytest.approx(2.1, abs=0.5)   # tilt: aimed level


def test_orientation_view_world_mode_prefers_imu_raw_gravity():
    from roomscan.protocol import ImuRawBatch
    batch = ImuRawBatch(
        gyro_dps=np.zeros((0, 3)), gyro_cnt=np.zeros(0, dtype=np.uint8),
        accel_g=np.zeros((0, 3)), accel_cnt=np.zeros(0, dtype=np.uint8),
        gravity_g=np.array([[1.0, 0.0, 0.0]]), gravity_cnt=np.zeros(1, dtype=np.uint8),
        gbias_dps=np.zeros((0, 3)), gbias_cnt=np.zeros(0, dtype=np.uint8),
        timestamp_ticks=np.zeros(0, dtype=np.uint32), timestamp_cnt=np.zeros(0, dtype=np.uint8),
        n_records=1)
    q = (1.0, 0.0, 0.0, 0.0)
    v = web.orientation_view("world", q, imu_raw_batch=batch)
    assert v["gravity_source"] == "imu_raw"


def test_orientation_view_world_mode_mag_anomaly_flags_invalid():
    from roomscan.magcal import MagCalibration
    cal = MagCalibration(offset=(0.0, 0.0, 0.0),
                         matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                         field_ut=49.87)
    q = (1.0, 0.0, 0.0, 0.0)
    # Raw mag magnitude ~107 uT against a 49.87 uT fit -- the live-rig anomaly
    # from the owner's report (~2.15x off).
    v = web.orientation_view("world", q, mag_ut_raw=(107.0, 0.0, 0.0), heading_full=12.3,
                              mag_cal=cal, imu_raw_batch=None)
    assert v["valid"] is False
    assert v["mag_norm_ut"] == pytest.approx(107.0, abs=0.5)
    assert v["mag_expected_ut"] == pytest.approx(49.87)
    assert "mag field" in v["reason"]
    assert v["yaw_deg"] == pytest.approx(12.3)   # heading is still reported, just flagged invalid


def test_orientation_view_world_mode_valid_within_anomaly_frac():
    from roomscan.magcal import MagCalibration
    cal = MagCalibration(offset=(0.0, 0.0, 0.0),
                         matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                         field_ut=50.0)
    q = (1.0, 0.0, 0.0, 0.0)
    v = web.orientation_view("world", q, mag_ut_raw=(50.0, 0.0, 0.0), heading_full=0.0, mag_cal=cal)
    assert v["valid"] is True
    assert v["reason"] is None


def test_orientation_view_world_mode_motion_flag_from_imu_raw_accel():
    from roomscan.magcal import MagCalibration
    from roomscan.protocol import ImuRawBatch
    cal = MagCalibration(offset=(0.0, 0.0, 0.0),
                         matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                         field_ut=50.0)
    moving = ImuRawBatch(
        gyro_dps=np.zeros((0, 3)), gyro_cnt=np.zeros(0, dtype=np.uint8),
        accel_g=np.array([[1.5, 0.0, 0.0]]), accel_cnt=np.zeros(1, dtype=np.uint8),
        gravity_g=np.zeros((0, 3)), gravity_cnt=np.zeros(0, dtype=np.uint8),
        gbias_dps=np.zeros((0, 3)), gbias_cnt=np.zeros(0, dtype=np.uint8),
        timestamp_ticks=np.zeros(0, dtype=np.uint32), timestamp_cnt=np.zeros(0, dtype=np.uint8),
        n_records=1)
    q = (1.0, 0.0, 0.0, 0.0)
    v = web.orientation_view("world", q, mag_ut_raw=(50.0, 0.0, 0.0), heading_full=0.0,
                              mag_cal=cal, imu_raw_batch=moving)
    assert v["motion_stable"] is False
    assert v["valid"] is False
    assert "accelerating" in v["reason"]


def test_orientation_view_unknown_mode_falls_back_to_zyx():
    from roomscan.sensors import quat_pitch_deg
    v = web.orientation_view("bogus", _Q_86_PITCH)
    assert v["pitch_deg"] == pytest.approx(quat_pitch_deg(_Q_86_PITCH))


# --- axis label sanitization + persistence ----------------------------------

def test_sanitize_axis_labels_defaults_on_bad_input():
    assert web._sanitize_axis_labels(None) == web.DEFAULT_AXIS_LABELS
    assert web._sanitize_axis_labels(["", "  ", "Pan"]) == ("Roll", "Tilt", "Pan")
    assert web._sanitize_axis_labels(["Tilt", "Swing", "Twist"]) == ("Tilt", "Swing", "Twist")


def test_sanitize_axis_labels_truncates_long_strings():
    long = "x" * 100
    out = web._sanitize_axis_labels([long, "Pitch", "Yaw"])
    assert len(out[0]) == web._MAX_LABEL_LEN


def test_ui_from_config_maps_orientation_labels_but_pins_mode_to_world():
    """The Sensors card's decomposition picker is gone (owner ask, 2026-07-31:
    the owner only ever used World) -- ui_from_config coerces ANY stored
    orientation_mode to "world", even a valid one, so a config written before
    this change can't strand a fresh boot in a mode with no picker to change
    it back from. Labels still map through untouched -- they're presentation
    only and independent of mode."""
    cfg = ViewerConfig(orientation_mode="boresight", orientation_labels="Tilt,Pan,Twist")
    ui = web.ui_from_config(cfg)
    assert ui.orientation_mode == "world"
    assert ui.orientation_labels == ("Tilt", "Pan", "Twist")


def test_ui_from_config_rejects_bad_orientation_mode():
    cfg = ViewerConfig(orientation_mode="nonsense")
    ui = web.ui_from_config(cfg)
    assert ui.orientation_mode == "world"


def test_ui_from_config_migrates_the_legacy_axis_labels():
    """A stored copy of the OLD default is treated as un-customised.

    `_persist_ui` writes every field on any UI change, so an install that never
    touched its axis labels still has "Roll,Pitch,Yaw" on disk -- which would
    shadow the new default forever. That is exactly backwards: those installs are
    the ones showing "Pitch"/"Yaw" against World's boresight tilt and absolute
    magnetic heading, i.e. the wrong names for the numbers beside them. This was
    found on the owner's own config, where the relabelling was invisible after a
    restart.
    """
    cfg = ViewerConfig(orientation_labels="Roll,Pitch,Yaw")
    assert web.ui_from_config(cfg).orientation_labels == ("Roll", "Tilt", "Heading")


def test_ui_from_config_keeps_genuinely_custom_axis_labels():
    """The migration is exact-match only -- anything the user actually chose is
    left alone, including a set that merely overlaps the old default."""
    for stored, expected in [
        ("Roll,Pitch,Twist", ("Roll", "Pitch", "Twist")),   # two of three match
        ("Tilt,Pan,Twist", ("Tilt", "Pan", "Twist")),
        ("Roll,Tilt,Heading", ("Roll", "Tilt", "Heading")),  # already migrated
    ]:
        assert web.ui_from_config(ViewerConfig(orientation_labels=stored)).orientation_labels == expected


def test_apply_ui_to_config_orientation_round_trips():
    cfg = ViewerConfig()
    ui = web.UiState(orientation_mode="world", orientation_labels=("A", "B", "C"))
    web.apply_ui_to_config(ui, cfg)
    assert cfg.orientation_mode == "world"
    assert cfg.orientation_labels == "A,B,C"
    back = web.ui_from_config(cfg)
    assert back.orientation_mode == "world"
    assert back.orientation_labels == ("A", "B", "C")


def test_set_orientation_handler_persists(tmp_path):
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    cfg = ViewerConfig()
    state = types.SimpleNamespace(config=cfg, ui_state=web.UiState(),
                                  clients=set(), controller=None)
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(state, {"type": "set_orientation",
                                                "mode": "zxy", "labels": ["A", "B", "C"]}))
    finally:
        config_mod.config_path = orig
    assert state.ui_state.orientation_mode == "zxy"
    assert state.ui_state.orientation_labels == ("A", "B", "C")
    loaded = ViewerConfig.load(p)
    assert loaded.orientation_mode == "zxy"
    assert loaded.orientation_labels == "A,B,C"


def test_set_orientation_handler_rejects_bad_mode():
    import types
    ui = web.UiState()
    state = types.SimpleNamespace(config=None, ui_state=ui, clients=set(), controller=None)
    asyncio.run(web._handle_inbound(state, {"type": "set_orientation", "mode": "bogus"}))
    assert ui.orientation_mode == "zyx"   # unchanged


# --- jitter follows the selected mode; orientation/heading stay independent ---

def test_jitter_roll_pitch_yaw_reset_on_mode_switch():
    jit = web.OrientationJitter()
    q0 = (1.0, 0.0, 0.0, 0.0)
    jit.update(q0, heading_deg=None, now=0.0, mode="zyx", roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)
    q1 = _perturb(q0, 2.0, axis=(1.0, 0.0, 0.0))
    out = jit.update(q1, heading_deg=None, now=0.1, mode="zyx", roll_deg=2.0, pitch_deg=0.0, yaw_deg=0.0)
    assert out["roll"]["n"] == 1

    # Switch mode with a WILDLY different roll number for the "same" instant
    # -- if not reset, this would register as a giant bogus jitter spike.
    q2 = _perturb(q1, 0.1, axis=(1.0, 0.0, 0.0))
    out = jit.update(q2, heading_deg=None, now=0.2, mode="boresight",
                      roll_deg=170.0, pitch_deg=5.0, yaw_deg=88.0)
    assert out["roll"]["n"] == 0   # reset: no prior sample under the new mode yet
    assert out["orientation"]["n"] == 2   # quat-angle jitter is convention-independent, unaffected


def test_jitter_orientation_and_heading_survive_mode_switch():
    jit = web.OrientationJitter()
    q0 = (1.0, 0.0, 0.0, 0.0)
    jit.update(q0, heading_deg=10.0, now=0.0, mode="zyx", roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)
    q1 = _perturb(q0, 1.0, axis=(0.0, 0.0, 1.0))
    jit.update(q1, heading_deg=11.0, now=0.1, mode="world", roll_deg=99.0, pitch_deg=1.0, yaw_deg=11.0)
    out = jit.update(q1, heading_deg=12.0, now=0.2, mode="world", roll_deg=99.0, pitch_deg=1.0, yaw_deg=11.0)
    assert out["heading"]["n"] == 2   # never reset by the mode switch
    assert out["orientation"]["n"] == 2


def test_build_sensor_message_includes_orientation_view_and_labels():
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *_Q_86_PITCH)))
    msg = web.build_sensor_message(ss, None, orientation_mode="zxy",
                                    axis_labels=("Tilt", "Swing", "Twist"))
    ov = msg["orientation_view"]
    assert ov["mode"] == "zxy"
    assert ov["labels"] == ["Tilt", "Swing", "Twist"]
    assert ov["roll_deg"] is not None


def test_build_sensor_message_orientation_view_null_without_quat():
    ss = SensorState()
    msg = web.build_sensor_message(ss, None)   # env-only path isn't reached; use ENV frame
    assert msg is None   # confirm baseline: truly empty state stays silent (existing contract)


def test_build_sensor_message_default_mode_matches_orientation_raw():
    # Default mode ("zyx") must report numbers identical to the always-present
    # orientation_raw fields -- "nothing changes unless a mode is chosen".
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *_Q_86_PITCH)))
    msg = web.build_sensor_message(ss, None)
    ov = msg["orientation_view"]
    raw = msg["orientation_raw"]
    assert ov["mode"] == "zyx"
    assert ov["roll_deg"] == pytest.approx(raw["roll_deg"], abs=1e-3)
    assert ov["pitch_deg"] == pytest.approx(raw["pitch_deg"], abs=1e-3)
    assert ov["yaw_deg"] == pytest.approx(raw["yaw_deg"], abs=1e-3)


def test_build_sensor_message_display_path_unaffected_by_orientation_mode():
    """The whole feature is presentation-only: `rot` (the display/point-cloud
    rotation) and `heading` must be byte-identical regardless of which
    orientation_view mode is selected."""
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *_Q_86_PITCH)))
    ss.feed(_sframe(StreamId.ENV, struct.pack("<5f", 101000.0, 20.0, -5.0, 40.0, 22.0)))
    baseline = web.build_sensor_message(ss, None, orientation_mode="zyx")
    for mode in ("zxy", "boresight", "world"):
        msg = web.build_sensor_message(ss, None, orientation_mode=mode)
        assert msg["rot"] == baseline["rot"]
        assert msg["heading"] == baseline["heading"]
        assert msg["orientation_raw"] == baseline["orientation_raw"]
        # And the display_rotation/fused_quat helpers themselves take no mode
        # argument at all -- there is no code path for a mode to reach them.
    assert "orientation_mode" not in web.display_rotation.__code__.co_varnames


# --- "Zero yaw here" (owner ask, 2026-07-29) --------------------------------

def _yaw_quat(pitch_deg: float, yaw_deg: float):
    """A quat at the given ZYX pitch with the given ZYX yaw grafted on (roll
    stays 0) -- built from the already-trusted `_zyx_pitch_quat` + `graft_yaw`
    primitives, exactly like `graft_yaw` itself composes a heading change."""
    from roomscan.sensors import graft_yaw
    return graft_yaw(_zyx_pitch_quat(pitch_deg), yaw_deg)


def test_build_sensor_message_display_path_unaffected_by_yaw_offset():
    """Mirrors the orientation_mode guard above: the offset is presentation-
    only and must not touch `rot`/`heading`/`orientation_raw` (the
    display/point-cloud/SLAM path), regardless of its value."""
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *_yaw_quat(20.0, 40.0))))
    ss.feed(_sframe(StreamId.ENV, struct.pack("<5f", 101000.0, 20.0, -5.0, 40.0, 22.0)))
    baseline = web.build_sensor_message(ss, None, yaw_offset_deg=0.0)
    for offset in (37.5, -120.0, 180.0):
        msg = web.build_sensor_message(ss, None, yaw_offset_deg=offset)
        assert msg["rot"] == baseline["rot"]
        assert msg["heading"] == baseline["heading"]
        assert msg["orientation_raw"] == baseline["orientation_raw"]
    assert "yaw_offset_deg" not in web.display_rotation.__code__.co_varnames
    assert "yaw_offset_deg" not in SensorState.fused_quat.__code__.co_varnames


@pytest.mark.parametrize("mode", ["zyx", "zxy", "boresight"])
def test_yaw_offset_zeroes_the_active_mode_at_capture(mode):
    """The core "Zero yaw here" contract: capture the current attitude's
    active-mode yaw via `_YAW_GRAFT_SIGN`, feed it back as `yaw_offset_deg`,
    and the SAME attitude must now report ~0 for that mode's yaw slot."""
    ss = SensorState()
    quat = _yaw_quat(25.0, 63.0)
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *quat)))
    raw = web.orientation_view(mode, quat)
    offset = web._YAW_GRAFT_SIGN[mode] * raw["yaw_deg"]
    msg = web.build_sensor_message(ss, None, orientation_mode=mode, yaw_offset_deg=offset)
    assert msg["orientation_view"]["yaw_deg"] == pytest.approx(0.0, abs=1e-3)
    # roll/pitch (tilt) must be untouched by the graft -- only yaw shifts.
    # wrap180 the roll diff: an angle that lands exactly on the +-180 seam
    # (as boresight's roll does for this fixture) is the same physical roll
    # whichever side of the seam it's reported on.
    assert web.wrap180(msg["orientation_view"]["roll_deg"] - raw["roll_deg"]) == pytest.approx(0.0, abs=1e-6)
    assert msg["orientation_view"]["pitch_deg"] == pytest.approx(raw["pitch_deg"], abs=1e-6)


def test_yaw_offset_not_applied_to_world_absolute_heading():
    """World mode's yaw slot is the absolute magnetic heading -- a nonzero
    offset must be a hard no-op there (both the applied value and the echoed
    `yaw_offset_deg` are forced to 0), unlike the relative modes."""
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *_yaw_quat(10.0, 50.0))))
    ss.feed(_sframe(StreamId.ENV, struct.pack("<5f", 101000.0, 20.0, -5.0, 40.0, 22.0)))
    baseline = web.build_sensor_message(ss, None, orientation_mode="world", yaw_offset_deg=0.0)
    offset_msg = web.build_sensor_message(ss, None, orientation_mode="world", yaw_offset_deg=77.0)
    assert offset_msg["orientation_view"]["yaw_deg"] == baseline["orientation_view"]["yaw_deg"]
    assert offset_msg["orientation_view"]["yaw_offset_deg"] == 0.0


def test_yaw_offset_does_not_change_jitter_magnitude():
    """A constant yaw offset must not change the jitter STATISTICS (mean/p95)
    -- it cancels exactly in any frame-to-frame diff. Feed the identical
    two-frame sequence through two independent trackers, one with an offset
    and one without, and require byte-identical yaw jitter."""
    q0 = _yaw_quat(15.0, 10.0)
    q1 = _yaw_quat(15.0, 10.6)   # a small yaw-only step

    def _feed_and_jitter(offset: float) -> dict:
        ss = SensorState()
        jit = web.OrientationJitter()
        ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *q0)))
        web.build_sensor_message(ss, None, jitter=jit, yaw_offset_deg=offset)
        ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *q1)))
        return web.build_sensor_message(ss, None, jitter=jit, yaw_offset_deg=offset)["jitter"]

    j_plain = _feed_and_jitter(0.0)
    j_offset = _feed_and_jitter(145.0)
    assert j_plain["yaw"]["n"] == 1 and j_offset["yaw"]["n"] == 1
    # A generous-looking but still tight tolerance: the offset is exact in
    # theory (it cancels algebraically in the diff) but going through
    # `graft_yaw`'s quat multiply + re-decomposition is floating-point, not
    # symbolic, so a ~1e-6 deg residual from a 145 deg graft is roundoff, not
    # a real magnitude change -- three orders of magnitude below the smallest
    # jitter this project measures (~0.01 deg/frame, see the orientation-
    # noise-floor work).
    assert j_offset["yaw"]["p95_deg"] == pytest.approx(j_plain["yaw"]["p95_deg"], abs=1e-4)
    assert j_offset["yaw"]["mean_deg"] == pytest.approx(j_plain["yaw"]["mean_deg"], abs=1e-4)


def test_ui_from_config_maps_yaw_offset():
    cfg = ViewerConfig(yaw_offset_deg=12.5)
    ui = web.ui_from_config(cfg)
    assert ui.yaw_offset_deg == pytest.approx(12.5)


def test_apply_ui_to_config_yaw_offset_round_trips():
    cfg = ViewerConfig()
    ui = web.UiState(yaw_offset_deg=-33.25)
    web.apply_ui_to_config(ui, cfg)
    assert cfg.yaw_offset_deg == pytest.approx(-33.25)
    back = web.ui_from_config(cfg)
    assert back.yaw_offset_deg == pytest.approx(-33.25)


def test_zero_yaw_handler_zeroes_zyx_and_persists(tmp_path):
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    cfg = ViewerConfig()
    quat = _yaw_quat(5.0, -43.0)
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *quat)))
    state = types.SimpleNamespace(config=cfg, ui_state=web.UiState(), clients=set(),
                                   controller=None, sensor_state=ss)
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(state, {"type": "zero_yaw"}))
    finally:
        config_mod.config_path = orig
    assert state.ui_state.yaw_offset_deg != 0.0
    msg = web.build_sensor_message(ss, None, orientation_mode="zyx",
                                    yaw_offset_deg=state.ui_state.yaw_offset_deg)
    assert msg["orientation_view"]["yaw_deg"] == pytest.approx(0.0, abs=1e-3)
    loaded = ViewerConfig.load(p)
    assert loaded.yaw_offset_deg == pytest.approx(state.ui_state.yaw_offset_deg)


def test_zero_yaw_handler_noop_in_world_mode():
    import types
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", *_yaw_quat(5.0, -43.0))))
    ui = web.UiState(orientation_mode="world")
    state = types.SimpleNamespace(config=None, ui_state=ui, clients=set(),
                                   controller=None, sensor_state=ss)
    asyncio.run(web._handle_inbound(state, {"type": "zero_yaw"}))
    assert ui.yaw_offset_deg == 0.0


def test_zero_yaw_handler_noop_without_orientation():
    import types
    ui = web.UiState(orientation_mode="zyx")
    state = types.SimpleNamespace(config=None, ui_state=ui, clients=set(),
                                   controller=None, sensor_state=SensorState())
    asyncio.run(web._handle_inbound(state, {"type": "zero_yaw"}))
    assert ui.yaw_offset_deg == 0.0


def test_clear_yaw_offset_handler_resets_and_persists(tmp_path):
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    cfg = ViewerConfig()
    state = types.SimpleNamespace(config=cfg, ui_state=web.UiState(yaw_offset_deg=99.0),
                                   clients=set(), controller=None, sensor_state=SensorState())
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(state, {"type": "clear_yaw_offset"}))
    finally:
        config_mod.config_path = orig
    assert state.ui_state.yaw_offset_deg == 0.0
    loaded = ViewerConfig.load(p)
    assert loaded.yaw_offset_deg == pytest.approx(0.0)


def test_state_message_carries_yaw_offset():
    m = web._state_message(web.UiState(yaw_offset_deg=8.0))
    assert m["yaw_offset_deg"] == 8.0


def _make_sensor_capture(path: Path, n: int = 6) -> None:
    """A capture of interleaved IMU_QUAT + ENV frames (no DEPTH), so the reader
    drives SensorState without ever filling the render slot."""
    out = bytearray()
    for i in range(n):
        q = struct.pack("<4f", 1.0, 0.01 * i, 0.0, 0.0)
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, i + 1,
                                      i * 35000, 0, 0, len(q)), q)
        env = struct.pack("<5f", 101000.0 + i, 20.0, -5.0, 40.0, 22.0 + 0.1 * i)
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.ENV, 0, i + 1,
                                      i * 35000, 0, 0, len(env)), env)
    path.write_bytes(bytes(out))


def test_sensor_state_populated_via_run_reader(tmp_path):
    import queue

    from roomscan.decoder import StreamDecoder

    cap = tmp_path / "sensors.bin"
    _make_sensor_capture(cap, n=6)

    ss = SensorState()
    stop = {"v": False}
    thread = threading.Thread(
        target=panel._run_reader,
        args=(FileSource(str(cap)), StreamDecoder(), TransformStage(outputs=("depth",)),
              Stats(), queue.Queue(maxsize=1), {}, LogBus(), None, None,
              panel._Pacer(interval=0.0), lambda: stop["v"]),
        kwargs={"state": ss},
        daemon=True,
    )
    thread.start()

    deadline = time.time() + 5.0
    while time.time() < deadline and (ss.latest_quat() is None or ss.pressure_history().size < 3):
        time.sleep(0.02)
    stop["v"] = True
    thread.join(timeout=5.0)

    # streams 9 + 10 both reached the SensorState through the shared reader.
    assert ss.latest_quat() is not None
    env = ss.latest_env()
    assert env is not None and env.temp_c == pytest.approx(22.0, abs=0.6)
    assert ss.pressure_history().size >= 3


def _make_depth_capture(path: Path, n_frames: int = 10, w: int = 8, h: int = 6) -> None:
    """Write a tiny DEPTH_ZF32 capture with n_frames DISTINCT frames.

    DEPTH_ZF32 passthrough needs no calib and no native DLL (TransformStage
    handles it directly), so this is a hermetic feed for the broadcaster.
    """
    out = bytearray()
    for i in range(n_frames):
        # Distinct, all-valid depth (mm): base shifts per frame so every frame's
        # packed point cloud differs.
        depth = (1000.0 + 50.0 * i + 100.0 * np.arange(w * h, dtype=np.float32)).reshape(h, w)
        payload = depth.astype("<f4").tobytes()
        header = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, i + 1,
                             i * 35000, w, h, len(payload))
        out += pack_frame(header, payload)
    path.write_bytes(bytes(out))


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _build_app_state(replay_path: Path, replay_fps: float = 20.0):
    """Mirror web.main()'s app.state setup against a FileSource replay, but WITH
    the pacer initially PAUSED so no frames flow until both clients connect --
    this makes the two clients' received frame sequences directly comparable
    regardless of connect-timing skew. Returns the pacer so the caller releases
    it after both clients are up."""
    import argparse

    source = FileSource(str(replay_path))
    from roomscan.decoder import StreamDecoder
    decoder = StreamDecoder()
    stats = Stats()
    bus = LogBus()
    metrics = MetricsRegistry(window_s=2.0)
    dispatcher = CommandDispatcher(None, on_message=bus.publish)
    stage = TransformStage(outputs=("depth", "reflectance", "confidence"))
    import queue
    slot: queue.Queue = queue.Queue(maxsize=1)
    fault: dict = {}
    pacer = panel._Pacer(interval=1.0 / replay_fps)
    pacer.paused.set()   # hold the reader before it publishes the first frame

    args = argparse.Namespace(fov_h=55.0, fov_v=42.0, replay=str(replay_path),
                              replay_fps=replay_fps)

    web.app.state.args = args
    web.app.state.source = source
    web.app.state.client = None
    web.app.state.stage = stage
    web.app.state.slot = slot
    web.app.state.bus = bus
    web.app.state.metrics = metrics
    web.app.state.dispatcher = dispatcher
    web.app.state.fault = fault
    web.app.state.fault_reported = False
    web.app.state.stats = stats
    web.app.state.pacer = pacer
    web.app.state.ui_state = web.UiState()
    sensor_state = SensorState()
    web.app.state.sensor_state = sensor_state
    web.app.state.mag_cal = None
    web.app.state.deproj = None
    web.app.state.orientation_smoother = web.OrientationSmoother()
    web.app.state.clients = set()
    web.app.state.command_labels = set()
    web.app.state.debounce = {}
    web.app.state.ready = True

    threading.Thread(
        target=panel._run_reader,
        args=(source, decoder, stage, stats, slot, fault, bus, None, None,
              pacer, lambda: False),
        kwargs={"state": sensor_state, "metrics": metrics},
        daemon=True,
    ).start()
    return pacer


def _point_clouds(messages):
    """Filter a list of received ws messages to POINT_CLOUD binary payloads."""
    out = []
    for m in messages:
        if isinstance(m, (bytes, bytearray)) and len(m) >= 4:
            (tag,) = struct.unpack_from("<I", m, 0)
            if tag == web.TAG_POINT_CLOUD:
                out.append(bytes(m))
    return out


def test_broadcaster_fanout_two_clients_same_frames(tmp_path):
    import uvicorn
    import websockets

    cap = tmp_path / "depth.bin"
    _make_depth_capture(cap, n_frames=10)
    pacer = _build_app_state(cap, replay_fps=20.0)

    port = _free_port()
    config = uvicorn.Config(web.app, host="127.0.0.1", port=port, log_level="warning")
    server = uvicorn.Server(config)

    def run_server():
        asyncio.run(server.serve())

    thread = threading.Thread(target=run_server, daemon=True)
    thread.start()

    # Wait for the server (and thus the startup broadcaster) to come up.
    deadline = time.time() + 10.0
    while not server.started and time.time() < deadline:
        time.sleep(0.02)
    assert server.started, "uvicorn server did not start"

    uri = f"ws://127.0.0.1:{port}/ws"
    N = 8   # point-cloud messages to collect from each client

    async def collect(ws, n):
        got = []
        while len(_point_clouds(got)) < n:
            m = await asyncio.wait_for(ws.recv(), timeout=8.0)
            got.append(m)
        return _point_clouds(got)[:n]

    async def run_clients():
        async with websockets.connect(uri) as ws1, websockets.connect(uri) as ws2:
            # Both connected: release the paced replay so frames start flowing
            # to a fully-populated client set.
            pacer.paused.clear()
            a = await collect(ws1, N)
            b = await collect(ws2, N)
            return a, b

    try:
        a, b = asyncio.run(asyncio.wait_for(run_clients(), timeout=20.0))
    finally:
        server.should_exit = True
        thread.join(timeout=10.0)

    # Fan-out: BOTH clients received the SAME ordered point-cloud stream
    # (single broadcast task, no frame-stealing between the two tabs).
    assert a == b
    # And it was not a single frozen frame -- multiple distinct frames flowed,
    # so an interleaving/stealing bug would have split them across clients.
    assert len(set(a)) >= 2


def test_metrics_broadcast_reports_reader_drops_and_gaps(tmp_path):
    """The `metrics` message must carry the READER's drops/gaps counters.

    MetricsRegistry has no notion of frame sequencing, so `MetricsSnapshot`
    leaves drops/gaps at 0 and the broadcaster has to merge in the reader's
    `Stats`. The web path never did, so the HUD's Drops/Gaps rows were pinned
    at 0 no matter what the link did -- and since a lost UDP fragment makes the
    host discard the whole frame, a seq gap is the ONLY evidence of transport
    loss the UI ever gets. Reintroducing the bug (dropping the `replace(...)`
    in the broadcaster) makes this fail with 0 != 7.
    """
    import uvicorn
    import websockets

    cap = tmp_path / "depth.bin"
    _make_depth_capture(cap, n_frames=10)
    pacer = _build_app_state(cap, replay_fps=20.0)

    # Stand in for a lossy link: the reader thread owns these counters at
    # runtime, so seed them directly rather than trying to corrupt a capture.
    web.app.state.stats.seq_gaps = 7
    web.app.state.stats.dropped_flags = 3

    port = _free_port()
    config = uvicorn.Config(web.app, host="127.0.0.1", port=port, log_level="warning")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=lambda: asyncio.run(server.serve()), daemon=True)
    thread.start()

    deadline = time.time() + 10.0
    while not server.started and time.time() < deadline:
        time.sleep(0.02)
    assert server.started, "uvicorn server did not start"

    async def first_metrics():
        async with websockets.connect(f"ws://127.0.0.1:{port}/ws") as ws:
            pacer.paused.clear()
            while True:
                m = await asyncio.wait_for(ws.recv(), timeout=8.0)
                if isinstance(m, str):
                    msg = json.loads(m)
                    if msg.get("type") == "metrics":
                        return msg

    try:
        msg = asyncio.run(asyncio.wait_for(first_metrics(), timeout=20.0))
    finally:
        server.should_exit = True
        thread.join(timeout=10.0)

    assert msg["gaps"] == 7
    assert msg["drops"] == 3


# =============================================================================
# 9. Recording & playback (Web Phase 3)
# =============================================================================
import os as _os

from roomscan.sources import Recorder
from roomscan.decoder import StreamDecoder as _StreamDecoder
from roomscan.metrics import MetricsRegistry as _MetricsRegistry
from roomscan.logbus import LogBus as _LogBus


def _make_depth_capture_flat(path: Path, n_frames: int, base: float,
                             w: int = 8, h: int = 6) -> None:
    """DEPTH_ZF32 capture whose frame i is a flat plane at `base + i` mm, so two
    captures with disjoint bases are distinguishable by depth.mean() from the
    render slot (no native DLL, passthrough)."""
    out = bytearray()
    for i in range(n_frames):
        depth = np.full((h, w), base + i, dtype=np.float32)
        payload = depth.astype("<f4").tobytes()
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, i + 1,
                                      i * 35000, w, h, len(payload)), payload)
    path.write_bytes(bytes(out))


def _make_raw_calib_capture(path: Path, n_raw: int = 5) -> tuple[bytes, int]:
    """One CALIB frame then n_raw RAW_3DMD frames (arbitrary payloads -- the index
    only scans headers/CRC, never runs the transform). Returns (calib_wire_bytes,
    byte_offset_of_first_raw)."""
    out = bytearray()
    calib_payload = bytes(range(200)) * 2
    calib_wire = pack_frame(FrameHeader(FrameType.DATA, StreamId.CALIB, 0, 0, 0, 0, 0,
                                        len(calib_payload)), calib_payload)
    out += calib_wire
    first_raw_off = len(out)
    for i in range(n_raw):
        payload = struct.pack("<H", i) * 64
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.RAW_3DMD, 0, i + 1,
                                      i * 35000, 54, 42, len(payload)), payload)
    path.write_bytes(bytes(out))
    return calib_wire, first_raw_off


# ---- pure helpers ----

def test_speed_to_interval():
    assert web.speed_to_interval(0) == 0.0
    assert web.speed_to_interval(-5) == 0.0
    assert web.speed_to_interval(30) == pytest.approx(1.0 / 30.0)


def test_sanitize_capture_name(tmp_path):
    (tmp_path / "good.bin").write_bytes(b"x")
    assert web.sanitize_capture_name("good.bin", tmp_path) == tmp_path / "good.bin"
    assert web.sanitize_capture_name("missing.bin", tmp_path) is None
    assert web.sanitize_capture_name("good.txt", tmp_path) is None       # wrong suffix
    assert web.sanitize_capture_name("../good.bin", tmp_path) is None    # traversal
    assert web.sanitize_capture_name("sub/good.bin", tmp_path) is None   # separator
    assert web.sanitize_capture_name("", tmp_path) is None
    assert web.sanitize_capture_name(None, tmp_path) is None


def test_sanitize_new_capture_name(tmp_path):
    (tmp_path / "taken.bin").write_bytes(b"x")
    assert web.sanitize_new_capture_name("my take", tmp_path) == "my take.bin"
    assert web.sanitize_new_capture_name("my take.bin", tmp_path) == "my take.bin"
    assert web.sanitize_new_capture_name("  padded  ", tmp_path) == "padded.bin"
    assert web.sanitize_new_capture_name("taken.bin", tmp_path) is None      # collision
    assert web.sanitize_new_capture_name("", tmp_path) is None
    assert web.sanitize_new_capture_name("   ", tmp_path) is None
    assert web.sanitize_new_capture_name(None, tmp_path) is None
    assert web.sanitize_new_capture_name("../escape.bin", tmp_path) is None  # traversal
    assert web.sanitize_new_capture_name("sub/name.bin", tmp_path) is None   # separator


def test_list_captures_newest_first(tmp_path):
    (tmp_path / "a.bin").write_bytes(b"aa")
    (tmp_path / "b.bin").write_bytes(b"bbbb")
    _os.utime(tmp_path / "a.bin", (1000, 1000))
    _os.utime(tmp_path / "b.bin", (2000, 2000))
    (tmp_path / "notacapture.txt").write_bytes(b"z")
    items = web.list_captures(tmp_path)
    assert [it["name"] for it in items] == ["b.bin", "a.bin"]   # newest (b) first
    assert items[0]["bytes"] == 4
    assert web.list_captures(tmp_path / "does-not-exist") == []


def test_build_capture_index_depth_offsets_are_frame_boundaries(tmp_path):
    cap = tmp_path / "depth.bin"
    _make_depth_capture_flat(cap, n_frames=10, base=1000.0)
    idx = web.build_capture_index(cap)
    assert idx["n_frames"] == 10
    assert idx["calib_spans"] == []
    assert idx["seqs"] == list(range(1, 11))
    # Every offset is a real frame boundary: FileSource(start=off) + decoder
    # yields that exact frame first.
    data = cap.read_bytes()
    for k, off in enumerate(idx["offsets"]):
        dec = _StreamDecoder()
        frames = dec.feed(data[off:])
        assert frames and frames[0].header.seq == idx["seqs"][k]


def test_build_capture_index_raw_records_calib_span(tmp_path):
    cap = tmp_path / "raw.bin"
    calib_wire, first_raw_off = _make_raw_calib_capture(cap, n_raw=5)
    idx = web.build_capture_index(cap)
    assert idx["n_frames"] == 5                       # RAW frames only
    assert len(idx["calib_spans"]) == 1
    s, e = idx["calib_spans"][0]
    assert (s, e) == (0, len(calib_wire))
    assert idx["offsets"][0] == first_raw_off


def test_build_capture_index_rejects_false_magic(tmp_path):
    """A MAGIC sequence inside a payload must not be mistaken for a frame start
    (CRC check rejects it)."""
    from roomscan.protocol import MAGIC
    cap = tmp_path / "trap.bin"
    payload = MAGIC + b"\x00" * 60           # embed MAGIC in a DEPTH payload
    out = pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 0, 4, 4,
                                 len(payload)), payload)
    cap.write_bytes(out)
    idx = web.build_capture_index(cap)
    assert idx["n_frames"] == 1              # the embedded MAGIC did NOT split it


def test_prefix_source_yields_calib_then_file(tmp_path):
    """Scrub-seek's calib re-injection: _PrefixSource emits the CALIB frame first,
    then the file from the seek offset, so the decoder sees CALIB before RAW."""
    cap = tmp_path / "raw.bin"
    calib_wire, first_raw_off = _make_raw_calib_capture(cap, n_raw=5)
    idx = web.build_capture_index(cap)
    seek_off = idx["offsets"][2]             # jump to the 3rd RAW frame
    src = web._PrefixSource(calib_wire, FileSource(str(cap), start=seek_off))
    dec = _StreamDecoder()
    seen = []
    for _ in range(20):
        data = src.read()
        if not data:
            break
        seen.extend(dec.feed(data))
    src.close()
    assert seen[0].header.stream_id == StreamId.CALIB          # calib first
    assert seen[1].header.stream_id == StreamId.RAW_3DMD
    assert seen[1].header.seq == idx["seqs"][2]                # resumed at the seek


def test_filesource_start_offset_reads_from_boundary(tmp_path):
    cap = tmp_path / "depth.bin"
    _make_depth_capture_flat(cap, n_frames=8, base=2000.0)
    idx = web.build_capture_index(cap)
    fs = FileSource(str(cap), start=idx["offsets"][5])
    dec = _StreamDecoder()
    frames = dec.feed(fs.read())
    fs.close()
    assert frames[0].header.seq == idx["seqs"][5]


def test_build_session_message_shape():
    m = web.build_session_message(
        "replay", "Replay · x.bin", False, rec_active=False, rec_path=None,
        rec_elapsed_s=0.0, rec_bytes=0, is_replay=True, capture_name="x.bin",
        paused=True, speed_fps=30.0, loop=True, position=0.5, total_frames=42,
        rec_last_name="web_20260101_000000.bin")
    assert m["type"] == "session" and m["mode"] == "replay"
    assert m["recording"]["last_name"] == "web_20260101_000000.bin"
    assert m["playback"]["is_replay"] and m["playback"]["position"] == 0.5
    assert m["playback"]["total_frames"] == 42 and m["playback"]["loop"] is True
    assert json.loads(json.dumps(m)) == m                     # JSON-round-trips


# ---- SessionController ----

def _make_controller(tmp_path, *, live_source=None, live_label="test",
                     replay_path=None, captures_dir=None, speed_fps=web._SPEED_BASE_FPS):
    stage = TransformStage(outputs=("depth", "reflectance", "confidence"))
    import queue
    slot = queue.Queue(maxsize=1)
    return web.SessionController(
        live_source=live_source, live_label=live_label, stage=stage, stats=Stats(),
        slot=slot, fault={}, bus=_LogBus(), client=None, recorder=Recorder(),
        pacer=panel._Pacer(interval=web.speed_to_interval(speed_fps)),
        sensor_state=SensorState(), metrics=_MetricsRegistry(window_s=2.0),
        captures_dir=str(captures_dir or tmp_path), initial_replay_path=replay_path,
        initial_speed_fps=speed_fps), slot


def _drain_depth_mean(slot, timeout):
    import queue
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            _, outputs = slot.get(timeout=0.1)
            return float(outputs["depth"].mean())
        except queue.Empty:
            continue
    return None


def _fake_imu_raw_batch():
    from roomscan.protocol import ImuRawBatch
    return ImuRawBatch(
        gyro_dps=np.zeros((0, 3)), gyro_cnt=np.zeros(0, dtype=np.uint8),
        accel_g=np.zeros((0, 3)), accel_cnt=np.zeros(0, dtype=np.uint8),
        gravity_g=np.array([[1.0, 0.0, 0.0]]), gravity_cnt=np.zeros(1, dtype=np.uint8),
        gbias_dps=np.zeros((0, 3)), gbias_cnt=np.zeros(0, dtype=np.uint8),
        timestamp_ticks=np.zeros(0, dtype=np.uint32), timestamp_cnt=np.zeros(0, dtype=np.uint8),
        n_records=1)


def test_switch_to_replay_clears_stale_imu_raw(tmp_path):
    # A source swap must not let the World orientation mode silently inherit
    # a gravity vector from whatever source was active before (owner-visible
    # bug found during browser verification, 2026-07-28: replaying a capture
    # with no stream 11 kept reporting the PRIOR live session's stale gravity
    # batch, producing a physically unrelated tilt reading).
    cap = tmp_path / "a.bin"
    _make_depth_capture_flat(cap, n_frames=4, base=1000.0)
    ctrl, _slot = _make_controller(tmp_path, replay_path=str(cap))
    ctrl.sensor_state._imu_raw = _fake_imu_raw_batch()
    assert ctrl.sensor_state.latest_imu_raw() is not None
    ctrl.switch_to_replay(str(cap))
    assert ctrl.sensor_state.latest_imu_raw() is None
    ctrl.close()


def test_switch_to_live_clears_stale_imu_raw(tmp_path):
    cap = tmp_path / "a.bin"
    _make_depth_capture_flat(cap, n_frames=4, base=1000.0)
    live = FileSource(str(cap))   # stand-in "live" source
    ctrl, _slot = _make_controller(tmp_path, live_source=live, replay_path=str(cap))
    ctrl.sensor_state._imu_raw = _fake_imu_raw_batch()
    ctrl.switch_to_live()
    assert ctrl.sensor_state.latest_imu_raw() is None
    ctrl.close()


def test_controller_switch_to_replay_changes_stream(tmp_path):
    capA = tmp_path / "a.bin"
    capB = tmp_path / "b.bin"
    _make_depth_capture_flat(capA, n_frames=40, base=1000.0)
    _make_depth_capture_flat(capB, n_frames=40, base=9000.0)

    ctrl, slot = _make_controller(tmp_path, replay_path=str(capA))
    ctrl.loop = True                          # keep A streaming until we swap
    ctrl.start()
    try:
        assert _drain_depth_mean(slot, 3.0) < 5000.0          # playing A
        ctrl.switch_to_replay(str(capB))
        import queue
        found_b = False
        deadline = time.time() + 4.0
        while time.time() < deadline:
            try:
                _, outputs = slot.get(timeout=0.1)
            except queue.Empty:
                continue
            if float(outputs["depth"].mean()) > 5000.0:
                found_b = True
                break
        assert found_b, "did not observe capB stream after switch_to_replay"
        assert ctrl.mode == "replay" and ctrl.index["n_frames"] == 40
    finally:
        ctrl.close()


def test_controller_record_gated_in_replay(tmp_path):
    cap = tmp_path / "a.bin"
    _make_depth_capture_flat(cap, n_frames=5, base=1000.0)
    ctrl, _ = _make_controller(tmp_path, replay_path=str(cap))
    ctrl.start_record()                       # replay mode -> refused
    assert not ctrl.recorder.active
    ctrl.close()


def test_controller_records_live_bytes(tmp_path):
    payload_cap = tmp_path / "src.bin"
    _make_depth_capture_flat(payload_cap, n_frames=3, base=1000.0)
    raw = payload_cap.read_bytes()

    class FakeLive:
        def read(self):
            time.sleep(0.02)
            return raw
        def write(self, d):
            pass
        def close(self):
            pass

    outdir = tmp_path / "caps"
    ctrl, _ = _make_controller(tmp_path, live_source=FakeLive(), captures_dir=outdir)
    ctrl.start()
    try:
        assert ctrl.mode == "live" and ctrl.has_live
        ctrl.start_record()
        assert ctrl.recorder.active
        # Let the reader tee at least one full chunk.
        deadline = time.time() + 2.0
        while time.time() < deadline:
            p = ctrl.recorder.path
            if p and _os.path.getsize(p) >= len(raw):
                break
            time.sleep(0.02)
        path = ctrl.recorder.path
        ctrl.stop_record()
        assert not ctrl.recorder.active
        rec = Path(path).read_bytes()
        assert len(rec) >= len(raw) and rec.startswith(raw)   # verbatim tee

        # Post-stop naming modal: rename the just-finished take.
        old_name = _os.path.basename(path)
        assert ctrl.session_message(None, time.time())["recording"]["last_name"] == old_name
        renamed = ctrl.rename_last_recording("my great scan")
        assert renamed == "my great scan.bin"
        assert not Path(path).exists()
        assert (Path(path).parent / "my great scan.bin").exists()
        assert ctrl.session_message(None, time.time())["recording"]["last_name"] == renamed

        # Renaming to a name that collides with an existing file is rejected,
        # leaving the current name in place.
        (Path(path).parent / "clash.bin").write_bytes(b"x")
        assert ctrl.rename_last_recording("clash") is None
        assert ctrl.session_message(None, time.time())["recording"]["last_name"] == renamed

        # Starting a new take clears the previous take's "just finished" name.
        ctrl.start_record()
        assert ctrl.session_message(None, time.time())["recording"]["last_name"] is None
        assert ctrl.rename_last_recording("too late") is None
        ctrl.stop_record()
    finally:
        ctrl.close()


def test_handle_inbound_rename_capture(tmp_path):
    import types

    (tmp_path / "web_20260101_000000.bin").write_bytes(b"x")
    ctrl, _slot = _make_controller(tmp_path, captures_dir=tmp_path)
    ctrl._last_recorded_name = "web_20260101_000000.bin"
    bus = _LogBus()
    handle = bus.subscribe()
    state = types.SimpleNamespace(controller=ctrl, clients=set(), bus=bus, ui_state=web.UiState())

    asyncio.run(web._handle_inbound(state, {"type": "rename_capture", "name": "renamed take"}))
    assert not (tmp_path / "web_20260101_000000.bin").exists()
    assert (tmp_path / "renamed take.bin").exists()
    assert ctrl._last_recorded_name == "renamed take.bin"
    assert bus.drain(handle) == []                            # no error published

    # Renaming again onto an existing name is rejected and reported on the bus.
    (tmp_path / "clash.bin").write_bytes(b"x")
    asyncio.run(web._handle_inbound(state, {"type": "rename_capture", "name": "clash"}))
    assert len(bus.drain(handle)) == 1
    assert ctrl._last_recorded_name == "renamed take.bin"   # unchanged
    ctrl.close()


def test_controller_session_message_live_vs_replay(tmp_path):
    cap = tmp_path / "a.bin"
    _make_depth_capture_flat(cap, n_frames=7, base=1000.0)

    ctrl_r, _ = _make_controller(tmp_path, replay_path=str(cap))
    m = ctrl_r.session_message(0.25, time.time())
    assert m["mode"] == "replay" and m["playback"]["is_replay"]
    assert m["playback"]["capture_name"] == "a.bin"
    assert m["playback"]["total_frames"] == 7
    assert m["has_live"] is False

    class FakeLive:
        def read(self): return b""
        def write(self, d): pass
        def close(self): pass

    ctrl_l, _ = _make_controller(tmp_path, live_source=FakeLive(), live_label="Serial CDC · COM7")
    m2 = ctrl_l.session_message(None, time.time())
    assert m2["mode"] == "live" and m2["has_live"] is True
    assert m2["playback"]["is_replay"] is False
    assert m2["source_label"] == "Serial CDC · COM7"


def test_recording_elapsed_is_measured_on_the_clock_that_started_it(tmp_path, monkeypatch):
    """BUG-050: `elapsed_s` was `time.time() - time.monotonic()`, i.e. the epoch.

    Every caller passed `time.time()` as `now`, but `_record_started` is a
    `time.monotonic()` stamp. The two clocks share no origin, so a 90-second take
    reported elapsed_s = 1784067285.5 -- and the UI happily rendered it, because
    nothing asserted the magnitude. The bug is invisible to a shape test (the key
    is present and is a float) which is exactly why it survived.
    """
    class FakeLive:
        def read(self): return b""
        def write(self, d): pass
        def close(self): pass

    ctrl, _ = _make_controller(tmp_path, live_source=FakeLive(), live_label="x")
    ctrl.start_record()
    try:
        # Advance the monotonic clock by a known amount; leave wall clock alone.
        base = time.monotonic()
        monkeypatch.setattr(web.time, "monotonic", lambda: base + 90.0)
        elapsed = ctrl.session_message(None, time.time())["recording"]["elapsed_s"]
    finally:
        ctrl.stop_record()
    assert 89.0 <= elapsed <= 91.0, f"elapsed_s was {elapsed!r}, expected ~90 s"


def test_controller_transport_speed_and_loop(tmp_path):
    cap = tmp_path / "a.bin"
    _make_depth_capture_flat(cap, n_frames=5, base=1000.0)
    ctrl, _ = _make_controller(tmp_path, replay_path=str(cap))
    ctrl.set_speed(15.0)
    assert ctrl.pacer.interval == pytest.approx(1.0 / 15.0)
    ctrl.set_speed(0.0)
    assert ctrl.pacer.interval == 0.0
    ctrl.set_loop(True)
    assert ctrl.loop is True
    ctrl.pause()
    assert ctrl.pacer.paused.is_set()
    ctrl.resume()
    assert not ctrl.pacer.paused.is_set()
    ctrl.close()


def test_controller_seek_sets_offset_and_resumes(tmp_path):
    cap = tmp_path / "d.bin"
    _make_depth_capture_flat(cap, n_frames=100, base=1000.0)
    ctrl, slot = _make_controller(tmp_path, replay_path=str(cap), speed_fps=20.0)
    ctrl.loop = True                          # keep streaming so a frame arrives post-seek
    ctrl.start()
    try:
        assert _drain_depth_mean(slot, 3.0) is not None    # producing
        ctrl.seek(0.5)
        idx = ctrl.index
        i = round(0.5 * (idx["n_frames"] - 1))
        assert ctrl._seek_offset == idx["offsets"][i]      # exact frame boundary
        assert ctrl._seek_prefix == b""                    # no calib in a DEPTH capture
        # A DEPTH capture reads correctly at the seek offset -> a frame still flows.
        assert _drain_depth_mean(slot, 3.0) is not None
    finally:
        ctrl.close()


# --- Web Phase 4: SLAM mode ---------------------------------------------------
# The protocol/plumbing is exercised with fake worker/meshprep (no Open3D/GPU);
# save uses a real tiny Open3D mesh so the write path is genuinely covered.
from roomscan.slam.meshprep import MeshPacket as _MeshPacket
from roomscan.slam.mapper import FrameStep as _FrameStep


def _synthetic_mesh_packet(*, mesh_seq=3, walls="split", decimated=False):
    nw_v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float64)
    nw_c = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float64)
    nw_t = np.array([[0, 1, 2]], np.int32)
    w_v = np.array([[0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]], np.float64)
    w_c = np.full((4, 3), 0.5)
    w_t = np.array([[0, 1, 2], [1, 2, 3]], np.int32)
    f_p = np.array([[0, 0, 0], [1, 0, 0], [0, 0, 1]], np.float64)
    f_l = np.array([[0, 1], [1, 2]], np.int64)
    return _MeshPacket(
        non_wall_verts=nw_v, non_wall_colors=nw_c, non_wall_tris=nw_t,
        wall_verts=w_v, wall_colors=w_c, wall_tris=w_t,
        floor_pts=f_p, floor_lines=f_l, mesh_seq=mesh_seq,
        source_vertex_count=7, decimated=decimated, wall_mode=walls)


def test_pack_mesh_roundtrip():
    pkt = _synthetic_mesh_packet(mesh_seq=5, walls="split", decimated=True)
    buf = web.pack_mesh(pkt)
    (tag, seq, flags, nnwv, nnwt, nwv, nwt, nfp, nfl) = struct.unpack_from("<IIIIIIIII", buf, 0)
    assert tag == web.TAG_MESH and seq == 5
    assert flags == (1 | 2)                    # decimated + walls_split
    assert (nnwv, nnwt) == (3, 1)
    assert (nwv, nwt) == (4, 2)
    assert (nfp, nfl) == (3, 2)
    off = 36                                   # 9 * u32
    nw_pos = np.frombuffer(buf, "<f4", 3 * nnwv, off); off += 4 * 3 * nnwv
    nw_col = np.frombuffer(buf, "<f4", 3 * nnwv, off); off += 4 * 3 * nnwv
    nw_idx = np.frombuffer(buf, "<u4", 3 * nnwt, off); off += 4 * 3 * nnwt
    np.testing.assert_allclose(nw_pos.reshape(-1, 3), pkt.non_wall_verts, atol=1e-6)
    np.testing.assert_allclose(nw_col.reshape(-1, 3), pkt.non_wall_colors, atol=1e-6)
    np.testing.assert_array_equal(nw_idx.reshape(-1, 3), pkt.non_wall_tris)
    # total size accounts for every declared array
    expect = 36 + 4 * (3*nnwv + 3*nnwv + 3*nnwt + 3*nwv + 3*nwv + 3*nwt + 3*nfp + 2*nfl)
    assert len(buf) == expect


def test_pack_mesh_empty_packet_is_header_only():
    empty = _MeshPacket(
        non_wall_verts=np.zeros((0, 3)), non_wall_colors=np.zeros((0, 3)),
        non_wall_tris=np.zeros((0, 3), np.int32), wall_verts=np.zeros((0, 3)),
        wall_colors=np.zeros((0, 3)), wall_tris=np.zeros((0, 3), np.int32),
        floor_pts=np.zeros((0, 3)), floor_lines=np.zeros((0, 2), np.int64),
        mesh_seq=1, source_vertex_count=0, decimated=False, wall_mode="solid")
    buf = web.pack_mesh(empty)
    assert len(buf) == 36                       # nothing but the 9-u32 header
    tag, seq, flags = struct.unpack_from("<III", buf, 0)
    assert tag == web.TAG_MESH and seq == 1 and flags == 0


def test_build_slam_message_shape_and_traj_bound():
    poses = [np.eye(4) for _ in range(1000)]
    for i, p in enumerate(poses):
        p[0, 3] = float(i)                      # x = frame index, so we can spot-check
    step = _FrameStep(pose=poses[-1], fitness=0.87, rmse=0.012,
                      tracking_lost=False, slam_ms=6.3)
    msg = web.build_slam_message(step, poses, frames_integrated=990,
                                 mesh_seq=4, source_vertex_count=51788)
    assert msg["type"] == "slam"
    assert len(msg["pose"]) == 16
    assert set(msg["follow"]) == {"eye", "center", "up"}
    assert len(msg["traj_tail"]) == web._TRAJ_TAIL_MAX      # downsampled, not 1000
    assert msg["traj_len"] == 1000
    assert msg["traj_tail"][0] == [0.0, 0.0, 0.0]
    assert msg["traj_tail"][-1][0] == 999.0                 # last real position kept
    assert msg["frames_integrated"] == 990 and msg["mesh_verts"] == 51788
    assert msg["tracking_lost"] is False


def test_state_message_carries_mode_and_slam_opts():
    ui = web.UiState()
    m = web._state_message(ui)
    assert m["mode"] == "realtime"
    assert m["slam_trajectory"] is True and m["slam_walls"] == "split" and m["slam_follow"] is True


def test_sanitize_result_name(tmp_path):
    (tmp_path / "web_x.ply").write_bytes(b"ply")
    (tmp_path / "web_x.tum").write_text("t")
    assert web.sanitize_result_name("web_x.ply", tmp_path) == tmp_path / "web_x.ply"
    assert web.sanitize_result_name("web_x.tum", tmp_path) == tmp_path / "web_x.tum"
    assert web.sanitize_result_name("../etc/passwd", tmp_path) is None      # traversal
    assert web.sanitize_result_name("web_x.exe", tmp_path) is None          # wrong ext
    assert web.sanitize_result_name("missing.ply", tmp_path) is None        # must exist
    assert web.sanitize_result_name("", tmp_path) is None


def test_list_results_newest_first(tmp_path):
    a = tmp_path / "a.ply"; a.write_bytes(b"1"); _os.utime(a, (1000, 1000))
    b = tmp_path / "b.ply"; b.write_bytes(b"22"); _os.utime(b, (2000, 2000))
    (tmp_path / "notes.txt").write_text("ignored")
    items = web.list_results(tmp_path)
    assert [it["name"] for it in items] == ["b.ply", "a.ply"]
    assert items[0]["bytes"] == 2


# ---- SlamRunner plumbing (fake worker/meshprep, no GPU) ----------------------

class _FakeWorker:
    def __init__(self, *a, **k):
        self.started = self.stopped = False
        self.submitted = []
        self.tracking_lost_count = 2
        self._traj = [np.eye(4) for _ in range(10)]
        self._mesh = object()               # opaque, identity-compared by SlamRunner
    def start(self): self.started = True
    def stop(self): self.stopped = True
    def submit(self, *a, **k): self.submitted.append((a, k))
    def latest(self):
        step = _FrameStep(pose=np.eye(4), fitness=0.5, rmse=0.02,
                          tracking_lost=False, slam_ms=5.0)
        return (self._mesh, self._traj, step)


class _FakeMeshPrep:
    def __init__(self, *a, **k): self.started = self.stopped = False; self.subs = []
    def start(self): self.started = True
    def stop(self): self.stopped = True
    def submit(self, mesh, *, mesh_seq, glow_origin, wall_mode):
        self.subs.append((mesh_seq, wall_mode))
    def latest(self): return _synthetic_mesh_packet(mesh_seq=len(self.subs))


@pytest.fixture
def _fake_slam(monkeypatch):
    import roomscan.slam.backend as backend
    import roomscan.slam.meshprep as meshprep
    made = {}
    def _mk(w, h, **k):
        made["worker"] = _FakeWorker(); made["wh"] = (w, h); return made["worker"]
    monkeypatch.setattr(backend, "make_slam_worker", _mk)
    monkeypatch.setattr(meshprep, "MeshPrep", lambda *a, **k: made.setdefault("mp", _FakeMeshPrep()))
    return made


def test_slamrunner_inactive_submit_builds_nothing(_fake_slam):
    r = web.SlamRunner(bus=LogBus())
    r.submit(np.zeros((6, 8), np.float32), (1, 0, 0, 0), None)
    assert "worker" not in _fake_slam        # no worker built while inactive


def test_slamrunner_no_quat_is_noop(_fake_slam):
    r = web.SlamRunner(bus=LogBus())
    r.set_active(True)
    r.submit(np.zeros((6, 8), np.float32), None, None)   # SLAM needs the orientation prior
    assert "worker" not in _fake_slam


def _await_build(runner, made, timeout=5.0):
    """BUG-060: `SlamRunner.submit` is now called from the READER thread, so it
    kicks the Open3D/CUDA construction onto its own thread and drops frames
    until that lands rather than stalling the reader (which would overflow the
    UDP socket). Tests that want a built pipeline have to wait for it."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with runner._lock:
            if runner._worker is not None:
                return
        time.sleep(0.005)
    raise AssertionError(f"SLAM worker was never built (made={list(made)})")


def test_slamrunner_lazy_build_and_poll(_fake_slam):
    r = web.SlamRunner(bus=LogBus())
    r.set_active(True)
    depth = np.zeros((6, 8), np.float32)
    r.submit(depth, (1, 0, 0, 0), 101325.0)              # kicks the async build
    _await_build(r, _fake_slam)
    assert _fake_slam["wh"] == (8, 6)                    # width, height from depth shape
    assert _fake_slam["worker"].started and _fake_slam["mp"].started
    r.submit(depth, (1, 0, 0, 0), 101325.0)
    assert len(_fake_slam["worker"].submitted) == 1
    msg, mesh_bytes = r.poll("split")
    assert msg is not None and msg["type"] == "slam"
    assert msg["frames_integrated"] == 10 - 2            # traj_len - tracking_lost_count
    assert mesh_bytes is not None
    tag, seq = struct.unpack_from("<II", mesh_bytes, 0)
    assert tag == web.TAG_MESH and seq == 1              # first new mesh -> seq 1


def test_slamrunner_set_active_false_tears_down(_fake_slam):
    r = web.SlamRunner(bus=LogBus())
    r.set_active(True)
    r.submit(np.zeros((6, 8), np.float32), (1, 0, 0, 0), None)
    _await_build(r, _fake_slam)
    w, mp = _fake_slam["worker"], _fake_slam["mp"]
    r.set_active(False)
    assert w.stopped and mp.stopped
    # poll after teardown is silent
    assert r.poll("split") == (None, None)


def test_slamrunner_save_writes_ply_and_tum(tmp_path, monkeypatch):
    import open3d as o3d
    # A real (tiny) tensor mesh so the save write path is genuinely exercised.
    tm = o3d.t.geometry.TriangleMesh()
    tm.vertex.positions = o3d.core.Tensor([[0, 0, 0], [1, 0, 0], [0, 1, 0]], o3d.core.float32)
    tm.triangle.indices = o3d.core.Tensor([[0, 1, 2]], o3d.core.int32)

    class _SaveWorker(_FakeWorker):
        def latest(self):
            step = _FrameStep(pose=np.eye(4), fitness=0.5, rmse=0.02,
                              tracking_lost=False, slam_ms=5.0)
            return (tm, [np.eye(4) for _ in range(4)], step)

    r = web.SlamRunner(bus=LogBus())
    with r._lock:
        r._worker = _SaveWorker()
    ply, tum = tmp_path / "m.ply", tmp_path / "m.tum"
    n = r.save(ply, tum)
    assert n == 3
    assert ply.is_file() and ply.stat().st_size > 0
    assert tum.is_file() and len(tum.read_text().splitlines()) == 4


def test_slamrunner_save_empty_map_raises(tmp_path):
    class _EmptyWorker(_FakeWorker):
        def latest(self): return None
    r = web.SlamRunner(bus=LogBus())
    with r._lock:
        r._worker = _EmptyWorker()
    with pytest.raises(ValueError):
        r.save(tmp_path / "m.ply", tmp_path / "m.tum")


# --- Web Phase 5: settings persistence -------------------------------------

from roomscan.config import ViewerConfig  # noqa: E402


def test_config_has_slam_display_fields_with_defaults():
    """The three web-owned SLAM display prefs were added to ViewerConfig with
    defaults matching the UiState defaults (so a fresh install agrees)."""
    cfg = ViewerConfig()
    ui = web.UiState()
    assert cfg.slam_trajectory is ui.slam_trajectory
    assert cfg.slam_walls == ui.slam_walls
    assert cfg.slam_follow is ui.slam_follow


def test_config_slam_fields_round_trip_toml(tmp_path):
    """The new fields survive the flat-TOML writer/reader round trip."""
    p = tmp_path / "roomscan.toml"
    ViewerConfig(slam_trajectory=False, slam_walls="solid", slam_follow=False).save(p)
    back = ViewerConfig.load(p)
    assert back.slam_trajectory is False
    assert back.slam_walls == "solid"
    assert back.slam_follow is False


def test_ui_from_config_maps_valid_fields():
    cfg = ViewerConfig(color="reflectance", ir_colormap="turbo",
                       ir_freeze_range=True, slam_trajectory=False,
                       slam_walls="solid", slam_follow=False)
    ui = web.ui_from_config(cfg)
    assert ui.color_mode == "reflectance"
    assert ui.ir_colormap == "turbo"
    assert ui.ir_freeze is True
    assert ui.slam_trajectory is False
    assert ui.slam_walls == "solid"
    assert ui.slam_follow is False


def test_ui_from_config_rejects_bad_values_falls_back_to_defaults():
    """A corrupt/unknown color or colormap or wall mode falls back to the
    UiState default rather than propagating a bad value into the running UI."""
    cfg = ViewerConfig(color="mauve", ir_colormap="plasma", slam_walls="wavy")
    ui = web.ui_from_config(cfg)
    default = web.UiState()
    assert ui.color_mode == default.color_mode
    assert ui.ir_colormap == default.ir_colormap
    assert ui.slam_walls == default.slam_walls


def test_ui_from_config_does_not_restore_mode():
    """`mode` is intentionally never restored -- SLAM arms lazily, so a restart
    always comes up in real-time even if the file says 'slam'."""
    cfg = ViewerConfig(mode="slam")
    assert web.ui_from_config(cfg).mode == "realtime"


def test_apply_ui_to_config_preserves_non_web_fields():
    """Writing web prefs back must leave every desktop-only field (fov, mode,
    yaw fusion, ...) untouched, so the two frontends share one file cleanly."""
    cfg = ViewerConfig(fov_h=60.0, mode="slam", yaw_fusion_tau=33.0, port="COM7")
    ui = web.UiState(color_mode="confidence", ir_colormap="turbo",
                     ir_freeze=True, slam_trajectory=False,
                     slam_walls="solid", slam_follow=False)
    web.apply_ui_to_config(ui, cfg)
    # web-owned fields updated
    assert cfg.color == "confidence"
    assert cfg.ir_colormap == "turbo"
    assert cfg.ir_freeze_range is True
    assert cfg.slam_trajectory is False and cfg.slam_walls == "solid" and cfg.slam_follow is False
    # desktop-only fields preserved verbatim
    assert cfg.fov_h == 60.0
    assert cfg.mode == "slam"
    assert cfg.yaw_fusion_tau == 33.0
    assert cfg.port == "COM7"


def test_persist_ui_writes_and_is_noop_without_config(tmp_path):
    import types
    p = tmp_path / "roomscan.toml"
    cfg = ViewerConfig()
    cfg.save(p)  # establishes the on-disk file at an explicit path

    # _persist_ui uses cfg.save() with no arg -> config_path(); redirect it here.
    ui = web.UiState(color_mode="confidence", slam_walls="solid")
    state = types.SimpleNamespace(config=cfg, ui_state=ui)
    # Patch the config module's path resolver so cfg.save() lands in tmp_path.
    import roomscan.config as config_mod
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        web._persist_ui(state)
    finally:
        config_mod.config_path = orig
    back = ViewerConfig.load(p)
    assert back.color == "confidence"
    assert back.slam_walls == "solid"

    # No config attached -> silently does nothing (the shape tests build).
    web._persist_ui(types.SimpleNamespace(config=None, ui_state=web.UiState()))


def test_set_color_handler_persists(tmp_path):
    """Driving the real inbound handler with a config attached writes the new
    color straight to roomscan.toml (end-to-end of the persistence path)."""
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    cfg = ViewerConfig()
    state = types.SimpleNamespace(config=cfg, ui_state=web.UiState(),
                                  clients=set(), controller=None)
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(state, {"type": "set_color", "mode": "confidence"}))
    finally:
        config_mod.config_path = orig
    assert state.ui_state.color_mode == "confidence"
    assert ViewerConfig.load(p).color == "confidence"


# --- auto point size (range-adaptive splats) --------------------------------

def test_config_point_size_auto_default_matches_uistate():
    """A fresh install must agree between file default and UiState default --
    auto is ON by default (owner decision)."""
    assert ViewerConfig().web_point_size_auto is web.UiState().point_size_auto is True


def test_config_point_size_auto_round_trips_toml(tmp_path):
    p = tmp_path / "roomscan.toml"
    ViewerConfig(web_point_size_auto=False, web_point_size=0.05).save(p)
    back = ViewerConfig.load(p)
    assert back.web_point_size_auto is False
    assert back.web_point_size == 0.05


def test_ui_from_config_maps_point_size_and_auto():
    ui = web.ui_from_config(ViewerConfig(web_point_size=0.08, web_point_size_auto=False))
    assert ui.point_size == 0.08
    assert ui.point_size_auto is False


def test_ui_from_config_rejects_out_of_range_point_size():
    """A corrupt/out-of-range size in the file must not reach the material --
    same 0.001..1.0 range `set_view` enforces on the wire."""
    default = web.UiState().point_size
    assert web.ui_from_config(ViewerConfig(web_point_size=0.0)).point_size == default
    assert web.ui_from_config(ViewerConfig(web_point_size=25.0)).point_size == default


def test_apply_ui_to_config_writes_point_size_auto():
    cfg = ViewerConfig()
    web.apply_ui_to_config(web.UiState(point_size=0.04, point_size_auto=False), cfg)
    assert cfg.web_point_size == 0.04
    assert cfg.web_point_size_auto is False


def test_state_message_carries_point_size_auto():
    m = web._state_message(web.UiState(point_size=0.03, point_size_auto=False))
    assert m["point_size"] == 0.03
    assert m["point_size_auto"] is False


def test_set_view_point_size_auto_updates_and_persists(tmp_path):
    """End-to-end through the real inbound handler: the toggle lands in UiState,
    in the `state` echo, and in roomscan.toml."""
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    state = types.SimpleNamespace(config=ViewerConfig(), ui_state=web.UiState(),
                                  clients=set(), controller=None)
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(
            state, {"type": "set_view", "point_size_auto": False, "point_size": 0.06}))
    finally:
        config_mod.config_path = orig
    assert state.ui_state.point_size_auto is False
    assert state.ui_state.point_size == 0.06
    assert web._state_message(state.ui_state)["point_size_auto"] is False
    back = ViewerConfig.load(p)
    assert back.web_point_size_auto is False and back.web_point_size == 0.06


def test_set_view_ignores_out_of_range_point_size():
    """Out-of-range sizes are dropped without disturbing the current value."""
    import types
    state = types.SimpleNamespace(config=None, ui_state=web.UiState(),
                                  clients=set(), controller=None)
    asyncio.run(web._handle_inbound(state, {"type": "set_view", "point_size": 99.0}))
    assert state.ui_state.point_size == web.UiState().point_size


# --- see-through / x-ray: state echo + persistence (owner ask, 2026-07-31) --

def test_see_through_defaults_off_everywhere():
    """The default must reproduce the opaque render: 0 in UiState, 0 in the
    config, and the client skips the extra pass entirely at 0."""
    assert web.UiState().see_through == 0.0
    assert ViewerConfig().web_see_through == 0.0
    assert web._state_message(web.UiState())["see_through"] == 0.0


def test_state_message_carries_see_through():
    assert web._state_message(web.UiState(see_through=0.4))["see_through"] == 0.4


def test_set_view_see_through_updates_and_persists(tmp_path):
    """End-to-end through the real inbound handler: the strength lands in
    UiState, in the `state` echo, and in roomscan.toml."""
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    state = types.SimpleNamespace(config=ViewerConfig(), ui_state=web.UiState(),
                                  clients=set(), controller=None)
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(state, {"type": "set_view", "see_through": 0.35}))
    finally:
        config_mod.config_path = orig
    assert state.ui_state.see_through == 0.35
    assert web._state_message(state.ui_state)["see_through"] == 0.35
    assert ViewerConfig.load(p).web_see_through == 0.35


def test_set_view_ignores_bad_see_through():
    """Out-of-range and non-numeric values are dropped, keeping the current one."""
    import types
    state = types.SimpleNamespace(config=None, ui_state=web.UiState(see_through=0.5),
                                  clients=set(), controller=None)
    for bad in (1.5, -0.2, "opaque", None):
        asyncio.run(web._handle_inbound(state, {"type": "set_view", "see_through": bad}))
        assert state.ui_state.see_through == 0.5


def test_ui_from_config_maps_see_through_and_rejects_out_of_range():
    assert web.ui_from_config(ViewerConfig(web_see_through=0.6)).see_through == 0.6
    default = web.UiState().see_through
    assert web.ui_from_config(ViewerConfig(web_see_through=2.0)).see_through == default
    assert web.ui_from_config(ViewerConfig(web_see_through="x")).see_through == default


def test_apply_ui_to_config_writes_see_through():
    cfg = ViewerConfig()
    web.apply_ui_to_config(web.UiState(see_through=0.25), cfg)
    assert cfg.web_see_through == 0.25


# --- view mode: state echo + persistence (owner ask, 2026-07-29) ------------

def test_state_message_carries_view_mode():
    assert web._state_message(web.UiState())["view_mode"] == "world"
    assert web._state_message(web.UiState(view_mode="mirror"))["view_mode"] == "mirror"


def test_set_view_view_mode_updates_and_persists(tmp_path):
    """End-to-end through the real inbound handler: the mode lands in UiState,
    in the `state` echo, and in roomscan.toml."""
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    state = types.SimpleNamespace(config=ViewerConfig(), ui_state=web.UiState(),
                                  clients=set(), controller=None)
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(state, {"type": "set_view", "view_mode": "fpv"}))
    finally:
        config_mod.config_path = orig
    assert state.ui_state.view_mode == "fpv"
    assert web._state_message(state.ui_state)["view_mode"] == "fpv"
    assert ViewerConfig.load(p).web_view_mode == "fpv"


def test_set_view_rejects_unknown_view_mode():
    """Untrusted inbound: an unknown mode must be dropped, not stored -- it
    would otherwise reach `view_rotation` and silently fall through to World."""
    import types
    state = types.SimpleNamespace(config=None, ui_state=web.UiState(),
                                  clients=set(), controller=None)
    asyncio.run(web._handle_inbound(state, {"type": "set_view", "view_mode": "selfie"}))
    assert state.ui_state.view_mode == "world"


def test_ui_from_config_maps_view_mode_and_rejects_garbage():
    assert web.ui_from_config(ViewerConfig(web_view_mode="mirror")).view_mode == "mirror"
    assert web.ui_from_config(ViewerConfig(web_view_mode="wat")).view_mode == "world"


def test_apply_ui_to_config_writes_view_mode():
    cfg = ViewerConfig()
    web.apply_ui_to_config(web.UiState(view_mode="fpv"), cfg)
    assert cfg.web_view_mode == "fpv"


# --- per-mode camera framing (owner ask, 2026-07-30) ------------------------

def test_default_view_cam_copies_are_independent():
    """`UiState` must not share the module-level `ViewCam` objects: they are
    mutable, so a slider drag in one session would silently rewrite the
    defaults (and, in tests, leak between cases)."""
    a, b = web.UiState(), web.UiState()
    a.view_cam["fpv"].distance_m = 9.0
    assert b.view_cam["fpv"].distance_m == web._DEFAULT_VIEW_CAM["fpv"].distance_m
    assert web._DEFAULT_VIEW_CAM["fpv"].distance_m != 9.0


def test_fpv_baseline_is_the_zero_offset():
    """The whole scheme is 'FPV is the ground truth, everything is an offset
    from it'. That only holds if all-zero really means 'at the sensor' — which
    the client relies on when it builds the pose (`scene.js` poseFor)."""
    zero = web.ViewCam()
    assert (zero.distance_m, zero.height_m, zero.rotation_deg) == (0.0, 0.0, 0.0)
    # ...and the shipped defaults are a small, non-zero nudge off that baseline,
    # because a camera exactly at the optical centre renders flat.
    fpv = web._DEFAULT_VIEW_CAM["fpv"]
    assert 0.0 < fpv.distance_m < 1.0 and 0.0 < fpv.height_m < 1.0


def test_state_message_carries_all_three_view_cams():
    m = web._state_message(web.UiState())["view_cam"]
    assert set(m) == {"world", "fpv", "mirror"}
    assert set(m["world"]) == {"distance_m", "height_m", "rotation_deg"}
    assert m["world"]["distance_m"] == web._DEFAULT_VIEW_CAM["world"].distance_m


def test_set_view_cam_edits_only_the_selected_mode(tmp_path):
    """The sliders show the selected mode, so that is the only slot they may
    touch — editing in FPV must not disturb World's framing."""
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    state = types.SimpleNamespace(config=ViewerConfig(), ui_state=web.UiState(view_mode="fpv"),
                                  clients=set(), controller=None)
    world_before = web.replace(state.ui_state.view_cam["world"])
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(state, {
            "type": "set_view", "cam_distance": 1.25, "cam_height": 0.8, "cam_rotation": -35.0}))
    finally:
        config_mod.config_path = orig
    fpv = state.ui_state.view_cam["fpv"]
    assert (fpv.distance_m, fpv.height_m, fpv.rotation_deg) == (1.25, 0.8, -35.0)
    assert state.ui_state.view_cam["world"] == world_before
    back = ViewerConfig.load(p)
    assert back.web_cam_fpv_distance_m == 1.25 and back.web_cam_fpv_rotation_deg == -35.0
    assert back.web_cam_world_distance_m == world_before.distance_m


def test_set_view_cam_lands_on_the_new_mode_when_both_change():
    """A combined message must apply the framing to the mode being switched TO
    — otherwise the first drag after a mode switch would edit the mode the user
    just left."""
    import types
    state = types.SimpleNamespace(config=None, ui_state=web.UiState(view_mode="world"),
                                  clients=set(), controller=None)
    asyncio.run(web._handle_inbound(
        state, {"type": "set_view", "view_mode": "mirror", "cam_distance": 2.0}))
    assert state.ui_state.view_cam["mirror"].distance_m == 2.0
    assert state.ui_state.view_cam["world"].distance_m == web._DEFAULT_VIEW_CAM["world"].distance_m


def test_set_view_cam_rejects_out_of_range_and_non_numeric():
    import types
    state = types.SimpleNamespace(config=None, ui_state=web.UiState(view_mode="fpv"),
                                  clients=set(), controller=None)
    before = web.replace(state.ui_state.view_cam["fpv"])
    for bad in ({"cam_distance": 999.0}, {"cam_distance": -1.0}, {"cam_height": 99.0},
                {"cam_rotation": 400.0}, {"cam_distance": "near"}, {"cam_height": None}):
        asyncio.run(web._handle_inbound(state, {"type": "set_view", **bad}))
    assert state.ui_state.view_cam["fpv"] == before


def test_set_view_cam_reset_restores_that_mode_only():
    import types
    state = types.SimpleNamespace(config=None, ui_state=web.UiState(view_mode="fpv"),
                                  clients=set(), controller=None)
    state.ui_state.view_cam["fpv"].distance_m = 7.0
    state.ui_state.view_cam["world"].distance_m = 7.0
    asyncio.run(web._handle_inbound(state, {"type": "set_view", "cam_reset": True}))
    assert state.ui_state.view_cam["fpv"] == web._DEFAULT_VIEW_CAM["fpv"]
    assert state.ui_state.view_cam["world"].distance_m == 7.0


def test_view_cam_round_trips_through_config():
    cfg = ViewerConfig()
    ui = web.UiState()
    ui.view_cam["mirror"] = web.ViewCam(3.5, -1.25, 90.0)
    web.apply_ui_to_config(ui, cfg)
    assert cfg.web_cam_mirror_distance_m == 3.5
    assert cfg.web_cam_mirror_height_m == -1.25
    assert cfg.web_cam_mirror_rotation_deg == 90.0
    assert web.ui_from_config(cfg).view_cam["mirror"] == web.ViewCam(3.5, -1.25, 90.0)


def test_orbit_defaults_off_at_a_slow_speed():
    """Off by default — a scene that starts spinning on its own would be a
    surprise — and 6 deg/s is one revolution per minute, i.e. "slow orbit"."""
    ui = web.UiState()
    assert ui.orbit_enabled is False
    assert ui.orbit_speed_deg_s == 6.0


def test_state_message_carries_orbit():
    m = web._state_message(web.UiState(orbit_enabled=True, orbit_speed_deg_s=-12.5))
    assert m["orbit_enabled"] is True
    assert m["orbit_speed_deg_s"] == -12.5


def test_set_view_orbit_updates_and_persists(tmp_path):
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    state = types.SimpleNamespace(config=ViewerConfig(), ui_state=web.UiState(),
                                  clients=set(), controller=None)
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(
            state, {"type": "set_view", "orbit": True, "orbit_speed": -20.0}))
    finally:
        config_mod.config_path = orig
    assert state.ui_state.orbit_enabled is True
    assert state.ui_state.orbit_speed_deg_s == -20.0
    back = ViewerConfig.load(p)
    assert back.web_orbit_enabled is True and back.web_orbit_speed_deg_s == -20.0
    assert web.ui_from_config(back).orbit_speed_deg_s == -20.0


def test_set_view_rejects_out_of_range_orbit_speed():
    """Negative is legal (it reverses); absurd is not. A bad value must leave
    the current speed alone rather than parking the orbit at 0."""
    import types
    state = types.SimpleNamespace(config=None, ui_state=web.UiState(orbit_speed_deg_s=6.0),
                                  clients=set(), controller=None)
    for bad in (600.0, -600.0, "fast", None):
        asyncio.run(web._handle_inbound(state, {"type": "set_view", "orbit_speed": bad}))
        assert state.ui_state.orbit_speed_deg_s == 6.0
    # ...and 0 IS in range: it parks the orbit without disabling it.
    asyncio.run(web._handle_inbound(state, {"type": "set_view", "orbit_speed": 0.0}))
    assert state.ui_state.orbit_speed_deg_s == 0.0


def test_ui_from_config_rejects_corrupt_orbit_speed():
    assert web.ui_from_config(ViewerConfig(web_orbit_speed_deg_s=1e6)).orbit_speed_deg_s == 6.0
    assert web.ui_from_config(ViewerConfig(web_orbit_speed_deg_s="quick")).orbit_speed_deg_s == 6.0


# --- Oscillate mode (owner ask, 2026-07-31) ----------------------------------

def test_orbit_mode_defaults_to_continuous():
    ui = web.UiState()
    assert ui.orbit_mode == "continuous"
    assert ui.orbit_amplitude_deg == 45.0


def test_state_message_carries_orbit_mode_and_amplitude():
    m = web._state_message(web.UiState(orbit_mode="oscillate", orbit_amplitude_deg=30.0))
    assert m["orbit_mode"] == "oscillate"
    assert m["orbit_amplitude_deg"] == 30.0


def test_set_view_orbit_mode_updates_and_persists(tmp_path):
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    state = types.SimpleNamespace(config=ViewerConfig(), ui_state=web.UiState(),
                                  clients=set(), controller=None)
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(
            state, {"type": "set_view", "orbit_mode": "oscillate", "orbit_amplitude": 30.0}))
    finally:
        config_mod.config_path = orig
    assert state.ui_state.orbit_mode == "oscillate"
    assert state.ui_state.orbit_amplitude_deg == 30.0
    back = ViewerConfig.load(p)
    assert back.web_orbit_mode == "oscillate" and back.web_orbit_amplitude_deg == 30.0
    assert web.ui_from_config(back).orbit_mode == "oscillate"
    assert web.ui_from_config(back).orbit_amplitude_deg == 30.0


def test_set_view_rejects_bad_orbit_mode():
    """Unknown mode: logged and dropped, current value untouched -- same
    reject/no-op shape as `set_view` colormap/surface_mode/view_mode."""
    import types
    state = types.SimpleNamespace(config=None, ui_state=web.UiState(orbit_mode="continuous"),
                                  clients=set(), controller=None)
    asyncio.run(web._handle_inbound(state, {"type": "set_view", "orbit_mode": "spiral"}))
    assert state.ui_state.orbit_mode == "continuous"


def test_set_view_rejects_out_of_range_orbit_amplitude():
    """Out-of-range or non-numeric amplitude is dropped, keeping the current
    value -- same shape as the orbit-speed reject test above."""
    import types
    state = types.SimpleNamespace(config=None, ui_state=web.UiState(orbit_amplitude_deg=45.0),
                                  clients=set(), controller=None)
    for bad in (0.0, 4.9, 180.1, 999.0, "wide", None):
        asyncio.run(web._handle_inbound(state, {"type": "set_view", "orbit_amplitude": bad}))
        assert state.ui_state.orbit_amplitude_deg == 45.0
    # ...and the slider's own boundaries ARE in range.
    asyncio.run(web._handle_inbound(state, {"type": "set_view", "orbit_amplitude": 5.0}))
    assert state.ui_state.orbit_amplitude_deg == 5.0
    asyncio.run(web._handle_inbound(state, {"type": "set_view", "orbit_amplitude": 180.0}))
    assert state.ui_state.orbit_amplitude_deg == 180.0


def test_ui_from_config_maps_orbit_mode_and_amplitude():
    cfg = ViewerConfig(web_orbit_mode="oscillate", web_orbit_amplitude_deg=90.0)
    ui = web.ui_from_config(cfg)
    assert ui.orbit_mode == "oscillate"
    assert ui.orbit_amplitude_deg == 90.0


def test_ui_from_config_rejects_corrupt_orbit_mode_and_amplitude():
    cfg = ViewerConfig(web_orbit_mode="spiral", web_orbit_amplitude_deg="wide")
    ui = web.ui_from_config(cfg)
    assert ui.orbit_mode == "continuous"
    assert ui.orbit_amplitude_deg == 45.0
    assert web.ui_from_config(ViewerConfig(web_orbit_amplitude_deg=999.0)).orbit_amplitude_deg == 45.0


def test_ui_from_config_rejects_corrupt_view_cam_values():
    """A hand-edited or corrupt roomscan.toml must not be able to park the
    camera 1000 m away with no way back through the UI."""
    cfg = ViewerConfig(web_cam_world_distance_m=1000.0, web_cam_fpv_rotation_deg="sideways",
                       web_cam_mirror_height_m=-99.0)
    ui = web.ui_from_config(cfg)
    assert ui.view_cam["world"] == web._DEFAULT_VIEW_CAM["world"]
    assert ui.view_cam["fpv"] == web._DEFAULT_VIEW_CAM["fpv"]
    assert ui.view_cam["mirror"] == web._DEFAULT_VIEW_CAM["mirror"]


def test_root_redirects_to_static_index():
    """The bare site root is a convenience redirect to the app entry point --
    typing the hostname alone used to 404 (only /static and /results were
    mounted)."""
    route = next(r for r in web.app.routes if getattr(r, "path", None) == "/")
    assert "GET" in route.methods
    resp = asyncio.run(web.root_redirect())
    assert resp.status_code == 307
    assert resp.headers["location"] == "/static/index.html"


# =============================================================================
# 13. Admin endpoints -- FileHub bridge mode + server restart (2026-07-31)
# =============================================================================

def test_bridge_mode_reports_missing_script(tmp_path):
    """A missing script must come back as a readable reason, not an exception:
    the button's whole job is to say what happened."""
    res = web.run_bridge_mode(script=tmp_path / "nope.sh")
    assert res["ok"] is False
    assert "not found" in res["error"]
    assert res["returncode"] is None


def test_bridge_mode_captures_output_and_success(tmp_path):
    script = tmp_path / "fake-bridge.sh"
    script.write_text("#!/bin/bash\necho 'bridge applied'\nexit 0\n")
    script.chmod(0o755)
    res = web.run_bridge_mode(script=script)
    assert res["ok"] is True
    assert res["returncode"] == 0
    assert "bridge applied" in res["output"]
    assert res["error"] is None


def test_bridge_mode_reports_nonzero_exit_as_failure(tmp_path):
    """A script that runs but fails is NOT ok -- an earlier draft keyed success
    off "did it execute", which would report a dead router as a fixed one."""
    script = tmp_path / "fail.sh"
    script.write_text("#!/bin/bash\necho 'cannot reach router' >&2\nexit 1\n")
    script.chmod(0o755)
    res = web.run_bridge_mode(script=script)
    assert res["ok"] is False
    assert res["returncode"] == 1
    assert "cannot reach router" in res["output"]


def test_bridge_mode_times_out_without_hanging(tmp_path):
    script = tmp_path / "hang.sh"
    script.write_text("#!/bin/bash\nsleep 30\n")
    script.chmod(0o755)
    res = web.run_bridge_mode(script=script, timeout_s=0.5)
    assert res["ok"] is False
    assert "did not finish" in res["error"]


def test_restart_argv_uses_module_form_not_argv0():
    import sys
    """Under `python -m roomscan.web`, sys.argv[0] is the expanded path to
    web.py; re-running that executes the module outside its package and its
    relative imports blow up. The relaunch must use -m."""
    argv = web.restart_argv()
    assert argv[0] == sys.executable
    assert argv[1:3] == ["-m", "roomscan.web"]
    assert not argv[1].endswith("web.py")


def test_restart_command_defers_and_execs():
    """The child must outlive this process and bind only after the port frees,
    so it sleeps first and `exec`s (no lingering sh in the process tree)."""
    cmd = web.restart_command(delay_s=2.0)
    assert cmd[:2] == ["sh", "-c"]
    assert cmd[2].startswith("sleep 2;")
    assert " exec " in cmd[2]
    assert "-m roomscan.web" in cmd[2]


def test_admin_endpoints_are_post_only():
    """POST-only so a prefetch, crawler, or browser refresh can never restart
    the server or reconfigure the router."""
    paths = {"/api/bridge-mode", "/api/restart"}
    found = {r.path: r.methods for r in web.app.routes if getattr(r, "path", "") in paths}
    assert set(found) == paths
    for path, methods in found.items():
        assert "POST" in methods, path
        assert "GET" not in methods, path


def test_transport_counters_none_when_not_udp():
    """Replay and serial sources have no fragment layer; the field must be null
    rather than fabricated zeros, which would read as a healthy UDP link."""
    class _S: pass
    s = _S(); s.controller = None
    assert web.transport_counters(s) is None
    s.controller = _S(); s.controller._live_underlying = object()
    assert web.transport_counters(s) is None


def test_transport_counters_reports_udp_fragment_health(monkeypatch):
    """`gaps` says a frame vanished; these say why. reordered vs lost is the
    split that makes "did the pacer help?" answerable at all (BUG-042)."""
    class _FakeUdp(web.UdpSource):
        def __init__(self):   # bypass socket/mDNS setup
            self.frames_incomplete = 2
            self.frags_lost = 3
            self.frags_reordered = 11
            self.frags_duplicate = 1
            self.frags_invalid = 0

    class _S: pass
    s = _S(); s.controller = _S(); s.controller._live_underlying = _FakeUdp()
    assert web.transport_counters(s) == {
        "frames_incomplete": 2, "frags_lost": 3,
        "frags_reordered": 11, "frags_duplicate": 1, "frags_invalid": 0,
    }


# --- 14. Live/View source + display (2026-07-31) ----------------------------
#
# The Live/View consolidation landed with no handler-level tests at all, and
# each of the three below fails if its defect is reintroduced -- verified by
# reverting the fix and watching it go red.

class _FakeCtrl:
    """Minimal SessionController stand-in for the inbound handlers."""
    def __init__(self, *, mode="replay", has_stream_9=True, captures_dir="captures",
                 replay_path="take.bin"):
        self.mode = mode
        self.replay_path = replay_path if mode == "replay" else None
        self.index = {"n_frames": 3, "seqs": [1, 2, 3], "has_stream_9": has_stream_9}
        self.captures_dir = captures_dir
        self.switched_to = None
        self.paused = False

    def switch_to_live(self):
        self.mode, self.replay_path, self.switched_to = "live", None, "live"

    def pause(self):
        self.paused = True


def _inbound_state(ui, ctrl):
    import types
    published = []
    bus = types.SimpleNamespace(publish=published.append)
    return types.SimpleNamespace(config=None, ui_state=ui, clients=set(),
                                 controller=ctrl, bus=bus, slam_runner=None,
                                 detailed_runner=None), published


def test_state_echo_keeps_capability_context_on_an_unrelated_control():
    """A colour change must not re-advertise SLAM on a legacy capture.

    The client drives its disabled state purely from this echo (one-way flow),
    so a context-free `_state_message(ui)` -- which defaults `slam_available`
    to True -- would silently re-enable the SLAM segments on a stream-9-less
    capture and clear the Detailed badge.
    """
    import asyncio, json
    ui = web.UiState(source="view", display="point_cloud", selected_capture="old.bin")
    ctrl = _FakeCtrl(has_stream_9=False)
    state, _ = _inbound_state(ui, ctrl)
    sent = []

    async def drive():
        async def capture_text(clients, text):
            sent.append(json.loads(text))
        orig, web._broadcast_text = web._broadcast_text, capture_text
        try:
            await web._handle_inbound(state, {"type": "set_color", "mode": "depth"})
        finally:
            web._broadcast_text = orig

    asyncio.run(drive())
    echoes = [m for m in sent if m.get("type") == "state"]
    assert echoes, "set_color must echo state"
    assert echoes[-1]["slam_available"] is False


def test_detailed_runner_status_reports_real_elapsed_time_and_eta(tmp_path, monkeypatch):
    """Detailed progress is server-owned so every tab gets the same ETA.

    The estimate attached to a capture is only a planning hint.  Once a build
    starts, elapsed time and ETA must come from the current worker progress,
    including the first batch before a mesh can be shown.
    """
    import types

    class _Worker:
        timestamps = [0.0, 1.0, 2.0, 3.0]

        @staticmethod
        def latest():
            return types.SimpleNamespace(fraction=0.25, done=False, stats=None)

    runner = web.DetailedRunner(bus=LogBus(), results_dir=tmp_path / "results")
    runner._worker = _Worker()
    runner._capture = tmp_path / "take.bin"
    runner._started_at = 100.0
    monkeypatch.setattr(web.time, "monotonic", lambda: 110.0)

    status = runner.status()
    assert status == {
        "type": "detailed", "capture": "take.bin", "phase": "frames", "processed": 1, "total": 4,
        "fraction": 0.25, "done": False, "stats": None, "elapsed_s": 10.0,
        "eta_s": 30.0, "mesh_every": runner.preset.mesh_every,
    }

    runner._worker.latest = lambda: None
    initial = runner.status()
    assert initial["type"] == "detailed"
    assert (initial["processed"], initial["fraction"], initial["eta_s"]) == (0, 0.0, None)


def test_detailed_runner_freezes_elapsed_time_when_the_build_finishes(tmp_path, monkeypatch):
    """"Completed in 4:12" must mean the build took 4:12 -- forever.

    `status()` derives elapsed from `time.monotonic() - _started_at`, which does
    not stop when the worker does. Observed on the real rig: a finished build
    reported `elapsed_s` climbing 4910.5 -> 4914.5 in four seconds, and the UI's
    "Completed in" read 81:01 for a build that had taken about three and a half
    minutes. Same family as BUG-050 -- a duration nobody thought to bound.
    """
    import types

    class _Worker:
        timestamps = [0.0, 1.0, 2.0, 3.0]

        @staticmethod
        def latest():
            return types.SimpleNamespace(fraction=1.0, done=True, stats={"frames": 4},
                                         trajectory=(), phase="offline_only")

    runner = web.DetailedRunner(bus=LogBus(), results_dir=tmp_path / "results")
    runner._worker = _Worker()
    runner._capture = tmp_path / "take.bin"
    runner._started_at = 100.0

    monkeypatch.setattr(web.time, "monotonic", lambda: 352.0)
    first = runner.status()
    assert first["done"] is True
    assert first["elapsed_s"] == 252.0

    # An hour later, the same finished build still took 252 seconds.
    monkeypatch.setattr(web.time, "monotonic", lambda: 3952.0)
    assert runner.status()["elapsed_s"] == 252.0

    # A fresh build clears it rather than inheriting the previous one's time.
    runner._elapsed_at_done = None
    runner._started_at = 4000.0
    monkeypatch.setattr(web.time, "monotonic", lambda: 4010.0)
    assert runner.status()["elapsed_s"] == 10.0


def test_detailed_runner_reports_cached_mesh_loading_without_blocking(tmp_path, monkeypatch):
    """The websocket can acknowledge a large PLY load before Open3D finishes it."""
    runner = web.DetailedRunner(bus=LogBus(), results_dir=tmp_path / "results")
    capture = tmp_path / "take.bin"
    runner._capture = capture
    runner._loading_capture = capture
    runner._loading_started_at = 100.0
    monkeypatch.setattr(web.time, "monotonic", lambda: 107.5)

    assert runner.status() == {
        "type": "detailed", "capture": "take.bin", "phase": "loading_cached",
        "processed": 0, "total": 0, "fraction": 0.0, "done": False,
        "stats": None, "elapsed_s": 7.5, "eta_s": None,
        "mesh_every": runner.preset.mesh_every,
    }


def test_detailed_runner_status_exposes_the_latest_scanner_pose(tmp_path):
    """Detailed maps stay world-fixed; FPV/Mirror therefore need the worker's
    latest world<-camera pose rather than a made-up transform in the browser."""
    import types

    pose = np.eye(4, dtype=np.float64)
    pose[:3, 3] = (1.25, -0.5, 2.75)

    class _Worker:
        timestamps = [0.0]

        @staticmethod
        def latest():
            return types.SimpleNamespace(
                fraction=1.0, done=True, stats=None, trajectory=[pose])

    runner = web.DetailedRunner(bus=LogBus(), results_dir=tmp_path / "results")
    runner._worker = _Worker()
    runner._capture = tmp_path / "take.bin"
    status = runner.status()
    assert status["pose"] == pytest.approx(pose.reshape(-1).tolist())


def test_last_tum_pose_recovers_the_cached_detailed_camera_pose(tmp_path):
    """A finished Detailed build has no worker, so its FPV/Mirror camera must
    recover the final scanner pose from the sidecar that was saved with it."""
    pose = np.eye(4, dtype=np.float64)
    pose[:3, 3] = (1.25, -0.5, 2.75)
    a = math.radians(30.0)
    pose[:3, :3] = np.array([[math.cos(a), -math.sin(a), 0.0],
                             [math.sin(a), math.cos(a), 0.0],
                             [0.0, 0.0, 1.0]])
    tum = tmp_path / "take.tum"
    from roomscan.slam.metrics import write_tum
    write_tum(tum, [1.0], [pose])
    assert web._last_tum_pose(tum) == pytest.approx(pose, abs=1e-5)


def test_save_is_live_slam_only():
    """Live SLAM keeps its one-shot export (owner decision, 2026-07-31): an
    unrecorded live scan is unrepeatable. Replay SLAM is a preview and must
    refuse with a reason rather than silently writing or silently doing
    nothing."""
    import asyncio
    saved = []

    class _Slam:
        def save(self, ply, tum):
            saved.append(ply.name)
            return 1234

    # Replay + SLAM -> refused, nothing written.
    ui = web.UiState(source="view", display="slam")
    state, published = _inbound_state(ui, _FakeCtrl())
    state.slam_runner = _Slam()
    asyncio.run(web._handle_inbound(state, {"type": "save"}))
    assert not saved
    assert any("preview" in line for line in published), published

    # Live + SLAM -> writes.
    ui = web.UiState(source="live", display="slam")
    state, published = _inbound_state(ui, _FakeCtrl(mode="live"))
    state.slam_runner = _Slam()
    asyncio.run(web._handle_inbound(state, {"type": "save"}))
    assert len(saved) == 1 and saved[0].endswith(".ply")
    assert any("1234 verts" in line for line in published), published


def test_set_display_refuses_slam_on_a_legacy_capture():
    """A capture with no stream 9 has no rotation prior, so SLAM would build an
    empty map. Refuse with an explanation and leave the display untouched."""
    import asyncio
    ui = web.UiState(source="view", display="point_cloud")
    state, published = _inbound_state(ui, _FakeCtrl(has_stream_9=False))
    asyncio.run(web._handle_inbound(state, {"type": "set_display", "display": "slam"}))
    assert ui.display == "point_cloud"
    assert any("stream 9" in line for line in published), published


def test_preview_is_a_view_only_display_for_the_loaded_capture():
    """Preview is a first-class display, but it cannot show a live source or
    an arbitrary path supplied by a websocket peer."""
    import asyncio
    ui = web.UiState(source="view", selected_capture="take.bin")
    state, _ = _inbound_state(ui, _FakeCtrl())
    asyncio.run(web._handle_inbound(state, {"type": "set_display", "display": "preview"}))
    assert ui.display == "preview" and ui.mode == "realtime"

    ui = web.UiState(source="live")
    state, published = _inbound_state(ui, _FakeCtrl(mode="live"))
    asyncio.run(web._handle_inbound(state, {"type": "set_display", "display": "preview"}))
    assert ui.display == "point_cloud"
    assert any("load a capture" in line for line in published), published


def test_detailed_display_pauses_replay_and_starts_cached_load_immediately():
    """Detailed has its own offline worker; replay must not run underneath it."""
    import asyncio

    class _Runner:
        def __init__(self):
            self.loaded = None

        def begin_load_cached(self, capture):
            self.loaded = capture
            return True

        def status(self):
            return {"type": "detailed", "phase": "loading_cached", "capture": "take.bin",
                    "processed": 0, "total": 0, "fraction": 0.0, "done": False}

    ctrl = _FakeCtrl()
    ui = web.UiState(source="view", display="point_cloud", selected_capture="take.bin")
    state, _ = _inbound_state(ui, ctrl)
    runner = _Runner()
    state.detailed_runner = runner
    asyncio.run(web._handle_inbound(state, {"type": "set_display", "display": "detailed"}))

    assert ui.display == "detailed"
    assert ctrl.paused is True
    assert runner.loaded == "take.bin"


def test_replay_controller_defaults_to_one_times_speed(tmp_path):
    cap = tmp_path / "take.bin"
    _make_depth_capture_flat(cap, n_frames=3, base=1000.0)
    ctrl, _ = _make_controller(tmp_path, replay_path=str(cap))
    try:
        assert ctrl.speed_fps == web._SPEED_BASE_FPS
        assert ctrl.pacer.interval == pytest.approx(1.0 / web._SPEED_BASE_FPS)
    finally:
        ctrl.close()

    live, _ = _make_controller(tmp_path)
    try:
        assert live.speed_fps == web._SPEED_BASE_FPS
    finally:
        live.close()


def test_rail_cards_data_attributes_and_default_collapsed_states():
    """Verify that all rail cards have data-card-id attributes and start with most
    collapsed by default (keeping telemetry and view expanded)."""
    html_path = Path(__file__).parent.parent / "src" / "roomscan" / "static" / "index.html"
    content = html_path.read_text(encoding="utf-8")

    expected_cards = {
        "telemetry": False,  # Expanded
        "sensors": True,     # Collapsed
        "slam-hud": True,    # Collapsed
        "resources": False,  # Expanded (only rendered at all in Live SLAM)
        "browser": False,    # Expanded (§12: the View page's file browser)
        "preview": False,    # Expanded (§12: rendered only with a tile selected)
        "ir-view": True,     # Collapsed
        "diag": True,        # Collapsed
        "device": True,      # Collapsed
        "view": False,       # Expanded
        "capture": False,    # Expanded (§11: one Record button, Live page only)
        "transport": False,  # Expanded (§11: Playback, rendered only in replay)
        "slam-ctrl": True,   # Collapsed
        "log": True,         # Collapsed
    }

    import re
    # Match element tags with data-card-id="..." and inspect their class string
    pattern = re.compile(r'<([a-z0-9]+)\s+[^>]*data-card-id="([^"]+)"[^>]*>', re.IGNORECASE)
    matches = pattern.findall(content)
    found = {card_id for _, card_id in matches}

    assert expected_cards.keys() <= found, f"Missing data-card-id tags: {expected_cards.keys() - found}"
    assert found <= expected_cards.keys(), (
        f"Cards present in index.html but not listed here: {found - expected_cards.keys()}. "
        "Add them to expected_cards (and to CARD_ICONS/CARD_TITLES in layout.js).")

    # Verify collapse status in initial HTML. Driven from `expected_cards`, not
    # from the regex matches: iterating the matches and indexing the dict raises
    # KeyError on any new card, which reads as a crash rather than a test failure.
    tags = {card_id: tag for tag, card_id in
            ((m.group(0), m.group("id")) for m in re.finditer(
                r'<[a-z0-9]+\s+[^>]*data-card-id="(?P<id>[^"]+)"[^>]*>', content, re.IGNORECASE))}
    for card_id, expected_collapsed in expected_cards.items():
        tag_str = tags[card_id]
        class_attr = tag_str.split('class="')[1].split('"')[0] if 'class="' in tag_str else ""
        has_collapsed_class = "collapsed" in class_attr.split()
        assert has_collapsed_class == expected_collapsed, f"Card '{card_id}' expected collapsed={expected_collapsed}, got {has_collapsed_class}"


def test_view_section_subgroups():
    """Verify that the View section is broken into collapsible sub-areas with data-subgroup-id."""
    html_path = Path(__file__).parent.parent / "src" / "roomscan" / "static" / "index.html"
    content = html_path.read_text(encoding="utf-8")

    import re
    pattern = re.compile(r'<details\s+[^>]*data-subgroup-id="([^"]+)"[^>]*>', re.IGNORECASE)
    matches = set(pattern.findall(content))

    expected_subgroups = {"view-camera", "view-color", "view-surface"}
    assert expected_subgroups <= matches, f"Missing view subgroups: {expected_subgroups - matches}"


def test_squircles_and_overflow_prevention():
    """Verify squircle styling & layout icons logic and overflow-x prevention."""
    static_dir = Path(__file__).parent.parent / "src" / "roomscan" / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    layout_js = (static_dir / "layout.js").read_text(encoding="utf-8")

    assert ".squircle-bar" in index_html
    assert ".squircle-btn" in index_html
    assert "overflow-x: hidden;" in index_html
    assert "CARD_ICONS" in layout_js
    assert "updateSquircles" in layout_js





# ---------------------------------------------------------------------------
# 15. Elevation readout (owner ask, 2026-07-31) — feet, hPa, and a Δ datum
# ---------------------------------------------------------------------------
#
# The Sensors card used to print raw pascals, which is not a number anyone
# reads. It now prints barometric height above sea level in feet, low-passed
# because BUG-037 measured ~267 mm RMS of white noise per barometer sample
# (~1.2 ft), with the absolute pressure beside it and the sea-level reference
# it is measured against under Diagnostics.

def _env_ss(pressure_pa=101000.0, temp_c=22.0, n=1):
    """A SensorState with a quat + n ENV samples at `pressure_pa`."""
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    for i in range(n):
        ss.feed(_sframe(StreamId.ENV,
                        struct.pack("<5f", pressure_pa, 1.0, 2.0, 3.0, temp_c),
                        t_us=1000 + i * 3_000_000))
    return ss


def test_build_sensor_message_reports_elevation_in_feet():
    """The altitude is `slam.frames.baro_height_m` — the SAME formula SLAM
    uses — converted to feet, never a second barometric formula."""
    from roomscan.slam.frames import baro_height_m
    ss = _env_ss(pressure_pa=98000.0)
    msg = web.build_sensor_message(ss, None, msl_pa=101300.0, msl_source="api", msl_age_s=12.0)
    expect_ft = baro_height_m(98000.0, 101300.0) * web.FT_PER_M
    assert msg["elevation_ft"] == pytest.approx(expect_ft, abs=0.15)
    assert msg["pressure_hpa"] == pytest.approx(980.0, abs=0.01)
    assert msg["msl_pa"] == pytest.approx(101300.0)
    assert msg["msl_source"] == "api"
    assert msg["msl_age_s"] == pytest.approx(12.0)
    # The raw pascals stay on the wire: the Diagnostics drawer still shows them
    # and nothing that already consumed them has to change.
    assert msg["pressure_pa"] == pytest.approx(98000.0, abs=0.5)
    json.dumps(msg)


def test_build_sensor_message_defaults_to_the_fallback_reference():
    """No `msl_pa` argument at all (the default, and what a test or an offline
    box gets) must still produce a number — against 101325 Pa — and SAY that is
    what happened."""
    msg = web.build_sensor_message(_env_ss(pressure_pa=101325.0), None)
    assert msg["msl_pa"] == pytest.approx(101325.0)
    assert msg["msl_source"] == "fallback"
    assert msg["msl_age_s"] is None
    assert msg["elevation_ft"] == pytest.approx(0.0, abs=0.05)


def test_build_sensor_message_elevation_null_without_env():
    """A ToF-only (or quat-only) session has no pressure — every elevation
    field must be null rather than an altitude computed from a zero."""
    ss = SensorState()
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    msg = web.build_sensor_message(ss, None)
    assert msg["elevation_ft"] is None
    assert msg["pressure_hpa"] is None
    assert msg["elevation_hist"] == []


def test_build_sensor_message_elevation_datum_defaults_to_none():
    """The `None` default of a NEW parameter, asserted explicitly.

    engineering-practices.md:81-86: this function has silently shadowed a new
    argument with a local of the same name before (the BUG-026 follow-up), and
    it was only a test asserting the None default that caught it. So: pass
    nothing, expect null.
    """
    msg = web.build_sensor_message(_env_ss(), None)
    assert msg["elevation_datum_ft"] is None


def test_build_sensor_message_echoes_the_elevation_datum():
    msg = web.build_sensor_message(_env_ss(), None, elevation_datum_ft=912.34)
    assert msg["elevation_datum_ft"] == pytest.approx(912.3)


def test_build_sensor_message_elevation_hist_matches_the_pressure_hist():
    """The sparkline and the readout must be the same quantity: elevation_hist
    is the SAME decimated samples the pressure sparkline draws, converted."""
    from roomscan.slam.frames import baro_height_m
    ss = _env_ss(pressure_pa=101000.0, n=3)
    msg = web.build_sensor_message(ss, None, msl_pa=101325.0)
    assert len(msg["elevation_hist"]) == len(msg["pressure_hist"]) == 3
    for pa, ft in zip(msg["pressure_hist"], msg["elevation_hist"]):
        assert ft == pytest.approx(baro_height_m(pa, 101325.0) * web.FT_PER_M, abs=0.15)


def test_build_sensor_message_low_passes_the_elevation():
    """A raw readout would flicker over a foot several times a second
    (BUG-037: ~267 mm RMS per sample). One big pressure step must move the
    displayed elevation only a fraction of the way on the first update."""
    clock = {"t": 0.0}
    sm = web.ElevationSmoother(tau_s=6.0, clock=lambda: clock["t"])
    ss = _env_ss(pressure_pa=101325.0)
    first = web.build_sensor_message(ss, None, msl_pa=101325.0, altitude_smoother=sm)
    assert first["elevation_ft"] == pytest.approx(0.0, abs=0.05)   # first sample adopted outright

    step_ss = _env_ss(pressure_pa=98000.0)                          # ~+930 ft step
    target = web.build_sensor_message(step_ss, None, msl_pa=101325.0)["elevation_ft"]
    clock["t"] = 1.0                                                # one second later
    smoothed = web.build_sensor_message(step_ss, None, msl_pa=101325.0,
                                        altitude_smoother=sm)["elevation_ft"]
    # 1 - exp(-1/6) = 15.35% of the way there.
    assert smoothed == pytest.approx(target * 0.1535, rel=0.02)
    assert smoothed < target * 0.25

    # ...and it converges: 60 s (10 tau) gets essentially all the way.
    for i in range(2, 62):
        clock["t"] = float(i)
        smoothed = web.build_sensor_message(step_ss, None, msl_pa=101325.0,
                                            altitude_smoother=sm)["elevation_ft"]
    assert smoothed == pytest.approx(target, rel=0.001)


def test_elevation_smoother_is_time_based_not_per_sample():
    """The `sensor` message rides an elapsed-time gate, and a stalled stream
    (paused replay, idled device) must not stretch the effective time
    constant. Same elapsed time => same response, whatever the sample count."""
    def run(n_steps):
        clock = {"t": 0.0}
        sm = web.ElevationSmoother(tau_s=6.0, clock=lambda: clock["t"])
        sm.update(0.0)
        for i in range(1, n_steps + 1):
            clock["t"] = 6.0 * i / n_steps
            sm.update(100.0)
        return sm.value_m

    assert run(1) == pytest.approx(run(90), rel=0.02)
    assert run(90) == pytest.approx(100.0 * (1 - math.exp(-1.0)), rel=0.02)


def test_elevation_smoother_ignores_none_and_reports_feet():
    sm = web.ElevationSmoother(tau_s=6.0)
    assert sm.value_m is None and sm.value_ft is None
    assert sm.update(None) is None
    sm.update(100.0)
    assert sm.value_ft == pytest.approx(100.0 * web.FT_PER_M)
    sm.reset()
    assert sm.value_m is None


# --- set_elevation_datum inbound -------------------------------------------

def _elev_state(ui=None, *, smoothed_ft=None, config=None):
    import types
    published = []
    bus = types.SimpleNamespace(publish=published.append)
    smoother = web.ElevationSmoother()
    if smoothed_ft is not None:
        smoother.update(smoothed_ft / web.FT_PER_M)
    return types.SimpleNamespace(
        config=config, ui_state=ui if ui is not None else web.UiState(),
        clients=set(), controller=None, bus=bus, slam_runner=None,
        detailed_runner=None, elevation_smoother=smoother), published


def test_set_elevation_datum_captures_the_smoothed_elevation():
    """Happy path: `on: true` stores the CURRENT SMOOTHED elevation.

    Smoothed, not raw, on purpose — a datum taken from one barometer sample
    would bake ~1.2 ft of noise in as a constant offset for the whole session,
    which is exactly the mistake BUG-037 found in the SLAM height datum.
    """
    state, _ = _elev_state(smoothed_ft=935.2)
    asyncio.run(web._handle_inbound(state, {"type": "set_elevation_datum", "on": True}))
    assert state.ui_state.elevation_datum_ft == pytest.approx(935.2, abs=0.1)

    asyncio.run(web._handle_inbound(state, {"type": "set_elevation_datum", "on": False}))
    assert state.ui_state.elevation_datum_ft is None


@pytest.mark.parametrize("bad", [None, "true", 1, 0, [], {}, "on"])
def test_set_elevation_datum_rejects_a_non_bool(bad):
    """Reject/no-op half. A truthy-looking string must not set a datum, and a
    falsy-looking one must not clear an existing one."""
    ui = web.UiState(elevation_datum_ft=100.0)
    state, _ = _elev_state(ui, smoothed_ft=935.2)
    msg = {"type": "set_elevation_datum"} if bad is None else {"type": "set_elevation_datum", "on": bad}
    asyncio.run(web._handle_inbound(state, msg))
    assert ui.elevation_datum_ft == 100.0      # untouched


def test_set_elevation_datum_without_a_reading_says_so_and_does_nothing():
    """No barometer yet (ToF-only session): refuse with a bus line rather than
    capturing a datum of 0 ft, which would silently offset every later
    reading by the true elevation."""
    state, published = _elev_state()            # smoother never fed
    asyncio.run(web._handle_inbound(state, {"type": "set_elevation_datum", "on": True}))
    assert state.ui_state.elevation_datum_ft is None
    assert any("no barometer" in line for line in published), published


def test_set_elevation_datum_echoes_state():
    """The Δ button takes its pressed state from the echo, never from the
    click — so the echo has to carry the datum."""
    state, _ = _elev_state(smoothed_ft=935.2)
    sent = []

    async def drive():
        async def capture_text(clients, text):
            sent.append(json.loads(text))
        orig, web._broadcast_text = web._broadcast_text, capture_text
        try:
            await web._handle_inbound(state, {"type": "set_elevation_datum", "on": True})
        finally:
            web._broadcast_text = orig

    asyncio.run(drive())
    echoes = [m for m in sent if m.get("type") == "state"]
    assert echoes and echoes[-1]["elevation_datum_ft"] == pytest.approx(935.2, abs=0.1)


def test_set_elevation_datum_persists(tmp_path):
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    state, _ = _elev_state(smoothed_ft=935.2, config=ViewerConfig())
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(state, {"type": "set_elevation_datum", "on": True}))
        assert p.exists()
        assert ViewerConfig.load(p).elevation_datum_ft == pytest.approx(935.2, abs=0.1)
        # ...and survives a "restart": a fresh UiState seeded from that file.
        assert web.ui_from_config(ViewerConfig.load(p)).elevation_datum_ft == \
            pytest.approx(935.2, abs=0.1)

        asyncio.run(web._handle_inbound(state, {"type": "set_elevation_datum", "on": False}))
        assert ViewerConfig.load(p).elevation_datum_ft is None
        assert web.ui_from_config(ViewerConfig.load(p)).elevation_datum_ft is None
    finally:
        config_mod.config_path = orig


def test_elevation_datum_round_trips_through_the_flat_toml_writer(tmp_path):
    """`None` goes through the TOML writer as `""` (there is no null), and
    `ViewerConfig.load` has to turn it back into None — otherwise an unset
    datum loads as the STRING "" and every consumer has to defend against it."""
    p = tmp_path / "roomscan.toml"
    ViewerConfig(elevation_datum_ft=None).save(p)
    assert 'elevation_datum_ft = ""' in p.read_text()
    assert ViewerConfig.load(p).elevation_datum_ft is None

    ViewerConfig(elevation_datum_ft=812.5).save(p)
    assert ViewerConfig.load(p).elevation_datum_ft == pytest.approx(812.5)


@pytest.mark.parametrize("stored", ["", "nonsense", float("nan"), float("inf")])
def test_ui_from_config_rejects_a_corrupt_elevation_datum(stored):
    cfg = ViewerConfig()
    cfg.elevation_datum_ft = stored
    assert web.ui_from_config(cfg).elevation_datum_ft is None


def test_apply_ui_to_config_round_trips_the_elevation_datum():
    cfg = ViewerConfig()
    web.apply_ui_to_config(web.UiState(elevation_datum_ft=421.5), cfg)
    assert cfg.elevation_datum_ft == pytest.approx(421.5)
    assert web.ui_from_config(cfg).elevation_datum_ft == pytest.approx(421.5)


# ---------------------------------------------------------------------------
# 16. Live SLAM resource monitor (owner ask, 2026-07-31)
# ---------------------------------------------------------------------------

def _resource_snapshot(**kw):
    from roomscan.metrics import ResourceSnapshot
    base = dict(proc_cpu_percent=42.0, n_cores=8, proc_rss=512 * 1024 * 1024,
                ram_total=32 * 1024**3, gpu_util=None, proc_vram=None,
                vram_total=None, gpu_name=None, gpu_source="n/a",
                sys_cpu_percent=17.5, ram_used=9 * 1024**3,
                device_vram_used=1725 * 1024**2, device_vram_total=8188 * 1024**2,
                device_vram_source="nvml")
    base.update(kw)
    return ResourceSnapshot(**base)


def test_build_metrics_message_serialises_resources():
    """`resources` was hardcoded null ("Phase 1, no ResourceSampler wired")
    since Phase 1. It now carries both scopes: this process AND the box."""
    snap = MetricsSnapshot(render_fps=28.0, streams=[], link_bytes_per_s=0.0,
                           resources=_resource_snapshot())
    msg = web.build_metrics_message(snap)
    r = msg["resources"]
    assert r is not None
    assert r["proc_cpu_percent"] == pytest.approx(42.0)
    assert r["sys_cpu_percent"] == pytest.approx(17.5)
    assert r["ram_used"] == 9 * 1024**3 and r["ram_total"] == 32 * 1024**3
    assert r["device_vram_used"] == 1725 * 1024**2
    assert r["device_vram_total"] == 8188 * 1024**2
    assert r["device_vram_source"] == "nvml"
    json.dumps(msg)


def test_build_metrics_message_resources_null_without_a_sampler():
    """No sampler (every existing test, and any state built by hand) must keep
    reading exactly as before."""
    snap = MetricsSnapshot(render_fps=28.0, streams=[], link_bytes_per_s=0.0, resources=None)
    assert web.build_metrics_message(snap)["resources"] is None


def test_resources_degrade_to_null_on_a_gpu_less_box():
    """Every GPU/system field must be None-able, not zero. A 0 where a
    measurement should be reads as "plenty of headroom" — the exact trap the
    verify-the-knob lesson is about."""
    snap = MetricsSnapshot(render_fps=0.0, streams=[], link_bytes_per_s=0.0,
                           resources=_resource_snapshot(
                               sys_cpu_percent=None, ram_used=None,
                               device_vram_used=None, device_vram_total=None,
                               device_vram_source="n/a"))
    r = web.build_metrics_message(snap)["resources"]
    assert r["sys_cpu_percent"] is None and r["ram_used"] is None
    assert r["device_vram_used"] is None and r["device_vram_total"] is None
    assert r["device_vram_source"] == "n/a"
    assert r["proc_cpu_percent"] == pytest.approx(42.0)     # per-process still real
    json.dumps(r)


def test_probe_device_vram_never_raises():
    """It is a ctypes NVML call behind a bare `Nvml()`; on a box with no
    driver library it must report absence, not throw."""
    from roomscan.metrics import probe_device_vram
    used, total, name, src = probe_device_vram()
    assert src in ("nvml", "n/a")
    if src == "n/a":
        assert used is None and total is None and name is None
    else:
        assert used >= 0 and total > 0 and used <= total
        assert name is None or isinstance(name, str)


def test_resource_sampler_uses_injected_probes():
    """Both probes are injectable so this runs identically on a GPU-less box."""
    from roomscan.metrics import ResourceSampler
    s = ResourceSampler(interval=0.02,
                        gpu_probe=lambda: (11.0, 123, 456, "FakeGPU", "test"),
                        vram_probe=lambda: (700, 8000, "TestGPU 9000", "test-nvml"))
    s.start()
    try:
        deadline = time.time() + 3.0
        while s.latest() is None and time.time() < deadline:
            time.sleep(0.02)
        snap = s.latest()
    finally:
        s.stop()
    assert snap is not None, "sampler published no snapshot"
    assert snap.device_vram_used == 700 and snap.device_vram_total == 8000
    assert snap.device_vram_source == "test-nvml"
    # The per-process probe supplied a name here, so it wins; the ctypes
    # device-wide name is the fallback for a box with no pynvml (i.e. this one).
    assert snap.gpu_name == "FakeGPU"
    assert snap.sys_cpu_percent is not None and 0.0 <= snap.sys_cpu_percent <= 100.0
    assert 0 < snap.ram_used <= snap.ram_total
    assert snap.gpu_source == "test"


# --- the TSDF block gauge (BUG-035) ----------------------------------------

def test_build_slam_message_carries_the_block_gauge():
    step = _FrameStep(pose=np.eye(4), fitness=0.9, rmse=0.01, tracking_lost=False,
                      slam_ms=7.0, blocks_used=38912, blocks_capacity=40000,
                      blocks_configured=160000)
    msg = web.build_slam_message(step, [np.eye(4)], frames_integrated=1, mesh_seq=1,
                                 source_vertex_count=10)
    assert msg["blocks_used"] == 38912
    assert msg["blocks_capacity"] == 40000
    assert msg["blocks_configured"] == 160000
    json.dumps(msg)


def test_build_slam_message_block_gauge_null_before_the_first_sample():
    """`block_usage()` is a device sync on CUDA, so the mapper samples it at
    ~4 Hz, not per frame — and a worker predating the gauge omits it entirely.
    Both must be null (unknown), never 0 (an empty map)."""
    step = _FrameStep(pose=np.eye(4), fitness=0.9, rmse=0.01, tracking_lost=False, slam_ms=7.0)
    msg = web.build_slam_message(step, [np.eye(4)], frames_integrated=1, mesh_seq=1,
                                 source_vertex_count=10)
    assert msg["blocks_used"] is None
    assert msg["blocks_capacity"] is None
    assert msg["blocks_configured"] is None


def test_frame_step_block_fields_are_backwards_compatible():
    """New FrameStep fields must have defaults — remote.py and ~20 tests build
    one positionally/with keywords and none of them know about the gauge."""
    step = _FrameStep(pose=np.eye(4), fitness=0.0, rmse=0.0, tracking_lost=True, slam_ms=1.0)
    assert step.blocks_used is None and step.blocks_capacity is None
    assert step.blocks_configured is None


# ---------------------------------------------------------------------------
# 17. Auto-record on entering Live SLAM (owner ask, 2026-07-31)
# ---------------------------------------------------------------------------
#
# A live scan is unrepeatable — the same reason BUG-043 kept Live SLAM's
# one-shot Save. The invariant that needs protecting is that a MANUALLY started
# recording is never ended by a display switch.

class _RecCtrl(_FakeCtrl):
    """_FakeCtrl plus the recording surface the auto-record path drives."""

    def __init__(self, *, mode="live", has_live=True, **kw):
        super().__init__(mode=mode, **kw)
        self.has_live = has_live
        self.recording = False
        self.starts = 0
        self.stops = 0
        self._auto_recording = False
        self.session_calls = 0

    # Mirrors SessionController's real logic, which is itself tested separately
    # against a real Recorder below.
    def start_record(self):
        if self.mode != "live":
            return
        self.recording = True
        self.starts += 1
        self._auto_recording = False

    def stop_record(self):
        if not self.recording:
            return
        self.recording = False
        self.stops += 1
        self._auto_recording = False

    def start_auto_record(self):
        if self.mode != "live" or not self.has_live or self.recording:
            return False
        self.start_record()
        self._auto_recording = True
        return True

    def stop_auto_record(self):
        if not self._auto_recording:
            return False
        self.stop_record()
        return True

    def session_message(self, pos, now):
        self.session_calls += 1
        return {"type": "session", "recording": {"active": self.recording}}


def _autorec_state(ui, ctrl, *, slam_auto_record=True):
    import types
    published = []
    bus = types.SimpleNamespace(publish=published.append)
    cfg = ViewerConfig(slam_auto_record=slam_auto_record)
    return types.SimpleNamespace(config=cfg, ui_state=ui, clients=set(),
                                 controller=ctrl, bus=bus, slam_runner=None,
                                 detailed_runner=None), published


def _run_inbound(state, msg):
    """Drive one inbound message with the broadcast fan-out stubbed out."""
    async def drive():
        async def noop_text(clients, text):
            pass

        async def noop_caps(state_, ctrl_):
            pass

        orig_t, orig_c = web._broadcast_text, web._broadcast_captures
        web._broadcast_text, web._broadcast_captures = noop_text, noop_caps
        try:
            await web._handle_inbound(state, msg)
        finally:
            web._broadcast_text, web._broadcast_captures = orig_t, orig_c

    asyncio.run(drive())


def test_entering_live_slam_starts_a_recording():
    ui = web.UiState(source="live", display="point_cloud")
    ctrl = _RecCtrl(mode="live")
    state, published = _autorec_state(ui, ctrl)
    _run_inbound(state, {"type": "set_display", "display": "slam"})
    assert ctrl.recording is True and ctrl.starts == 1
    assert ctrl._auto_recording is True
    assert any("auto-recording" in line for line in published), published


def test_leaving_live_slam_stops_the_auto_recording():
    ui = web.UiState(source="live", display="point_cloud")
    ctrl = _RecCtrl(mode="live")
    state, _ = _autorec_state(ui, ctrl)
    _run_inbound(state, {"type": "set_display", "display": "slam"})
    _run_inbound(state, {"type": "set_display", "display": "point_cloud"})
    assert ctrl.recording is False and ctrl.stops == 1
    assert ctrl._auto_recording is False


def test_go_live_stops_the_auto_recording():
    ui = web.UiState(source="live", display="slam")
    ctrl = _RecCtrl(mode="live")
    state, _ = _autorec_state(ui, ctrl)
    _run_inbound(state, {"type": "set_display", "display": "slam"})
    _run_inbound(state, {"type": "go_live"})
    assert ctrl.recording is False and ctrl.stops == 1


def test_a_manual_recording_is_never_stopped_by_a_display_switch():
    """The whole reason `_auto_recording` exists. The operator pressed Record;
    entering and leaving SLAM must not touch their take."""
    ui = web.UiState(source="live", display="point_cloud")
    ctrl = _RecCtrl(mode="live")
    state, _ = _autorec_state(ui, ctrl)
    _run_inbound(state, {"type": "record", "on": True})     # manual
    assert ctrl.recording is True and ctrl._auto_recording is False

    _run_inbound(state, {"type": "set_display", "display": "slam"})
    assert ctrl.recording is True and ctrl.starts == 1      # not restarted
    assert ctrl._auto_recording is False                    # still MANUALLY owned

    _run_inbound(state, {"type": "set_display", "display": "point_cloud"})
    assert ctrl.recording is True and ctrl.stops == 0       # and NOT stopped


def test_view_source_slam_does_not_auto_record():
    """Replay SLAM is repeatable; there is nothing to preserve, and the
    controller would refuse anyway."""
    ui = web.UiState(source="view", display="point_cloud", selected_capture="take.bin")
    ctrl = _RecCtrl(mode="replay")
    state, _ = _autorec_state(ui, ctrl)
    _run_inbound(state, {"type": "set_display", "display": "slam"})
    assert ctrl.recording is False and ctrl.starts == 0


def test_auto_record_is_a_no_op_without_a_live_source():
    """A `--replay`-launched process has has_live False. That must be a no-op,
    not an error, and must not block SLAM from arming."""
    ui = web.UiState(source="live", display="point_cloud")
    ctrl = _RecCtrl(mode="live", has_live=False)
    state, published = _autorec_state(ui, ctrl)
    _run_inbound(state, {"type": "set_display", "display": "slam"})
    assert ui.display == "slam"                              # SLAM still armed
    assert ctrl.recording is False and ctrl.starts == 0
    assert not any("auto-recording" in line for line in published), published


def test_auto_record_can_be_disabled_by_config():
    ui = web.UiState(source="live", display="point_cloud")
    ctrl = _RecCtrl(mode="live")
    state, _ = _autorec_state(ui, ctrl, slam_auto_record=False)
    _run_inbound(state, {"type": "set_display", "display": "slam"})
    assert ui.display == "slam"
    assert ctrl.recording is False and ctrl.starts == 0


def test_set_mode_alias_also_auto_records():
    """`set_mode` is still how the rig_* MCP tools enter SLAM, and a scan
    started that way is just as unrepeatable."""
    ui = web.UiState(source="live", display="point_cloud")
    ctrl = _RecCtrl(mode="live")
    state, _ = _autorec_state(ui, ctrl)
    _run_inbound(state, {"type": "set_mode", "mode": "slam"})
    assert ctrl.recording is True and ctrl._auto_recording is True
    _run_inbound(state, {"type": "set_mode", "mode": "realtime"})
    assert ctrl.recording is False


def test_controller_auto_record_flags_on_the_real_controller(tmp_path):
    """The real SessionController's ownership flag, not the fake's."""
    class FakeLive:
        def read(self):
            time.sleep(0.02)
            return b""

        def write(self, d):
            pass

        def close(self):
            pass

    outdir = tmp_path / "caps"
    ctrl, _ = _make_controller(tmp_path, live_source=FakeLive(), captures_dir=outdir)
    try:
        assert ctrl.start_auto_record() is True
        assert ctrl.recorder.active and ctrl._auto_recording is True
        # A second call while one is running must not start another.
        assert ctrl.start_auto_record() is False
        assert ctrl.stop_auto_record() is True
        assert not ctrl.recorder.active and ctrl._auto_recording is False
        assert ctrl.stop_auto_record() is False           # nothing to stop

        # A MANUAL take is not auto-owned, so stop_auto_record leaves it alone.
        ctrl.start_record()
        assert ctrl.recorder.active and ctrl._auto_recording is False
        assert ctrl.stop_auto_record() is False
        assert ctrl.recorder.active
        ctrl.stop_record()
    finally:
        ctrl.close()


def test_controller_auto_record_refused_in_replay(tmp_path):
    cap = tmp_path / "a.bin"
    _make_depth_capture_flat(cap, n_frames=5, base=1000.0)
    ctrl, _ = _make_controller(tmp_path, replay_path=str(cap))
    try:
        assert ctrl.start_auto_record() is False
        assert not ctrl.recorder.active
    finally:
        ctrl.close()


def test_device_vram_name_is_the_fallback_when_pynvml_is_absent():
    """`pynvml` is not installed here, so the per-process probe returns
    `gpu_name=None` and `gpu_source="n/a"`. Without the ctypes fallback the
    Resources card could say a GPU exists but never WHICH card's ceiling you
    are approaching, and `gpu_name` would be a permanently-null wire field."""
    from roomscan.metrics import ResourceSampler
    s = ResourceSampler(interval=0.02,
                        gpu_probe=lambda: (None, None, None, None, "n/a"),
                        vram_probe=lambda: (700, 8000, "TestGPU 9000", "test-nvml"))
    s.start()
    try:
        deadline = time.time() + 3.0
        while s.latest() is None and time.time() < deadline:
            time.sleep(0.02)
        snap = s.latest()
    finally:
        s.stop()
    assert snap is not None
    assert snap.gpu_source == "n/a"
    assert snap.gpu_name == "TestGPU 9000"


# =============================================================================
# §12 -- the View page's capture file browser (owner ask, 2026-07-31)
# =============================================================================
#
# Server half: `list_captures`'s new fields, `_sidecar_summary`, the `/thumb`
# route, `SessionController.rename_capture` / `delete_capture`, and the three
# inbound handlers (`rename_capture` with `from`, `delete_captures`,
# `set_browser`). Every new inbound type gets a happy path, a reject/no-op, and
# an adversarial/malformed-input case.

def _cap_with_quat(path: Path, n_frames: int = 12, w: int = 8, h: int = 6) -> None:
    """DEPTH_ZF32 + IMU_QUAT capture -- enough for `has_stream_9` and a thumbnail."""
    out = bytearray()
    for i in range(n_frames):
        q = struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, i,
                                      i * 35000, 0, 0, len(q)), q)
        depth = np.full((h, w), 1000.0 + 5.0 * i, dtype=np.float32)
        payload = depth.astype("<f4").tobytes()
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, i + 1,
                                      i * 35000, w, h, len(payload)), payload)
    path.write_bytes(bytes(out))


def _write_sidecar(results_dir: Path, capture: Path, *, stats: dict) -> None:
    from roomscan.slam.detailed import build_manifest, sidecar_paths
    from roomscan.slam.config import DetailedSlamPreset
    results_dir.mkdir(parents=True, exist_ok=True)
    paths = sidecar_paths(capture, results_dir)
    paths["ply"].write_bytes(b"ply\n")
    paths["tum"].write_text("0 0 0 0 0 0 0 1\n")
    manifest = build_manifest(capture, DetailedSlamPreset(), stats=stats,
                              estimate={"frames": 1, "seconds": 1.0})
    paths["manifest"].write_text(json.dumps(manifest), encoding="utf-8")


# --- list_captures: additive fields, existing keys untouched ----------------

def test_list_captures_keeps_its_existing_keys_and_gains_the_browser_ones(tmp_path):
    """`list_captures(dir)` must stay call-compatible: `results_dir`/`thumbs_dir`
    are keyword-defaulted precisely so every existing caller and test is
    untouched by §12."""
    cap = tmp_path / "a.bin"
    _cap_with_quat(cap)
    (items,) = web.list_captures(tmp_path, results_dir=tmp_path / "results",
                                 thumbs_dir=tmp_path / "thumbs")
    for key in ("name", "bytes", "mtime", "frames", "has_stream_9", "duration_s", "timestamped"):
        assert key in items, key
    assert items["has_thumb"] is False
    assert items["slam"] is None


def test_list_captures_reports_the_reconstruction_where_one_exists(tmp_path):
    cap = tmp_path / "a.bin"
    _cap_with_quat(cap)
    results = tmp_path / "results"
    _write_sidecar(results, cap, stats={"frames": 900, "path_m": 23.9, "gap_m": 0.74,
                                        "area_m2": 31.2})
    (item,) = web.list_captures(tmp_path, results_dir=results, thumbs_dir=tmp_path / "t")
    assert item["slam"] == {"exists": True, "current": True, "frames": 900,
                            "path_m": 23.9, "gap_m": 0.74, "area_m2": 31.2}


def test_sidecar_summary_reads_a_pre_area_manifest_as_none(tmp_path):
    """`area_m2` is additive: every manifest written before 2026-07-31 lacks it
    and must read as None (the tile renders an em dash), NOT as 0 -- a
    reconstruction that covered nothing and one that was never measured are
    different facts."""
    cap = tmp_path / "a.bin"
    _cap_with_quat(cap)
    results = tmp_path / "results"
    _write_sidecar(results, cap, stats={"frames": 10, "path_m": 1.0, "gap_m": 0.1})
    summary = web._sidecar_summary(cap, results)
    assert summary["area_m2"] is None
    assert summary["path_m"] == 1.0


def test_sidecar_summary_cache_is_keyed_on_the_manifest_not_the_capture(tmp_path):
    """The sidecar changes independently of the capture (a rebuild rewrites it
    while the capture is byte-identical), so a capture-keyed cache would serve a
    stale summary forever."""
    cap = tmp_path / "a.bin"
    _cap_with_quat(cap)
    results = tmp_path / "results"
    _write_sidecar(results, cap, stats={"frames": 1, "path_m": 1.0, "gap_m": 0.0, "area_m2": 1.0})
    assert web._sidecar_summary(cap, results)["area_m2"] == 1.0
    _write_sidecar(results, cap, stats={"frames": 2, "path_m": 2.0, "gap_m": 0.0, "area_m2": 9.0})
    assert web._sidecar_summary(cap, results)["area_m2"] == 9.0


def test_forget_capture_caches_drops_both_caches(tmp_path):
    """Both caches are unbounded and keyed on (path, size, mtime_ns): without
    this a long-lived server orphans one dead entry per rename/delete forever."""
    cap = tmp_path / "a.bin"
    _cap_with_quat(cap)
    results = tmp_path / "results"
    _write_sidecar(results, cap, stats={"frames": 1, "path_m": 1.0, "gap_m": 0.0})
    web._capture_info(cap)
    web._sidecar_summary(cap, results)
    assert any(k[0] == str(cap) for k in web._CAPTURE_INFO_CACHE)
    assert any(Path(k[0]).name == "a.slam.json" for k in web._SIDECAR_SUMMARY_CACHE)
    web._forget_capture_caches(cap)
    assert not any(k[0] == str(cap) for k in web._CAPTURE_INFO_CACHE)
    assert not any(Path(k[0]).name == "a.slam.json" for k in web._SIDECAR_SUMMARY_CACHE)


# --- GET /thumb/{name} ------------------------------------------------------

def _thumb_client(tmp_path, monkeypatch):
    import types
    from fastapi.testclient import TestClient
    monkeypatch.setattr(web.thumbs_mod, "THUMBS_DIR", str(tmp_path / "thumbs"))
    monkeypatch.setattr(web.app.state, "controller",
                        types.SimpleNamespace(captures_dir=str(tmp_path)), raising=False)
    return TestClient(web.app)


def test_thumb_route_generates_and_serves_a_png(tmp_path, monkeypatch):
    cap = tmp_path / "a.bin"
    _cap_with_quat(cap, n_frames=20)
    client = _thumb_client(tmp_path, monkeypatch)
    r = client.get("/thumb/a.bin")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/png"
    assert r.content[:8] == b"\x89PNG\r\n\x1a\n"
    # Safe to mark immutable: the capture's identity is in the cache filename
    # AND the client's ?v=, so a rewritten capture is a different URL.
    assert "immutable" in r.headers["cache-control"]


@pytest.mark.parametrize("name", ["../../etc/passwd", "..%2Fsecret.bin", "sub/dir.bin",
                                  "nope.bin", "a.txt", "", "a.bin.png"])
def test_thumb_route_never_500s_on_a_hostile_or_unknown_name(tmp_path, monkeypatch, name):
    """`sanitize_capture_name` is the WHOLE security surface, same as
    `load_capture`. Everything else is 404 + no-store -- never a 500, never a
    cached failure."""
    _cap_with_quat(tmp_path / "a.bin")
    client = _thumb_client(tmp_path, monkeypatch)
    r = client.get("/thumb/" + name)
    assert r.status_code == 404
    if r.headers.get("cache-control"):
        assert r.headers["cache-control"] == "no-store"


def test_thumb_route_404s_a_capture_it_cannot_render(tmp_path, monkeypatch):
    (tmp_path / "junk.bin").write_bytes(b"not a capture")
    client = _thumb_client(tmp_path, monkeypatch)
    r = client.get("/thumb/junk.bin")
    assert r.status_code == 404
    assert r.headers["cache-control"] == "no-store"


# --- SessionController.rename_capture ---------------------------------------

def test_rename_capture_moves_the_sidecar_and_patches_the_manifest(tmp_path):
    """The manifest patch is the load-bearing half: `sidecar_status` compares
    `manifest["capture"]` against `capture_identity(capture)`, whose `name` is
    the basename -- so a moved-but-unpatched manifest reports `stale` forever
    and the UI offers a rebuild that would recompute a byte-identical result."""
    caps, results, thumbs_dir = tmp_path / "caps", tmp_path / "results", tmp_path / "thumbs"
    caps.mkdir()
    cap = caps / "old.bin"
    _cap_with_quat(cap)
    _write_sidecar(results, cap, stats={"frames": 5, "path_m": 3.0, "gap_m": 0.1, "area_m2": 7.0})
    thumbs_dir.mkdir()
    (thumbs_dir / f"old__256_{cap.stat().st_mtime_ns}.png").write_bytes(b"png")

    import roomscan.thumbs as thumbs_mod
    old_dir = thumbs_mod.THUMBS_DIR
    thumbs_mod.THUMBS_DIR = str(thumbs_dir)
    try:
        ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
        ctrl.results_dir = str(results)
        try:
            assert ctrl.rename_capture("old.bin", "new name") == "new name.bin"
        finally:
            ctrl.close()
    finally:
        thumbs_mod.THUMBS_DIR = old_dir

    assert not (caps / "old.bin").exists() and (caps / "new name.bin").is_file()
    assert (results / "new name.ply").is_file() and (results / "new name.tum").is_file()
    manifest = json.loads((results / "new name.slam.json").read_text())
    assert manifest["capture"]["name"] == "new name.bin"
    from roomscan.slam.detailed import sidecar_status
    assert sidecar_status(caps / "new name.bin", results)["current"] is True
    assert list(thumbs_dir.glob("new name__*.png"))


@pytest.mark.parametrize("bad", ["../escape.bin", "sub/dir.bin", "missing.bin",
                                 "notabin.txt", "", None, 17, ["a.bin"]])
def test_rename_capture_rejects_a_hostile_source(tmp_path, bad):
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "real.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    try:
        assert ctrl.rename_capture(bad, "whatever") is None
        assert (caps / "real.bin").is_file()      # nothing collateral happened
    finally:
        ctrl.close()


@pytest.mark.parametrize("bad_target", ["../escape", "sub/dir", "", "   ", None, 5])
def test_rename_capture_rejects_a_hostile_target(tmp_path, bad_target):
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "real.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    try:
        assert ctrl.rename_capture("real.bin", bad_target) is None
        assert (caps / "real.bin").is_file()
    finally:
        ctrl.close()


def test_rename_capture_refuses_the_file_being_recorded(tmp_path):
    """The `Recorder` holds an open write handle and `_last_recorded_name` would
    go stale -- this is genuinely wrong rather than merely awkward, so it is
    refused rather than silently no-op'd."""
    caps = tmp_path / "caps"
    caps.mkdir()

    class FakeLive:
        def read(self): return b""
        def write(self, d): pass
        def close(self): pass

    ctrl, _ = _make_controller(tmp_path, live_source=FakeLive(), captures_dir=caps)
    try:
        ctrl.start_record()
        active = Path(ctrl.recorder.path).name
        assert ctrl.rename_capture(active, "sneaky") is None
        assert (caps / active).is_file()
        ctrl.stop_record()
    finally:
        ctrl.close()


def test_rename_capture_handles_the_capture_being_replayed(tmp_path):
    """POSIX `rename(2)` keeps `FileSource`'s open fd valid, so the replay does
    not even hiccup -- but `replay_path` must follow, or the session label and
    any later seek address a file that is no longer there."""
    caps = tmp_path / "caps"
    caps.mkdir()
    cap = caps / "playing.bin"
    _make_depth_capture_flat(cap, n_frames=5, base=1000.0)
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps, replay_path=str(cap))
    try:
        assert ctrl.rename_capture("playing.bin", "renamed") == "renamed.bin"
        assert Path(ctrl.replay_path).name == "renamed.bin"
        assert ctrl.session_message(None, time.time())["playback"]["capture_name"] == "renamed.bin"
    finally:
        ctrl.close()


# --- inbound: rename_capture with the optional `from` -----------------------

def test_inbound_rename_capture_without_from_is_unchanged(tmp_path):
    """Absent `from` MUST be byte-identical to the pre-§12 behaviour: the
    post-recording modal and the `rig_*` MCP tools depend on "rename the take
    that just stopped"."""
    import types
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "web_20260101_000000.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    ctrl._last_recorded_name = "web_20260101_000000.bin"
    state = types.SimpleNamespace(controller=ctrl, clients=set(), bus=_LogBus(),
                                  ui_state=web.UiState(), detailed_runner=None)
    asyncio.run(web._handle_inbound(state, {"type": "rename_capture", "name": "last take"}))
    assert (caps / "last take.bin").is_file()
    assert ctrl._last_recorded_name == "last take.bin"
    ctrl.close()


def test_inbound_rename_capture_with_from_renames_that_file(tmp_path):
    import types
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "target.bin")
    _cap_with_quat(caps / "web_recent.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    ctrl._last_recorded_name = "web_recent.bin"
    state = types.SimpleNamespace(controller=ctrl, clients=set(), bus=_LogBus(),
                                  ui_state=web.UiState(), detailed_runner=None)
    asyncio.run(web._handle_inbound(state, {"type": "rename_capture",
                                            "from": "target.bin", "name": "renamed"}))
    assert (caps / "renamed.bin").is_file() and not (caps / "target.bin").exists()
    assert (caps / "web_recent.bin").is_file()    # the last take was NOT touched
    assert ctrl._last_recorded_name == "web_recent.bin"
    ctrl.close()


@pytest.mark.parametrize("src", ["../escape.bin", "sub/x.bin", "missing.bin", 42, ["a"]])
def test_inbound_rename_capture_with_a_hostile_from_is_a_no_op(tmp_path, src):
    import types
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "safe.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    bus = _LogBus()
    handle = bus.subscribe()
    state = types.SimpleNamespace(controller=ctrl, clients=set(), bus=bus,
                                  ui_state=web.UiState(), detailed_runner=None)
    asyncio.run(web._handle_inbound(state, {"type": "rename_capture",
                                            "from": src, "name": "x"}))
    assert sorted(p.name for p in caps.glob("*.bin")) == ["safe.bin"]
    assert any("rename ->" in line for line in bus.drain(handle))
    ctrl.close()


# --- inbound: delete_captures ----------------------------------------------

def _delete_state(tmp_path, ctrl):
    import types
    return types.SimpleNamespace(controller=ctrl, clients=set(), bus=_LogBus(),
                                 ui_state=web.UiState(), detailed_runner=None,
                                 slam_runner=None, config=None)


def test_delete_captures_removes_the_files_and_their_sidecars(tmp_path):
    caps, results = tmp_path / "caps", tmp_path / "results"
    caps.mkdir()
    for n in ("a.bin", "b.bin", "keep.bin"):
        _cap_with_quat(caps / n)
    _write_sidecar(results, caps / "a.bin", stats={"frames": 1, "path_m": 1.0, "gap_m": 0.0})
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    ctrl.results_dir = str(results)
    state = _delete_state(tmp_path, ctrl)
    res = asyncio.run(web._handle_delete_captures(
        state, ctrl, {"type": "delete_captures", "names": ["a.bin", "b.bin"]}))
    assert sorted(d["name"] for d in res["deleted"]) == ["a.bin", "b.bin"]
    assert res["refused"] == []
    assert res["bytes"] > 0
    assert sorted(p.name for p in caps.glob("*.bin")) == ["keep.bin"]
    assert not (results / "a.ply").exists()      # an orphan .ply would keep
    assert not (results / "a.slam.json").exists()  # showing in the Saved list
    ctrl.close()


def test_delete_captures_can_keep_the_sidecars(tmp_path):
    caps, results = tmp_path / "caps", tmp_path / "results"
    caps.mkdir()
    _cap_with_quat(caps / "a.bin")
    _write_sidecar(results, caps / "a.bin", stats={"frames": 1, "path_m": 1.0, "gap_m": 0.0})
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    ctrl.results_dir = str(results)
    state = _delete_state(tmp_path, ctrl)
    asyncio.run(web._handle_delete_captures(
        state, ctrl, {"names": ["a.bin"], "sidecars": False}))
    assert not (caps / "a.bin").exists()
    assert (results / "a.ply").is_file() and (results / "a.slam.json").is_file()
    ctrl.close()


def test_delete_captures_refuses_the_file_being_recorded(tmp_path):
    """The `Recorder` holds an open write handle: deleting it deletes a file
    still being appended to."""
    caps = tmp_path / "caps"
    caps.mkdir()

    class FakeLive:
        def read(self): return b""
        def write(self, d): pass
        def close(self): pass

    ctrl, _ = _make_controller(tmp_path, live_source=FakeLive(), captures_dir=caps)
    try:
        ctrl.start_record()
        active = Path(ctrl.recorder.path).name
        state = _delete_state(tmp_path, ctrl)
        res = asyncio.run(web._handle_delete_captures(state, ctrl, {"names": [active]}))
        assert res["deleted"] == []
        assert res["refused"] == [{"name": active, "reason": "currently recording"}]
        assert (caps / active).is_file()
        ctrl.stop_record()
    finally:
        ctrl.close()


def test_delete_captures_refuses_the_replaying_file_when_there_is_no_live_source(tmp_path):
    """Unlinking under a running replay reader gives a broken stream, not a
    clean state -- and a `--replay`-launched process has nowhere to switch to."""
    caps = tmp_path / "caps"
    caps.mkdir()
    cap = caps / "playing.bin"
    _make_depth_capture_flat(cap, n_frames=5, base=1000.0)
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps, replay_path=str(cap))
    try:
        assert ctrl.has_live is False
        state = _delete_state(tmp_path, ctrl)
        res = asyncio.run(web._handle_delete_captures(state, ctrl, {"names": ["playing.bin"]}))
        assert res["deleted"] == []
        assert res["refused"][0]["name"] == "playing.bin"
        assert "no live source" in res["refused"][0]["reason"]
        assert cap.is_file()
    finally:
        ctrl.close()


def test_delete_captures_switches_away_from_the_replaying_file_first(tmp_path):
    caps = tmp_path / "caps"
    caps.mkdir()
    cap = caps / "playing.bin"
    _make_depth_capture_flat(cap, n_frames=5, base=1000.0)

    class FakeLive:
        def read(self): return b""
        def write(self, d): pass
        def close(self): pass

    ctrl, _ = _make_controller(tmp_path, live_source=FakeLive(), captures_dir=caps,
                               replay_path=str(cap))
    try:
        state = _delete_state(tmp_path, ctrl)
        state.ui_state.source = "view"
        state.ui_state.selected_capture = "playing.bin"
        res = asyncio.run(web._handle_delete_captures(state, ctrl, {"names": ["playing.bin"]}))
        assert [d["name"] for d in res["deleted"]] == ["playing.bin"]
        assert not cap.exists()
        assert ctrl.mode == "live"
        assert state.ui_state.source == "live"
        assert state.ui_state.selected_capture is None
        assert state.ui_state.display == "point_cloud"
    finally:
        ctrl.close()


@pytest.mark.parametrize("names", ["a.bin", None, 5, {"a": 1}])
def test_delete_captures_rejects_a_non_list_names(tmp_path, names):
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "a.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    state = _delete_state(tmp_path, ctrl)
    res = asyncio.run(web._handle_delete_captures(state, ctrl, {"names": names}))
    assert res == {"deleted": [], "refused": [], "bytes": 0}
    assert (caps / "a.bin").is_file()
    ctrl.close()


@pytest.mark.parametrize("hostile", ["../../etc/passwd", "..", "sub/dir.bin",
                                     "a.txt", "", None, 3, ["nested"]])
def test_delete_captures_drops_hostile_names_without_touching_anything(tmp_path, hostile):
    """An unresolvable entry is logged and DROPPED, never fatal to the rest of
    the batch -- but it is reported as refused, because a batch delete that
    silently skips a file is worse than one that refuses it loudly."""
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "real.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    state = _delete_state(tmp_path, ctrl)
    res = asyncio.run(web._handle_delete_captures(state, ctrl, {"names": [hostile]}))
    assert res["deleted"] == []
    assert len(res["refused"]) == 1 and res["refused"][0]["reason"] == "not found"
    assert (caps / "real.bin").is_file()
    ctrl.close()


def test_delete_captures_mixed_batch_reports_what_actually_happened(tmp_path):
    """"The result must report what was ACTUALLY deleted, not what was
    requested" -- the whole point of the refusal list."""
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "gone.bin")
    _cap_with_quat(caps / "stays.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    state = _delete_state(tmp_path, ctrl)
    res = asyncio.run(web._handle_delete_captures(
        state, ctrl, {"names": ["gone.bin", "../evil.bin", "ghost.bin"]}))
    assert [d["name"] for d in res["deleted"]] == ["gone.bin"]
    assert {r["name"] for r in res["refused"]} == {"../evil.bin", "ghost.bin"}
    assert (caps / "stays.bin").is_file()
    ctrl.close()


def test_delete_captures_caps_the_batch(tmp_path):
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "a.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    state = _delete_state(tmp_path, ctrl)
    res = asyncio.run(web._handle_delete_captures(
        state, ctrl, {"names": ["a.bin"] * (web._MAX_DELETE_BATCH + 1)}))
    assert res == {"deleted": [], "refused": [], "bytes": 0}
    assert (caps / "a.bin").is_file()
    ctrl.close()


def test_delete_captures_is_gated_on_a_controller(tmp_path):
    """The whole record/load_capture family is gated this way; without it a
    `delete_captures` before the controller is attached would raise inside the
    socket handler."""
    import types
    state = types.SimpleNamespace(controller=None, clients=set(), bus=_LogBus(),
                                  ui_state=web.UiState(), config=None)
    asyncio.run(web._handle_inbound(state, {"type": "delete_captures", "names": ["a.bin"]}))


# --- inbound: set_browser ---------------------------------------------------

def test_set_browser_persists(tmp_path):
    import types
    import roomscan.config as config_mod
    p = tmp_path / "roomscan.toml"
    cfg = ViewerConfig()
    state = types.SimpleNamespace(config=cfg, ui_state=web.UiState(), clients=set(),
                                  controller=None, detailed_runner=None)
    orig = config_mod.config_path
    config_mod.config_path = lambda: p
    try:
        asyncio.run(web._handle_inbound(state, {"type": "set_browser", "sort": "size",
                                                "view": "list", "thumbs": False}))
    finally:
        config_mod.config_path = orig
    assert (state.ui_state.browser_sort, state.ui_state.browser_view,
            state.ui_state.browser_thumbs) == ("size", "list", False)
    loaded = ViewerConfig.load(p)
    assert loaded.web_browser_sort == "size"
    assert loaded.web_browser_view == "list"
    assert loaded.web_browser_thumbs is False
    assert web.ui_from_config(loaded).browser_sort == "size"


@pytest.mark.parametrize("msg", [
    {"sort": "bogus"}, {"view": "carousel"}, {"thumbs": "yes"}, {"thumbs": 1},
    {"sort": "name", "view": "carousel"},         # valid + invalid: NO partial mutation
])
def test_set_browser_rejects_bad_values_without_partial_mutation(msg):
    import types
    ui = web.UiState()
    state = types.SimpleNamespace(config=None, ui_state=ui, clients=set(),
                                  controller=None, detailed_runner=None)
    asyncio.run(web._handle_inbound(state, {"type": "set_browser", **msg}))
    assert (ui.browser_sort, ui.browser_view, ui.browser_thumbs) == ("recent", "grid", True)


def test_state_message_carries_the_browser_prefs():
    ui = web.UiState(browser_sort="duration", browser_view="list", browser_thumbs=False)
    m = web._state_message(ui)
    assert m["browser_sort"] == "duration"
    assert m["browser_view"] == "list"
    assert m["browser_thumbs"] is False


def test_ui_from_config_ignores_a_corrupt_browser_pref():
    cfg = ViewerConfig()
    cfg.web_browser_sort = "sideways"
    cfg.web_browser_view = "hologram"
    ui = web.ui_from_config(cfg)
    assert (ui.browser_sort, ui.browser_view) == ("recent", "grid")


def test_set_source_view_with_nothing_selected_opens_the_browser(tmp_path):
    """View with no capture selected is a legitimate state -- it is the capture
    BROWSER (§12).

    Before the browser existed this returned early with "select a capture
    first", which became a dead end the moment the library moved onto the View
    page: the only way to select a capture is a card that only renders when
    `source == "view"`. The reader is deliberately NOT swapped (still live,
    `is_replay` false, so the Playback card stays hidden) until a `load_capture`
    from the browser does it.
    """
    import types
    caps = tmp_path / "caps"
    caps.mkdir()
    _cap_with_quat(caps / "a.bin")
    ctrl, _ = _make_controller(tmp_path, captures_dir=caps)
    bus = _LogBus()
    handle = bus.subscribe()
    state = types.SimpleNamespace(controller=ctrl, clients=set(), bus=bus, config=None,
                                  ui_state=web.UiState(), detailed_runner=None,
                                  slam_runner=None)
    try:
        asyncio.run(web._handle_inbound(state, {"type": "set_source", "source": "view"}))
        assert state.ui_state.source == "view"
        assert state.ui_state.selected_capture is None
        assert ctrl.mode == "live"                 # reader untouched
        assert any("pick a capture" in line for line in bus.drain(handle))
    finally:
        ctrl.close()


# --- BUG-060: the live-SLAM 1 Hz viewport ------------------------------------

def test_uvicorn_runs_with_permessage_deflate_disabled():
    """uvicorn defaults `ws_per_message_deflate` ON, and `_broadcast_bytes`
    awaits `send_bytes` PER CLIENT -- so each binary payload is deflated once
    per connected tab, on the event loop. Measured at +130 ms of whole-process
    freeze per client on a 3.2 MB SLAM mesh, for an 80% compression ratio.
    A source guard, because the cost is invisible until someone profiles it."""
    src = pathlib.Path(web.__file__).read_text()
    call = src[src.index("uvicorn.run(app"):]
    call = call[:call.index(")") + 1]
    assert "ws_per_message_deflate=False" in call, call


def test_slam_config_carries_a_live_mesh_byte_budget():
    from roomscan.slam.config import SlamConfig
    assert SlamConfig().live_mesh_bytes_per_s == 12_000_000.0


class _ConsumeOncePrep:
    """MeshPrep's real contract: `latest()` yields a packet once per submit."""
    def __init__(self): self.subs = []; self._out = None
    def submit(self, mesh, *, mesh_seq, glow_origin, wall_mode):
        self.subs.append(mesh_seq)
        self._out = _synthetic_mesh_packet(mesh_seq=mesh_seq)
    def latest(self):
        out, self._out = self._out, None
        return out
    def stop(self): pass


def _governed_runner(rate):
    r = web.SlamRunner(bus=LogBus())
    prep = _ConsumeOncePrep()
    with r._lock:
        r._active = True
        r._worker = _FakeWorker()
        r._meshprep = prep
    r._mesh_bytes_per_s = rate
    return r, prep


def test_slamrunner_governor_spaces_publishes_by_payload_size():
    """BUG-060: the map is re-sent whole and grows without bound (3.2 MB at 63k
    verts, 31 MB at 611k), so cadence must fall as size rises or the link
    saturates and the broadcaster stalls on socket backpressure."""
    r, prep = _governed_runner(1_000_000.0)      # 1 MB/s, so the maths is readable
    _msg, first = r.poll("split")
    assert first is not None                     # first mesh publishes immediately
    hold_s = len(first) / 1_000_000.0
    assert r._next_mesh_at >= time.monotonic() + hold_s - 0.05

    r._worker._mesh = object()                   # a NEW worker mesh is available
    # The tiny synthetic payload's real hold window is a few ms, which wall
    # time can consume between two polls on a loaded machine; the spacing
    # arithmetic is already asserted above, so pin the clock inside the
    # window to make the withhold check deterministic.
    r._next_mesh_at = time.monotonic() + 60.0
    assert r.poll("split")[1] is None            # ...but the governor withholds it
    assert prep.subs == [1]                      # nothing was even prepped

    r._next_mesh_at = 0.0                        # pretend the interval elapsed
    assert r.poll("split")[1] is not None
    assert prep.subs == [1, 2]


def test_slamrunner_governor_off_when_rate_is_zero():
    """A zero/absent budget must mean "never withhold", not "divide by zero"."""
    r, prep = _governed_runner(0.0)
    assert r.poll("split")[1] is not None
    r._worker._mesh = object()
    assert r.poll("split")[1] is not None
    assert prep.subs == [1, 2]


def test_slamrunner_governor_resets_on_a_map_teardown():
    """A source swap starts a fresh map; its first mesh must not wait out the
    old map's (possibly multi-second) interval."""
    r = web.SlamRunner(bus=LogBus())
    r._next_mesh_at = time.monotonic() + 999.0
    r.reset()
    assert r._next_mesh_at == 0.0


def test_packed_or_pack_prefers_the_bytes_meshprep_already_made():
    """Packing is O(map) numpy work; MeshPrep does it on its own thread now."""
    prepacked = SimpleNamespace(packed=b"from-the-prep-thread")
    assert web._packed_or_pack(prepacked) == b"from-the-prep-thread"
    assert web._packed_or_pack(None) is None


def _feed_state(display, *, quat=(1.0, 0.0, 0.0, 0.0), env=None):
    submitted = []
    slam = SimpleNamespace(submit=lambda *a, **k: submitted.append((a, k)))
    state = SimpleNamespace(
        ui_state=SimpleNamespace(display=display),
        slam_runner=slam,
        sensor_state=SimpleNamespace(fused_quat=lambda: quat,
                                     latest_env=lambda: env))
    return state, submitted


def test_slam_feed_submits_every_frame_in_slam_display():
    state, submitted = _feed_state("slam", env=SimpleNamespace(pressure_pa=98000.0))
    feed = web.make_slam_feed(state)
    outputs = {"depth": np.zeros((4, 4), np.float32),
               "reflectance": np.ones((4, 4), np.float32),
               "confidence": np.ones((4, 4), np.float32)}
    for _ in range(3):
        feed(SimpleNamespace(seq=1), outputs)
    assert len(submitted) == 3
    (depth, quat, pressure), kw = submitted[0]
    assert depth is outputs["depth"] and quat == (1.0, 0.0, 0.0, 0.0)
    assert pressure == 98000.0
    assert kw["reflectance"] is outputs["reflectance"]
    assert kw["confidence"] is outputs["confidence"]


def test_slam_feed_is_silent_outside_slam_display():
    state, submitted = _feed_state("point_cloud")
    web.make_slam_feed(state)(SimpleNamespace(seq=1),
                              {"depth": np.zeros((4, 4), np.float32)})
    assert submitted == []


def test_slam_feed_tolerates_a_frame_with_no_depth():
    state, submitted = _feed_state("slam")
    web.make_slam_feed(state)(SimpleNamespace(seq=1), {"reflectance": None})
    assert submitted == []


def test_broadcaster_no_longer_feeds_slam():
    """The feed moved to the reader thread. If it comes back to the broadcaster,
    reconstruction rate silently becomes a function of event-loop health again
    -- which is what held Live SLAM at 5.0 fps against a 30.3 Hz stream."""
    src = pathlib.Path(web.__file__).read_text()
    body = src[src.index("async def _broadcaster()"):]
    assert "slam.submit(" not in body
    assert "slam.poll(" in body          # the poll side legitimately stays


def test_slamrunner_first_submit_does_not_block_the_caller(_fake_slam, monkeypatch):
    """`submit` runs on the reader thread. Building inline there stalls the
    stream long enough to overflow the UDP socket, so the build is kicked onto
    its own thread and the caller returns immediately (BUG-060)."""
    import roomscan.slam.backend as backend
    gate = threading.Event()
    real = backend.make_slam_worker

    def _slow(*a, **k):
        gate.wait(5.0)                       # a build that takes "forever"
        return real(*a, **k)

    monkeypatch.setattr(backend, "make_slam_worker", _slow)
    r = web.SlamRunner(bus=LogBus())
    r.set_active(True)
    t0 = time.monotonic()
    r.submit(np.zeros((6, 8), np.float32), (1, 0, 0, 0), None)
    elapsed = time.monotonic() - t0
    assert elapsed < 0.5, f"submit blocked {elapsed:.2f}s on the build"
    assert r.poll("split") == (None, None)   # nothing published until it lands
    gate.set()
    _await_build(r, _fake_slam)


def test_slamrunner_teardown_during_build_discards_the_stale_worker(_fake_slam, monkeypatch):
    """A source swap mid-construction must not adopt the pipeline it orphaned."""
    import roomscan.slam.backend as backend
    gate = threading.Event()
    real = backend.make_slam_worker

    def _slow(*a, **k):
        gate.wait(5.0)
        return real(*a, **k)

    monkeypatch.setattr(backend, "make_slam_worker", _slow)
    r = web.SlamRunner(bus=LogBus())
    r.set_active(True)
    r.submit(np.zeros((6, 8), np.float32), (1, 0, 0, 0), None)
    r.reset()                                # swap lands while the build is in flight
    gate.set()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and "worker" not in _fake_slam:
        time.sleep(0.005)
    time.sleep(0.1)                          # let _build_async finish installing/dropping
    with r._lock:
        assert r._worker is None             # the orphan was NOT installed
    assert _fake_slam["worker"].stopped      # ...and was stopped rather than leaked


def test_slamrunner_poll_reports_the_resolved_device(_fake_slam):
    """BUG-061 Part B: the SLAM compute device was only visible in a log line
    (`[slam] worker started on {device} ...`) -- it now rides in the `slam`
    message so the HUD can show it without trusting a log grep."""
    r = web.SlamRunner(bus=LogBus())
    assert r._device is None
    r.set_active(True)
    r.submit(np.zeros((6, 8), np.float32), (1, 0, 0, 0), None)
    _await_build(r, _fake_slam)
    assert isinstance(r._device, str) and r._device
    msg, _mesh = r.poll("split")
    assert msg["device"] == r._device


# ---------------------------------------------------------------------------
# BUG-061 Part A -- credit-gated `/ws-mesh` mesh transport
# ---------------------------------------------------------------------------
#
# No real backpressure exists on `/ws`: `send_bytes` on an unbounded transport
# buffer returns immediately no matter how big the payload or how slow the
# link, so the old open-loop broadcast (resend the whole map every tick MESH
# was ready) piled MESH bytes ahead of the 30 Hz `slam` pose on the same
# socket -- seconds of head-of-line blocking on Wi-Fi, which is what the
# owner saw as a 15 s display lag. `/ws-mesh` fixes it with one credit in
# flight per client, latest-wins by identity (never `mesh_seq`, which resets
# to 0 on `_reset_slam`).

class _FakeMeshWs:
    """Stand-in for a `/ws-mesh` WebSocket -- just records what it was sent."""
    def __init__(self, fail: bool = False):
        self.sent: list[bytes] = []
        self._fail = fail
        self.closed = False

    async def send_bytes(self, data: bytes) -> None:
        if self._fail:
            raise RuntimeError("send failed")
        self.sent.append(data)

    async def close(self) -> None:
        self.closed = True


def _mesh_bytes(seq: int) -> bytes:
    """Minimal bytes shaped enough for `_cache_latest_mesh`'s
    `struct.unpack_from("<I", b, 4)` seq parse -- not a real `pack_mesh` frame."""
    return struct.pack("<III", web.TAG_MESH, seq, 0) + b"\x00" * 8


def _mesh_state(**extra) -> SimpleNamespace:
    return SimpleNamespace(mesh_clients={}, latest_mesh=None, latest_mesh_seq=0, **extra)


def test_mesh_delivery_is_credit_gated_one_in_flight():
    """The headline contract test: under the OLD open-loop broadcast this
    would fail immediately, because there was no credit check at all -- every
    cached mesh would go out to every client on every tick. Here: an un-acked
    flow gets nothing further, a mesh that was overwritten before ever being
    sent counts as `superseded` (not silently redelivered), and an ack
    advances a flow straight to the newest mesh, skipping anything in between."""
    state = _mesh_state()
    a, b = _FakeMeshWs(), _FakeMeshWs()
    flow_a, flow_b = web.MeshFlow(), web.MeshFlow()
    state.mesh_clients[a] = flow_a
    state.mesh_clients[b] = flow_b
    now = time.monotonic()

    web._cache_latest_mesh(state, _mesh_bytes(1))
    asyncio.run(web._pump_mesh(state, now))
    assert len(a.sent) == 1 and len(b.sent) == 1
    assert flow_a.superseded == 0 and flow_b.superseded == 0

    # Two more meshes land with nobody acking -- neither flow has a free
    # credit, so nothing more goes out no matter how many tick.
    web._cache_latest_mesh(state, _mesh_bytes(2))
    asyncio.run(web._pump_mesh(state, now))
    web._cache_latest_mesh(state, _mesh_bytes(3))
    asyncio.run(web._pump_mesh(state, now))
    assert len(a.sent) == 1 and len(b.sent) == 1
    # Mesh #2 was cached over and never sent to either flow before #3 arrived.
    assert flow_a.superseded == 1 and flow_b.superseded == 1

    # A acks its outstanding credit -> jumps straight to #3 (never sees #2).
    flow_a.in_flight = False
    flow_a.acked_ever = True
    flow_a.acks += 1
    asyncio.run(web._pump_mesh(state, now))
    assert len(a.sent) == 2
    tag, seq, _flags = struct.unpack_from("<III", a.sent[-1], 0)
    assert (tag, seq) == (web.TAG_MESH, 3)
    assert flow_a.superseded == 1          # unchanged by the ack/resend itself
    # B never acked -- still holds #1, no further legacy send inside the window.
    assert len(b.sent) == 1


def test_mesh_legacy_client_is_throttled_to_the_interval():
    """A client that never sends `mesh_ack` (an old cached tab, a diagnostic
    tool) still gets the latest mesh, but at a bounded cadence rather than
    every tick -- `LEGACY_MESH_INTERVAL_S`."""
    state = _mesh_state()
    ws = _FakeMeshWs()
    state.mesh_clients[ws] = web.MeshFlow()
    t0 = 1_000.0

    web._cache_latest_mesh(state, _mesh_bytes(1))
    asyncio.run(web._pump_mesh(state, t0))
    assert len(ws.sent) == 1

    web._cache_latest_mesh(state, _mesh_bytes(2))
    asyncio.run(web._pump_mesh(state, t0 + 1.0))     # inside the window
    assert len(ws.sent) == 1

    asyncio.run(web._pump_mesh(state, t0 + web.LEGACY_MESH_INTERVAL_S + 0.1))
    assert len(ws.sent) == 2


def test_mesh_ack_timeout_clears_credit_without_resending_the_same_mesh():
    """An ack that never arrives must not wedge the credit forever, but the
    timeout alone does not re-push identical bytes -- newness is by identity,
    so only a genuinely new mesh gets sent after the credit frees up."""
    state = _mesh_state()
    ws = _FakeMeshWs()
    flow = web.MeshFlow(acked_ever=True)     # a normal acking client
    state.mesh_clients[ws] = flow
    t0 = 2_000.0

    web._cache_latest_mesh(state, _mesh_bytes(1))
    asyncio.run(web._pump_mesh(state, t0))
    assert len(ws.sent) == 1 and flow.in_flight and flow.ack_timeouts == 0

    asyncio.run(web._pump_mesh(state, t0 + 10.0))    # well before the timeout
    assert len(ws.sent) == 1 and flow.ack_timeouts == 0

    asyncio.run(web._pump_mesh(state, t0 + web.MESH_ACK_TIMEOUT_S + 1.0))
    assert flow.ack_timeouts == 1 and not flow.in_flight
    assert len(ws.sent) == 1                          # same mesh, not resent

    web._cache_latest_mesh(state, _mesh_bytes(2))
    asyncio.run(web._pump_mesh(state, t0 + web.MESH_ACK_TIMEOUT_S + 2.0))
    assert len(ws.sent) == 2                          # the freed credit is usable again


def test_mesh_pump_drops_a_client_whose_send_fails():
    state = _mesh_state()
    ws = _FakeMeshWs(fail=True)
    state.mesh_clients[ws] = web.MeshFlow()
    web._cache_latest_mesh(state, _mesh_bytes(1))
    asyncio.run(web._pump_mesh(state, time.monotonic()))
    assert ws not in state.mesh_clients
    assert ws.closed


def test_reset_slam_clears_the_cached_mesh_survives_flow_identity():
    """`_reset_slam` drops the cache so a torn-down map's mesh is never resent
    -- but flow bookkeeping is untouched, and a fresh map's first mesh (a new
    bytes object) reaches an already-acked flow even though nothing compares
    `mesh_seq` (BUG-061: seq resets to 0 on reset, so a seq-based newness
    check would wedge here)."""
    old_mesh = _mesh_bytes(9)
    state = SimpleNamespace(mesh_clients={}, latest_mesh=old_mesh, latest_mesh_seq=9,
                            slam_runner=None)
    ws = _FakeMeshWs()
    state.mesh_clients[ws] = web.MeshFlow(acked_ever=True, last_sent_obj=old_mesh)

    asyncio.run(web._reset_slam(state))
    assert state.latest_mesh is None and state.latest_mesh_seq == 0

    web._cache_latest_mesh(state, _mesh_bytes(1))
    asyncio.run(web._pump_mesh(state, time.monotonic()))
    assert len(ws.sent) == 1


def test_ws_mesh_late_joiner_gets_the_cached_mesh_and_cleans_up_on_disconnect():
    from starlette.testclient import TestClient
    client = TestClient(web.app)
    mesh = _mesh_bytes(7)
    web.app.state.mesh_clients = {}
    web.app.state.latest_mesh = mesh
    web.app.state.latest_mesh_seq = 7
    with client.websocket_connect("/ws-mesh") as ws:
        data = ws.receive_bytes()
        assert data == mesh
        assert len(web.app.state.mesh_clients) == 1
        flow = next(iter(web.app.state.mesh_clients.values()))
        assert flow.sent == 1 and flow.in_flight
    assert len(web.app.state.mesh_clients) == 0     # disconnect popped the flow


def test_ws_mesh_ack_advances_straight_to_a_newer_cached_mesh():
    from starlette.testclient import TestClient
    client = TestClient(web.app)
    web.app.state.mesh_clients = {}
    web.app.state.latest_mesh = _mesh_bytes(1)
    web.app.state.latest_mesh_seq = 1
    with client.websocket_connect("/ws-mesh") as ws:
        first = ws.receive_bytes()
        assert first == _mesh_bytes(1)
        web.app.state.latest_mesh = _mesh_bytes(2)   # a newer map lands before the ack
        ws.send_text(json.dumps({"type": "mesh_ack"}))
        second = ws.receive_bytes()
        assert second == _mesh_bytes(2)
        flow = next(iter(web.app.state.mesh_clients.values()))
        assert flow.acked_ever and flow.acks == 1 and flow.sent == 2


def test_broadcaster_sends_slam_and_detailed_pose_before_caching_mesh():
    """BUG-061 A2: pose JSON must never sit behind mesh bytes in the same
    tick. Source-level, mirroring `test_broadcaster_no_longer_feeds_slam`."""
    src = pathlib.Path(web.__file__).read_text()
    body = src[src.index("async def _broadcaster()"):src.index('@app.websocket("/ws")')]

    slam_block = body[body.index('if ui.display == "slam" and slam is not None'):
                      body.index("# IR_IMAGE on its own slower cadence")]
    assert (slam_block.index("_broadcast_text(clients, json.dumps(smsg))")
            < slam_block.index("_cache_latest_mesh(state, mesh_bytes)"))

    detailed_block = body[body.index('if ui.display == "detailed" and detailed is not None'):
                          body.index("# Give every")]
    assert (detailed_block.index("_broadcast_text(clients, json.dumps(dmsg))")
            < detailed_block.index("_cache_latest_mesh(state, mesh_bytes)"))


def test_broadcaster_no_longer_broadcasts_mesh_on_ws():
    """MESH (tag 3) moved to `/ws-mesh` entirely (BUG-061 A3) -- if
    `_broadcast_bytes` ever gets called with mesh bytes again on the main
    socket, pose is back to sitting behind a multi-MB payload."""
    src = pathlib.Path(web.__file__).read_text()
    body = src[src.index("async def _broadcaster()"):src.index('@app.websocket("/ws")')]
    assert "_broadcast_bytes(clients, mesh_bytes)" not in body


def test_ws_flow_counters_shape():
    state = _mesh_state(clients={object(), object()})
    ws1, ws2 = object(), object()
    state.mesh_clients[ws1] = web.MeshFlow(in_flight=True, acked_ever=True,
                                           superseded=2, ack_timeouts=1,
                                           last_ack_lag_s=0.05)
    state.mesh_clients[ws2] = web.MeshFlow(in_flight=False, acked_ever=False)
    state.latest_mesh = b"1234"
    counters = web.ws_flow_counters(state)
    assert counters == {
        "clients": 2,
        "mesh_clients": 2,
        "mesh_in_flight": 1,
        "legacy_mesh_clients": 1,
        "mesh_superseded_total": 2,
        "mesh_ack_lag_s_max": 0.05,
        "mesh_ack_timeouts_total": 1,
        "latest_mesh_bytes": 4,
    }


def test_ws_flow_counters_is_getattr_safe_on_a_bare_state():
    """A hand-built `app.state` (most unit tests) has neither `mesh_clients`
    nor `latest_mesh` -- this must degrade to zeros/empty, never raise."""
    state = SimpleNamespace()
    counters = web.ws_flow_counters(state)
    assert counters["clients"] == 0 and counters["mesh_clients"] == 0
    assert counters["latest_mesh_bytes"] == 0
    assert counters["mesh_ack_lag_s_max"] is None


def test_broadcaster_metrics_attach_ws_counters_beside_transport():
    src = pathlib.Path(web.__file__).read_text()
    body = src[src.index("async def _broadcaster()"):]
    chunk = body[body.index('msg["transport"] = transport_counters(state)'):]
    assert 'msg["ws"] = ws_flow_counters(state)' in chunk[:200]


def test_build_slam_message_carries_ts_and_device():
    step = _FrameStep(pose=np.eye(4), fitness=0.9, rmse=0.01,
                      tracking_lost=False, slam_ms=3.0)
    before = time.time()
    msg = web.build_slam_message(step, [np.eye(4)], frames_integrated=1, mesh_seq=1,
                                 source_vertex_count=10, device="CUDA:0")
    after = time.time()
    assert before <= msg["ts"] <= after
    assert msg["device"] == "CUDA:0"

    msg2 = web.build_slam_message(step, [np.eye(4)], frames_integrated=1, mesh_seq=1,
                                  source_vertex_count=10)
    assert msg2["device"] is None


# ---------------------------------------------------------------------------
# Plan item 2 (2026-08-02) -- counters, stage timing, GPU/VRAM scope labeling
# ---------------------------------------------------------------------------

def test_build_slam_message_carries_stage_timing_from_the_step():
    step = _FrameStep(pose=np.eye(4), fitness=0.9, rmse=0.01, tracking_lost=False,
                      slam_ms=12.0, raycast_ms=3.5, icp_ms=4.25, integrate_ms=1.0)
    msg = web.build_slam_message(step, [np.eye(4)], frames_integrated=1, mesh_seq=1,
                                 source_vertex_count=10)
    assert msg["raycast_ms"] == pytest.approx(3.5)
    assert msg["icp_ms"] == pytest.approx(4.25)
    assert msg["integrate_ms"] == pytest.approx(1.0)


def test_build_slam_message_omitted_instrumentation_fields_are_null():
    """Every new field is optional -- a caller that doesn't pass them (an
    older code path, or a worker lacking the attribute) must get None, not a
    fabricated 0 that would read as a real measurement."""
    step = _FrameStep(pose=np.eye(4), fitness=0.9, rmse=0.01,
                      tracking_lost=False, slam_ms=1.0)
    msg = web.build_slam_message(step, [np.eye(4)], frames_integrated=1, mesh_seq=1,
                                 source_vertex_count=10)
    assert msg["backend"] is None
    assert msg["frames_submitted"] is None
    assert msg["frames_processed"] is None
    assert msg["frames_overwritten"] is None
    assert msg["mesh_extract_ms"] is None
    assert msg["mesh_prep_ms"] is None
    assert msg["mesh_pack_ms"] is None
    assert msg["mesh_payload_bytes"] is None
    assert msg["gpu"] is None
    # stage timings fall back to FrameStep's own 0.0 defaults, not None --
    # they came off `step` itself, which always has SOME value.
    assert msg["raycast_ms"] == 0.0
    assert msg["icp_ms"] == 0.0
    assert msg["integrate_ms"] == 0.0


def test_build_slam_message_carries_counters_and_backend():
    step = _FrameStep(pose=np.eye(4), fitness=0.9, rmse=0.01,
                      tracking_lost=False, slam_ms=1.0)
    msg = web.build_slam_message(
        step, [np.eye(4)], frames_integrated=1, mesh_seq=1, source_vertex_count=10,
        backend="remote", frames_submitted=30, frames_processed=25,
        frames_overwritten=5, mesh_extract_ms=12.3, mesh_prep_ms=4.5,
        mesh_pack_ms=1.2, mesh_payload_bytes=3_200_000)
    assert msg["backend"] == "remote"
    assert msg["frames_submitted"] == 30
    assert msg["frames_processed"] == 25
    assert msg["frames_overwritten"] == 5
    # the accounting invariant the worker-level tests pin directly
    assert msg["frames_processed"] + msg["frames_overwritten"] == msg["frames_submitted"]
    assert msg["mesh_extract_ms"] == pytest.approx(12.3)
    assert msg["mesh_prep_ms"] == pytest.approx(4.5)
    assert msg["mesh_pack_ms"] == pytest.approx(1.2)
    assert msg["mesh_payload_bytes"] == 3_200_000


def test_slam_gpu_fields_labels_vram_as_device_wide_always():
    res = _resource_snapshot(gpu_util=None, gpu_source="n/a")
    out = web._slam_gpu_fields(res)
    assert out["vram_scope"] == "device-wide"
    assert out["vram_used_bytes"] == res.device_vram_used
    assert out["vram_total_bytes"] == res.device_vram_total


def test_slam_gpu_fields_carries_the_real_gpu_source_as_scope():
    """gpu_util's scope is NOT fixed (per-process if pynvml is ever installed,
    device-wide otherwise) -- the field must say which, not assume."""
    res = _resource_snapshot(gpu_util=55.0, gpu_source="nvml-device")
    out = web._slam_gpu_fields(res)
    assert out["gpu_util_pct"] == 55.0
    assert out["gpu_util_scope"] == "nvml-device"

    res2 = _resource_snapshot(gpu_util=12.0, gpu_source="pynvml")
    out2 = web._slam_gpu_fields(res2)
    assert out2["gpu_util_scope"] == "pynvml"


def test_slam_gpu_fields_all_null_with_no_resources():
    out = web._slam_gpu_fields(None)
    assert out["gpu_util_pct"] is None
    assert out["vram_used_bytes"] is None
    assert out["vram_total_bytes"] is None
    assert out["vram_scope"] == "device-wide"     # the scope claim is always honest, even when null


class _InstrumentedFakeWorker(_FakeWorker):
    """`_FakeWorker` plus the plan-item-2 instrumentation surface, so
    `SlamRunner.poll()` can be tested against a worker that actually reports
    it (mirrors the real SlamWorker's property names exactly)."""
    device = "CUDA:0"
    backend = "local"

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.frames_submitted = 40
        self.frames_processed = 33
        self.frames_overwritten = 7
        self.mesh_extract_ms = 9.5


class _InstrumentedFakeMeshPrep(_FakeMeshPrep):
    prep_ms = 3.0
    pack_ms = 0.7
    payload_bytes = 65536


def test_slamrunner_poll_surfaces_worker_instrumentation():
    r = web.SlamRunner(bus=LogBus())
    with r._lock:
        r._active = True
        r._worker = _InstrumentedFakeWorker()
        r._meshprep = _InstrumentedFakeMeshPrep()
    msg, _mesh_bytes = r.poll("split")
    assert msg["device"] == "CUDA:0"          # read from worker.device, not host inference
    assert msg["backend"] == "local"
    assert msg["frames_submitted"] == 40
    assert msg["frames_processed"] == 33
    assert msg["frames_overwritten"] == 7
    assert msg["mesh_extract_ms"] == pytest.approx(9.5)
    assert msg["mesh_prep_ms"] == pytest.approx(3.0)
    assert msg["mesh_pack_ms"] == pytest.approx(0.7)
    assert msg["mesh_payload_bytes"] == 65536


def test_slamrunner_poll_refreshes_device_as_it_becomes_known():
    """A remote worker's device is None until its first response -- poll()
    must pick it up on a LATER tick rather than staying stuck at whatever was
    true when the pipeline was first built."""
    class _SlowDeviceWorker(_FakeWorker):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.device = None
            self.backend = "remote"

    w = _SlowDeviceWorker()
    r = web.SlamRunner(bus=LogBus())
    with r._lock:
        r._active = True
        r._worker = w
        r._meshprep = _FakeMeshPrep()
    msg, _ = r.poll("split")
    assert msg["device"] is None
    assert msg["backend"] == "remote"

    w.device = "CUDA:0"      # the "first response arrived" moment
    msg2, _ = r.poll("split")
    assert msg2["device"] == "CUDA:0"


# ---------------------------------------------------------------------------
# BUG-061 Part B -- device-wide GPU utilization fallback
# ---------------------------------------------------------------------------

def test_resources_to_dict_falls_back_to_device_wide_gpu_util(monkeypatch):
    """`pynvml` is absent here, so the per-process sampler always reports
    `gpu_util=None`; the device-wide NVML utilization now fills that hole
    rather than leaving the HUD blank."""
    res = _resource_snapshot(gpu_util=None, gpu_source="n/a")
    monkeypatch.setattr(web, "_device_gpu_util", lambda: {"gpu_pct": 42, "mem_pct": 10})
    out = web.resources_to_dict(res)
    assert out["gpu_util"] == 42.0
    assert out["gpu_source"] == "nvml-device"


def test_resources_to_dict_prefers_the_per_process_gpu_util_when_present():
    """If a future host ever has `pynvml`, that per-process figure must win
    over the coarser device-wide fallback, not be silently overwritten."""
    res = _resource_snapshot(gpu_util=17.0, gpu_source="pynvml")
    out = web.resources_to_dict(res)
    assert out["gpu_util"] == 17.0
    assert out["gpu_source"] == "pynvml"


def test_resources_to_dict_stays_null_when_no_gpu_source_at_all(monkeypatch):
    res = _resource_snapshot(gpu_util=None, gpu_source="n/a")
    monkeypatch.setattr(web, "_device_gpu_util", lambda: None)
    out = web.resources_to_dict(res)
    assert out["gpu_util"] is None
    assert out["gpu_source"] == "n/a"


def test_device_gpu_util_is_defensive_when_gpumem_lacks_the_function(monkeypatch):
    """`device_utilization()` lands in `roomscan.slam.gpumem` from a change
    landing CONCURRENTLY with this one -- it may not exist yet when this runs.
    That must never be the reason a `metrics` message fails to build."""
    import roomscan.slam.gpumem as gpumem
    monkeypatch.delattr(gpumem, "device_utilization", raising=False)
    assert web._device_gpu_util() is None


def test_device_gpu_util_returns_none_on_a_probe_exception(monkeypatch):
    import roomscan.slam.gpumem as gpumem

    def _boom():
        raise RuntimeError("nvml broke")

    monkeypatch.setattr(gpumem, "device_utilization", _boom)
    assert web._device_gpu_util() is None


# --- BUG-062: every [slam] mapper knob must reach the LIVE worker -------------

def test_live_slam_forwards_every_configured_mapper_knob(monkeypatch):
    """BUG-062: `_construct` hand-picked five keys, so a user who set
    `icp_mode`/`voxel_size`/`max_iter`/`max_dist`/the quality gates/the
    stationarity knobs changed the CLI and Detailed paths but not Live SLAM --
    silently. Set every shared field to a non-default value and prove each one
    arrives at the constructed worker."""
    import roomscan.slam.backend as backend
    import roomscan.slam.meshprep as meshprep
    from roomscan.slam.config import SlamConfig

    overrides = dict(
        icp_mode="6dof", voxel_size=0.02, max_dist=0.07, icp_retry_dist=0.19,
        max_iter=11, min_fitness=0.44, max_rmse=0.066, min_confidence=33.0,
        weight_threshold=4.5, baro_authority=0.11, baro_tau_frames=450,
        stationary_hold=False, stationary_window=17, stationary_coherence=0.71,
        stationary_step_ceiling=0.041, stationary_rot_ceiling=0.55,
        release_cache_every=3, block_count=222_000,
        # item 5 (2026-08-02): plumbed like block_count, so it is pinned here
        # like block_count. A device selector that silently fails to apply is
        # indistinguishable from one that applied, because the change it makes
        # is bit-identical by design -- this is the only thing that can tell.
        icp_device="CUDA:3",
    )
    cfg = SlamConfig(**overrides)
    # every override must actually differ from the default, or the test proves nothing
    stock = SlamConfig()
    assert all(getattr(stock, k) != v for k, v in overrides.items())

    monkeypatch.setattr(SlamConfig, "load", classmethod(lambda cls, *a, **k: cfg))
    seen = {}
    def _mk(w, h, **kw):
        seen.update(kw); return _FakeWorker()
    monkeypatch.setattr(backend, "make_slam_worker", _mk)
    monkeypatch.setattr(meshprep, "MeshPrep", lambda *a, **k: _FakeMeshPrep())

    web.SlamRunner(bus=LogBus())._construct(54, 42)

    missing = {k: (v, seen.get(k)) for k, v in overrides.items() if seen.get(k) != v}
    assert not missing, f"[slam] keys that never reached the live mapper: {missing}"


def test_live_slam_fov_and_device_override_the_config(monkeypatch):
    """The two live-specific overrides must win over `mapper_kwargs()`: the FOV
    is the measured sensor geometry and the device is the resolved one, so a
    `[slam]` key must not be able to shadow either."""
    import roomscan.slam.backend as backend
    import roomscan.slam.meshprep as meshprep
    from roomscan.slam.config import SlamConfig

    monkeypatch.setattr(SlamConfig, "load",
                        classmethod(lambda cls, *a, **k: SlamConfig(fov_h=1.0, fov_v=2.0)))
    seen = {}
    def _mk(w, h, **kw):
        seen.update(kw); return _FakeWorker()
    monkeypatch.setattr(backend, "make_slam_worker", _mk)
    monkeypatch.setattr(meshprep, "MeshPrep", lambda *a, **k: _FakeMeshPrep())

    r = web.SlamRunner(bus=LogBus())
    device = r._construct(54, 42)[2]
    assert (seen["fov_h"], seen["fov_v"]) == (r._fov_h, r._fov_v) != (1.0, 2.0)
    assert str(seen["device"]) == device
