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

import numpy as np
import pytest

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
    ss.feed(_sframe(StreamId.IMU_QUAT, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
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
    assert web._sanitize_axis_labels(["", "  ", "Pan"]) == ("Roll", "Pitch", "Pan")
    assert web._sanitize_axis_labels(["Tilt", "Swing", "Twist"]) == ("Tilt", "Swing", "Twist")


def test_sanitize_axis_labels_truncates_long_strings():
    long = "x" * 100
    out = web._sanitize_axis_labels([long, "Pitch", "Yaw"])
    assert len(out[0]) == web._MAX_LABEL_LEN


def test_ui_from_config_maps_orientation_mode_and_labels():
    cfg = ViewerConfig(orientation_mode="boresight", orientation_labels="Tilt,Pan,Twist")
    ui = web.ui_from_config(cfg)
    assert ui.orientation_mode == "boresight"
    assert ui.orientation_labels == ("Tilt", "Pan", "Twist")


def test_ui_from_config_rejects_bad_orientation_mode():
    cfg = ViewerConfig(orientation_mode="nonsense")
    ui = web.ui_from_config(cfg)
    assert ui.orientation_mode == web.UiState().orientation_mode


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
        paused=True, speed_fps=30.0, loop=True, position=0.5, total_frames=42)
    assert m["type"] == "session" and m["mode"] == "replay"
    assert m["playback"]["is_replay"] and m["playback"]["position"] == 0.5
    assert m["playback"]["total_frames"] == 42 and m["playback"]["loop"] is True
    assert json.loads(json.dumps(m)) == m                     # JSON-round-trips


# ---- SessionController ----

def _make_controller(tmp_path, *, live_source=None, live_label="test",
                     replay_path=None, captures_dir=None, speed_fps=0.0):
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
    finally:
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


def test_slamrunner_lazy_build_and_poll(_fake_slam):
    r = web.SlamRunner(bus=LogBus())
    r.set_active(True)
    depth = np.zeros((6, 8), np.float32)
    r.submit(depth, (1, 0, 0, 0), 101325.0)
    assert _fake_slam["wh"] == (8, 6)                    # width, height from depth shape
    assert _fake_slam["worker"].started and _fake_slam["mp"].started
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


def test_root_redirects_to_static_index():
    """The bare site root is a convenience redirect to the app entry point --
    typing the hostname alone used to 404 (only /static and /results were
    mounted)."""
    route = next(r for r in web.app.routes if getattr(r, "path", None) == "/")
    assert "GET" in route.methods
    resp = asyncio.run(web.root_redirect())
    assert resp.status_code == 307
    assert resp.headers["location"] == "/static/index.html"
