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

def _sframe(sid, payload: bytes) -> Frame:
    return Frame(FrameHeader(FrameType.DATA, sid, 0, 1, 1000, 0, 0, len(payload)), payload)


def test_build_sensor_message_none_when_empty():
    # A ToF-only session (no 9/10 frames ever) must produce no sensor traffic.
    assert web.build_sensor_message(SensorState(), None) is None


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
        ss.feed(_sframe(StreamId.ENV, struct.pack("<5f", 101000.0 + i, 1.0, 2.0, 3.0, 22.0 + i)))
    msg = web.build_sensor_message(ss, None)
    assert msg is not None
    assert msg["pressure_pa"] == pytest.approx(101002.0, abs=0.5)   # latest wins
    assert msg["temp_c"] == pytest.approx(24.0, abs=0.05)
    assert msg["mag_ut"] == [1.0, 2.0, 3.0]                          # raw (no mag_cal)
    assert msg["heading"] is not None                               # quat+env present
    assert len(msg["pressure_hist"]) == 3 and len(msg["temp_hist"]) == 3
    assert msg["pressure_hist"][0] == pytest.approx(101000.0, abs=0.5)
    json.dumps(msg)


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
