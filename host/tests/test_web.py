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
    FrameHeader,
    FrameType,
    StreamId,
    pack_frame,
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
    web.app.state.deproj = None
    web.app.state.clients = set()
    web.app.state.command_labels = set()
    web.app.state.debounce = {}
    web.app.state.ready = True

    threading.Thread(
        target=panel._run_reader,
        args=(source, decoder, stage, stats, slot, fault, bus, None, None,
              pacer, lambda: False),
        kwargs={"state": None, "metrics": metrics},
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
