"""Issue #101 regression coverage: "View mode can display current live data
instead of only the loaded capture."

Two tiers, per the fleet-worker scope decision (no browser here -- that is
the orchestrator's job):

  * Unit tests for the mechanism: `SessionController.generation`, the four
    `_Generation*` guard wrappers, `_view_ready`, `MetricsRegistry.reset_source`,
    `OrientationSmoother.reset`/`OrientationJitter.reset`, and the View-readiness
    gates in `_handle_inbound`.
  * End-to-end tests against the REAL server (`roomscan.web.app` under a real
    `uvicorn.Server`) and REAL `websockets` clients on `/ws` (+ `/ws-mesh` for
    the mesh-reset unit test) -- no browser, asserting on the actual messages
    and binary payloads crossing the wire, per `test_web.py`'s existing
    `test_broadcaster_fanout_two_clients_same_frames` pattern.

A fake LIVE source (`_LoopingLiveSource`) continuously produces a distinctive,
recognizable DEPTH_ZF32 value (never present in any capture fixture used here)
so a test can tell "this point cloud came from the live device" from "this
point cloud came from the loaded capture" just by looking at depth.
"""
from __future__ import annotations

import asyncio
import json
import queue
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from roomscan import panel, web
from roomscan.logbus import LogBus
from roomscan.metrics import MetricsRegistry
from roomscan.pipeline import TransformStage
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame
from roomscan.sensors import SensorState
from roomscan.sources import Recorder
from roomscan.viewer import Stats

# ---------------------------------------------------------------------------
# fixtures / helpers
# ---------------------------------------------------------------------------

W, H = 8, 6
LIVE_DEPTH_MM = 8000.0     # distinctive: near the 10 m deprojection ceiling
CAPTURE_DEPTH_MM = 1500.0  # distinctive: well inside 3 m


def _depth_frame_bytes(depth_mm: float, seq: int, w: int = W, h: int = H) -> bytes:
    depth = np.full((h, w), depth_mm, dtype=np.float32)
    payload = depth.astype("<f4").tobytes()
    header = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, seq,
                         seq * 35000, w, h, len(payload))
    return pack_frame(header, payload)


def _make_capture(path: Path, depth_mm: float, n_frames: int = 30) -> None:
    out = bytearray()
    for i in range(n_frames):
        out += _depth_frame_bytes(depth_mm, i + 1)
    path.write_bytes(bytes(out))


class _LoopingLiveSource:
    """A `source.read()` stand-in that never signals EOF (unlike `FileSource`,
    `pump()` only stops on EOF for a `FileSource` instance or a truthy
    `eof_on_empty`), cycling a small buffer of frames at a fixed DISTINCTIVE
    depth forever -- the "live device that never stops streaming" this issue
    is about."""

    def __init__(self, depth_mm: float, n_frames: int = 50, chunk: int = 4096):
        buf = bytearray()
        for i in range(n_frames):
            buf += _depth_frame_bytes(depth_mm, i + 1)
        self._buf = bytes(buf)
        self._chunk = chunk
        self._pos = 0

    def read(self) -> bytes:
        if self._pos >= len(self._buf):
            self._pos = 0
        out = self._buf[self._pos:self._pos + self._chunk]
        self._pos += len(out)
        if not out:                 # empty buffer edge case; never actually hit here
            time.sleep(0.01)
        return out

    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


def _make_controller(tmp_path, *, live_depth_mm: float = LIVE_DEPTH_MM,
                     replay_path=None, captures_dir=None):
    stage = TransformStage(outputs=("depth", "reflectance", "confidence"))
    slot: queue.Queue = queue.Queue(maxsize=1)
    live = _LoopingLiveSource(live_depth_mm)
    return web.SessionController(
        live_source=live, live_label="test-live", stage=stage, stats=Stats(),
        slot=slot, fault={}, bus=LogBus(), client=None, recorder=Recorder(),
        pacer=panel._Pacer(interval=0.0),
        sensor_state=SensorState(), metrics=MetricsRegistry(window_s=2.0),
        captures_dir=str(captures_dir or tmp_path), initial_replay_path=replay_path,
    ), slot


def _pc_z_values(buf: bytes) -> np.ndarray:
    """Decode a POINT_CLOUD payload's positions (§6.1) and return just Z
    (roughly metres of depth, modulo the per-pixel FOV cosine) -- enough to
    tell the two fixtures' depths apart at a glance."""
    data = np.frombuffer(buf, dtype="<f4", offset=4)
    n = data.size // 6
    pos = data[: n * 3].reshape(n, 3)
    return pos[:, 2]


def _is_point_cloud(m) -> bool:
    return isinstance(m, (bytes, bytearray)) and len(m) >= 4 and \
        int.from_bytes(m[:4], "little") == web.TAG_POINT_CLOUD


# ---------------------------------------------------------------------------
# Unit tests: the generation mechanism (step 1)
# ---------------------------------------------------------------------------

def test_session_controller_generation_starts_at_zero_and_bumps_on_switch(tmp_path):
    cap_a = tmp_path / "a.bin"
    cap_b = tmp_path / "b.bin"
    _make_capture(cap_a, CAPTURE_DEPTH_MM)
    _make_capture(cap_b, CAPTURE_DEPTH_MM + 1)
    ctrl, _slot = _make_controller(tmp_path)
    assert ctrl.generation == 0
    ctrl.switch_to_replay(str(cap_a))
    assert ctrl.generation == 1
    ctrl.switch_to_replay(str(cap_b))
    assert ctrl.generation == 2
    ctrl.switch_to_live()
    assert ctrl.generation == 3
    ctrl.close()


def test_session_controller_generation_unchanged_by_seek_and_restart(tmp_path):
    """Seek/restart are timeline discontinuities WITHIN one capture -- not a
    source switch -- so they must not trip the client's reset barrier."""
    cap = tmp_path / "a.bin"
    _make_capture(cap, CAPTURE_DEPTH_MM, n_frames=50)
    ctrl, _slot = _make_controller(tmp_path, replay_path=str(cap))
    assert ctrl.generation == 0
    ctrl.seek(0.5)
    assert ctrl.generation == 0
    ctrl.restart()
    assert ctrl.generation == 0
    ctrl.close()


def test_generation_slot_tags_puts_with_the_captured_generation():
    real = queue.Queue(maxsize=1)
    slot = web._GenerationSlot(real, generation=7)
    header = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 0, W, H, 4)
    slot.put((header, {"depth": np.zeros((H, W), dtype=np.float32)}))
    gen, hdr, outputs = real.get_nowait()
    assert gen == 7
    assert hdr is header
    assert outputs["depth"].shape == (H, W)


def test_generation_sensor_state_drops_feed_from_a_superseded_generation():
    """The exact mechanism the end-to-end regression below depends on: once
    the CONTROLLER's live generation has moved past the value captured at
    reader-spawn time, `.feed()` must become a no-op."""
    ctrl = SimpleNamespace(generation=1)
    real = SensorState()
    guarded = web._GenerationSensorState(ctrl, generation=1, state=real)
    quat_frame_payload = np.array([1.0, 0.0, 0.0, 0.0], dtype="<f4").tobytes()
    header = FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, 0, 0, 0,
                         len(quat_frame_payload))
    from roomscan.protocol import Frame
    frame = Frame(header, quat_frame_payload)

    guarded.feed(frame)
    assert real.latest_quat() == pytest.approx((1.0, 0.0, 0.0, 0.0))

    # The controller has moved on (a switch happened) -- this reader's
    # generation (1) is now stale, so its frame must be discarded.
    real.reset_source()
    ctrl.generation = 2
    header2 = FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 2, 0, 0, 0,
                          len(quat_frame_payload))
    guarded.feed(Frame(header2, quat_frame_payload))
    assert real.latest_quat() is None, "a superseded-generation frame must not write state"


def test_generation_guard_removal_makes_stale_feed_reach_sensor_state(monkeypatch):
    """Prove the guard in `test_generation_sensor_state_drops_feed_from_a_superseded_generation`
    is load-bearing (docs/engineering-practices.md: prove a regression test by
    reintroducing the defect): with the generation check short-circuited to
    "always current", the same stale frame DOES land -- i.e. the assertion
    above would fail without the guard."""
    ctrl = SimpleNamespace(generation=1)
    real = SensorState()
    guarded = web._GenerationSensorState(ctrl, generation=1, state=real)

    def _unguarded_feed(self, frame):
        self._state.feed(frame)   # the pre-#101 behaviour: no generation check at all
    monkeypatch.setattr(web._GenerationSensorState, "feed", _unguarded_feed)

    from roomscan.protocol import Frame
    quat_frame_payload = np.array([0.0, 1.0, 0.0, 0.0], dtype="<f4").tobytes()
    ctrl.generation = 2   # controller has already moved on
    header = FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, 0, 0, 0,
                         len(quat_frame_payload))
    guarded.feed(Frame(header, quat_frame_payload))
    # With the guard removed, the stale frame DID land -- this is the failure
    # mode the real (guarded) code prevents.
    assert real.latest_quat() == pytest.approx((0.0, 1.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# Unit tests: source-owned state resets (step 2)
# ---------------------------------------------------------------------------

def test_metrics_registry_reset_source_clears_meters_and_ticks():
    m = MetricsRegistry(window_s=2.0)
    header = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 0, W, H, 4)
    now = time.monotonic()
    m.record(header, 128, now)
    m.tick_render(now)
    m.tick_transform(now)
    m.tick_browser(now)
    snap = m.snapshot(now + 0.01)
    assert len(snap.streams) == 1

    m.reset_source()
    snap2 = m.snapshot(now + 0.02)
    assert snap2.streams == []
    assert snap2.render_fps == 0.0
    assert snap2.transform_fps == 0.0
    assert snap2.browser_fps == 0.0


def test_orientation_smoother_reset_forgets_the_held_quat():
    sm = web.OrientationSmoother()
    q1 = (1.0, 0.0, 0.0, 0.0)
    assert sm.update(q1) == pytest.approx(q1)
    assert sm.held is not None
    sm.reset()
    assert sm.held is None
    # First sample after reset is adopted outright (same as a true first call).
    q2 = (0.7071, 0.0, 0.7071, 0.0)
    out = sm.update(q2)
    assert out == pytest.approx(q2, abs=1e-3)


def test_orientation_jitter_reset_forgets_previous_values():
    j = web.OrientationJitter()
    j.update((1.0, 0.0, 0.0, 0.0), heading_deg=10.0, now=0.0)
    out = j.update((1.0, 0.0, 0.0, 0.0), heading_deg=190.0, now=0.1)
    # A real 180 deg jump wrapped: diff should be ~180, not tiny.
    assert out["heading"]["n"] >= 1
    j.reset()
    out2 = j.update((1.0, 0.0, 0.0, 0.0), heading_deg=10.0, now=0.2)
    assert out2["heading"]["n"] == 0     # no previous value to diff against


# ---------------------------------------------------------------------------
# Unit tests: the View-readiness predicate + state-message wiring (step 3)
# ---------------------------------------------------------------------------

class _FakeCtrl:
    def __init__(self, *, mode="replay", replay_path="a.bin", generation=0):
        self.mode = mode
        self.replay_path = replay_path
        self.generation = generation
        self.index = {"has_stream_9": True}

    def start_auto_record(self) -> bool:
        # Keep the Live-SLAM-allowed test focused on the readiness gate, not
        # the (separately tested) auto-record side effects.
        return False

    def stop_auto_record(self) -> bool:
        return False


def test_view_ready_false_for_non_view_sources():
    ui = web.UiState(source="live", selected_capture=None)
    assert web._view_ready(ui, _FakeCtrl()) is False   # predicate itself, not the widened OR


def test_view_ready_false_while_browsing_with_nothing_selected():
    ui = web.UiState(source="view", selected_capture=None)
    assert web._view_ready(ui, _FakeCtrl()) is False


def test_view_ready_false_when_controller_still_live():
    ui = web.UiState(source="view", selected_capture="a.bin")
    assert web._view_ready(ui, _FakeCtrl(mode="live", replay_path=None)) is False


def test_view_ready_false_on_mismatched_capture_mid_swap():
    """Between `load_capture` swapping the reader and `ctrl.replay_path`
    landing on the NEW file, or after a browser click that raced a slow
    swap, the UI's selection and the controller's actual file can briefly
    disagree -- must not read as ready."""
    ui = web.UiState(source="view", selected_capture="b.bin")
    assert web._view_ready(ui, _FakeCtrl(replay_path="a.bin")) is False


def test_view_ready_true_when_selection_matches_replaying_file():
    ui = web.UiState(source="view", selected_capture="a.bin")
    assert web._view_ready(ui, _FakeCtrl(replay_path="a.bin")) is True


def test_state_message_carries_stream_generation_and_ready():
    ui = web.UiState(source="view", selected_capture="a.bin")
    ctrl = _FakeCtrl(replay_path="a.bin", generation=5)
    msg = web._state_message(ui, ctrl)
    assert msg["stream_generation"] == 5
    assert msg["stream_ready"] is True

    ui2 = web.UiState(source="view", selected_capture=None)
    msg2 = web._state_message(ui2, ctrl)
    assert msg2["stream_ready"] is False
    assert msg2["stream_generation"] == 5   # generation is controller-owned, not view-gated


def test_state_message_live_source_always_ready():
    ui = web.UiState(source="live")
    msg = web._state_message(ui, _FakeCtrl(mode="live", replay_path=None))
    assert msg["stream_ready"] is True


# ---------------------------------------------------------------------------
# Unit tests: set_display/set_mode readiness gates
# ---------------------------------------------------------------------------

def _inbound_state(ui, ctrl):
    published = []
    bus = SimpleNamespace(publish=published.append)
    return SimpleNamespace(config=None, ui_state=ui, clients=set(), controller=ctrl,
                           bus=bus, slam_runner=None, detailed_runner=None), published


def test_set_display_slam_refused_while_browsing_view_with_no_capture():
    ui = web.UiState(source="view", display="point_cloud", selected_capture=None)
    ctrl = _FakeCtrl()
    state, published = _inbound_state(ui, ctrl)
    asyncio.run(web._handle_inbound(state, {"type": "set_display", "display": "slam"}))
    assert ui.display == "point_cloud"
    assert any("select a capture" in line for line in published), published


def test_set_display_slam_allowed_on_live_source_even_though_view_ready_is_false():
    """Live SLAM must not be collaterally blocked by a predicate that only
    describes View."""
    ui = web.UiState(source="live", display="point_cloud", selected_capture=None)
    ctrl = _FakeCtrl(mode="live", replay_path=None)
    state, published = _inbound_state(ui, ctrl)
    asyncio.run(web._handle_inbound(state, {"type": "set_display", "display": "slam"}))
    assert ui.display == "slam", published


def test_set_display_preview_refused_on_mismatched_capture():
    ui = web.UiState(source="view", display="point_cloud", selected_capture="b.bin")
    ctrl = _FakeCtrl(replay_path="a.bin")
    state, published = _inbound_state(ui, ctrl)
    asyncio.run(web._handle_inbound(state, {"type": "set_display", "display": "preview"}))
    assert ui.display == "point_cloud"
    assert any("load a capture" in line for line in published), published


def test_set_mode_slam_refused_while_browsing_view():
    ui = web.UiState(source="view", display="point_cloud", selected_capture=None)
    ctrl = _FakeCtrl()
    state, published = _inbound_state(ui, ctrl)
    asyncio.run(web._handle_inbound(state, {"type": "set_mode", "mode": "slam"}))
    assert ui.display == "point_cloud"
    assert any("select a capture" in line for line in published), published


# ---------------------------------------------------------------------------
# Unit test: mesh-reset ordering (step 5)
# ---------------------------------------------------------------------------

class _FakeMeshWs:
    """A minimal hashable `send_text` stand-in. NOT `SimpleNamespace`: that
    class defines `__eq__`, which makes Python drop its default identity
    `__hash__` -- and `_mesh_clients` is keyed by websocket identity."""

    def __init__(self):
        self.sent: list[str] = []

    async def send_text(self, text: str) -> None:
        self.sent.append(text)


def test_broadcast_mesh_reset_sends_control_frame_and_clears_flow_bookkeeping():
    ws = _FakeMeshWs()
    flow = web.MeshFlow(in_flight=True, last_sent_obj=b"stale-mesh-bytes")
    state = SimpleNamespace(mesh_clients={ws: flow})

    asyncio.run(web._broadcast_mesh_reset(state, generation=3))

    assert len(ws.sent) == 1
    msg = json.loads(ws.sent[0])
    assert msg == {"type": "mesh_reset", "generation": 3}
    # In-flight bookkeeping for the OLD mesh must not block the new
    # generation's first mesh from being sent.
    assert flow.in_flight is False
    assert flow.last_sent_obj is None


# ---------------------------------------------------------------------------
# End-to-end: real server + real /ws clients (no browser)
# ---------------------------------------------------------------------------

def _start_server(app, port):
    import uvicorn
    config = uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=lambda: asyncio.run(server.serve()), daemon=True)
    thread.start()
    deadline = time.time() + 10.0
    while not server.started and time.time() < deadline:
        time.sleep(0.02)
    assert server.started, "uvicorn server did not start"
    return server, thread


def _free_port() -> int:
    import socket as _socket
    s = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _build_view_app_state(tmp_path):
    """Full `app.state` with a REAL `SessionController` (live source + a
    captures dir), so `set_source`/`load_capture`/`go_live` inbound messages
    exercise the actual production code path (issue #101), not a hand-rolled
    stand-in. Mirrors `test_web.py::_build_app_state`'s uvicorn-server
    pattern but wires the controller instead of a bare `_run_reader` thread.
    """
    import argparse
    from roomscan.control import CommandDispatcher

    captures_dir = tmp_path / "captures"
    captures_dir.mkdir()
    ctrl, _slot = _make_controller(tmp_path, captures_dir=captures_dir)

    bus = ctrl.bus
    dispatcher = CommandDispatcher(None, on_message=bus.publish)
    args = argparse.Namespace(fov_h=55.0, fov_v=42.0, replay=None, replay_fps=20.0)

    state = web.app.state
    state.args = args
    state.source = None
    state.client = None
    state.stage = ctrl.stage
    state.slot = ctrl.slot
    state.bus = bus
    state.metrics = ctrl.metrics
    state.dispatcher = dispatcher
    state.fault = ctrl.fault
    state.fault_reported = False
    state.stats = ctrl.stats
    state.pacer = ctrl.pacer
    state.ui_state = web.UiState()
    state.sensor_state = ctrl.sensor_state
    state.mag_cal = None
    state.deproj = None
    state.tof_meta = None
    state.orientation_smoother = web.OrientationSmoother()
    state.orientation_jitter = web.OrientationJitter()
    state.elevation_smoother = web.ElevationSmoother()
    state.clients = set()
    state.command_labels = set()
    state.debounce = {}
    state.ready = True
    state.controller = ctrl
    state.latest_mesh = None
    state.mesh_clients = {}
    state.slam_runner = None
    state.detailed_runner = None
    state.splat_runner = None
    ctrl.start()
    return ctrl, captures_dir


async def _recv_for(ws, seconds: float) -> list:
    """Collect every message that arrives within `seconds` (best-effort --
    used to assert an ABSENCE, so a short, generous window is the point)."""
    got = []
    deadline = time.monotonic() + seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            m = await asyncio.wait_for(ws.recv(), timeout=remaining)
        except asyncio.TimeoutError:
            break
        got.append(m)
    return got


async def _collect_point_clouds(ws, n: int, timeout: float = 8.0) -> list:
    got = []
    deadline = time.monotonic() + timeout
    while len([m for m in got if _is_point_cloud(m)]) < n:
        remaining = deadline - time.monotonic()
        assert remaining > 0, "timed out waiting for point clouds"
        m = await asyncio.wait_for(ws.recv(), timeout=remaining)
        got.append(m)
    return [m for m in got if _is_point_cloud(m)][:n]


async def _wait_for_state(ws, timeout: float = 8.0, **want) -> dict:
    """Drain messages until a `state` echo matches every `want` key/value --
    the CLIENT-observable "the swap has landed" boundary (issue #101's
    `stream_generation`/`stream_ready` fields exist for exactly this).

    Sending a control message and then immediately asserting on the next few
    point clouds is racy: the broadcaster may have already queued/sent
    several PRE-swap frames before the swap (which runs via
    `asyncio.to_thread`) actually completes, and those are still sitting in
    the client's receive backlog in front of anything new. Waiting for the
    matching `state` echo finds the actual boundary the client can observe,
    so "never shows live/stale data AFTER the reset barrier" is testable
    without a race."""
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        assert remaining > 0, f"timed out waiting for state matching {want}"
        m = await asyncio.wait_for(ws.recv(), timeout=remaining)
        if not isinstance(m, str):
            continue
        d = json.loads(m)
        if d.get("type") != "state":
            continue
        if all(d.get(k) == v for k, v in want.items()):
            return d


def test_view_mode_shows_only_capture_data_never_live(tmp_path):
    """The core issue #101 regression: after `load_capture`, every point
    cloud must reflect the CAPTURE's depth, never the live device's -- even
    though the live reader was running (and streaming distinctive frames)
    right up until the swap."""
    import websockets

    ctrl, captures_dir = _build_view_app_state(tmp_path)
    cap = captures_dir / "room.bin"
    _make_capture(cap, CAPTURE_DEPTH_MM, n_frames=60)

    port = _free_port()
    server, thread = _start_server(web.app, port)
    uri = f"ws://127.0.0.1:{port}/ws"
    try:
        async def run():
            async with websockets.connect(uri) as ws:
                # Sanity: the live source really is streaming before the swap.
                live_pcs = await _collect_point_clouds(ws, 3)
                for pc in live_pcs:
                    z = _pc_z_values(pc)
                    assert z.mean() > 5.0, "expected LIVE depth before the swap"

                await ws.send(json.dumps({"type": "load_capture", "name": "room.bin"}))
                # The boundary the client can actually observe -- everything
                # AFTER this `state` echo must be the capture, never live.
                await _wait_for_state(ws, source="view", selected_capture="room.bin",
                                      stream_ready=True)

                cap_pcs = await _collect_point_clouds(ws, 10)
                for pc in cap_pcs:
                    z = _pc_z_values(pc)
                    assert z.mean() < 3.0, \
                        f"post-load_capture point cloud shows non-capture depth (mean z={z.mean()})"
        asyncio.run(asyncio.wait_for(run(), timeout=30.0))
    finally:
        server.should_exit = True
        thread.join(timeout=10.0)
        ctrl.close()


def test_entering_view_with_no_capture_emits_no_display_data(tmp_path):
    """`set_source view` with nothing selected must show the browser
    (`captures`/`state` keep flowing, `stream_ready: false`) but emit no
    point-cloud / IR / sensor / metrics data, even though the live reader
    keeps running underneath it."""
    import websockets

    ctrl, captures_dir = _build_view_app_state(tmp_path)
    port = _free_port()
    server, thread = _start_server(web.app, port)
    uri = f"ws://127.0.0.1:{port}/ws"
    try:
        async def run():
            async with websockets.connect(uri) as ws:
                # Drain the initial live point clouds so they don't
                # contaminate the "nothing after this" window below.
                await _collect_point_clouds(ws, 2)
                await ws.send(json.dumps({"type": "set_source", "source": "view"}))

                # Confirm the state echo landed and reports not-ready.
                deadline = time.monotonic() + 5.0
                saw_not_ready = False
                msgs = []
                while time.monotonic() < deadline:
                    m = await asyncio.wait_for(ws.recv(), timeout=deadline - time.monotonic())
                    msgs.append(m)
                    if isinstance(m, str):
                        d = json.loads(m)
                        if d.get("type") == "state" and d.get("source") == "view":
                            assert d.get("stream_ready") is False
                            saw_not_ready = True
                            break
                assert saw_not_ready, msgs

                # Now assert an ABSENCE over a generous window: no point
                # cloud, IR image, sensor, or metrics message -- even though
                # the live reader is still producing frames underneath.
                more = await _recv_for(ws, 1.5)
                for m in more:
                    if _is_point_cloud(m):
                        pytest.fail("point_cloud emitted while View has no selected capture")
                    if isinstance(m, (bytes, bytearray)) and len(m) >= 4:
                        tag = int.from_bytes(m[:4], "little")
                        assert tag != web.TAG_IR_IMAGE, "IR emitted while browsing View"
                    if isinstance(m, str):
                        d = json.loads(m)
                        assert d.get("type") != "sensor", "sensor emitted while browsing View"
                        assert d.get("type") != "metrics", "metrics emitted while browsing View"
        asyncio.run(asyncio.wait_for(run(), timeout=30.0))
    finally:
        server.should_exit = True
        thread.join(timeout=10.0)
        ctrl.close()


def test_returning_to_live_restores_live_updates(tmp_path):
    import websockets

    ctrl, captures_dir = _build_view_app_state(tmp_path)
    cap = captures_dir / "room.bin"
    _make_capture(cap, CAPTURE_DEPTH_MM, n_frames=60)

    port = _free_port()
    server, thread = _start_server(web.app, port)
    uri = f"ws://127.0.0.1:{port}/ws"
    try:
        async def run():
            async with websockets.connect(uri) as ws:
                await _collect_point_clouds(ws, 2)   # live, before the swap
                await ws.send(json.dumps({"type": "load_capture", "name": "room.bin"}))
                await _wait_for_state(ws, source="view", selected_capture="room.bin",
                                      stream_ready=True)
                cap_pcs = await _collect_point_clouds(ws, 5)
                for pc in cap_pcs:
                    assert _pc_z_values(pc).mean() < 3.0

                await ws.send(json.dumps({"type": "go_live"}))
                await _wait_for_state(ws, source="live", stream_ready=True)
                live_pcs = await _collect_point_clouds(ws, 5)
                for pc in live_pcs:
                    assert _pc_z_values(pc).mean() > 5.0, \
                        "go_live must restore live point clouds, not leave the capture's"
        asyncio.run(asyncio.wait_for(run(), timeout=30.0))
    finally:
        server.should_exit = True
        thread.join(timeout=10.0)
        ctrl.close()
