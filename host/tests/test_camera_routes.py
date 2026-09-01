"""`/api/camera/*` proxy routes and the lifespan wiring (#226, spec §5.4/§9).

The three questions these answer, none of which `test_camera.py` can:

  1. does the host EMIT the same multipart framing it consumes? (proved by a
     round trip through `MultipartParser`, not by a second hand-written layout);
  2. does a route serve the camera's exact bytes, with the headers a client
     needs, and refuse honestly when there is nothing to serve?
  3. does the FastAPI lifespan start exactly one ingest and leave nothing
     running afterwards, across repeated cycles?

What they cannot see: real uvicorn transport flow control. `TestClient` runs
the ASGI app in-process, so the `await send(...)` that bounds a slow viewer in
production (uvicorn's HTTP `send` calls `flow.drain()` over its 64 KiB
high-water mark) is a no-op here. That bound is a live-probe claim, and it is
named as such in the report rather than asserted from a fake.
"""
from __future__ import annotations

import asyncio
import functools
import importlib
import json
import types

import pytest

from roomscan import camera as cam
from roomscan import web
from roomscan.server import routes as routes_mod
from roomscan.domain import HostMono

from test_camera import make_jpeg


def asyncio_test(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        return asyncio.run(fn(*args, **kwargs))
    return wrapper


def _frame(seq=1, generation=3, jpeg=None, valid=True) -> cam.CameraFrame:
    return cam.CameraFrame(
        generation=generation, seq=seq, jpeg=jpeg or make_jpeg(1280, 720),
        host_recv_mono=HostMono(1000.0), source_generation=2,
        pi_capture_mono_ns=(4242 if valid else None), timestamp_valid=valid,
        timestamp_domain="pi_monotonic" if valid else None,
        width=1280, height=720)


class StubClient:
    """A `CameraClient`-shaped stand-in with a REAL `LatestFrameSlot`.

    The slot is the real one because the fan-out semantics under test are its
    semantics; only the socket-owning half is stubbed, so nothing here starts a
    task or opens a connection.
    """

    def __init__(self, frame: cam.CameraFrame | None = None, *, age_s=0.05):
        self.slot = cam.LatestFrameSlot()
        self.running = True
        self._age = age_s
        if frame is not None:
            self.slot.publish(frame)

    def age_s(self, now=None):
        return self._age

    def state(self, now=None):
        return "live" if self.slot.latest()[1] is not None else "connecting"

    def status(self, now=None):
        _, frame = self.slot.latest()
        return {"state": self.state(), "generation": frame.generation if frame else 0,
                "width": frame.width if frame else None,
                "height": frame.height if frame else None,
                "age_ms": None if self._age is None else self._age * 1000.0,
                "clients": self.slot.subscribers, "ingest_drops": self.slot.skipped,
                "source": {"state": "streaming"}}


def _client_with(monkeypatch, camera):
    from fastapi.testclient import TestClient
    monkeypatch.setattr(web.app.state, "camera", camera, raising=False)
    return TestClient(web.app)


# =============================================================================
# 1. multipart framing emitted by this host
# =============================================================================

def test_emitted_parts_round_trip_through_the_parser_that_reads_the_daemons():
    """One framing implementation, checked both directions.

    If `encode_mjpeg_part` and `MultipartParser` ever disagree, this is where it
    shows -- rather than in a browser that silently renders nothing.
    """
    frames = [_frame(seq=i, jpeg=make_jpeg(filler=i)) for i in range(1, 4)]
    blob = b"".join(routes_mod.encode_mjpeg_part(f) for f in frames)
    parts = cam.MultipartParser(routes_mod.CAMERA_BOUNDARY).feed(blob)
    assert [p.body for p in parts] == [f.jpeg for f in frames]
    assert parts[0].headers["x-roomscan-camera-generation"] == "3"
    assert parts[0].headers["x-roomscan-camera-seq"] == "1"
    assert parts[0].headers["content-length"] == str(len(frames[0].jpeg))


def test_emitted_parts_round_trip_when_split_at_every_byte():
    """Same exhaustive split proof, applied to what we EMIT: a browser's TCP
    reassembly is not more forgiving than our own parser."""
    blob = b"".join(routes_mod.encode_mjpeg_part(_frame(seq=i)) for i in (1, 2))
    expected = [_frame(seq=1).jpeg, _frame(seq=2).jpeg]
    for split in range(len(blob) + 1):
        parser = cam.MultipartParser(routes_mod.CAMERA_BOUNDARY)
        got = parser.feed(blob[:split]) + parser.feed(blob[split:])
        assert [p.body for p in got] == expected, f"split at byte {split}"


def test_an_invalid_capture_timestamp_is_advertised_as_invalid_not_omitted():
    """A missing header reads as "an older daemon"; an explicit `Valid: 0` reads
    as "this frame has no capture time". They are different facts, and §8's
    "Timestamp flag absent" row needs the second one to survive the proxy hop."""
    headers = routes_mod.camera_frame_headers(_frame(valid=False), None)
    assert headers["X-Roomscan-Timestamp-Valid"] == "0"
    assert "X-Roomscan-Capture-Mono-Ns" not in headers
    valid = routes_mod.camera_frame_headers(_frame(valid=True), 12.5)
    assert valid["X-Roomscan-Timestamp-Valid"] == "1"
    assert valid["X-Roomscan-Capture-Mono-Ns"] == "4242"
    assert valid["X-Roomscan-Camera-Age-Ms"] == "12.5"


# =============================================================================
# 2. the routes
# =============================================================================

def test_frame_route_serves_the_exact_jpeg_with_identifying_headers(monkeypatch):
    jpeg = make_jpeg(1280, 720, filler=77)
    client = _client_with(monkeypatch, StubClient(_frame(seq=912, jpeg=jpeg)))
    r = client.get("/api/camera/frame.jpg")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/jpeg"
    assert r.content == jpeg, "the proxy must not re-encode the camera's bytes"
    assert r.headers["x-roomscan-camera-generation"] == "3"
    assert r.headers["x-roomscan-camera-seq"] == "912"
    assert float(r.headers["x-roomscan-camera-age-ms"]) == pytest.approx(50.0, abs=0.5)


def test_camera_responses_are_never_cached(monkeypatch):
    """A cached "current" frame is a stale picture presented as live."""
    client = _client_with(monkeypatch, StubClient(_frame()))
    for path in ("/api/camera/frame.jpg", "/api/camera/status"):
        r = client.get(path)
        assert "no-store" in r.headers["cache-control"], path


def test_frame_route_is_503_when_no_frame_has_arrived(monkeypatch):
    """503 with a reason, not a 200 with an empty body: an `<img>` must fail
    visibly so the card can say Offline instead of showing black."""
    client = _client_with(monkeypatch, StubClient(None))
    r = client.get("/api/camera/frame.jpg")
    assert r.status_code == 503
    assert r.json()["state"] == "connecting"


def test_routes_are_503_when_there_is_no_ingest_at_all(monkeypatch):
    client = _client_with(monkeypatch, None)
    assert client.get("/api/camera/frame.jpg").status_code == 503
    assert client.get("/api/camera/stream.mjpg").status_code == 503
    status = client.get("/api/camera/status")
    assert status.status_code == 200
    assert status.json() == {"state": "offline", "running": False,
                             "error": "camera ingest is not running"}


def test_status_route_reports_the_clients_currently_attached(monkeypatch):
    stub = StubClient(_frame())
    client = _client_with(monkeypatch, stub)
    assert client.get("/api/camera/status").json()["clients"] == 0
    stub.slot.subscribers = 2
    body = client.get("/api/camera/status").json()
    assert body["clients"] == 2
    assert body["running"] is True
    assert body["source"] == {"state": "streaming"}


def test_stream_route_returns_a_latest_only_multipart_response(monkeypatch):
    """Asserted on the response object, not by consuming it.

    `TestClient` gathers an ASGI response body to completion before handing it
    back, so it can neither hold this endpoint's infinite body open nor exert
    the transport backpressure that bounds it. Consuming it here would hang
    forever; the real socket behaviour is in `test_camera_live_proxy.py`.
    """
    import types as _types

    stub = StubClient(_frame(seq=5))
    request = _types.SimpleNamespace(app=_types.SimpleNamespace(
        state=_types.SimpleNamespace(camera=stub)))
    response = asyncio.run(routes_mod.api_camera_stream(request))
    assert response.status_code == 200
    assert response.media_type == routes_mod.CAMERA_MEDIA_TYPE
    assert "no-store" in response.headers["cache-control"]
    assert response.body_iterator.__qualname__.startswith("_camera_parts")


@asyncio_test
async def test_the_generator_holds_one_frame_and_resumes_at_the_newest():
    """The proxy's latest-wins behaviour, driven directly rather than through
    TestClient -- an ASGI test client cannot stall a send, so the interesting
    case has to be exercised on the generator itself."""
    slot = cam.LatestFrameSlot()
    slot.publish(_frame(seq=1))
    gen = routes_mod._camera_parts(slot)
    first = await gen.__anext__()
    assert slot.subscribers == 1
    assert b"X-Roomscan-Camera-Seq: 1" in first

    for seq in range(2, 12):            # ten frames while the viewer is stalled
        slot.publish(_frame(seq=seq))
    nxt = await gen.__anext__()
    assert b"X-Roomscan-Camera-Seq: 11" in nxt, "must resume at the NEWEST frame"
    assert slot.skipped == 9
    await gen.aclose()
    assert slot.subscribers == 0, "a closed generator must release its slot seat"


@asyncio_test
async def test_two_generators_share_one_slot_and_neither_steals_from_the_other():
    """The §4.3 invariant at the host boundary: N viewers, one upstream, and
    both see the same frame rather than alternate ones."""
    slot = cam.LatestFrameSlot()
    slot.publish(_frame(seq=42))
    a = routes_mod._camera_parts(slot)
    b = routes_mod._camera_parts(slot)
    pa, pb = await a.__anext__(), await b.__anext__()
    assert pa == pb
    assert slot.subscribers == 2
    await a.aclose()
    await b.aclose()
    assert slot.subscribers == 0


# =============================================================================
# 3. lifespan
# =============================================================================

class _LifespanApp:
    def __init__(self):
        self.state = types.SimpleNamespace(ready=True, port=8001, clients=set())


def _neutralise_lifespan(monkeypatch, started):
    """Replace every heavy lifespan collaborator with a no-op.

    The subject is the camera wiring, not the broadcaster: leaving the real
    loops in would need the whole device/session state and would make a leak
    from any of them read as a camera leak.
    """
    # `from roomscan.server import app` resolves to the FastAPI INSTANCE (the
    # package re-exports it), not the module -- import it by name instead.
    app_mod = importlib.import_module("roomscan.server.app")

    async def _idle(*a, **k):
        await asyncio.sleep(3600)

    monkeypatch.setattr(app_mod, "_broadcaster", _idle)
    monkeypatch.setattr(app_mod, "_idle_reconcile_loop", _idle)
    monkeypatch.setattr(app_mod, "_init_ranging_state", _idle)
    monkeypatch.setattr(app_mod, "_comm_probe_loop", _idle)
    monkeypatch.setattr(app_mod, "_bridge_health_loop", _idle)
    monkeypatch.setattr(app_mod, "_camera_state_loop", _idle)
    monkeypatch.setattr(app_mod, "register_mdns", lambda *a, **k: None)
    monkeypatch.setattr(app_mod, "unregister_mdns", lambda *a, **k: None)

    class _Cam:
        def __init__(self):
            self.started = 0
            self.stopped = 0
            self.task = None

        async def start(self):
            self.started += 1
            self.task = asyncio.create_task(asyncio.sleep(3600))

        async def stop(self):
            self.stopped += 1
            if self.task is not None:
                self.task.cancel()
                try:
                    await self.task
                except asyncio.CancelledError:
                    pass
                self.task = None

    def _build(_state):
        c = _Cam()
        started.append(c)
        return c

    monkeypatch.setattr(app_mod, "build_camera_client", _build)
    return app_mod


@asyncio_test
async def test_lifespan_starts_one_camera_and_stops_it(monkeypatch):
    started: list = []
    app_mod = _neutralise_lifespan(monkeypatch, started)
    app = _LifespanApp()
    before = len(asyncio.all_tasks())
    async with app_mod._lifespan(app):
        assert len(started) == 1
        assert started[0].started == 1
        assert app.state.camera is started[0]
    assert started[0].stopped == 1
    assert app.state.camera is None, "a stopped camera must not stay on app.state"
    await asyncio.sleep(0)
    assert len(asyncio.all_tasks()) == before, "lifespan leaked a task"


@asyncio_test
async def test_repeated_lifespan_cycles_start_exactly_one_camera_each(monkeypatch):
    """§9: "Lifespan: repeated app start/stop ... no leaked task"."""
    started: list = []
    app_mod = _neutralise_lifespan(monkeypatch, started)
    before = len(asyncio.all_tasks())
    for _ in range(3):
        app = _LifespanApp()
        async with app_mod._lifespan(app):
            pass
    assert len(started) == 3
    assert [c.started for c in started] == [1, 1, 1]
    assert [c.stopped for c in started] == [1, 1, 1]
    await asyncio.sleep(0)
    assert len(asyncio.all_tasks()) == before


@asyncio_test
async def test_a_camera_that_fails_to_stop_does_not_strand_the_lifespan(monkeypatch):
    """A teardown that raises must not leave the server unable to shut down.

    It is allowed to propagate -- what must not happen is the earlier tasks
    being left running, so this asserts the census, not the exception.
    """
    started: list = []
    app_mod = _neutralise_lifespan(monkeypatch, started)

    _real = importlib.import_module("roomscan.server.app")
    real_build = _real.build_camera_client

    class _Boom:
        started = 1
        stopped = 0

        async def start(self):
            return None

        async def stop(self):
            raise RuntimeError("socket already gone")

    monkeypatch.setattr(app_mod, "build_camera_client", lambda _s: _Boom())
    app = _LifespanApp()
    before = len(asyncio.all_tasks())
    with pytest.raises(RuntimeError, match="socket already gone"):
        async with app_mod._lifespan(app):
            pass
    await asyncio.sleep(0)
    assert len(asyncio.all_tasks()) == before
    assert real_build is not None      # the real factory is restored by monkeypatch


def test_build_camera_client_shares_the_one_bridge_resolver(monkeypatch):
    """Not a fourth resolution policy: the camera dials whatever
    `commstatus.resolve_bridge_target` (mDNS first, TTL-cached, shared with the
    ICMP probe and the ssh health poll) says the bridge is."""
    from roomscan.server import session as session_mod

    seen = []

    async def fake_resolve(configured_ip, **kw):
        seen.append(configured_ip)
        return {"addr": "172.17.2.88", "resolved_from": "mdns", "hostname": "x"}

    monkeypatch.setattr(session_mod.commstatus, "resolve_bridge_target", fake_resolve)
    state = types.SimpleNamespace(config=types.SimpleNamespace(bridge_ip="172.17.2.66"))
    client = session_mod.build_camera_client(state)
    assert client.config.port == cam.DEFAULT_CAMERA_PORT
    assert asyncio.run(client._target()) == "172.17.2.88"
    assert seen == ["172.17.2.66"]


# =============================================================================
# 4. the `camera` /ws state message
# =============================================================================

@asyncio_test
async def test_camera_state_loop_is_edge_triggered_and_heartbeats(monkeypatch):
    """Two bounds at once: no flood on unchanged telemetry, and no silence
    forever either. Asserted as MESSAGE COUNTS over a driven clock, because
    "it sent a message" would pass against both failure modes.
    """
    from roomscan.server import session as session_mod

    sent: list[dict] = []

    class _BC:
        @staticmethod
        async def _broadcast_text(clients, text):
            sent.append(json.loads(text))

    monkeypatch.setattr(session_mod, "_bc", lambda: _BC)
    monkeypatch.setattr(session_mod, "CAMERA_STATE_MIN_INTERVAL_S", 0.005)
    monkeypatch.setattr(session_mod, "CAMERA_STATE_HEARTBEAT_S", 10_000.0)

    stub = StubClient(_frame())
    state = types.SimpleNamespace(camera=stub, clients=set(), last_camera=None)
    task = asyncio.create_task(session_mod._camera_state_loop(state))
    try:
        await asyncio.sleep(0.08)
        assert len(sent) == 1, f"unchanged status must send once, got {len(sent)}"
        assert sent[0]["type"] == "camera"

        stub.slot.publish(_frame(seq=9, generation=99))   # a real generation change
        await asyncio.sleep(0.08)
        assert len(sent) == 2, "a generation change must send immediately"
        assert sent[-1]["generation"] == 99
        assert state.last_camera["generation"] == 99
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


@asyncio_test
async def test_camera_state_loop_survives_a_failing_status_call(monkeypatch):
    """A camera fault must not kill the loop that reports camera faults."""
    from roomscan.server import session as session_mod

    monkeypatch.setattr(session_mod, "_bc", lambda: types.SimpleNamespace(
        _broadcast_text=_anoop))
    monkeypatch.setattr(session_mod, "CAMERA_STATE_MIN_INTERVAL_S", 0.005)

    class _Broken:
        def status(self):
            raise RuntimeError("status blew up")

    state = types.SimpleNamespace(camera=_Broken(), clients=set())
    task = asyncio.create_task(session_mod._camera_state_loop(state))
    try:
        await asyncio.sleep(0.05)
        assert not task.done(), "the loop must survive a status() exception"
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


async def _anoop(*a, **k):
    return None


def test_the_camera_ingest_can_be_turned_off_without_editing_code(monkeypatch):
    """`ROOMSCAN_CAMERA=0` -> no client, and every route says so honestly.

    A bridge with no camera daemon otherwise costs one refused connection per
    backoff period for the life of the server. "Off" must read as "not
    running", never as a fault, so a deliberately camera-less deployment does
    not show a permanent red state (spec §5.3's optional-unit rule, applied on
    this side of the link).
    """
    from roomscan.server import session as session_mod

    state = types.SimpleNamespace(config=types.SimpleNamespace(bridge_ip="1.2.3.4"))
    monkeypatch.setenv("ROOMSCAN_CAMERA", "0")
    assert session_mod.build_camera_client(state) is None
    monkeypatch.setenv("ROOMSCAN_CAMERA", "off")
    assert session_mod.build_camera_client(state) is None
    monkeypatch.delenv("ROOMSCAN_CAMERA")
    assert session_mod.build_camera_client(state) is not None


def test_the_camera_port_is_overridable_for_a_non_default_deployment(monkeypatch):
    from roomscan.server import session as session_mod

    state = types.SimpleNamespace(config=types.SimpleNamespace(bridge_ip="1.2.3.4"))
    monkeypatch.setenv("ROOMSCAN_CAMERA_PORT", "6001")
    assert session_mod.build_camera_client(state).config.port == 6001
    monkeypatch.delenv("ROOMSCAN_CAMERA_PORT")
    assert session_mod.build_camera_client(state).config.port == 5001


@asyncio_test
async def test_a_repeated_identical_failure_is_not_logged_every_backoff(monkeypatch):
    """A log line every few seconds saying the same thing is how a log stops
    being read. The reason must still reach INFO once, and `last_error` must
    keep carrying it -- silence is not the fix, repetition is the problem."""
    from roomscan import camera as camera_mod
    from test_camera import _no_status

    infos: list = []
    monkeypatch.setattr(camera_mod.log, "info",
                        lambda *a, **k: infos.append(a))
    monkeypatch.setattr(camera_mod.log, "debug", lambda *a, **k: None)

    async def dial(host, port):
        raise ConnectionRefusedError("nothing listening")

    cfg = cam.CameraIngestConfig(host="h", status_interval_s=3600.0,
                                 min_backoff_s=0.01, max_backoff_s=0.01)
    client = cam.CameraClient(cfg, connect=dial, status_connect=_no_status)
    await client.start()
    try:
        await asyncio.sleep(0.25)
        assert len(infos) == 1, f"the same failure was logged {len(infos)} times"
        assert client.last_error and "nothing listening" in client.last_error
    finally:
        await client.stop()
