"""Unit tests for the pure halves the MCP tools call.

These functions were previously unreachable except by running a CLI and reading
prose, so this is the first coverage they have had.
"""
from __future__ import annotations

import re
import struct
import zlib
from pathlib import Path


from tools.analyze_capture import MAGIC, scan
from tools.headless_doctor import Doctor

REPO = Path(__file__).resolve().parents[2]


def _frame(*, stream: int = 7, seq: int = 0, payload: bytes = b"\x01\x02\x03\x04",
           ftype: int = 1, corrupt_crc: bool = False) -> bytes:
    header = struct.pack("<4sBBBBIQHHII", MAGIC, 1, ftype, stream, 0, seq, 1000 * seq,
                         2, 2, len(payload), 0)
    body = header + payload
    crc = zlib.crc32(body)
    if corrupt_crc:
        crc ^= 0xFFFF
    return body + struct.pack("<I", crc)


# --- analyze_capture.scan ----------------------------------------------------

def test_scan_reports_a_clean_capture(tmp_path):
    p = tmp_path / "clean.bin"
    p.write_bytes(b"".join(_frame(seq=i) for i in range(5)))

    r = scan(str(p))

    assert r["frames_decoded"] == 5
    assert r["crc_failures"] == 0
    assert r["bytes_skipped"] == 0
    assert r["clean"] is True
    assert r["anomalies"] == []


def test_scan_pins_a_crc_failure_to_its_offset(tmp_path):
    good = _frame(seq=0)
    bad = _frame(seq=1, corrupt_crc=True)
    p = tmp_path / "crc.bin"
    p.write_bytes(good + bad)

    r = scan(str(p))

    assert r["crc_failures"] == 1
    fail = next(a for a in r["anomalies"] if a["kind"] == "CRC_FAIL")
    assert fail["offset"] == len(good), "CRC anomaly must carry the byte offset"
    assert fail["seq"] == 1
    assert fail["computed_crc"] != fail["wire_crc"]
    assert r["clean"] is False


def test_scan_records_a_skip_run_with_neighbouring_frames(tmp_path):
    a = _frame(seq=0)
    b = _frame(seq=1)
    p = tmp_path / "skip.bin"
    p.write_bytes(a + b"\xaa" * 64 + b)

    r = scan(str(p))

    assert r["frames_decoded"] == 2
    run = next(c for c in r["skip_context"] if c["run_len"] == 64)
    assert run["prev_frame"]["seq"] == 0
    assert run["next_frame"]["seq"] == 1
    assert run["prev_frame"]["ends_at"] == len(a)


def test_scan_flags_a_frame_truncated_at_eof(tmp_path):
    p = tmp_path / "trunc.bin"
    full = _frame(seq=0)
    p.write_bytes(full[:-6])  # lose the CRC and some payload

    r = scan(str(p))

    kinds = {a["kind"] for a in r["anomalies"]}
    assert "TRUNCATED_AT_EOF" in kinds


# --- stream continuity -------------------------------------------------------
#
# A capture can be byte-perfect and still be missing seconds of frames: that is what
# `clean` could not see, and what let three 2026-07-31 multi-room captures read as
# fine while losing 2.3-9.4% of RAW frames. `clean` and `continuity.complete` are
# deliberately separate properties -- these tests pin that separation.

def test_a_byte_clean_capture_with_lost_frames_is_not_complete(tmp_path):
    p = tmp_path / "lossy.bin"
    p.write_bytes(b"".join(_frame(seq=i) for i in [0, 1, 2, 9, 10]))

    r = scan(str(p))

    assert r["clean"] is True, "the bytes present decode fine"
    c = r["continuity"]
    assert c["complete"] is False, "but six frames never arrived"
    assert c["frames_lost"] == 6
    raw = c["streams"]["RAW_3DMD"]
    assert raw["span"] == 11 and raw["received"] == 5
    assert raw["max_gap"] == 6
    assert raw["worst_gaps"][0]["after_seq"] == 2


def test_continuity_separates_whole_group_loss_from_single_stream_loss(tmp_path):
    # seq 2 vanishes everywhere (a link outage); seq 4 loses only the big RAW
    # datagram while its small sibling arrives (fragment loss).
    frames = []
    for seq in range(6):
        if seq != 2:
            frames.append(_frame(stream=9, seq=seq))
        if seq not in (2, 4):
            frames.append(_frame(stream=7, seq=seq))
    p = tmp_path / "mixed.bin"
    p.write_bytes(b"".join(frames))

    c = scan(str(p))["continuity"]

    assert c["whole_group_lost"] == 1
    assert c["partial_group_lost"] == 1
    assert c["streams"]["RAW_3DMD"]["missing"] == 2
    assert c["streams"]["IMU_QUAT"]["missing"] == 1


def test_cadenced_streams_are_not_counted_as_gaps(tmp_path):
    # CALIB rides a 64-frame cadence, so spacing 64 is the design, not loss.
    frames = [_frame(stream=7, seq=i) for i in range(129)]
    frames += [_frame(stream=8, seq=i) for i in (0, 64, 128)]
    p = tmp_path / "calib.bin"
    p.write_bytes(b"".join(frames))

    c = scan(str(p))["continuity"]

    assert c["complete"] is True
    assert c["cadenced"]["CALIB"]["missed"] == 0
    assert c["cadenced"]["CALIB"]["spacings_seen"] == [64]


def test_a_missed_calib_retransmit_is_reported(tmp_path):
    frames = [_frame(stream=7, seq=i) for i in range(129)]
    frames += [_frame(stream=8, seq=i) for i in (0, 128)]  # the seq-64 CALIB was lost
    p = tmp_path / "calibgap.bin"
    p.write_bytes(b"".join(frames))

    c = scan(str(p))["continuity"]

    assert c["cadenced"]["CALIB"]["missed"] == 1
    assert c["complete"] is False


def test_continuity_ignores_non_data_frames(tmp_path):
    # EVENT/ACK carry their own seq space; counting them would invent a huge gap.
    p = tmp_path / "evt.bin"
    p.write_bytes(b"".join(_frame(stream=7, seq=i) for i in range(4))
                  + _frame(stream=7, seq=9999, ftype=2))

    c = scan(str(p))["continuity"]

    assert c["complete"] is True
    assert c["streams"]["RAW_3DMD"]["span"] == 4


def test_scan_omits_raw_bytes_so_results_stay_small(tmp_path):
    p = tmp_path / "big.bin"
    p.write_bytes(b"".join(_frame(seq=i, payload=b"\x00" * 4096) for i in range(20)))

    r = scan(str(p))

    assert not any(isinstance(v, (bytes, bytearray)) for v in r.values())


def test_scan_hexdump_is_rendered_text_not_bytes(tmp_path):
    p = tmp_path / "dump.bin"
    p.write_bytes(_frame(seq=0) + b"\xaa" * 80 + _frame(seq=1))

    r = scan(str(p), dump_bytes=48)

    run = next(c for c in r["skip_context"] if "hexdump" in c)
    assert isinstance(run["hexdump"]["text"], str)
    assert "aa" in run["hexdump"]["text"]


def test_scan_respects_the_zero_scan_frame_budget(tmp_path):
    p = tmp_path / "zeros.bin"
    p.write_bytes(b"".join(_frame(seq=i, payload=b"\x00" * 256) for i in range(10)))

    r = scan(str(p), min_zero_run=16, zero_scan_frames=3)

    assert len(r["raw_zero_runs"]) == 3
    assert r["raw_zero_runs"][0]["zero_runs"], "a 256-byte zero payload must register"


# --- headless_doctor.Doctor --------------------------------------------------

def test_doctor_accumulates_structured_results_when_quiet(capsys):
    d = Doctor(quiet=True)
    failed = d.run(build=False, net=False)

    assert capsys.readouterr().out == "", "quiet=True must print nothing"
    assert isinstance(failed, int)
    assert d.results, "every check must be recorded"
    assert {r["status"] for r in d.results} <= {"pass", "fail", "warn"}
    assert all(r["check"] for r in d.results)


def test_doctor_still_prints_by_default(capsys):
    Doctor().run(build=False, net=False)

    out = capsys.readouterr().out
    assert "roomscan headless-host doctor" in out
    assert "PASS" in out or "FAIL" in out


def test_doctor_failed_count_matches_recorded_failures():
    d = Doctor(quiet=True)
    failed = d.run(build=False, net=False)

    assert failed == sum(1 for r in d.results if r["status"] == "fail")


def test_doctor_failures_carry_a_fix():
    d = Doctor(quiet=True)
    d.run(build=False, net=False)

    for r in d.results:
        if r["status"] == "fail":
            assert r["fix"], f"{r['check']} failed without telling the user how to fix it"


# --- survey (capture_list's stream detection) --------------------------------

def test_survey_detects_stream_presence(tmp_path):
    from roomscan.mcp_server.tools_data import _survey

    p = tmp_path / "mixed.bin"
    p.write_bytes(_frame(stream=7, seq=0) + _frame(stream=9, seq=1) + _frame(stream=7, seq=2))

    s = _survey(p)

    assert s["streams"]["RAW_3DMD"] == 2
    assert s["streams"]["IMU_QUAT"] == 1
    assert s["frames_sampled"] == 3


def test_survey_is_bounded_by_max_frames(tmp_path):
    from roomscan.mcp_server.tools_data import _survey

    p = tmp_path / "many.bin"
    p.write_bytes(b"".join(_frame(seq=i) for i in range(50)))

    assert _survey(p, max_frames=10)["frames_sampled"] == 10


def test_only_ok_counts_as_a_successful_command():
    """`rig_command` first treated everything but "error" as success.

    A device that never answered came back status="timeout" and was reported as
    ok=True (seen on-rig 2026-07-29 against a wedged board). Pin the vocabulary to
    web.py's `_cmd_status` so a new status can't silently be read as success.
    """
    import inspect

    from roomscan import web

    src = inspect.getsource(web._cmd_status)
    statuses = set(re.findall(r'return "(\w+)"', src))

    assert statuses == {"ok", "error", "busy", "timeout"}, (
        f"web._cmd_status vocabulary changed to {statuses}; review rig_command's "
        "success test in roomscan/mcp_server/tools_rig.py")


def test_stream_map_matches_the_protocol_enum():
    """A stream missing here surfaces as a bare number in capture_list/analyze."""
    from roomscan.protocol import StreamId
    from tools.analyze_capture import STREAMS

    assert {int(s): s.name for s in StreamId} == STREAMS


def test_tool_modules_do_not_import_open3d_eagerly():
    """open3d import costs seconds; the server must not pay it on every start.

    Runs in a subprocess: asserting on this interpreter's sys.modules would be
    meaningless (open3d is already imported by other tests) and unpicking it would
    corrupt state for everything that runs afterwards.
    """
    import subprocess
    import sys

    probe = (
        "import sys;"
        "import roomscan.mcp_server.server as s; s.build();"
        "mods=[m for m in ('open3d','kiss_icp') if m in sys.modules];"
        "print(','.join(mods))"
    )
    r = subprocess.run([sys.executable, "-c", probe], cwd=str(REPO / "host"),
                       capture_output=True, text=True,
                       env={"PYTHONPATH": "src:.", "PATH": "/usr/bin:/bin"})
    assert r.returncode == 0, f"probe failed: {r.stderr[-800:]}"
    assert r.stdout.strip() == "", f"heavy modules imported at server build: {r.stdout!r}"


def test_cdp_browser_disables_the_http_cache_on_start():
    """`renavigate=True` is a navigation, not a cache bypass.

    This browser only ever looks at files edited seconds ago, so a cache hit is a
    silent stale read that reports success (2026-07-30: modal copy verified twice
    against a cached page). Asserting on the CDP command sequence rather than on
    a screenshot, because the failure mode is invisible in the pixels.
    """
    import asyncio

    from roomscan.mcp_server.session import CdpSession

    sent: list[tuple[str, dict | None]] = []
    s = CdpSession()

    async def fake_cmd(method, params=None):
        sent.append((method, params))
        return {}

    s.cmd = fake_cmd                      # type: ignore[method-assign]
    asyncio.run(s._disable_http_cache())

    assert ("Network.enable", {}) in [(m, p or {}) for m, p in sent]
    assert ("Network.setCacheDisabled", {"cacheDisabled": True}) in sent


def test_cdp_cache_disable_is_non_fatal_on_an_old_target():
    """A CDP build without the Network domain must still yield a usable browser."""
    import asyncio

    from roomscan.mcp_server.session import CdpSession

    s = CdpSession()

    async def boom(method, params=None):
        raise RuntimeError(f"CDP {method}: not supported")

    s.cmd = boom                          # type: ignore[method-assign]
    asyncio.run(s._disable_http_cache())  # must not raise


# --- ui_screenshot viewport resize (#168) ------------------------------------
#
# `ui_screenshot` calls `browser.start(width=..., height=...)` on EVERY call, not
# just the first (see tools_ui.py). Both browser backends used to return early
# once already launched, so a second call at a different size left the actual
# viewport (and `window.innerWidth/Height`) at whatever the browser launched
# with -- `width`/`height` were silently inert past the first screenshot. These
# assert the resize command actually reaches the driver on a REPEAT call, which
# is exactly the call the bug made a no-op; reverting the `session.py` fix turns
# both red (see PR description for the exact failure).

def test_cdp_session_reapplies_viewport_on_a_repeat_start_call():
    import asyncio

    from roomscan.mcp_server.session import CdpSession

    sent: list[tuple[str, dict | None]] = []
    s = CdpSession()

    async def fake_cmd(method, params=None):
        sent.append((method, params))
        return {}

    s.cmd = fake_cmd                      # type: ignore[method-assign]
    s._ws = object()                      # simulate an already-launched browser

    asyncio.run(s.start(width=820, height=700))

    assert ("Emulation.setDeviceMetricsOverride",
            {"width": 820, "height": 700, "deviceScaleFactor": 1, "mobile": False}) in sent, (
        "start() on an already-running CdpSession must still push the requested "
        f"viewport size; commands sent were {sent!r}")


def test_cdp_session_resizes_to_each_of_two_different_requests():
    """The exact reproduction from the issue: two different sizes, back to back."""
    import asyncio

    from roomscan.mcp_server.session import CdpSession

    sizes: list[tuple[int, int]] = []
    s = CdpSession()

    async def fake_cmd(method, params=None):
        if method == "Emulation.setDeviceMetricsOverride":
            sizes.append((params["width"], params["height"]))
        return {}

    s.cmd = fake_cmd                      # type: ignore[method-assign]
    s._ws = object()

    asyncio.run(s.start(width=1100, height=560))
    asyncio.run(s.start(width=820, height=700))

    assert sizes == [(1100, 560), (820, 700)], (
        "each start() call must resize to ITS OWN request, not repeat the first")


def test_playwright_session_reapplies_viewport_on_a_repeat_start_call():
    import asyncio

    from roomscan.mcp_server.session import PlaywrightSession

    calls: list[dict] = []
    s = PlaywrightSession()

    class _FakePage:
        async def set_viewport_size(self, size):
            calls.append(size)

    s._page = _FakePage()                 # simulate an already-launched page

    asyncio.run(s.start(width=1100, height=560))

    assert calls == [{"width": 1100, "height": 560}], (
        "start() on an already-running PlaywrightSession must call "
        f"set_viewport_size with the requested size; got {calls!r}")


def test_web_ui_shot_cli_sends_the_device_metrics_override_before_navigate(tmp_path):
    """`host/tools/web_ui_shot.py` launches a fresh Chrome per invocation, so it
    does not have the repeat-call bug above -- but the issue explicitly asks
    whether `height` reaches the raw-CDP fallback there too. It does: assert the
    CDP command sequence carries both dimensions and precedes the navigate.
    """
    import argparse
    import asyncio
    import base64
    import json as _json

    from tools import web_ui_shot

    class _FakeCdpWs:
        def __init__(self):
            self.sent: list[dict] = []

        async def send(self, msg):
            self.sent.append(_json.loads(msg))

        async def recv(self):
            last = self.sent[-1]
            result: dict = {}
            if last.get("method") == "Page.captureScreenshot":
                result = {"data": base64.b64encode(b"fake-png-bytes").decode()}
            return _json.dumps({"id": last["id"], "result": result})

        async def __aenter__(self):
            return self

        async def __aexit__(self, *exc):
            return False

    fake_ws = _FakeCdpWs()

    def fake_connect(url, max_size=None):
        return fake_ws  # used as `async with websockets.connect(...)`, not awaited

    orig_connect = web_ui_shot.websockets.connect
    web_ui_shot.websockets.connect = fake_connect  # type: ignore[assignment]
    try:
        args = argparse.Namespace(url="http://localhost:8000/x", out=str(tmp_path / "shot.png"),
                                  width=1100, height=560, settle=0.0)
        asyncio.run(web_ui_shot._run("ws://fake", args, []))
    finally:
        web_ui_shot.websockets.connect = orig_connect

    methods = [m.get("method") for m in fake_ws.sent]
    override = next(m for m in fake_ws.sent
                    if m.get("method") == "Emulation.setDeviceMetricsOverride")
    assert override["params"] == {"width": 1100, "height": 560,
                                  "deviceScaleFactor": 1, "mobile": False}
    assert methods.index("Emulation.setDeviceMetricsOverride") < methods.index("Page.navigate"), (
        "the viewport must be sized before navigation, or the first paint happens "
        "at the wrong size")


# --- RigSession /ws-mesh (BUG-061 A6) ----------------------------------------

class _FakeMeshWs:
    """Enough of a websockets connection for `_pump_mesh` to drive."""

    def __init__(self, frames):
        self._frames = frames
        self.sent: list[str] = []

    def __aiter__(self):
        return self._gen()

    async def _gen(self):
        for f in self._frames:
            yield f

    async def send(self, msg):
        self.sent.append(msg)


def _mesh_frame(seq: int, tag: int = 3, tail: bytes = b"\x00" * 16) -> bytes:
    """A MESH (tag 3) packet: u32 tag, u32 mesh_seq at byte offset 4 (per
    docs/web-protocol.md's `/ws-mesh` credit contract), then arbitrary payload.
    """
    return struct.pack("<II", tag, seq) + tail


def test_rig_session_auto_acks_mesh_frames():
    """`_pump_mesh` must count tag 3 into `binary_counts` (so `rig_status`'s
    `binary_tags_seen`/`streaming` keep seeing MESH after it moves off `/ws`)
    and immediately ack with the seq carried at byte offset 4.
    """
    import asyncio
    import json as _json

    from roomscan.mcp_server.session import RigSession

    fake = _FakeMeshWs([_mesh_frame(seq=42)])
    rig = RigSession()
    rig._ws_mesh = fake

    asyncio.run(rig._pump_mesh())

    assert rig.binary_counts.get(3) == 1
    assert len(fake.sent) == 1
    assert _json.loads(fake.sent[0]) == {"type": "mesh_ack", "seq": 42}


def test_rig_session_acks_each_frame_in_sequence():
    """Multiple mesh frames each get their own ack with the matching seq."""
    import asyncio
    import json as _json

    from roomscan.mcp_server.session import RigSession

    fake = _FakeMeshWs([_mesh_frame(seq=1), _mesh_frame(seq=2), _mesh_frame(seq=5)])
    rig = RigSession()
    rig._ws_mesh = fake

    asyncio.run(rig._pump_mesh())

    assert rig.binary_counts.get(3) == 3
    acked_seqs = [_json.loads(m)["seq"] for m in fake.sent]
    assert acked_seqs == [1, 2, 5]


def test_rig_session_mesh_connect_failure_is_non_fatal():
    """`/ws-mesh` is best-effort (an older server has no such route): a failed
    connect must leave `mesh_connect_error` set but must not raise and must not
    touch the main-session state.
    """
    import asyncio

    from roomscan.mcp_server.session import RigSession

    rig = RigSession(url="ws://127.0.0.1:1/ws")  # port 1: nothing listens, refused fast
    asyncio.run(rig._connect_mesh(timeout=1.0))

    assert rig._ws_mesh is None
    assert rig.mesh_connected is False
    assert rig.mesh_connect_error is not None
    assert "ws-mesh" in rig.mesh_connect_error


def test_rig_session_stays_functional_when_ws_mesh_is_unavailable(monkeypatch):
    """End-to-end via `connect()`: the main `/ws` socket must come up and stay
    usable even when `/ws-mesh` is unreachable -- graceful degradation is the
    whole point of making the mesh socket best-effort (BUG-061 A6).
    """
    import asyncio

    import websockets

    from roomscan.mcp_server.session import RigSession

    class _FakeMainWs:
        def __aiter__(self):
            return self._gen()

        async def _gen(self):
            # Never terminates on its own -- mirrors a live connection that's
            # still open, so the reader task stays pending (not `.done()`).
            await asyncio.Future()
            yield  # pragma: no cover - unreachable, satisfies generator syntax

        async def close(self):
            pass

    call_urls: list[str] = []

    async def fake_connect(url, max_size=None):
        call_urls.append(url)
        if url.endswith("/ws-mesh"):
            raise OSError("connection refused")
        return _FakeMainWs()

    monkeypatch.setattr(websockets, "connect", fake_connect)

    rig = RigSession(url="ws://localhost:8000/ws")

    async def run():
        # Assertions must happen before the loop closes: `asyncio.run` cancels
        # any still-pending tasks (the reader, by design here) once its
        # coroutine returns, so checking `rig.connected` afterwards would see
        # a task the runner itself just tore down, not the state `connect()`
        # actually left behind.
        ok = await rig.connect(timeout=1.0)
        assert ok is True
        assert rig.connected is True
        assert rig.mesh_connected is False
        assert rig.mesh_connect_error is not None
        assert call_urls == ["ws://localhost:8000/ws", "ws://localhost:8000/ws-mesh"]
        await rig.close()

    asyncio.run(run())


# --- rig_profile / rig_imu_env_rate (plan Task 11) ---------------------------
#
# Every test here drives the real tool against a scripted `ranging` stream,
# because the whole point of these two tools is WHICH broadcast they are willing
# to accept as proof. The server re-broadcasts `ranging` on its ~4 Hz metrics
# tick, so "a message arrived saying the right thing" is not evidence that the
# command did anything -- these pin that distinction rather than the happy path.

class _FakeRig:
    """A scripted `RigSession`: `wait_for("ranging")` walks a list of broadcasts.

    `latest` is seeded independently so a test can pre-load a cached message the
    tool must refuse to count (`rig.latest` is exactly the stale-proof trap).
    """

    def __init__(self, script, latest=None):
        self.script = list(script)
        self.sent: list[dict] = []
        self.latest: dict[str, dict] = dict(latest or {})
        self.binary_counts: dict[int, int] = {}
        self.connected = True

    async def connect(self, timeout=10.0):
        return True

    async def send(self, message):
        self.sent.append(message)

    async def wait_for(self, type_, timeout=5.0):
        if self.script and self.script[0].get("type") == type_:
            msg = self.script.pop(0)
            self.latest[type_] = msg
            return msg
        return None


def _imu_env(rate_hz=0, *, pending=False, error=None, warning=None, initialized=True):
    return {"initialized": initialized, "applied_rate_hz": rate_hz,
            "coupled": rate_hz in (None, 0), "requested_rate_hz": None,
            "pending": pending, "warning": warning, "error": error}


def _applied(profile="precision", ranging_mode="precision", fps=30, exposure_ms=10,
             power_mode="ulp"):
    return {"profile": profile, "ranging_mode": ranging_mode, "fps": fps,
            "exposure_ms": exposure_ms, "power_mode": power_mode}


def _preset_applied(name):
    from roomscan import profiles

    cfg = profiles.PRESETS[profiles.STR_TO_PROFILE_ID[name]]
    return _applied(name, profiles.RANGING_MODE_TO_STR[cfg.ranging_mode], cfg.fps,
                    cfg.exposure_ms, profiles.POWER_MODE_TO_STR[cfg.power_mode])


def _ranging(applied=None, *, pending=False, error=None, transport="udp",
             measured_fps=29.9, imu_env=None, estimate=None):
    return {"type": "ranging", "transport": transport, "initialized": applied is not None,
            "applied": applied, "measured_fps": measured_fps,
            "estimate": estimate if estimate is not None else {"expected_delivered_fps": 30.0,
                                                               "warnings": [],
                                                               "transport_warning": None},
            "requested": {"kind": None, "profile": None, "manual": None},
            "pending": pending, "error": error,
            "imu_env": imu_env if imu_env is not None else _imu_env()}


def _run_profile(monkeypatch, script, latest=None, **kwargs):
    import asyncio

    from roomscan.mcp_server import tools_rig

    fake = _FakeRig(script, latest=latest)
    monkeypatch.setattr(tools_rig, "rig", fake)
    kwargs.setdefault("timeout", 2.0)
    return asyncio.run(tools_rig.rig_profile(**kwargs)), fake


def _run_rate(monkeypatch, script, latest=None, **kwargs):
    import asyncio

    from roomscan.mcp_server import tools_rig

    fake = _FakeRig(script, latest=latest)
    monkeypatch.setattr(tools_rig, "rig", fake)
    kwargs.setdefault("timeout", 2.0)
    return asyncio.run(tools_rig.rig_imu_env_rate(**kwargs)), fake


def test_rig_profile_confirms_an_exact_device_readback(monkeypatch):
    hfr = _preset_applied("high_framerate")
    r, fake = _run_profile(monkeypatch, [
        _ranging(_preset_applied("stability")),          # pre-flight
        _ranging(_preset_applied("stability"), pending=True),
        _ranging(hfr, measured_fps=45.6),
    ], profile="high_framerate")

    assert r["ok"] is True, r
    assert fake.sent == [{"type": "set_profile", "profile": "high_framerate"}]
    assert r["applied"] == hfr
    assert r["measured_fps"] == 45.6
    assert r["transport"] == "udp"
    assert r["requested"]["fps"] == 46, "the preset's own fps must be reported, not guessed"


def test_rig_profile_does_not_accept_a_stale_cached_broadcast(monkeypatch):
    """The cached `ranging` can already say the right thing and mean nothing.

    Seeded here with the exact config being requested: if the tool ever counted
    `rig.latest` as proof, this would report a successful profile change that the
    device never made.
    """
    target = _preset_applied("high_framerate")
    r, fake = _run_profile(monkeypatch,
                           [_ranging(_preset_applied("stability"))],
                           latest={"ranging": _ranging(target)},
                           profile="high_framerate")

    assert r["ok"] is False
    assert "no confirmed readback" in r["error"]
    assert fake.sent, "the command itself must still have been sent"


def test_rig_profile_times_out_when_the_command_never_settles(monkeypatch):
    r, _ = _run_profile(monkeypatch, [
        _ranging(_preset_applied("stability")),
        _ranging(_preset_applied("stability"), pending=True),
        _ranging(_preset_applied("stability"), pending=True),
    ], profile="precision")

    assert r["ok"] is False
    assert "no confirmed readback" in r["error"]
    assert r["applied"] == _preset_applied("stability")


def test_rig_profile_reports_a_device_error(monkeypatch):
    r, _ = _run_profile(monkeypatch, [
        _ranging(_preset_applied("stability")),
        _ranging(_preset_applied("stability"), pending=True),
        _ranging(_preset_applied("stability"), error="SET_RANGING_PROFILE -> BUSY"),
    ], profile="precision")

    assert r["ok"] is False
    assert r["error"] == "SET_RANGING_PROFILE -> BUSY"


def test_rig_profile_ignores_a_previous_commands_error_that_predates_the_send(monkeypatch):
    """A metrics-tick broadcast can slip in between the send and the server
    acting on it, still carrying the LAST command's error. Reporting that as
    this command's result would be a lie -- and a self-fulfilling one, since the
    tool would stop waiting for the real answer."""
    stale = "SET_RANGING_PROFILE timeout: no ACK"
    r, _ = _run_profile(monkeypatch, [
        _ranging(_preset_applied("stability"), error=stale),        # pre-flight
        _ranging(_preset_applied("stability"), error=stale),        # tick, still stale
        _ranging(_preset_applied("stability"), pending=True),
        _ranging(_preset_applied("precision")),
    ], profile="precision")

    assert r["ok"] is True, r
    assert r.get("error") is None


def test_rig_profile_refuses_while_another_command_is_pending(monkeypatch):
    r, fake = _run_profile(monkeypatch,
                           [_ranging(_preset_applied("stability"), pending=True)],
                           profile="precision")

    assert r["ok"] is False
    assert "busy" in r["error"]
    assert fake.sent == [], "a busy server must not be sent a second command"


def test_rig_profile_refuses_on_a_replay_source(monkeypatch):
    r, fake = _run_profile(monkeypatch,
                           [_ranging(_preset_applied("stability"), transport="replay")],
                           profile="precision")

    assert r["ok"] is False
    assert "replay" in r["error"]
    assert fake.sent == []


def test_rig_profile_rejects_invalid_manual_params_before_the_device(monkeypatch):
    """16 ms of exposure cannot fit a 90 fps (11111 us) frame period. The host
    model knows that, so the device must never be asked."""
    r, fake = _run_profile(monkeypatch, [_ranging(_preset_applied("stability"))],
                           ranging_mode="precision", fps=90, exposure_ms=16,
                           power_mode="regular")

    assert r["ok"] is False
    assert "does not fit" in r["error"]
    assert fake.sent == []


def test_rig_profile_refuses_an_unsupported_cdc_rate_but_force_sends_it(monkeypatch):
    from roomscan import profiles

    over = profiles.TRANSPORT_CDC_FPS_CEILING + 30
    args = dict(ranging_mode="precision", fps=over, exposure_ms=2, power_mode="regular")

    refused, fake = _run_profile(monkeypatch,
                                 [_ranging(_preset_applied("stability"), transport="cdc")],
                                 **args)
    assert refused["ok"] is False
    assert "USB CDC" in refused["error"] and "force=True" in refused["error"]
    assert refused["warnings"], "the CDC warning itself must be returned, not just refused"
    assert fake.sent == []

    applied = _applied("manual", "precision", over, 2, "regular")
    forced, fake2 = _run_profile(monkeypatch, [
        _ranging(_preset_applied("stability"), transport="cdc"),
        _ranging(_preset_applied("stability"), transport="cdc", pending=True),
        _ranging(applied, transport="cdc"),
    ], force=True, **args)
    assert forced["ok"] is True, forced
    assert fake2.sent[0]["type"] == "set_manual_params"


def test_rig_profile_reports_an_applied_mismatch_rather_than_the_request(monkeypatch):
    """The device is free to apply something else; that is a failure, and the
    result must show what it actually did."""
    other = _applied("manual", "precision", 45, 4, "regular")
    r, _ = _run_profile(monkeypatch, [
        _ranging(_preset_applied("stability")),
        _ranging(_preset_applied("stability"), pending=True),
        _ranging(other),
    ], ranging_mode="precision", fps=50, exposure_ms=4, power_mode="regular")

    assert r["ok"] is False
    assert r["applied"] == other
    assert r["requested"]["fps"] == 50


def test_rig_profile_applies_two_sequential_manual_changes(monkeypatch):
    for fps, exposure in ((40, 4), (25, 8)):
        applied = _applied("manual", "precision", fps, exposure, "regular")
        r, fake = _run_profile(monkeypatch, [
            _ranging(_preset_applied("stability")),
            _ranging(_preset_applied("stability"), pending=True),
            _ranging(applied),
        ], ranging_mode="precision", fps=fps, exposure_ms=exposure, power_mode="regular")

        assert r["ok"] is True, r
        assert fake.sent == [{"type": "set_manual_params", "ranging_mode": "precision",
                              "fps": fps, "exposure_ms": exposure, "power_mode": "regular"}]
        assert r["applied"]["fps"] == fps


def test_rig_profile_query_returns_state_without_sending_anything(monkeypatch):
    r, fake = _run_profile(monkeypatch, [_ranging(_preset_applied("precision"))])

    assert r["ok"] is True
    assert r["requested"] == {"kind": "query"}
    assert r["applied"] == _preset_applied("precision")
    assert fake.sent == []


def test_rig_profile_query_is_not_ok_before_the_first_device_readback(monkeypatch):
    r, _ = _run_profile(monkeypatch, [_ranging(None)])

    assert r["ok"] is False
    assert "initialized" in r["error"]


def test_rig_profile_rejects_a_preset_name_mixed_with_manual_fields(monkeypatch):
    r, fake = _run_profile(monkeypatch, [], profile="precision", fps=44)

    assert r["ok"] is False and fake.sent == []


def test_rig_imu_env_rate_confirms_the_applied_rate(monkeypatch):
    r, fake = _run_rate(monkeypatch, [
        _ranging(_preset_applied("precision")),
        _ranging(_preset_applied("precision"), imu_env=_imu_env(0, pending=True)),
        _ranging(_preset_applied("precision"), imu_env=_imu_env(90)),
    ], rate_hz=90)

    assert r["ok"] is True, r
    assert fake.sent == [{"type": "set_imu_env_rate", "rate_hz": 90}]
    assert r["imu_env"]["applied_rate_hz"] == 90
    assert any("sensor-hub" in w for w in r["warnings"]), \
        "90 Hz sub-samples stream 10; that must be reported, not swallowed"


def test_rig_imu_env_rate_recouples(monkeypatch):
    r, fake = _run_rate(monkeypatch, [
        _ranging(_preset_applied("precision"), imu_env=_imu_env(90)),
        _ranging(_preset_applied("precision"), imu_env=_imu_env(90, pending=True)),
        _ranging(_preset_applied("precision"), imu_env=_imu_env(0)),
    ], coupled=True)

    assert r["ok"] is True, r
    assert fake.sent == [{"type": "set_imu_env_rate", "rate_hz": 0}]
    assert r["imu_env"]["coupled"] is True


def test_rig_imu_env_rate_rejects_an_out_of_range_rate(monkeypatch):
    from roomscan import profiles

    r, fake = _run_rate(monkeypatch, [], rate_hz=profiles.IMU_ENV_RATE_MAX_HZ + 1)

    assert r["ok"] is False
    assert str(profiles.IMU_ENV_RATE_MAX_HZ) in r["error"]
    assert fake.sent == []


def test_rig_imu_env_rate_refuses_above_the_hub_cycle_when_full_env_is_required(monkeypatch):
    r, fake = _run_rate(monkeypatch, [], rate_hz=120, require_full_env=True)

    assert r["ok"] is False
    assert "sub-sampled" in r["error"]
    assert fake.sent == [], "an unreachable requirement must not reach the device"


def test_rig_imu_env_rate_does_not_accept_a_stale_cached_broadcast(monkeypatch):
    r, fake = _run_rate(monkeypatch,
                        [_ranging(_preset_applied("precision"), imu_env=_imu_env(0))],
                        latest={"ranging": _ranging(_preset_applied("precision"),
                                                    imu_env=_imu_env(30))},
                        rate_hz=30)

    assert r["ok"] is False
    assert "no confirmed readback" in r["error"]
    assert fake.sent


def test_rig_imu_env_rate_refuses_while_a_rate_command_is_pending(monkeypatch):
    r, fake = _run_rate(monkeypatch,
                        [_ranging(_preset_applied("precision"),
                                  imu_env=_imu_env(0, pending=True))],
                        rate_hz=30)

    assert r["ok"] is False and "busy" in r["error"] and fake.sent == []


def test_rig_imu_env_rate_refuses_on_a_replay_source(monkeypatch):
    r, fake = _run_rate(monkeypatch,
                        [_ranging(_preset_applied("precision"), transport="replay")],
                        rate_hz=30)

    assert r["ok"] is False and "replay" in r["error"] and fake.sent == []


def test_rig_imu_env_rate_reports_a_device_error(monkeypatch):
    r, _ = _run_rate(monkeypatch, [
        _ranging(_preset_applied("precision")),
        _ranging(_preset_applied("precision"), imu_env=_imu_env(0, pending=True)),
        _ranging(_preset_applied("precision"),
                 imu_env=_imu_env(0, error="SET_IMU_ENV_RATE -> UNKNOWN_CMD")),
    ], rate_hz=30)

    assert r["ok"] is False
    assert r["error"] == "SET_IMU_ENV_RATE -> UNKNOWN_CMD"


def test_rig_imu_env_rate_applies_two_sequential_changes(monkeypatch):
    for rate in (30, 90):
        r, fake = _run_rate(monkeypatch, [
            _ranging(_preset_applied("precision")),
            _ranging(_preset_applied("precision"), imu_env=_imu_env(0, pending=True)),
            _ranging(_preset_applied("precision"), imu_env=_imu_env(rate)),
        ], rate_hz=rate)

        assert r["ok"] is True, r
        assert fake.sent == [{"type": "set_imu_env_rate", "rate_hz": rate}]


# -- interleaving: the two commands share ONE `ranging` message but are two
#    independent pending commands. Each waiter must read only its own half --
#    otherwise the other control's error fails a healthy command, and the other
#    control's settle "confirms" one that never landed.

def test_a_profile_change_is_not_failed_by_an_imu_env_error_landing_mid_flight(monkeypatch):
    r, _ = _run_profile(monkeypatch, [
        _ranging(_preset_applied("stability")),
        _ranging(_preset_applied("stability"), pending=True),
        _ranging(_preset_applied("stability"), pending=True,
                 imu_env=_imu_env(0, error="SET_IMU_ENV_RATE -> BUSY")),
        _ranging(_preset_applied("precision"),
                 imu_env=_imu_env(0, error="SET_IMU_ENV_RATE -> BUSY")),
    ], profile="precision")

    assert r["ok"] is True, r
    assert r["imu_env"]["error"] == "SET_IMU_ENV_RATE -> BUSY", \
        "the other half's error is still reported, just not as this command's verdict"


def test_an_imu_env_change_is_not_failed_or_confirmed_by_the_ranging_half(monkeypatch):
    r, _ = _run_rate(monkeypatch, [
        _ranging(_preset_applied("stability")),
        _ranging(_preset_applied("stability"), imu_env=_imu_env(0, pending=True)),
        # A profile command lands (and fails) while ours is still in flight:
        # neither its settle nor its error may decide this tool's verdict.
        _ranging(_preset_applied("precision"), error="SET_RANGING_PROFILE -> BAD_PARAM",
                 imu_env=_imu_env(0, pending=True)),
        _ranging(_preset_applied("precision"), error="SET_RANGING_PROFILE -> BAD_PARAM",
                 imu_env=_imu_env(45)),
    ], rate_hz=45)

    assert r["ok"] is True, r
    assert r["imu_env"]["applied_rate_hz"] == 45


def test_rig_status_reports_the_ranging_state(monkeypatch):
    import asyncio

    from roomscan.mcp_server import tools_rig

    fake = _FakeRig([], latest={
        "ranging": _ranging(_preset_applied("high_framerate"), measured_fps=45.6,
                            imu_env=_imu_env(90)),
        "state": {"type": "state"}, "metrics": {"type": "metrics"},
        "session": {"type": "session"}})
    monkeypatch.setattr(tools_rig, "rig", fake)
    monkeypatch.setattr(tools_rig, "_http_ok", lambda timeout=2.0: True)
    monkeypatch.setattr(tools_rig, "_server_procs", lambda: [])

    r = asyncio.run(tools_rig.rig_status(reconnect=False))

    assert r["ranging_profile"] == "high_framerate"
    assert r["ranging_measured_fps"] == 45.6
    assert r["imu_env_rate_hz"] == 90
    assert r["imu_env_coupled"] is False
    assert r["ranging"]["applied"]["fps"] == 46


# --- rig_command ACK correlation (#171) ---------------------------------------
#
# `rig_command()` used to accept the NEXT `cmd` broadcast of any label as proof,
# so a differently-named command's late ACK -- still in flight from an earlier,
# already-timed-out call -- could satisfy a later call's wait and be reported as
# its result. These pin the correlation, reusing `_FakeRig`'s `wait_for` script
# mechanism (each script entry is popped one per `wait_for("cmd", ...)` call).

def _cmd_ack(label, status="ok", detail="OK applied=1"):
    return {"type": "cmd", "label": label, "status": status, "detail": detail}


def test_rig_command_does_not_accept_another_commands_late_ack(monkeypatch):
    """Reproduces #171's exact reported sequence: a `standby` call gets no ACK
    at all and times out, then a `ping` call must not be satisfied by the
    standby ACK that finally lands on the bus while the ping call is waiting."""
    import asyncio

    from roomscan.mcp_server import tools_rig

    fake = _FakeRig([])   # call 1: nothing on the bus before it gives up
    monkeypatch.setattr(tools_rig, "rig", fake)

    r1 = asyncio.run(tools_rig.rig_command(name="standby", param=1, timeout=0.05))
    assert r1["ok"] is False
    assert "no ACK within 0.05s" in r1["error"]
    assert r1.get("unmatched_acks") is None

    # The device's real (late) standby ACK now lands, ready for whichever
    # wait_for("cmd") call comes next -- exactly the mechanism #171 exploited.
    fake.script.append(_cmd_ack("standby 1"))

    r2 = asyncio.run(tools_rig.rig_command(name="ping", timeout=0.05))

    assert r2["ok"] is False, r2
    assert r2.get("ack") is None
    assert r2["sent"] == {"type": "cmd", "name": "ping"}
    assert r2["unmatched_acks"] == [_cmd_ack("standby 1")], (
        "the ping call's wait was satisfied by the standby ACK -- #171's bug")


def test_rig_command_skips_a_mismatched_ack_and_returns_the_real_one(monkeypatch):
    """A non-matching ACK observed while waiting is reported, not silently
    dropped, and does not stop the wait from finding the real ACK after it."""
    import asyncio

    from roomscan.mcp_server import tools_rig

    fake = _FakeRig([_cmd_ack("standby 1"), _cmd_ack("ping")])
    monkeypatch.setattr(tools_rig, "rig", fake)

    r = asyncio.run(tools_rig.rig_command(name="ping", timeout=1.0))

    assert r["ok"] is True
    assert r["status"] == "ok"
    assert r["ack"] == _cmd_ack("ping")
    assert r["unmatched_acks"] == [_cmd_ack("standby 1")]


def test_rig_command_standbys_default_timeout_is_longer_than_pings(monkeypatch):
    """#171's second half: the 5s default was too short for SET_STANDBY (no ACK
    within 5s but a clean ACK within 20s, on-rig). `standby` gets a longer
    per-command default; other commands keep 5s."""
    import asyncio

    from roomscan.mcp_server import tools_rig

    fake = _FakeRig([])
    monkeypatch.setattr(tools_rig, "rig", fake)

    r_ping = asyncio.run(tools_rig.rig_command(name="ping"))
    r_standby = asyncio.run(tools_rig.rig_command(name="standby", param=1))

    assert tools_rig._COMMAND_TIMEOUT_S["standby"] > tools_rig._DEFAULT_COMMAND_TIMEOUT_S
    assert f"no ACK within {tools_rig._DEFAULT_COMMAND_TIMEOUT_S}s" in r_ping["error"]
    assert f"no ACK within {tools_rig._COMMAND_TIMEOUT_S['standby']}s" in r_standby["error"]


def test_rig_command_explicit_timeout_overrides_the_per_command_default(monkeypatch):
    import asyncio

    from roomscan.mcp_server import tools_rig

    fake = _FakeRig([])
    monkeypatch.setattr(tools_rig, "rig", fake)

    r = asyncio.run(tools_rig.rig_command(name="standby", param=1, timeout=3.0))

    assert "no ACK within 3.0s" in r["error"]


# --- profile_estimate (the offline half) -------------------------------------

def test_profile_estimate_matches_the_profiles_model_for_a_preset():
    from roomscan import profiles
    from roomscan.mcp_server.tools_data import profile_estimate

    r = profile_estimate(profile="high_framerate")
    expected = profiles.estimate_to_json(
        profiles.estimate_preset(profiles.ProfileId.HIGH_FRAMERATE))

    assert r["ok"] is True
    assert r["estimate"] == expected
    assert r["estimate"]["fps"] == 46


def test_profile_estimate_reports_the_delivered_rate_not_the_requested_one():
    """A 90 fps request at 2 ms exposure is really ~45 fps (measured 2026-08-03).
    An estimate that echoed the request would hide exactly that."""
    from roomscan.mcp_server.tools_data import profile_estimate

    r = profile_estimate(ranging_mode="precision", fps=90, exposure_ms=2,
                            power_mode="regular")

    assert r["ok"] is True
    assert r["estimate"]["fps"] == 90
    assert r["estimate"]["expected_delivered_fps"] < 50
    assert any("ceiling" in w for w in r["warnings"])


def test_profile_estimate_rejects_an_invalid_manual_candidate():
    from roomscan.mcp_server.tools_data import profile_estimate

    r = profile_estimate(ranging_mode="ambient", fps=90, exposure_ms=4,
                            power_mode="regular")

    assert r["ok"] is False
    assert any("DSS" in e for e in r["errors"])


def test_profile_estimate_with_no_arguments_describes_every_preset():
    from roomscan import profiles
    from roomscan.mcp_server.tools_data import profile_estimate

    r = profile_estimate()

    assert set(r["presets"]) == {profiles.PROFILE_ID_TO_STR[p] for p in profiles.PRESETS}
    assert r["presets"]["stability"]["fps"] == 30


def test_profile_estimate_names_the_cdc_ceiling_only_on_cdc():
    from roomscan.mcp_server.tools_data import profile_estimate

    args = dict(ranging_mode="precision", fps=90, exposure_ms=2, power_mode="regular")
    on_cdc = profile_estimate(transport="cdc", **args)
    on_eth = profile_estimate(transport="ethernet", **args)

    assert "CDC" in (on_cdc["estimate"]["transport_warning"] or "")
    assert on_eth["estimate"]["transport_warning"] is None


def test_profile_estimate_requires_a_whole_manual_candidate():
    from roomscan.mcp_server.tools_data import profile_estimate

    r = profile_estimate(ranging_mode="precision", fps=45)

    assert r["ok"] is False
    assert "exposure_ms" in r["errors"][0]
