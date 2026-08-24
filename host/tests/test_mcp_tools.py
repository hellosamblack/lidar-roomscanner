"""Unit tests for the pure halves the MCP tools call.

These functions were previously unreachable except by running a CLI and reading
prose, so this is the first coverage they have had.
"""
from __future__ import annotations

import re
import struct
import zlib
from pathlib import Path
from types import SimpleNamespace


from tools.analyze_capture import MAGIC, scan
from tools.headless_doctor import Doctor

REPO = Path(__file__).resolve().parents[2]


def test_slam_rerender_threads_an_opt_in_icp_trace_window(tmp_path, monkeypatch):
    import json
    from roomscan.mcp_server import tools_data

    capture = tmp_path / "capture.bin"
    capture.write_bytes(b"capture")
    seen = {}

    def fake_run(cmd, **_kwargs):
        seen["cmd"] = cmd
        report = Path(cmd[cmd.index("--json") + 1])
        report.write_text(json.dumps({
            "chosen_mode": "translation",
            "modes": {"translation": {
                "map": {"saturated": False},
                "tracking": {"died": False},
                "icp_trace": {"window_s": [100.0, 175.0], "frames": []},
            }},
        }))
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(tools_data.subprocess, "run", fake_run)
    result = tools_data.slam_rerender(
        str(capture), icp_trace_start_s=100.0, icp_trace_end_s=175.0,
    )

    assert result["ok"] is True
    i = seen["cmd"].index("--icp-trace")
    assert seen["cmd"][i + 1] == "100.0:175.0"


def test_slam_rerender_rejects_a_half_specified_icp_trace_window(tmp_path):
    from roomscan.mcp_server.tools_data import slam_rerender

    capture = tmp_path / "capture.bin"
    capture.write_bytes(b"capture")
    result = slam_rerender(str(capture), icp_trace_start_s=100.0)
    assert result["ok"] is False
    assert "both" in result["error"]


def _frame(*, stream: int = 7, seq: int = 0, payload: bytes = b"\x01\x02\x03\x04",
           ftype: int = 1, corrupt_crc: bool = False, version: int = 1) -> bytes:
    header = struct.pack("<4sBBBBIQHHII", MAGIC, version, ftype, stream, 0, seq, 1000 * seq,
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


def test_survey_matches_analyze_capture_on_a_v2_capture(tmp_path):
    """Regression for #178: analyze_capture.scan() was taught protocol v2
    (SUPPORTED_VERSIONS, commit 56ee9ba) but `_survey()` kept its own
    hardcoded `ver != 1` check, so `capture_list(surveyed=True)` silently
    reported `streams: {}` / `frames_sampled: 0` / `has_stream_9: false` for
    every capture recorded since the wire format moved to v2 (2b8a9ee,
    2026-08-03) while `capture_analyze` decoded the identical bytes fine.

    Pin `_survey()`'s per-stream census against `scan()`'s over the SAME
    v2-shaped bytes so the two tools cannot desync again on the next version
    bump.
    """
    from collections import Counter

    from roomscan.mcp_server.tools_data import _survey
    from tools.analyze_capture import scan

    p = tmp_path / "v2.bin"
    p.write_bytes(
        _frame(version=2, stream=7, seq=0)     # RAW_3DMD
        + _frame(version=2, stream=9, seq=1)   # IMU_QUAT
        + _frame(version=2, stream=7, seq=2)   # RAW_3DMD
        + _frame(version=2, stream=10, seq=3)  # ENV
        + _frame(version=2, stream=7, seq=4))  # RAW_3DMD

    reference = scan(str(p))
    expected_streams = Counter(
        rec["stream_id"] for rec in reference["frame_log"] if rec["frame_type"] == "DATA")

    survey = _survey(p)

    assert reference["frames_decoded"] == 5, "fixture itself must be v2-decodable"
    assert survey["frames_sampled"] == 5
    assert dict(survey["streams"]) == dict(expected_streams)
    assert survey["streams"]["RAW_3DMD"] == 3
    assert survey["streams"]["IMU_QUAT"] == 1
    assert survey["streams"]["ENV"] == 1


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


def test_ui_screenshot_applies_the_requested_viewport_through_the_public_tool(monkeypatch):
    """Drive the regression through `ui_screenshot`, not `CdpSession.start()`.

    The tests above cover `start()` in isolation and did NOT see the actual bug:
    `ui_screenshot` calls `browser.start(width=..., height=...)` correctly, but then
    calls `browser.screenshot()` and `browser.diag_tail()` (-> `evaluate()`), and
    both of those call `self.start()` with NO arguments just to ensure the browser
    is up. Before the fix, that argless call carried hardcoded defaults
    (`width: int = 1600, height: int = 1000`) and silently reasserted them,
    reverting the size a moment before the pixels were actually captured -- the
    exact failure named in the issue (`["820x700", "1600x1000"]` from a single
    `ui_screenshot(width=820, height=700)` call).

    Runs the three sizes docs/web-ui-testing.md prescribes, back to back on one
    warm session (as real repeat calls are), and asserts the viewport in force AT
    THE MOMENT `Page.captureScreenshot` fires -- the property the issue is
    actually about, since a later re-apply (e.g. `diag_tail()`'s `evaluate()`,
    which runs after the capture) could coincidentally match the request and mask
    a capture-time miss. Reintroducing the old hardcoded
    `width: int = 1600, height: int = 1000` defaults on `CdpSession.start` turns
    this red (verified while writing this test; see commit message for the exact
    failure text).
    """
    import asyncio
    import base64

    from roomscan.mcp_server import tools_ui
    from roomscan.mcp_server.session import CdpSession

    applied: list[tuple[int, int]] = []
    at_capture: list[tuple[int, int]] = []
    s = CdpSession()
    s._ws = object()            # already-launched browser (as the tests above simulate)
    s.url = "http://existing"   # so ui_screenshot takes the no-renavigate branch

    async def fake_cmd(method, params=None):
        if method == "Emulation.setDeviceMetricsOverride":
            applied.append((params["width"], params["height"]))
            return {}
        if method == "Page.captureScreenshot":
            # Snapshot the viewport in force AT THIS INSTANT -- not after
            # `ui_screenshot` returns, when a later argless re-apply (diag_tail)
            # could coincidentally match and hide a capture-time miss.
            at_capture.append(applied[-1])
            return {"data": base64.b64encode(b"fake-png-bytes").decode()}
        if method == "Runtime.evaluate":
            return {"result": {"value": "log tail"}}
        return {}

    s.cmd = fake_cmd             # type: ignore[method-assign]
    monkeypatch.setattr(tools_ui, "browser", s)

    for width, height in [(1280, 800), (1100, 560), (820, 700)]:
        asyncio.run(tools_ui.ui_screenshot(width=width, height=height, settle=0))
        assert at_capture[-1] == (width, height), (
            f"ui_screenshot(width={width}, height={height}): the viewport at the "
            f"moment Page.captureScreenshot fired was {at_capture[-1]}, not the "
            f"requested size -- full capture-time sequence {at_capture!r}, full "
            f"apply sequence {applied!r}")
        # Belt and braces: the post-return size (what a caller would read back)
        # should also match, though the line above is the one that must hold.
        assert applied[-1] == (width, height), (
            f"ui_screenshot(width={width}, height={height}) did not survive to "
            f"return either: last viewport applied was {applied[-1]}, full "
            f"sequence {applied!r}")


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

    async def fake_connect(url, max_size=None, **kwargs):
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


def test_rig_session_connect_disables_keepalive_ping(monkeypatch):
    """`RigSession.connect()` must pass `ping_interval=None` to `websockets.connect`
    on both `/ws` and `/ws-mesh` (issue #220) to prevent keepalive ping timeouts
    under load/GIL stalls.
    """
    import asyncio

    import websockets

    from roomscan.mcp_server.session import RigSession

    class _FakeWs:
        def __aiter__(self):
            return self._gen()

        async def _gen(self):
            await asyncio.Future()
            yield

        async def close(self):
            pass

    calls: list[dict] = []

    async def fake_connect(url, **kwargs):
        calls.append({"url": url, "kwargs": kwargs})
        return _FakeWs()

    monkeypatch.setattr(websockets, "connect", fake_connect)

    rig = RigSession(url="ws://localhost:8000/ws")

    async def run():
        ok = await rig.connect(timeout=1.0)
        assert ok is True
        assert len(calls) == 2
        assert calls[0]["url"] == "ws://localhost:8000/ws"
        assert calls[0]["kwargs"].get("ping_interval") is None
        assert calls[0]["kwargs"].get("max_size") is None
        assert calls[1]["url"] == "ws://localhost:8000/ws-mesh"
        assert calls[1]["kwargs"].get("ping_interval") is None
        assert calls[1]["kwargs"].get("max_size") is None
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

    async def request(self, message, expect, timeout=5.0):
        await self.send(message)
        return await self.wait_for(expect, timeout=timeout)


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


def _run_view(monkeypatch, script, latest=None, **kwargs):
    import asyncio

    from roomscan.mcp_server import tools_rig

    fake = _FakeRig(script, latest=latest)
    monkeypatch.setattr(tools_rig, "rig", fake)
    kwargs.setdefault("timeout", 2.0)
    return asyncio.run(tools_rig.rig_view(**kwargs)), fake


def _view_state(**extra):
    # roomscan-web always follows `detailed`/`regenerate_detailed`'s reply with
    # a `_broadcast_state`, whether the rebuild started or was refused.
    return {"type": "state", "source": "view", "display": "detailed", **extra}


def test_rig_view_regenerate_reports_a_started_rebuild(monkeypatch):
    r, fake = _run_view(monkeypatch, [
        {"type": "detailed", "started": True, "capture": "room.rec", "phase": "frames",
         "processed": 0, "total": 900, "fraction": 0.0, "done": False},
        _view_state(),
    ], regenerate=True)
    assert r["detailed"]["started"] is True
    assert r["detailed"]["processed"] == 0
    assert r["detailed"]["total"] == 900
    assert r["ok"] is True
    assert {"type": "regenerate_detailed"} in fake.sent


def test_rig_view_regenerate_reports_a_refused_rebuild_with_its_reason(monkeypatch):
    r, fake = _run_view(monkeypatch, [
        {"type": "detailed", "started": False, "reason": "another Detailed build is running"},
        _view_state(),
    ], regenerate=True)
    assert r["detailed"]["started"] is False
    assert r["detailed"]["reason"] == "another Detailed build is running"
    assert r["ok"] is False
    assert "another Detailed build is running" in r["error"]


def test_rig_view_regenerate_two_calls_in_a_row_are_not_identical(monkeypatch):
    from roomscan.mcp_server import tools_rig
    import asyncio

    fake = _FakeRig([
        {"type": "detailed", "started": True, "capture": "room.rec", "phase": "frames",
         "processed": 0, "total": 900, "fraction": 0.0, "done": False},
        _view_state(),
    ])
    monkeypatch.setattr(tools_rig, "rig", fake)
    first = asyncio.run(tools_rig.rig_view(regenerate=True, timeout=2.0))
    fake.script.append({"type": "detailed", "started": False,
                        "reason": "another Detailed build is running"})
    fake.script.append(_view_state())
    second = asyncio.run(tools_rig.rig_view(regenerate=True, timeout=2.0))
    assert first["detailed"] != second["detailed"]
    assert first["ok"] is True
    assert second["ok"] is False


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


def test_unit_helpers_parse_systemctl_output(monkeypatch):
    """`_unit_loaded`/`_unit_active` gate the whole managed lifecycle path (#207).
    They must read `systemctl --user` output exactly, and treat any failure to
    reach the user bus as "unmanaged" so the bare-process path runs unchanged."""
    from roomscan.mcp_server import tools_rig as T

    calls = []

    def fake_run(argv, **kw):
        calls.append(argv)
        assert argv[:2] == ["systemctl", "--user"], argv
        if "show" in argv:                       # LoadState query
            return SimpleNamespace(returncode=0, stdout="loaded\n", stderr="")
        if "is-active" in argv:
            return SimpleNamespace(returncode=0, stdout="active\n", stderr="")
        raise AssertionError(argv)

    monkeypatch.setattr(T.subprocess, "run", fake_run)
    assert T._unit_loaded() is True
    assert T._unit_active() is True

    # not-loaded / inactive
    monkeypatch.setattr(T.subprocess, "run", lambda a, **k: SimpleNamespace(
        returncode=1, stdout="not-found\n" if "show" in a else "inactive\n", stderr=""))
    assert T._unit_loaded() is False
    assert T._unit_active() is False

    # unreachable user bus (no systemd, headless cron) -> unmanaged, never raises
    def boom(a, **k):
        raise OSError("no bus")
    monkeypatch.setattr(T.subprocess, "run", boom)
    assert T._unit_loaded() is False
    assert T._unit_active() is False


# --- browser idle reaping (#219) ---------------------------------------------
#
# The `ui_*` browser is kept warm across tool calls on purpose, but nothing ever
# closed it, and the page it sits on is a Three.js scene software-rasterized by
# SwiftShader. Measured 2026-08-24: two abandoned browsers from two other agent
# sessions were burning 1115% and 750% CPU -- 18.6 of 26 cores -- one with 96 h of
# accumulated CPU time, next to a `roomscan-web` using 27.7%. `_IdleReapMixin` parks
# the page to about:blank on an idle timer and then stops the browser.
#
# The failure mode these must rule out is NOT "it failed to reap" -- that one is
# loud. It is "it reaped and then quietly served the blank page", which returns
# `null` for every selector and reads exactly like a successful evaluate.

def _fake_browser(**attrs):
    """A minimal `_IdleReapMixin` backend whose page content is observable.

    `_evaluate_impl` answers from whatever page is actually loaded, so a test can
    tell a restored page apart from a parked one by the VALUE it gets back, rather
    than by re-reading the flag the code under test just set.
    """
    from roomscan.mcp_server.session import BLANK_URL, _IdleReapMixin

    class _Fake(_IdleReapMixin):
        restore_timeout_s = 0.0           # skip the readiness gate in tests
        reap_tick_s = 0.01

        def __init__(self):
            self.url = None
            self.page = BLANK_URL         # what the browser is REALLY showing
            self.alive = True
            self.navigations: list[str] = []
            self.stopped = 0
            self._idle_init()

        def _alive(self):
            return self.alive

        async def _navigate_raw(self, url):
            self.navigations.append(url)
            self.page = url

        async def _start_impl(self, width=None, height=None):
            self.alive = True

        async def _stop_impl(self):
            self.stopped += 1
            self.alive = False
            self.page = None
            self.url = None

        async def _goto_impl(self, url, settle=8.0):
            await self.start()
            await self._navigate_raw(url)
            self.url = url

        async def _evaluate_impl(self, expression, await_promise=True):
            await self.start()
            # A blank page has no app on it -- the silent-wrong-answer case.
            return {"ok": True, "value": None if self.page == BLANK_URL else self.page}

        async def _wait_for_impl(self, predicate_js, timeout_s=15.0, poll_s=0.25):
            await self.start()
            return {"ok": True, "waited_s": 0.0}

        async def _screenshot_impl(self):
            await self.start()
            return b"png:" + (self.page or "none").encode()

    s = _Fake()
    for k, v in attrs.items():
        setattr(s, k, v)
    return s


def test_idle_browser_parks_to_about_blank_but_keeps_its_url():
    """Parking must drop the WebGL page without forgetting where it was.

    `self.url` stays pointing at the real page: it is still the page this session
    is logically on, and `ui_screenshot`'s `browser.url is None` cold-path check
    keys off it. Only `_parked_url` records that a restore is owed.
    """
    import asyncio

    from roomscan.mcp_server.session import BLANK_URL

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 5.0               # pretend 5 s of idle
        await s._reap_once()

    asyncio.run(scenario())

    assert s.page == BLANK_URL, "the parked browser must not still be on the live page"
    assert s.parked is True
    assert s.url == "http://localhost:8000/", "park must not forget the page it left"
    assert s.park_count == 1


def test_a_parked_browser_restores_before_evaluating_instead_of_reading_a_blank_page():
    """The acceptance test with teeth: a restored evaluate sees the REAL page.

    Asserting on the returned value, not on `parked` going False -- a read-back of
    the flag the restore just cleared would pass even if the navigation never
    happened, which is the `sceneOpacity` getter mistake (#106).
    """
    import asyncio

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 5.0
        await s._reap_once()
        assert s.parked is True
        return await s.evaluate("document.title")

    r = asyncio.run(scenario())

    assert r["value"] == "http://localhost:8000/", (
        f"evaluate on a parked browser must restore the page first, got {r!r}")
    assert s.restore_count == 1
    assert s.last_restore["url"] == "http://localhost:8000/"


def test_any_interaction_resets_the_idle_clock():
    """An in-use browser is never reaped -- that is what "TTL" has to mean here."""
    import asyncio

    s = _fake_browser(park_after_s=1.0, stop_after_s=2.0)

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 5.0               # old enough to be reaped...
        await s.evaluate("1")             # ...but this call resets the clock
        await s._reap_once()

    asyncio.run(scenario())

    assert s.parked is False, "a browser used a moment ago must not be parked"
    assert s.stopped == 0
    assert s.park_count == 0


def test_a_long_wait_for_is_not_parked_out_from_under_itself():
    """`ui_wait_for` can legitimately block past the park TTL.

    The timestamp alone is not enough: a wait that outlives `park_after_s` would be
    parked mid-flight and fail for a reason its caller cannot see. The busy counter
    is what makes that impossible.
    """
    import asyncio

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)

    async def scenario():
        await s.goto("http://localhost:8000/")

        async def slow_wait(predicate_js, timeout_s=15.0, poll_s=0.25):
            s._last_used -= 5.0           # this wait has outlived the park TTL
            await s._reap_once()          # reaper ticks mid-wait
            return {"ok": True, "waited_s": 5.0}

        s._wait_for_impl = slow_wait
        return await s.wait_for("window.ready")

    r = asyncio.run(scenario())

    assert r["ok"] is True
    assert s.parked is False, "the reaper must not park a browser with an operation in flight"


def test_idle_browser_stops_after_the_longer_ttl():
    import asyncio

    s = _fake_browser(park_after_s=1.0, stop_after_s=2.0)

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 5.0
        await s._reap_once()

    asyncio.run(scenario())

    assert s.stopped == 1
    assert s.alive is False
    assert s.stop_count == 1
    assert s.parked is False, "a stopped browser owes no restore"


def test_goto_cancels_a_pending_restore_rather_than_paying_for_it():
    """A caller about to navigate must not first be charged an 8 s restore settle."""
    import asyncio

    from roomscan.mcp_server.session import BLANK_URL

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 5.0
        await s._reap_once()
        s.navigations.clear()
        await s.goto("http://localhost:8000/?x=1")

    asyncio.run(scenario())

    assert s.navigations == ["http://localhost:8000/?x=1"], (
        f"goto must navigate once, not restore-then-navigate; got {s.navigations!r}")
    assert s.restore_count == 0
    assert s.page != BLANK_URL


def test_a_failed_restore_does_not_leave_the_session_looping_on_it():
    """Clearing `_parked_url` before navigating: a page we can never reach must not
    make every subsequent call retry it forever."""
    import asyncio

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 5.0
        await s._reap_once()

        async def boom(url):
            raise RuntimeError("navigation failed")

        s._navigate_raw = boom
        try:
            await s.evaluate("1")
        except RuntimeError:
            pass
        return s.parked

    assert asyncio.run(scenario()) is False


def test_a_failed_park_leaves_the_browser_usable():
    """Parking is an optimisation; the stop stage is the real backstop."""
    import asyncio

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)

    async def scenario():
        await s.goto("http://localhost:8000/")

        async def boom(url):
            raise RuntimeError("navigation failed")

        s._navigate_raw = boom
        s._last_used -= 5.0
        await s._reap_once()

    asyncio.run(scenario())

    assert s.parked is False, "a park that never happened must not owe a restore"
    assert s.park_count == 0
    assert s.url == "http://localhost:8000/"


def test_zero_ttl_disables_reaping():
    """The env vars must be able to turn this off outright on a machine with a GPU."""
    import asyncio

    s = _fake_browser(park_after_s=0.0, stop_after_s=0.0)

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 100000.0
        await s._reap_once()

    asyncio.run(scenario())

    assert s.parked is False
    assert s.stopped == 0


def test_env_seconds_rejects_junk_and_clamps_negatives():
    from roomscan.mcp_server.session import _env_seconds

    import os as _os
    for raw, expected in (("30", 30.0), ("", 9.0), ("  ", 9.0), ("junk", 9.0),
                          ("-5", 0.0), ("0", 0.0)):
        _os.environ["ROOMSCAN_TEST_TTL"] = raw
        assert _env_seconds("ROOMSCAN_TEST_TTL", 9.0) == expected, raw
    del _os.environ["ROOMSCAN_TEST_TTL"]
    assert _env_seconds("ROOMSCAN_TEST_TTL", 9.0) == 9.0


def test_both_browser_backends_share_one_reaping_implementation():
    """One implementation, not two -- a backend that defined its own `goto` would
    silently opt out of the idle bookkeeping."""
    from roomscan.mcp_server.session import (CdpSession, PlaywrightSession,
                                             _IdleReapMixin)

    for cls in (CdpSession, PlaywrightSession):
        assert issubclass(cls, _IdleReapMixin)
        for name in ("goto", "evaluate", "wait_for", "screenshot", "start", "stop"):
            assert name not in vars(cls), (
                f"{cls.__name__}.{name} overrides the mixin and escapes idle "
                f"bookkeeping; implement it as _{name}_impl instead")
            assert f"_{name}_impl" in vars(cls) or name in ("start", "stop"), name


def test_ui_eval_reports_a_restore_instead_of_hiding_it(monkeypatch):
    """A silent 8 s restore looks like a slow page; the caller has to be told."""
    import asyncio

    from roomscan.mcp_server import tools_ui

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)
    monkeypatch.setattr(tools_ui, "browser", s)

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 5.0
        await s._reap_once()
        return await tools_ui.ui_eval("document.title")

    r = asyncio.run(scenario())

    assert r["value"] == "http://localhost:8000/"
    assert r["restored"]["url"] == "http://localhost:8000/"


def test_ui_browser_status_reports_the_reap_state():
    import asyncio

    # stop TTL well clear of the idle we simulate, so this exercises the PARK stage
    # rather than tripping straight through to stop.
    s = _fake_browser(park_after_s=1.0, stop_after_s=100.0)

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 5.0
        await s._reap_once()
        return s.status()

    st = asyncio.run(scenario())

    assert st["parked"] is True
    assert st["url"] == "http://localhost:8000/"
    assert st["park_after_s"] == 1.0 and st["stop_after_s"] == 100.0
    assert st["parks"] == 1
    assert st["idle_s"] >= 5.0


def test_the_reaper_completes_a_teardown_it_triggers_itself():
    """Regression: `stop()` used to cancel the task that called it.

    `_reap_once` -> `stop()` -> `self._reaper.cancel()` raises CancelledError at the
    next await INSIDE `_stop_impl`, halfway through teardown. The fake backends above
    never caught it because their `_stop_impl` has no await point and the other tests
    call `_reap_once()` directly instead of going through the reaper task. The live
    probe did: `stops` incremented and most of the chrome tree exited, but `_page`
    was never cleared, so `_alive()` stayed True with no working browser behind it.

    So this drives the real `_reap_loop` task and puts an await in the middle of
    teardown, which is the only shape that reproduces it.
    """
    import asyncio

    s = _fake_browser(park_after_s=0.0, stop_after_s=0.05)
    s.teardown_finished = False

    async def slow_stop():
        await asyncio.sleep(0.02)         # the await the cancellation landed on
        s.stopped += 1
        s.alive = False
        s.page = None
        s.url = None
        s.teardown_finished = True

    s._stop_impl = slow_stop

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._ensure_reaper()
        for _ in range(200):
            await asyncio.sleep(0.01)
            if not s.alive:
                break

    asyncio.run(scenario())

    assert s.teardown_finished is True, (
        "the reaper cancelled its own teardown midway; stop() must not cancel the "
        "task that called it")
    assert s.alive is False
    assert s.stop_count == 1


def test_cdp_socket_disables_the_websockets_keepalive(monkeypatch):
    """`websockets` kills its own CDP connection on a busy page.

    The client defaults to a 20 s keepalive ping with a 20 s pong deadline, and Chrome
    cannot answer it while SwiftShader saturates a dozen cores rendering the live
    scene -- so the CLIENT closes a healthy connection with `1011 keepalive ping
    timeout`. Pre-existing; #219's reaper only made it visible, because the park
    navigation is the first CDP command this backend ever issues after ~20 s of idle
    on a loaded page. Every park attempt failed while the identical call succeeded by
    hand on a quiet page.

    Keepalive buys nothing here: Chrome is our own subprocess and `_proc.poll()` is
    the liveness signal that matters.
    """
    import asyncio

    import websockets

    from roomscan.mcp_server.session import CdpSession

    seen: dict = {}

    async def fake_connect(url, **kwargs):
        seen.update(kwargs)
        return object()

    monkeypatch.setattr(websockets, "connect", fake_connect)

    s = CdpSession()
    s._proc = SimpleNamespace(poll=lambda: None)   # already "launched"
    monkeypatch.setattr(s, "_cdp_target", lambda *a, **k: "ws://localhost:9222/x")

    async def fake_cmd(method, params=None):
        return {}

    s.cmd = fake_cmd                      # type: ignore[method-assign]
    asyncio.run(s.start())

    assert "ping_interval" in seen and seen["ping_interval"] is None, (
        "the CDP socket must disable the websockets keepalive, else a busy page "
        f"gets its own connection torn down; connect kwargs were {seen!r}")


def test_a_failed_park_is_recorded_not_swallowed():
    """A silent `except: return` hid a TOTAL park failure on the CDP backend.

    The browser merely looked like it had gone quiet and stopped on schedule, so
    nothing about the observable behaviour said "parking never once worked". The
    reason has to survive into `status()`.
    """
    import asyncio

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)

    async def scenario():
        await s.goto("http://localhost:8000/")

        async def boom(url):
            raise RuntimeError("keepalive ping timeout")

        s._navigate_raw = boom
        s._last_used -= 5.0
        await s._reap_once()

    asyncio.run(scenario())

    assert s.parked is False
    assert "keepalive ping timeout" in (s.last_park_error or ""), (
        f"a park failure must be recorded, got {s.last_park_error!r}")
    assert "keepalive" in (s.status()["last_park_error"] or "")


def test_a_successful_park_clears_a_stale_park_error():
    import asyncio

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)
    s.last_park_error = "RuntimeError: something old"

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._last_used -= 5.0
        await s._reap_once()

    asyncio.run(scenario())

    assert s.parked is True
    assert s.last_park_error is None, "a park that worked must not leave a stale error"


def test_the_reap_ttls_are_long_enough_not_to_interrupt_a_working_session():
    """Owner call (2026-08-24): the reaper must never be why a `ui_eval` is slow.

    Pinned deliberately. The failure being prevented is a browser burning ~11 cores
    for DAYS, so 30 min is already a ~99.9% reduction; shorter starts costing real
    sessions, which can easily go half an hour between UI checks. Anyone shortening
    these should have to change this test and think about that trade.
    """
    from roomscan.mcp_server import session as S

    assert S.BROWSER_PARK_S == 1800.0
    assert S.BROWSER_STOP_S == 14400.0
    assert S.BROWSER_STOP_S > S.BROWSER_PARK_S, "stop must not pre-empt park"


def test_restore_waits_on_a_readiness_predicate_not_a_clock():
    """A fixed sleep is the wrong shape: too long on a fast restore, and no guarantee
    at all on a slow one. Measured readiness is ~1.8 s against the old blind 8 s."""
    import asyncio

    from roomscan.mcp_server.session import RESTORE_READY_JS

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)
    s.restore_timeout_s = 15.0
    waited: list[tuple[str, float]] = []

    async def record_wait(predicate_js, timeout_s=15.0, poll_s=0.25):
        waited.append((predicate_js, timeout_s))
        return {"ok": True, "waited_s": 1.8}

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._wait_for_impl = record_wait
        s._last_used -= 5.0
        await s._reap_once()
        return await s.evaluate("document.title")

    r = asyncio.run(scenario())

    assert r["value"] == "http://localhost:8000/"
    assert waited and waited[0][0] == RESTORE_READY_JS, (
        f"restore must gate on the readiness predicate; waited on {waited!r}")
    assert waited[0][1] == 15.0
    assert s.last_restore["ready"] is True


def test_a_restore_that_times_out_proceeds_and_says_so():
    """The gate is a give-up point, not a hang. The caller still gets its call --
    but `ready: False` records that the page may still have been settling."""
    import asyncio

    s = _fake_browser(park_after_s=1.0, stop_after_s=0.0)
    s.restore_timeout_s = 0.05

    async def never_ready(predicate_js, timeout_s=15.0, poll_s=0.25):
        return {"ok": False, "timeout_s": timeout_s}

    async def scenario():
        await s.goto("http://localhost:8000/")
        s._wait_for_impl = never_ready
        s._last_used -= 5.0
        await s._reap_once()
        return await s.evaluate("document.title")

    r = asyncio.run(scenario())

    assert r["value"] == "http://localhost:8000/", "a timed-out gate must not fail the call"
    assert s.last_restore["ready"] is False
    assert s.restore_count == 1


def test_the_readiness_predicate_degrades_on_a_page_without_the_app():
    """Each app-specific clause is skipped when its element is absent.

    If the predicate threw or read false on a non-roomscan page, every restore of
    such a page would burn the entire timeout. Checked structurally here and
    executed for real in `browser_reap_probe.py`.
    """
    from roomscan.mcp_server.session import RESTORE_READY_JS

    assert "!c ||" in RESTORE_READY_JS, "canvas clause must pass when there is no canvas"
    assert "!e ||" in RESTORE_READY_JS, "fps clause must pass when the HUD is absent"
    assert "readyState === 'complete'" in RESTORE_READY_JS
    # Guard the measured trap: gating on pixels never goes true on this context.
    assert "readPixels" not in RESTORE_READY_JS, (
        "readPixels reads uniform with preserveDrawingBuffer off -- gating on it "
        "would burn the full timeout on every restore")


# --- orphaned browser sweep (#221) -------------------------------------------
#
# The #219 reaper lives INSIDE the MCP server process, so a killed or crashed server
# leaves its browser immortal with no reaper at all -- POSIX reparents children, it
# does not kill them. Verified live: a Chrome whose parent was SIGKILLed kept running
# and kept burning. Restarting a server to pick up new code is one way to cause it,
# which made the #219 fix itself a way to leak a browser.
#
# The whole safety argument is OWNERSHIP: several agent sessions share this box, so
# the sweep must never touch a browser a live server still owns, and must never match
# the user's ordinary Chrome. These pin that; the live end-to-end (real SIGKILL, real
# reparenting) is `tools/browser_reap_probe.py --orphans`.

class _FakeProc:
    def __init__(self, pid, cmd, parent=None, running=True):
        self.pid = pid
        self._cmd = cmd
        self._parent = parent
        self._running = running

    def cmdline(self):
        return self._cmd.split()

    def parent(self):
        return self._parent

    def is_running(self):
        return self._running


def test_a_browser_owned_by_a_live_server_is_never_swept():
    """The load-bearing case: other agent sessions are working on this box."""
    from roomscan.mcp_server.browser_reap import _owner_is_alive

    server = _FakeProc(100, "python -m roomscan.mcp_server")
    driver = _FakeProc(101, "node playwright/driver/cli.js", parent=server)
    chrome = _FakeProc(102, "chrome --user-data-dir=/tmp/roomscan-mcp-chrome-x", parent=driver)

    assert _owner_is_alive(chrome, self_pid=999) is True


def test_a_browser_with_no_live_server_ancestor_is_an_orphan():
    from roomscan.mcp_server.browser_reap import _owner_is_alive

    init = _FakeProc(1, "/sbin/init")
    chrome = _FakeProc(102, "chrome --user-data-dir=/tmp/roomscan-mcp-chrome-x", parent=init)

    assert _owner_is_alive(chrome, self_pid=999) is False


def test_the_current_process_counts_as_an_owner():
    """A server sweeping at startup must not kill the browser it is about to use."""
    from roomscan.mcp_server.browser_reap import _owner_is_alive

    me = _FakeProc(555, "python -c something-that-does-not-say-the-mark")
    chrome = _FakeProc(556, "chrome --user-data-dir=/tmp/roomscan-mcp-chrome-x", parent=me)

    assert _owner_is_alive(chrome, self_pid=555) is True


def test_the_ownership_walk_terminates_on_a_parent_cycle():
    """A pid-reuse race can present as a cycle; the walk must not spin forever."""
    from roomscan.mcp_server.browser_reap import _owner_is_alive

    a = _FakeProc(10, "a")
    b = _FakeProc(11, "b", parent=a)
    a._parent = b                          # cycle

    assert _owner_is_alive(b, self_pid=999) is False


def test_the_sweep_only_matches_browsers_this_project_created():
    """It must never match the user's own Chrome, or another project's playwright."""
    from roomscan.mcp_server import browser_reap as B

    class _FakePsutil:
        @staticmethod
        def process_iter(_attrs=None):
            return [
                _FakeProc(1, "/opt/google/chrome/chrome --user-data-dir=/home/sam/.config/google-chrome"),
                _FakeProc(2, "/opt/google/chrome/chrome --profile-directory=Default"),
                _FakeProc(3, "node /other/project/node_modules/playwright/driver/cli.js"),
                _FakeProc(4, "chrome --user-data-dir=/tmp/roomscan-mcp-chrome-abc"),
            ]

    matched = {p.pid for p in B._candidates(_FakePsutil)}
    assert 4 in matched, "our own profile prefix must match"
    assert 1 not in matched and 2 not in matched, (
        "the user's ordinary Chrome must never be a sweep candidate")
    # A foreign playwright driver DOES match the path marker -- that is deliberate and
    # safe, because ownership decides the kill and a foreign driver has no
    # roomscan.mcp_server ancestor... but it means the ownership check is not optional.
    assert 3 in matched


def test_sweep_reports_what_it_did_rather_than_only_succeeding():
    """A sweep that killed nothing because its matcher broke must be distinguishable
    from a clean box."""
    from roomscan.mcp_server.browser_reap import sweep_orphaned_browsers

    r = sweep_orphaned_browsers()
    assert r["ok"] is True
    for key in ("killed", "kept", "killed_count", "kept_count"):
        assert key in r, f"sweep report is missing {key}"


def test_pdeathsig_is_installed_on_the_chrome_we_spawn():
    """The only cleanup that survives the parent being SIGKILLed, since it needs no
    cooperation from the dying process."""
    import inspect

    from roomscan.mcp_server import session

    src = inspect.getsource(session.CdpSession._start_impl)
    assert "preexec_fn=browser_reap.pdeathsig" in src, (
        "CdpSession must spawn Chrome with PR_SET_PDEATHSIG so a SIGKILLed server "
        "cannot leak it")
