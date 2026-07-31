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
