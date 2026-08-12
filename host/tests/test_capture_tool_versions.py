"""Pin capture.py's decode path to the live protocol version set.

Regression for #167: `decode_file()` hardcoded `ver != 1` while the wire protocol
had moved to VERSION 2, so every frame of a byte-perfect capture was rejected one
byte at a time. `report()` then printed "no frames decoded at all -- capture is
entirely garbage" and returned 1, which is capture.py's process exit code. The
capture file was fine; only the report lied -- which is worse, because that report
is what a session reads to decide whether firmware is healthy.

analyze_capture.py had already been fixed for the identical bug on 2026-08-04;
capture.py was missed. These tests exist so the next version bump breaks a test
instead of silently disabling the tool again.
"""

import zlib

from roomscan import protocol
from tools import capture


def _frame(version: int, seq: int = 1, payload: bytes = b"\x00\x01\x02\x03") -> bytes:
    """One wire frame at an arbitrary version, built the way the device builds it."""
    header = capture._HEADER.pack(
        capture.MAGIC,
        version,
        1,  # frame_type DATA
        int(protocol.StreamId.RAW_3DMD),
        0,  # flags
        seq,
        1_000 * seq,  # t_us
        8,  # width
        4,  # height
        len(payload),
        0,  # reserved
    )
    body = header + payload
    return body + zlib.crc32(body).to_bytes(4, "little")


def test_supported_versions_match_the_protocol_module():
    """The whole bug was this list drifting from the protocol's."""
    assert capture.SUPPORTED_VERSIONS == protocol.SUPPORTED_VERSIONS


def test_current_protocol_version_is_accepted():
    """Guards the specific drift that happened: VERSION advanced past the list."""
    assert protocol.VERSION in capture.SUPPORTED_VERSIONS


def test_decode_file_decodes_current_version_frames(tmp_path):
    path = tmp_path / "v_current.bin"
    path.write_bytes(b"".join(_frame(protocol.VERSION, seq=i) for i in range(1, 6)))

    frames, crc_failures, bytes_skipped, first_good, *_ = capture.decode_file(path)

    assert len(frames) == 5, "current-version frames must decode, not be skipped"
    assert crc_failures == 0
    assert bytes_skipped == 0
    assert first_good == 0
    assert [f[3] for f in frames] == [1, 2, 3, 4, 5]


def test_decode_file_still_decodes_legacy_v1_frames(tmp_path):
    """v1 stays supported -- older captures on disk must remain readable."""
    path = tmp_path / "v1.bin"
    path.write_bytes(b"".join(_frame(1, seq=i) for i in range(1, 4)))

    frames, crc_failures, _, first_good, *_ = capture.decode_file(path)

    assert len(frames) == 3
    assert crc_failures == 0
    assert first_good == 0


def test_report_exit_code_is_zero_for_a_clean_capture(tmp_path, capsys):
    """report()'s return value IS the process exit code (capture.py:408/432)."""
    path = tmp_path / "clean.bin"
    path.write_bytes(b"".join(_frame(protocol.VERSION, seq=i) for i in range(1, 4)))

    rc = capture.report(path, wall_elapsed=1.0, requested_seconds=1.0)

    assert rc == 0
    assert "entirely garbage" not in capsys.readouterr().out


def test_unknown_future_version_is_still_rejected(tmp_path):
    """The check must stay a real check -- not widened into accepting anything."""
    path = tmp_path / "v99.bin"
    path.write_bytes(_frame(99))

    frames, _, bytes_skipped, first_good, *_ = capture.decode_file(path)

    assert frames == []
    assert first_good is None
    assert bytes_skipped > 0
