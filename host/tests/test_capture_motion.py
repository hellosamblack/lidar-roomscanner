"""Tests for tools/capture_motion.py -- the operator-motion census.

Several ROADMAP data-collection gates are conditions on what the operator did
(DC-F's "10 s stationary at both ends of every take", DC-E's 15 s tilt holds,
DC-D's "slowly panning the whole time", DC-A's fast whips), so the structure this
tool reports is load-bearing for accepting or rejecting a capture.
"""
from __future__ import annotations

import math
import struct
import zlib

import numpy as np

from tools.capture_motion import (angular_rate, describe, read_orientations,
                                  segment, boresight_tilt_deg)

MAGIC = b"RSCN"


def _quat_frame(seq: int, t_us: int, quat) -> bytes:
    payload = struct.pack("<4f", *quat)
    header = struct.pack("<4sBBBBIQHHII", MAGIC, 1, 1, 9, 0, seq, t_us,
                         1, 1, len(payload), 0)
    body = header + payload
    return body + struct.pack("<I", zlib.crc32(body))


def _yaw_quat(deg: float):
    """Rotation of `deg` about world Z -- a pure pan, leaving tilt untouched."""
    h = math.radians(deg) / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _write(tmp_path, samples, name="m.bin"):
    p = tmp_path / name
    p.write_bytes(b"".join(_quat_frame(i, t, q) for i, (t, q) in enumerate(samples)))
    return str(p)


def test_reads_orientations_with_the_tim2_clock(tmp_path):
    path = _write(tmp_path, [(1_000_000, _yaw_quat(0)), (1_033_000, _yaw_quat(1))])

    t_s, quats = read_orientations(path)

    assert t_s.size == 2
    assert t_s[0] == 0.0, "time is relative to the first orientation frame"
    assert abs(t_s[1] - 0.033) < 1e-6
    assert quats.shape == (2, 4)


def test_rate_uses_measured_dt_not_a_nominal_frame_period(tmp_path):
    # 10 degrees over two frame periods is 150 deg/s, not the 300 deg/s a fixed
    # 1/30 s divisor would report. Frames ARE lost on this link, so this is the
    # difference between a real number and a phantom doubling exactly where the
    # data is worst.
    t_s = np.array([0.0, 2 / 30.0])
    quats = np.array([_yaw_quat(0), _yaw_quat(10)])

    _mid, rate, valid = angular_rate(t_s, quats)

    assert valid[0]
    assert abs(rate[0] - 150.0) < 1.0


def test_a_long_dropout_is_unmeasured_rather_than_interpolated(tmp_path):
    # During a 7 s hole the operator's motion is simply unknown; smoothing across
    # it would fabricate the very thing the DC gates check.
    t_s = np.array([0.0, 7.0, 7.033])
    quats = np.array([_yaw_quat(0), _yaw_quat(90), _yaw_quat(91)])

    _mid, _rate, valid = angular_rate(t_s, quats, max_dt_s=0.25)

    assert not valid[0], "the 7 s gap must not be reported as slow motion"
    assert valid[1]


def test_a_decelerating_pan_is_one_take_not_several(tmp_path):
    # A real pan slows through the middle of its arc and dips under the hold
    # threshold. DC-F's takes were reported as 4 fragments before this merge.
    rate = np.array([50.0, 50.0, 2.0, 50.0, 50.0])
    t_mid = np.arange(5) * 0.2
    valid = np.ones(5, dtype=bool)
    tilt = np.zeros(5)

    segs = segment(t_mid, rate, valid, tilt, hold_deg_s=8.0, min_hold_s=1.5)

    moves = [s for s in segs if s["kind"] == "move"]
    assert len(moves) == 1, f"expected one merged take, got {[s['kind'] for s in segs]}"


def test_a_genuine_hold_survives_the_merge(tmp_path):
    rate = np.concatenate([np.full(5, 50.0), np.full(40, 0.5), np.full(5, 50.0)])
    t_mid = np.arange(rate.size) * 0.1
    segs = segment(t_mid, rate, np.ones(rate.size, dtype=bool), np.zeros(rate.size),
                   hold_deg_s=8.0, min_hold_s=1.5)

    kinds = [s["kind"] for s in segs]
    assert kinds == ["move", "hold", "move"]
    assert segs[1]["duration_s"] >= 1.5


def test_bookends_and_fast_events_are_reported(tmp_path):
    # hold 2 s, whip, hold 2 s -- DC-A's gate is the whip count, DC-F's the bookends.
    samples = []
    t = 0
    ang = 0.0
    for _ in range(60):                       # 2 s still
        samples.append((t, _yaw_quat(ang)))
        t += 33_000
    for _ in range(30):                       # ~1 s pan at ~150 deg/s
        ang += 5.0
        samples.append((t, _yaw_quat(ang)))
        t += 33_000
    for _ in range(60):                       # 2 s still
        samples.append((t, _yaw_quat(ang)))
        t += 33_000
    path = _write(tmp_path, samples)

    r = describe(path, min_hold_s=1.0, fast_deg_s=100.0)

    assert r["starts_with_hold"] and r["ends_with_hold"]
    assert r["holds"] == 2
    assert r["takes"] == 1
    assert r["fast_events"] == 1
    assert r["rate_deg_s"]["max"] > 100.0


def test_a_capture_without_stream_9_reports_an_error(tmp_path):
    p = tmp_path / "empty.bin"
    p.write_bytes(b"")

    r = describe(str(p))

    assert "error" in r and "stream 9" in r["error"]


def test_tilt_is_measured_in_the_sflp_world_frame():
    # A pan with the boresight held HORIZONTAL must leave tilt unchanged. The
    # boresight has to be off the pan axis for this to discriminate: pointing it
    # along world Z leaves it fixed in every frame and the test proves nothing.
    # Deriving tilt in the renderer's Y-up frame instead sweeps it a full 180
    # degrees here -- the defect that made DC-F, a pan set, read as a tilt sweep.
    from roomscan.sensors import quat_mul

    level = (math.cos(math.radians(-45)), 0.0, math.sin(math.radians(-45)), 0.0)
    quats = np.array([quat_mul(_yaw_quat(d), level) for d in (0, 45, 90, 180, 270)])

    tilt = boresight_tilt_deg(quats)

    assert abs(tilt.mean() - 90.0) < 1e-6, f"boresight should be horizontal, got {tilt}"
    assert tilt.max() - tilt.min() < 1e-6, f"pan must not move tilt, got {tilt}"
