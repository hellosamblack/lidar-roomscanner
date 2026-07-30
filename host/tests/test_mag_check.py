"""`tools/mag_check.py` -- scoring a magnetometer calibration against a capture.

Built 2026-07-30 to validate the BUG-030 re-fit. Tests drive REAL wire bytes
through `pack_frame` and the real `StreamDecoder`, so a protocol change breaks
them the same way it breaks the tool.

The pair that carries the point:
  * `test_good_calibration_*` -- a correct fit measured while the room's field
    drifts must come out GOOD with a flat tilt table.
  * `test_bad_calibration_*` -- the superseded 2026-07-15-style hard-iron error
    must ramp monotonically across the tilt table, the BUG-030 signature.
"""
from __future__ import annotations

import json
import math
import struct

import numpy as np
import pytest

from roomscan.magcal import MagCalibration
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame
from roomscan.sensors import AXIS_CONVENTION
from tools import mag_check as mc

FIELD = 50.0
HARD_IRON = np.array([44.4, -27.6, -41.7])
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _cal(offset=HARD_IRON, field=FIELD):
    return MagCalibration(offset=tuple(float(v) for v in offset), matrix=IDENTITY,
                          field_ut=field)


def _raw_from_body(body_dirs, offset=HARD_IRON, field=FIELD):
    v = np.asarray(body_dirs, dtype=np.float64).reshape(-1, 3) * field
    v = v @ np.linalg.inv(np.asarray(AXIS_CONVENTION, dtype=np.float64)).T
    return v + np.asarray(offset, dtype=np.float64)


def _quat_pitch(deg):
    """Quaternion tilting the boresight (body +Z) `deg` away from straight up."""
    a = math.radians(deg) / 2.0
    return (math.cos(a), math.sin(a), 0.0, 0.0)


def _write_capture(path, mags, quats, rate_hz=30.0):
    """Synthesise a capture carrying stream 10 (ENV) + stream 9 (IMU_QUAT)."""
    out = bytearray()
    for i, (m, q) in enumerate(zip(mags, quats)):
        t_us = int(i * 1e6 / rate_hz)
        qp = struct.pack("<4f", *q)
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, i, t_us,
                                      0, 0, len(qp)), qp)
        ep = struct.pack("<5f", 101325.0, float(m[0]), float(m[1]), float(m[2]), 21.0)
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.ENV, 0, i, t_us,
                                      0, 0, len(ep)), ep)
    path.write_bytes(bytes(out))
    return path


def _tilt_walk(n=3600, rate_hz=30.0, seed=5):
    """A capture's worth of attitudes: the boresight sweeps 0..180 deg from
    vertical several times (so every tilt bin fills) while the heading spins,
    and the ambient field level breathes as if the operator were walking."""
    t = np.arange(n) / rate_hz
    tilt = 90.0 - 90.0 * np.cos(2 * np.pi * t / 20.0)      # 0..180 and back
    yaw = 2 * np.pi * t / 7.0
    quats = np.array([_quat_pitch(v) for v in tilt])
    # body-frame field direction implied by that attitude, plus a heading spin
    dirs = np.column_stack([
        np.sin(np.radians(tilt)) * np.cos(yaw),
        np.sin(np.radians(tilt)) * np.sin(yaw),
        np.cos(np.radians(tilt)),
    ])
    drift = 1.0 + 0.06 * np.sin(2 * np.pi * t / 60.0)
    return dirs * drift[:, None], quats, seed


@pytest.fixture(scope="module")
def walk_capture(tmp_path_factory):
    dirs, quats, _ = _tilt_walk()
    p = tmp_path_factory.mktemp("mag") / "walk.bin"
    _write_capture(p, _raw_from_body(dirs), quats)
    return p


@pytest.fixture(scope="module")
def cal_files(tmp_path_factory):
    d = tmp_path_factory.mktemp("cals")
    good, bad = d / "good.json", d / "bad.json"
    _cal().save(good)
    # the BUG-030 failure mode: hard iron wrong by ~59 uT, mostly in z
    _cal(offset=HARD_IRON + np.array([-19.0, 22.0, 51.0])).save(bad)
    return good, bad


def test_reads_both_streams_and_pairs_them(walk_capture):
    mags, t_s, quats = mc.read_mag_frames(walk_capture)
    assert mags.shape == (3600, 3)
    assert quats.shape == (3600, 4)
    assert t_s[0] == 0.0
    assert t_s[-1] == pytest.approx(3599 / 30.0, rel=1e-6)


def test_good_calibration_is_good_and_flat(walk_capture, cal_files):
    r = mc.check_capture(walk_capture, cal_files[0])
    assert r["verdict"] == "good"
    assert r["attitude"]["attitude_locked_pct"] < 2.0
    assert r["has_orientation"]
    means = [row["mean_ut"] for row in r["tilt"]]
    assert len(means) >= 8
    # flat: no systematic ramp with tilt, only the room's +-6% breathing
    assert max(means) / min(means) < 1.10


def test_bad_calibration_ramps_across_the_tilt_table(walk_capture, cal_files):
    r = mc.check_capture(walk_capture, cal_files[1])
    assert r["verdict"] == "bad"
    means = [row["mean_ut"] for row in r["tilt"]]
    assert max(means) / min(means) > 1.5
    # monotone in tilt -- BUG-030's actual signature, not just "noisy"
    assert means == sorted(means) or means == sorted(means, reverse=True)


def test_the_tilt_ramp_catches_what_detrending_hides(walk_capture, cal_files):
    """Why `verdict` is not the attitude number alone. This walk holds each
    attitude family longer than the 5 s detrend window, so a 59 uT hard-iron
    error is absorbed into the trend and `attitude` scores GOOD -- while the
    detrend-free tilt ramp reads 3.5x. Either one alone would have passed a
    BUG-030-grade calibration."""
    r = mc.check_capture(walk_capture, cal_files[1])
    assert r["attitude"]["verdict"] == "good"
    assert r["tilt_ramp"]["verdict"] == "bad"
    assert r["verdict"] == "bad"


def test_unknown_blocks_a_clean_pass_but_a_measured_failure_outranks_it():
    assert mc._combine("good", "unknown") == "unknown"
    assert mc._combine("bad", "unknown") == "bad"
    assert mc._combine("good", "marginal") == "marginal"
    assert mc._combine("good", "good") == "good"


def test_field_consistency_is_reported_but_is_not_the_verdict(walk_capture, cal_files):
    """The tumble-time metric condemns the good fit on a walk (its bias term
    absorbs the room). It must still be reported, and must not drive `verdict`."""
    r = mc.check_capture(walk_capture, cal_files[0])
    assert r["field"]["verdict"] in ("marginal", "bad")
    assert r["verdict"] == "good"


def test_missing_capture_and_missing_calibration_are_errors(tmp_path, cal_files):
    assert "error" in mc.check_capture(tmp_path / "nope.bin", cal_files[0])
    p = tmp_path / "empty.bin"
    _write_capture(p, [], [])
    r = mc.check_capture(p, cal_files[0])
    assert "no stream 10" in r["error"]
    dirs, quats, _ = _tilt_walk(n=100)
    q = tmp_path / "ok.bin"
    _write_capture(q, _raw_from_body(dirs), quats)
    assert "no readable calibration" in mc.check_capture(q, tmp_path / "absent.json")["error"]


def test_capture_without_orientation_reports_no_tilt_table(tmp_path, cal_files):
    """A ToF+ENV capture with no stream 9 must say so, not silently pile every
    sample into one tilt bin and call the table flat."""
    dirs, _q, _ = _tilt_walk(n=900)
    out = bytearray()
    for i, m in enumerate(_raw_from_body(dirs)):
        ep = struct.pack("<5f", 101325.0, *[float(v) for v in m], 21.0)
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.ENV, 0, i,
                                      int(i * 1e6 / 30.0), 0, 0, len(ep)), ep)
    p = tmp_path / "noquat.bin"
    p.write_bytes(bytes(out))
    r = mc.check_capture(p, cal_files[0])
    assert r["has_orientation"] is False
    assert r["tilt"] == []
    assert "no stream 9" in mc.format_report(r)


def test_cli_compares_calibrations(walk_capture, cal_files, capsys):
    rc = mc.main([str(walk_capture), "--compare", str(cal_files[0]), str(cal_files[1]), "--json"])
    assert rc == 0
    reports = json.loads(capsys.readouterr().out)
    assert [r["verdict"] for r in reports] == ["good", "bad"]


def test_cli_prints_a_human_report(walk_capture, cal_files, capsys):
    assert mc.main([str(walk_capture), "--cal", str(cal_files[0])]) == 0
    out = capsys.readouterr().out
    assert "ATTITUDE-LOCKED ERROR" in out
    assert "GOOD" in out
