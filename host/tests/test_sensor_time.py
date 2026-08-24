"""Unit tests for roomscan.sensor_time (issue #155).

The buffer's job is to answer "what was the orientation at tick T?" from
timestamped stream-9 samples, and each property here guards a specific way that
answer can silently go wrong:

- interpolation must follow the TIMESTAMPS, not any assumed constant phase —
  a hard-coded 7.76 ms would pass a golden-capture test and fail on DebugCapF,
  where the same offset re-measured at +5.13 ms (#143);
- it must never extrapolate — outside the bracket "no answer" is the correct
  answer, because a fabricated orientation feeds straight into the SLAM prior
  (BUG-067 turns prior error into fabricated translation);
- it must survive the uint32 LSM rollover (~26 h) — plain subtraction there
  reports a ±26 h interval once per wrap;
- interpolating across a dropped-frame hole is smoothing, not phase correction,
  so the span guard must refuse it.
"""
import math

import numpy as np
import pytest

from roomscan.sensor_time import (
    TICK_MASK,
    LiveGroupAggregator,
    PairedGroupFrame,
    TimestampedQuaternionBuffer,
    interpolate_quaternion_at,
    signed_tick_delta,
    slerp,
)


def _quat_z(deg: float) -> tuple[float, float, float, float]:
    h = math.radians(deg) / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _angle_deg(a, b) -> float:
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


# --- signed_tick_delta -----------------------------------------------------------------

def test_signed_delta_plain():
    assert signed_tick_delta(1000, 1350) == 350.0
    assert signed_tick_delta(1350, 1000) == -350.0
    assert signed_tick_delta(1000, 1000) == 0.0


def test_signed_delta_across_rollover():
    """A ~350-tick physical interval that happens to straddle the uint32 wrap must
    read as ~350 ticks, not ~2^32. This is the exact failure quat_offset_us() had
    before #155 made it modular."""
    a = TICK_MASK - 100          # 100 ticks before the wrap
    b = 250                      # 250 ticks after it
    assert signed_tick_delta(a, b) == 351.0
    assert signed_tick_delta(b, a) == -351.0


def test_signed_delta_fractional():
    """Stream 13's frame-ready instant is a fractional tick (latch minus its own
    delay); the delta must not truncate it."""
    assert signed_tick_delta(1000.25, 1350.75) == pytest.approx(350.5)


# --- slerp (canonical home moved here from slam.frames; contract unchanged) ------------

def test_slerp_endpoints_and_midpoint():
    a, b = _quat_z(0.0), _quat_z(90.0)
    assert slerp(a, b, 0.0) == pytest.approx(a)
    assert slerp(a, b, 1.0) == pytest.approx(b)
    assert _angle_deg(slerp(a, b, 0.5), _quat_z(45.0)) == pytest.approx(0.0, abs=1e-9)


def test_slerp_short_arc_negated_input():
    """q and -q are the same orientation; interpolation must take the short arc,
    not swing 315 degrees the long way round."""
    a = _quat_z(0.0)
    b = tuple(-c for c in _quat_z(90.0))
    mid = slerp(a, b, 0.5)
    assert _angle_deg(mid, _quat_z(45.0)) == pytest.approx(0.0, abs=1e-9)


def test_slerp_reexported_from_slam_frames():
    """Existing `roomscan.slam.frames.slerp` callers must keep working after the
    move to sensor_time."""
    from roomscan.slam.frames import slerp as slerp_frames
    assert slerp_frames is slerp


# --- buffer add() policies -------------------------------------------------------------

def test_add_rejects_degenerate_and_nonfinite():
    buf = TimestampedQuaternionBuffer()
    assert not buf.add(100, (0.0, 0.0, 0.0, 0.0))
    assert not buf.add(100, (float("nan"), 0.0, 0.0, 1.0))
    assert not buf.add(float("nan"), _quat_z(0.0))
    assert len(buf) == 0
    assert buf.add(100, _quat_z(0.0))


def test_add_rejects_duplicate_and_out_of_order():
    """First-seen wins: a duplicate tick or a late-arriving older sample must not
    displace what a query may already have interpolated against."""
    buf = TimestampedQuaternionBuffer()
    assert buf.add(100, _quat_z(0.0))
    assert buf.add(200, _quat_z(10.0))
    assert not buf.add(200, _quat_z(90.0))
    assert not buf.add(150, _quat_z(90.0))
    assert buf.at(200) == pytest.approx(_quat_z(10.0))


def test_add_normalizes():
    buf = TimestampedQuaternionBuffer()
    q = tuple(2.0 * c for c in _quat_z(30.0))
    assert buf.add(100, q)
    assert np.linalg.norm(buf.at(100)) == pytest.approx(1.0)


# --- buffer at() -----------------------------------------------------------------------

def test_at_exact_hit():
    buf = TimestampedQuaternionBuffer()
    buf.add(100, _quat_z(0.0))
    buf.add(200, _quat_z(10.0))
    assert buf.at(100) == pytest.approx(_quat_z(0.0))
    assert buf.at(200) == pytest.approx(_quat_z(10.0))


def test_at_follows_timestamps_not_a_constant():
    """The whole point of #155: the answer is a function of the query timestamp.
    Non-uniform spacing must change the interpolation fraction accordingly —
    nothing here may assume a 33 ms frame period or a 7.76 ms phase."""
    buf = TimestampedQuaternionBuffer()
    buf.add(1000, _quat_z(0.0))
    buf.add(2000, _quat_z(10.0))
    assert _angle_deg(buf.at(1250), _quat_z(2.5)) == pytest.approx(0.0, abs=1e-9)
    assert _angle_deg(buf.at(1750), _quat_z(7.5)) == pytest.approx(0.0, abs=1e-9)
    # Same rotation, uneven spacing: the fraction moves with the tick gap.
    buf2 = TimestampedQuaternionBuffer()
    buf2.add(1000, _quat_z(0.0))
    buf2.add(1500, _quat_z(10.0))
    assert _angle_deg(buf2.at(1250), _quat_z(5.0)) == pytest.approx(0.0, abs=1e-9)


def test_at_never_extrapolates():
    buf = TimestampedQuaternionBuffer()
    buf.add(1000, _quat_z(0.0))
    buf.add(2000, _quat_z(10.0))
    assert buf.at(999) is None
    assert buf.at(2001) is None
    assert buf.at(0) is None


def test_at_empty_and_single_sample():
    buf = TimestampedQuaternionBuffer()
    assert buf.at(100) is None
    buf.add(100, _quat_z(5.0))
    assert buf.at(100) == pytest.approx(_quat_z(5.0))   # exact hit is fine
    assert buf.at(101) is None                           # but nothing to bracket


def test_at_across_uint32_rollover():
    """Samples straddling the wrap must present a monotone ~512-tick interval; a
    query between them interpolates by the true fraction. Plain arithmetic would
    see the second sample ~2^32 ticks in the past and refuse (or worse)."""
    buf = TimestampedQuaternionBuffer()
    a = TICK_MASK - 255                      # 255 ticks before wrap
    b = 256                                  # 256 past it -> span 512
    assert buf.add(a, _quat_z(0.0))
    assert buf.add(b, _quat_z(10.0))
    q = buf.at((a + 128) % (TICK_MASK + 1))  # quarter of the span
    assert q is not None
    assert _angle_deg(q, _quat_z(2.5)) == pytest.approx(0.0, abs=1e-6)
    # And the query itself may sit past the wrap:
    q2 = buf.at(128)                         # 3/4 of the span
    assert _angle_deg(q2, _quat_z(7.5)) == pytest.approx(0.0, abs=1e-6)


def test_max_span_guard_refuses_holes():
    """A multi-frame capture hole still yields a bracket; interpolating across it
    is low-pass smoothing of the prior, which BUG-067 showed reads as improvement
    on a tripod regardless of correctness. The guard must return None instead."""
    buf = TimestampedQuaternionBuffer(max_span_ticks=500)
    buf.add(1000, _quat_z(0.0))
    buf.add(1400, _quat_z(5.0))
    buf.add(3000, _quat_z(40.0))             # 1600-tick hole
    assert buf.at(1200) is not None          # inside the healthy pair
    assert buf.at(2000) is None              # inside the hole -> refuse
    assert buf.at(3000) == pytest.approx(_quat_z(40.0))  # exact hit unaffected


def test_capacity_evicts_oldest():
    buf = TimestampedQuaternionBuffer(capacity=3)
    for i in range(6):
        assert buf.add(1000 + 100 * i, _quat_z(float(i)))
    assert len(buf) == 3
    assert buf.at(1050) is None              # evicted region: refuse, don't invent
    assert buf.at(1450) is not None


def test_clear_resets():
    buf = TimestampedQuaternionBuffer()
    buf.add(100, _quat_z(0.0))
    buf.clear()
    assert len(buf) == 0
    assert buf.at(100) is None
    assert buf.add(50, _quat_z(1.0))         # earlier tick OK after clear


# --- interpolate_quaternion_at helper --------------------------------------------------

def test_interpolate_quaternion_at_buffer():
    buf = TimestampedQuaternionBuffer()
    buf.add(1000, _quat_z(0.0))
    buf.add(2000, _quat_z(10.0))
    q = interpolate_quaternion_at(buf, 1500)
    assert q is not None
    assert _angle_deg(q, _quat_z(5.0)) == pytest.approx(0.0, abs=1e-6)


def test_interpolate_quaternion_at_samples_sequence():
    samples = [(1000.0, _quat_z(0.0)), (2000.0, _quat_z(10.0))]
    q = interpolate_quaternion_at(samples, 1250.0)
    assert q is not None
    assert _angle_deg(q, _quat_z(2.5)) == pytest.approx(0.0, abs=1e-6)


# --- LiveGroupAggregator (live streaming pairing / exact-group association #181) ------

class _FakeHeader:
    def __init__(self, seq: int, t_us: int, stream_id: int = 1):
        self.seq = seq
        self.t_us = t_us
        self.stream_id = stream_id


class _FakeSync:
    def __init__(self, lsm_ticks: int, quat_mid_ticks: int, latch_delay_us: int = 100,
                 quat_n: int = 16, valid: bool = True):
        self.lsm_ticks = lsm_ticks
        self.quat_mid_ticks = quat_mid_ticks
        self.latch_delay_us = latch_delay_us
        self.quat_n = quat_n
        self.valid = valid

    def frame_ready_ticks(self, tick_us: float) -> float:
        return self.lsm_ticks - self.latch_delay_us / tick_us

    def quat_offset_us(self, tick_us: float) -> float:
        return (signed_tick_delta(self.lsm_ticks, self.quat_mid_ticks) + self.latch_delay_us / tick_us) * tick_us


def test_live_group_aggregator_pairs_exact_group_quats_and_sync():
    """Depth frame N must pair with group N's quaternion and sync, not lag by 1 frame."""
    agg = LiveGroupAggregator(tick_us=20.0, apply_quat_phase=False)
    hdr1 = _FakeHeader(seq=1, t_us=100_000)
    outputs1 = {"depth": np.zeros((4, 4), dtype=np.float32)}

    # Group 1 depth arrives
    assert agg.add_depth(hdr1, outputs1) is None  # held pending matching group sensors

    # Group 1 sensors arrive
    agg.add_quat(1, _quat_z(10.0))
    agg.add_env_pressure(1, 101325.0)
    agg.add_imu_raw(1, "raw_batch_1")
    sync1 = _FakeSync(lsm_ticks=1000, quat_mid_ticks=1050)
    emitted = agg.add_sync(1, sync1)

    assert isinstance(emitted, PairedGroupFrame)
    assert emitted.header.seq == 1
    assert emitted.outputs is outputs1
    assert emitted.quat is not None
    assert _angle_deg(emitted.quat, _quat_z(10.0)) == pytest.approx(0.0, abs=1e-6)
    assert emitted.pressure_pa == 101325.0
    assert emitted.imu_raw == "raw_batch_1"
    assert emitted.imu_sync is sync1
    assert emitted.t_device_us == 100_000


def test_live_group_aggregator_applies_quat_phase_slerp_when_enabled():
    """With apply_quat_phase=True, orientation at frame_ready_ticks is bracketed
    by group N-1 and group N quat_mid_ticks and interpolated via SLERP."""
    agg = LiveGroupAggregator(tick_us=20.0, apply_quat_phase=True)

    # Seed group 1 quat in the buffer
    agg.add_quat(1, _quat_z(0.0))
    sync1 = _FakeSync(lsm_ticks=1000, quat_mid_ticks=1000)
    agg.add_sync(1, sync1)

    # Group 2 arrives: depth frame_ready at tick 1500 (lsm 1505, latch 100 us / 20 = 5 ticks)
    hdr2 = _FakeHeader(seq=2, t_us=133_333)
    outputs2 = {"depth": np.ones((4, 4), dtype=np.float32)}
    agg.add_depth(hdr2, outputs2)

    # Group 2 sensors: quat midpoint at tick 2000 with 10 degrees yaw
    agg.add_quat(2, _quat_z(10.0))
    sync2 = _FakeSync(lsm_ticks=1505, quat_mid_ticks=2000, latch_delay_us=100)
    emitted = agg.add_sync(2, sync2)

    assert emitted is not None
    # frame_ready_ticks = 1505 - 5 = 1500 (midpoint between 1000 and 2000 -> 5.0 deg)
    assert emitted.quat is not None
    assert _angle_deg(emitted.quat, _quat_z(5.0)) == pytest.approx(0.0, abs=1e-6)


def test_live_group_aggregator_flushes_on_next_depth_when_sync_missing():
    """When stream 13 is absent or dropped, arrival of the next depth frame flushes
    the previous pending frame with its group quaternion."""
    agg = LiveGroupAggregator(apply_quat_phase=False)
    hdr1 = _FakeHeader(seq=1, t_us=100_000)
    outputs1 = {"depth": np.zeros((2, 2))}
    agg.add_depth(hdr1, outputs1)
    agg.add_quat(1, _quat_z(15.0))

    hdr2 = _FakeHeader(seq=2, t_us=133_333)
    outputs2 = {"depth": np.ones((2, 2))}
    emitted1 = agg.add_depth(hdr2, outputs2)

    assert emitted1 is not None
    assert emitted1.header.seq == 1
    assert emitted1.quat is not None
    assert _angle_deg(emitted1.quat, _quat_z(15.0)) == pytest.approx(0.0, abs=1e-6)

    # Final flush gets frame 2
    emitted2 = agg.flush()
    assert emitted2 is not None
    assert emitted2.header.seq == 2


def test_live_group_aggregator_clear_resets():
    agg = LiveGroupAggregator()
    agg.add_depth(_FakeHeader(seq=1, t_us=1000), {"depth": 1})
    agg.add_quat(1, _quat_z(5.0))
    agg.clear()
    assert agg.flush() is None
    assert len(agg.quat_buf) == 0

