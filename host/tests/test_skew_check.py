"""Unit tests for the pure math in tools/skew_check.py (BUG-031's measurement).

Synthetic clocks only — no capture, no hardware. The properties under test are the
ones a before/after skew number depends on being true:

* the fit survives the real magnitudes (~1e10 µs). The prototype of this tool
  returned a slope of 0.044 instead of 1.003 on real data, silently, purely from
  conditioning — a nonsense residual would have been quoted as a measurement.
* windowing removes slow oscillator wander (which a single global fit reports as
  if it were per-frame jitter) while leaving genuine per-frame jitter alone.
* the windowed residual and its per-frame covariates stay index-aligned, since the
  trailing partial window is dropped. The CALIB load test compares residuals
  against a per-frame flag, so a silent misalignment there produces a plausible,
  wrong causal claim.
"""
import numpy as np
import pytest

from roomscan.protocol import FrameHeader, FrameType, ImuFifoTag, StreamId, pack_frame
from tools.skew_check import (
    LSM_SAMPLE_PERIOD_US,
    _fit_residual,
    collect_frames,
    summarize_us,
    windowed_residuals,
)


def _clocks(n=600, period_us=33_000.0, ratio=1.0033, jitter_us=0.0, wander_us=0.0,
            seed=0):
    """A ToF clock (y) and an LSM clock (x) that differ by a fixed ratio, plus
    optional per-frame jitter and a slow sinusoidal wander between them."""
    rng = np.random.default_rng(seed)
    y = 3.8e9 + np.arange(n) * period_us              # MCU µs, realistic magnitude
    x = 8.7e10 + np.arange(n) * (period_us / ratio)   # LSM µs, realistic magnitude
    if jitter_us:
        y = y + rng.normal(0.0, jitter_us, n)
    if wander_us:
        y = y + wander_us * np.sin(np.linspace(0, 2 * np.pi, n))
    return x, y


def test_fit_survives_real_magnitudes():
    """~1e10 µs values: an uncentred [x, 1] lstsq returns a nonsense slope."""
    x, y = _clocks()
    a, resid = _fit_residual(x, y)
    assert a == pytest.approx(1.0033, rel=1e-4)
    assert np.abs(resid).max() < 1.0        # noiseless: residual is float error only


def test_windowing_suppresses_slow_wander_that_a_global_fit_reports_as_jitter():
    """Suppresses, not eliminates: a window still contains some of the wander's
    curvature, so this is a ~4x reduction (665 -> 161 µs here), not a floor."""
    x, y = _clocks(n=1800, wander_us=1500.0)   # ±1.5 ms of drift, no per-frame noise
    _a, global_resid = _fit_residual(x, y)
    windowed, _slope, _used = windowed_residuals(x, y, window_s=20.0)
    g = summarize_us(global_resid)["rms_us"]
    w = summarize_us(windowed)["rms_us"]
    assert g > 500.0
    assert w < g / 3.0


def test_windowing_preserves_genuine_per_frame_jitter():
    x, y = _clocks(n=1800, jitter_us=1000.0)
    windowed, _slope, _used = windowed_residuals(x, y, window_s=20.0)
    assert summarize_us(windowed)["rms_us"] == pytest.approx(1000.0, rel=0.1)


def test_windowed_residual_and_used_mask_stay_aligned():
    """The trailing partial window is dropped, so residuals are shorter than the
    input — a per-frame covariate must be masked by `used` before comparison."""
    # 20 s at 33 ms is 606 frames, so 620 leaves a 14-frame tail — under min_points
    x, y = _clocks(n=620, jitter_us=10.0)
    resid, _slope, used = windowed_residuals(x, y, window_s=20.0)
    assert resid.size == int(used.sum()) < x.size
    covariate = np.arange(x.size)
    assert covariate[used].size == resid.size


def test_windowed_residual_recovers_an_injected_load_shift():
    """The CALIB-load test's whole content: a subset stamped systematically late
    must come back as a shift of that size, and the rest must not move."""
    x, y = _clocks(n=1800, jitter_us=200.0)
    flag = np.zeros(x.size, dtype=bool)
    flag[::64] = True                      # the CALIB cadence
    y = y - np.where(flag, 650.0, 0.0)     # those frames' pairing lands 650 µs late
    resid, _slope, used = windowed_residuals(x, y, window_s=20.0)
    f = flag[used]
    assert resid[f].mean() - resid[~f].mean() == pytest.approx(-650.0, abs=60.0)


def test_summarize_us_reports_magnitude_statistics():
    s = summarize_us(np.array([-3.0, 4.0, 0.0, 0.0]))
    assert s["n"] == 4
    assert s["rms_us"] == pytest.approx(2.5)
    assert s["max_us"] == pytest.approx(4.0)     # |residual|, not the signed max
    assert summarize_us(np.zeros(0)) == {"n": 0}


def test_fifo_estimator_floor_is_one_sample_period_of_phase():
    """The `fifo` estimator can never beat this, so an improvement claimed below
    it would be a measurement artefact (480 Hz ODR => 96 ticks => ~2.08 ms)."""
    assert LSM_SAMPLE_PERIOD_US == pytest.approx(2083.2, abs=1.0)
    assert LSM_SAMPLE_PERIOD_US / np.sqrt(12.0) == pytest.approx(601.4, abs=1.0)


def test_degenerate_inputs_do_not_raise():
    assert _fit_residual(np.zeros(5), np.arange(5.0))[0] != _fit_residual(
        np.zeros(5), np.arange(5.0))[0] or True     # nan slope, no exception
    resid, slope, used = windowed_residuals(np.arange(3.0), np.arange(3.0), 20.0)
    assert resid.size == 0 and np.isnan(slope) and not used.any()


# --- collect_frames(): Task 7 N:1 IMU_RAW-per-seq rework --------------------------------
#
# A decoupled IMU/env rate (SET_IMU_ENV_RATE, cmd 11) can drain the LSM FIFO several
# times off its own schedule while one ToF seq is still current (g_last_seq frozen
# between frames -- see rs_lsm_service_tick() in vl53l9_app.c and docs/protocol.md).
# collect_frames() used to keep `row["imu_raw"] = frame.payload`, a plain overwrite --
# every IMU_RAW payload but the last one sharing a seq was silently discarded. These
# tests build tiny synthetic captures (no real device/hardware involved) directly out
# of the wire protocol to pin that every payload sharing a seq now survives.

def _imu_raw_payload(ticks: list[int]) -> bytes:
    """One synthetic stream-11 payload: len(ticks) TIMESTAMP-tag (0x04) records, cnt=0,
    each carrying one tick value in its 4-byte LE field (the other 2 data bytes and the
    reserved byte are unused/zero -- decode_imu_raw never reads them for this tag)."""
    tag_byte = int(ImuFifoTag.TIMESTAMP) << 3   # TAG_CNT=0, bit0 (not_used0) always 0
    out = bytearray()
    for t in ticks:
        rec = bytearray(8)
        rec[0] = tag_byte
        rec[1:5] = int(t).to_bytes(4, "little")
        out += rec
    return bytes(out)


def _wire_frame(frame_type: int, stream_id: int, seq: int, t_us: int, payload: bytes) -> bytes:
    hdr = FrameHeader(frame_type, stream_id, 0, seq, t_us, 0, 0, len(payload))
    return pack_frame(hdr, payload)


def test_collect_frames_retains_all_imu_raw_sends_sharing_one_frozen_seq(tmp_path):
    """The rework's whole point: 2+ decoupled IMU/env sends between two ToF frames
    (sharing one frozen seq) must ALL be retained, not last-write-wins overwritten."""
    seq = 42
    raw1 = _imu_raw_payload([100, 200, 300])   # first off-cycle drain, 3 TIMESTAMP words
    raw2 = _imu_raw_payload([400, 500])        # second off-cycle drain, 2 more
    data = (_wire_frame(FrameType.DATA, StreamId.RAW_3DMD, seq, 1_000_000, b"\x00")
           + _wire_frame(FrameType.DATA, StreamId.IMU_RAW, seq, 1_000_000, raw1)
           + _wire_frame(FrameType.DATA, StreamId.IMU_RAW, seq, 1_000_000, raw2))
    p = tmp_path / "cap.bin"
    p.write_bytes(data)

    rows, tick_us, _tick_from_device = collect_frames(p)

    assert len(rows) == 1
    row = rows[0]
    assert row["seq"] == seq
    assert row["n_imu_raw_sends"] == 2                       # both sends counted
    assert row["n_ts"] == 5                                  # 3 + 2 -- NOT just raw2's 2
    # last tick overall is raw2's last (500), not raw1's (300) -- arrival order preserved
    assert row["ts_last_us"] == pytest.approx(500 * tick_us)


def test_collect_frames_last_write_wins_would_have_lost_the_earlier_send():
    """Pin the DEFECT this replaces: proves the new test above can actually fail --
    a last-write-wins implementation (row["imu_raw"] = payload, no list) would report
    n_ts=2 (only raw2's words) and silently drop raw1's 3 entirely. Reimplements just
    enough of the old overwrite behavior in isolation (no capture I/O) to show the
    numbers the fixed collect_frames() must NOT reproduce."""
    from roomscan.protocol import decode_imu_raw
    raw1 = _imu_raw_payload([100, 200, 300])
    raw2 = _imu_raw_payload([400, 500])
    old_style_row: dict = {}
    for payload in (raw1, raw2):
        old_style_row["imu_raw"] = payload   # last-write-wins, the pre-Task-7 behavior
    batch = decode_imu_raw(old_style_row["imu_raw"], tick_us=21.7)
    assert batch.timestamp_ticks.size == 2                   # would have lost raw1's 3
    assert int(batch.timestamp_ticks.size) != 5               # != the correct total


def test_collect_frames_single_imu_raw_send_is_unaffected_coupled_mode(tmp_path):
    """Coupled mode (today's default, rate 0): exactly one IMU_RAW per seq, so the
    rework changes nothing observable -- same n_ts/ts_last_us as a plain single decode."""
    seq = 7
    raw = _imu_raw_payload([10, 20, 30])
    data = (_wire_frame(FrameType.DATA, StreamId.RAW_3DMD, seq, 500_000, b"\x00")
           + _wire_frame(FrameType.DATA, StreamId.IMU_RAW, seq, 500_000, raw))
    p = tmp_path / "cap.bin"
    p.write_bytes(data)
    rows, tick_us, _ = collect_frames(p)
    assert len(rows) == 1
    assert rows[0]["n_imu_raw_sends"] == 1
    assert rows[0]["n_ts"] == 3
    assert rows[0]["ts_last_us"] == pytest.approx(30 * tick_us)


def test_collect_frames_two_tof_frames_each_keep_their_own_imu_raw_sends(tmp_path):
    """Multiple seqs in one capture must not cross-contaminate each other's IMU_RAW
    lists -- seq 1's two sends and seq 2's one send stay correctly partitioned."""
    data = (
        _wire_frame(FrameType.DATA, StreamId.RAW_3DMD, 1, 0, b"\x00")
        + _wire_frame(FrameType.DATA, StreamId.IMU_RAW, 1, 0, _imu_raw_payload([1, 2]))
        + _wire_frame(FrameType.DATA, StreamId.IMU_RAW, 1, 0, _imu_raw_payload([3]))
        + _wire_frame(FrameType.DATA, StreamId.RAW_3DMD, 2, 33_000, b"\x00")
        + _wire_frame(FrameType.DATA, StreamId.IMU_RAW, 2, 33_000, _imu_raw_payload([4, 5, 6]))
    )
    p = tmp_path / "cap.bin"
    p.write_bytes(data)
    rows, _tick_us, _ = collect_frames(p)
    assert len(rows) == 2
    by_seq = {r["seq"]: r for r in rows}
    assert by_seq[1]["n_imu_raw_sends"] == 2
    assert by_seq[1]["n_ts"] == 3
    assert by_seq[2]["n_imu_raw_sends"] == 1
    assert by_seq[2]["n_ts"] == 3


def test_collect_frames_records_signed_quat_lead(tmp_path):
    """#155: the stream-9 batch-midpoint lead must be reported per frame, SIGNED —
    it measured +5.1 ms on 2026-08 rigs but NEGATIVE on officeFullScanAug6, so a
    magnitude (or an assumed constant) would hide exactly the fact that killed the
    constant-offset plan. Wrap-safety and the value itself are protocol.py's tests;
    this pins the plumbing into skew_check rows."""
    import struct

    def sync_payload(lsm, mid):
        return struct.pack("<IIIIHHBB", lsm, 217, 24_109, mid, 44, 15, 1, 0)

    data = (_wire_frame(FrameType.DATA, StreamId.RAW_3DMD, 42, 1_000_000, b"\x00")
            + _wire_frame(FrameType.DATA, StreamId.IMU_SYNC, 42, 1_000_000,
                          sync_payload(10_010, 10_358))       # lead: +358 ticks + latch
            + _wire_frame(FrameType.DATA, StreamId.RAW_3DMD, 43, 1_033_333, b"\x00")
            + _wire_frame(FrameType.DATA, StreamId.IMU_SYNC, 43, 1_033_333,
                          sync_payload(20_010, 19_662)))      # NEGATIVE: mid before latch
    p = tmp_path / "cap.bin"
    p.write_bytes(data)

    rows, tick_us, _ = collect_frames(p)
    assert len(rows) == 2
    # +348 ticks (mid - latch) + 10 ticks (latch back-out at 217 us) = +358 ticks
    assert rows[0]["quat_lead_us"] == pytest.approx(358 * tick_us)
    # -348 ticks + 10 ticks = -338 ticks: the sign must survive
    assert rows[1]["quat_lead_us"] == pytest.approx(-338 * tick_us)
