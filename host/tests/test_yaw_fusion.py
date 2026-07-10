import math

import numpy as np
import pytest

from roomscan.magcal import MagCalibration
from roomscan.sensors import YawFusion, quat_yaw_deg, quat_pitch_deg, wrap180

IDENT_CAL = MagCalibration(offset=(0.0, 0.0, 0.0),
                           matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                           field_ut=50.0)
LEVEL = (1.0, 0.0, 0.0, 0.0)


def _mag_for_heading(deg):
    # level device: world == body; mag in horizontal plane pointing so that
    # tilt_compensated_heading returns `deg` (atan2(my, mx)).
    r = math.radians(deg)
    return (50.0 * math.cos(r), 50.0 * math.sin(r), 0.0)


def test_converges_to_mag_heading_when_static():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL)
    mag = _mag_for_heading(30.0)
    t = 0
    for _ in range(200):          # ~2 s at 100 Hz
        t += 10_000               # 10 ms
        f.update(LEVEL, mag, t)
    fused = f.fused_quat()
    assert f.status == "active"
    assert wrap180(quat_yaw_deg(fused) - 30.0) == pytest.approx(0.0, abs=1.0)


def test_lowpass_tracks_moving_target():
    # Snap to 30 deg, then STEP the mag heading to 90 deg and verify the
    # low-pass increment actually drives delta toward the new target with the
    # expected ~tau time constant (residual ~1/e after one tau). A broken gain
    # formula (sign flip, or tau/(dt+tau) inverted) fails this.
    dt_us, tau_s = 10_000, 1.0
    f = YawFusion(tau_s=tau_s, calibration=IDENT_CAL)
    t = 0
    for _ in range(5):                       # settle at 30 deg
        t += dt_us
        f.update(LEVEL, _mag_for_heading(30.0), t)
    assert wrap180(quat_yaw_deg(f.fused_quat()) - 30.0) == pytest.approx(0.0, abs=0.5)
    # step target to 90 deg; after ~1 tau (100 steps @ 10 ms) residual ~ 1/e of 60 deg
    for _ in range(100):
        t += dt_us
        f.update(LEVEL, _mag_for_heading(90.0), t)
    yaw_1tau = quat_yaw_deg(f.fused_quat())
    residual = 90.0 - yaw_1tau               # remaining error toward target
    assert residual == pytest.approx(60.0 / math.e, abs=6.0)   # ~22 deg, not 0 and not 60
    # after several more taus it converges to the target
    for _ in range(500):
        t += dt_us
        f.update(LEVEL, _mag_for_heading(90.0), t)
    assert wrap180(quat_yaw_deg(f.fused_quat()) - 90.0) == pytest.approx(0.0, abs=1.0)


def test_rejects_sflp_yaw_drift():
    # THE property the feature exists to provide: with the device physically
    # STATIC (mag fixed in the body frame), a drifting SFLP yaw must NOT drag the
    # fused yaw along. The buggy full-quat de-tilt made fused_yaw follow the drift.
    f = YawFusion(tau_s=0.3, calibration=IDENT_CAL)
    mag_body = _mag_for_heading(20.0)   # body-fixed field; device truly static
    t = 0
    # SFLP yaw ramps 0 -> 40 deg (pure drift), then holds at 40 to let it settle
    for i in range(200):
        t += 10_000
        d = math.radians(40.0 * i / 200) / 2
        f.update((math.cos(d), 0.0, 0.0, math.sin(d)), mag_body, t)
    for _ in range(300):
        t += 10_000
        d = math.radians(40.0) / 2
        f.update((math.cos(d), 0.0, 0.0, math.sin(d)), mag_body, t)
    # fused yaw stays at the absolute heading (~20), NOT dragged to 20+40=60
    assert wrap180(quat_yaw_deg(f.fused_quat()) - 20.0) == pytest.approx(0.0, abs=2.0)


def test_snaps_on_first_valid_sample():
    f = YawFusion(tau_s=100.0, calibration=IDENT_CAL)
    f.update(LEVEL, _mag_for_heading(80.0), 10_000)   # first: init, no dt
    f.update(LEVEL, _mag_for_heading(80.0), 20_000)   # second: snaps despite huge tau
    assert wrap180(quat_yaw_deg(f.fused_quat()) - 80.0) == pytest.approx(0.0, abs=1.0)


def test_gate_anomaly_holds_delta():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL, anomaly_frac=0.3)
    f.update(LEVEL, _mag_for_heading(0.0), 10_000)
    f.update(LEVEL, _mag_for_heading(0.0), 20_000)    # establish delta ~0
    strong = tuple(3.0 * c for c in _mag_for_heading(90.0))  # |mag| far from field
    f.update(LEVEL, strong, 30_000)
    assert f.status == "gated:anomaly"
    assert wrap180(quat_yaw_deg(f.fused_quat()) - 0.0) == pytest.approx(0.0, abs=1.0)


def test_gate_motion_holds_delta():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL, motion_rate_dps=40.0)
    f.update(LEVEL, _mag_for_heading(25.0), 0)
    f.update(LEVEL, _mag_for_heading(25.0), 1_000_000)   # snap: delta -> 25
    held = f._delta
    assert held == pytest.approx(25.0, abs=1.0)
    # now a big orientation jump over a tiny dt => high angular rate; a wrong
    # mag heading (90) would move delta if the gate didn't freeze it
    s = math.sqrt(0.5)
    fast = (s, 0.0, 0.0, s)   # 90 deg in 1 ms
    f.update(fast, _mag_for_heading(90.0), 1_001_000)
    assert f.status == "gated:motion"
    assert f._delta == pytest.approx(held)   # delta held, not pulled toward 90


def test_gate_gimbal_holds_delta():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL, gimbal_margin_deg=15.0)
    f.update(LEVEL, _mag_for_heading(25.0), 10_000)
    f.update(LEVEL, _mag_for_heading(25.0), 20_000)     # snap: delta -> 25
    held = f._delta
    assert held == pytest.approx(25.0, abs=1.0)
    a = math.radians(85.0) / 2   # pitch 85 deg -> within 15 of 90
    steep = (math.cos(a), 0.0, math.sin(a), 0.0)
    f.update(steep, _mag_for_heading(90.0), 30_000)
    assert f.status == "gated:gimbal"
    assert f._delta == pytest.approx(held)   # delta held despite a valid-looking mag


def test_no_calibration_returns_raw():
    f = YawFusion(tau_s=1.0, calibration=None)
    f.update(LEVEL, (1.0, 0.0, 0.0), 10_000)
    assert f.status == "gated:no-cal"
    assert f.fused_quat() == pytest.approx(LEVEL)


def test_tilt_preserved():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL)
    a = math.radians(20.0) / 2
    tilted = (math.cos(a), math.sin(a), 0.0, 0.0)   # 20 deg roll
    t = 0
    for _ in range(100):
        t += 10_000
        f.update(tilted, _mag_for_heading(45.0), t)
    assert quat_pitch_deg(f.fused_quat()) == pytest.approx(quat_pitch_deg(tilted), abs=0.5)


def _roll_deg(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))


def test_tilt_preserved_pitch_and_roll():
    # Combined 15 deg roll + 25 deg pitch: grafting yaw must leave BOTH unchanged.
    from roomscan.sensors import quat_mul
    ar, ap = math.radians(15.0) / 2, math.radians(25.0) / 2
    qroll = (math.cos(ar), math.sin(ar), 0.0, 0.0)
    qpitch = (math.cos(ap), 0.0, math.sin(ap), 0.0)
    q = quat_mul(qpitch, qroll)
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL)
    t = 0
    for _ in range(100):
        t += 10_000
        f.update(q, _mag_for_heading(70.0), t)
    fused = f.fused_quat()
    assert quat_pitch_deg(fused) == pytest.approx(quat_pitch_deg(q), abs=0.5)
    assert _roll_deg(fused) == pytest.approx(_roll_deg(q), abs=0.5)
