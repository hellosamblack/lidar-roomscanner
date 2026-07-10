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
    f.update(LEVEL, _mag_for_heading(0.0), 0)
    f.update(LEVEL, _mag_for_heading(0.0), 1_000_000)   # settle at 0
    # now a big orientation jump over a tiny dt => high angular rate
    s = math.sqrt(0.5)
    fast = (s, 0.0, 0.0, s)   # 90 deg in 1 ms
    f.update(fast, _mag_for_heading(90.0), 1_001_000)
    assert f.status == "gated:motion"


def test_gate_gimbal_holds_delta():
    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL, gimbal_margin_deg=15.0)
    a = math.radians(85.0) / 2   # pitch 85 deg -> within 15 of 90
    steep = (math.cos(a), 0.0, math.sin(a), 0.0)
    f.update(steep, _mag_for_heading(0.0), 10_000)
    f.update(steep, _mag_for_heading(0.0), 20_000)
    assert f.status == "gated:gimbal"


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
