import math

import numpy as np
import pytest

from roomscan.magcal import MagCalibration
from roomscan.sensors import (AXIS_CONVENTION, YawFusion, absolute_heading, graft_yaw,
                              graft_yaw_error_deg, magnetic_north_bearing_deg, quat_mul,
                              quat_pitch_deg, quat_to_matrix, quat_yaw_deg,
                              tilt_from_down_deg, wrap180, yaw_twist_deg)

IDENT_CAL = MagCalibration(offset=(0.0, 0.0, 0.0),
                           matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                           field_ut=50.0)
LEVEL = (1.0, 0.0, 0.0, 0.0)


def _mag_for_heading(deg):
    """Raw body mag for a device at `LEVEL` (body == world) whose world frame
    has magnetic north at COMPASS BEARING `deg` -- i.e. the frame's +X datum
    sits `deg` west of north, which is exactly the drift the fusion corrects.

    Compass bearing, CW: `atan2(-y, x)`. It used to be built CCW (`atan2(y, x)`,
    `tilt_compensated_heading`'s convention), which is where BUG-058's mirrored
    fused quat hid -- every assertion here was on a magnitude, and both the
    field and the filter were wound the same wrong way.
    """
    r = math.radians(deg)
    target = np.array([50.0 * math.cos(r), -50.0 * math.sin(r), 0.0])
    mag = np.linalg.solve(AXIS_CONVENTION, target)
    return tuple(mag)


def _north_bearing_after_fusion(f, raw_mag):
    """Where magnetic north lands in the FUSED frame. The fusion's whole job is
    to drive this to 0 -- assert it, not just the size of some yaw."""
    return magnetic_north_bearing_deg(
        f.fused_quat(), tuple(AXIS_CONVENTION @ IDENT_CAL.apply(raw_mag)))


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
    # ...and the reason that number is right: north ends up AT north.
    assert _north_bearing_after_fusion(f, mag) == pytest.approx(0.0, abs=1.0)


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
    assert _north_bearing_after_fusion(f, mag_body) == pytest.approx(0.0, abs=2.0)


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


def test_normal_upright_grip_is_not_gated():
    """The pose the instrument is actually held in must let fusion RUN (BUG-051).

    The old gimbal gate froze within 15 deg of |ZYX pitch| = 90. This device's
    SFLP body frame has X = Up, so ZYX pitch is the elevation of the structural
    up axis: held upright it is ~87 deg and the gate tripped forever, reporting
    "Gimbal lock" while the boresight sat ~2 deg off horizontal. Reintroducing
    the gate fails this test, which is the point -- every other test in this
    file uses LEVEL (identity), the one attitude the bug could not reach.
    """
    # The owner's live quat, read off /ws in the normal handheld grip.
    grip = (0.604421, 0.35965, 0.593567, -0.391159)
    assert abs(quat_pitch_deg(grip)) > 75.0          # the old gate's trip region
    assert abs(tilt_from_down_deg(                    # ...yet aimed nearly level
        tuple(quat_to_matrix(*grip).T @ np.array([0.0, 0.0, -1.0])))) < 15.0

    f = YawFusion(tau_s=1.0, calibration=IDENT_CAL)
    f.update(grip, _mag_for_heading(25.0), 10_000)
    f.update(grip, _mag_for_heading(25.0), 20_000)
    assert f.status == "active"


DIP_DEG, FIELD_UT = 66.0, 50.0
B_WORLD = np.array([FIELD_UT * math.cos(math.radians(DIP_DEG)), 0.0,
                    -FIELD_UT * math.sin(math.radians(DIP_DEG))])   # north + down


def _axis_quat(axis, deg):
    a = math.radians(deg) / 2.0
    v = [0.0, 0.0, 0.0]
    v["xyz".index(axis)] = math.sin(a)
    return (math.cos(a), *v)


# Base grip: boresight (body +Z) level and North, body +X pointing DOWN.
_BASE = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
_W = math.sqrt(1.0 + _BASE.trace()) / 2.0
_QBASE = (_W, (_BASE[2, 1] - _BASE[1, 2]) / (4 * _W),
          (_BASE[0, 2] - _BASE[2, 0]) / (4 * _W), (_BASE[1, 0] - _BASE[0, 1]) / (4 * _W))


def _pose(bearing_deg, roll_deg=0.0, pitch_deg=0.0):
    """(quat, body-frame field) for a device aimed at compass `bearing_deg`,
    rolled `roll_deg` about its own boresight and pitched `pitch_deg` up.

    World is X=North, Y=West, Z=Up, so bearing is a NEGATIVE rotation about
    world Z (pre-multiplied); roll and pitch are about BODY axes (post-
    multiplied). Composed as quaternions, not by converting the product matrix
    -- that conversion's `sqrt(1 + trace)` branch degenerates at bearings near
    180 deg, which is exactly where this is meant to be trusted.
    """
    q = quat_mul(_axis_quat("z", -bearing_deg), _QBASE)
    q = quat_mul(q, _axis_quat("y", -pitch_deg))
    q = quat_mul(q, _axis_quat("z", roll_deg))
    return q, tuple(quat_to_matrix(*q).T @ B_WORLD)


def test_absolute_heading_recovers_true_bearing_at_the_operating_pose():
    """`absolute_heading` must return the boresight's true compass bearing at
    the attitude the device is held in, not just at bearings of 0/90/180/270.

    This is the test the ZYX yaw-strip failed (BUG-051): it was exact on the
    axis-aligned bearings -- which is why it survived review and a 180/90 deg
    eyeball check -- and reported 26.57 deg for a true 45 deg bearing in the
    normal grip, an 18.4 deg systematic error. Off-axis bearings are load-
    bearing here; do not "simplify" this to the cardinal four.

    The heading is ABSOLUTE, so there is no convention offset to subtract --
    asserting the bare value is deliberate. This test used to fit and remove
    one, which let it pass under a heading that was 180 deg out.
    """
    for bearing in (0.0, 30.0, 45.0, 90.0, 137.0, 180.0, 270.0, 315.0):
        q, mag_body = _pose(bearing)
        r = quat_to_matrix(*q)
        true = math.degrees(math.atan2(-r[1, 2], r[0, 2])) % 360.0   # world Y = West
        assert true == pytest.approx(bearing, abs=1e-6)
        got = absolute_heading(q, mag_body)
        assert wrap180(got - bearing) == pytest.approx(0.0, abs=1e-6), (
            f"bearing {bearing}: heading off by {wrap180(got - bearing):.2f} deg")


def test_absolute_heading_is_unmoved_by_roll_about_the_boresight():
    """BUG-058, and the axis the BUG-051 test held fixed: rolling the device in
    the hand while aiming at one bearing is not a turn, and must not read as one.

    `captures/NorthFacingRoll.bin` is the owner's demonstration -- 154 deg of
    reported heading swing over 153 deg of roll at a fixed bearing, slope
    -0.978, because `yaw_twist_deg` (swing-twist about world Z) absorbs roll
    about a horizontal boresight degree for degree. Pitch is swept alongside it
    because that axis was never covered either; it happened to be fine.
    """
    for bearing in (0.0, 45.0, 137.0, 300.0):
        for roll in (-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0):
            q, mag_body = _pose(bearing, roll_deg=roll)
            got = absolute_heading(q, mag_body)
            assert wrap180(got - bearing) == pytest.approx(0.0, abs=1e-6), (
                f"bearing {bearing} rolled {roll}: heading moved "
                f"{wrap180(got - bearing):.2f} deg")
        for pitch in (-60.0, -30.0, 30.0, 60.0):
            q, mag_body = _pose(bearing, pitch_deg=pitch)
            assert wrap180(absolute_heading(q, mag_body) - bearing) == pytest.approx(0.0, abs=1e-6)


def test_absolute_heading_is_none_where_no_bearing_exists():
    """Aimed at the ceiling there is no compass bearing to report, and the old
    formula reported one anyway -- a number that was purely a function of roll.
    None is the honest answer; `web.orientation_view` turns it into a reason
    string and the client into a dash."""
    for pitch in (85.0, 90.0, -85.0, -90.0):
        q, mag_body = _pose(137.0, pitch_deg=pitch)
        assert absolute_heading(q, mag_body) is None
    q, mag_body = _pose(137.0, pitch_deg=75.0)          # 15 deg of margin: fine
    assert absolute_heading(q, mag_body) == pytest.approx(137.0, abs=1e-6)
    # A vertical field has no north to point at, at any aim.
    q, _ = _pose(137.0)
    assert absolute_heading(q, tuple(quat_to_matrix(*q).T @ np.array([0.0, 0.0, -50.0]))) is None


def test_fusion_lands_the_fused_boresight_on_the_true_magnetic_bearing():
    """End-to-end, against ground truth: feed a quat carrying a known yaw drift
    plus the true field, and the FUSED quat's boresight must come out at the
    device's real bearing -- at any roll, any pitch, any drift.

    Under BUG-058 this test reads -bearing: `heading - yaw` differenced a
    device bearing against a world-Z twist, so the device term did not cancel
    and every turn was counted twice. The mirror was invisible to every
    assertion the suite had, because they all pinned |delta| at LEVEL, where
    the two conventions coincide.
    """
    from roomscan.sensors import boresight_bearing_deg
    for bearing, roll, pitch, drift in ((0.0, 0.0, 0.0, 25.0), (45.0, 0.0, 0.0, 25.0),
                                        (137.0, 0.0, 0.0, -70.0), (300.0, 0.0, 0.0, 0.0),
                                        (45.0, 90.0, 0.0, 25.0), (45.0, -135.0, 0.0, 25.0),
                                        (137.0, 30.0, 10.0, 70.0), (300.0, 180.0, -40.0, 12.0)):
        q_true, mag_body = _pose(bearing, roll_deg=roll, pitch_deg=pitch)
        q_est = graft_yaw(q_true, drift)          # SFLP datum has wandered off north
        raw = tuple(np.linalg.solve(AXIS_CONVENTION, np.asarray(mag_body)))
        f = YawFusion(tau_s=0.2, calibration=IDENT_CAL)
        t = 0
        for _ in range(400):
            t += 10_000
            f.update(q_est, raw, t)
        assert f.status == "active"
        fused = boresight_bearing_deg(f.fused_quat())
        assert wrap180(fused - bearing) == pytest.approx(0.0, abs=0.5), (
            f"bearing {bearing} roll {roll} pitch {pitch} drift {drift}: "
            f"fused boresight at {fused:.2f}")


def test_yaw_twist_is_the_negated_graft_yaw_error_from_identity():
    """`yaw_twist_deg` and `graft_yaw_error_deg` are the same swing-twist twist
    about world Z -- primitive and loop form. Pinned so they cannot drift."""
    for q in (LEVEL, (0.604421, 0.35965, 0.593567, -0.391159),
              (0.5, -0.5, 0.5, 0.5), (0.1, 0.2, -0.3, 0.9)):
        assert wrap180(yaw_twist_deg(q)
                       + graft_yaw_error_deg((1.0, 0.0, 0.0, 0.0), q)) == pytest.approx(0.0, abs=1e-9)


def test_yaw_twist_is_exactly_additive_under_graft_yaw():
    """The property `quat_yaw_deg` lacks, and the reason a converged `_delta`
    lands the fused quat's heading exactly on the mag heading."""
    q = (0.604421, 0.35965, 0.593567, -0.391159)
    for d in (-170.0, -33.0, 5.0, 91.0, 179.0):
        assert wrap180(yaw_twist_deg(graft_yaw(q, d))
                       - wrap180(yaw_twist_deg(q) + d)) == pytest.approx(0.0, abs=1e-9)


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
