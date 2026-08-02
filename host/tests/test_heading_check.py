"""`tools.heading_check.score` -- the instrument that would have caught BUG-058.

Driven with synthetic series whose answer is known, including a deliberately
DEFECTIVE heading. A checker only validated against working data is untested in
the direction that matters: it has never been shown it can fail.
"""
import numpy as np
import pytest

from tools.heading_check import COEF_TOL, score

RATE = 30.0


def _series(n=900, roll_amp=80.0, bearing_amp=0.0, noise=0.0, seed=1):
    """A synthetic take: the operator swings the bearing and rolls their wrist,
    at different periods so the two are not collinear."""
    rng = np.random.default_rng(seed)
    t = np.arange(n) / RATE
    roll = roll_amp * np.sin(2 * np.pi * t / 5.0)
    bearing = bearing_amp * np.sin(2 * np.pi * t / 13.0)
    drift = noise * np.cumsum(rng.normal(size=n)) / np.sqrt(n)   # slow wander, not white
    return bearing, roll, drift


def test_a_correct_heading_scores_good_on_both_axes():
    bearing, roll, drift = _series(bearing_amp=140.0, noise=2.0)
    heading = bearing + 37.0 + drift              # tracks bearing, ignores roll
    r = score(bearing, heading, roll, rate_hz=RATE)
    assert r["verdict"] == "good"
    assert r["bearing"]["coef"] == pytest.approx(1.0, abs=COEF_TOL)
    assert r["roll"]["coef"] == pytest.approx(0.0, abs=COEF_TOL)


def test_the_bug_058_defect_is_reported_bad_on_the_roll_axis():
    """Heading = constant - roll, which is what `yaw_twist_deg` reduced to with
    the boresight horizontal. Measured -0.984 on captures/NorthFacingRoll.bin."""
    bearing, roll, _ = _series(bearing_amp=140.0)
    heading = 37.0 - roll
    r = score(bearing, heading, roll, rate_hz=RATE)
    assert r["verdict"] == "bad"
    assert r["roll"]["verdict"] == "bad"
    assert r["roll"]["coef"] == pytest.approx(-1.0, abs=0.05)
    assert "ROLL" in r["reason"]


def test_a_heading_that_ignores_the_bearing_is_reported_bad():
    """The other half: a heading that does not move with where the sensor points
    (BUG-051's shape) must fail the bearing axis, not slip through on roll."""
    bearing, roll, _ = _series(bearing_amp=140.0)
    heading = 0.4 * bearing + 37.0
    r = score(bearing, heading, roll, rate_hz=RATE)
    assert r["verdict"] == "bad"
    assert r["bearing"]["verdict"] == "bad"


def test_an_axis_the_capture_never_exercised_is_inconclusive_not_good():
    """A capture that never rolled cannot exonerate the roll axis. The failure
    mode this guards is a green report from a take with no information in it."""
    bearing, roll, _ = _series(roll_amp=2.0, bearing_amp=140.0)
    heading = bearing + 37.0
    r = score(bearing, heading, roll, rate_hz=RATE)
    assert r["roll"]["verdict"] == "inconclusive"
    assert r["verdict"] == "inconclusive"
    assert "does not exercise that axis" in r["roll"]["reason"]


def test_a_drifty_capture_with_little_roll_is_inconclusive_not_bad():
    """The false positive that block-bootstrapping fixes: on a real circuit
    (coffeeRoomCircuitNoMnt) the fitted roll coefficient came out 0.181 with only
    36 deg of roll and 5.8 deg of drift. Treating autocorrelated wander as white
    noise made that read BAD; the interval must be wide enough to say it cannot
    tell."""
    bearing, roll, drift = _series(roll_amp=18.0, bearing_amp=200.0, noise=14.0, seed=7)
    heading = bearing + 37.0 + drift
    r = score(bearing, heading, roll, rate_hz=RATE)
    assert r["roll"]["verdict"] == "inconclusive"
    lo, hi = r["roll"]["ci95"]
    assert hi - lo > 2 * COEF_TOL          # the interval, not the point estimate, decides


def test_too_few_samples_is_inconclusive():
    bearing, roll, _ = _series(n=20, bearing_amp=140.0)
    r = score(bearing, bearing + 37.0, roll, rate_hz=RATE)
    assert r["verdict"] == "inconclusive"
    assert r["bearing"] is None


def test_score_is_deterministic():
    """The bootstrap is seeded, so two runs of one capture are comparable."""
    bearing, roll, drift = _series(bearing_amp=140.0, noise=3.0)
    heading = bearing + 37.0 + drift
    a = score(bearing, heading, roll, rate_hz=RATE)
    b = score(bearing, heading, roll, rate_hz=RATE)
    assert a["roll"]["ci95"] == b["roll"]["ci95"]


def test_inclination_is_positive_for_a_northern_hemisphere_field():
    """Earth's field points north and DOWN here, ~70 deg below horizontal."""
    from tools.heading_check import inclination_deg
    dip = np.radians(70.0)
    field = np.tile([50 * np.cos(dip), 0.0, -50 * np.sin(dip)], (100, 1))
    assert inclination_deg(field) == pytest.approx(70.0, abs=0.1)


def test_inclination_is_negative_for_an_anti_parallel_field():
    """BUG-059's signature: right magnitude, exactly reversed. Measured -70.0 on
    captures/NorthFacingRoll.bin before the fix and +70.0 after."""
    from tools.heading_check import inclination_deg
    dip = np.radians(70.0)
    field = np.tile([50 * np.cos(dip), 0.0, -50 * np.sin(dip)], (100, 1))
    assert inclination_deg(-field) == pytest.approx(-70.0, abs=0.1)


def test_a_constant_offset_is_invisible_to_the_regression():
    """Why the dip check exists at all: shift every heading by 180 deg and the
    coefficients do not move. Pinning this stops someone 'simplifying' the
    inclination field away as redundant."""
    bearing, roll, _ = _series(bearing_amp=140.0)
    good = score(bearing, bearing + 37.0, roll, rate_hz=RATE)
    flipped = score(bearing, bearing + 37.0 + 180.0, roll, rate_hz=RATE)
    assert flipped["bearing"]["coef"] == pytest.approx(good["bearing"]["coef"])
    assert flipped["roll"]["coef"] == pytest.approx(good["roll"]["coef"])
    assert flipped["verdict"] == good["verdict"] == "good"
