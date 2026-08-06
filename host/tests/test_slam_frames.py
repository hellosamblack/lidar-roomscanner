import numpy as np
import pytest
from roomscan.slam.frames import (prior_rotation, predict_pose, world_up, baro_height_m,
                                  slerp, apply_quat_phase)
from roomscan.sensors import quat_to_matrix, quat_yaw_deg, T_CV_TO_BODY, T_WORLD_TO_CV

def test_prior_rotation_is_the_documented_sandwich():
    q = (0.9239, 0.0, 0.3827, 0.0)  # ~45deg about y
    R = prior_rotation(q)
    expected = T_WORLD_TO_CV @ quat_to_matrix(*q) @ T_CV_TO_BODY
    assert np.allclose(R, expected)
    # proper rotation
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-9)

def test_predict_pose_places_translation():
    q = (1.0, 0.0, 0.0, 0.0)
    t = np.array([0.1, -0.2, 0.3])
    T = predict_pose(q, t)
    assert T.shape == (4, 4)
    assert np.allclose(T[:3, 3], t)
    assert np.allclose(T[:3, :3], prior_rotation(q))
    assert np.allclose(T[3], [0, 0, 0, 1])

def test_world_up_is_open3d_minus_y():
    assert np.allclose(world_up(), [0.0, -1.0, 0.0])


def test_slerp_endpoints_and_midpoint():
    a = (1.0, 0.0, 0.0, 0.0)
    b = quat_to_q(np.radians(60.0))                 # 60deg about z
    assert np.allclose(slerp(a, b, 0.0), a)
    assert np.allclose(slerp(a, b, 1.0), b)
    mid = slerp(a, b, 0.5)                           # halfway = 30deg about z
    assert np.allclose(mid, quat_to_q(np.radians(30.0)), atol=1e-6)


def test_slerp_takes_the_short_arc_across_the_sign_flip():
    a = (1.0, 0.0, 0.0, 0.0)
    b = (-0.9999, 0.0, 0.0, np.sqrt(1 - 0.9999 ** 2))   # same orientation, negated hemisphere
    out = slerp(a, b, 0.5)
    assert out[0] > 0.99                              # stayed near identity, did not swing 180


def test_slerp_returns_unit_quaternion():
    out = slerp((1.0, 0, 0, 0), quat_to_q(np.radians(120.0)), 0.37)
    assert abs(np.linalg.norm(out) - 1.0) < 1e-9


def quat_to_q(theta):
    return (float(np.cos(theta / 2)), 0.0, 0.0, float(np.sin(theta / 2)))


# ---- apply_quat_phase: roll the SFLP prior back to the frame instant (BUG-031/067)

def test_apply_quat_phase_is_a_noop_without_offset_or_gyro():
    q = quat_to_q(np.radians(20.0))
    assert apply_quat_phase(q, [0.0, 0.0, 100.0], 0.0) == q       # offset 0
    assert apply_quat_phase(q, [0.0, 0.0, 100.0], None) == q      # no offset
    assert apply_quat_phase(q, None, 7760.0) == q                 # no gyro
    assert apply_quat_phase(q, [0.0, 0.0, 0.0], 7760.0) == q      # zero rate


def test_apply_quat_phase_rolls_the_orientation_backward():
    """A +100 deg/s yaw rate with a +18 ms lead means the quat (batch midpoint)
    sits 1.8 deg AHEAD of the frame; the corrected orientation is rolled BACK by
    ~1.8 deg about +z, i.e. its yaw decreases by ~1.8 deg."""
    q0 = (1.0, 0.0, 0.0, 0.0)
    corrected = apply_quat_phase(q0, [0.0, 0.0, 100.0], 18000.0)
    assert quat_yaw_deg(corrected) == pytest.approx(-1.8, abs=0.05)


def test_apply_quat_phase_scales_with_the_lead():
    q0 = (1.0, 0.0, 0.0, 0.0)
    small = quat_yaw_deg(apply_quat_phase(q0, [0.0, 0.0, 100.0], 9000.0))
    big = quat_yaw_deg(apply_quat_phase(q0, [0.0, 0.0, 100.0], 18000.0))
    assert big == pytest.approx(2.0 * small, rel=0.02)

def test_baro_height_sign_and_zero():
    assert baro_height_m(101325.0, 101325.0) == 0.0
    # lower pressure => higher altitude => positive height
    assert baro_height_m(101225.0, 101325.0) > 0.0
    # ~ -8.3 m per +100 Pa near sea level; check order of magnitude
    h = baro_height_m(101225.0, 101325.0)
    assert 6.0 < h < 10.0
