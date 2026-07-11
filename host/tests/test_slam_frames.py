import numpy as np
from roomscan.slam.frames import prior_rotation, predict_pose, world_up, baro_height_m
from roomscan.sensors import quat_to_matrix, T_CV_TO_BODY, T_WORLD_TO_CV

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

def test_baro_height_sign_and_zero():
    assert baro_height_m(101325.0, 101325.0) == 0.0
    # lower pressure => higher altitude => positive height
    assert baro_height_m(101225.0, 101325.0) > 0.0
    # ~ -8.3 m per +100 Pa near sea level; check order of magnitude
    h = baro_height_m(101225.0, 101325.0)
    assert 6.0 < h < 10.0
