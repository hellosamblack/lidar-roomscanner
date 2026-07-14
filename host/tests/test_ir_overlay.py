"""Pure geometry tests for the first-person IR overlay quad."""
import numpy as np

from roomscan.ir_overlay import camera_locked_quad

_WORLD_UP = np.array([0.0, -1.0, 0.0])


def test_quad_shapes():
    v, uv, t = camera_locked_quad([0, 0, 0], [0, 0, 1], _WORLD_UP, 55.0, 42.0, 1.0)
    assert v.shape == (4, 3)
    assert uv.shape == (4, 2)
    assert t.shape == (2, 3)


def test_quad_is_planar():
    v, _, _ = camera_locked_quad([0, 0, 0], [0, 0, 1], _WORLD_UP, 55.0, 42.0, 1.0)
    n = np.cross(v[1] - v[0], v[2] - v[0]); n /= np.linalg.norm(n)
    assert abs(np.dot(v[3] - v[0], n)) < 1e-9


def test_quad_center_is_dist_ahead():
    dist = 0.8
    v, _, _ = camera_locked_quad([0, 0, 0], [0, 0, 1], _WORLD_UP, 55.0, 42.0, dist)
    center = v.mean(axis=0)
    assert np.allclose(center, [0, 0, dist], atol=1e-9)


def test_quad_faces_the_eye():
    # The quad normal should point back toward the eye (dot with forward < 0
    # or > 0 consistently) -- i.e. the ray from center to eye is ~antiparallel
    # to forward.
    eye = np.array([0.0, 0.0, 0.0]); fwd = np.array([0.0, 0.0, 1.0])
    v, _, _ = camera_locked_quad(eye, fwd, _WORLD_UP, 55.0, 42.0, 1.0)
    center = v.mean(axis=0)
    to_eye = eye - center
    assert np.dot(to_eye, fwd) < 0


def test_quad_size_matches_fov():
    dist, fov_h, fov_v = 1.0, 60.0, 40.0
    v, _, _ = camera_locked_quad([0, 0, 0], [0, 0, 1], _WORLD_UP, fov_h, fov_v, dist)
    width = np.linalg.norm(v[1] - v[0])       # top-left -> top-right
    height = np.linalg.norm(v[2] - v[1])      # top-right -> bottom-right
    np.testing.assert_allclose(width, 2 * dist * np.tan(np.deg2rad(fov_h) / 2), rtol=1e-6)
    np.testing.assert_allclose(height, 2 * dist * np.tan(np.deg2rad(fov_v) / 2), rtol=1e-6)


def test_quad_translated_eye():
    v, _, _ = camera_locked_quad([5, 2, -3], [0, 0, 1], _WORLD_UP, 55.0, 42.0, 1.0)
    assert np.allclose(v.mean(axis=0), [5, 2, -2], atol=1e-9)
