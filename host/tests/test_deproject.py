import numpy as np
import pytest

from roomscan.deproject import Deprojector


def test_center_zone_projects_straight_ahead():
    # 3x3 grid: the middle zone's angular center is exactly 0
    d = Deprojector(width=3, height=3, fov_h_deg=90.0, fov_v_deg=90.0)
    depth = np.full((3, 3), 2000.0, dtype=np.float32)   # 2 m everywhere
    pts = d(depth)
    assert pts.shape == (9, 3)
    center = pts[4]
    assert np.allclose(center, [0.0, 0.0, 2.0], atol=1e-9)


def test_corner_zone_angle():
    d = Deprojector(width=3, height=3, fov_h_deg=90.0, fov_v_deg=90.0)
    depth = np.full((3, 3), 1000.0, dtype=np.float32)
    pts = d(depth)
    # rightmost column zone center: ((2+0.5)/3 - 0.5) * 90° = 30°
    expected_x = 1.0 * np.tan(np.deg2rad(30.0))
    assert np.isclose(pts[5][0], expected_x, atol=1e-9)   # row 1, col 2
    assert np.isclose(pts[5][2], 1.0)


def test_invalid_zones_filtered():
    d = Deprojector(width=2, height=2)
    depth = np.array([[0.0, np.inf], [np.nan, 1500.0]], dtype=np.float32)
    pts = d(depth)
    assert pts.shape == (1, 3)
    assert np.isclose(pts[0][2], 1.5)


def test_out_of_range_filtered():
    d = Deprojector(width=1, height=1, max_range_mm=4000.0)
    assert d(np.array([[5000.0]], dtype=np.float32)).shape == (0, 3)


def test_zone_tan_table_overrides_linear_model():
    # A per-zone table can encode angles the separable linear model cannot
    # (e.g. non-uniform spacing) -- here zone (0, 1)'s tan_x is set far from
    # what any fov_h_deg would produce for a uniform 2-column grid.
    zone_tan_x = np.array([[0.0, 5.0]])   # (h=1, w=2)
    zone_tan_y = np.array([[0.0, 0.0]])
    d = Deprojector(width=2, height=1, zone_tan_x=zone_tan_x, zone_tan_y=zone_tan_y)
    depth = np.array([[1000.0, 1000.0]], dtype=np.float32)
    pts = d(depth)
    assert np.allclose(pts[0], [0.0, 0.0, 1.0], atol=1e-9)
    assert np.allclose(pts[1], [5.0, 0.0, 1.0], atol=1e-9)


def test_zone_tan_table_requires_matching_shape():
    with pytest.raises(ValueError):
        Deprojector(width=2, height=2, zone_tan_x=np.zeros((2, 2)), zone_tan_y=np.zeros((3, 2)))
    with pytest.raises(ValueError):
        Deprojector(width=2, height=2, zone_tan_x=np.zeros((3, 3)), zone_tan_y=np.zeros((3, 3)))


def test_zone_tan_table_requires_both_or_neither():
    with pytest.raises(ValueError):
        Deprojector(width=2, height=2, zone_tan_x=np.zeros((2, 2)))
    with pytest.raises(ValueError):
        Deprojector(width=2, height=2, zone_tan_y=np.zeros((2, 2)))
