"""Pure-function tests for the 'depth-scope' stage helpers (theme.py)."""
import numpy as np

from roomscan import theme


def test_vertical_gradient_shape_and_endpoints():
    img = theme.vertical_gradient(8, 20, (0.10, 0.13, 0.20), (0.02, 0.03, 0.05))
    assert img.shape == (20, 8, 3)
    assert img.dtype == np.uint8
    # top row == top color, bottom row == bottom color (uint8-rounded)
    np.testing.assert_array_equal(img[0, 0], np.array([26, 33, 51]))
    np.testing.assert_array_equal(img[-1, 0], np.array([5, 8, 13]))
    # every column identical (purely vertical)
    for c in range(8):
        np.testing.assert_array_equal(img[:, c], img[:, 0])


def test_vertical_gradient_monotonic_when_top_brighter():
    img = theme.vertical_gradient(4, 32, (0.9, 0.9, 0.9), (0.1, 0.1, 0.1))
    col = img[:, 0, 0].astype(int)
    assert np.all(np.diff(col) <= 0)          # darkens downward
    assert col[0] > col[-1]


def test_vertical_gradient_degenerate_size():
    img = theme.vertical_gradient(0, 0, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    assert img.shape == (1, 1, 3)


def test_floor_grid_sits_at_floor_ydown():
    # y-down world (up=[0,-1,0]): floor is the MAX y of the box.
    lo, hi = [-1.0, -2.0, -1.0], [1.0, 0.5, 1.0]
    pts, lines = theme.floor_grid_lines(lo, hi, spacing=0.5, pad=0.0)
    assert len(pts) > 0 and lines.shape[1] == 2
    assert np.allclose(pts[:, 1], 0.5)        # every grid vertex on the floor plane
    # grid spans the horizontal extent (x,z), not beyond +/- pad
    assert pts[:, 0].min() >= -1.0 - 1e-9 and pts[:, 0].max() <= 1.0 + 1e-9
    assert pts[:, 2].min() >= -1.0 - 1e-9 and pts[:, 2].max() <= 1.0 + 1e-9


def test_floor_grid_respects_pad_and_indices_valid():
    pts, lines = theme.floor_grid_lines([0, 0, 0], [2, 1, 2], spacing=1.0, pad=0.5)
    assert pts[:, 0].min() == -0.5 and pts[:, 0].max() == 2.5
    assert lines.max() < len(pts)             # all line endpoints reference real points
    assert lines.min() >= 0


def test_floor_grid_degenerate_box_is_empty():
    pts, lines = theme.floor_grid_lines([0, 0, 0], [0, 0, 0], spacing=0.5)
    assert len(pts) == 0 and len(lines) == 0


def test_floor_grid_y_up_convention():
    # up=[0,1,0]: floor is the MIN y.
    pts, _ = theme.floor_grid_lines([-1, -2, -1], [1, 0.5, 1], up=[0, 1, 0], spacing=0.5, pad=0.0)
    assert np.allclose(pts[:, 1], -2.0)


def test_trajectory_ramp_fades_old_to_new():
    ramp = theme.trajectory_ramp(10)
    assert ramp.shape == (10, 3)
    # last segment is the brightest (most recent), first the dimmest
    assert np.sum(ramp[-1]) > np.sum(ramp[0])
    # monotonic brightening toward the head
    assert np.all(np.diff(ramp.sum(axis=1)) >= -1e-9)


def test_trajectory_ramp_empty():
    assert theme.trajectory_ramp(0).shape == (0, 3)
    assert theme.trajectory_ramp(-3).shape == (0, 3)
