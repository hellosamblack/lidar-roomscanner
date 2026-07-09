import numpy as np

from roomscan.surface import grid_triangles


def _flat_grid(z_grid):
    h, w = z_grid.shape
    pts = np.zeros((h, w, 3))
    pts[..., 2] = z_grid
    return pts


def test_flat_surface_fully_triangulated():
    pts = _flat_grid(np.ones((3, 3)))
    valid = np.ones((3, 3), dtype=bool)
    triangles, covered = grid_triangles(pts, valid, threshold_pct=5.0)
    assert triangles.shape == (8, 3)   # 2x2 quads, 2 triangles each
    assert covered.all()


def test_step_edge_refuses_bridging_triangle():
    # (2,3) grid: cols 0-1 near (z=1.0), col 2 far (z=2.0, a 100% jump).
    # The quad spanning cols 1-2 straddles the step and must be refused;
    # col 2's points have no other neighbor quad (they're the last column),
    # so they end up uncovered.
    z = np.array([[1.0, 1.0, 2.0], [1.0, 1.0, 2.0]])
    pts = _flat_grid(z)
    valid = np.ones((2, 3), dtype=bool)
    triangles, covered = grid_triangles(pts, valid, threshold_pct=5.0)
    assert triangles.shape == (2, 3)                  # only the cols 0-1 quad
    assert covered.tolist() == [True, True, False, True, True, False]


def test_invalid_row_blocks_triangles_that_touch_it():
    # (4,2) grid, row 1 invalid: row 0 only quads with row 1 (blocked, stays
    # uncovered); rows 2-3 are both valid and quad normally.
    pts = _flat_grid(np.ones((4, 2)))
    valid = np.ones((4, 2), dtype=bool)
    valid[1, :] = False
    triangles, covered = grid_triangles(pts, valid, threshold_pct=5.0)
    assert triangles.shape == (2, 3)                  # only the rows 2-3 quad
    assert covered.tolist() == [False, False, False, False, True, True, True, True]
