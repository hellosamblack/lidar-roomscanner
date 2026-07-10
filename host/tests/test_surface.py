import numpy as np

from roomscan.surface import grid_triangles, grid_triangles_3d


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


def test_grid_triangles_3d_fully_triangulated():
    pts = _flat_grid(np.ones((3, 3)))
    valid = np.ones((3, 3), dtype=bool)
    triangles, covered = grid_triangles_3d(pts, valid, threshold_m=0.1)
    assert triangles.shape == (8, 3)   # 2x2 quads, 2 triangles each
    assert covered.all()


def test_grid_triangles_3d_step_edge():
    # Grid where one column has a large X/Y/Z physical 3D distance jump.
    # We place points on a 2x3 grid:
    # Row 0: [0, 0, 1.0], [0.05, 0, 1.0], [1.0, 0, 1.0]
    # Row 1: [0, 0.05, 1.0], [0.05, 0.05, 1.0], [1.0, 0.05, 1.0]
    # The jump between column 1 and column 2 is ~0.95 meters, which exceeds the threshold_m of 0.1.
    # Therefore, no triangles should cross from col 1 to col 2, and col 2 points stay uncovered.
    pts = np.array([
        [[0.0, 0.0, 1.0], [0.05, 0.0, 1.0], [1.0, 0.0, 1.0]],
        [[0.0, 0.05, 1.0], [0.05, 0.05, 1.0], [1.0, 0.05, 1.0]],
    ])
    valid = np.ones((2, 3), dtype=bool)
    triangles, covered = grid_triangles_3d(pts, valid, threshold_m=0.1)
    assert triangles.shape == (2, 3)  # only the cols 0-1 quad (2 triangles)
    assert covered.tolist() == [True, True, False, True, True, False]


def test_grid_triangles_3d_invalid_row():
    # 4x2 grid, row 1 invalid.
    # Using pts where adjacent points are 0.05m apart.
    pts = np.zeros((4, 2, 3))
    for r in range(4):
        for c in range(2):
            pts[r, c] = [c * 0.05, r * 0.05, 1.0]
    valid = np.ones((4, 2), dtype=bool)
    valid[1, :] = False
    triangles, covered = grid_triangles_3d(pts, valid, threshold_m=0.1)
    assert triangles.shape == (2, 3)  # only the rows 2-3 quad
    assert covered.tolist() == [False, False, False, False, True, True, True, True]

