import numpy as np
import open3d as o3d

from roomscan.surface import grid_triangles, alpha_shape_mesh


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


def _make_pcd(points, colors=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def test_alpha_shape_too_few_points_returns_empty_uncovered():
    pcd = _make_pcd(np.random.default_rng(0).random((3, 3)))
    mesh, covered = alpha_shape_mesh(pcd, threshold_m=0.1)
    assert len(mesh.triangles) == 0
    assert covered.tolist() == [False, False, False]


def test_alpha_shape_covers_a_flat_patch_but_not_a_far_outlier():
    # A small flat patch (mimics a single depth-camera sweep, which is
    # inherently surface-like) plus one point far away. Alpha shape builds a
    # 2D boundary surface -- points strictly interior to a genuine 3D blob
    # would NOT all be covered, but a planar patch's points all sit on that
    # boundary, so they should all end up covered; the outlier can't join
    # any simplex within the threshold and must stay uncovered.
    rng = np.random.default_rng(0)
    xs, ys = np.meshgrid(np.linspace(-0.1, 0.1, 6), np.linspace(-0.1, 0.1, 6))
    patch = np.stack([xs.ravel(), ys.ravel(), np.full(36, 1.0) + rng.normal(0, 0.001, 36)], axis=1)
    outlier = np.array([[5.0, 5.0, 5.0]])
    pts = np.vstack([patch, outlier])
    colors = np.tile([0.2, 0.4, 0.6], (37, 1))
    pcd = _make_pcd(pts, colors)

    mesh, covered = alpha_shape_mesh(pcd, threshold_m=0.08)
    assert len(mesh.triangles) > 0
    assert mesh.has_vertex_colors()
    assert covered[:36].all()
    assert not covered[36]


def test_alpha_shape_degenerate_coplanar_input_returns_empty_uncovered():
    # >=4 points but exactly coplanar (all z=0) -- Qhull cannot build a 3D
    # tetrahedralization from a degenerate/flat point set and raises
    # RuntimeError. alpha_shape_mesh must catch this and return the same
    # "nothing covered" result as the n<4 case, not propagate the exception
    # (this runs inside a live GUI render loop -- a crash would be user-visible).
    pts = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.5, 0.5, 0.0],
        [0.2, 0.8, 0.0],
    ])
    pcd = _make_pcd(pts)
    mesh, covered = alpha_shape_mesh(pcd, threshold_m=0.5)
    assert len(mesh.triangles) == 0
    assert covered.tolist() == [False] * 6
