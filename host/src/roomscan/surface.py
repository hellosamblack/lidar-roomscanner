"""Mesh builders for the panel's opt-in surface-interpolation mode: cover
adjacent depth-camera points with a triangle mesh instead of drawing them as
dots, when they're close enough. Two adjacency strategies -- see
docs/superpowers/plans/2026-07-09-surface-interpolation-design.md.
"""
from __future__ import annotations

import numpy as np


def grid_triangles(pts_grid: np.ndarray, valid: np.ndarray,
                   threshold_pct: float) -> tuple[np.ndarray, np.ndarray]:
    """Raster-adjacency triangulation: pts_grid is (h, w, 3), valid is (h, w)
    bool (both from Deprojector.grid()). For each 2x2 cell, considers two
    candidate triangles (upper-left: (r,c),(r,c+1),(r+1,c); lower-right:
    (r,c+1),(r+1,c+1),(r+1,c)). A triangle is emitted iff all three corners
    are valid AND every pairwise depth gap among them satisfies
    abs(za - zb) <= threshold_pct/100 * min(za, zb) -- relative to the
    nearer point, so one threshold behaves consistently from 0.5m to 5m, and
    a foreground/background straddle (gap large relative to either depth) is
    refused. Fully vectorized (no Python-level loop over cells).

    Returns (triangles, covered): triangles is (T, 3) int64 flat-index
    (r*w+c) vertex triples; covered is (h*w,) bool, True where a grid point
    participates in >=1 emitted triangle (i.e. should be hidden from the dot
    cloud)."""
    h, w, _ = pts_grid.shape
    z = pts_grid[..., 2]
    idx = np.arange(h * w).reshape(h, w)

    def close(za, zb):
        return np.abs(za - zb) <= (threshold_pct / 100.0) * np.minimum(za, zb)

    z00, z01, z10, z11 = z[:-1, :-1], z[:-1, 1:], z[1:, :-1], z[1:, 1:]
    v00, v01, v10, v11 = valid[:-1, :-1], valid[:-1, 1:], valid[1:, :-1], valid[1:, 1:]
    i00, i01, i10, i11 = idx[:-1, :-1], idx[:-1, 1:], idx[1:, :-1], idx[1:, 1:]

    ul_ok = v00 & v01 & v10 & close(z00, z01) & close(z00, z10) & close(z01, z10)
    lr_ok = v01 & v11 & v10 & close(z01, z11) & close(z01, z10) & close(z11, z10)

    ul_tris = np.stack([i00[ul_ok], i01[ul_ok], i10[ul_ok]], axis=1)
    lr_tris = np.stack([i01[lr_ok], i11[lr_ok], i10[lr_ok]], axis=1)
    triangles = np.concatenate([ul_tris, lr_tris], axis=0).astype(np.int64)

    covered = np.zeros(h * w, dtype=bool)
    if triangles.size:
        covered[triangles.ravel()] = True
    return triangles, covered


def alpha_shape_mesh(pcd, threshold_m: float):
    """3D-proximity ("spatial") adjacency via Open3D's alpha-shape
    reconstruction: alpha *is* the distance threshold. pcd is an
    o3d.geometry.PointCloud the caller has already populated with .points
    (and, for coloring, .colors).

    Needs >=4 non-degenerate points; with fewer (or a degenerate/coplanar
    configuration Qhull rejects) returns (empty mesh, all-False) rather than
    raising -- the caller falls back to drawing every point as a dot that
    frame.

    Returns (mesh, covered): mesh is the raw create_from_point_cloud_alpha_shape
    result (own vertex/vertex_color arrays -- NOT indexed into pcd, and not
    even the same vertex count: alpha shape drops points that don't end up on
    the reconstructed 2D boundary, and reorders + reindexes the rest).
    covered is an (N,) bool over pcd's ORIGINAL point order, recovered by
    nearest-neighbor matching each mesh vertex back to pcd within a 1e-4 m
    tolerance -- empirically, alpha shape's own float32 round-trip only ever
    displaces a vertex by ~1e-7 m at scanner scale, so this tolerance is
    generous against that noise while staying far below any real point
    spacing (no risk of matching the wrong point)."""
    import open3d as o3d

    n = len(pcd.points)
    empty = o3d.geometry.TriangleMesh()
    covered = np.zeros(n, dtype=bool)
    if n < 4:
        return empty, covered
    try:
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha=threshold_m)
    except RuntimeError:
        return empty, covered
    mesh_verts = np.asarray(mesh.vertices)
    if len(mesh_verts) == 0:
        return mesh, covered
    tree = o3d.geometry.KDTreeFlann(pcd)
    tol2 = (1e-4) ** 2
    for v in mesh_verts:
        _, idx, dist2 = tree.search_knn_vector_3d(v, 1)
        if dist2[0] <= tol2:
            covered[idx[0]] = True
    return mesh, covered
