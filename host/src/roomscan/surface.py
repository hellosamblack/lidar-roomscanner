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


def grid_triangles_3d(pts_grid: np.ndarray, valid: np.ndarray,
                      threshold_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Raster-adjacency triangulation with a 3D Euclidean distance threshold:
    pts_grid is (h, w, 3), valid is (h, w) bool. For each 2x2 cell, considers
    two candidate triangles (upper-left: (r,c),(r,c+1),(r+1,c); lower-right:
    (r,c+1),(r+1,c+1),(r+1,c)). A triangle is emitted iff all three corners
    are valid AND every pairwise 3D Euclidean distance among them satisfies
    dist(pa, pb) <= threshold_m.

    Returns (triangles, covered) in the same format as grid_triangles."""
    h, w, _ = pts_grid.shape
    idx = np.arange(h * w).reshape(h, w)

    def close_3d(pa, pb):
        return np.sum((pa - pb) ** 2, axis=-1) <= threshold_m ** 2

    p00, p01, p10, p11 = pts_grid[:-1, :-1], pts_grid[:-1, 1:], pts_grid[1:, :-1], pts_grid[1:, 1:]
    v00, v01, v10, v11 = valid[:-1, :-1], valid[:-1, 1:], valid[1:, :-1], valid[1:, 1:]
    i00, i01, i10, i11 = idx[:-1, :-1], idx[:-1, 1:], idx[1:, :-1], idx[1:, 1:]

    ul_ok = v00 & v01 & v10 & close_3d(p00, p01) & close_3d(p00, p10) & close_3d(p01, p10)
    lr_ok = v01 & v11 & v10 & close_3d(p01, p11) & close_3d(p01, p10) & close_3d(p11, p10)

    ul_tris = np.stack([i00[ul_ok], i01[ul_ok], i10[ul_ok]], axis=1)
    lr_tris = np.stack([i01[lr_ok], i11[lr_ok], i10[lr_ok]], axis=1)
    triangles = np.concatenate([ul_tris, lr_tris], axis=0).astype(np.int64)

    covered = np.zeros(h * w, dtype=bool)
    if triangles.size:
        covered[triangles.ravel()] = True
    return triangles, covered

