"""Vertex shading for SLAM/Showcase meshes.

The TSDF mesh (`TsdfMap.extract_triangle_mesh()` / `Mapper.mesh()`) never gets
real vertex colors: `TsdfMap.integrate()` only ever calls the depth-only
overload of `VoxelBlockGrid.integrate()` (see tsdf.py's module docstring), so
every vertex color comes back `[0, 0, 0]`. The panel uploads that mesh with
the `defaultUnlit` material (`mesh_material`, panel.py) into a scene that sets
up no lights at all (`_build_scene` has no sun/IBL) -- so even switching to
`defaultLit` would render black too. Net effect (user report): "it just draws
a green line ... I don't see the scene being constructed" -- the mesh is
there, it's just pure black against the dark background.

`shade_colors` bakes a fixed-light two-sided Lambert term directly into the
vertex colors so the (still) unlit shader shows a legible shaded surface
without needing real scene lighting.
"""
from __future__ import annotations

import numpy as np

_LIGHT_DIR = np.array([0.3, -0.8, 0.5])
_LIGHT_DIR = _LIGHT_DIR / np.linalg.norm(_LIGHT_DIR)
_BASE_COLOR = np.array([0.82, 0.80, 0.75])   # warm off-white
_AMBIENT = 0.35
_DIFFUSE = 0.65


def shade_colors(normals: np.ndarray) -> np.ndarray:
    """Per-vertex RGB (N,3) from per-vertex `normals` (N,3): two-sided Lambert
    against a fixed light, baked so an unlit shader still shows a shaded
    surface (the scene has no lights).

    Two-sided (`abs` of the dot product) so back-facing triangles -- normal
    orientation out of `extract_triangle_mesh()` / `compute_vertex_normals()`
    isn't guaranteed to face the camera -- never go black either; an ambient
    floor (`_AMBIENT`) keeps even a grazing normal above pure black.
    """
    normals = np.asarray(normals, dtype=np.float64)
    if normals.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    lam = np.abs(normals @ _LIGHT_DIR)
    shade = _AMBIENT + _DIFFUSE * np.clip(lam, 0.0, 1.0)
    return np.clip(shade[:, None] * _BASE_COLOR, 0.0, 1.0)
