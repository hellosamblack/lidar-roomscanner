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


def shade_brightness(normals: np.ndarray) -> np.ndarray:
    """Scalar per-vertex Lambert brightness (N,) in [_AMBIENT, _AMBIENT +
    _DIFFUSE] from per-vertex `normals` (N,3) -- the same fixed-light,
    two-sided lighting term `shade_colors` bakes into its fixed warm base
    color, exposed separately (Task 14) so a caller with its OWN base color
    (e.g. a reflectance-tinted mesh vertex) can modulate by just the lighting
    term: `reflectance_rgb * shade_brightness(normals)`. See `shade_colors`'s
    docstring for the two-sided/ambient-floor rationale.
    """
    normals = np.asarray(normals, dtype=np.float64)
    if normals.shape[0] == 0:
        return np.zeros((0,), dtype=np.float64)
    lam = np.abs(normals @ _LIGHT_DIR)
    return _AMBIENT + _DIFFUSE * np.clip(lam, 0.0, 1.0)


def shade_colors(normals: np.ndarray) -> np.ndarray:
    """Per-vertex RGB (N,3) from per-vertex `normals` (N,3): two-sided Lambert
    against a fixed light, baked so an unlit shader still shows a shaded
    surface (the scene has no lights).

    Two-sided (`abs` of the dot product) so back-facing triangles -- normal
    orientation out of `extract_triangle_mesh()` / `compute_vertex_normals()`
    isn't guaranteed to face the camera -- never go black either; an ambient
    floor (`_AMBIENT`) keeps even a grazing normal above pure black.
    """
    shade = shade_brightness(normals)
    if shade.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return np.clip(shade[:, None] * _BASE_COLOR, 0.0, 1.0)


def mesh_colors_are_meaningful(vertex_colors: np.ndarray) -> bool:
    """True when `vertex_colors` (an (N,3) array straight off `to_legacy()`,
    BEFORE any shading is baked in) carries real per-vertex information --
    i.e. the mesh was TSDF-integrated with a `color` image (Task 13's
    reflectance path, `TsdfMap.integrate(..., color=...)`) -- rather than the
    uniform all-[0,0,0] black that the depth-only integrate overload always
    produces (see this module's docstring). Empty input (no color attribute
    at all, e.g. a hand-built mesh in a test) also counts as "not
    meaningful". Pure -- unit-tested; used by panel.py's `_upload_slam_mesh`
    to decide whether to modulate the mesh's own reflectance color by
    `shade_brightness` or fall back to `shade_colors`'s fixed base color.
    """
    vertex_colors = np.asarray(vertex_colors)
    if vertex_colors.size == 0:
        return False
    return bool(np.max(vertex_colors) > 1e-6)


def wall_triangle_mask(tri_normals: np.ndarray, up: np.ndarray | None = None,
                        thresh: float = 0.5) -> np.ndarray:
    """True where a triangle is a 'wall' (vertical surface): its face normal is
    roughly perpendicular to world-up, i.e. |normal . up| < thresh. Floor/ceiling
    (|normal.up| ~ 1) are False. `up` defaults to Open3D CV world-up
    (`roomscan.slam.frames.world_up()`, `[0,-1,0]`).

    Orientation-based, not camera-facing-based: a wall is a wall from any
    orbit angle, so the "see-through walls" render modes (panel.py) never
    have to reclassify per frame.
    """
    tri_normals = np.asarray(tri_normals, dtype=np.float64)
    if tri_normals.shape[0] == 0:
        return np.zeros((0,), dtype=bool)
    if up is None:
        from .frames import world_up
        up = world_up()
    up = np.asarray(up, dtype=np.float64)
    up = up / np.linalg.norm(up)
    norms = np.linalg.norm(tri_normals, axis=1, keepdims=True)
    norms[norms == 0.0] = 1.0
    unit = tri_normals / norms
    dot = np.abs(unit @ up)
    return dot < thresh
