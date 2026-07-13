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

# Height-cued base albedo ("the stage", Phase 6 UX): grade the surface from a
# cool tone at the FLOOR to a warm off-white at the UPPER surfaces so depth
# reads at a glance even before any real scene lighting. Applied only where the
# TSDF mesh has no meaningful integrated color (the depth-only all-[0,0,0] case
# -- see `mesh_colors_are_meaningful`); a reflectance-textured mesh keeps its
# own color. Endpoints match the demonstrated 'stage' look.
_FLOOR_TINT = np.array([0.34, 0.52, 0.60])   # cool blue -- lowest surfaces
_UPPER_TINT = np.array([0.86, 0.84, 0.80])   # warm off-white -- highest surfaces


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


def shade_colors(normals: np.ndarray, base: np.ndarray | None = None) -> np.ndarray:
    """Per-vertex RGB (N,3) from per-vertex `normals` (N,3): two-sided Lambert
    against a fixed light, baked so an unlit shader still shows a shaded
    surface (the scene has no lights).

    `base` is the albedo the lighting term modulates: a single RGB triple
    (default `_BASE_COLOR`, the warm off-white -- byte-identical to the
    pre-stage behavior) or a per-vertex (N,3) array (e.g. `height_base_colors`,
    the height-cued 'stage' albedo). Two-sided (`abs` of the dot product) so
    back-facing triangles -- normal orientation out of
    `extract_triangle_mesh()` / `compute_vertex_normals()` isn't guaranteed to
    face the camera -- never go black either; an ambient floor (`_AMBIENT`)
    keeps even a grazing normal above pure black.
    """
    shade = shade_brightness(normals)
    if shade.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if base is None:
        base = _BASE_COLOR
    base = np.asarray(base, dtype=np.float64)
    return np.clip(shade[:, None] * base, 0.0, 1.0)


_HUE_FLOOR = np.array([0.72, 0.90, 1.12])   # cool multiplier (bluish) at the floor
_HUE_UPPER = np.array([1.12, 1.02, 0.82])   # warm multiplier (amber) up high


def height_tint_hue(vertices: np.ndarray, up: np.ndarray | None = None) -> np.ndarray:
    """Per-vertex RGB *multiplier* (N,3) graded by height along `up`: a cool
    (bluish) multiplier at the floor -> warm (amber) up high, each centered
    near luma 1 so multiplying an existing color (e.g. a reflectance grey)
    tints it without darkening. This is what makes the 'stage' height cue show
    on a LIVE reflectance-integrated mesh, whose vertices already carry a grey
    reflectance color (so `height_base_colors`, an absolute albedo, is never
    reached -- see `_upload_slam_mesh`). Degenerate/empty -> all-ones (no-op).
    Pure -- unit-tested."""
    vertices = np.asarray(vertices, dtype=np.float64)
    if vertices.shape[0] == 0:
        return np.ones((0, 3), dtype=np.float64)
    if up is None:
        from .frames import world_up
        up = world_up()
    up = np.asarray(up, dtype=np.float64)
    height = vertices @ up
    lo, hi = float(height.min()), float(height.max())
    if hi - lo < 1e-9:
        return np.ones((vertices.shape[0], 3), dtype=np.float64)
    t = ((height - lo) / (hi - lo))[:, None]        # 0 at floor, 1 at top
    return _HUE_FLOOR[None, :] * (1.0 - t) + _HUE_UPPER[None, :] * t


def height_base_colors(vertices: np.ndarray, up: np.ndarray | None = None) -> np.ndarray:
    """Per-vertex base albedo (N,3) graded by height along `up`: `_FLOOR_TINT`
    (cool) at the lowest surfaces up to `_UPPER_TINT` (warm off-white) at the
    highest. `up` defaults to Open3D CV world-up `[0,-1,0]` (y-down), so a
    smaller y (physically higher) grades toward the warm tint. Degenerate
    (all-equal height, or empty) falls back to the flat `_BASE_COLOR` so the
    result is never NaN. Pure -- unit-tested; feeds `shade_colors(base=...)`
    for the depth-only 'stage' mesh (see `_upload_slam_mesh`)."""
    vertices = np.asarray(vertices, dtype=np.float64)
    if vertices.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if up is None:
        from .frames import world_up
        up = world_up()
    up = np.asarray(up, dtype=np.float64)
    height = vertices @ up             # physical-up coordinate (up=[0,-1,0] -> -y, larger=higher)
    lo, hi = float(height.min()), float(height.max())
    if hi - lo < 1e-9:
        return np.tile(_BASE_COLOR, (vertices.shape[0], 1))
    t = ((height - lo) / (hi - lo))[:, None]        # 0 at floor, 1 at top
    return _FLOOR_TINT[None, :] * (1.0 - t) + _UPPER_TINT[None, :] * t


# Materialization wavefront ("the signature", Phase 6 UX): surfaces near where
# the sensor is currently looking glow the signal cyan and fade into the base
# material with distance -- so you watch the room get "painted" as you sweep.
# Proximity-to-sensor stands in for integration-recency: on a handheld sweep the
# surface nearest the sensor IS the one being freshly integrated. `_ACCENT_GLOW`
# is theme.ACCENT (kept in sync by hand, as elsewhere) so the glow, the capture
# beam, and the trajectory head are the same cyan.
_ACCENT_GLOW = np.array([0.18, 0.88, 0.82])
_WAVEFRONT_RADIUS = 1.2     # metres: glow reach around the sensor position
_WAVEFRONT_STRENGTH = 0.85  # max blend toward accent at the sensor


def wavefront_glow(vertices: np.ndarray, origin, colors: np.ndarray,
                   radius: float = _WAVEFRONT_RADIUS,
                   strength: float = _WAVEFRONT_STRENGTH,
                   accent: np.ndarray | None = None) -> np.ndarray:
    """Blend per-vertex `colors` (N,3) toward `accent` by proximity of each
    vertex to `origin` (the sensor's current world position): the
    materialization wavefront. `g = strength * smoothstep(1 - d/radius)` (0
    beyond `radius`, `strength` at the sensor); returns `lerp(colors, accent,
    g)`, clipped to [0,1]. `colors` passed through unchanged for empty input.
    Pure -- unit-tested; applied only in the LIVE scanning views (SLAM /
    Showcase RECORDING), never on the finished PROCESSING/FINAL mesh."""
    vertices = np.asarray(vertices, dtype=np.float64)
    colors = np.asarray(colors, dtype=np.float64)
    if vertices.shape[0] == 0:
        return colors
    if accent is None:
        accent = _ACCENT_GLOW
    accent = np.asarray(accent, dtype=np.float64)
    d = np.linalg.norm(vertices - np.asarray(origin, dtype=np.float64), axis=1)
    t = np.clip(1.0 - d / max(radius, 1e-6), 0.0, 1.0)
    s = t * t * (3.0 - 2.0 * t)                       # smoothstep
    g = (strength * s)[:, None]
    return np.clip(colors * (1.0 - g) + accent[None, :] * g, 0.0, 1.0)


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
