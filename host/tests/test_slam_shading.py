"""Unit tests for Issue #1's fix: `roomscan.slam.shading.shade_colors`, the
pure-numpy helper that bakes a fixed-light shade into mesh vertex colors so
the SLAM/Showcase mesh (uploaded with Open3D's `defaultUnlit` material into a
scene with no lights -- see panel.py's `_build_scene`) isn't invisible black.

Also includes a headless, non-GUI integration check that reproduces the
actual bug end-to-end: build a `TsdfMap`, integrate a few synthetic wall
frames, extract the mesh (confirming its vertex colors are the all-zero
black `tsdf.py` always produces), then run it through the same
`compute_vertex_normals()` + `shade_colors()` sequence the panel's mesh-
upload path (`_render_slam_frame` / `_show_showcase_mesh`) uses, and assert
the result is non-black and varies across vertices.
"""
import numpy as np
import open3d as o3d

from roomscan.slam.shading import shade_colors, wall_triangle_mask
from roomscan.slam.tsdf import TsdfMap
from roomscan.slam.intrinsics import pinhole
from roomscan.slam.frames import world_up

W, H = 54, 42


# ---- shade_colors (pure function) ------------------------------------------

def test_shape_and_dtype():
    normals = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    out = shade_colors(normals)
    assert out.shape == (3, 3)
    assert np.all((out >= 0.0) & (out <= 1.0))


def test_empty_input():
    out = shade_colors(np.zeros((0, 3)))
    assert out.shape == (0, 3)


def test_never_black():
    # A grid of normal directions, including ones near-perpendicular to the
    # fixed light -- even the dimmest should sit at/above the ambient floor,
    # never collapse to [0,0,0] (the bug this fixes).
    rng = np.random.default_rng(0)
    normals = rng.normal(size=(200, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    out = shade_colors(normals)
    assert out.max() > 0.1
    assert out.min() >= 0.0
    # ambient floor: 0.35 * base is the darkest any vertex can get
    assert out.sum(axis=1).min() > 0.0


def test_varies_with_normal_direction():
    # A normal aligned with the light should shade brighter than one
    # perpendicular to it.
    light = np.array([0.3, -0.8, 0.5])
    light = light / np.linalg.norm(light)
    aligned = shade_colors(light[None, :])[0]
    # build a normal perpendicular to `light`
    arbitrary = np.array([1.0, 0.0, 0.0])
    perp = arbitrary - np.dot(arbitrary, light) * light
    perp /= np.linalg.norm(perp)
    perpendicular = shade_colors(perp[None, :])[0]
    assert aligned.sum() > perpendicular.sum()
    assert not np.allclose(aligned, perpendicular)


def test_two_sided_lambert_matches_its_mirror():
    # abs() of the dot product -> a normal and its exact opposite shade
    # identically (two-sided, so back-facing triangles never go black).
    n = np.array([[0.2, 0.5, 0.8]])
    n = n / np.linalg.norm(n)
    out_fwd = shade_colors(n)
    out_back = shade_colors(-n)
    assert np.allclose(out_fwd, out_back)


# ---- headless end-to-end: TsdfMap mesh -> shade_colors ---------------------

def _curved_wall_depth(z_m):
    # Mild curvature (not a flat fronto-parallel plane) so the extracted
    # mesh's vertex normals actually vary across the surface -- a perfectly
    # flat wall would shade every vertex nearly identically and wouldn't
    # exercise the "colors vary" assertion below.
    rows = np.linspace(-0.4, 0.4, H)[:, None]
    cols = np.linspace(-0.5, 0.5, W)[None, :]
    curve = 0.15 * (rows ** 2 + cols ** 2)
    return ((z_m + curve) * 1000.0).astype(np.float32)


def test_slam_mesh_colors_are_nonblack_and_vary_after_shading():
    """Reproduces the reported bug end-to-end, headlessly (no GUI/Filament):
    a real TsdfMap mesh's vertex colors come back all-zero black (confirming
    the root cause), and running it through the panel's actual upload
    sequence (`to_legacy()` -> `compute_vertex_normals()` -> `shade_colors()`
    -> assign `vertex_colors`) produces a legible, non-black, varying result."""
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    for z in (1.30, 1.28, 1.26, 1.24):
        m.integrate(_curved_wall_depth(z), K, np.eye(4))

    mesh = m.mesh()
    assert len(mesh.vertex.positions) > 0

    # Root cause, confirmed: TsdfMap never populates real colors.
    raw_colors = mesh.vertex.colors.numpy()
    assert np.allclose(raw_colors, 0.0)

    # The panel's actual upload path (see _render_slam_frame / _show_showcase_mesh):
    legacy_mesh = mesh.to_legacy()
    legacy_mesh.compute_vertex_normals()
    legacy_mesh.vertex_colors = o3d.utility.Vector3dVector(
        shade_colors(np.asarray(legacy_mesh.vertex_normals)))

    shaded = np.asarray(legacy_mesh.vertex_colors)
    assert shaded.shape[0] == len(mesh.vertex.positions)
    assert shaded.max() > 0.1          # non-black
    assert shaded.std(axis=0).max() > 0.001   # varies across vertices (not one flat color)


# ---- wall_triangle_mask ("see-through walls", Phase 6) ----------------------
#
# World-up is Open3D CV world's [0,-1,0] (Y-down, see frames.world_up()). A
# "wall" is a vertical surface -- its face normal points roughly *sideways*,
# i.e. roughly perpendicular to up, so |normal . up| is small. Floor/ceiling
# normals point roughly *along* up (or -up), so |normal . up| is close to 1.

def test_horizontal_normal_is_a_wall():
    # normal points sideways (+X) -- perpendicular to world-up -> wall.
    normals = np.array([[1.0, 0.0, 0.0]])
    mask = wall_triangle_mask(normals)
    assert mask.tolist() == [True]


def test_vertical_normal_is_not_a_wall():
    # normal points along +Y -- this is close to -world_up ([0,-1,0]), i.e.
    # a floor/ceiling face -> not a wall.
    normals = np.array([[0.0, 1.0, 0.0]])
    mask = wall_triangle_mask(normals)
    assert mask.tolist() == [False]


def test_threshold_boundary_is_strict_less_than():
    # Construct a normal whose |dot(normal, up)| lands exactly on `thresh`
    # (0.5 by default): tilted 60 degrees off horizontal, dot = -0.5.
    up = world_up()
    assert up.tolist() == [0.0, -1.0, 0.0]
    normal = np.array([[np.sqrt(3) / 2, 0.5, 0.0]])
    dot = float(np.abs(normal[0] @ up))
    assert abs(dot - 0.5) < 1e-9
    mask = wall_triangle_mask(normal, thresh=0.5)
    assert mask.tolist() == [False]   # strict "<", not "<=" -> boundary is NOT a wall
    # Nudge just inside the threshold and it flips to a wall.
    normal_in = np.array([[np.sqrt(1 - 0.49 ** 2), 0.49, 0.0]])
    assert wall_triangle_mask(normal_in, thresh=0.5).tolist() == [True]


def test_mix_returns_correct_shape_and_values():
    normals = np.array([
        [1.0, 0.0, 0.0],    # wall
        [0.0, 1.0, 0.0],    # floor/ceiling
        [0.0, -1.0, 0.0],   # floor/ceiling (ceiling side)
        [0.0, 0.0, 1.0],    # wall
    ])
    mask = wall_triangle_mask(normals)
    assert mask.shape == (4,)
    assert mask.dtype == np.bool_
    assert mask.tolist() == [True, False, False, True]


def test_empty_triangles():
    mask = wall_triangle_mask(np.zeros((0, 3)))
    assert mask.shape == (0,)


def test_custom_up_vector():
    # With world-up swapped to [0, 1, 0] (opposite convention), the same
    # normal's wall/floor classification flips accordingly is NOT expected --
    # |dot| is sign-agnostic, so [0,1,0] as `up` behaves identically for
    # this vertical-normal case (still "not a wall").
    normals = np.array([[0.0, 1.0, 0.0]])
    mask = wall_triangle_mask(normals, up=np.array([0.0, 1.0, 0.0]))
    assert mask.tolist() == [False]
