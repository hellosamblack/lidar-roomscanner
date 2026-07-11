"""Headless tests for the "see-through walls" feature (Phase 6, owner request):
`ControlPanel._upload_slam_mesh` / `_submesh` / `_on_wall_mode`.

Follows test_panel_showcase.py's pattern -- call the real, unmodified
`ControlPanel` methods directly (unbound, on a lightweight stand-in object)
rather than instantiating a real `ControlPanel`, which needs a live Open3D/
Filament window (fails headless on this box). The stand-in's `scene_widget.
scene` is a small fake recording add_geometry/remove_geometry calls instead
of a real `Open3DScene`, so no GUI/Filament dependency is needed; the mesh
data fed through it is a real `open3d.t.geometry.TriangleMesh`, so the actual
`to_legacy()` / `compute_vertex_normals()` / `compute_triangle_normals()` /
`wall_triangle_mask` / submesh-building code all really runs.
"""
import numpy as np
import open3d as o3d
import pytest

import roomscan.panel as panel_mod
from roomscan.logbus import LogBus


class _FakeScene:
    def __init__(self):
        self.geoms = {}   # name -> (geometry, material)

    def has_geometry(self, name):
        return name in self.geoms

    def add_geometry(self, name, geom, material):
        self.geoms[name] = (geom, material)

    def remove_geometry(self, name):
        del self.geoms[name]


class _FakeSceneWidget:
    def __init__(self):
        self.scene = _FakeScene()


class _FakeWallPanel:
    """Only the attributes `_upload_slam_mesh` (and the `_submesh` staticmethod
    it calls) actually touch."""

    def __init__(self, wall_mode="solid"):
        self._o3d = o3d
        self.scene_widget = _FakeSceneWidget()
        self.wall_mode = wall_mode
        self.mesh_material = "MESH_MATERIAL"
        self.wall_translucent_material = "WALL_TRANSLUCENT_MATERIAL"
        self.wall_wire_material = "WALL_WIRE_MATERIAL"
        self.bus = LogBus()
        self._slam_last_mesh_obj = "sentinel"
        self._showcase_last_mesh_obj = "sentinel"
        # `_remove_slam_geometries` now also tears down the FoV indicator
        # (Task 14, Issue #2) -- give the stand-in what that real method
        # touches so it runs unmodified through this fake scene.
        self._fov_last_pose = None

    def _remove_fov_geometry(self):
        panel_mod.ControlPanel._remove_fov_geometry(self)


def _corner_tensor_mesh():
    """A minimal tensor TriangleMesh with one unambiguous wall triangle
    (spans X/Y, face-normal ~world-Z -- perpendicular to world-up [0,-1,0])
    and one unambiguous floor triangle (spans X/Z, face-normal ~world-Y --
    parallel to world-up)."""
    verts = np.array([
        [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0],   # wall triangle
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0],   # floor triangle
    ], dtype=np.float32)
    tris = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int32)
    m = o3d.t.geometry.TriangleMesh()
    m.vertex.positions = o3d.core.Tensor(verts)
    m.triangle.indices = o3d.core.Tensor(tris)
    return m


_MESH_GEOM = panel_mod._MESH_GEOM
_MESH_WALLS_GEOM = panel_mod._MESH_WALLS_GEOM


def test_solid_mode_uploads_one_opaque_mesh_no_walls_geom():
    fake = _FakeWallPanel(wall_mode="solid")
    mesh = _corner_tensor_mesh()
    panel_mod.ControlPanel._upload_slam_mesh(fake, mesh)

    geoms = fake.scene_widget.scene.geoms
    assert set(geoms) == {_MESH_GEOM}
    geom, material = geoms[_MESH_GEOM]
    assert material == "MESH_MATERIAL"
    assert len(geom.triangles) == 2   # whole mesh, unsplit


def test_translucent_mode_splits_wall_from_floor():
    fake = _FakeWallPanel(wall_mode="translucent")
    mesh = _corner_tensor_mesh()
    panel_mod.ControlPanel._upload_slam_mesh(fake, mesh)

    geoms = fake.scene_widget.scene.geoms
    assert set(geoms) == {_MESH_GEOM, _MESH_WALLS_GEOM}
    floor_geom, floor_mat = geoms[_MESH_GEOM]
    wall_geom, wall_mat = geoms[_MESH_WALLS_GEOM]
    assert floor_mat == "MESH_MATERIAL"
    assert wall_mat == "WALL_TRANSLUCENT_MATERIAL"
    assert isinstance(wall_geom, o3d.geometry.TriangleMesh)
    assert len(floor_geom.triangles) == 1
    assert len(wall_geom.triangles) == 1


def test_wireframe_mode_uploads_walls_as_lineset():
    fake = _FakeWallPanel(wall_mode="wireframe")
    mesh = _corner_tensor_mesh()
    panel_mod.ControlPanel._upload_slam_mesh(fake, mesh)

    geoms = fake.scene_widget.scene.geoms
    assert set(geoms) == {_MESH_GEOM, _MESH_WALLS_GEOM}
    wall_geom, wall_mat = geoms[_MESH_WALLS_GEOM]
    assert isinstance(wall_geom, o3d.geometry.LineSet)
    assert wall_mat == "WALL_WIRE_MATERIAL"
    assert len(wall_geom.lines) == 3   # one triangle -> 3 edges


def test_stale_walls_geometry_removed_when_switching_to_solid():
    """A previous translucent/wireframe upload must not leave a stale
    `_MESH_WALLS_GEOM` behind once the mode flips back to solid."""
    fake = _FakeWallPanel(wall_mode="translucent")
    mesh = _corner_tensor_mesh()
    panel_mod.ControlPanel._upload_slam_mesh(fake, mesh)
    assert _MESH_WALLS_GEOM in fake.scene_widget.scene.geoms

    fake.wall_mode = "solid"
    panel_mod.ControlPanel._upload_slam_mesh(fake, mesh)
    assert set(fake.scene_widget.scene.geoms) == {_MESH_GEOM}


def test_remove_slam_geometries_clears_walls_geom():
    fake = _FakeWallPanel(wall_mode="translucent")
    mesh = _corner_tensor_mesh()
    panel_mod.ControlPanel._upload_slam_mesh(fake, mesh)
    assert _MESH_WALLS_GEOM in fake.scene_widget.scene.geoms

    panel_mod.ControlPanel._remove_slam_geometries(fake)
    assert fake.scene_widget.scene.geoms == {}


def test_on_wall_mode_updates_state_and_invalidates_identity_caches():
    fake = _FakeWallPanel(wall_mode="solid")
    panel_mod.ControlPanel._on_wall_mode(fake, "wireframe", 2)
    assert fake.wall_mode == "wireframe"
    assert fake._slam_last_mesh_obj is None
    assert fake._showcase_last_mesh_obj is None


@pytest.mark.parametrize("mode", ["solid", "translucent", "wireframe"])
def test_all_wall_triangles_produces_no_floor_geometry(mode):
    """An all-wall mesh (no floor/ceiling triangles) must not upload an empty
    `_MESH_GEOM`."""
    fake = _FakeWallPanel(wall_mode=mode)
    verts = np.array([
        [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0],
    ], dtype=np.float32)
    tris = np.array([[0, 1, 2]], dtype=np.int32)
    m = o3d.t.geometry.TriangleMesh()
    m.vertex.positions = o3d.core.Tensor(verts)
    m.triangle.indices = o3d.core.Tensor(tris)

    panel_mod.ControlPanel._upload_slam_mesh(fake, m)
    geoms = fake.scene_widget.scene.geoms
    if mode == "solid":
        assert set(geoms) == {_MESH_GEOM}   # solid never splits
    else:
        assert _MESH_GEOM not in geoms      # nothing non-wall to show
        assert _MESH_WALLS_GEOM in geoms
