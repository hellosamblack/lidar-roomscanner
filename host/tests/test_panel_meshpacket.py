import numpy as np
import pytest

pytest.importorskip("open3d")
import open3d as o3d

import roomscan.panel as panel_mod
from roomscan.slam.meshprep import MeshPacket

_MESH_GEOM = panel_mod._MESH_GEOM
_MESH_WALLS_GEOM = panel_mod._MESH_WALLS_GEOM
_FLOOR_GRID_GEOM = panel_mod._FLOOR_GRID_GEOM


class _FakeScene:
    def __init__(self):
        self.geoms = {}

    def has_geometry(self, name):
        return name in self.geoms

    def add_geometry(self, name, geom, material):
        self.geoms[name] = (geom, material)

    def remove_geometry(self, name):
        del self.geoms[name]


class _FakeSceneWidget:
    def __init__(self):
        self.scene = _FakeScene()


class _FakePanel:
    def __init__(self, wall_mode="solid"):
        self._o3d = o3d
        self.scene_widget = _FakeSceneWidget()
        self.wall_mode = wall_mode
        self.mesh_material = "MESH_MATERIAL"
        self.wall_translucent_material = "WALL_TRANSLUCENT_MATERIAL"
        self.wall_wire_material = "WALL_WIRE_MATERIAL"
        self.floor_material = "FLOOR_MATERIAL"
        self._floor_last_bounds = None

    def _upload_floor_grid_from_packet(self, pts, lines):
        # `_upload_mesh_packet` calls this sibling method via `self`; the
        # class-unbound-call pattern (see test_panel_walls.py's
        # `_FakeWallPanel`) needs a shim so the real, unmodified
        # `ControlPanel._upload_floor_grid_from_packet` runs against this
        # fake's `scene_widget.scene` instead of AttributeError'ing.
        panel_mod.ControlPanel._upload_floor_grid_from_packet(self, pts, lines)


def _solid_packet(wall_mode="solid"):
    verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    colors = np.tile([0.5, 0.5, 0.5], (3, 1))
    tris = np.array([[0, 1, 2]], np.int32)
    return MeshPacket(
        non_wall_verts=verts, non_wall_colors=colors, non_wall_tris=tris,
        wall_verts=np.zeros((0, 3)), wall_colors=np.zeros((0, 3)),
        wall_tris=np.zeros((0, 3), np.int32),
        floor_pts=np.zeros((0, 3)), floor_lines=np.zeros((0, 2), np.int64),
        mesh_seq=1, source_vertex_count=3, decimated=False, wall_mode=wall_mode)


def _split_packet(wall_mode="translucent"):
    nwv = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    wv = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    tri = np.array([[0, 1, 2]], np.int32)
    grey = lambda n: np.tile([0.5, 0.5, 0.5], (n, 1))
    return MeshPacket(
        non_wall_verts=nwv, non_wall_colors=grey(3), non_wall_tris=tri,
        wall_verts=wv, wall_colors=grey(3), wall_tris=tri,
        floor_pts=np.zeros((0, 3)), floor_lines=np.zeros((0, 2), np.int64),
        mesh_seq=1, source_vertex_count=6, decimated=False, wall_mode=wall_mode)


def test_solid_packet_uploads_one_opaque_mesh():
    fake = _FakePanel()
    panel_mod.ControlPanel._upload_mesh_packet(fake, _solid_packet())
    geoms = fake.scene_widget.scene.geoms
    assert set(geoms) == {_MESH_GEOM}
    geom, material = geoms[_MESH_GEOM]
    assert material == "MESH_MATERIAL"
    assert len(geom.triangles) == 1


def test_translucent_packet_uploads_wall_and_floor_geoms():
    fake = _FakePanel(wall_mode="translucent")
    panel_mod.ControlPanel._upload_mesh_packet(fake, _split_packet("translucent"))
    geoms = fake.scene_widget.scene.geoms
    assert set(geoms) == {_MESH_GEOM, _MESH_WALLS_GEOM}
    assert geoms[_MESH_WALLS_GEOM][1] == "WALL_TRANSLUCENT_MATERIAL"
    assert isinstance(geoms[_MESH_WALLS_GEOM][0], o3d.geometry.TriangleMesh)


def test_wireframe_packet_uploads_walls_as_lineset():
    fake = _FakePanel(wall_mode="wireframe")
    panel_mod.ControlPanel._upload_mesh_packet(fake, _split_packet("wireframe"))
    geoms = fake.scene_widget.scene.geoms
    assert geoms[_MESH_WALLS_GEOM][1] == "WALL_WIRE_MATERIAL"
    assert isinstance(geoms[_MESH_WALLS_GEOM][0], o3d.geometry.LineSet)


def test_stale_geoms_cleared_on_reupload():
    fake = _FakePanel(wall_mode="translucent")
    panel_mod.ControlPanel._upload_mesh_packet(fake, _split_packet("translucent"))
    assert _MESH_WALLS_GEOM in fake.scene_widget.scene.geoms
    # a later solid packet must drop the stale walls geom
    panel_mod.ControlPanel._upload_mesh_packet(fake, _solid_packet("solid"))
    assert set(fake.scene_widget.scene.geoms) == {_MESH_GEOM}


def test_floor_grid_uploaded_when_present():
    fake = _FakePanel()
    pkt = _solid_packet()
    pkt.floor_pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    pkt.floor_lines = np.array([[0, 1]], np.int64)
    panel_mod.ControlPanel._upload_mesh_packet(fake, pkt)
    assert _FLOOR_GRID_GEOM in fake.scene_widget.scene.geoms
