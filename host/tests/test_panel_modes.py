"""Pure state-model predicates for the two-mode / two-camera panel redesign."""
import roomscan.panel as p


def test_follow_active_only_slam_first_person():
    assert p.follow_active(p.VIEW_SLAM, p.CAM_FIRST_PERSON) is True
    assert p.follow_active(p.VIEW_SLAM, p.CAM_ORBIT) is False
    assert p.follow_active(p.VIEW_REAL_TIME, p.CAM_FIRST_PERSON) is False


def test_gizmo_should_update_only_orbit():
    assert p.gizmo_should_update(p.CAM_ORBIT, True) is True
    assert p.gizmo_should_update(p.CAM_FIRST_PERSON, True) is False
    assert p.gizmo_should_update(p.CAM_ORBIT, False) is False


def test_real_time_first_person():
    assert p.real_time_first_person(p.VIEW_REAL_TIME, p.CAM_FIRST_PERSON) is True
    assert p.real_time_first_person(p.VIEW_SLAM, p.CAM_FIRST_PERSON) is False


def test_load_kind_by_suffix():
    assert p.load_kind("captures/panel_x.bin") == "capture"
    assert p.load_kind("results/showcase_y.PLY") == "mesh"
    assert p.load_kind("foo.txt") == "unknown"


import numpy as np
import roomscan.panel as panel_mod


class _FakeGizmoScene:
    def __init__(self):
        self.geoms = {}

    def has_geometry(self, n):
        return n in self.geoms

    def add_geometry(self, n, g, m):
        self.geoms[n] = g

    def remove_geometry(self, n):
        self.geoms.pop(n, None)

    def set_geometry_transform(self, n, t):
        pass


class _FakeGizmoPanel:
    def __init__(self, camera):
        import open3d as o3d
        self._o3d = o3d
        self.imu_gizmo = True
        self.camera_mode = camera
        self.gizmo_scale = 0.15
        self._gizmo_added = False
        self.mesh_material = "M"
        self.scene_widget = type("SW", (), {"scene": _FakeGizmoScene()})()


def test_gizmo_not_added_in_first_person():
    pytest_o3d = __import__("pytest").importorskip("open3d")
    fake = _FakeGizmoPanel(panel_mod.CAM_FIRST_PERSON)
    quat = (1.0, 0.0, 0.0, 0.0)
    panel_mod.ControlPanel._update_camera_gizmo(fake, quat)
    assert panel_mod._GIZMO_GEOM not in fake.scene_widget.scene.geoms
    assert fake._gizmo_added is False


def test_gizmo_added_in_orbit():
    __import__("pytest").importorskip("open3d")
    fake = _FakeGizmoPanel(panel_mod.CAM_ORBIT)
    panel_mod.ControlPanel._update_camera_gizmo(fake, (1.0, 0.0, 0.0, 0.0))
    assert panel_mod._GIZMO_GEOM in fake.scene_widget.scene.geoms
