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


import types

import numpy as np
import roomscan.panel as panel_mod


class _FakeSetModePanel:
    """Stand-in exercising the real _set_mode wiring (which toggle it arms)."""
    def __init__(self, mode):
        self.mode = mode
        self.showcase_enabled = False
        self.slam_enabled = False
        self.calls = []
        self.window = types.SimpleNamespace(set_needs_layout=lambda: None)
        self.bus = types.SimpleNamespace(publish=lambda m: None)

    def _on_showcase_toggle(self, c): self.calls.append(("showcase", c)); self.showcase_enabled = c
    def _on_slam_toggle(self, c): self.calls.append(("slam", c)); self.slam_enabled = c
    def _apply_camera_mode(self): self.calls.append("camera_mode")


def test_set_mode_slam_arms_showcase_not_classic_slam():
    # Spec: SLAM mode IS the record->process->reveal (Showcase) pipeline; the
    # action cluster drives ShowcasePhase, so SLAM must arm the showcase machine.
    fake = _FakeSetModePanel(panel_mod.VIEW_REAL_TIME)
    panel_mod.ControlPanel._set_mode(fake, panel_mod.VIEW_SLAM)
    assert ("showcase", True) in fake.calls
    assert ("slam", True) not in fake.calls
    assert fake.mode == panel_mod.VIEW_SLAM


def test_set_mode_real_time_disables_showcase():
    fake = _FakeSetModePanel(panel_mod.VIEW_SLAM)
    fake.showcase_enabled = True
    panel_mod.ControlPanel._set_mode(fake, panel_mod.VIEW_REAL_TIME)
    assert ("showcase", False) in fake.calls
    assert fake.mode == panel_mod.VIEW_REAL_TIME


class _FakeCam:
    class FovType:
        Vertical = "V"

    def __init__(self):
        self.proj = None

    def set_projection(self, fov, aspect, near, far, ftype):
        self.proj = (fov, aspect, near, far, ftype)


def _fp_fake(width=800, height=600):
    cam = _FakeCam()
    look_at = []
    fake = types.SimpleNamespace(
        scene_widget=types.SimpleNamespace(
            frame=types.SimpleNamespace(width=width, height=height),
            scene=types.SimpleNamespace(camera=cam),
            look_at=lambda c, e, u: look_at.append((c, e, u))),
        _camera_set=False)
    return fake, cam, look_at


def test_real_time_first_person_aims_view_without_pinning():
    # Only aims the fixed look_at view. It must NOT pin _camera_set nor set the
    # projection: pinning blocked _reset_camera from establishing the projection,
    # leaving first-person broken until an orbit->first-person round-trip primed
    # it. Projection is owned by _reset_camera; the view is re-applied per frame.
    fake, cam, look_at = _fp_fake()
    fake._camera_set = False
    panel_mod.ControlPanel._apply_real_time_first_person(fake)
    assert len(look_at) == 1
    assert fake._camera_set is False      # not pinned -> _reset_camera still runs
    assert cam.proj is None               # projection left to _reset_camera


def test_real_time_first_person_noop_without_viewport():
    fake, cam, look_at = _fp_fake(width=0, height=0)
    panel_mod.ControlPanel._apply_real_time_first_person(fake)
    assert look_at == []


def test_follow_alpha_stationary_smooths_motion_snaps():
    from roomscan.panel import _follow_alpha, _FOLLOW_SMOOTH, _FOLLOW_SNAP_M
    assert _follow_alpha(0.0) == _FOLLOW_SMOOTH               # ~stationary -> smoothed
    assert _follow_alpha(_FOLLOW_SNAP_M) == 1.0               # real motion -> 1:1
    assert _follow_alpha(10 * _FOLLOW_SNAP_M) == 1.0          # clamped
    mid = _follow_alpha(_FOLLOW_SNAP_M / 2)
    assert _FOLLOW_SMOOTH <= mid <= 1.0                       # monotone ramp


def test_apply_camera_mode_orbit_clears_camera_set():
    # Entering ORBIT (either mode) must clear _camera_set so the view refits to
    # all content on the next frame (owner: auto-zoom to fit).
    for mode in (panel_mod.VIEW_SLAM, panel_mod.VIEW_REAL_TIME):
        fake = types.SimpleNamespace(
            mode=mode, camera_mode=panel_mod.CAM_ORBIT,
            follow_camera_enabled=False, _camera_set=True,
            _follow_eye=None, _follow_center=None,
            _apply_camera=lambda: None, _remove_ir_overlay=lambda: None)
        panel_mod.ControlPanel._apply_camera_mode(fake)
        assert fake._camera_set is False, mode


def test_set_ir_opacity_slider_enables_overlay():
    # Slider doubles as on/off: >0 opacity enables the overlay (owner: "slider to
    # full -> see IR"); the render gate is `fp and ir_overlay_enabled`.
    fake = types.SimpleNamespace(ir_opacity=0.0, ir_overlay_enabled=False,
                                 _remove_ir_overlay=lambda: None)
    panel_mod.ControlPanel._set_ir_opacity(fake, 1.0)
    assert fake.ir_opacity == 1.0 and fake.ir_overlay_enabled is True


def test_set_ir_opacity_zero_disables_overlay():
    removed = []
    fake = types.SimpleNamespace(ir_opacity=1.0, ir_overlay_enabled=True,
                                 _remove_ir_overlay=lambda: removed.append(1))
    panel_mod.ControlPanel._set_ir_opacity(fake, 0.0)
    assert fake.ir_overlay_enabled is False and removed == [1]


def test_toggle_ir_on_at_zero_opacity_bumps_it_visible():
    fake = types.SimpleNamespace(ir_overlay_enabled=False, ir_opacity=0.0,
                                 _remove_ir_overlay=lambda: None,
                                 bus=types.SimpleNamespace(publish=lambda m: None))
    panel_mod.ControlPanel._toggle_ir_overlay(fake)
    assert fake.ir_overlay_enabled is True and fake.ir_opacity == 1.0


def test_on_mouse_swallows_nav_in_real_time_first_person():
    gui = __import__("pytest").importorskip("open3d").visualization.gui
    fake = types.SimpleNamespace(
        _gui=gui, mode=panel_mod.VIEW_REAL_TIME, camera_mode=panel_mod.CAM_FIRST_PERSON,
        _cam_target=None, follow_camera_enabled=False)
    ev = types.SimpleNamespace(type=gui.MouseEvent.Type.DRAG, x=10, y=10)
    res = panel_mod.ControlPanel._on_mouse(fake, ev)
    assert res == gui.SceneWidget.EventCallbackResult.CONSUMED   # fixed view, nav swallowed


def test_remove_camera_gizmo_clears_geometry_and_flag():
    removed = []
    fake = types.SimpleNamespace(
        _gizmo_added=True,
        scene_widget=types.SimpleNamespace(scene=types.SimpleNamespace(
            has_geometry=lambda n: True,
            remove_geometry=lambda n: removed.append(n))))
    panel_mod.ControlPanel._remove_camera_gizmo(fake)
    assert removed == [panel_mod._GIZMO_GEOM]
    assert fake._gizmo_added is False


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


class _FakeHudPanel:
    def __init__(self):
        self.mode = panel_mod.VIEW_REAL_TIME
        self.camera_mode = panel_mod.CAM_FIRST_PERSON
        self.ir_overlay_enabled = False
        self.ir_opacity = 0.5
        self._mode_calls = []
        self._cam_calls = []

    # stubs the dispatch calls into (Task 10 supplies the real ones)
    def _set_mode(self, m): self.mode = m; self._mode_calls.append(m)
    def _set_camera(self, c): self.camera_mode = c; self._cam_calls.append(c)
    def _do_action(self, seg): pass
    def _toggle_ir_overlay(self): self.ir_overlay_enabled = not self.ir_overlay_enabled
    def _set_ir_opacity(self, f): self.ir_opacity = f
    def _hud_action_labels(self):
        return ["REC", "LOAD", "CLR"]


def test_dispatch_mode_switch_sets_slam():
    from roomscan.hud import ControlHit, MODE_SWITCH
    fake = _FakeHudPanel()
    consumed = panel_mod.ControlPanel._dispatch_hud_hit(fake, ControlHit(MODE_SWITCH, segment=1))
    assert consumed is True
    assert fake.mode == panel_mod.VIEW_SLAM


def test_dispatch_view_toggle_sets_orbit():
    from roomscan.hud import ControlHit, VIEW_TOGGLE
    fake = _FakeHudPanel()
    panel_mod.ControlPanel._dispatch_hud_hit(fake, ControlHit(VIEW_TOGGLE, segment=1))
    assert fake.camera_mode == panel_mod.CAM_ORBIT


def test_dispatch_ir_fraction_sets_opacity():
    from roomscan.hud import ControlHit, IR_CONTROL
    fake = _FakeHudPanel()
    panel_mod.ControlPanel._dispatch_hud_hit(fake, ControlHit(IR_CONTROL, fraction=0.75))
    assert fake.ir_opacity == 0.75


def test_dispatch_ir_label_toggles():
    from roomscan.hud import ControlHit, IR_CONTROL
    fake = _FakeHudPanel()
    panel_mod.ControlPanel._dispatch_hud_hit(fake, ControlHit(IR_CONTROL, segment=0))
    assert fake.ir_overlay_enabled is True


def test_on_hud_widget_mouse_dispatches_click_and_consumes():
    """The floating-HUD ImageWidgets carry their OWN set_on_mouse (they sit atop
    the SceneWidget, which never sees clicks over them -- the mouse-passthrough
    bug). A BUTTON_DOWN over a control must route through HudLayout.hit_test ->
    _dispatch_hud_hit and be consumed."""
    gui = __import__("pytest").importorskip("open3d").visualization.gui
    from roomscan.hud import HudLayout, MODE_SWITCH
    fake = _FakeHudPanel()
    fake._gui = gui
    fake._hud_layout = HudLayout(0, 0, 1000, 700, mode="slam")
    fake._dispatch_hud_hit = lambda hit: panel_mod.ControlPanel._dispatch_hud_hit(fake, hit)
    x, y, w, h = fake._hud_layout.rects()[MODE_SWITCH]
    ev = type("E", (), {"type": gui.MouseEvent.Type.BUTTON_DOWN,
                        "x": x + w * 0.75, "y": y + h / 2})()
    res = panel_mod.ControlPanel._on_hud_widget_mouse(fake, ev)
    assert res == gui.Widget.EventCallbackResult.CONSUMED
    assert fake.mode == panel_mod.VIEW_SLAM          # second segment -> SLAM


def test_on_hud_widget_mouse_consumes_move_without_dispatch():
    """MOVE/hover over a control is consumed (so it never leaks to camera nav)
    but drives no action."""
    gui = __import__("pytest").importorskip("open3d").visualization.gui
    from roomscan.hud import HudLayout, MODE_SWITCH
    fake = _FakeHudPanel()
    fake._gui = gui
    fake._hud_layout = HudLayout(0, 0, 1000, 700, mode="slam")
    fake._dispatch_hud_hit = lambda hit: panel_mod.ControlPanel._dispatch_hud_hit(fake, hit)
    x, y, w, h = fake._hud_layout.rects()[MODE_SWITCH]
    ev = type("E", (), {"type": gui.MouseEvent.Type.MOVE,
                        "x": x + w * 0.75, "y": y + h / 2})()
    res = panel_mod.ControlPanel._on_hud_widget_mouse(fake, ev)
    assert res == gui.Widget.EventCallbackResult.CONSUMED
    assert fake._mode_calls == []                    # MOVE dispatched nothing


def test_load_dialog_dispatches_by_kind(monkeypatch):
    calls = {}

    class _FakeLoadPanel:
        def _process_capture(self, path): calls["capture"] = path
        def _display_mesh_file(self, path): calls["mesh"] = path
        bus = type("B", (), {"publish": lambda self, m: None})()

    fake = _FakeLoadPanel()
    panel_mod.ControlPanel._load_path(fake, "captures/a.bin")
    panel_mod.ControlPanel._load_path(fake, "results/b.ply")
    panel_mod.ControlPanel._load_path(fake, "x.txt")
    assert calls == {"capture": "captures/a.bin", "mesh": "results/b.ply"}


def test_ir_overlay_builds_and_removes_geometry():
    __import__("pytest").importorskip("open3d")
    import numpy as np
    import open3d as o3d

    class _Scene:
        def __init__(self): self.geoms = {}
        def has_geometry(self, n): return n in self.geoms
        def add_geometry(self, n, g, m): self.geoms[n] = g
        def remove_geometry(self, n): self.geoms.pop(n, None)

    class _Fake:
        def __init__(self):
            self._o3d = o3d
            self.scene_widget = type("SW", (), {"scene": _Scene()})()
            self.args = type("A", (), {"fov_h": 55.0, "fov_v": 42.0})()
            self.ir_opacity = 0.5
            self._latest_outputs = {"reflectance": np.full((42, 54), 0.5, np.float32)}
            self.ir_colormap = "gray"
            self.ir_overlay_material = "M"

    fake = _Fake()
    panel_mod.ControlPanel._update_ir_overlay(fake, [0, 0, 0], [0, 0, 1])
    assert panel_mod._IR_OVERLAY_GEOM in fake.scene_widget.scene.geoms
    # Double-sided (4 triangles): the quad must render from the first-person eye,
    # which sees its back face -- a single-sided quad is culled (invisible).
    mesh = fake.scene_widget.scene.geoms[panel_mod._IR_OVERLAY_GEOM]
    assert len(mesh.triangles) == 4
    panel_mod.ControlPanel._remove_ir_overlay(fake)
    assert panel_mod._IR_OVERLAY_GEOM not in fake.scene_widget.scene.geoms
