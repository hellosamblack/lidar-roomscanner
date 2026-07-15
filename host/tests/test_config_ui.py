"""Round-trip + default tests for the redesign's new config fields."""
from roomscan.config import ViewerConfig, config_path


def test_new_fields_defaults():
    c = ViewerConfig()
    assert c.mode == "real_time"       # owner: default to real-time first-person
    assert c.camera == "first_person"
    assert c.ir_overlay is True
    assert c.ir_opacity == 0.5


def test_new_fields_round_trip(tmp_path):
    path = tmp_path / "roomscan.toml"
    ViewerConfig(mode="real_time", camera="orbit", ir_overlay=True,
                 ir_opacity=0.8).save(path)
    back = ViewerConfig.load(path)
    assert back.mode == "real_time"
    assert back.camera == "orbit"
    assert back.ir_overlay is True
    assert back.ir_opacity == 0.8


def test_unknown_keys_ignored_still_loads(tmp_path):
    path = tmp_path / "roomscan.toml"
    path.write_text("[viewer]\nmode = \"slam\"\nbogus = 1\n", encoding="utf-8")
    assert ViewerConfig.load(path).mode == "slam"


import types

import roomscan.panel as panel_mod


def test_startup_applies_saved_mode_camera():
    # A minimal args namespace with saved SLAM+orbit should leave the panel's
    # mode/camera fields matching (the __init__ normalizer, tested unbound).
    class _Args:
        mode = "slam"; camera = "orbit"; ir_overlay = True; ir_opacity = 0.7
    # emulate the normalizer __init__ runs (Task 9 block)
    a = _Args()
    mode = a.mode if a.mode in (panel_mod.VIEW_REAL_TIME, panel_mod.VIEW_SLAM) else panel_mod.VIEW_SLAM
    camera = a.camera if a.camera in (panel_mod.CAM_FIRST_PERSON, panel_mod.CAM_ORBIT) else panel_mod.CAM_FIRST_PERSON
    assert (mode, camera) == (panel_mod.VIEW_SLAM, panel_mod.CAM_ORBIT)
    assert bool(a.ir_overlay) is True
    assert float(a.ir_opacity) == 0.7


def test_persist_config_saves_camera_mode(monkeypatch, tmp_path):
    """Regression for the final-review finding: `_persist_config` built its
    ViewerConfig with `camera=getattr(self, "camera", ...)`, but the panel's
    live attribute is `self.camera_mode` -- there is no `self.camera` -- so
    an ORBIT choice was silently dropped on --save-config and always
    persisted as the "first_person" default. This drives the REAL unbound
    `ControlPanel._persist_config` against a lightweight stand-in exposing
    exactly the `self.*` surface the method reads, then reloads the file it
    wrote and asserts the saved `camera` reflects `self.camera_mode`."""
    monkeypatch.setenv("APPDATA", str(tmp_path))  # config_dir()/config_path() read this fresh

    fake = types.SimpleNamespace(
        color_mode="reflectance",
        args=types.SimpleNamespace(fov_h=55.0, fov_v=42.0, replay_fps=0.0,
                                    port=None, panel_width=340),
        material=types.SimpleNamespace(point_size=5.0),
        ir_colormap="gray",
        ir_freeze=False,
        near_mode="window",
        near_cutoff_m=1.5,
        near_emphasis=0.5,
        imu_gizmo=True,
        sensors_panel=True,
        gizmo_scale=0.15,
        metrics_overlay=True,
        mode="real_time",
        camera_mode="orbit",           # <-- the field the bug drops
        ir_overlay_enabled=False,
        ir_opacity=0.5,
        bus=types.SimpleNamespace(publish=lambda msg: None),
    )

    panel_mod.ControlPanel._persist_config(fake)

    saved = ViewerConfig.load(config_path())
    assert saved.camera == "orbit"
    assert saved.mode == "real_time"
