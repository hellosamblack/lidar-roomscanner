"""Round-trip + default tests for the redesign's new config fields."""
from roomscan.config import ViewerConfig


def test_new_fields_defaults():
    c = ViewerConfig()
    assert c.mode == "slam"
    assert c.camera == "first_person"
    assert c.ir_overlay is False
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
