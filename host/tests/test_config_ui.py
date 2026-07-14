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
