"""Pure tests for the floating HUD renders + hit-test model (no GUI)."""
import numpy as np

from roomscan import hud, instrument


def _accent_pixels(img):
    rgb = img[..., :3].reshape(-1, 3)
    return int(np.all(rgb == np.array(instrument.ACCENT_U8), axis=1).sum())


def test_render_mode_switch_shape_and_dtype():
    img = hud.render_mode_switch("slam")
    assert img.shape == (34, 220, 4)
    assert img.dtype == np.uint8


def test_render_mode_switch_uses_accent():
    assert _accent_pixels(hud.render_mode_switch("slam")) > 10


def test_render_mode_switch_active_differs():
    a = hud.render_mode_switch("real_time")
    b = hud.render_mode_switch("slam")
    assert not np.array_equal(a, b)          # the highlighted segment moved


def test_render_mode_switch_deterministic():
    np.testing.assert_array_equal(hud.render_mode_switch("slam"),
                                  hud.render_mode_switch("slam"))


def test_render_view_toggle_shape():
    assert hud.render_view_toggle("first_person").shape == (34, 190, 4)


def test_render_action_cluster_phases_differ():
    idle = hud.render_action_cluster("idle", is_replay=False)
    rec = hud.render_action_cluster("recording", is_replay=False)
    assert idle.shape == (44, 300, 4)
    assert not np.array_equal(idle, rec)


def test_render_ir_control_opacity_changes_render():
    lo = hud.render_ir_control(True, 0.1)
    hi = hud.render_ir_control(True, 0.9)
    assert lo.shape == (40, 220, 4)
    assert not np.array_equal(lo, hi)        # the slider fill width moved


def test_render_status_chip_shape_and_ascii_only_inputs():
    img = hud.render_status_chip("ok", 58.0)
    assert img.shape == (28, 200, 4)


def test_renders_have_transparent_margin():
    img = hud.render_mode_switch("slam")
    # top-left corner pixel is in the 1px transparent margin
    assert img[0, 0, 3] == 0


from roomscan.hud import ControlHit, HudLayout


def test_layout_rects_present_for_slam():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    r = lay.rects()
    assert set(r) >= {hud.MODE_SWITCH, hud.VIEW_TOGGLE, hud.STATUS_CHIP,
                      hud.ACTION_CLUSTER, hud.IR_CONTROL}


def test_layout_real_time_hides_action_cluster():
    lay = HudLayout(0, 0, 1000, 700, mode="real_time")
    assert hud.ACTION_CLUSTER not in lay.rects()


def test_mode_switch_is_top_center():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    x, y, w, h = lay.rects()[hud.MODE_SWITCH]
    assert y == 12                                   # top row
    assert abs((x + w / 2) - 500) <= 1               # horizontally centered


def test_hit_test_mode_switch_second_segment():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    x, y, w, h = lay.rects()[hud.MODE_SWITCH]
    hit = lay.hit_test(int(x + w * 0.75), int(y + h / 2))
    assert hit == ControlHit(hud.MODE_SWITCH, segment=1)


def test_hit_test_miss_returns_none():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    assert lay.hit_test(500, 350) is None            # dead center of the scene


def test_hit_test_ir_slider_fraction():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    x, y, w, h = lay.rects()[hud.IR_CONTROL]
    # click ~ the far right of the track region -> fraction near 1.0
    hit = lay.hit_test(int(x + w - 14), int(y + h / 2))
    assert hit.control == hud.IR_CONTROL
    assert hit.fraction is not None and hit.fraction > 0.8


def test_hit_test_status_chip_readonly():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    x, y, w, h = lay.rects()[hud.STATUS_CHIP]
    hit = lay.hit_test(int(x + w / 2), int(y + h / 2))
    assert hit == ControlHit(hud.STATUS_CHIP)
