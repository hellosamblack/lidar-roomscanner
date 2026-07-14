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
