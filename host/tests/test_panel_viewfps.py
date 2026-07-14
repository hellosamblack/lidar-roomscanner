from collections import deque
import roomscan.panel as panel_mod


class _FakeViewFpsPanel:
    def __init__(self):
        self._view_ticks = deque()
        self._view_fps_window_s = 1.0


def test_view_fps_zero_below_two_ticks():
    fake = _FakeViewFpsPanel()
    assert panel_mod.ControlPanel._view_fps(fake, 0.0) == 0.0
    panel_mod.ControlPanel._record_view_tick(fake, 0.0)
    assert panel_mod.ControlPanel._view_fps(fake, 0.0) == 0.0   # still one tick


def test_view_fps_counts_ticks_per_second():
    fake = _FakeViewFpsPanel()
    for i in range(11):                       # 11 ticks spanning 1.0 s -> 10 fps
        panel_mod.ControlPanel._record_view_tick(fake, i * 0.1)
    fps = panel_mod.ControlPanel._view_fps(fake, 1.0)
    assert abs(fps - 10.0) < 1e-6


def test_view_fps_trims_old_ticks_outside_window():
    fake = _FakeViewFpsPanel()
    panel_mod.ControlPanel._record_view_tick(fake, 0.0)   # older than the window
    for i in range(1, 7):
        panel_mod.ControlPanel._record_view_tick(fake, 2.0 + i * 0.1)
    # at now=2.6, the t=0.0 tick is >1s old and must be dropped
    assert all(t >= 1.6 for t in fake._view_ticks)


def test_render_hud_accepts_view_fps():
    import numpy as np
    from roomscan.metrics import MetricsSnapshot
    from roomscan.metrics_hud import render_hud
    snap = MetricsSnapshot(0.0, [], 0.0, None)
    img = render_hud(snap, view_fps=42.0)
    assert isinstance(img, np.ndarray) and img.ndim == 3
