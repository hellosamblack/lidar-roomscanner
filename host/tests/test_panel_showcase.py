"""Headless tests for two Showcase-mode `ControlPanel` fixes that don't need
a live GUI window: Issue #3 (FINAL-phase camera ease must yield to a manual
drag, not resume-and-snap after it) and Issue #4 (a superseded post-process
loader thread must not clobber the live worker slot).

Both call the real, unmodified `ControlPanel` methods directly (unbound, on a
lightweight stand-in object) rather than instantiating a real `ControlPanel`
-- that requires a live Open3D/Filament window, which fails headless on this
box (see panel.py's own non-GUI test seams in test_panel.py). Every attribute
the method under test actually touches is present on the stand-in; nothing
else about `ControlPanel` is exercised.
"""
import threading
import types

import numpy as np
import open3d.visualization.gui as gui
import pytest

import roomscan.panel as panel_mod
from roomscan.logbus import LogBus

# ---- Issue #3: manual drag must cancel (not merely pause) the camera ease --


class _FakeMouseEvent:
    def __init__(self, type_, x=0, y=0):
        self.type = type_
        self.x = x
        self.y = y


class _FakeMouseSelf:
    def __init__(self, showcase_ease=None):
        self._gui = gui
        self._cam_target = np.zeros(3)   # non-None -> BUTTON_DOWN isn't ignored
        self._drag = None
        self._showcase_ease = showcase_ease


def test_button_down_cancels_in_flight_showcase_ease():
    fake = _FakeMouseSelf(showcase_ease={"t0": 0.0, "duration": 1.5})
    ev = _FakeMouseEvent(gui.MouseEvent.Type.BUTTON_DOWN, x=5, y=7)
    result = panel_mod.ControlPanel._on_mouse(fake, ev)
    assert fake._drag == (5, 7)
    assert fake._showcase_ease is None   # cancelled outright, not paused
    assert result == gui.SceneWidget.EventCallbackResult.CONSUMED


def test_button_down_is_a_noop_on_ease_when_none_in_flight():
    fake = _FakeMouseSelf(showcase_ease=None)
    ev = _FakeMouseEvent(gui.MouseEvent.Type.BUTTON_DOWN, x=1, y=2)
    panel_mod.ControlPanel._on_mouse(fake, ev)
    assert fake._showcase_ease is None


# ---- Issue #4: a superseded post-process loader must not clobber the slot -


class _FakeWorker:
    def __init__(self):
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True


class _FakePostProcessPanel:
    """Only the attributes `_start_showcase_post_process` /
    `_join_showcase_workers` actually touch."""

    def __init__(self):
        self._showcase_generation = 0
        self._showcase_post_worker = None
        self._showcase_preview_worker = None
        self._showcase_loader_thread = None
        self.bus = LogBus()
        self.args = types.SimpleNamespace(fov_h=60.0, fov_v=45.0)


def test_start_post_process_assigns_worker_on_the_happy_path(monkeypatch):
    fp = _FakePostProcessPanel()
    worker = _FakeWorker()

    import roomscan.slam.showcase as showcase_mod
    monkeypatch.setattr(showcase_mod.PostProcessWorker, "from_capture",
                        staticmethod(lambda path, **kw: worker))

    panel_mod.ControlPanel._start_showcase_post_process(fp, "fake.bin")
    fp._showcase_loader_thread.join(timeout=2.0)

    assert fp._showcase_post_worker is worker
    assert worker.started


def test_superseded_loader_does_not_publish_into_live_slot(monkeypatch):
    """Reproduces Issue #4: a loader thread is still in `from_capture`
    (simulating a slow capture load) when the panel "moves on" -- a new
    recording, Clear, or window close, all of which bump
    `_showcase_generation` via `_join_showcase_workers`. The stale load must
    finish without ever assigning its (possibly still-running) worker into
    `self._showcase_post_worker`, and must stop a worker it already
    started."""
    fp = _FakePostProcessPanel()
    worker = _FakeWorker()
    loading_started = threading.Event()
    release_load = threading.Event()

    def _slow_from_capture(path, **kw):
        loading_started.set()
        assert release_load.wait(timeout=5.0), "test setup: release_load never set"
        return worker

    import roomscan.slam.showcase as showcase_mod
    monkeypatch.setattr(showcase_mod.PostProcessWorker, "from_capture",
                        staticmethod(_slow_from_capture))

    panel_mod.ControlPanel._start_showcase_post_process(fp, "fake.bin")
    assert loading_started.wait(timeout=2.0), "loader never started"

    # The panel moves on while the load is still in flight -- e.g. a fresh
    # Record press or Clear -- which is exactly what _join_showcase_workers
    # does to the generation counter. _join_showcase_workers bumps the
    # generation as its very first step and only then joins the (still
    # in-flight) loader thread, so schedule the unblock a beat later instead
    # of releasing it up front -- otherwise the loader could race past its
    # generation check before the bump ever happens, defeating the point of
    # this test.
    threading.Timer(0.05, release_load.set).start()
    panel_mod.ControlPanel._join_showcase_workers(fp)

    assert fp._showcase_post_worker is None   # never clobbered
    assert worker.stopped or not worker.started   # never left running unsupervised


def test_join_showcase_workers_bumps_generation():
    fp = _FakePostProcessPanel()
    fp._showcase_generation = 3
    panel_mod.ControlPanel._join_showcase_workers(fp)
    assert fp._showcase_generation == 4
