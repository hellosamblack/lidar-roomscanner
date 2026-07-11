"""Unit tests for the headless SLAM worker (Task 10): the frame handler that
sits behind the panel's "SLAM" view mode, not the GUI itself (that part is
live-only, validated on hardware in Task 11)."""
import time

import numpy as np

from roomscan.slam.worker import SlamWorker

W, H = 54, 42


def _wall(z_m=1.0):
    return np.full((H, W), z_m * 1000.0, dtype=np.float32)


def test_worker_processes_and_publishes():
    w = SlamWorker(W, H, voxel_size=0.02)
    w.submit(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    w.run_once()                         # synchronous single-step for testing
    latest = w.latest()
    assert latest is not None
    mesh, traj, step = latest
    assert len(traj) == 1
    assert not step.tracking_lost


def test_latest_is_none_before_any_run_once():
    w = SlamWorker(W, H, voxel_size=0.02)
    assert w.latest() is None


def test_submit_is_latest_wins():
    """Two submits before one run_once() must process only the second -- the
    first (an all-invalid, degenerate frame) is dropped unprocessed. Proven
    behaviorally: the very first frame ever is a bootstrap, so if the
    *degenerate* frame had won, tracking would be lost; since the *wall*
    frame has plenty of valid points, it bootstraps cleanly instead."""
    w = SlamWorker(W, H, voxel_size=0.02)
    degenerate = np.zeros((H, W), dtype=np.float32)   # all-invalid -> would be lost
    w.submit(degenerate, (1.0, 0.0, 0.0, 0.0), 101325.0)
    w.submit(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)   # latest input wins
    w.run_once()
    latest = w.latest()
    assert latest is not None
    _, traj, step = latest
    assert len(traj) == 1              # exactly one step ran (the dropped submit never processed)
    assert not step.tracking_lost      # ran on the wall, not the degenerate frame


def test_run_once_returns_false_when_nothing_submitted():
    w = SlamWorker(W, H, voxel_size=0.02)
    assert w.run_once() is False
    assert w.latest() is None


def test_start_stop_processes_in_background_and_does_not_hang():
    w = SlamWorker(W, H, voxel_size=0.02)
    w.start()
    try:
        w.submit(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
        deadline = time.monotonic() + 2.0
        while w.latest() is None and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        t0 = time.monotonic()
        w.stop()
        stop_elapsed = time.monotonic() - t0
    assert w.latest() is not None
    assert stop_elapsed < 1.5           # stop() must not hang / block indefinitely
    assert w._thread is None            # joined and cleared
