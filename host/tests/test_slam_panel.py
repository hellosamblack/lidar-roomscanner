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


def test_first_frame_tracking_lost_does_not_crash():
    """If the very first frame ever submitted is degenerate (tracking-lost),
    Mapper.step never integrates into the TSDF, so the map is still empty
    when the worker's mesh-extraction throttle would otherwise fire on
    "first processed frame". This must not raise (Task 10 bugfix) -- the
    worker should still publish a result (so the panel HUD updates), just
    with no/empty mesh."""
    w = SlamWorker(W, H, voxel_size=0.02)
    degenerate = np.zeros((H, W), dtype=np.float32)   # all-invalid -> bootstrap frame is lost
    w.submit(degenerate, (1.0, 0.0, 0.0, 0.0), 101325.0)
    w.run_once()                                       # must not raise
    latest = w.latest()
    assert latest is not None
    mesh, traj, step = latest
    assert len(traj) == 1
    assert step.tracking_lost
    assert mesh is None or len(mesh.vertex.positions) == 0

    # NOTE: Mapper.step's bootstrap gate is keyed off `not self.trajectory`
    # (i.e. "has step() ever run"), not "has the TSDF ever integrated" --
    # once *any* frame has run (lost or not), later frames take the
    # model-based ICP path, which can never recover if the map is still
    # empty (raycast() -> None -> lost again). So a worker whose very first
    # frame is lost stays lost forever; it does NOT self-heal on a
    # subsequent valid frame. That's a separate, pre-existing gap in
    # Mapper's tracking state machine (out of scope here -- this task's
    # fix is only "don't crash"; see task-10-report.md). A *fresh* worker
    # bootstraps normally, per test_worker_processes_and_publishes above.


def test_mesh_throttle_counts_successful_frames_not_processed_frames():
    """The mesh-extraction throttle must key off successful (integrated)
    frames, not merely processed ones -- a lost frame must not itself
    trigger (a pointless, and pre-fix crash-prone) mesh() call, but must
    still publish so the HUD updates."""
    w = SlamWorker(W, H, voxel_size=0.02, mesh_every=2)
    w.submit(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    w.run_once()                                        # frame 1: successful bootstrap
    mesh1, traj1, step1 = w.latest()
    assert not step1.tracking_lost
    assert mesh1 is not None and len(mesh1.vertex.positions) >= 0
    assert w._frames_integrated == 1

    degenerate = np.zeros((H, W), dtype=np.float32)
    w.submit(degenerate, (1.0, 0.0, 0.0, 0.0), 101325.0)
    w.run_once()                                        # frame 2: tracking-lost, still publishes
    mesh2, traj2, step2 = w.latest()
    assert step2.tracking_lost
    assert len(traj2) == 2
    assert w._frames_integrated == 1                    # lost frame must not bump the counter


def test_submit_forwards_reflectance_and_confidence_to_mapper_step(monkeypatch):
    # Task 13: submit()'s optional reflectance/confidence must reach
    # Mapper.step, not be silently dropped by the worker's in-slot plumbing.
    from roomscan.slam.mapper import Mapper

    seen = {}
    orig_step = Mapper.step

    def spy_step(self, depth, quat, pressure_pa=None, reflectance=None, confidence=None):
        seen["reflectance"] = reflectance
        seen["confidence"] = confidence
        return orig_step(self, depth, quat, pressure_pa, reflectance=reflectance, confidence=confidence)

    monkeypatch.setattr(Mapper, "step", spy_step)
    w = SlamWorker(W, H, voxel_size=0.02)
    reflectance = np.full((H, W), 42.0, dtype=np.float32)
    confidence = np.full((H, W), 200.0, dtype=np.float32)
    w.submit(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0, reflectance=reflectance, confidence=confidence)
    w.run_once()
    assert seen["reflectance"] is reflectance
    assert seen["confidence"] is confidence


def test_submit_defaults_reflectance_and_confidence_to_none():
    # Existing 3-positional-arg call sites (panel.py) must keep working
    # unchanged -- reflectance/confidence default to None.
    w = SlamWorker(W, H, voxel_size=0.02)
    w.submit(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    w.run_once()
    latest = w.latest()
    assert latest is not None
    assert not latest[2].tracking_lost


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
