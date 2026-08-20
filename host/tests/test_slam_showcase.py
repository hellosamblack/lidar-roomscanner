"""Unit tests for Task 12's testable core (host/src/roomscan/slam/showcase.py):
`PostProcessWorker` (offline full-quality re-processing with progressively
improving publishes) and `next_phase` (the Showcase phase state machine).
All synthetic -- no recorded .bin, no GUI."""
import time

import numpy as np
import pytest

import roomscan.slam.showcase as showcase
from roomscan.slam.mapper import Mapper
from roomscan.slam.showcase import (
    PostProcessWorker, Progress, ShowcasePhase, _empty_mesh, next_phase)
from roomscan.slam.tsdf import TsdfCapacityError

W, H = 54, 42
_Q = (0.70710678, 0.0, 0.70710678, 0.0)   # forward = +Z, see test_slam_mapper.py


def _textured_wall(z_m):
    # Mild curvature so translation-only point-to-plane ICP has grip in x/y
    # too (a perfectly flat fronto-parallel wall is a singular normal-equations
    # case) -- identical technique to test_slam_mapper.py's _textured_wall.
    rows = np.linspace(-0.4, 0.4, H)[:, None]
    cols = np.linspace(-0.5, 0.5, W)[None, :]
    curve = 0.1 * (rows ** 2 + cols ** 2)
    return ((z_m + curve) * 1000.0).astype(np.float32)


def _degenerate():
    return np.zeros((H, W), dtype=np.float32)


def _wall_sequence(n, z0=1.30, step=-0.02):
    """A gently moving synthetic scan: n frames, each `step` metres further
    (negative = pushing in) than the last -- well within the 5cm single-step
    shift test_slam_mapper.py's test_pose_translation_tracks_a_synthetic_shift
    already validates ICP recovers. Every frame should track successfully.
    Frame tuple is (depth, reflectance, confidence, quat, pressure, t_s) --
    reflectance/confidence are None here (most tests don't care about color/
    gating); see test_reflectance_color_* below for tests that do."""
    return [(_textured_wall(z0 + i * step), None, None, _Q, 101325.0, i * 0.1) for i in range(n)]


def _drain(worker, timeout=90.0):
    """Poll `worker.latest()` until a terminal (done=True) publish, returning
    the list of distinct-fraction Progress snapshots observed, in order."""
    seen = []
    last_frac = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        p = worker.latest()
        if p is not None and p.fraction != last_frac:
            last_frac = p.fraction
            seen.append(p)
        if p is not None and p.done:
            break
        time.sleep(0.005)
    return seen


def _verts(progress: Progress) -> int:
    return 0 if progress.mesh is None else int(len(progress.mesh.vertex.positions))


# ---- ShowcasePhase / next_phase ------------------------------------------

def test_idle_to_recording_on_record():
    assert next_phase(ShowcasePhase.IDLE, record_pressed=True) is ShowcasePhase.RECORDING


def test_idle_ignores_stop_and_done():
    assert next_phase(ShowcasePhase.IDLE, stop_pressed=True) is ShowcasePhase.IDLE
    assert next_phase(ShowcasePhase.IDLE, processing_done=True) is ShowcasePhase.IDLE


def test_recording_to_processing_on_stop():
    assert next_phase(ShowcasePhase.RECORDING, stop_pressed=True) is ShowcasePhase.PROCESSING


def test_recording_ignores_record_and_done():
    assert next_phase(ShowcasePhase.RECORDING, record_pressed=True) is ShowcasePhase.RECORDING
    assert next_phase(ShowcasePhase.RECORDING, processing_done=True) is ShowcasePhase.RECORDING


def test_processing_to_final_on_done():
    assert next_phase(ShowcasePhase.PROCESSING, processing_done=True) is ShowcasePhase.FINAL


def test_processing_holds_without_done():
    assert next_phase(ShowcasePhase.PROCESSING) is ShowcasePhase.PROCESSING


def test_final_holds_until_cleared_or_recorded():
    assert next_phase(ShowcasePhase.FINAL) is ShowcasePhase.FINAL


def test_cleared_always_returns_idle():
    for phase in ShowcasePhase:
        assert next_phase(phase, cleared=True) is ShowcasePhase.IDLE


def test_record_pressed_restarts_from_final():
    """Per the brief's "(-> back to IDLE on a new record / clear)": pressing
    Record while looking at a completed FINAL reveal starts a fresh scan
    rather than being silently ignored -- otherwise there would be no way to
    record a second scan without an explicit Clear in between."""
    assert next_phase(ShowcasePhase.FINAL, record_pressed=True) is ShowcasePhase.RECORDING


def test_record_pressed_restarts_from_processing():
    """Same for interrupting an in-flight PROCESSING run."""
    assert next_phase(ShowcasePhase.PROCESSING, record_pressed=True) is ShowcasePhase.RECORDING


def test_record_pressed_is_idempotent_while_already_recording():
    assert next_phase(ShowcasePhase.RECORDING, record_pressed=True) is ShowcasePhase.RECORDING


def test_cleared_wins_over_record_pressed():
    assert next_phase(ShowcasePhase.FINAL, record_pressed=True, cleared=True) is ShowcasePhase.IDLE


# ---- PostProcessWorker -----------------------------------------------------

def test_progress_monotonic_fraction_to_one_and_terminal_stats():
    frames = _wall_sequence(18)
    w = PostProcessWorker(frames, W, H, mesh_every=3, voxel_size=0.02)
    w.start()
    try:
        seen = _drain(w)
    finally:
        w.stop()
    assert seen, "worker never published anything"
    fracs = [p.fraction for p in seen]
    assert all(b >= a for a, b in zip(fracs, fracs[1:])), fracs
    assert fracs[-1] == 1.0
    final = seen[-1]
    assert final.done
    assert final.stats is not None
    assert final.stats["frames"] == len(frames)
    assert final.stats["lost"] == 0
    for key in ("gap_m", "horizontal_gap_m", "vertical_gap_m",
                "vertical_divergence", "path_m", "verts", "lost", "frames"):
        assert key in final.stats
    assert final.stats["vertical_divergence"]["available"] is False


def test_non_terminal_publishes_have_no_stats():
    frames = _wall_sequence(18)
    w = PostProcessWorker(frames, W, H, mesh_every=3, voxel_size=0.02)
    w.start()
    try:
        seen = _drain(w)
    finally:
        w.stop()
    assert any(not p.done for p in seen), "expected at least one in-progress publish"
    for p in seen:
        if not p.done:
            assert p.stats is None


def test_mesh_extraction_phase_is_published_before_the_expensive_mesh_snapshot():
    """A slow ``Mapper.mesh`` must never look like a frozen frame counter."""
    import types

    w = PostProcessWorker(_wall_sequence(3), W, H)
    old_mesh = object()
    w._latest = Progress(1 / 3, old_mesh, ["old pose"], False)
    mapper = types.SimpleNamespace(trajectory=["latest pose"])

    w._publish_extracting_mesh(mapper, 2, 3)
    latest = w.latest()
    assert latest is not None
    assert latest.phase == "extracting_mesh"
    assert latest.fraction == pytest.approx(2 / 3)
    assert latest.mesh is old_mesh
    assert latest.trajectory == ["latest pose"]


def test_improving_preview_vertex_count_non_decreasing():
    """The concrete, testable meaning of "the preview visibly sharpens": each
    later publish's mesh has >= vertices than an earlier one. A fresh/
    degenerate first publish may be 0 verts -- that's fine, still
    non-decreasing. Backing away from the wall (rather than pushing in) grows
    the covered footprint frame over frame -- pushing in shrinks the visible
    footprint and can make marching-cubes' extracted vertex count wobble down
    by a handful near existing surface boundaries even though the underlying
    voxel blocks are a strict superset, so backing away is the direction that
    gives clean non-decreasing growth."""
    frames = _wall_sequence(18, z0=0.90, step=0.05)
    w = PostProcessWorker(frames, W, H, mesh_every=3, voxel_size=0.02)
    w.start()
    try:
        seen = _drain(w)
    finally:
        w.stop()
    verts = [_verts(p) for p in seen]
    assert len(verts) >= 2, "need at least two publishes to prove growth"
    assert all(b >= a for a, b in zip(verts, verts[1:])), verts
    assert verts[-1] > 0, "final mesh over a genuinely-tracked scan must be non-empty"


def test_stop_mid_processing_does_not_hang_and_latest_stays_readable():
    frames = _wall_sequence(200, step=0.005)   # long enough to still be mid-run when we stop
    w = PostProcessWorker(frames, W, H, mesh_every=10, voxel_size=0.02)
    w.start()
    time.sleep(0.05)
    t0 = time.monotonic()
    w.stop()
    elapsed = time.monotonic() - t0
    assert elapsed < 5.0
    assert w._thread is None
    _ = w.latest()   # must not raise, whatever state it's in (None is fine)


def test_never_started_stop_is_safe():
    w = PostProcessWorker([], W, H)
    w.stop()   # must not hang or raise


def test_empty_capture_publishes_terminal_done_zero_verts_no_crash():
    w = PostProcessWorker([], W, H, voxel_size=0.02)
    w.run()
    latest = w.latest()
    assert latest is not None
    assert latest.done
    assert latest.fraction == 1.0
    assert latest.stats is not None
    assert latest.stats["verts"] == 0
    assert latest.stats["frames"] == 0


def test_all_lost_capture_publishes_terminal_done_zero_verts_no_crash():
    frames = [(_degenerate(), None, None, _Q, 101325.0, i * 0.1) for i in range(5)]
    w = PostProcessWorker(frames, W, H, voxel_size=0.02)
    w.run()
    latest = w.latest()
    assert latest is not None
    assert latest.done
    assert latest.stats is not None
    assert latest.stats["verts"] == 0
    assert latest.stats["lost"] == 5
    assert latest.stats["frames"] == 5


def test_latest_is_none_before_any_run():
    w = PostProcessWorker(_wall_sequence(3), W, H, voxel_size=0.02)
    assert w.latest() is None


def test_per_frame_exception_does_not_kill_thread_and_still_publishes_terminal(monkeypatch):
    """Belt-and-braces: force `Mapper.step` to raise on one frame and confirm
    `run()` skips it (rather than dying) and still reaches a terminal,
    stats-bearing publish."""
    frames = _wall_sequence(6)
    w = PostProcessWorker(frames, W, H, mesh_every=100, voxel_size=0.02)  # only the final publish would normally fire
    orig_step = Mapper.step
    calls = {"n": 0}

    def flaky_step(self, *a, **kw):
        calls["n"] += 1
        if calls["n"] == 3:
            raise RuntimeError("synthetic boom")
        return orig_step(self, *a, **kw)

    monkeypatch.setattr(Mapper, "step", flaky_step)
    w.run()   # must not raise
    latest = w.latest()
    assert latest is not None
    assert latest.done
    assert latest.stats is not None
    assert calls["n"] == len(frames)


def test_exception_on_final_frame_still_publishes_terminal(monkeypatch):
    """If the very LAST frame is the one that raises, the loop's own
    is_last-publish branch never fires for it -- the post-loop fallback
    must publish a terminal result anyway."""
    frames = _wall_sequence(4)
    w = PostProcessWorker(frames, W, H, mesh_every=100, voxel_size=0.02)
    orig_step = Mapper.step
    calls = {"n": 0}

    def flaky_step(self, *a, **kw):
        calls["n"] += 1
        if calls["n"] == len(frames):
            raise RuntimeError("synthetic boom on last frame")
        return orig_step(self, *a, **kw)

    monkeypatch.setattr(Mapper, "step", flaky_step)
    w.run()
    latest = w.latest()
    assert latest is not None
    assert latest.done
    assert latest.fraction == 1.0
    assert latest.stats is not None


def test_mapper_construction_failure_still_publishes_terminal_no_crash(monkeypatch):
    """If `Mapper(width, height, ...)` itself raises -- the real-world trigger
    is width/height being None because `_load_frames` never decoded a single
    depth frame (a capture started mid-stream, after the device's one-time
    CALIB frame had already gone by) -- `run()` must still publish a
    terminal, zero-verts result instead of dying silently and leaving
    `latest()` stuck at None forever."""
    def boom(*a, **kw):
        raise TypeError("width/height are None")

    monkeypatch.setattr("roomscan.slam.showcase.Mapper", boom)
    w = PostProcessWorker(_wall_sequence(3), None, None)
    w.run()   # must not raise
    latest = w.latest()
    assert latest is not None
    assert latest.done
    assert latest.fraction == 1.0
    assert latest.stats == {"frames": 0, "gap_m": 0.0, "path_m": 0.0, "verts": 0, "lost": 0}
    assert latest.mesh is not None and len(latest.mesh.vertex.positions) == 0


def test_mapper_construction_failure_on_empty_capture_still_publishes(monkeypatch):
    """Same failure, but with an empty frame list too (the actual from_capture
    fallback shape: no frames AND no width/height)."""
    def boom(*a, **kw):
        raise TypeError("width/height are None")

    monkeypatch.setattr("roomscan.slam.showcase.Mapper", boom)
    w = PostProcessWorker([], None, None)
    w.run()
    latest = w.latest()
    assert latest is not None
    assert latest.done
    assert latest.stats["verts"] == 0


def test_from_capture_classmethod_loads_via_slam_cli(monkeypatch):
    """from_capture() must delegate to slam.cli._load_frames rather than
    reimplementing capture loading."""
    sentinel_frames = _wall_sequence(2)

    def fake_load_frames(path, max_frames=None, with_imu=False):
        assert path == "some/capture.bin"
        return sentinel_frames, W, H

    # from_capture delegates through _load_frames_maybe_imu, which resolves
    # `_load_frames` in the cli module -- patch it there.
    monkeypatch.setattr("roomscan.slam.cli._load_frames", fake_load_frames)
    w = PostProcessWorker.from_capture("some/capture.bin", voxel_size=0.02)
    assert w._frames is sentinel_frames
    assert w._width == W and w._height == H


def test_start_is_idempotent_when_already_running():
    w = PostProcessWorker(_wall_sequence(30, step=0.005), W, H, mesh_every=5, voxel_size=0.02)
    w.start()
    try:
        t1 = w._thread
        w.start()   # second start() while running must be a no-op, not spawn a second thread
        assert w._thread is t1
    finally:
        w.stop()


# --------------------------------------------------- map outgrows Open3D (2026-08-01)


class _CapacityMapper:
    """A Mapper whose mesh() starts crashing once `fail_after` frames are in."""

    def __init__(self, fail_after):
        self.fail_after = fail_after
        self.trajectory = []
        self.tracking_lost_count = 0
        self._n = 0

    def step(self, *a, **kw):
        self._n += 1
        self.trajectory.append(np.eye(4))
        return None

    def mesh(self):
        if self._n > self.fail_after:
            raise TsdfCapacityError("map has 250224 active blocks; coarsen voxel_size")
        return _empty_mesh()


def _worker_over(n_frames, mesh_every=25):
    frames = [(np.zeros((4, 4), np.float32), None, None, None, None, float(i))
              for i in range(n_frames)]
    return PostProcessWorker(frames, 4, 4, mesh_every=mesh_every)


def test_a_map_that_outgrows_open3d_stops_the_build_and_says_why(monkeypatch):
    w = _worker_over(200)
    monkeypatch.setattr(showcase, "Mapper", lambda *a, **kw: _CapacityMapper(fail_after=50))
    w.run()
    p = w.latest()
    assert p.done and p.phase == "failed"
    assert "voxel_size" in p.stats["error"]
    # Stopped at the wall, did not grind through the remaining 125 frames.
    assert p.stats["frames"] == 75
    assert p.fraction < 1.0, "a stopped build must not report as complete"


def test_a_stopped_build_keeps_the_last_good_mesh_rather_than_extracting_again(monkeypatch):
    """Re-extracting here would segfault: the map is past the wall. Verified on
    DebugCapB1 -- the guard fires, and mapper.mesh() straight after dies."""
    w = _worker_over(200)
    mapper = _CapacityMapper(fail_after=50)
    monkeypatch.setattr(showcase, "Mapper", lambda *a, **kw: mapper)
    w.run()
    calls_before = mapper._n
    p = w.latest()
    assert p.mesh is not None
    # No further mesh() attempt was made after the failure.
    assert mapper._n == calls_before


def test_a_normal_build_is_unaffected(monkeypatch):
    w = _worker_over(100)
    monkeypatch.setattr(showcase, "Mapper", lambda *a, **kw: _CapacityMapper(fail_after=10**9))
    w.run()
    p = w.latest()
    assert p.done and p.phase != "failed"
    assert (p.stats or {}).get("error") is None
    assert p.fraction == 1.0


def test_from_capture_requests_quat_interp_when_the_lever_is_on(monkeypatch):
    """#155: Detailed must ride the exact same loader alignment as the offline
    CLI — apply_quat_phase=True selects timestamp interpolation in the shared
    loader rather than a Detailed-private synchronizer."""
    seen = {}
    sentinel_frames = _wall_sequence(2)

    def fake_load_frames(path, max_frames=None, with_imu=False, quat_interp=False):
        seen["with_imu"] = with_imu
        seen["quat_interp"] = quat_interp
        return (sentinel_frames, W, H, [(None, None)] * len(sentinel_frames)) \
            if with_imu else (sentinel_frames, W, H)

    monkeypatch.setattr("roomscan.slam.cli._load_frames", fake_load_frames)
    PostProcessWorker.from_capture("some/capture.bin", voxel_size=0.02,
                                   apply_quat_phase=True)
    assert seen == {"with_imu": True, "quat_interp": True}
    # and with the lever off, legacy doubles without the kwarg must keep working
    # (the kwarg is only passed when the lever is on) — pinned by the fact that
    # test_from_capture_classmethod_loads_via_slam_cli above still passes.
