"""SlamWorker instrumentation: submit/processed/overwrite counters and the
mesh-extraction timer (plan item 2, 2026-08-02 SLAM compute/transport
follow-ups). Per that plan's own instruction -- "never trust a 0 from a
counter nothing writes" -- these tests drive an EXACT known number of
submits against a deliberately stalled worker (run_once never called) and
assert the exact overwrite count, not just "some overwrites happened"."""
import numpy as np
import pytest

pytest.importorskip("open3d")

from roomscan.slam.worker import SlamWorker

W, H = 54, 42


def _wall(z_m=1.0):
    return np.full((H, W), z_m * 1000.0, dtype=np.float32)


def _submit_n(worker, n):
    for _ in range(n):
        worker.submit(_wall(), (1.0, 0.0, 0.0, 0.0), 101325.0)


def test_stalled_worker_overwrite_count_is_exact():
    """N submits with run_once() NEVER called: the slot holds only the last
    one, so exactly N-1 were overwritten and 0 were processed. This is the
    genuinely missing piece the plan calls out -- SlamWorker.run_once() only
    ever pops ONE item regardless of how many submits happened first."""
    w = SlamWorker(W, H, voxel_size=0.02)
    _submit_n(w, 7)
    assert w.frames_submitted == 7
    assert w.frames_overwritten == 6
    assert w.frames_processed == 0


def test_processed_plus_overwritten_accounts_for_submitted():
    """Invariant: once every submitted frame has been either popped (processed)
    or replaced before being popped (overwritten), the two counters sum to
    exactly the submit count -- no frame is silently unaccounted for."""
    w = SlamWorker(W, H, voxel_size=0.02)
    _submit_n(w, 5)
    assert w.run_once() is True          # pops the 5th (last) submitted item
    assert w.frames_submitted == 5
    assert w.frames_processed == 1
    assert w.frames_overwritten == 4
    assert w.frames_processed + w.frames_overwritten == w.frames_submitted

    # Drive it again, interleaving processing with bursts of submits, and
    # re-check the invariant holds throughout rather than only once.
    _submit_n(w, 3)
    assert w.run_once() is True
    assert w.frames_submitted == 8
    assert w.frames_processed == 2
    assert w.frames_overwritten == 6
    assert w.frames_processed + w.frames_overwritten == w.frames_submitted

    # An empty slot: run_once() returns False and touches neither counter.
    assert w.run_once() is False
    assert w.frames_processed == 2
    assert w.frames_overwritten == 6


def test_run_once_on_empty_slot_does_not_move_any_counter():
    w = SlamWorker(W, H, voxel_size=0.02)
    assert w.run_once() is False
    assert w.frames_submitted == 0
    assert w.frames_processed == 0
    assert w.frames_overwritten == 0


def test_overwritten_frames_are_not_tracking_lost():
    """An overwritten input never reaches Mapper.step at all -- the plan is
    explicit that this must not be confused with tracking loss, which is a
    frame that DID reach the mapper and failed to register. Submit a burst
    that overwrites, then process only the last one (a good frame); tracking
    loss must stay 0 even though 6 submitted frames never got processed."""
    w = SlamWorker(W, H, voxel_size=0.02)
    _submit_n(w, 7)
    assert w.run_once() is True
    assert w.frames_overwritten == 6
    assert w.tracking_lost_count == 0     # not conflated with the mapper's own counter


def test_device_and_backend_reported_by_worker_not_inferred():
    w = SlamWorker(W, H, voxel_size=0.02, device="CPU:0")
    assert w.device == "CPU:0"          # read from the built Mapper, not re-guessed
    assert w.device == w._mapper.device
    assert w.backend == "local"


def test_mesh_extract_ms_is_zero_before_first_extraction_then_measured():
    w = SlamWorker(W, H, voxel_size=0.02, mesh_every=1)
    assert w.mesh_extract_ms == 0.0
    w.submit(_wall(), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert w.run_once() is True
    # First integrated frame always extracts once (mesh_every==1 or the
    # "== 1" special case), so the timer must have actually run.
    assert w.mesh_extract_ms >= 0.0
    assert isinstance(w.mesh_extract_ms, float)
    # Discriminate against a stub that always reports 0: a real Mapper.mesh()
    # call on even a tiny map takes measurable time on SOME runs, but timer
    # noise means we can't assert > 0 reliably -- instead assert the field
    # actually moved from its pre-extraction sentinel by monkeypatching the
    # clock-free call is unnecessary here since mesh() is real Open3D work;
    # assert it is finite and non-negative, which a units/wiring bug (e.g.
    # returning the WRONG timer, or never assigning it) would still likely
    # violate via a stale 0.0 -- covered by the next, stronger test.
    assert w.mesh_extract_ms == w.mesh_extract_ms  # not NaN


def test_run_once_applies_imu_rate_hz_to_the_mapper_under_the_worker_thread():
    """Task 8 step 2: baro_tau_frames must be updated by run_once() (the
    worker thread), never by submit() (the reader/producer thread)."""
    w = SlamWorker(W, H, voxel_size=0.02, baro_tau_frames=555)
    w.submit(_wall(), (1.0, 0.0, 0.0, 0.0), 101325.0, imu_rate_hz=90.0)
    assert w._mapper.baro_tau_frames == 555     # not yet applied -- still queued
    assert w.run_once() is True
    assert w._mapper.baro_tau_frames == 2700    # applied once the worker thread popped it


def test_run_once_without_imu_rate_hz_leaves_baro_tau_frames_untouched():
    """Backward compatible: an existing caller that never passes imu_rate_hz
    (default None) must see byte-identical behavior."""
    w = SlamWorker(W, H, voxel_size=0.02, baro_tau_frames=555)
    w.submit(_wall(), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert w.run_once() is True
    assert w._mapper.baro_tau_frames == 555


def test_mesh_extract_ms_reflects_a_slow_mesh_call(monkeypatch):
    """Strong version of the timer test: patch Mapper.mesh to sleep a known
    amount and assert the reported time is at least that long. A counter
    that always reads 0 (nothing ever assigns it) would fail this outright."""
    import time as _time
    w = SlamWorker(W, H, voxel_size=0.02, mesh_every=1)
    real_mesh = w._mapper.mesh

    def _slow_mesh():
        _time.sleep(0.05)
        return real_mesh()

    monkeypatch.setattr(w._mapper, "mesh", _slow_mesh)
    w.submit(_wall(), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert w.run_once() is True
    assert w.mesh_extract_ms >= 40.0    # measured >=0.05s sleep, generous floor
