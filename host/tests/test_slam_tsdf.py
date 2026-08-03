import pytest
import types

import numpy as np
import open3d as o3d
import roomscan.slam.tsdf as tsdf_mod
from roomscan.slam.tsdf import TsdfMap
from roomscan.slam.intrinsics import pinhole

W, H = 54, 42

def _wall_depth(z_m=1.0):
    # a flat wall at constant z; depth image in millimetres
    return np.full((H, W), z_m * 1000.0, dtype=np.float32)

def test_raycast_none_before_integrate():
    m = TsdfMap(voxel_size=0.02)
    K = pinhole(W, H)
    assert m.raycast(K, np.eye(4), W, H) is None

def test_integrate_then_raycast_recovers_wall():
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    depth = _wall_depth(1.0)
    m.integrate(depth, K, np.eye(4))               # identity pose: world==camera
    model = m.raycast(K, np.eye(4), W, H)
    assert model is not None
    pts = model.point.positions.numpy()
    assert len(pts) > 500
    # the wall sits near z=1.0 m in camera/world frame
    assert abs(np.median(pts[:, 2]) - 1.0) < m_voxel_tol()
    # normals point roughly back toward the camera (-z)
    nz = model.point.normals.numpy()[:, 2]
    assert np.median(nz) < -0.5

def m_voxel_tol():
    return 0.05  # within a few voxels of the true plane

def test_raycast_with_depth_hint_matches_full_raycast():
    # Task 9.5 Lever 1: raycast() bounded to the current-view frustum (via a
    # depth_hint) must return geometry equivalent to the original
    # all-active-blocks path, not merely "some points".
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    depth = _wall_depth(1.0)
    m.integrate(depth, K, np.eye(4))
    full = m.raycast(K, np.eye(4), W, H)
    bounded = m.raycast(K, np.eye(4), W, H, depth_hint=depth)
    assert full is not None and bounded is not None
    full_pts = full.point.positions.numpy()
    bounded_pts = bounded.point.positions.numpy()
    # same wall recovered from the bounded query
    assert abs(np.median(bounded_pts[:, 2]) - np.median(full_pts[:, 2])) < 1e-6
    assert abs(len(bounded_pts) - len(full_pts)) <= 2   # frustum == whole map here

def test_raycast_with_explicit_block_coords():
    # frustum_block_coords() + raycast(block_coords=...) is the lower-level
    # entry point Mapper uses when it wants to reuse computed coords.
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    depth = _wall_depth(1.0)
    m.integrate(depth, K, np.eye(4))
    coords = m.frustum_block_coords(depth, K, np.eye(4))
    assert coords.shape[0] > 0
    model = m.raycast(K, np.eye(4), W, H, block_coords=coords)
    assert model is not None
    pts = model.point.positions.numpy()
    assert len(pts) > 500
    assert abs(np.median(pts[:, 2]) - 1.0) < m_voxel_tol()

def test_mesh_and_point_cloud_on_empty_map_return_empty_not_raise():
    # Task 10 bugfix: extract_triangle_mesh()/extract_point_cloud() raise a
    # C++ HashMap error ("Input number of keys should > 0") when nothing has
    # ever been integrated. mesh()/point_cloud() must guard this and return
    # an empty geometry of the correct type instead.
    m = TsdfMap(voxel_size=0.02)
    mesh = m.mesh()
    assert isinstance(mesh, o3d.t.geometry.TriangleMesh)
    assert len(mesh.vertex.positions) == 0

    pc = m.point_cloud()
    assert isinstance(pc, o3d.t.geometry.PointCloud)
    assert pc.point.positions.numpy().shape[0] == 0

def test_raycast_empty_map_with_depth_hint_returns_none():
    # The empty-map guard must fire before any block-coord computation, even
    # when a depth_hint is supplied (Mapper may pass one before any
    # integration has happened, e.g. after a lost bootstrap frame).
    m = TsdfMap(voxel_size=0.02)
    K = pinhole(W, H)
    assert m.raycast(K, np.eye(4), W, H, depth_hint=_wall_depth(1.0)) is None


def _gradient_color():
    # (H, W, 3) float32 [0,1] gradient across columns -- varied, non-black.
    grad = (np.arange(W, dtype=np.float32) / (W - 1))
    c = np.repeat(grad[None, :, None], H, axis=0)
    return np.repeat(c, 3, axis=2).astype(np.float32)


def test_integrate_with_color_populates_non_black_varied_mesh_colors():
    # Task 13: the color-integrate overload populates the VBG's `color`
    # attribute so extract_triangle_mesh() returns real (non-black) vertex
    # colors instead of all-zero.
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    depth = _wall_depth(1.0)
    color = _gradient_color()
    for _ in range(4):   # a few integrations so weight_threshold=3 default keeps voxels
        m.integrate(depth, K, np.eye(4), color=color)
    mesh = m.mesh()
    colors = mesh.vertex.colors.numpy()
    assert len(colors) > 100
    assert colors.max() > 0.0                    # non-black
    assert (colors.max(axis=0) - colors.min(axis=0)).max() > 0.05   # varied, not flat


def test_integrate_without_color_keeps_black_mesh_colors():
    # Unchanged default behavior: omitting `color` uses the depth-only
    # overload, so vertex colors stay at their zero-initialized default.
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    depth = _wall_depth(1.0)
    for _ in range(4):
        m.integrate(depth, K, np.eye(4))
    mesh = m.mesh()
    colors = mesh.vertex.colors.numpy()
    assert len(colors) > 100
    assert np.allclose(colors, 0.0)


def test_weight_threshold_reduces_extracted_vertex_count():
    # A voxel seen only once (default single integration) is dropped by a
    # weight_threshold > 1 -- the mechanism Task 13 uses to drop
    # transient/noise voxels from the final extraction.
    K = pinhole(W, H)
    depth = _wall_depth(1.0)

    m_low = TsdfMap(voxel_size=0.02, depth_max=5.0, weight_threshold=0.0)
    m_low.integrate(depth, K, np.eye(4))
    n_low = len(m_low.mesh().vertex.positions)

    m_high = TsdfMap(voxel_size=0.02, depth_max=5.0, weight_threshold=3.0)
    m_high.integrate(depth, K, np.eye(4))   # only integrated once: weight==1 < 3
    n_high = len(m_high.mesh().vertex.positions)

    assert n_low > 0
    assert n_high < n_low


def test_weight_threshold_defaults_to_three_matching_prior_behavior():
    # Before Task 13, mesh()/point_cloud() called extract_*() with no
    # arguments, which defaults to Open3D's own weight_threshold=3.0 --
    # verify the new explicit constructor knob preserves that default.
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    assert m.weight_threshold == 3.0


def test_device_defaults_to_cpu_and_accepts_explicit_string_or_device():
    # Device-configurability (Phase 6 follow-up): default is unchanged
    # ("CPU:0"), and both a device string and an already-resolved
    # o3d.core.Device are accepted -- the two forms a caller (Mapper, CLI)
    # might pass through.
    assert TsdfMap(voxel_size=0.02)._device == o3d.core.Device("CPU:0")
    assert TsdfMap(voxel_size=0.02, device="CPU:0")._device == o3d.core.Device("CPU:0")
    assert TsdfMap(voxel_size=0.02, device=o3d.core.Device("CPU:0"))._device == o3d.core.Device("CPU:0")


def test_geometry_created_on_the_configured_device():
    # Every Open3D object TsdfMap creates (VBG-backed extractions, the
    # raycast point cloud, the empty-map placeholders) must live on
    # self._device, not a hard-coded CPU -- otherwise a future CUDA run
    # would silently produce CPU/CUDA-mismatched geometry.
    dev = o3d.core.Device("CPU:0")
    m = TsdfMap(voxel_size=0.02, depth_max=5.0, device=dev)
    K = pinhole(W, H, device=dev)
    depth = _wall_depth(1.0)

    # empty-map placeholders
    assert m.mesh().vertex.positions.device == dev
    assert m.point_cloud().point.positions.device == dev

    m.integrate(depth, K, np.eye(4))
    model = m.raycast(K, np.eye(4), W, H)
    assert model is not None
    assert model.point.positions.device == dev
    # This is the exact "GPU-safety" pattern used throughout the SLAM code
    # (a CUDA tensor must be moved home before .numpy()) -- .cpu() is a
    # documented no-op on an already-CPU tensor, so this is safe to run
    # unconditionally regardless of device.
    pts = model.point.positions.cpu().numpy()
    assert len(pts) > 500
    assert m.mesh().vertex.positions.device == dev


# --------------------------------------------------------------- sub-phase 6.G
# The long-scan OOM: on CUDA it is the throttled mesh()/point_cloud()
# extraction, not the per-frame path, that grows device memory (~5.1 MiB/frame
# measured), because `_extract_vbg()`'s whole-grid `.cpu()` copy leaves
# ever-larger temporaries in Open3D's caching allocator. These tests cover the
# cadence + wiring on a CPU box; the on-GPU ceiling assertion lives in
# tools/slam-container/cuda_smoke.py.


def _populated_map():
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    depth = _wall_depth(1.0)
    for _ in range(4):
        m.integrate(depth, K, np.eye(4))
    return m


def test_release_cache_every_defaults_to_one_and_clamps_negatives():
    assert TsdfMap(voxel_size=0.02).release_cache_every == 1
    assert TsdfMap(voxel_size=0.02, release_cache_every=0).release_cache_every == 0
    assert TsdfMap(voxel_size=0.02, release_cache_every=-5).release_cache_every == 0


def test_release_cache_is_a_noop_on_a_cpu_grid():
    # A CPU grid has no CUDA cache to release: extraction must still work and
    # the release must never fire (o3d.core.cuda.release_cache() on a
    # CUDA-less build is exactly what we must not call).
    m = _populated_map()
    assert len(m.mesh().vertex.positions) > 100
    # Point-cloud extraction can legitimately return 0 points on this map at
    # weight_threshold=3.0 -- what matters here is that it runs and releases
    # nothing, not how much geometry it finds.
    m.point_cloud()
    assert m.cache_releases == 0


def test_release_cache_fires_on_every_extraction_by_default(monkeypatch):
    # The cadence logic itself is device-independent, so stub out the two
    # device-specific pieces and assert it on any box.
    calls = []
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: True)
    monkeypatch.setattr(o3d.core.cuda, "release_cache", lambda: calls.append(1))
    m = TsdfMap(voxel_size=0.02, release_cache_every=1)
    for _ in range(3):
        m._release_cache_if_due()
    assert len(calls) == 3
    assert m.cache_releases == 3


def test_release_cache_throttles_to_every_nth_extraction(monkeypatch):
    calls = []
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: True)
    monkeypatch.setattr(o3d.core.cuda, "release_cache", lambda: calls.append(1))
    m = TsdfMap(voxel_size=0.02, release_cache_every=5)
    for _ in range(12):
        m._release_cache_if_due()
    assert len(calls) == 2            # extractions 5 and 10
    assert m.cache_releases == 2


def test_release_cache_every_zero_never_fires(monkeypatch):
    calls = []
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: True)
    monkeypatch.setattr(o3d.core.cuda, "release_cache", lambda: calls.append(1))
    m = TsdfMap(voxel_size=0.02, release_cache_every=0)
    for _ in range(10):
        m._release_cache_if_due()
    assert calls == []
    assert m.cache_releases == 0


def test_mesh_and_point_cloud_each_count_as_one_extraction():
    # Wiring check: both extraction entry points must go through the hook,
    # since both perform the `.cpu()` grid copy that dirties the cache.
    m = _populated_map()
    seen = []
    m._release_cache_if_due = lambda: seen.append(1)   # type: ignore[method-assign]
    m.mesh()
    m.point_cloud()
    assert len(seen) == 2


# ------------------------------------------------------------------- BUG-035
# Running a scan near its configured block_count stalls map growth and collapses
# frame-to-model tracking -- on the owner's room sweep, 560 of the last 646
# frames were lost at 40,000 vs 11 at 120,000. NOTE the grid *does* rehash to
# grow (test below pins that), so the warning is deliberately measured against
# the CONFIGURED capacity, not the live one.


def test_default_block_count_covers_a_full_room_sweep():
    # The owner's roomSweepFull20260730.bin needs 42,917 blocks at 1 cm voxels.
    # The old 40,000 default sat *below* real demand, which is what made the
    # failure so easy to hit -- keep a real margin over it.
    assert tsdf_mod.DEFAULT_BLOCK_COUNT >= 3 * 42917
    assert TsdfMap(voxel_size=0.02).block_count == tsdf_mod.DEFAULT_BLOCK_COUNT
    assert TsdfMap(voxel_size=0.02, block_count=1234).block_count == 1234


def test_block_usage_reports_active_blocks_and_LIVE_capacity():
    m = TsdfMap(voxel_size=0.02, depth_max=5.0, block_count=5000)
    used, cap = m.block_usage()
    assert (used, cap) == (0, 5000)
    m.integrate(_wall_depth(1.0), pinhole(W, H), np.eye(4))
    used, cap = m.block_usage()
    assert used > 0 and cap >= 5000


def test_the_grid_rehashes_to_grow_past_its_initial_block_count():
    """Pins the correction to BUG-035's original (wrong) explanation: the grid is
    NOT a hard cap. Measured on CUDA it rehashes 40,000 -> 80,000 at 99.2% load;
    this is the same behaviour at a size a CPU test can reach quickly. If this
    ever fails, the 'does not grow' story becomes true and the warning text in
    _check_saturation needs revisiting."""
    m = TsdfMap(voxel_size=0.02, depth_max=5.0, block_count=64)
    K = pinhole(W, H)
    for i in range(40):
        T = np.eye(4)
        T[0, 3] = 0.25 * i          # slide along the wall: new blocks every frame
        m.integrate(_wall_depth(1.5), K, np.linalg.inv(T))
    used, live_cap = m.block_usage()
    assert used > 64, "expected the map to grow past its initial block_count"
    assert live_cap > 64, f"hashmap should have rehashed; capacity still {live_cap}"


def test_saturation_warns_once_when_the_map_nears_capacity(monkeypatch, caplog):
    # A capacity small enough that one wall integration blows past 90%.
    # Cadence pinned to 1 so this test is about the warn-once behaviour, not
    # the polling stride (that's covered separately below).
    monkeypatch.setattr(tsdf_mod, "_SATURATION_CHECK_EVERY", 1)
    m = TsdfMap(voxel_size=0.02, depth_max=5.0, block_count=8)
    with caplog.at_level("WARNING"):
        m.integrate(_wall_depth(1.0), pinhole(W, H), np.eye(4))
        m.integrate(_wall_depth(1.0), pinhole(W, H), np.eye(4))
    hits = [r for r in caplog.records if "block" in r.getMessage()]
    assert len(hits) == 1, "must warn exactly once, not once per frame"
    assert "block_count" in hits[0].getMessage()
    # The message must not resurrect the disproven "cannot grow" claim.
    assert "does not grow" not in hits[0].getMessage()


def test_no_saturation_warning_with_headroom(monkeypatch, caplog):
    monkeypatch.setattr(tsdf_mod, "_SATURATION_CHECK_EVERY", 1)
    m = TsdfMap(voxel_size=0.02, depth_max=5.0, block_count=40000)
    with caplog.at_level("WARNING"):
        m.integrate(_wall_depth(1.0), pinhole(W, H), np.eye(4))
    assert not [r for r in caplog.records if "block_count" in r.getMessage()]


# ------------------------------------------------------- item 7 (2026-08-02)
# _check_saturation() now polls the hashmap size on a _SATURATION_CHECK_EVERY
# stride instead of every integrate() (BUG-035's warning cost was measured at
# ~5.8us/call, ~0.02s/sweep -- real but not worth paying every call). These
# pin: the poll actually happens on that cadence and not more often, a
# threshold crossed BETWEEN polls is still caught (late, not missed) on the
# very next poll, and the ceiling/capacity-error machinery below is untouched.


class _CountingHashmap:
    """Stands in for `self._vbg.hashmap()`: counts `.size()` calls and reports
    whatever `size_fn()` says the true block count is *right now* -- letting a
    test grow the "real" map every simulated integrate while only sampling it
    on the check's own stride, exactly like the CUDA hashmap being polled less
    often than the grid actually grows."""

    def __init__(self, size_fn):
        self._size_fn = size_fn
        self.query_count = 0

    def size(self):
        self.query_count += 1
        return self._size_fn()


def _fake_map(block_count, size_fn):
    m = TsdfMap(voxel_size=0.02, block_count=block_count)
    hm = _CountingHashmap(size_fn)
    m._vbg = types.SimpleNamespace(hashmap=lambda: hm)
    return m, hm


def test_check_saturation_polls_the_hashmap_only_every_nth_call():
    # Far from the 90% threshold the whole time, so only the polling cadence
    # is under test here, not the warning.
    m, hm = _fake_map(block_count=1_000_000, size_fn=lambda: 10)
    n_calls = 3 * tsdf_mod._SATURATION_CHECK_EVERY + 4
    for _ in range(n_calls):
        m._check_saturation()
    assert hm.query_count == n_calls // tsdf_mod._SATURATION_CHECK_EVERY


def test_check_saturation_does_not_poll_at_all_before_the_first_stride():
    m, hm = _fake_map(block_count=1_000_000, size_fn=lambda: 999_999)
    for _ in range(tsdf_mod._SATURATION_CHECK_EVERY - 1):
        m._check_saturation()
    assert hm.query_count == 0


def test_saturation_crossing_between_polls_still_warns_but_up_to_a_stride_late(caplog):
    # The map's TRUE size grows every simulated integrate (3 blocks/call --
    # driven by the test loop, independent of whether the check samples it),
    # so it crosses the 90-block threshold (block_count=100) at call 30 --
    # NOT a multiple of the 25-call stride. The check only samples the
    # hashmap at calls 25 and 50, so the warning must NOT fire at the true
    # crossing (30) but MUST fire by the next poll (50): a stride delays the
    # warning, it does not lose it.
    true_size = {"n": 0}
    m, hm = _fake_map(block_count=100, size_fn=lambda: true_size["n"])

    with caplog.at_level("WARNING"):
        for i in range(1, 51):
            true_size["n"] = i * 3   # the map's real growth, whether polled or not
            m._check_saturation()
            hit = any("block_count" in r.getMessage() for r in caplog.records)
            if i < 50:
                assert not hit, f"warned early at call {i} (true crossing is call 30)"
    hits = [r for r in caplog.records if "block_count" in r.getMessage()]
    assert len(hits) == 1, "must still warn exactly once after the delayed poll"
    assert hm.query_count == 2, "polled at calls 25 and 50, not every call"


def test_saturation_check_every_matches_the_headroom_check_cadence():
    # Not load-bearing that these two constants be EQUAL, but they were
    # deliberately chosen to match (same reasoning: a device-sync hashmap
    # read, cheap enough to pay every 25 integrates). Pin the value so a
    # change to either doesn't silently drift from the other, and so this
    # constant can't quietly regress to "every call" (1) without a test
    # noticing something changed.
    assert tsdf_mod._SATURATION_CHECK_EVERY == tsdf_mod._HEADROOM_CHECK_EVERY == 25


def test_saturation_check_cadence_is_independent_of_the_headroom_check():
    # _check_rehash_headroom (CUDA-only, its own counter) must not be
    # perturbed by _check_saturation's cadence bookkeeping, or vice versa.
    m, _ = _fake_map(block_count=1_000_000, size_fn=lambda: 10)
    for _ in range(tsdf_mod._SATURATION_CHECK_EVERY):
        m._check_saturation()
    assert m._integrates_since_headroom_check == 0
    assert m._integrates_since_saturation_check == 0


def test_empty_map_extraction_does_not_count_as_an_extraction():
    # The empty-map guards return a placeholder without touching the grid --
    # nothing was allocated, so there is nothing to release.
    m = TsdfMap(voxel_size=0.02)
    seen = []
    m._release_cache_if_due = lambda: seen.append(1)   # type: ignore[method-assign]
    m.mesh()
    m.point_cloud()
    assert seen == []


# ------------------------------------------- the Detailed-build lockup (2026-08-01)
# `_extract_vbg()` used to take `self._vbg.cpu()` on EVERY extraction: a copy of
# the whole PREALLOCATED grid, so its cost tracked `block_count` rather than how
# much map there was. Measured at the Detailed preset (block_count 320,000) that
# copy was 1.11 s against 0.04 s to extract in place, it did not shrink for a
# small map, and Open3D holds the GIL throughout -- which starved roomscan-web's
# asyncio loop for 78-84% of wall clock and froze the progress bar and the 3D
# view. These pin the decision, not the timing: the fast path is taken when
# there is measured NVML headroom, and every escape hatch still returns a
# host-resident result.


class _FakeNvml:
    """Stands in for gpumem.Nvml with a settable free-byte reading."""

    def __init__(self, ok=True, free=0):
        self.ok, self._free = ok, free
        self.queries = 0

    def free_bytes(self):
        self.queries += 1
        return self._free


def _cuda_map(monkeypatch, nvml, blocks=1000):
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: True)
    m = TsdfMap(voxel_size=0.02)
    m._nvml = nvml
    # The real hashmap lives behind a C++ call; only its size matters here.
    m._vbg = types.SimpleNamespace(
        hashmap=lambda: types.SimpleNamespace(size=lambda: blocks),
        cpu=lambda: "HOST-GRID")
    return m


def test_extraction_stays_on_the_gpu_when_nvml_reports_headroom(monkeypatch):
    nvml = _FakeNvml(free=1000 * tsdf_mod._CUDA_EXTRACT_BYTES_PER_BLOCK)
    m = _cuda_map(monkeypatch, nvml, blocks=1000)
    assert m._extract_vbg() is m._vbg, "the whole-grid host copy is the slow path"
    assert nvml.queries == 1


def test_extraction_falls_back_to_the_host_grid_when_vram_is_tight(monkeypatch):
    # One byte short of the measured requirement is still short.
    nvml = _FakeNvml(free=1000 * tsdf_mod._CUDA_EXTRACT_BYTES_PER_BLOCK - 1)
    m = _cuda_map(monkeypatch, nvml, blocks=1000)
    assert m._extract_vbg() == "HOST-GRID"


def test_extraction_falls_back_when_nvml_is_unavailable(monkeypatch):
    # No free-memory number => nothing authorizes the fast path. A box without
    # libnvidia-ml must keep the slow-but-correct behaviour, not guess.
    m = _cuda_map(monkeypatch, _FakeNvml(ok=False, free=1 << 60), blocks=1000)
    assert m._extract_vbg() == "HOST-GRID"


def test_cpu_grid_never_takes_a_copy_of_itself(monkeypatch):
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: False)
    m = TsdfMap(voxel_size=0.02)
    assert m._extract_vbg() is m._vbg


def test_gpu_extraction_returns_a_host_resident_result(monkeypatch):
    """The host copy used to make this true by construction. Extracting on the
    device does not, so `_extract` must download the geometry -- 0.10 s for a
    4.09M-vertex mesh, against the 1.11 s grid copy it replaces. Callers
    (MeshPrep, DetailedRunner._commit, the CLI writers) all assume host."""
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: True)
    m = TsdfMap(voxel_size=0.02)
    m._nvml = _FakeNvml(free=1 << 60)
    downloaded = []

    class _Result:
        def cpu(self):
            downloaded.append(1)
            return "HOST-MESH"

    m._vbg = types.SimpleNamespace(
        hashmap=lambda: types.SimpleNamespace(size=lambda: 10),
        extract_triangle_mesh=lambda w: _Result(),
        cpu=lambda: None)
    assert m._extract("extract_triangle_mesh") == "HOST-MESH"
    assert downloaded == [1]


def test_a_gpu_extraction_oom_latches_the_host_path_for_good(monkeypatch):
    """After a CUDA OOM Open3D's cached allocator can be left inconsistent
    enough to terminate() the process later, so one failure retires the device
    path for this map rather than being retried every 25 frames."""
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: True)
    monkeypatch.setattr(o3d.core.cuda, "release_cache", lambda: None)
    m = TsdfMap(voxel_size=0.02)
    m._nvml = _FakeNvml(free=1 << 60)          # headroom check says "go"
    host_calls = []

    def _boom(_w):
        raise RuntimeError("CUDA runtime error: out of memory")

    host_grid = types.SimpleNamespace(
        extract_triangle_mesh=lambda w: host_calls.append(1) or "HOST-MESH")
    m._vbg = types.SimpleNamespace(
        hashmap=lambda: types.SimpleNamespace(size=lambda: 10),
        extract_triangle_mesh=_boom,
        cpu=lambda: host_grid)

    assert m._extract("extract_triangle_mesh") == "HOST-MESH"
    assert "out of memory" in m._host_extract_reason
    # Latched: the second call must not touch the device extractor at all.
    assert m._extract_vbg() is host_grid
    assert m._extract("extract_triangle_mesh") == "HOST-MESH"
    assert len(host_calls) == 2


# ------------------------------------ rehash headroom / TsdfCapacityError (2026-08-01)
# A saturated grid grows by allocating a NEW buffer of twice the capacity beside
# the live one. When that will not fit, the failure is NOT survivable: Open3D
# raises, but unwinding leaves its cached allocator inconsistent and the process
# dies later in a destructor (`terminate called`). Measured on DebugCapB1 at the
# Detailed preset: the map fills around frame 3100 of 4808 and takes the whole
# roomscan-web server with it. So detect it just BEFORE, and raise something a
# caller can actually catch.


def _cuda_map_at_capacity(monkeypatch, nvml, used, capacity):
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: True)
    m = TsdfMap(voxel_size=0.02, block_count=capacity)
    m._nvml = nvml
    m.block_usage = lambda: (used, capacity)          # type: ignore[method-assign]
    m._integrates_since_headroom_check = tsdf_mod._HEADROOM_CHECK_EVERY - 1
    return m


def test_capacity_error_when_a_full_map_cannot_afford_its_next_rehash(monkeypatch):
    cap = 320000
    m = _cuda_map_at_capacity(monkeypatch, _FakeNvml(free=1), used=cap, capacity=cap)
    with pytest.raises(tsdf_mod.TsdfCapacityError) as exc:
        m._check_rehash_headroom()
    msg = str(exc.value)
    # Actionable: the two things that actually fix it, and the dominant one first.
    assert "voxel_size" in msg and "CPU:0" in msg
    assert "320000" in msg


def test_no_capacity_error_while_the_rehash_still_fits(monkeypatch):
    cap = 320000
    m = TsdfMap(voxel_size=0.02, block_count=cap)
    need = 2 * m._block_buffer_bytes(cap)
    m2 = _cuda_map_at_capacity(monkeypatch, _FakeNvml(free=need), used=cap, capacity=cap)
    m2._check_rehash_headroom()          # exactly enough is enough


def test_no_capacity_error_with_capacity_to_spare(monkeypatch):
    cap = 320000
    m = _cuda_map_at_capacity(monkeypatch, _FakeNvml(free=1),
                              used=int(0.5 * cap), capacity=cap)
    m._check_rehash_headroom()           # half full: nothing is about to rehash


def test_capacity_check_is_a_noop_without_nvml_or_on_cpu(monkeypatch):
    cap = 320000
    m = _cuda_map_at_capacity(monkeypatch, _FakeNvml(ok=False, free=0),
                              used=cap, capacity=cap)
    m._check_rehash_headroom()           # no free-memory number => no verdict
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: False)
    cpu = TsdfMap(voxel_size=0.02, block_count=cap)
    cpu._nvml = _FakeNvml(free=0)
    cpu.block_usage = lambda: (cap, cap)  # type: ignore[method-assign]
    cpu._check_rehash_headroom()         # system RAM, no CUDA allocator to wreck


def test_block_buffer_bytes_matches_the_grids_declared_attributes():
    """tsdf f32 + weight f32 + 3-channel color f32, over block_resolution**3
    voxels. 320,000 blocks at the default resolution 8 is the 3.05 GiB that does
    not leave room to double on an 8 GiB card."""
    m = TsdfMap(voxel_size=0.02, block_resolution=8)
    assert m._block_buffer_bytes(1) == 8 ** 3 * 20
    assert m._block_buffer_bytes(320000) / 2 ** 30 == pytest.approx(3.05, abs=0.01)


def test_extraction_is_refused_above_the_measured_crash_threshold(monkeypatch):
    """Above ~250k active blocks Open3D's marching cubes does not fail, it kills
    the process (host: segfault; CUDA: illegal memory access -> terminate), so
    there is nothing to catch downstream and the call must not be made."""
    m = TsdfMap(voxel_size=0.02)
    m._empty = False
    m._vbg = types.SimpleNamespace(
        hashmap=lambda: types.SimpleNamespace(
            size=lambda: tsdf_mod._MAX_SAFE_EXTRACT_BLOCKS + 1))
    with pytest.raises(tsdf_mod.TsdfCapacityError) as exc:
        m.mesh()
    msg = str(exc.value)
    assert "voxel_size" in msg
    # Raising block_count is the intuitive move and it is WRONG -- the same
    # block count crashed at 68.4% load in a 400k grid. Say so.
    assert "does NOT help" in msg
    with pytest.raises(tsdf_mod.TsdfCapacityError):
        m.point_cloud()


def test_extraction_is_allowed_right_up_to_the_threshold(monkeypatch):
    monkeypatch.setattr(tsdf_mod, "_is_cuda", lambda dev: False)
    m = TsdfMap(voxel_size=0.02)
    m._empty = False
    m._vbg = types.SimpleNamespace(
        hashmap=lambda: types.SimpleNamespace(
            size=lambda: tsdf_mod._MAX_SAFE_EXTRACT_BLOCKS),
        extract_triangle_mesh=lambda w: "MESH")
    assert m.mesh() == "MESH"


def test_the_safe_threshold_sits_below_the_last_measured_good_extraction():
    """258,161 blocks extracted cleanly; 273,521 crashed. The constant must stay
    under the good one -- if someone re-bisects on a new Open3D, this is the
    assertion that should make them think."""
    assert tsdf_mod._MAX_SAFE_EXTRACT_BLOCKS <= 258161
