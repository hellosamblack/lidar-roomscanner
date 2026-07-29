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


def test_empty_map_extraction_does_not_count_as_an_extraction():
    # The empty-map guards return a placeholder without touching the grid --
    # nothing was allocated, so there is nothing to release.
    m = TsdfMap(voxel_size=0.02)
    seen = []
    m._release_cache_if_due = lambda: seen.append(1)   # type: ignore[method-assign]
    m.mesh()
    m.point_cloud()
    assert seen == []
