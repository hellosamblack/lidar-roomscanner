"""TSDF map: a thin wrapper over Open3D's tensor VoxelBlockGrid for frame-to-model
SLAM. All poses are 4x4 world<-camera; integrate/raycast take the world->camera
`extrinsic` = inv(pose). Runs on whichever `device` (str or o3d.core.Device,
default "CPU:0") it's constructed with -- CPU-only today because the
installed Open3D 0.19 build has no CUDA support here, but every tensor this
class creates lives on `self._device`, so a CUDA-enabled build would run
unchanged with `device="CUDA:0"`. `raycast()`'s `.numpy()` pulls (`vertex`/
`normal`/`depth`) go through `.cpu()` first since those tensors live on the
compute device.

Open3D 0.19 API notes (verified against the installed build -- see
.superpowers/sdd/task-4-report.md for the full trace):
  - `ray_cast(...)`'s render_attributes keys ARE `vertex`/`normal`/`depth` as the
    task brief expected; results come back as an `o3d.t.geometry.TensorMap`
    (dict-like via `result['vertex']`, not `.keys()`).
  - `ray_cast`'s default `range_map_down_factor=8` hangs/burns CPU indefinitely
    on our small (~54x42) depth images (repeated "Could not generate full range
    map" reallocation that never converges). MUST pass `range_map_down_factor=1`
    for images this small.
  - `ray_cast`'s returned `normal` points *away* from the camera (+z into the
    surface for a fronto-parallel wall), the opposite of the usual "outward
    toward the sensor" convention -- confirmed empirically (median normal.z was
    exactly +1.0 for a wall at z=+1 viewed from the origin). We negate it so
    callers get camera-facing normals as point-to-plane ICP conventionally
    expects.
  - `extract_point_cloud()` / `extract_triangle_mesh()` raise a C++
    `SetPointColors` shape-mismatch error if `attr_names` omits `color` --
    unconditionally, even with 0 points requested. We declare a `color`
    attribute so those two methods are always usable; `integrate()` now has
    an optional `color` overload (Task 13) that populates it with a
    reflectance-derived image, but still defaults to the depth-only overload
    when no color is given.
  - `integrate()`'s color overload (`VoxelBlockGrid.integrate(block_coords,
    depth, color, intrinsic, extrinsic, depth_scale, depth_max,
    trunc_voxel_multiplier)`, verified against the installed 0.19 build's
    `help()`) requires `depth`/`color` to be the SAME dtype pairing: either
    both float32, or depth uint16 + color uint8 -- (float32, uint8) raises
    "Unsupported input data type combination" from the C++ kernel. Since our
    depth image is already float32 (millimetres), the color image must also
    be float32, with values in [0, 1] (verified empirically: a float32 [0,1]
    gradient image round-trips through `extract_triangle_mesh()` unchanged,
    e.g. an input of 0.81 comes back as vertex color 0.81).
  - `extract_triangle_mesh()`/`extract_point_cloud()` both accept a
    `weight_threshold: float = 3.0` first argument (verified via `help()`) --
    voxels integrated fewer than this many times are dropped from the
    extraction. 3.0 was already Open3D's own default (we previously called
    both with no arguments), so exposing it as a `TsdfMap` constructor knob
    with the same 3.0 default changes nothing unless a caller raises it.

Task 9.5 perf note: `raycast()`'s cost is dominated by how many voxel blocks
it visits. Passing `hashmap().active_buf_indices()` (ALL blocks ever
integrated) makes cost scale with total map size, not the current view -- the
`t_reconstruction_system/ray_casting.py` example instead bounds this to
`vbg.compute_unique_block_coordinates(depth, intrinsic, extrinsic,
depth_scale, depth_max)`, the same frustum-bounded set `integrate()` already
uses. `raycast()` now accepts an optional `block_coords` (or a `depth_hint`
to derive them) so `Mapper` can pass the current frame's frustum; omitting it
keeps the original all-active-blocks behavior for callers/tests that don't
have a depth hint handy.

Sub-phase 6.G note: on CUDA the throttled `mesh()`/`point_cloud()` extraction
-- not the per-frame path -- is what grows device memory over a long scan, and
`release_cache_every` bounds it. See `_release_cache_if_due` for the measured
numbers and the mechanism.
"""
from __future__ import annotations

import logging

import numpy as np
import open3d as o3d

from . import gpumem as _gpumem


def _resolve_device(device) -> o3d.core.Device:
    return device if isinstance(device, o3d.core.Device) else o3d.core.Device(device)


def _is_cuda(device: o3d.core.Device) -> bool:
    return "CUDA" in str(device).upper()


# BUG-035. Running a scan near its configured `block_count` stalls map growth
# and collapses frame-to-model tracking. The EFFECT is solid and reproducible;
# the mechanism is not fully pinned down -- see below, and do not repeat the
# wrong explanation.
#
# Measured on the owner's full room sweep (captures/roomSweepFull20260730.bin,
# 3525 depth frames, 1 cm voxels): the scan needs 42,917 blocks -- just 7% over
# the old 40,000 default. At 40,000 the block count froze at 38,937 (97.3%) on
# frame 2879 and never moved again; tracking-lost began 30 frames later and took
# 560 of the remaining 646 frames (median ICP fitness 0.887 -> 0.127, plus
# `stdgpu::vector::size: Size out of bounds: -2 ... Clamping to 0` from Open3D).
# At 120,000 and at 160,000 the same capture peaked at 42,917 and lost 11.
#
# What is NOT the mechanism: "the grid cannot grow". It can, and does -- driving
# a CUDA grid across the boundary shows a clean rehash 40,000 -> 80,000 at 99.2%
# load, continuing to 52,027 blocks with no errors. An earlier version of this
# comment (and of BUGS.md) asserted the opposite; it was wrong.
#
# Best current hypothesis, UNPROVEN: insertion failures in the ~97-99% load band
# below the rehash trigger, which the stdgpu underflow message is consistent
# with. The alternative -- that tracking degraded first and the frozen block
# count is a symptom rather than a cause -- is not excluded by the frame
# ordering alone, though it does not explain why 3x the capacity fixes it.
# Either way the mitigation is the same and is validated: give the scan enough
# initial capacity that it never runs near the limit.
#
# 160,000 is ~3.7x that scan, costing ~2.3 GiB of pre-allocated device memory
# (measured 1707 MiB at 120,000, i.e. ~14.2 KiB/block). That fits alongside the
# ~520 MiB steady-state working set on an 8 GiB card. Rooms bigger than this,
# or finer voxels, want `[slam] block_count` raised -- and on a CPU grid it can
# go far higher, since system RAM is the only limit (see the class docstring).
DEFAULT_BLOCK_COUNT = 160000

# Warn once the map crosses this fraction of capacity. The failure above was
# invisible until the trajectory was already ruined; ~30 frames of warning at
# 90% is enough to abort a scan rather than discover it in post.
_SATURATION_WARN_FRAC = 0.90

# Open3D rehashes at ~99% load (measured: 40,000 -> 80,000 at 99.2%). Check for
# room to do that from 95%, and only every Nth integrate -- the hashmap size read
# is a device sync, and at the ~29 blocks/frame this capture grows at, 95% of a
# 320,000-block grid is still hundreds of frames of warning.
_REHASH_IMMINENT_FRAC = 0.95
_HEADROOM_CHECK_EVERY = 25

# Item 7 of docs/superpowers/plans/2026-08-02-slam-compute-and-transport-followups.md
# (BUG-035 follow-up). BUG-035 measured the hashmap-size read
# _check_saturation() does on every integrate() at ~5.8us/call, ~0.02s over a
# full sweep -- not a leading cost -- but there's no reason to pay even that
# every call when _check_rehash_headroom right below already polls on a
# 25-integrate stride. Same cadence here, for the same reason.
#
# This DELAYS the warning by up to _SATURATION_CHECK_EVERY - 1 integrates past
# the moment the map actually crosses 90%: the map only grows (never shrinks),
# so a stride can never cause a MISSED warning, only a late one, worst case 24
# integrates late. BUG-035's budget -- ~30 frames of headroom at 90% before
# tracking collapsed -- comfortably absorbs that. State this rather than
# assuming the stride is free.
_SATURATION_CHECK_EVERY = 25


# Largest active-block count Open3D's marching cubes survives, on EITHER device.
#
# Bisected on captures/DebugCapB1.bin (voxel 0.005), extracting once at a given
# frame with no other extractions in flight:
#
#     blocks    load   host extract         CUDA extract
#     240,061   75.0%  OK (4.10M verts)     --
#     258,161   80.7%  OK (4.39M verts)     --
#     273,521   85.5%  SEGFAULT             illegal memory access -> terminate
#     285,411   89.2%  SEGFAULT             illegal memory access -> terminate
#     298,173   93.2%  SEGFAULT             --
#
# The trigger is the ABSOLUTE block count, not the load fraction and not free
# memory: re-running the 273,521-block case in a 400,000-block grid -- 68.4%
# load, 3.7 GiB free -- dies identically on both devices. So RAISING
# `block_count` DOES NOT HELP; only producing fewer blocks does (a coarser
# `voxel_size`, which is quadratic in block count for surface-dominated maps).
#
# Neither failure is catchable: the host path segfaults outright, and the CUDA
# path raises a RuntimeError whose unwind then aborts in a destructor. A build
# that walks into this takes the whole roomscan-web server with it, which is why
# extraction is refused BEFORE the call rather than guarded around it.
#
# 250,000 sits below the last measured-good 258,161. Treat it as a property of
# the installed Open3D build (0.19): re-bisect if that changes.
_MAX_SAFE_EXTRACT_BLOCKS = 250000


class TsdfCapacityError(RuntimeError):
    """The map has outgrown what this Open3D build can do with it -- extracting
    or growing it would abort the process rather than raise. Carries the
    remedy in its message. See `_check_extractable` / `_check_rehash_headroom`."""


# Open3D's VoxelBlockGrid takes the camera intrinsic/extrinsic as CPU:0 Float64
# tensors REGARDLESS of the grid's own device -- integrate/ray_cast/
# compute_unique_block_coordinates internally call InverseTransformation, which
# asserts CPU:0. On a CPU grid self._device is already CPU:0 so it never
# mattered; on CUDA, passing a CUDA extrinsic raises "Tensor has device CUDA:0,
# but is expected to have CPU:0". Keep these two tensors on CPU always; the
# depth/color images and raycast outputs stay on the compute device.
_CPU = o3d.core.Device("CPU:0")

# Device bytes one GPU-side extraction needs, per ACTIVE block, INCLUDING the
# mesh it produces. Measured on an RTX 2000 Ada (8 GiB) by sampling NVML either
# side of `extract_triangle_mesh` on a growing map (captures/DebugCapB1.bin,
# voxel 0.01, block_count 200,000): the peak sat between 8.0 and 9.5 KiB/block
# from 4.6k blocks (104k verts) to 139k blocks (4.09M verts) -- flat, not
# superlinear. 20 KiB is ~2x the worst of that, so a transient the sweep did
# not cover degrades to the host path instead of OOMing the card.
_CUDA_EXTRACT_BYTES_PER_BLOCK = 20 * 1024


class TsdfMap:
    def __init__(self, voxel_size: float = 0.01, trunc_multiplier: float = 8.0,
                 block_resolution: int = 8, block_count: int = DEFAULT_BLOCK_COUNT,
                 depth_scale: float = 1000.0, depth_max: float = 5.0,
                 weight_threshold: float = 3.0,
                 device: str | o3d.core.Device = "CPU:0",
                 release_cache_every: int = 1):
        self.voxel_size = voxel_size
        self.trunc_multiplier = trunc_multiplier
        self.depth_scale = depth_scale
        self.depth_max = depth_max
        self.weight_threshold = weight_threshold
        self.block_count = int(block_count)
        self._saturation_warned = False
        self._device = _resolve_device(device)
        # Sub-phase 6.G: release Open3D's CUDA cache every N extractions
        # (0 disables). See _release_cache_if_due for why extractions, not
        # frames, are the right cadence.
        self.release_cache_every = max(0, int(release_cache_every))
        self._extractions = 0
        self.cache_releases = 0
        self._empty = True
        # Lazily-opened NVML handle + the latch that retires the GPU extraction
        # path for this map. See `_extract_vbg` / `_extract`.
        self._nvml = None
        self._host_extract_reason: str | None = None
        self.block_resolution = int(block_resolution)
        self._integrates_since_headroom_check = 0
        self._integrates_since_saturation_check = 0
        self._vbg = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("tsdf", "weight", "color"),
            attr_dtypes=(o3d.core.float32, o3d.core.float32, o3d.core.float32),
            attr_channels=((1,), (1,), (3,)),
            voxel_size=voxel_size,
            block_resolution=block_resolution,
            block_count=block_count,
            device=self._device,
        )

    def _depth_image(self, depth_mm: np.ndarray) -> o3d.t.geometry.Image:
        d = np.ascontiguousarray(depth_mm, dtype=np.float32)
        return o3d.t.geometry.Image(o3d.core.Tensor(d, device=self._device))

    def _color_image(self, color: np.ndarray) -> o3d.t.geometry.Image:
        # Must be float32 in [0,1] to pair with our float32 depth image --
        # see the module docstring's "color overload" note.
        c = np.ascontiguousarray(color, dtype=np.float32)
        return o3d.t.geometry.Image(o3d.core.Tensor(c, device=self._device))

    def integrate(self, depth_mm: np.ndarray, intrinsic: o3d.core.Tensor,
                  extrinsic: np.ndarray, color: np.ndarray | None = None) -> None:
        """`color`, if given, is an (h, w, 3) float32 array in [0, 1] (e.g. a
        reflectance-derived grayscale image) integrated via the VBG's
        color-integrate overload, populating the `color` voxel attribute so
        `mesh()`/`point_cloud()` return non-black vertex colors. Omitting it
        (the default) keeps the original depth-only overload -- unchanged
        behavior for callers that don't have a color image handy."""
        depth = self._depth_image(depth_mm)
        ext = o3d.core.Tensor(np.asarray(extrinsic, dtype=np.float64), device=_CPU)
        intr = intrinsic.to(_CPU)
        coords = self._vbg.compute_unique_block_coordinates(
            depth, intr, ext, self.depth_scale, self.depth_max, self.trunc_multiplier)
        if color is not None:
            color_img = self._color_image(color)
            self._vbg.integrate(coords, depth, color_img, intr, ext,
                                self.depth_scale, self.depth_max, self.trunc_multiplier)
        else:
            self._vbg.integrate(coords, depth, intr, ext,
                                self.depth_scale, self.depth_max, self.trunc_multiplier)
        self._empty = False
        self._check_saturation()
        self._check_rehash_headroom()

    def block_usage(self) -> tuple[int, int]:
        """(active blocks, LIVE hashmap capacity). Two cheap hashmap reads.

        Reports the hashmap's *current* capacity rather than the `block_count`
        it was constructed with: Open3D does rehash (measured, on CUDA:
        40,000 -> 80,000 at 99.2% load), so the constructed value goes stale and
        would misreport headroom as soon as that happens."""
        hm = self._vbg.hashmap()
        return int(hm.size()), int(hm.capacity())

    def _check_saturation(self) -> None:
        """Warn (once) when the map outgrows its CONFIGURED capacity -- BUG-035.

        Deliberately measured against the constructed `block_count`, not the
        live capacity. The grid grows, so a live-capacity test would fire on
        every scan and reset itself after each rehash -- noise. What is
        actionable is "this scan needed more than you configured for it",
        because the run that failed was the one sitting at 97% of its initial
        capacity, while every run given headroom it never had to grow into
        completed cleanly.

        Polled every `_SATURATION_CHECK_EVERY` integrates rather than every
        one (item 7 of the 2026-08-02 follow-ups plan) -- see that constant's
        comment for why this is safe to delay but never safe to skip."""
        # Early-out first: this runs on every integrate, and after the warning
        # has fired there is nothing left to do, so don't pay for the hashmap
        # size read (a device sync on CUDA) for the rest of the scan.
        if self._saturation_warned or self.block_count <= 0:
            return
        self._integrates_since_saturation_check += 1
        if self._integrates_since_saturation_check < _SATURATION_CHECK_EVERY:
            return
        self._integrates_since_saturation_check = 0
        used = int(self._vbg.hashmap().size())
        cap = self.block_count
        if used >= _SATURATION_WARN_FRAC * cap:
            self._saturation_warned = True
            logging.getLogger(__name__).warning(
                "[slam] TSDF map at %d blocks, %.0f%% of the configured "
                "block_count (%d). Open3D will rehash to grow, but a scan that "
                "ran near its initial capacity is exactly where map growth "
                "stalled and frame-to-model tracking collapsed (BUG-035). Raise "
                "[slam] block_count (or coarsen [slam] voxel_size) for a scan "
                "this large.", used, 100.0 * used / cap, cap)

    def _block_buffer_bytes(self, blocks: int) -> int:
        """Device bytes a block buffer of `blocks` blocks occupies: one float32
        tsdf + one float32 weight + a 3-channel float32 color per voxel, and
        `block_resolution**3` voxels per block."""
        return int(blocks) * self.block_resolution ** 3 * (4 + 4 + 3 * 4)

    def _check_rehash_headroom(self) -> None:
        """Fail cleanly, BEFORE Open3D's rehash OOMs the card and aborts us.

        A saturated grid grows by allocating a NEW block buffer of twice the
        capacity while the old one is still live, so the moment of growth wants
        ~2x the current buffer in free device memory. When that is not there the
        allocation fails inside `integrate`, and the failure is not survivable
        from Python: it surfaces as a RuntimeError, but unwinding it leaves
        Open3D's cached allocator inconsistent and the process dies in a
        destructor with `MemoryCache::Free ... should have been recorded` ->
        `terminate called`. Measured on captures/DebugCapB1.bin at the Detailed
        preset (voxel 0.005, block_count 320,000 = 3.05 GiB of buffer on an
        8 GiB card): the map reaches capacity around frame 3100 of 4808 and
        takes the whole roomscan-web server down with it.

        Raising `TsdfCapacityError` a bit before that turns an un-catchable
        process abort into an ordinary Python exception the caller can report,
        which is the difference between "the build stopped and told you why" and
        "the server vanished". No-op on a CPU grid (system RAM, and no CUDA
        allocator to corrupt) and when NVML cannot tell us what is free.
        """
        if not _is_cuda(self._device):
            return
        self._integrates_since_headroom_check += 1
        if self._integrates_since_headroom_check < _HEADROOM_CHECK_EVERY:
            return
        self._integrates_since_headroom_check = 0
        nvml = self._nvml
        if nvml is None:
            nvml = self._nvml = _gpumem.Nvml()
        if not nvml.ok:
            return
        used, capacity = self.block_usage()
        if capacity <= 0 or used < _REHASH_IMMINENT_FRAC * capacity:
            return
        need = 2 * self._block_buffer_bytes(capacity)
        free = nvml.free_bytes()
        if free >= need:
            return
        raise TsdfCapacityError(
            f"TSDF map is full ({used} of {capacity} blocks) and cannot grow: the next "
            f"rehash needs {need / 2**30:.1f} GiB of free device memory, {free / 2**30:.1f} GiB "
            f"is free. Growing anyway would abort the process, not raise. Re-run this "
            f"reconstruction with a coarser voxel_size (the dominant term -- this map "
            f"needs ~4x fewer blocks at 0.01 than at 0.005), or on the CPU device where "
            f"system RAM is the only limit ([slam] device = \"CPU:0\").")

    def frustum_block_coords(self, depth_mm: np.ndarray, intrinsic: o3d.core.Tensor,
                              extrinsic: np.ndarray) -> o3d.core.Tensor:
        """Unique voxel-block coordinates visible from `extrinsic` given a depth
        hint -- the same frustum-bounded set `integrate()` uses, exposed so
        `Mapper` can bound `raycast()`'s cost to the current view instead of
        the whole map."""
        depth = self._depth_image(depth_mm)
        ext = o3d.core.Tensor(np.asarray(extrinsic, dtype=np.float64), device=_CPU)
        intr = intrinsic.to(_CPU)
        return self._vbg.compute_unique_block_coordinates(
            depth, intr, ext, self.depth_scale, self.depth_max, self.trunc_multiplier)

    def raycast(self, intrinsic: o3d.core.Tensor, extrinsic: np.ndarray,
                width: int, height: int,
                block_coords: o3d.core.Tensor | None = None,
                depth_hint: np.ndarray | None = None) -> o3d.t.geometry.PointCloud | None:
        """block_coords/depth_hint are optional and bound raycast cost to a
        subset of blocks (e.g. the current view frustum) instead of every
        active block in the map. Pass `block_coords` directly (from
        `frustum_block_coords`) or a `depth_hint` to have it computed here;
        omit both to fall back to the original all-active-blocks behavior."""
        if self._empty:
            return None
        ext = o3d.core.Tensor(np.asarray(extrinsic, dtype=np.float64), device=_CPU)
        intr = intrinsic.to(_CPU)
        if block_coords is not None:
            coords = block_coords
        elif depth_hint is not None:
            coords = self.frustum_block_coords(depth_hint, intrinsic, extrinsic)
        else:
            hashmap = self._vbg.hashmap()
            active_idx = hashmap.active_buf_indices()
            if active_idx.shape[0] == 0:
                return None
            coords = hashmap.key_tensor()[active_idx]
        if coords.shape[0] == 0:
            return None
        result = self._vbg.ray_cast(
            coords, intr, ext, width, height,
            render_attributes=["vertex", "normal", "depth"],
            depth_scale=self.depth_scale, depth_min=0.1,
            depth_max=self.depth_max, weight_threshold=1.0,
            trunc_voxel_multiplier=self.trunc_multiplier,
            range_map_down_factor=1)
        # ray_cast's outputs live on self._device (may be CUDA) -- move to
        # host before .numpy() (a no-op when self._device is already CPU).
        vertex = result["vertex"].cpu().numpy().reshape(-1, 3)
        normal = -result["normal"].cpu().numpy().reshape(-1, 3)
        depth = result["depth"].cpu().numpy().reshape(-1)
        keep = depth > 0.0
        if not keep.any():
            return None
        pc = o3d.t.geometry.PointCloud(self._device)
        pc.point.positions = o3d.core.Tensor(vertex[keep].astype(np.float32), device=self._device)
        pc.point.normals = o3d.core.Tensor(normal[keep].astype(np.float32), device=self._device)
        return pc

    def _cuda_extract_fits(self) -> bool:
        """Whether a GPU-side extraction has room, by measured NVML headroom.

        The historical worry (below) was that `ExtractTriangleMeshCUDA` OOMs on
        a grown map. That is a real failure mode, but it is a function of FREE
        DEVICE MEMORY vs. active blocks, which is a thing we can measure --
        `_CUDA_EXTRACT_BYTES_PER_BLOCK` is the measured per-block cost. Asking
        the question each time also means a map that outgrows the card mid-scan
        degrades to the host path exactly when it needs to, rather than every
        extraction paying for the worst case.

        Answers False when NVML is unavailable: without a free-memory number
        there is nothing to authorize the fast path with, and the host path is
        slow but always correct."""
        nvml = self._nvml
        if nvml is None:
            nvml = self._nvml = _gpumem.Nvml()
        if not nvml.ok:
            return False
        free = nvml.free_bytes()
        need = self._vbg.hashmap().size() * _CUDA_EXTRACT_BYTES_PER_BLOCK
        return free >= need

    def _extract_vbg(self):
        """The VoxelBlockGrid to extract a mesh/point cloud FROM.

        On CUDA the marching-cubes extractor (`ExtractTriangleMeshCUDA`)
        allocates an "assistance mesh structure" sized to the active-block
        count that can OOM on a grown map (Open3D's own error: "consider ...
        tsdf_volume.cpu() to perform mesh extraction on CPU"); we saw it fail
        at ~25k blocks on a 12 GB GPU. The original mitigation was to take
        `self._vbg.cpu()` UNCONDITIONALLY and extract on the host.

        That mitigation was far more expensive than the thing it avoided,
        because `.cpu()` copies the WHOLE PREALLOCATED grid -- capacity, not
        occupancy -- so its cost is set by `block_count` and does not shrink
        for a small map. Measured on captures/DebugCapB1.bin at the Detailed
        preset (block_count 320,000 = 3.05 GiB of block buffer):

          * `.cpu()` copy 1.11 s, then marching cubes on the host 0.20 s;
          * the SAME extraction run in place on the device: 0.04 s.

        and the copy is flat at 1.1 s from the very first extraction (17k
        verts) onward. Open3D holds the GIL throughout, so those 1.1 s are 1.1 s
        in which no other Python thread runs at all -- including the asyncio
        event loop of `roomscan-web`. At the Detailed cadence (one extraction
        per 25 frames) that starved the server's loop for 78-84% of wall clock:
        the UI's progress bar and 3D view both froze solid, which is what this
        was reported as. Extracting in place instead, over the full 4808-frame
        capture: 192 extractions, 6.5 s of starvation total (11.7% of wall,
        worst single stall 0.08 s), whole build 55.6 s, peak device memory
        bounded at 2461 MiB and host RSS 4.24 -> 1.03 GB.

        So: extract on the device when `_cuda_extract_fits()` says there is
        room, and keep the whole-grid host copy as the fallback -- for a map
        too big for the extractor, for a box with no NVML, and (latched by
        `_host_extract_reason`) for a card that OOMs anyway despite the
        headroom check. Callers still always receive a HOST-resident result;
        see `_extract`."""
        if _is_cuda(self._device) and self._host_extract_reason is None \
                and self._cuda_extract_fits():
            return self._vbg
        if _is_cuda(self._device):
            return self._vbg.cpu()
        return self._vbg

    def _check_extractable(self) -> None:
        """Refuse to extract a map big enough to crash the marching cubes.

        See `_MAX_SAFE_EXTRACT_BLOCKS` for the bisection. Raising here is the
        only available move: past that size the extractor does not fail, it
        kills the process, so there is nothing to catch downstream."""
        blocks = int(self._vbg.hashmap().size())
        if blocks <= _MAX_SAFE_EXTRACT_BLOCKS:
            return
        raise TsdfCapacityError(
            f"map has {blocks} active blocks; Open3D's mesh extraction crashes the "
            f"process above ~{_MAX_SAFE_EXTRACT_BLOCKS} (measured: fine at 258,161, "
            f"segfault/illegal-access at 273,521, on both CPU and CUDA). Raising "
            f"block_count does NOT help -- the limit is the block count itself, not "
            f"the load factor. Re-run with a coarser voxel_size: this map needs ~4x "
            f"fewer blocks at 0.01 than at 0.005.")

    def _extract(self, method: str):
        """Run `method` ("extract_triangle_mesh"/"extract_point_cloud") and
        return a host-resident result, whichever grid it came from.

        Downloading the extracted GEOMETRY is the cheap direction: 0.10 s for
        the 4.09M-vertex mesh at the end of DebugCapB1, against 1.11 s to copy
        the grid that produced it. Keeping the return value on the host also
        keeps this method's contract exactly what it was when the host copy was
        unconditional, so no caller (MeshPrep, `_commit`, the CLI writers) has
        to learn about device-resident meshes.

        An OOM here is caught once and latches the host path permanently: after
        a CUDA OOM Open3D's cached allocator can be left inconsistent enough to
        `terminate()` the process later, so retrying the device path is not a
        risk worth taking for a display-only extraction."""
        vbg = self._extract_vbg()
        on_device = vbg is self._vbg and _is_cuda(self._device)
        if not on_device:
            return getattr(vbg, method)(self.weight_threshold)
        try:
            return getattr(vbg, method)(self.weight_threshold).cpu()
        except RuntimeError as exc:
            self._host_extract_reason = " ".join(str(exc).split())[:200]
            logging.getLogger(__name__).warning(
                "[slam] GPU mesh extraction failed at %d blocks despite free-memory "
                "headroom; falling back to the (much slower) whole-grid host copy for "
                "the rest of this map: %s",
                self._vbg.hashmap().size(), self._host_extract_reason)
            o3d.core.cuda.release_cache()
            return getattr(self._vbg.cpu(), method)(self.weight_threshold)

    def _release_cache_if_due(self) -> None:
        """Drop Open3D's cached-but-unused CUDA blocks after an extraction.

        Sub-phase 6.G (the long-scan OOM). Measured on an RTX 2000 Ada (8 GiB)
        with `host/tools/slam_gpu_memory.py --synthetic`, device bytes above a
        pre-Open3D NVML baseline:

          * the per-frame path alone (integrate + raycast + ICP, no extraction)
            is BYTE-FLAT over 4000 frames / 80 m -- device memory never moved
            off 937361408 bytes while the map grew 900 -> 17k blocks. The map
            is not the leak, and neither are the per-frame temporaries.
          * add extraction at the live cadence (`SlamWorker._MESH_EVERY = 5`)
            and over 1500 frames / 30 m device memory climbs 523 -> 5483 MiB,
            5.13 MiB/frame, with the block count nearly flat -- i.e. it tracks
            EXTRACTIONS, not map size. On an 8 GiB card that OOMs a few hundred
            frames later.
          * with this fix at the default cadence, the same workload run out to
            4000 frames / 80 m peaks at 651 MiB and ends at 523, tail growth
            0.005 MiB/frame. No per-frame cost: over an identical 1500-frame
            A/B, step latency was p50 6.1 ms both ways (p90 7.0 -> 7.1,
            p99 8.6 -> 8.8) and wall time 62.3 s -> 62.7 s.

        The mechanism is `_extract_vbg()`'s `self._vbg.cpu()`: a whole-grid
        device->host copy whose temporaries scale with the active-block count,
        so each extraction asks the caching allocator for a slightly LARGER
        block than the last and the previous one is cached, never reused. That
        is why the cadence here is extractions rather than frames -- releasing
        on a frame counter would fire mostly on frames that allocated nothing.

        No-op on CPU grids and when `release_cache_every` is 0."""
        if not self.release_cache_every or not _is_cuda(self._device):
            return
        self._extractions += 1
        if self._extractions % self.release_cache_every == 0:
            o3d.core.cuda.release_cache()
            self.cache_releases += 1

    def mesh(self) -> o3d.t.geometry.TriangleMesh:
        if self._empty:
            # `extract_triangle_mesh()` raises a C++ HashMap error ("Input
            # number of keys should > 0") on an empty map -- e.g. the very
            # first frame ever submitted is tracking-lost, so `integrate()`
            # is never called. Return an empty mesh of the same shape/dtypes
            # `extract_triangle_mesh()` itself returns for a populated-but-
            # isosurface-free map (verified empirically), instead of
            # propagating that crash to callers (worker/CLI/panel).
            m = o3d.t.geometry.TriangleMesh(device=self._device)
            m.vertex.positions = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=self._device)
            m.vertex.colors = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=self._device)
            m.triangle.indices = o3d.core.Tensor(np.zeros((0, 3), dtype=np.int32), device=self._device)
            return m
        self._check_extractable()
        out = self._extract("extract_triangle_mesh")
        self._release_cache_if_due()
        return out

    def point_cloud(self) -> o3d.t.geometry.PointCloud:
        if self._empty:
            pc = o3d.t.geometry.PointCloud(self._device)
            pc.point.positions = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=self._device)
            pc.point.colors = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=self._device)
            return pc
        self._check_extractable()
        out = self._extract("extract_point_cloud")
        self._release_cache_if_due()
        return out
