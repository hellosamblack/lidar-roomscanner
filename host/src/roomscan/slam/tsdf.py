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
"""
from __future__ import annotations

import numpy as np
import open3d as o3d


def _resolve_device(device) -> o3d.core.Device:
    return device if isinstance(device, o3d.core.Device) else o3d.core.Device(device)


# Open3D's VoxelBlockGrid takes the camera intrinsic/extrinsic as CPU:0 Float64
# tensors REGARDLESS of the grid's own device -- integrate/ray_cast/
# compute_unique_block_coordinates internally call InverseTransformation, which
# asserts CPU:0. On a CPU grid self._device is already CPU:0 so it never
# mattered; on CUDA, passing a CUDA extrinsic raises "Tensor has device CUDA:0,
# but is expected to have CPU:0". Keep these two tensors on CPU always; the
# depth/color images and raycast outputs stay on the compute device.
_CPU = o3d.core.Device("CPU:0")


class TsdfMap:
    def __init__(self, voxel_size: float = 0.01, trunc_multiplier: float = 8.0,
                 block_resolution: int = 8, block_count: int = 40000,
                 depth_scale: float = 1000.0, depth_max: float = 5.0,
                 weight_threshold: float = 3.0,
                 device: str | o3d.core.Device = "CPU:0"):
        self.voxel_size = voxel_size
        self.trunc_multiplier = trunc_multiplier
        self.depth_scale = depth_scale
        self.depth_max = depth_max
        self.weight_threshold = weight_threshold
        self._device = _resolve_device(device)
        self._empty = True
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

    def _extract_vbg(self):
        """The VoxelBlockGrid to extract a mesh/point cloud FROM. On CUDA the
        marching-cubes extractor (`ExtractTriangleMeshCUDA`) allocates an
        "assistance mesh structure" sized to the active-block count that OOMs
        on a grown map (Open3D's own error: "consider ... tsdf_volume.cpu() to
        perform mesh extraction on CPU"); we saw it fail at ~25k blocks on a
        12 GB GPU. Per-frame integrate/raycast stay on the GPU (that's where
        the speedup is); only this throttled, display-only extraction moves to
        the host. `.cpu()` copies the grid to CPU; no-op guard keeps the CPU
        path (self._device already CPU) allocation-free."""
        if "CUDA" in str(self._device).upper():
            return self._vbg.cpu()
        return self._vbg

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
        return self._extract_vbg().extract_triangle_mesh(self.weight_threshold)

    def point_cloud(self) -> o3d.t.geometry.PointCloud:
        if self._empty:
            pc = o3d.t.geometry.PointCloud(self._device)
            pc.point.positions = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=self._device)
            pc.point.colors = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=self._device)
            return pc
        return self._extract_vbg().extract_point_cloud(self.weight_threshold)
