"""Read-only SLAM config from the [slam] table of roomscan.toml.

Deliberately NO writer -- roomscan.config's single-table writer is off-limits
(its docstring forbids growing it). Priority for the CLI is:
  flag > this file > dataclass default.
"""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Optional

from ..config import config_path


def preferred_device() -> str:
    """Best available Open3D compute device string: ``"CUDA:0"`` when the
    installed Open3D build reports working CUDA support, else ``"CPU:0"``.

    Lets the live panel auto-accelerate the SLAM tensor pipeline the moment a
    CUDA-enabled Open3D is installed, with zero config change, while staying on
    CPU with the stock (CPU-only) wheel. ``open3d`` is imported lazily so this
    module stays importable in environments without it (e.g. some tests). Any
    import/attribute error degrades safely to CPU."""
    try:
        import open3d as o3d
        if o3d.core.cuda.is_available():
            return "CUDA:0"
    except Exception:
        pass
    return "CPU:0"


@dataclass
class SlamConfig:
    """SLAM configuration read from [slam] table in roomscan.toml.

    Missing or corrupt config files are tolerated -- all fields fall back to
    built-in defaults. Only recognized fields are pulled from a present
    ``[slam]`` table; anything else is ignored.
    """

    icp_mode: str = "translation"
    voxel_size: float = 0.01
    baro_weight: float = 0.05
    max_dist: float = 0.05
    # Wider ICP correspondence radius retried ONLY when `max_dist` fails its
    # gate; 0 disables. A single fixed radius cannot be both accurate and
    # robust: 0.05 is the accuracy optimum, but one frame whose residual
    # exceeds it finds zero correspondences and -- with translation frozen on a
    # lost frame and no relocalization -- kills the rest of the scan silently
    # (captures/coffeeRoomCircuitMnt.bin: 423 frames lost from one failure).
    # Escalating only on failure fixed that run (423 lost -> 0) while leaving a
    # clean run bit-identical (0 escalations). See odometry.register_escalating.
    icp_retry_dist: float = 0.10
    min_fitness: float = 0.3
    max_rmse: float = 0.05
    fov_h: float = 55.0
    fov_v: float = 42.0
    # Task 13 (data-quality): reflectance color + noise reduction, tuned against
    # captures/phase6_motion_ref.bin -- see task-quality-report.md.
    min_confidence: float = 20.0
    weight_threshold: float = 3.0
    # Stationarity hold: freeze the pose when the ICP translation is incoherent
    # jitter (device effectively still) so the estimate doesn't random-walk on
    # a stationary sensor. Coherent motion passes untouched. See slam/motion.py.
    stationary_hold: bool = True
    stationary_window: int = 10
    stationary_coherence: float = 0.5
    stationary_step_ceiling: float = 0.03
    stationary_rot_ceiling: float = 0.3
    # Compute device for the Open3D tensor pipeline (TsdfMap/pinhole/
    # source_cloud/register). "CPU:0" today -- the installed Open3D 0.19
    # build here has no CUDA support -- but "CUDA:0" (or any other
    # o3d.core.Device string) runs unchanged once a CUDA-enabled build is
    # installed; see slam/mapper.py's docstring.
    device: str = "CPU:0"
    # Sub-phase 6.G (long-scan OOM): on CUDA, release Open3D's cached-but-unused
    # device blocks every N mesh/point-cloud EXTRACTIONS. The per-frame
    # integrate/raycast/ICP path is byte-flat; the throttled extraction's
    # whole-grid `.cpu()` copy is what grew device memory ~5.1 MiB/frame until
    # it OOM'd. 1 = release after every extraction (measured: same wall time and
    # same p50/p90/p99 step latency as off); 0 disables. No-op on a CPU device.
    # Measured with host/tools/slam_gpu_memory.py; see slam/tsdf.py.
    release_cache_every: int = 1
    # BUG-035: VoxelBlockGrid initial capacity. Running a scan NEAR this value
    # stalls map growth and collapses frame-to-model tracking (the grid does
    # rehash to grow -- that is not the problem; running at ~97% of it is).
    # The owner's full room sweep needs 42,917 blocks at 1 cm
    # voxels; the default is ~3.7x that, at ~14.2 KiB/block of device memory.
    # Raise for larger rooms or finer voxels. A CPU grid ([slam] device =
    # "CPU:0") can go far higher -- system RAM, not VRAM, is the limit there
    # (it completed that same sweep with 0 lost frames), at the ~2.1x CPU/GPU
    # per-step ratio from the CUDA at-scale validation. See slam/tsdf.py.
    #
    # Spelled as a literal, not imported from slam.tsdf: this module must stay
    # importable without open3d (see preferred_device's lazy import), and
    # importing tsdf would pull it in at module scope. test_slam_config pins
    # this to tsdf.DEFAULT_BLOCK_COUNT so the two cannot drift.
    block_count: int = 160000
    # Compute backend for the live worker: "local" runs Mapper in-process
    # (default, unchanged behavior); "remote" ships frames to a SlamService
    # (GPU WSL container) at remote_addr, falling back to local if unreachable.
    backend: str = "local"
    remote_addr: str = "127.0.0.1:5555"
    # Live-view render cadence (Component A -- off-thread adaptive mesh). The
    # heavy mesh/ribbon/floor upload runs at most `mesh_upload_hz` times/sec on
    # the GUI tick; MeshPrep decimates a packet to ~`live_vertex_budget` verts
    # only once an upload's measured wall-time exceeds `fps_budget_ms` (~120 fps
    # per-upload ceiling). Display-only: the saved map is always full-res.
    mesh_upload_hz: float = 3.0
    live_vertex_budget: int = 150000
    fps_budget_ms: float = 8.0

    @classmethod
    def load(cls, path: Optional[Path] = None) -> "SlamConfig":
        """Load SLAM config from [slam] table in roomscan.toml.

        Args:
            path: Path to TOML file. If None, uses config_path() from roomscan.config.

        Returns:
            SlamConfig with values from file, or defaults if file is missing,
            unreadable, corrupt, or missing the [slam] table.
        """
        path = Path(path) if path is not None else config_path()
        try:
            raw = path.read_text(encoding="utf-8")
        except OSError:
            return cls()
        try:
            data = tomllib.loads(raw)
        except tomllib.TOMLDecodeError:
            return cls()
        table = data.get("slam")
        if not isinstance(table, dict):
            return cls()
        known = {f.name for f in fields(cls)}
        kwargs = {k: v for k, v in table.items() if k in known}
        try:
            return cls(**kwargs)
        except TypeError:
            return cls()
