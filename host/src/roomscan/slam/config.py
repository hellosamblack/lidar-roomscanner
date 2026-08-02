"""Read-only SLAM config from the [slam] table of roomscan.toml.

Deliberately NO writer -- roomscan.config's single-table writer is off-limits
(its docstring forbids growing it). Priority for the CLI is:
  flag > this file > dataclass default.
"""
from __future__ import annotations

import tomllib
import dataclasses
import hashlib
import json
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
    # BUG-037. `baro_weight` (a per-frame blend gain toward the RAW barometric
    # height, whose DC authority was 1.0) is retired and no longer read; an
    # old config that still sets it is ignored, as with any unknown key.
    # `baro_authority` is the barometer's share of a low-passed height
    # disagreement -- the least-squares blend of two drifting estimates, from
    # the measured drift rates (ICP ~0.09 m/min vs barometer ~0.45 m/min).
    # `baro_tau_frames` is that low-pass, in frames: the raw signal is ~267 mm
    # RMS of white noise, and ~900 frames (30 s at 30 fps) leaves ~6 mm of it.
    # 0 authority disables the constraint entirely. See mapper._apply_baro_z.
    baro_authority: float = 0.05
    baro_tau_frames: int = 900
    max_dist: float = 0.05
    # Six is the proven baseline. Detailed only adopts 8/10/12 after matched
    # ensemble validation; a one-off circuit is intentionally not evidence.
    max_iter: int = 6
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


@dataclass
class DetailedSlamPreset:
    """Resolved offline reconstruction preset.

    The values intentionally live beside ``SlamConfig`` but are not inherited
    implicitly: a Detailed sidecar must say exactly which quality settings made
    it.  ``per_frame_ms`` and ``global_opt_ms`` are calibration values, not
    claims; zero means **benchmark me** and the UI labels the estimate as such.
    Loop closure is opt-in only after the two-circuit validation gate records an
    accepted decision.
    """
    # 0.01, not the 0.005 this shipped with (2026-08-01). At 5 mm a room-sized
    # capture builds more blocks than Open3D's marching cubes can extract --
    # captures/DebugCapB1.bin (4808 frames) crosses `_MAX_SAFE_EXTRACT_BLOCKS`
    # at frame 2625 and can never produce a mesh, whatever block_count is set
    # to (tsdf.py has the bisection). The same capture at 0.01 needs 139,785
    # blocks, completes all 4808 frames in 56 s and yields 4.1M vertices. The
    # `benchmark_note` below asked for exactly this measurement and had never
    # been run; 5 mm was only ever exercised on captures small enough to fit.
    voxel_size: float = 0.01
    block_count: int = 320000
    max_iter: int = 6
    max_dist: float = 0.05
    retry_dist: float = 0.10
    mesh_every: int = 25
    per_frame_ms: float = 0.0
    global_opt_ms: float = 0.0
    loop_closure: bool = False
    benchmark_note: str = "benchmark me on CUDA:0 with a full-room capture"

    @classmethod
    def load(cls, path: Optional[Path] = None) -> "DetailedSlamPreset":
        path = Path(path) if path is not None else config_path()
        try:
            raw = tomllib.loads(path.read_text(encoding="utf-8"))
            table = raw.get("slam", {}).get("detailed", {})
        except (OSError, tomllib.TOMLDecodeError, AttributeError):
            return cls()
        if not isinstance(table, dict):
            return cls()
        known = {f.name for f in dataclasses.fields(cls)}
        try:
            return cls(**{k: v for k, v in table.items() if k in known})
        except (TypeError, ValueError):
            return cls()

    # Fields that describe how long a build TAKES, not what it PRODUCES. They
    # are excluded from the fingerprint on purpose: calibrating the estimate --
    # which is the explicitly planned next step -- must not mark every existing
    # sidecar stale. A staleness flag that fires on unrelated changes is one the
    # user learns to ignore, and then it cannot warn about a real one.
    _NON_RECONSTRUCTION_FIELDS = ("per_frame_ms", "global_opt_ms", "benchmark_note")

    def fingerprint(self) -> str:
        payload = {k: v for k, v in dataclasses.asdict(self).items()
                   if k not in self._NON_RECONSTRUCTION_FIELDS}
        return hashlib.sha256(json.dumps(payload, sort_keys=True,
                                         separators=(",", ":")).encode()).hexdigest()[:16]

    def mapper_kwargs(self, base: SlamConfig | None = None) -> dict:
        base = base or SlamConfig.load()
        # Detailed always has the wider retry available.  More than six ICP
        # iterations is used only after the benchmark establishes it helps.
        return {
            "fov_h": base.fov_h, "fov_v": base.fov_v, "icp_mode": base.icp_mode,
            "voxel_size": self.voxel_size, "block_count": self.block_count,
            "max_dist": self.max_dist, "icp_retry_dist": self.retry_dist,
            "max_iter": self.max_iter, "min_fitness": base.min_fitness,
            "max_rmse": base.max_rmse, "min_confidence": base.min_confidence,
            "weight_threshold": base.weight_threshold,
            "baro_authority": base.baro_authority, "baro_tau_frames": base.baro_tau_frames,
            "stationary_hold": False, "release_cache_every": base.release_cache_every,
            "device": preferred_device(),
        }
