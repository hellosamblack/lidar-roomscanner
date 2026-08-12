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

    # "translation" (default, shipped): IMU-rotation + ICP-translation. "6dof":
    # full ICP (study/benchmark only). "adaptive": EXPERIMENTAL LiDAR-primary --
    # measured WORSE than translation on this sensor (its 6dof rotation is noisier
    # than the SFLP IMU), kept opt-in only. See slam/odometry.py's module docstring.
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
    # Cap on the effective condition number of the 3x3 point-to-plane normal
    # equations (BUG-068). Below it the solve is unchanged; above it the step is
    # bounded along the directions the geometry cannot observe, instead of the
    # frame being rejected -- rejection is terminal here (see icp_retry_dist).
    # 0 disables the cap, restoring the pre-BUG-068 unbounded solve.
    # Literal, not an import: `config` must stay Open3D-free. The value is
    # pinned to `odometry._COND_CAP` by
    # test_mapper_kwargs_defaults_match_mapper_signature.
    icp_cond_cap: float = 20.0
    # Soft-prior rotational damping (BUG-067), a DIMENSIONLESS multiple of the
    # rotation block's own mean stiffness. Only read when icp_mode="soft_prior":
    # rotation is held NEAR the SFLP prior rather than frozen at it, so a
    # rotation-prior error lands in a small bounded rotation instead of being
    # fabricated as translation and latched into the TSDF. -> inf reproduces the
    # frozen-rotation "translation" solve; 0 is undamped 6-DoF. Provisional
    # default pending the matched-ensemble sweep; pinned to odometry._ROT_PRIOR_WEIGHT
    # by test_mapper_kwargs_defaults_match_mapper_signature. See slam/odometry.py.
    icp_rot_prior_weight: float = 10.0
    min_fitness: float = 0.3
    max_rmse: float = 0.05
    # icp_mode="adaptive" (LiDAR-primary, IMU-gated): the STRICTER bar the full
    # 6dof solve must clear before its rotation is trusted over the IMU prior.
    # Above these it falls back to the translation solve (rotation locked to the
    # prior). Stricter than min_fitness/max_rmse on purpose -- Open3D's 6dof
    # reports fitness >= min_fitness even where it diverges. `adapt_max_corr_deg`
    # rejects an implausibly large per-frame rotation correction (divergence
    # backstop). Unused unless icp_mode == "adaptive". Pinned to Mapper.__init__
    # by test_mapper_kwargs_defaults_match_mapper_signature.
    adapt_min_fitness: float = 0.6
    adapt_max_rmse: float = 0.03
    adapt_max_corr_deg: float = 20.0
    # Rotational-observability gate for adaptive mode: max condition number of the
    # point-to-plane rotation Hessian (sum (p x n)(p x n)^T) below which the 6dof
    # rotation is trusted. A flat wall is rank-deficient here (rotation about its
    # normal unobservable) and fits with fitness ~1 anyway, so fitness alone let
    # the 6dof rotation drift 17.9 m on a flat-wall tripod capture; this defers to
    # the IMU exactly when geometry cannot constrain rotation. 0 disables the gate.
    adapt_rot_cond_cap: float = 100.0
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
    # Rotation-prior smoothing (BUG-067 lever): causal EMA/slerp weight on the
    # PREVIOUS smoothed prior quaternion. 0 = off (raw prior, byte-identical);
    # higher low-passes the orientation prior more. A noisy prior becomes
    # fabricated translation in translation/soft-prior ICP (rotation cannot argue
    # with it), so smoothing it collapsed the tripod instability in BUG-067's
    # phase sweep. See mapper._smooth_prior.
    prior_smooth_alpha: float = 0.0
    # Quat-phase alignment (BUG-031/067, mechanism superseded by #155): give each
    # depth frame the orientation AT its own frame-ready instant. Since #155 the
    # recorded/offline loader does this by TIMESTAMP INTERPOLATION — SLERP between
    # the exact-group stream-9 samples bracketing the frame time on the LSM clock
    # (`_load_frames(quat_interp=...)`) — because the lead is not a constant
    # (+7.76 ms on the golden capture, +5.13 ms on DebugCapF). The original fixed
    # gyro rollback (mapper.step / frames.apply_quat_phase) remains only as the
    # per-frame fallback where interpolation has no bracket. OFF by default -- it
    # changes the rotation prior and wants its own before/after on a moving
    # capture (#126 measured the FIXED version worse and bistable on the tripod).
    # Needs stream 11 + 13 in the capture; a no-op where either is absent.
    apply_quat_phase: bool = False
    # Accelerometer ZUPT (BUG-069): a MAP-REACHING zero-velocity constraint that
    # fires on |a| ~= 1 g, so it works during a pan (unlike the display-only
    # StationarityGate). ON by default -- validated clean (0 tracking loss) and
    # beneficial on three captures: tripod closure sd 0.069 -> 0.015, coffee
    # circuit 0.735 -> 0.671 m, roomSweep neutral (owner decision 2026-08-06).
    # A no-op wherever stream 11 (accel) is absent, so it degrades gracefully.
    # `zupt_accel_tol_g` is the tolerance band around 1 g (fraction of g);
    # `zupt_window` frames must all be still before it trips. See mapper.step /
    # motion.ZuptDetector. Set false to restore pre-2026-08-06 behaviour.
    zupt_enabled: bool = True
    zupt_window: int = 6
    zupt_accel_tol_g: float = 0.04
    # Translation-coherence veto on the ZUPT: steady walking also reads ~1 g, so
    # the accel gate alone froze the pose mid-stride on a real circuit. Holding
    # only when the recent ICP translation is ALSO incoherent (jitter, not a
    # directed walk) is what separates a tripod pan from a walk. 0 disables the
    # veto (accel-only, measured unsafe on real motion). See motion.ZuptDetector.
    zupt_coherence: float = 0.5
    # IMU spike pre-gate: inter-frame SFLP rotation (deg) above which the quat is
    # held at the last good orientation instead of corrupting the raycast viewpoint
    # and the pose. DEFAULT 0 = DISABLED (defensive-only): the capture that looked
    # like an IMU spike (2026-08-05-crazySLAM.bin, capture_motion read 14737 deg/s)
    # was actually a BAROMETER dropout -- the mapper sees only 2.9 deg/frame there --
    # so there is no measured threshold to ship active. ~30 deg/frame is sane if
    # enabled. Pinned to Mapper.__init__'s default by
    # test_mapper_kwargs_defaults_match_mapper_signature.
    imu_spike_deg: float = 0.0
    # Barometer outlier rejection: drop a pressure sample whose implied altitude
    # vs the datum exceeds this many metres (an indoor handheld scan never moves
    # that far), plus a hard reject of non-physical pressure <= 0. A dropout reads
    # 0.0 Pa -> 44330 m -> a ~2.45 m vertical step per sample; 4 of them fabricated
    # 2026-08-05-crazySLAM.bin's entire 8.6 m of vertical "translation". 0 disables.
    # See mapper._apply_baro_z / _baro_is_outlier.
    baro_reject_m: float = 50.0
    # Compute device for the Open3D tensor pipeline (TsdfMap/pinhole/
    # source_cloud/register). "CPU:0" today -- the installed Open3D 0.19
    # build here has no CUDA support -- but "CUDA:0" (or any other
    # o3d.core.Device string) runs unchanged once a CUDA-enabled build is
    # installed; see slam/mapper.py's docstring. NOT read by the web app's
    # live SLAM path: `web.SlamRunner._construct` calls `preferred_device()`
    # directly (CUDA:0 when available) and never looks at this field or this
    # dataclass's instance at all -- this default only reaches the CLI/offline
    # paths (slam/cli.py) and DetailedSlamPreset.mapper_kwargs, which also
    # overrides it with `preferred_device()`.
    device: str = "CPU:0"
    # Device for ICP's nearest-neighbour index ONLY -- everything else (TSDF
    # integrate, raycast, the source cloud) stays on `device` above. Unlike
    # `device` this IS read by every path including Live SLAM.
    #
    # "CPU:0" because the shipped `translation` solve already downloads source
    # positions, target positions and target normals and does all its
    # arithmetic in numpy: the compute device only picks which hybrid index
    # runs the search, so running it on the host removes a device round-trip
    # instead of adding one. Output is bit-identical (3177 ICP calls across two
    # captures, plus a 1979-frame x 10-perturbation ensemble that matched the
    # baseline to the last digit) and it measured -0.2 to -0.55 ms/frame, i.e.
    # -1.5% to -10% of a SLAM step, depending on how loaded the box's CPU is.
    # See docs/superpowers/plans/2026-08-02-cuda-icp-study.md SS B/C/E.
    #
    # It is a KNOB, not a constant, because it converts GPU wait into CPU work
    # and roomscan-web's asyncio loop, reader thread and broadcaster share that
    # CPU: set "CUDA:0" to restore the pre-2026-08-02 behaviour. Ignored by the
    # `6dof` icp_mode, whose ICP is Open3D's own and must run where its point
    # clouds live (see Mapper._register_device).
    icp_device: str = "CPU:0"
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
    # Live MESH publish budget, bytes/second (BUG-060). The live map is sent
    # whole every update and grows without bound -- 3.2 MB at 63k verts, 31 MB
    # at 611k (1500 frames of roomSweepFull) -- so at the raw extraction cadence
    # a grown map would push >100 MB/s at the browser. (This comment used to say
    # that stalls the broadcaster "on socket backpressure"; BUG-061 proved there
    # is NO backpressure -- uvicorn never blocks `send_bytes`, so the excess
    # silently piles into an unbounded per-client buffer instead. That is why
    # this budget alone was not enough and MESH now has its own credit-gated
    # `/ws-mesh` channel.) `SlamRunner` therefore spaces publishes by
    # `last_payload_bytes / live_mesh_bytes_per_s`: a small map still updates at
    # the full extraction rate, a big one updates more slowly, and the wire rate
    # is flat. This bounds the payload by CADENCE rather than by decimating it,
    # because quadric decimation measured 14x more expensive than the bandwidth
    # it saved (see meshprep.MeshPrep). 12 MB/s ~= 100 Mbit/s.
    live_mesh_bytes_per_s: float = 12_000_000.0

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

    def mapper_kwargs(self) -> dict:
        """Every `Mapper` knob this config owns, as constructor kwargs.

        BUG-062: the live web path used to hand-pick five of these
        (`release_cache_every`, `block_count`, `icp_retry_dist`,
        `baro_authority`, `baro_tau_frames`), so setting `icp_mode`,
        `voxel_size`, `max_iter`, `max_dist`, `min_fitness`, `max_rmse`,
        `min_confidence`, `weight_threshold` or any `stationary_*` key in
        `[slam]` changed the CLI and Detailed paths but **not Live SLAM** --
        silently, with no warning and no way to tell from the UI. Anything
        `Mapper.__init__` accepts and `SlamConfig` carries belongs here, so a
        knob cannot be added to one path and forgotten on the other.

        Behaviour-neutral at stock config: every default here equals the
        corresponding `Mapper.__init__` default (pinned by
        `test_mapper_kwargs_defaults_match_mapper_signature`), so this only
        changes what a user who actually set a value gets.

        `fov_h`/`fov_v` are included, but a caller with a measured sensor FOV
        (`web.SlamRunner`) overrides them. `device` resolves via
        `preferred_device()` rather than `self.device` -- see that field's note;
        the live path has never read it and making it authoritative here would
        silently move Live SLAM onto CPU for everyone on a stock config.
        """
        return {
            "fov_h": self.fov_h, "fov_v": self.fov_v,
            "icp_mode": self.icp_mode, "voxel_size": self.voxel_size,
            "max_dist": self.max_dist, "icp_retry_dist": self.icp_retry_dist,
            "icp_cond_cap": self.icp_cond_cap,
            "icp_rot_prior_weight": self.icp_rot_prior_weight,
            "max_iter": self.max_iter,
            "min_fitness": self.min_fitness, "max_rmse": self.max_rmse,
            "adapt_min_fitness": self.adapt_min_fitness,
            "adapt_max_rmse": self.adapt_max_rmse,
            "adapt_max_corr_deg": self.adapt_max_corr_deg,
            "adapt_rot_cond_cap": self.adapt_rot_cond_cap,
            "min_confidence": self.min_confidence,
            "weight_threshold": self.weight_threshold,
            "baro_authority": self.baro_authority,
            "baro_tau_frames": self.baro_tau_frames,
            "baro_reject_m": self.baro_reject_m,
            "stationary_hold": self.stationary_hold,
            "stationary_window": self.stationary_window,
            "stationary_coherence": self.stationary_coherence,
            "stationary_step_ceiling": self.stationary_step_ceiling,
            "stationary_rot_ceiling": self.stationary_rot_ceiling,
            "prior_smooth_alpha": self.prior_smooth_alpha,
            "apply_quat_phase": self.apply_quat_phase,
            "zupt_enabled": self.zupt_enabled,
            "zupt_window": self.zupt_window,
            "zupt_accel_tol_g": self.zupt_accel_tol_g,
            "zupt_coherence": self.zupt_coherence,
            "imu_spike_deg": self.imu_spike_deg,
            "release_cache_every": self.release_cache_every,
            "block_count": self.block_count,
            "icp_device": self.icp_device,
            "device": preferred_device(),
        }


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
        """`SlamConfig`'s knobs, with the preset's reconstruction settings on top.

        Built by overriding `base.mapper_kwargs()` rather than by re-listing the
        fields (item 5, 2026-08-02). This was the third place that knew the
        `Mapper` field list, and BUG-062 is exactly what a second place costs:
        a knob added to `SlamConfig` and forgotten here is honoured by the CLI
        and Live SLAM and silently ignored by every Detailed reconstruction.

        Behaviour-neutral at the time of the change: the only keys this newly
        forwards are the four `stationary_*` tuning values, which `Mapper` reads
        only when building a `StationarityGate` -- and `stationary_hold` is
        pinned False below, so no gate is built (pinned by
        `test_detailed_mapper_kwargs_covers_every_shared_field`).
        """
        base = base or SlamConfig.load()
        # Detailed always has the wider retry available.  More than six ICP
        # iterations is used only after the benchmark establishes it helps.
        kw = base.mapper_kwargs()
        kw.update(
            voxel_size=self.voxel_size, block_count=self.block_count,
            max_dist=self.max_dist, icp_retry_dist=self.retry_dist,
            max_iter=self.max_iter,
            # An offline rebuild is not a live preview: nothing is being watched
            # while it runs, so the display-only de-jitter has nothing to
            # de-jitter and only costs accuracy-neutral work.
            stationary_hold=False,
        )
        return kw
