"""Task 12: Showcase mode -- the engine behind "record -> live preview ->
post-process -> final reveal". Two testable units:

  * `ShowcasePhase` + `next_phase()`: the (tiny) state machine the panel steps
    through. Pulled out as a pure function -- even though the transitions are
    simple -- so the panel's phase logic has a unit-testable home instead of
    living only inside GUI callbacks (see docs/superpowers/sdd/
    task-showcase-brief.md).
  * `PostProcessWorker`: re-runs the FULL-quality `Mapper` over an entire
    recorded capture on a background thread, republishing an
    increasingly-complete `(fraction, mesh, trajectory, done, stats)` every
    `mesh_every` frames -- this is what lets the panel show the map visibly
    sharpen while "Processing..." is up, instead of just a progress bar.

Mirrors slam/worker.py's threading contract (that module's docstring is the
canonical statement of the rules; repeated in short form here):
  * `latest()` is lock-guarded and never blocks; it returns a COPY of the
    trajectory (and a Progress instance) so a caller can't see it mutate
    after the fact.
  * `start()` runs everything on a background thread; `stop()` sets a stop
    event and joins, bounded -- it must never hang, including a never-started
    worker's stop() and stopping mid-run (which simply publishes nothing
    further).
  * No serial writes happen here, ever -- this worker only ever touches an
    already-loaded frame list and a `Mapper`.
  * A per-frame exception must not kill the thread silently: `run()` guards
    `mapper.step` per frame and, even if every single frame raises, still
    publishes a terminal `done=True` result so a caller's `latest()` is never
    left stuck mid-progress with the thread already dead.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum, auto

import numpy as np
import open3d as o3d

from . import metrics as _metrics
from .cli import _load_frames, _load_frames_maybe_imu
from .mapper import Mapper
from .tsdf import TsdfCapacityError

_MESH_EVERY = 25
_CPU = o3d.core.Device("CPU:0")


def _footprint_of(mesh) -> float:
    """`footprint_area_m2` of a tensor mesh's vertices, tolerant of an empty or
    device-resident mesh. Never raises: this is one field of a progress stats
    dict, and a stats failure must not lose the whole reconstruction."""
    try:
        pts = mesh.vertex.positions.cpu().numpy()
    except Exception:
        return 0.0
    return round(_metrics.footprint_area_m2(pts), 2)


def _empty_mesh() -> "o3d.t.geometry.TriangleMesh":
    """A 0-vertex/0-triangle mesh of the same shape/dtypes `TsdfMap.mesh()`
    itself returns for an empty map (see tsdf.py) -- used when there's no
    `Mapper` at all to ask (a construction failure, see `run()`)."""
    m = o3d.t.geometry.TriangleMesh(device=_CPU)
    m.vertex.positions = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=_CPU)
    m.vertex.colors = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=_CPU)
    m.triangle.indices = o3d.core.Tensor(np.zeros((0, 3), dtype=np.int32), device=_CPU)
    return m


class ShowcasePhase(Enum):
    IDLE = auto()
    RECORDING = auto()
    PROCESSING = auto()
    FINAL = auto()


def next_phase(phase: ShowcasePhase, *, record_pressed: bool = False,
               stop_pressed: bool = False, processing_done: bool = False,
               cleared: bool = False) -> ShowcasePhase:
    """Pure phase transition -- the panel calls this instead of hand-rolling
    the same if/elif chain inline, so the (small but real) conditional logic
    stays unit-testable.

    `cleared` always wins (Clear/reset returns to IDLE from any phase).
    `record_pressed` always (re)starts a fresh scan from ANY phase except an
    already-running RECORDING (idempotent) -- per the brief's "(-> back to
    IDLE on a new record / clear)": pressing Record while looking at a
    FINAL reveal, or mid-PROCESSING, restarts rather than being ignored (the
    panel is responsible for tearing down whatever the interrupted phase was
    using -- see panel.py's _enter_showcase_recording). Otherwise each phase
    only reacts to its own trigger and holds."""
    if cleared:
        return ShowcasePhase.IDLE
    if record_pressed and phase is not ShowcasePhase.RECORDING:
        return ShowcasePhase.RECORDING
    if phase is ShowcasePhase.RECORDING and stop_pressed:
        return ShowcasePhase.PROCESSING
    if phase is ShowcasePhase.PROCESSING and processing_done:
        return ShowcasePhase.FINAL
    return phase


@dataclass
class Progress:
    fraction: float
    mesh: object
    trajectory: list
    done: bool
    stats: dict | None = None
    # Extracting a tensor mesh is a heavyweight GPU synchronization.  Publish
    # that distinct phase before asking Open3D for the mesh so clients don't
    # mistake a stationary frame count for a stalled reconstruction.
    phase: str = "frames"


class PostProcessWorker:
    """Re-processes a recorded capture at full quality on a background
    thread. `frames` is the ctor arg (not a path) so this is unit-testable
    with synthetic data, matching `SlamWorker`'s pattern; `from_capture` is
    the convenience constructor for the live panel path."""

    def __init__(self, frames, width: int, height: int,
                 mesh_every: int = _MESH_EVERY, imu_aux=None, **mapper_kwargs):
        self._frames = frames
        self._imu_aux = imu_aux          # per-frame (ImuRawBatch|None, quat_offset_us|None)
        self._width = width
        self._height = height
        self._mesh_every = max(1, int(mesh_every))
        self._mapper_kwargs = mapper_kwargs

        self._lock = threading.Lock()
        self._latest: Progress | None = None

        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    @classmethod
    def from_capture(cls, path, mesh_every: int = _MESH_EVERY, **kw) -> "PostProcessWorker":
        # Decode the raw IMU (streams 11/13) only when a lever that needs it is on
        # (ZUPT is on by default) -- stream 11 is the biggest in the file, so an
        # unconditional decode would tax a build that isn't using it.
        need_imu = bool(kw.get("apply_quat_phase") or kw.get("zupt_enabled"))
        frames, width, height, imu_aux = _load_frames_maybe_imu(path, need_imu=need_imu)
        return cls(frames, width, height, mesh_every=mesh_every, imu_aux=imu_aux, **kw)

    # ---- reader side (GUI thread) --------------------------------------
    def latest(self) -> Progress | None:
        """Latest published `Progress`, or None before the first publish.
        Returns a fresh copy (trajectory list copied) so the caller can't
        observe it change out from under it."""
        with self._lock:
            p = self._latest
            if p is None:
                return None
            return Progress(p.fraction, p.mesh, list(p.trajectory), p.done, p.stats, p.phase)

    @property
    def timestamps(self) -> list[float]:
        """Capture header times paired with the offline trajectory."""
        return [float(frame[5]) for frame in self._frames]

    # ---- worker side -----------------------------------------------------
    def _publish(self, mapper: Mapper, frames_done: int, total: int, done: bool) -> None:
        mesh = mapper.mesh()
        stats = None
        if done:
            tstats = _metrics.trajectory_stats(mapper.trajectory)
            stats = {
                "frames": frames_done,
                "gap_m": tstats["start_end_gap_m"],
                "path_m": tstats["path_length_m"],
                "verts": int(len(mesh.vertex.positions)),
                "lost": mapper.tracking_lost_count,
                # Floor-projected footprint of the mesh, NOT its surface area
                # -- see footprint_area_m2's docstring. Computed here because
                # this is the one place the finished mesh is in hand; `_commit`
                # passes `stats` straight into `build_manifest`, so the sidecar
                # gains the field with no change in detailed.py. Additive: the
                # manifest `schema` is deliberately NOT bumped (`sidecar_status`
                # never reads `stats`), so a pre-existing manifest simply has no
                # `area_m2` and the UI renders an em dash.
                "area_m2": _footprint_of(mesh),
            }
        progress = Progress(
            fraction=(frames_done / total) if total else 1.0,
            mesh=mesh,
            trajectory=list(mapper.trajectory),
            done=done,
            stats=stats,
            phase="offline_only" if done else "frames",
        )
        with self._lock:
            self._latest = progress

    def _publish_extracting_mesh(self, mapper: Mapper, frames_done: int, total: int) -> None:
        """Expose mesh extraction before the potentially long ``mapper.mesh``.

        Detailed maps can take substantial time to turn their TSDF into a
        tensor mesh.  That work is intentionally still on this worker thread,
        but its *phase* is available to the web UI immediately.  Reuse the
        prior mesh, if any, so presentation can continue while the next one is
        being extracted.
        """
        with self._lock:
            previous_mesh = self._latest.mesh if self._latest is not None else None
        # A number of consumers use fraction==1 as the terminal signal.  The
        # final mesh extraction is still real work, so keep it infinitesimally
        # below 1 until `_publish(..., done=True)` has completed.
        fraction = frames_done / total if total else 1.0
        if total and frames_done >= total:
            fraction = (total - 0.001) / total
        with self._lock:
            self._latest = Progress(
                fraction=fraction,
                mesh=previous_mesh,
                trajectory=list(mapper.trajectory),
                done=False,
                stats=None,
                phase="extracting_mesh",
            )

    def _publish_capacity_failure(self, mapper: Mapper, frames_done: int,
                                  total: int, exc: Exception) -> None:
        """Terminal publish for a map that outgrew its device mid-build.

        `done=True` because nothing more will happen, but `error` is set and
        `frames` says how far it got -- the caller must be able to tell this
        apart from a completed run, since committing a partial map as a current
        sidecar would present two thirds of a room as the whole thing.

        Reuses the LAST PUBLISHED mesh rather than extracting a fresh one, and
        that is load-bearing: a map sitting against its capacity wall cannot be
        safely extracted. Asking for one here segfaults inside Open3D's host
        marching cubes (verified on DebugCapB1 at the Detailed preset: guard
        fires at frame 3038, `mapper.mesh()` immediately after dies in
        `_extract`), which would swap the un-catchable crash we just prevented
        for a different un-catchable crash. The previous mesh is at most
        `mesh_every` frames old and was extracted while the map was healthy."""
        with self._lock:
            previous_mesh = self._latest.mesh if self._latest is not None else None
            self._latest = Progress(
                fraction=(frames_done / total) if total else 1.0,
                mesh=previous_mesh if previous_mesh is not None else _empty_mesh(),
                trajectory=list(mapper.trajectory),
                done=True,
                stats={"frames": frames_done, "error": str(exc)},
                phase="failed",
            )

    def _publish_construction_failure(self) -> None:
        """Terminal, zero-progress publish for a failure so total there's no
        `Mapper` to even ask for stats -- e.g. width/height are None because
        `_load_frames` never decoded a single depth frame (a capture started
        mid-stream, after the device's one-time CALIB frame had already gone
        by, has nothing to transform raw frames with -- see
        panel.py's `_enter_showcase_recording`, which re-requests CALIB on
        Record for exactly this reason). Still publishes so a caller's
        `latest()` is never left stuck at None with the thread already
        dead."""
        with self._lock:
            self._latest = Progress(
                fraction=1.0, mesh=_empty_mesh(), trajectory=[], done=True,
                stats={"frames": 0, "gap_m": 0.0, "path_m": 0.0, "verts": 0, "lost": 0},
            )

    def run(self) -> None:
        """Synchronous full run over every frame -- what `start()` runs in
        the background thread, and what tests call directly for
        determinism. Safe to call on an empty frame list."""
        total = len(self._frames)
        try:
            mapper = Mapper(self._width, self._height, **self._mapper_kwargs)
        except Exception:
            # Belt-and-braces, same spirit as the per-frame guard below:
            # construction itself can fail (see _publish_construction_failure)
            # and must not kill this thread silently.
            self._publish_construction_failure()
            return
        if total == 0:
            self._publish(mapper, 0, 0, done=True)
            return
        published_final = False
        for i, (depth, reflectance, confidence, quat, pressure, _t_s) in enumerate(self._frames, start=1):
            if self._stop_evt.is_set():
                return   # stopping mid-run: publish nothing further
            imu_raw = offset = None
            if self._imu_aux is not None and i - 1 < len(self._imu_aux):
                imu_raw, offset = self._imu_aux[i - 1]
            try:
                mapper.step(depth, quat, pressure, reflectance=reflectance, confidence=confidence,
                            imu_raw=imu_raw, quat_offset_us=offset)
            except TsdfCapacityError as exc:
                # NOT a bad frame -- the map has outgrown the device and every
                # remaining frame would raise the same thing. `continue` here
                # would spin through the rest of the capture doing nothing and
                # then present the partial map as a finished one, so stop and
                # publish the reason instead.
                self._publish_capacity_failure(mapper, i, total, exc)
                return
            except Exception:
                # Belt-and-braces (Mapper.step already degrades tracking-lost
                # gracefully on its own): one bad frame must not kill this
                # thread silently and leave `latest()` stuck mid-progress.
                continue
            is_last = i == total
            if is_last or i % self._mesh_every == 0:
                self._publish_extracting_mesh(mapper, i, total)
                try:
                    self._publish(mapper, i, total, done=is_last)
                except TsdfCapacityError as exc:
                    # The map has grown past what Open3D can extract. Every
                    # later publish would hit the same wall and the build's
                    # whole product is that final mesh, so stop here rather
                    # than integrate 1800 more frames toward nothing.
                    self._publish_capacity_failure(mapper, i, total, exc)
                    return
                published_final = published_final or is_last
        if not published_final:
            # Every remaining frame (at least the last one) raised, so the
            # loop above never hit its is_last publish -- still publish a
            # terminal result so a caller's latest() is never left stuck
            # mid-progress (or None) with the thread already dead.
            self._publish_extracting_mesh(mapper, total, total)
            self._publish(mapper, total, total, done=True)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
