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
from .cli import _load_frames
from .mapper import Mapper

_MESH_EVERY = 25
_CPU = o3d.core.Device("CPU:0")


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


class PostProcessWorker:
    """Re-processes a recorded capture at full quality on a background
    thread. `frames` is the ctor arg (not a path) so this is unit-testable
    with synthetic data, matching `SlamWorker`'s pattern; `from_capture` is
    the convenience constructor for the live panel path."""

    def __init__(self, frames, width: int, height: int,
                 mesh_every: int = _MESH_EVERY, **mapper_kwargs):
        self._frames = frames
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
        frames, width, height = _load_frames(path)
        return cls(frames, width, height, mesh_every=mesh_every, **kw)

    # ---- reader side (GUI thread) --------------------------------------
    def latest(self) -> Progress | None:
        """Latest published `Progress`, or None before the first publish.
        Returns a fresh copy (trajectory list copied) so the caller can't
        observe it change out from under it."""
        with self._lock:
            p = self._latest
            if p is None:
                return None
            return Progress(p.fraction, p.mesh, list(p.trajectory), p.done, p.stats)

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
            }
        progress = Progress(
            fraction=(frames_done / total) if total else 1.0,
            mesh=mesh,
            trajectory=list(mapper.trajectory),
            done=done,
            stats=stats,
        )
        with self._lock:
            self._latest = progress

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
        for i, (depth, quat, pressure, _t_s) in enumerate(self._frames, start=1):
            if self._stop_evt.is_set():
                return   # stopping mid-run: publish nothing further
            try:
                mapper.step(depth, quat, pressure)
            except Exception:
                # Belt-and-braces (Mapper.step already degrades tracking-lost
                # gracefully on its own): one bad frame must not kill this
                # thread silently and leave `latest()` stuck mid-progress.
                continue
            is_last = i == total
            if is_last or i % self._mesh_every == 0:
                self._publish(mapper, i, total, done=is_last)
                published_final = published_final or is_last
        if not published_final:
            # Every remaining frame (at least the last one) raised, so the
            # loop above never hit its is_last publish -- still publish a
            # terminal result so a caller's latest() is never left stuck
            # mid-progress (or None) with the thread already dead.
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
