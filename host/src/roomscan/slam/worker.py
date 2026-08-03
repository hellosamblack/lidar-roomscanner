"""Background SLAM worker (Task 10). Decouples per-frame `Mapper.step` work
from both the panel's reader thread (serial IO) and its GUI thread (rendering),
via two lock-guarded latest-wins slots -- the same pattern `panel.py`'s reader
already uses for its `queue.Queue(maxsize=1)` render slot (see `_run_reader`,
panel.py:197): a producer overwrites the single pending item so a slow
consumer only ever sees the newest input, never a backlog.

Threading contract (mirrors panel.py's, docs at the top of that file):
  * `submit()` is called from whatever thread has the newest depth/quat/
    pressure (the GUI tick, in panel.py's wiring) -- cheap, lock-guarded,
    never blocks.
  * The worker's own thread (started via `start()`) pops the latest submitted
    input and runs `Mapper.step` on it, which can take tens of ms -- this
    must never happen on the reader or GUI thread.
  * `latest()` is called from the GUI thread to fetch the newest published
    result; never blocks.
  * No serial writes happen on this thread, ever.
"""
from __future__ import annotations

import logging
import threading
import time

from .mapper import Mapper
from .tsdf import TsdfCapacityError

_MESH_EVERY = 5           # mesh extraction is the expensive part of a step; throttle it
_IDLE_SLEEP_S = 0.005     # poll interval when the submit slot is empty


class SlamWorker:
    """Owns a `Mapper` and runs it off the GUI/reader threads.

    `submit(depth, quat, pressure, reflectance=None, confidence=None)` stores
    the latest input (dropping any older, unprocessed one) -- reflectance/
    confidence are optional (Task 13) and simply forwarded to `Mapper.step`;
    the live panel does not yet supply them (a follow-up task wires that), so
    today they default to None and the live preview stays uncolored/
    ungated, unchanged from before this task. `run_once()` pops it, steps the
    mapper, and publishes `(mesh, trajectory, FrameStep)` to `latest()` --
    TWICE, if extraction runs: once immediately with the *previous* mesh
    (fresh pose, stale mesh, so a caller is never held up by extraction) and
    again with the fresh mesh once `mapper.mesh()` returns (BUG-061). Mesh
    extraction itself is throttled to every `mesh_every` *successful*
    (non-tracking-lost) frames, since only those integrate into the TSDF
    (always on the first success, so a caller sees geometry as soon as there
    is any); trajectory + step publish every frame regardless, so the HUD
    keeps updating even while tracking-lost.
    `start()`/`stop()` run `run_once()` in a background loop, mirroring
    `panel.py`'s `_run_reader` lifecycle (daemon thread, joined on stop).
    """

    def __init__(self, width: int, height: int, mesh_every: int = _MESH_EVERY,
                 **mapper_kwargs):
        self._mapper = Mapper(width, height, **mapper_kwargs)
        self._mesh_every = max(1, int(mesh_every))
        self._frames_processed = 0
        self._frames_integrated = 0     # successful (non-tracking-lost) frames only
        # Plan item 2 (2026-08-02): submit()/overwrite counters. `submit()`
        # forwards into a size-one latest-wins slot (module docstring); every
        # submit either lands in an empty slot or REPLACES a still-pending one
        # -- the latter is an overwrite and never reaches `Mapper.step` at
        # all, so it must not be confused with `Mapper.tracking_lost_count`
        # (a frame that DID reach the mapper and failed to register). At any
        # instant: frames_submitted == frames_processed + frames_overwritten
        # + (1 if a frame is currently sitting in the slot, unprocessed, else 0).
        self._frames_submitted = 0
        self._frames_overwritten = 0
        self._last_mesh = None
        self._last_mesh_extract_ms = 0.0   # time in the last Mapper.mesh() call
        # Set once if the map grows past what Open3D can extract; the live view
        # then holds `_last_mesh`. See run_once.
        self.extraction_blocked_reason: str | None = None

        self._in_lock = threading.Lock()
        self._in_slot = None    # (depth, quat, pressure, reflectance, confidence) | None

        self._out_lock = threading.Lock()
        self._out_slot = None   # (mesh, trajectory, FrameStep) | None

        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    # ---- producer side (GUI/reader thread) ----------------------------------
    def submit(self, depth, quat, pressure, reflectance=None, confidence=None) -> None:
        with self._in_lock:
            self._frames_submitted += 1
            if self._in_slot is not None:
                self._frames_overwritten += 1
            self._in_slot = (depth, quat, pressure, reflectance, confidence)

    # ---- worker side ---------------------------------------------------------
    def run_once(self) -> bool:
        """Pop the latest submitted input (if any) and process it. Returns
        True if a frame was processed, False if the input slot was empty.
        Synchronous -- this is what tests call directly, no thread needed."""
        with self._in_lock:
            item, self._in_slot = self._in_slot, None
        if item is None:
            return False
        depth, quat, pressure, reflectance, confidence = item
        step = self._mapper.step(depth, quat, pressure, reflectance=reflectance, confidence=confidence)
        self._frames_processed += 1
        trajectory = list(self._mapper.trajectory)   # copy: caller must not see it mutate later
        # Publish the pose the instant it's known -- stale mesh, fresh pose --
        # so a caller never waits out the extraction below (tens of ms) to see
        # where the sensor is (BUG-061: this used to be the only publish, and
        # it sat after extraction, so the pose lagged by however long that
        # took).
        with self._out_lock:
            self._out_slot = (self._last_mesh, trajectory, step)
        if not step.tracking_lost:
            # Only a successful (integrated) frame can have changed the TSDF,
            # and only then is `mesh()` guaranteed non-empty -- gate the
            # throttle on this count, not on frames merely processed. A
            # tracking-lost frame still published above (HUD keeps updating),
            # it just doesn't force a fresh (and, on a still-empty map,
            # pointless) mesh extraction. See tsdf.py's own empty-map guard
            # for the belt-and-braces backstop.
            self._frames_integrated += 1
            if self._frames_integrated == 1 or self._frames_integrated % self._mesh_every == 0:
                t_extract0 = time.perf_counter()
                try:
                    self._last_mesh = self._mapper.mesh()
                except TsdfCapacityError as exc:
                    # The map has outgrown what Open3D can extract (tsdf.py's
                    # `_MAX_SAFE_EXTRACT_BLOCKS`). Unlike the offline builder,
                    # a LIVE scan must not stop: tracking and integration are
                    # unaffected, and the operator is mid-sweep with an
                    # unrepeatable capture. Hold the last good mesh so the view
                    # freezes rather than the server dying, and say so once --
                    # a silently-frozen map is exactly the BUG-035 failure.
                    self._last_mesh_extract_ms = (time.perf_counter() - t_extract0) * 1000.0
                    if self.extraction_blocked_reason is None:
                        self.extraction_blocked_reason = str(exc)
                        logging.getLogger(__name__).warning(
                            "[slam] live map view frozen: %s", exc)
                else:
                    self._last_mesh_extract_ms = (time.perf_counter() - t_extract0) * 1000.0
                    # Republish: trajectory/step are unchanged from the publish
                    # above, but the mesh just got fresher.
                    with self._out_lock:
                        self._out_slot = (self._last_mesh, trajectory, step)
        return True

    def latest(self):
        """Latest published `(mesh, trajectory, FrameStep)`, or None before
        the first processed frame."""
        with self._out_lock:
            return self._out_slot

    @property
    def tracking_lost_count(self) -> int:
        return self._mapper.tracking_lost_count

    # ---- instrumentation (plan item 2, 2026-08-02) ---------------------------
    @property
    def frames_submitted(self) -> int:
        """Total `submit()` calls -- includes ones later overwritten."""
        return self._frames_submitted

    @property
    def frames_processed(self) -> int:
        """Frames that actually reached `Mapper.step` (popped by `run_once`).
        Includes tracking-lost frames -- see `Mapper.lost_flags`/
        `tracking_lost_count` for that distinction; this counter only answers
        "did it reach the mapper at all", never "did it register"."""
        return self._frames_processed

    @property
    def frames_overwritten(self) -> int:
        """Submits that replaced a still-pending, not-yet-processed input --
        i.e. frames that NEVER reached `Mapper.step`. Do not fold these into
        `Mapper.tracking_lost_count`/`lost_flags`: a tracking-lost frame was
        run through the mapper and failed to register; an overwritten one was
        never run at all."""
        return self._frames_overwritten

    @property
    def mesh_extract_ms(self) -> float:
        """Wall time of the most recent `Mapper.mesh()` call (extraction),
        whether it succeeded or raised `TsdfCapacityError`. 0.0 before the
        first extraction. This IS a synchronous measurement even on CUDA --
        `TsdfMap._extract()` always returns a host-resident result (see its
        docstring), so there is no pending device work left when this timer
        stops."""
        return self._last_mesh_extract_ms

    @property
    def device(self) -> str:
        """The Mapper's own resolved compute device string -- see
        `Mapper.device`'s docstring for why this must be read from the built
        object rather than re-inferred by a caller."""
        return self._mapper.device

    @property
    def icp_device(self) -> str:
        """The Mapper's resolved ICP nearest-neighbour device (item 5,
        2026-08-02) -- usually "CPU:0" while `device` is "CUDA:0". Read from
        the built object for the same reason as `device`: the point of the
        knob is that it can be configured, so the only trustworthy report is
        the one the object gives about itself."""
        return self._mapper.icp_device

    @property
    def backend(self) -> str:
        """"local": this worker runs `Mapper.step` in-process. Contrast
        `RemoteSlamWorker.backend == "remote"`."""
        return "local"

    # ---- lifecycle (mirrors panel.py's _run_reader thread) -------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self) -> None:
        while not self._stop_evt.is_set():
            if not self.run_once():
                time.sleep(_IDLE_SLEEP_S)

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None
