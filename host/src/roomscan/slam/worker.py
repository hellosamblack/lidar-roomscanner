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

import threading
import time

from .mapper import Mapper

_MESH_EVERY = 5           # mesh extraction is the expensive part of a step; throttle it
_IDLE_SLEEP_S = 0.005     # poll interval when the submit slot is empty


class SlamWorker:
    """Owns a `Mapper` and runs it off the GUI/reader threads.

    `submit(depth, quat, pressure, reflectance=None, confidence=None)` stores
    the latest input (dropping any older, unprocessed one) -- reflectance/
    confidence are optional (Task 13) and simply forwarded to `Mapper.step`;
    the live panel does not yet supply them (a follow-up task wires that), so
    today they default to None and the live preview stays uncolored/
    ungated, unchanged from before this task. `run_once()` pops it, steps the mapper, and
    publishes `(mesh, trajectory, FrameStep)` -- mesh extraction is throttled
    to every `mesh_every` *successful* (non-tracking-lost) frames, since only
    those integrate into the TSDF (always on the first success, so a caller
    sees geometry as soon as there is any); trajectory + step publish every
    frame regardless, so the HUD keeps updating even while tracking-lost.
    `start()`/`stop()` run `run_once()` in a background loop, mirroring
    `panel.py`'s `_run_reader` lifecycle (daemon thread, joined on stop).
    """

    def __init__(self, width: int, height: int, mesh_every: int = _MESH_EVERY,
                 **mapper_kwargs):
        self._mapper = Mapper(width, height, **mapper_kwargs)
        self._mesh_every = max(1, int(mesh_every))
        self._frames_processed = 0
        self._frames_integrated = 0     # successful (non-tracking-lost) frames only
        self._last_mesh = None

        self._in_lock = threading.Lock()
        self._in_slot = None    # (depth, quat, pressure, reflectance, confidence) | None

        self._out_lock = threading.Lock()
        self._out_slot = None   # (mesh, trajectory, FrameStep) | None

        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    # ---- producer side (GUI/reader thread) ----------------------------------
    def submit(self, depth, quat, pressure, reflectance=None, confidence=None) -> None:
        with self._in_lock:
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
        if not step.tracking_lost:
            # Only a successful (integrated) frame can have changed the TSDF,
            # and only then is `mesh()` guaranteed non-empty -- gate the
            # throttle on this count, not on frames merely processed. A
            # tracking-lost frame still publishes below (HUD keeps updating),
            # it just doesn't force a fresh (and, on a still-empty map,
            # pointless) mesh extraction. See tsdf.py's own empty-map guard
            # for the belt-and-braces backstop.
            self._frames_integrated += 1
            if self._frames_integrated == 1 or self._frames_integrated % self._mesh_every == 0:
                self._last_mesh = self._mapper.mesh()
        trajectory = list(self._mapper.trajectory)   # copy: caller must not see it mutate later
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
