"""Stationarity detection for the SLAM translation estimate.

The 3-DoF point-to-plane ICP (odometry.py) produces a per-frame translation
estimate that, on a genuinely stationary sensor, is not zero but ~1-4 cm of
zero-mean noise on the 54x42 ToF depth. Because `Mapper` accumulates each
frame's translation into `_t_prev`, that noise random-walks the position even
with the device sitting still on a tripod -- the visible "jitter" the owner
reported.

A fixed magnitude deadband can't fix this: stationary jitter (~11-45 mm/frame)
overlaps real slow motion (~35 mm/frame at a gentle walking pace), so any
threshold that suppresses the jitter also eats real motion -- unacceptable when
the final reconstruction accuracy is the priority.

The discriminator that *does* separate them is directional COHERENCE. Real
motion is directionally consistent: the net displacement over a short window is
close to the summed path length (coherence -> 1). Zero-mean ICP jitter points
every which way, so the increments largely cancel: net << path
(coherence -> 1/sqrt(window)). Gating on coherence holds the pose only when the
recent motion is incoherent jitter, and lets any coherent motion through
untouched.
"""
from __future__ import annotations

from collections import deque

import numpy as np


def coherence(increments) -> float:
    """Directional coherence of a sequence of 3D displacement increments:
    ``||sum(inc)|| / sum(||inc||)``.

    - ~1.0 for consistent straight-line motion (increments reinforce),
    - ~1/sqrt(N) for N zero-mean random jitter steps (increments cancel),
    - 0.0 for no motion at all (all increments zero),
    - 1.0 for an empty set (no evidence of jitter -> treat as "moving", i.e.
      never suppress on no data).
    """
    inc = np.asarray(increments, dtype=np.float64).reshape(-1, 3)
    if inc.shape[0] == 0:
        return 1.0
    path = float(np.linalg.norm(inc, axis=1).sum())
    if path < 1e-9:
        return 0.0
    net = float(np.linalg.norm(inc.sum(axis=0)))
    return net / path


class StationarityGate:
    """Coherence-gated stationary detector for the per-frame ICP translation.

    Feed the RAW (ungated) world-frame position increment each frame via
    :meth:`update`; it returns True when the sensor is effectively still and
    the caller should HOLD the pose (freeze translation) to stop the estimate
    random-walking. Always feed the raw ICP estimate -- never the held/gated
    value -- or the gate can never observe motion resuming.

    A frame is "stationary" only when ALL of these hold over the trailing
    `window`:
      * mean per-frame ROTATION is small (<= `rot_ceiling_deg`) -- during a
        real scan the user is almost always rotating the sensor to aim at the
        scene, so any appreciable rotation means "actively scanning, not
        still". This is the signal that separates a tripod (rotation ~0) from a
        handheld scan even when the translation looks jittery; without it,
        coherence alone misfires on a scan's curved path and eats real motion.
      * mean per-frame TRANSLATION step is small (<= `step_ceiling_m`) -- large
        strides are motion regardless of coherence, and large *incoherent*
        jumps are tracking trouble we must not silently hide.
      * directional coherence < `coherence_thresh` -- the increments cancel,
        i.e. zero-mean jitter rather than travel.

    The window must be full before the gate can trip, so motion is never
    suppressed at startup. `window=10, coherence_thresh=0.5` gives a stationary
    coherence of ~1/sqrt(10)=0.32 (well under 0.5) vs. ~1.0 for straight
    motion. Rotation is a per-frame angular delta in degrees (fixed frame rate,
    so deg/frame is proportional to deg/s); a tripod sits at ~0.03-0.08
    deg/frame vs. ~1 deg/frame while scanning, so `rot_ceiling_deg=0.3` has a
    wide margin either way.
    """

    def __init__(self, window: int = 10, coherence_thresh: float = 0.5,
                 step_ceiling_m: float = 0.03, rot_ceiling_deg: float = 0.3):
        self.window = int(window)
        self.coherence_thresh = float(coherence_thresh)
        self.step_ceiling_m = float(step_ceiling_m)
        self.rot_ceiling_deg = float(rot_ceiling_deg)
        self._hist: deque = deque(maxlen=self.window)
        self._rot: deque = deque(maxlen=self.window)

    def update(self, increment, rot_delta_deg: float = 0.0) -> bool:
        inc = np.asarray(increment, dtype=np.float64).reshape(3)
        self._hist.append(inc)
        self._rot.append(float(rot_delta_deg))
        if len(self._hist) < self.window:
            return False
        if float(np.mean(self._rot)) > self.rot_ceiling_deg:
            return False
        arr = np.array(self._hist)
        mean_step = float(np.linalg.norm(arr, axis=1).mean())
        if mean_step > self.step_ceiling_m:
            return False
        return coherence(arr) < self.coherence_thresh

    def reset(self) -> None:
        self._hist.clear()
        self._rot.clear()
