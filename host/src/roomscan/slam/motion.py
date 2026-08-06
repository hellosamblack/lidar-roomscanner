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

# `coherence` is generic (it also gates the web display's orientation smoother),
# so it lives in the Open3D-free `roomscan.motion`. Re-exported here because it
# is documented above as part of this module's story, and SLAM code imports it
# from here.
from ..motion import coherence

__all__ = ["coherence", "StationarityGate", "ZuptDetector"]

# Standard gravity, m/s^2. The accelerometer at rest reads one g of specific
# force regardless of orientation, so |a| ~= 1 g is the orientation-free "not
# translating" signal -- and it holds during a pure pan, unlike a rotation-gated
# stationarity test (BUG-069).
_G = 9.80665


class ZuptDetector:
    """Accelerometer zero-velocity detector for BUG-069.

    Feed each frame's specific-force magnitude (m/s^2, the norm of the mean raw
    accel over the frame's IMU batch) via :meth:`update`; returns True when the
    sensor is confidently NOT translating and the caller may apply a hard
    zero-velocity constraint to the *reconstruction* pose (not just the preview).

    This is the discriminator `StationarityGate` could not be. That gate keys on
    ICP translation coherence and is disabled above `rot_ceiling_deg` precisely so
    a scan's aiming rotation doesn't fool it -- which also makes it blind to a
    tripod pan (rotation, zero translation), the exact case BUG-067 fabricates
    from. The accelerometer sees the difference the LiDAR cannot: a pure rotation
    leaves |a| at 1 g (gravity only; centripetal a = w^2 r ~= 0.005 g at a cm
    lever arm and 1 rad/s), while walking adds gait specific force that pushes
    |a| off 1 g. So this fires DURING a pan and is deliberately NOT rotation-gated.

    A frame is a zero-velocity candidate when ``| |a| - g | <= accel_tol_g * g``.
    The verdict trips only after `window` consecutive candidates, so a transient
    (a footfall, a bump) cannot latch a hold, and it clears the instant one frame
    leaves the band.

    Accel alone is not enough, and the measurement proved it: steady walking is
    also ~1 g of specific force between footfalls, so an accel-only gate froze the
    pose mid-stride on a real room circuit (76 lost frames, a 59-frame freeze).
    The discriminator that separates a tripod PAN from a WALK is directional
    COHERENCE of the ICP translation: a walk's increments are directionally
    consistent (net ~ path, coherence -> 1), while a stationary sensor's ICP
    jitter points every which way and cancels (coherence -> 1/sqrt(window)). So
    the hold requires BOTH accel-still AND incoherent ICP translation -- feed the
    raw per-frame ICP increment via `increment=`. This is the accel signal
    supplying the pan-capable "not translating" evidence and the coherence signal
    vetoing genuine directed travel. `coherence_thresh <= 0` disables the veto
    (accel-only, the measured-unsafe mode, kept for A/B)."""

    def __init__(self, window: int = 6, accel_tol_g: float = 0.04,
                 coherence_thresh: float = 0.5):
        self.window = int(window)
        self.accel_tol_g = float(accel_tol_g)
        self.coherence_thresh = float(coherence_thresh)
        self._still = deque(maxlen=self.window)
        self._inc: deque = deque(maxlen=self.window)

    def update(self, accel_mag_mps2: float | None, increment=None) -> bool:
        if accel_mag_mps2 is None:
            self._still.clear()                     # no signal -> never hold
            self._inc.clear()
            return False
        candidate = abs(float(accel_mag_mps2) - _G) <= self.accel_tol_g * _G
        self._still.append(candidate)
        if increment is not None:
            self._inc.append(np.asarray(increment, dtype=np.float64).reshape(3))
        if len(self._still) < self.window or not all(self._still):
            return False
        # Translation-coherence veto: coherent recent motion is a real walk, not
        # a still sensor's jitter -- do not hold it even though accel reads 1 g.
        if self.coherence_thresh > 0.0 and len(self._inc) >= self.window:
            if coherence(np.array(self._inc)) >= self.coherence_thresh:
                return False
        return True

    def reset(self) -> None:
        self._still.clear()
        self._inc.clear()


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
