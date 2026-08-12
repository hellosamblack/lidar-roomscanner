"""Timestamped orientation samples on the LSM tick clock (issue #155).

Stream 9's SFLP quaternion is a FIFO-batch MEAN: it carries the batch's midpoint
orientation, which sits several ms AFTER the depth frame's FRAME_READY edge
(+7.76 ms on the golden capture, +5.13 ms re-measured on DebugCapF — the phase is
not a constant, see BUG-031 / #126 / #155). Stream 13 places both instants on the
LSM's own uint32 tick clock: `quat_mid_ticks` for the quaternion, and
`ImuSync.frame_ready_ticks()` for the depth frame. This module holds the pieces
needed to resolve that skew by construction instead of correcting it by a
constant: wrap-safe tick arithmetic, the canonical hemisphere-correct `slerp`,
and a bounded buffer that answers "what was the orientation at tick T?" by
interpolating between the two samples that bracket T.

Deliberately lightweight: numpy only, no Open3D — `roomscan.slam.frames`
re-exports `slerp` from here, not the other way round, so a future live consumer
(web UI, sensors) can interpolate without acquiring the SLAM stack.
"""
from __future__ import annotations

from collections import deque

import numpy as np

#: The LSM6DSV16X TIMESTAMP register is a free-running uint32 (~21.7 us/LSB,
#: wraps every ~26 h). All deltas below are modular so the wrap is a non-event.
TICK_MASK = 0xFFFFFFFF
_TICK_SPAN = float(TICK_MASK) + 1.0


def signed_tick_delta(a: float, b: float) -> float:
    """Signed tick delta a -> b, choosing the nearest modular representative.

    The intervals this module reasons about are milliseconds on a ~26 h counter,
    so the nearest representative is unambiguous by ~7 orders of magnitude.
    Accepts floats: stream 13's frame-ready instant is a fractional tick
    (`lsm_ticks - latch_delay_us / tick_us`)."""
    d = (float(b) - float(a)) % _TICK_SPAN
    if d > _TICK_SPAN / 2.0:
        d -= _TICK_SPAN
    return d


def slerp(a, b, t: float) -> tuple[float, float, float, float]:
    """Spherical linear interpolation between unit quaternions [w,x,y,z], from
    `a` at t=0 to `b` at t=1. Falls back to a normalized lerp when the two are
    nearly parallel (numerically safer, and the regime the SLAM prior smoother
    lives in). Hemisphere-corrects so it takes the short arc."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        out = a + t * (b - a)
    else:
        theta = np.arccos(max(-1.0, min(1.0, dot)))
        s = np.sin(theta)
        out = (np.sin((1.0 - t) * theta) / s) * a + (np.sin(t * theta) / s) * b
    n = np.linalg.norm(out)
    out = out / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])
    return (float(out[0]), float(out[1]), float(out[2]), float(out[3]))


def _valid_quat(q) -> tuple[float, float, float, float] | None:
    """Normalize q ([w,x,y,z]) or return None for non-finite/degenerate input."""
    try:
        w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    except (TypeError, ValueError, IndexError):
        return None
    n2 = w * w + x * x + y * y + z * z
    if not np.isfinite(n2) or n2 < 1e-12:
        return None
    n = n2 ** 0.5
    return (w / n, x / n, y / n, z / n)


class TimestampedQuaternionBuffer:
    """Bounded history of (LSM tick, quaternion) samples with interpolated lookup.

    Modeled on RTAB-Map CameraMobile's pose buffer (#155), sized for our actual
    problem: resolving a one-frame phase relationship needs the current and
    previous samples, not a thousand. Ticks are kept on an internally unwrapped
    monotone timeline so the uint32 rollover is invisible to lookups.

    Policies (each pinned by tests/test_sensor_time.py):
    - `add` rejects (returns False) non-finite/degenerate quaternions and any
      sample not strictly after the newest accepted one — duplicates and
      out-of-order arrivals keep the first-seen sample.
    - `at` returns the exact stored quaternion on an exact tick hit; a SLERP
      between the bracketing pair otherwise; and None when the query falls
      before the oldest / after the newest sample (NEVER extrapolates) or when
      the bracketing pair is further apart than `max_span_ticks` (interpolating
      across a multi-frame hole is smoothing, not phase correction — the exact
      failure mode this mechanism exists to avoid, see BUG-067).
    """

    def __init__(self, capacity: int = 64, max_span_ticks: float | None = None):
        self._samples: deque[tuple[float, tuple[float, float, float, float]]] = \
            deque(maxlen=int(capacity))
        self._last_raw: float | None = None
        self.max_span_ticks = max_span_ticks

    def __len__(self) -> int:
        return len(self._samples)

    def clear(self) -> None:
        self._samples.clear()
        self._last_raw = None

    def add(self, ticks: float, quat) -> bool:
        """Insert a sample at raw (possibly wrapped) LSM tick `ticks`."""
        q = _valid_quat(quat)
        if q is None or not np.isfinite(float(ticks)):
            return False
        if self._last_raw is None:
            self._samples.append((0.0, q))
        else:
            d = signed_tick_delta(self._last_raw, ticks)
            if d <= 0.0:
                return False
            self._samples.append((self._samples[-1][0] + d, q))
        self._last_raw = float(ticks) % _TICK_SPAN
        return True

    def _unwrap_query(self, ticks: float) -> float | None:
        if self._last_raw is None or not np.isfinite(float(ticks)):
            return None
        return self._samples[-1][0] + signed_tick_delta(self._last_raw, ticks)

    def at(self, ticks: float) -> tuple[float, float, float, float] | None:
        """Orientation at raw LSM tick `ticks`, or None if not bracketed."""
        t = self._unwrap_query(ticks)
        if t is None or len(self._samples) == 0:
            return None
        times = [s[0] for s in self._samples]
        if t < times[0] or t > times[-1]:
            return None
        # Bisect by hand: the deque is small (bounded) and always sorted.
        for i in range(len(times) - 1, -1, -1):
            if times[i] <= t:
                if times[i] == t:
                    return self._samples[i][1]
                lo_t, lo_q = self._samples[i]
                hi_t, hi_q = self._samples[i + 1]
                span = hi_t - lo_t
                if self.max_span_ticks is not None and span > self.max_span_ticks:
                    return None
                return slerp(lo_q, hi_q, (t - lo_t) / span)
        return None
