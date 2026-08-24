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
from dataclasses import dataclass
from typing import Any, Sequence

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

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, TimestampedQuaternionBuffer):
            return False
        return (list(self._samples) == list(other._samples)
                and self._last_raw == other._last_raw
                and self.max_span_ticks == other.max_span_ticks
                and self._samples.maxlen == other._samples.maxlen)

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


def interpolate_quaternion_at(
    buffer_or_samples: TimestampedQuaternionBuffer | Sequence[tuple[float, Any]],
    target_ticks: float,
    max_span_ticks: float | None = None,
) -> tuple[float, float, float, float] | None:
    """Query orientation at target LSM tick, interpolating between bracketing samples.

    Accepts either a `TimestampedQuaternionBuffer` or a sequence of `(ticks, quat)`
    pairs. Returns None if the target is outside the bracket or separated by more
    than `max_span_ticks`."""
    if isinstance(buffer_or_samples, TimestampedQuaternionBuffer):
        return buffer_or_samples.at(target_ticks)
    buf = TimestampedQuaternionBuffer(
        capacity=max(64, len(buffer_or_samples)), max_span_ticks=max_span_ticks
    )
    for ticks, quat in buffer_or_samples:
        buf.add(ticks, quat)
    return buf.at(target_ticks)


@dataclass(frozen=True)
class PairedGroupFrame:
    header: Any
    outputs: dict[str, Any]
    quat: tuple[float, float, float, float] | None
    pressure_pa: float | None = None
    imu_raw: Any = None
    imu_sync: Any = None
    quat_offset_us: float | None = None
    t_device_us: int | float | None = None


class LiveGroupAggregator:
    """Pairs streaming depth frames with matching group sensor data in wire order (#181).

    On the wire, depth (stream 0/1) arrives before the group's orientation (stream 9),
    env (stream 10), raw IMU (stream 11), and sync (stream 13). LiveGroupAggregator
    holds the pending depth frame until the group's sensor frames arrive (or the next
    group begins), guaranteeing depth frames pair with their matching group quaternion
    rather than lagging by a full frame.

    When `apply_quat_phase` is enabled and stream 13 is present, queries
    `TimestampedQuaternionBuffer` at `sync.frame_ready_ticks()` to interpolate the
    orientation at the exact frame-ready edge.
    """

    def __init__(self, capacity: int = 64, max_span_ticks: float | None = None,
                 apply_quat_phase: bool = False, tick_us: float = 21.7):
        self.quat_buf = TimestampedQuaternionBuffer(capacity=capacity, max_span_ticks=max_span_ticks)
        self.apply_quat_phase = bool(apply_quat_phase)
        self.tick_us = float(tick_us)
        self._pending_depth: tuple[Any, dict[str, Any]] | None = None
        self._pending_seq: int | None = None
        self._latest_quat: tuple[float, float, float, float] | None = None
        self._latest_env_pa: float | None = None
        self._latest_raw: Any = None
        self._latest_sync: Any = None
        self._group_quats: dict[int, tuple[float, float, float, float]] = {}
        self._group_envs: dict[int, float] = {}
        self._group_raws: dict[int, Any] = {}
        self._group_syncs: dict[int, Any] = {}

    def clear(self) -> None:
        self.quat_buf.clear()
        self._pending_depth = None
        self._pending_seq = None
        self._latest_quat = None
        self._latest_env_pa = None
        self._latest_raw = None
        self._latest_sync = None
        self._group_quats.clear()
        self._group_envs.clear()
        self._group_raws.clear()
        self._group_syncs.clear()

    def add_quat(self, seq: int, quat: Any) -> None:
        q = _valid_quat(quat)
        if q is not None:
            self._latest_quat = q
            self._group_quats[seq] = q
            sync = self._group_syncs.get(seq)
            if sync is not None and getattr(sync, "valid", False) and getattr(sync, "quat_mid_ticks", None):
                self.quat_buf.add(sync.quat_mid_ticks, q)

    def add_env_pressure(self, seq: int, pressure_pa: float | None) -> None:
        if pressure_pa is not None:
            self._latest_env_pa = float(pressure_pa)
            self._group_envs[seq] = float(pressure_pa)

    def add_imu_raw(self, seq: int, imu_raw: Any) -> None:
        if imu_raw is not None:
            self._latest_raw = imu_raw
            self._group_raws[seq] = imu_raw

    def add_sync(self, seq: int, sync: Any) -> PairedGroupFrame | None:
        if sync is None:
            return None
        self._latest_sync = sync
        self._group_syncs[seq] = sync
        if getattr(sync, "valid", False) and getattr(sync, "quat_mid_ticks", None):
            q = self._group_quats.get(seq, self._latest_quat)
            if q is not None:
                self.quat_buf.add(sync.quat_mid_ticks, q)
        if self._pending_depth is not None and self._pending_seq == seq:
            return self._finalize_pending(seq)
        return None

    def add_depth(self, header: Any, outputs: dict[str, Any]) -> PairedGroupFrame | None:
        seq = getattr(header, "seq", None)
        emitted = None
        if self._pending_depth is not None and self._pending_seq != seq:
            emitted = self._finalize_pending(self._pending_seq)
        self._pending_depth = (header, outputs)
        self._pending_seq = seq
        return emitted

    def flush(self) -> PairedGroupFrame | None:
        if self._pending_depth is not None:
            return self._finalize_pending(self._pending_seq)
        return None

    def _finalize_pending(self, seq: int | None) -> PairedGroupFrame:
        header, outputs = self._pending_depth
        self._pending_depth = None
        self._pending_seq = None

        sync = self._group_syncs.pop(seq, self._latest_sync) if seq is not None else self._latest_sync
        group_q = self._group_quats.pop(seq, self._latest_quat) if seq is not None else self._latest_quat
        pressure = self._group_envs.pop(seq, self._latest_env_pa) if seq is not None else self._latest_env_pa
        raw = self._group_raws.pop(seq, self._latest_raw) if seq is not None else self._latest_raw

        # Prune old group entries
        if seq is not None:
            for d in (self._group_quats, self._group_envs, self._group_raws, self._group_syncs):
                for old_k in [k for k in d if k < seq - 10]:
                    d.pop(old_k, None)

        quat = None
        offset_us = None
        if sync is not None and getattr(sync, "valid", False):
            offset_fn = getattr(sync, "quat_offset_us", None)
            if offset_fn is not None:
                offset_us = offset_fn(self.tick_us)
            if self.apply_quat_phase:
                target_fn = getattr(sync, "frame_ready_ticks", None)
                if target_fn is not None:
                    target_ticks = target_fn(self.tick_us)
                    quat = self.quat_buf.at(target_ticks)
        if quat is None:
            quat = group_q

        t_us = getattr(header, "t_us", None)
        return PairedGroupFrame(
            header=header,
            outputs=outputs,
            quat=quat,
            pressure_pa=pressure,
            imu_raw=raw,
            imu_sync=sync,
            quat_offset_us=offset_us,
            t_device_us=t_us,
        )

    def feed_frame(self, frame: Any, outputs: dict[str, Any] | None = None) -> PairedGroupFrame | None:
        """Process one protocol Frame or transformed outputs."""
        hdr = getattr(frame, "header", frame)
        sid = getattr(hdr, "stream_id", None)
        seq = getattr(hdr, "seq", 0)
        from .protocol import StreamId, decode_env, decode_imu_quat, decode_imu_raw, decode_imu_sync
        if sid in (StreamId.RAW_3DMD, StreamId.DEPTH_ZF32) or outputs is not None:
            if outputs is not None:
                return self.add_depth(hdr, outputs)
            return None
        elif sid == StreamId.IMU_QUAT:
            q = decode_imu_quat(frame.payload)
            self.add_quat(seq, q)
        elif sid == StreamId.ENV:
            p, _mag, _t = decode_env(frame.payload)
            self.add_env_pressure(seq, p)
        elif sid == StreamId.IMU_RAW:
            batch = decode_imu_raw(frame.payload, self.tick_us)
            self.add_imu_raw(seq, batch)
        elif sid == StreamId.IMU_SYNC:
            sync = decode_imu_sync(frame.payload)
            return self.add_sync(seq, sync)
        return None

