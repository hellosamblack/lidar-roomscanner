"""Transport-neutral reader loop + follow-camera math, shared by every frontend.

These three helpers -- the reader-thread body (`_run_reader`), the replay pacer
(`_Pacer`), and the follow-camera placement (`follow_camera_target`) -- are pure
data plumbing with NO Open3D / GUI dependency. They lived in `panel.py` while the
desktop panel was the only consumer; Web Phase 5 (retire `panel.py`) hoists them
here so the web server (`web.py`) can reuse them without importing the deprecated
GUI module. `panel.py` re-imports them from here, so `panel._run_reader` etc.
still resolve for the existing desktop code and its tests.

Names keep their original spelling (leading underscore on the two internals)
purely so the many `panel._run_reader` / `panel._Pacer` call-sites and tests keep
working unchanged; within this module they are the public surface.
"""
from __future__ import annotations

import queue
import threading
import time

import numpy as np

from .protocol import HEADER_SIZE, FrameType, ProtocolError, parse_event
from .sources import pump

# Fixed world-up convention (== slam.frames.world_up(), [0,-1,0]); the follow
# camera never tilts/rolls off it.
_WORLD_UP = np.array([0.0, -1.0, 0.0], dtype=np.float32)
_FOLLOW_BACK_OFF_M = 0.3
_FOLLOW_LOOK_AHEAD_M = 1.0


def follow_camera_target(pose, back_off: float = _FOLLOW_BACK_OFF_M,
                         look_ahead: float = _FOLLOW_LOOK_AHEAD_M, up=None):
    """(eye, center, up) camera placement for camera-follow mode (owner
    request: "make SLAM mode be from the perspective of the camera"). `eye`
    sits `back_off` metres BEHIND the sensor along -forward (a hair of
    context so the view isn't pinned exactly to the sensor's nose; pass 0 to
    put the eye exactly at the sensor position), `center` sits `look_ahead`
    metres AHEAD of the sensor along +forward (what `look_at` aims the
    camera at, so the view translates+rotates with the sensor as it's
    carried around), and `up` is the fixed world-up convention (`_WORLD_UP`
    == `slam.frames.world_up()`, `[0,-1,0]`) unless overridden.

    `pose` is a 4x4 world<-camera matrix, same convention as
    `capture_square_corners`/`_fov_frustum_lines`. Pure -- unit-tested; feeds
    `_apply_follow_camera`, which additionally smooths eye/center across
    ticks so per-frame pose noise doesn't jitter the view."""
    pose = np.asarray(pose, dtype=np.float64)
    sensor_pos = pose[:3, 3]
    forward = pose[:3, 2]
    if up is None:
        up = _WORLD_UP
    eye = sensor_pos - back_off * forward
    center = sensor_pos + look_ahead * forward
    return eye, center, np.asarray(up, dtype=np.float64)


class _Pacer:
    """Mutable replay-pacing + pause control shared with the reader thread.

    `interval` (seconds/frame, 0 = as-fast-as-decoded) is read live so the fps
    slider takes effect immediately; `paused` (an Event) blocks the reader
    between frames. Live capture leaves interval 0 and never pauses.
    """

    def __init__(self, interval: float = 0.0):
        self.interval = interval
        self.paused = threading.Event()


def _run_reader(source, decoder, stage, stats, slot, fault, bus, client, recorder,
                pacer, is_stopped, state=None, metrics=None):
    """Reader-thread body (module-level so it's unit-testable without a window).

    Owns source+decoder+transform; routes device EVENT -> log bus, ACK ->
    CommandClient, and each transformed DATA frame -> the latest-wins render
    slot. Honors the pacer's live `interval` (replay fps) and `paused` gate, and
    tees raw bytes into `recorder`. Any exception is surfaced via `fault` (unless
    we're stopping) exactly like the classic viewer's reader. `state` (a
    SensorState, optional -- defaults to None for callers that don't care about
    IMU/env streams, e.g. existing tests) is fed every DATA frame; it ignores
    any stream that isn't IMU_QUAT/ENV, mirroring `stage.feed`'s own filtering.
    """
    last_pace = 0.0
    last_paced_seq = None
    try:
        for frame in pump(source, decoder, recorder=recorder):
            if is_stopped():
                break
            ft = frame.header.frame_type
            if ft == FrameType.EVENT:
                try:
                    code, detail, msg = parse_event(frame.payload)
                    bus.publish(f"[event] code={code} detail={detail} {msg}")
                except ProtocolError:
                    bus.publish(f"[event] undecodable payload ({len(frame.payload)} B)")
                continue
            if ft == FrameType.ACK:
                if client is not None:
                    client.offer(frame)
                continue
            if ft != FrameType.DATA:
                continue
            if metrics is not None:
                # Feed every DATA frame (RAW/DEPTH/CALIB/IMU/ENV) so per-sensor
                # rates and link bandwidth see the full stream, not just the
                # frames that survive stage.feed's RAW->depth filter. Wire size
                # = header + payload + CRC32.
                metrics.record(frame.header, HEADER_SIZE + frame.header.payload_len + 4,
                               time.monotonic())
            if state is not None:
                try:
                    state.feed(frame)   # streams 9/10 -> SensorState; ignores others
                except Exception:
                    pass  # a malformed IMU/ENV payload must never kill the reader (ToF continues)
            result = stage.feed(frame)
            if result is None:
                continue
            header, outputs = result
            stats.update(header)
            while pacer.paused.is_set() and not is_stopped():
                time.sleep(0.05)
            if is_stopped():
                break
            interval = pacer.interval
            if interval > 0.0 and header.seq != last_paced_seq:
                wait = last_pace + interval - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                last_pace = time.monotonic()
                last_paced_seq = header.seq
            try:
                slot.get_nowait()
            except queue.Empty:
                pass
            slot.put((header, outputs))
    except Exception as exc:  # surface, don't vanish
        if not is_stopped():
            fault["error"] = exc
