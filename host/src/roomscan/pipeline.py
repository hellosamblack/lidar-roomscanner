"""Live PC-transform pipeline stage: turns decoded Frames into depth arrays.

Bridges the wire protocol (RAW_3DMD + CALIB streams, Task 2/5 firmware) and the
native transform (Task 3's roomscan.native.Transform) so the viewer can render
raw-only recordings/live streams the same way it already renders Phase 1's
on-device DEPTH_ZF32 frames. See host/src/roomscan/native.py for the DLL wrapper
and host/tests/golden.py for the fixture this stage's tests draw on.
"""
from __future__ import annotations

import numpy as np

from .native import Transform
from .protocol import Frame, FrameHeader, FrameType, StreamId


class TransformStage:
    """Feeds on decoded Frames; turns RAW_3DMD into depth arrays via the native transform.

    - CALIB frame: creates/keeps the Transform (first CALIB wins; identical repeats
      ignored; a DIFFERENT calib payload replaces the Transform -- new sensor/boot:
      the old handle is destroyed and a fresh one built, resetting the transform's
      internal TNR state).
    - RAW frame before any CALIB seen: counted in .raw_skipped_awaiting_calib, dropped
      (returns None).
    - RAW frame after CALIB: returns (header, depth ndarray (42, 54) f32).
    - DEPTH_ZF32 frame: returns (header, decoded ndarray (h, w) f32) -- Phase 1
      passthrough, works with no DLL and no CALIB.
    - Everything else (STATUS/AMBIENT/... DATA streams, non-DATA frame types): None.

    Construction is cheap and never touches the DLL -- the Transform is only built
    lazily on the first CALIB frame, so replay of depth-only (Phase 1) recordings
    never needs roomscan_transform.dll. A RuntimeError from Transform's own
    constructor (DLL not built) propagates out of feed() at that point.
    """

    def __init__(self):
        self._transform: Transform | None = None
        self._calib_payload: bytes | None = None
        self.raw_skipped_awaiting_calib = 0
        self.raw_transformed = 0

    @property
    def active(self) -> bool:
        """True once a Transform has been constructed from a CALIB frame."""
        return self._transform is not None

    def feed(self, frame: Frame) -> tuple[FrameHeader, np.ndarray] | None:
        header = frame.header
        if header.frame_type != FrameType.DATA:
            return None

        if header.stream_id == StreamId.CALIB:
            self._on_calib(frame.payload)
            return None

        if header.stream_id == StreamId.RAW_3DMD:
            if self._transform is None:
                self.raw_skipped_awaiting_calib += 1
                return None
            depth = self._transform.process(frame.payload)
            self.raw_transformed += 1
            return header, depth

        if header.stream_id == StreamId.DEPTH_ZF32:
            depth = np.frombuffer(frame.payload, dtype="<f4").reshape(header.height, header.width)
            return header, depth

        return None

    def _on_calib(self, payload: bytes) -> None:
        if payload == self._calib_payload:
            return  # identical repeat: keep the existing Transform (and its TNR state)
        old = self._transform
        self._transform = Transform(payload)  # may raise RuntimeError if DLL not built
        self._calib_payload = payload
        if old is not None:
            old.destroy()
