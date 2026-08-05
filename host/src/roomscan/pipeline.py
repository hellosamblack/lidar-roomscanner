"""Live PC-transform pipeline stage: turns decoded Frames into depth arrays.

Bridges the wire protocol (RAW_3DMD + CALIB streams, Task 2/5 firmware) and the
native transform (Task 3's roomscan.native.Transform) so the viewer can render
raw-only recordings/live streams the same way it already renders Phase 1's
on-device DEPTH_ZF32 frames. See host/src/roomscan/native.py for the DLL wrapper
and host/tests/golden.py for the fixture this stage's tests draw on.
"""
from __future__ import annotations

import numpy as np

from .flatfield import FlatField, FlatFieldSet
from .native import Transform
from .protocol import Frame, FrameHeader, FrameType, StreamId


class TransformStage:
    """Feeds on decoded Frames; turns RAW_3DMD into output arrays via the native transform.

    - CALIB frame: creates/keeps the Transform (first CALIB wins; identical repeats
      ignored; a DIFFERENT calib payload replaces the Transform -- new sensor/boot:
      the old handle is destroyed and a fresh one built, resetting the transform's
      internal TNR state).
    - RAW frame before any CALIB seen: counted in .raw_skipped_awaiting_calib, dropped
      (returns None).
    - RAW frame after CALIB: returns (header, {name: ndarray}) for every name in
      `outputs` (see roomscan.native.Transform for per-output dtypes/shapes).
    - DEPTH_ZF32 frame: returns (header, {"depth": decoded ndarray (h, w) f32}) --
      Phase 1 passthrough, works with no DLL and no CALIB.
    - Everything else (STATUS/AMBIENT/... DATA streams, non-DATA frame types): None.

    Construction is cheap and never touches the DLL -- the Transform is only built
    lazily on the first CALIB frame, so replay of depth-only (Phase 1) recordings
    never needs roomscan_transform.dll. A RuntimeError from Transform's own
    constructor (DLL not built) propagates out of feed() at that point.
    """

    def __init__(self, outputs: tuple[str, ...] = ("depth",),
                 flatfield: "FlatField | FlatFieldSet | None" = None,
                 flatfield_enabled: bool = True):
        self._outputs = tuple(outputs)
        self._transform: Transform | None = None
        self._calib_payload: bytes | None = None
        # Reflectance FPN correction: a single `FlatField` (legacy: applied to every
        # frame), a `FlatFieldSet` (mode-aware: the map for the current ranging mode),
        # or None. `flatfield_enabled` is the runtime "Calibrated" toggle and
        # `ranging_mode` (0 Precision / 1 Ambient) selects within a set -- both plain
        # attributes the owner (web server) writes from another thread; the reader
        # thread reads them each frame, one-frame-stale at worst, which is harmless.
        self._flatfield = flatfield
        self.flatfield_enabled = flatfield_enabled
        self.ranging_mode: int | None = None
        self.raw_skipped_awaiting_calib = 0
        self.raw_transformed = 0

    @property
    def active(self) -> bool:
        """True once a Transform has been constructed from a CALIB frame."""
        return self._transform is not None

    @property
    def has_flatfield(self) -> bool:
        """True when a correction map is configured (regardless of the enable
        toggle) -- lets the UI show whether calibration is even available."""
        ff = self._flatfield
        if isinstance(ff, FlatFieldSet):
            return not ff.is_empty
        return ff is not None

    def set_ranging_mode(self, ranging_mode: int | None) -> None:
        """Point a mode-aware `FlatFieldSet` at the device's current ranging mode
        (called from the ranging-config ACK commit). No-op for a single map."""
        self.ranging_mode = int(ranging_mode) if ranging_mode is not None else None

    def _active_flatfield(self) -> "FlatField | None":
        """The map to apply this frame, honouring the enable toggle and, for a
        set, the current ranging mode. None => leave reflectance uncorrected."""
        ff = self._flatfield
        if ff is None or not self.flatfield_enabled:
            return None
        if isinstance(ff, FlatFieldSet):
            return ff.for_mode(self.ranging_mode)
        return ff

    def feed(self, frame: Frame) -> tuple[FrameHeader, dict[str, np.ndarray]] | None:
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
            outputs = self._transform.process(frame.payload)
            active_ff = self._active_flatfield()
            if active_ff is not None and "reflectance" in outputs:
                outputs["reflectance"] = active_ff.apply(outputs["reflectance"])
            self.raw_transformed += 1
            return header, outputs

        if header.stream_id == StreamId.DEPTH_ZF32:
            depth = np.frombuffer(frame.payload, dtype="<f4").reshape(header.height, header.width)
            return header, {"depth": depth}

        return None

    def _on_calib(self, payload: bytes) -> None:
        if payload == self._calib_payload:
            return  # identical repeat: keep the existing Transform (and its TNR state)
        old = self._transform
        self._transform = Transform(payload, outputs=self._outputs)  # may raise RuntimeError if DLL not built
        self._calib_payload = payload
        if old is not None:
            old.destroy()
