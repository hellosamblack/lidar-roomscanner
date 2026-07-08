"""Wire protocol v1 — see docs/protocol.md. Keep in lockstep via protocol-change skill."""
from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from enum import IntEnum

MAGIC = b"RSCN"
VERSION = 1
HEADER_SIZE = 32
FLAG_DROPPED = 0x01

_HEADER = struct.Struct("<4sBBBBIQHHII")  # magic, ver, type, stream, flags, seq, t_us, w, h, plen, reserved
assert _HEADER.size == HEADER_SIZE


class FrameType(IntEnum):
    DATA = 1
    EVENT = 2


class StreamId(IntEnum):
    DEPTH_ZF32 = 0
    DEPTH_ZAPC = 1
    AMBIENT = 2
    AMPLITUDE = 3
    CONFIDENCE = 4
    REFLECTANCE = 5
    STATUS = 6
    RAW_3DMD = 7
    CALIB = 8


class EventCode(IntEnum):
    SENSOR_INIT_FAIL = 1
    TRIGGER_TIMEOUT = 2
    DMA_TIMEOUT = 3
    SENSOR_ERROR_STATUS = 4
    TX_OVERFLOW = 5


DEPTH_NO_RETURN_MM = 12000.0  # empirical no-return sentinel in DEPTH_ZF32 payloads (Task 8)
RAW_3DMD_SIZE_BIN2 = 14842  # size in bytes at binning=2 (54×42 zones)
CALIB_SIZE = 2332  # VL53L9_CALIB_DATA_SIZE per-device calibration blob


class ProtocolError(Exception):
    pass


@dataclass(frozen=True)
class FrameHeader:
    frame_type: int
    stream_id: int
    flags: int
    seq: int
    t_us: int
    width: int
    height: int
    payload_len: int

    @classmethod
    def unpack(cls, buf: bytes) -> "FrameHeader":
        magic, ver, ftype, stream, flags, seq, t_us, w, h, plen, _res = _HEADER.unpack(buf)
        if magic != MAGIC:
            raise ProtocolError(f"bad magic {magic!r}")
        if ver != VERSION:
            raise ProtocolError(f"unsupported version {ver}")
        return cls(ftype, stream, flags, seq, t_us, w, h, plen)

    def pack(self) -> bytes:
        return _HEADER.pack(MAGIC, VERSION, self.frame_type, self.stream_id, self.flags,
                            self.seq, self.t_us, self.width, self.height, self.payload_len, 0)


@dataclass(frozen=True)
class Frame:
    header: FrameHeader
    payload: bytes


def pack_frame(header: FrameHeader, payload: bytes) -> bytes:
    if len(payload) != header.payload_len:
        raise ProtocolError(f"payload length {len(payload)} != header {header.payload_len}")
    body = header.pack() + payload
    return body + zlib.crc32(body).to_bytes(4, "little")


def parse_event(payload: bytes) -> tuple[int, int, str]:
    """Decode a frame_type=EVENT payload -> (code, detail, message)."""
    if len(payload) < 8:
        raise ProtocolError(f"event payload too short: {len(payload)} bytes")
    code, detail = struct.unpack_from("<II", payload, 0)
    return code, detail, payload[8:].decode("ascii", "replace")
