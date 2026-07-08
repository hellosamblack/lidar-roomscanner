"""Incremental frame decoder: tolerates garbage, partial reads, and corruption."""
from __future__ import annotations

import struct
import zlib

from .protocol import HEADER_SIZE, MAGIC, VERSION, Frame, FrameHeader, ProtocolError


class StreamDecoder:
    def __init__(self, max_payload: int = 1 << 20):
        self._buf = bytearray()
        self.max_payload = max_payload
        self.frames_decoded = 0
        self.crc_failures = 0
        self.bytes_skipped = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buf.extend(data)
        out: list[Frame] = []
        while True:
            idx = self._buf.find(MAGIC)
            if idx < 0:
                # keep a magic-sized tail in case the magic straddles feeds
                keep = min(len(self._buf), len(MAGIC) - 1)
                self.bytes_skipped += len(self._buf) - keep
                del self._buf[: len(self._buf) - keep]
                return out
            if idx > 0:
                self.bytes_skipped += idx
                del self._buf[:idx]
            if len(self._buf) < HEADER_SIZE:
                return out
            try:
                hdr = FrameHeader.unpack(bytes(self._buf[:HEADER_SIZE]))
            except ProtocolError:
                self.bytes_skipped += 1
                del self._buf[:1]
                continue
            if hdr.payload_len > self.max_payload:
                self.bytes_skipped += 1
                del self._buf[:1]
                continue
            total = HEADER_SIZE + hdr.payload_len + 4
            if len(self._buf) < total:
                return out
            body = bytes(self._buf[:total])
            (crc,) = struct.unpack_from("<I", body, total - 4)
            if zlib.crc32(body[:-4]) != crc:
                self.crc_failures += 1
                del self._buf[:1]
                continue
            out.append(Frame(hdr, body[HEADER_SIZE:-4]))
            self.frames_decoded += 1
            del self._buf[:total]
