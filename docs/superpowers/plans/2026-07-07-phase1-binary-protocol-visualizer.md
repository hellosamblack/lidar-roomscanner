# Phase 1: Binary Frame Protocol + Real-Time 3D Visualizer — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the firmware's ASCII depth dump with a versioned binary frame protocol and build a PC app that renders the depth stream as a live 3D point cloud.

**Architecture:** A transport-agnostic 32-byte-header frame protocol (magic + version + seq + timestamp + CRC32) is implemented three times from one spec: `docs/protocol.md`, a HAL-free C encoder in the firmware fork, and a Python decoder — kept in lockstep by golden byte fixtures. The host pipeline is source → streaming decoder → deprojection → Open3D viewer, with every layer below the viewer unit-tested and hardware-free via file replay. Firmware ships in two milestones: **1a** binary frames over the existing ST-Link VCOM at 921600 baud (~10 fps, zero new middleware — proves the whole chain), then **1b** native USB CDC FS via TinyUSB (~1 MB/s, full sensor rate).

**Tech Stack:** STM32H563 bare-metal C (ST HAL, CMake/Ninja/arm-none-eabi-gcc), TinyUSB (milestone 1b); Python ≥3.11, numpy, pyserial, open3d, pytest.

## Global Constraints

- `../53L9A1/` is read-only; never modified. Our firmware fork: `firmware/scanner-stream/` (copies `<APP>` = `../53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/`).
- Protocol: little-endian; `RS_PROTO_VERSION = 1`; CRC32 = IEEE/zlib over header+payload, transmitted last; header is exactly 32 bytes.
- Firmware protocol code (`rs_protocol.h/.c`) must be HAL-free (host-compilable).
- Python package `roomscan` lives under `host/` with `src/` layout; `requires-python = ">=3.11"` (open3d wheel availability).
- Sequence numbers come from the sensor's own `frame_counter` metadata; they increment per *captured* frame so host-side gaps quantify drops.
- Tasks marked **[HW]** need the NUCLEO board attached; all others run hardware-free.
- Working directory for host commands: `F:\git\personal\lidar\roomscanner\host`; for firmware: the app dir being built.
- Commit after every task (prefixes: `feat:`, `fix:`, `docs:`, `test:`, `chore:`).

---

### Task 1: Protocol spec, host scaffold, and `protocol.py` with golden fixtures

**Files:**
- Create: `docs/protocol.md`
- Create: `host/pyproject.toml`
- Create: `host/src/roomscan/__init__.py` (empty)
- Create: `host/src/roomscan/protocol.py`
- Create: `host/tests/make_fixtures.py`
- Create: `host/tests/fixtures/golden_depth_2x2.bin` (generated)
- Test: `host/tests/test_protocol.py`
- Modify: `.gitignore` (create if absent)

**Interfaces:**
- Produces: `MAGIC: bytes = b"RSCN"`, `VERSION = 1`, `HEADER_SIZE = 32`, `FrameType.DATA = 1`, `FrameType.EVENT = 2`, `StreamId.DEPTH_ZF32 = 0`, `FLAG_DROPPED = 0x01`; `@dataclass FrameHeader(frame_type, stream_id, flags, seq, t_us, width, height, payload_len)` with `FrameHeader.unpack(buf: bytes) -> FrameHeader` (raises `ProtocolError` on bad magic/version); `pack_frame(header: FrameHeader, payload: bytes) -> bytes`; `Frame = namedtuple/dataclass (header: FrameHeader, payload: bytes)`; `class ProtocolError(Exception)`.

- [ ] **Step 1: Write the protocol spec**

Create `docs/protocol.md`:

````markdown
# roomscanner wire protocol — v1

Transport-agnostic binary framing for sensor→host streams. Little-endian throughout.
One frame = 32-byte header, payload, CRC32. See the `protocol-change` skill before editing.

## Frame layout

| Offset | Size | Field         | Notes                                                        |
|--------|------|---------------|--------------------------------------------------------------|
| 0      | 4    | `magic`       | ASCII `RSCN` (bytes `52 53 43 4E`)                           |
| 4      | 1    | `version`     | `1`                                                          |
| 5      | 1    | `frame_type`  | `1` = DATA, `2` = EVENT (device error/log)                   |
| 6      | 1    | `stream_id`   | `0` = DEPTH_ZF32 (float32 perpendicular depth, millimetres)  |
| 7      | 1    | `flags`       | bit0 = DROPPED: ≥1 frame was skipped since the last one sent |
| 8      | 4    | `seq`         | u32; sensor `frame_counter`, increments per *captured* frame |
| 12     | 8    | `t_us`        | u64 µs since boot (v1 source: `HAL_GetTick()*1000`, 1 ms resolution) |
| 20     | 2    | `width`       | zones                                                        |
| 22     | 2    | `height`      | zones                                                        |
| 24     | 4    | `payload_len` | bytes; DEPTH_ZF32 ⇒ `width*height*4`                         |
| 28     | 4    | `reserved`    | 0                                                            |
| 32     | N    | payload       | row-major, stream-defined encoding                           |
| 32+N   | 4    | `crc32`       | IEEE 802.3 / zlib `crc32` over bytes `[0, 32+N)`             |

## Decoder requirements

- Resync by scanning for `magic`; tolerate arbitrary garbage (e.g. ASCII boot text) between frames.
- Bound `payload_len` (reject > 1 MiB) before buffering; treat reject like a CRC failure.
- On CRC failure: advance one byte past the magic candidate and rescan; count failures, never raise.
- Skip unknown `stream_id`/`frame_type` values silently (forward compatibility, no version bump needed).

## USB identification

- Milestone 1a: ST-Link VCOM (VID `0x0483`), 921600 8N1.
- Milestone 1b: native CDC ACM, VID `0xCAFE` PID `0x4001` (TinyUSB descriptors).

## Version history

- **v1** (2026-07): initial — DATA/EVENT frame types, DEPTH_ZF32 stream.
````

- [ ] **Step 2: Create the package scaffold**

`host/pyproject.toml`:

```toml
[project]
name = "roomscan"
version = "0.1.0"
description = "Host-side tools for the roomscanner ToF streamer"
requires-python = ">=3.11"
dependencies = ["numpy>=1.26", "pyserial>=3.5", "open3d>=0.18"]

[project.optional-dependencies]
dev = ["pytest>=8", "ruff>=0.4"]

[project.scripts]
roomscan-view = "roomscan.viewer:main"

[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
where = ["src"]
```

`.gitignore` (repo root, create/extend):

```
build/
captures/
__pycache__/
*.egg-info/
.venv/
.pytest_cache/
```

Create the venv and install: `cd host && python -m venv .venv && .venv\Scripts\pip install -e .[dev]`

- [ ] **Step 3: Write the failing tests**

`host/tests/test_protocol.py`:

```python
import struct
import zlib
from pathlib import Path

import pytest

from roomscan.protocol import (
    FLAG_DROPPED, HEADER_SIZE, MAGIC, VERSION,
    Frame, FrameHeader, FrameType, ProtocolError, StreamId, pack_frame,
)

FIXTURES = Path(__file__).parent / "fixtures"

GOLDEN_HEADER = FrameHeader(
    frame_type=FrameType.DATA, stream_id=StreamId.DEPTH_ZF32, flags=0,
    seq=7, t_us=123_456_789, width=2, height=2, payload_len=16,
)
GOLDEN_PAYLOAD = struct.pack("<4f", 1000.0, 2000.0, 0.0, 500.0)


def test_pack_frame_layout():
    frame = pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD)
    assert len(frame) == HEADER_SIZE + 16 + 4
    # hand-verifiable prefix: magic, version, type, stream, flags, seq
    assert frame[:8] == b"RSCN" + bytes([1, 1, 0, 0])
    assert frame[8:12] == (7).to_bytes(4, "little")
    assert frame[20:24] == (2).to_bytes(2, "little") + (2).to_bytes(2, "little")
    # CRC over everything before it
    assert frame[-4:] == zlib.crc32(frame[:-4]).to_bytes(4, "little")


def test_header_roundtrip():
    frame = pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD)
    hdr = FrameHeader.unpack(frame[:HEADER_SIZE])
    assert hdr == GOLDEN_HEADER


def test_unpack_rejects_bad_magic():
    frame = bytearray(pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD))
    frame[0] = 0x00
    with pytest.raises(ProtocolError):
        FrameHeader.unpack(bytes(frame[:HEADER_SIZE]))


def test_unpack_rejects_bad_version():
    frame = bytearray(pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD))
    frame[4] = 99
    with pytest.raises(ProtocolError):
        FrameHeader.unpack(bytes(frame[:HEADER_SIZE]))


def test_golden_fixture_matches_pack():
    golden = (FIXTURES / "golden_depth_2x2.bin").read_bytes()
    assert pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD) == golden
```

- [ ] **Step 4: Run tests, verify they fail**

Run: `cd host && .venv\Scripts\pytest tests/test_protocol.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'roomscan.protocol'`

- [ ] **Step 5: Implement `protocol.py`**

`host/src/roomscan/protocol.py`:

```python
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
```

- [ ] **Step 6: Generate the golden fixture (independently of `protocol.py`)**

`host/tests/make_fixtures.py`:

```python
"""Regenerate golden fixtures from raw struct calls — deliberately NOT via roomscan.protocol,
so a bug in protocol.py cannot hide inside its own fixture."""
import struct
import zlib
from pathlib import Path

FIXTURES = Path(__file__).parent / "fixtures"


def golden_depth_2x2() -> bytes:
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 1, 1, 0, 0, 7, 123_456_789, 2, 2, 16, 0)
    payload = struct.pack("<4f", 1000.0, 2000.0, 0.0, 500.0)
    return header + payload + zlib.crc32(header + payload).to_bytes(4, "little")


if __name__ == "__main__":
    FIXTURES.mkdir(exist_ok=True)
    (FIXTURES / "golden_depth_2x2.bin").write_bytes(golden_depth_2x2())
    print("fixtures written")
```

Run: `.venv\Scripts\python tests/make_fixtures.py`
Expected: `fixtures written`, file `tests/fixtures/golden_depth_2x2.bin` exists (52 bytes).

- [ ] **Step 7: Run tests, verify they pass**

Run: `.venv\Scripts\pytest tests/test_protocol.py -v`
Expected: 5 passed

- [ ] **Step 8: Commit**

```bash
git add docs/protocol.md host/ .gitignore
git commit -m "feat(protocol): v1 spec, python codec, golden fixtures"
```

---

### Task 2: Streaming decoder with resync

**Files:**
- Create: `host/src/roomscan/decoder.py`
- Test: `host/tests/test_decoder.py`

**Interfaces:**
- Consumes: `roomscan.protocol` (Task 1).
- Produces: `class StreamDecoder(max_payload: int = 1 << 20)` with `feed(data: bytes) -> list[Frame]` and counters `frames_decoded: int`, `crc_failures: int`, `bytes_skipped: int`.

- [ ] **Step 1: Write the failing tests**

`host/tests/test_decoder.py`:

```python
import struct

from roomscan.decoder import StreamDecoder
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame

HDR = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 1000, 2, 2, 16)
PAYLOAD = struct.pack("<4f", 1.0, 2.0, 3.0, 4.0)
FRAME = pack_frame(HDR, PAYLOAD)


def test_single_frame():
    d = StreamDecoder()
    frames = d.feed(FRAME)
    assert len(frames) == 1
    assert frames[0].header == HDR and frames[0].payload == PAYLOAD


def test_partial_feed_boundary_anywhere():
    d = StreamDecoder()
    got = []
    for i in range(len(FRAME)):          # feed one byte at a time
        got += d.feed(FRAME[i:i + 1])
    assert len(got) == 1


def test_resync_after_ascii_garbage():
    d = StreamDecoder()
    noise = b"streams_inspect: depth ZF32 54x42\r\n"
    frames = d.feed(noise + FRAME + noise + FRAME)
    assert len(frames) == 2
    assert d.bytes_skipped >= len(noise)


def test_corrupt_crc_dropped_then_recovers():
    bad = bytearray(FRAME)
    bad[40] ^= 0xFF                       # flip a payload byte
    d = StreamDecoder()
    frames = d.feed(bytes(bad) + FRAME)
    assert len(frames) == 1
    assert d.crc_failures == 1


def test_oversize_payload_rejected():
    hdr = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 0, 2, 2, 1 << 30)
    raw = hdr.pack() + b"x" * 8           # lies about its length; would stall a naive decoder
    d = StreamDecoder()
    frames = d.feed(raw + FRAME)          # must skip the liar and still decode the real frame
    assert len(frames) == 1
    assert frames[0].payload == PAYLOAD
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `.venv\Scripts\pytest tests/test_decoder.py -v`
Expected: FAIL — `No module named 'roomscan.decoder'`

- [ ] **Step 3: Implement**

`host/src/roomscan/decoder.py`:

```python
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
```

- [ ] **Step 4: Run tests, verify they pass**

Run: `.venv\Scripts\pytest tests/test_decoder.py tests/test_protocol.py -v`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/decoder.py host/tests/test_decoder.py
git commit -m "feat(host): streaming decoder with magic resync and CRC drop-counting"
```

---

### Task 3: Depth → point-cloud deprojection

**Files:**
- Create: `host/src/roomscan/deproject.py`
- Test: `host/tests/test_deproject.py`

**Interfaces:**
- Produces: `class Deprojector(width: int, height: int, fov_h_deg: float = 60.0, fov_v_deg: float = 45.0, max_range_mm: float = 10000.0)`, callable: `deprojector(depth_mm: np.ndarray[(h, w), float32]) -> np.ndarray[(M, 3), float64]` in **metres** (x right, y down, z forward), invalid zones (≤0, non-finite, ≥max range) removed.
- Note: default FoV values are **placeholders pending datasheet/`streams_inspect` confirmation** (Task 7); they are constructor parameters and a viewer CLI flag precisely so the correction is a one-line config change, not a code change. ZF32 is perpendicular Z (`radial_to_perp.c` in the transform algo set), so planar `x = z·tan(θx)` deprojection is the correct model.

- [ ] **Step 1: Write the failing tests**

`host/tests/test_deproject.py`:

```python
import numpy as np

from roomscan.deproject import Deprojector


def test_center_zone_projects_straight_ahead():
    # 3x3 grid: the middle zone's angular center is exactly 0
    d = Deprojector(width=3, height=3, fov_h_deg=90.0, fov_v_deg=90.0)
    depth = np.full((3, 3), 2000.0, dtype=np.float32)   # 2 m everywhere
    pts = d(depth)
    assert pts.shape == (9, 3)
    center = pts[4]
    assert np.allclose(center, [0.0, 0.0, 2.0], atol=1e-9)


def test_corner_zone_angle():
    d = Deprojector(width=3, height=3, fov_h_deg=90.0, fov_v_deg=90.0)
    depth = np.full((3, 3), 1000.0, dtype=np.float32)
    pts = d(depth)
    # rightmost column zone center: ((2+0.5)/3 - 0.5) * 90° = 30°
    expected_x = 1.0 * np.tan(np.deg2rad(30.0))
    assert np.isclose(pts[5][0], expected_x, atol=1e-9)   # row 1, col 2
    assert np.isclose(pts[5][2], 1.0)


def test_invalid_zones_filtered():
    d = Deprojector(width=2, height=2)
    depth = np.array([[0.0, np.inf], [np.nan, 1500.0]], dtype=np.float32)
    pts = d(depth)
    assert pts.shape == (1, 3)
    assert np.isclose(pts[0][2], 1.5)


def test_out_of_range_filtered():
    d = Deprojector(width=1, height=1, max_range_mm=4000.0)
    assert d(np.array([[5000.0]], dtype=np.float32)).shape == (0, 3)
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `.venv\Scripts\pytest tests/test_deproject.py -v`
Expected: FAIL — `No module named 'roomscan.deproject'`

- [ ] **Step 3: Implement**

`host/src/roomscan/deproject.py`:

```python
"""Depth (perpendicular Z, mm) -> 3D points (m). FoV defaults are placeholders
until confirmed against the VL53L9CX datasheet / streams_inspect capture."""
from __future__ import annotations

import numpy as np


class Deprojector:
    def __init__(self, width: int, height: int, fov_h_deg: float = 60.0,
                 fov_v_deg: float = 45.0, max_range_mm: float = 10000.0):
        ax = np.deg2rad(((np.arange(width) + 0.5) / width - 0.5) * fov_h_deg)
        ay = np.deg2rad(((np.arange(height) + 0.5) / height - 0.5) * fov_v_deg)
        self._tan_x = np.tan(ax)[None, :]   # (1, w)
        self._tan_y = np.tan(ay)[:, None]   # (h, 1)
        self.max_range_mm = max_range_mm

    def __call__(self, depth_mm: np.ndarray) -> np.ndarray:
        z = depth_mm.astype(np.float64, copy=False)
        valid = np.isfinite(z) & (z > 0.0) & (z < self.max_range_mm)
        x = z * self._tan_x
        y = z * self._tan_y
        y = np.broadcast_to(y, z.shape)
        x = np.broadcast_to(x, z.shape)
        return np.stack([x[valid], y[valid], z[valid]], axis=1) / 1000.0
```

- [ ] **Step 4: Run tests, verify they pass**

Run: `.venv\Scripts\pytest tests/ -v`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/deproject.py host/tests/test_deproject.py
git commit -m "feat(host): parametric-FoV depth deprojection with validity filtering"
```

---

### Task 4: Byte sources (serial, file replay) + recording pump

**Files:**
- Create: `host/src/roomscan/sources.py`
- Test: `host/tests/test_sources.py`

**Interfaces:**
- Consumes: `StreamDecoder` (Task 2).
- Produces: `class FileSource(path, chunk=4096)` with `read() -> bytes` (b"" at EOF) and `close()`; `class SerialSource(port=None, baud=921600, timeout=0.05)` with the same `read()/close()` and `SerialSource.find_port() -> str` (prefers CDC `0xCAFE:0x4001`, falls back to any ST VID `0x0483`, raises `RuntimeError` if none); `pump(source, decoder, record_path=None)` generator yielding `Frame`s, optionally teeing raw bytes to `record_path`.

- [ ] **Step 1: Write the failing tests**

`host/tests/test_sources.py`:

```python
import struct

from roomscan.decoder import StreamDecoder
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame
from roomscan.sources import FileSource, pump

HDR = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 0, 2, 2, 16)
FRAME = pack_frame(HDR, struct.pack("<4f", 1.0, 2.0, 3.0, 4.0))


def test_file_source_replays_all_frames(tmp_path):
    p = tmp_path / "cap.bin"
    p.write_bytes(b"boot noise\r\n" + FRAME * 5)
    frames = list(pump(FileSource(p), StreamDecoder()))
    assert len(frames) == 5


def test_pump_records_raw_bytes(tmp_path):
    src_file = tmp_path / "cap.bin"
    src_file.write_bytes(FRAME * 3)
    rec = tmp_path / "rec.bin"
    frames = list(pump(FileSource(src_file), StreamDecoder(), record_path=rec))
    assert len(frames) == 3
    assert rec.read_bytes() == FRAME * 3   # byte-exact tee


def test_recorded_capture_replays_identically(tmp_path):
    src_file = tmp_path / "cap.bin"
    src_file.write_bytes(b"junk" + FRAME * 2)
    rec = tmp_path / "rec.bin"
    first = list(pump(FileSource(src_file), StreamDecoder(), record_path=rec))
    second = list(pump(FileSource(rec), StreamDecoder()))
    assert [f.payload for f in first] == [f.payload for f in second]
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `.venv\Scripts\pytest tests/test_sources.py -v`
Expected: FAIL — `No module named 'roomscan.sources'`

- [ ] **Step 3: Implement**

`host/src/roomscan/sources.py`:

```python
"""Byte sources and the frame pump. All I/O lives here — decoder/deproject stay pure."""
from __future__ import annotations

from pathlib import Path
from typing import Iterator, Optional

from .decoder import StreamDecoder
from .protocol import Frame

CDC_VID, CDC_PID = 0xCAFE, 0x4001   # milestone 1b TinyUSB descriptors (docs/protocol.md)
ST_VID = 0x0483                     # ST-Link VCOM fallback (milestone 1a)


class FileSource:
    def __init__(self, path, chunk: int = 4096):
        self._f = open(path, "rb")
        self._chunk = chunk

    def read(self) -> bytes:
        return self._f.read(self._chunk)

    def close(self) -> None:
        self._f.close()


class SerialSource:
    def __init__(self, port: Optional[str] = None, baud: int = 921600, timeout: float = 0.05):
        import serial  # deferred: tests must not need pyserial hardware access
        if port is None:
            port = self.find_port()
        self._ser = serial.Serial(port, baud, timeout=timeout)
        self.port = port

    @staticmethod
    def find_port() -> str:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        for p in ports:
            if p.vid == CDC_VID and p.pid == CDC_PID:
                return p.device
        for p in ports:
            if p.vid == ST_VID:
                return p.device
        raise RuntimeError(f"no scanner serial port found among {[p.device for p in ports]}")

    def read(self) -> bytes:
        return self._ser.read(4096)

    def close(self) -> None:
        self._ser.close()


def pump(source, decoder: StreamDecoder, record_path=None) -> Iterator[Frame]:
    rec = open(record_path, "wb") if record_path else None
    try:
        while True:
            data = source.read()
            if not data:
                if isinstance(source, FileSource):
                    return          # EOF on replay; live sources just idle
                continue
            if rec:
                rec.write(data)
            yield from decoder.feed(data)
    finally:
        if rec:
            rec.close()
        source.close()
```

- [ ] **Step 4: Run tests, verify they pass**

Run: `.venv\Scripts\pytest tests/ -v`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/sources.py host/tests/test_sources.py
git commit -m "feat(host): serial/file byte sources with raw-capture recording pump"
```

---

### Task 5: Live Open3D viewer CLI

**Files:**
- Create: `host/src/roomscan/viewer.py`

**Interfaces:**
- Consumes: everything from Tasks 1–4.
- Produces: `main(argv=None)` console entry (`roomscan-view`). Flags: `--port`, `--baud` (default 921600), `--replay PATH`, `--record PATH`, `--fov-h 60.0`, `--fov-v 45.0`. The render loop itself is validated manually (it's a window); all logic beneath it is already tested.

- [ ] **Step 1: Implement**

`host/src/roomscan/viewer.py`:

```python
"""Live point-cloud viewer. Reader thread: source -> decoder -> latest-frame slot;
main thread: Open3D non-blocking render loop + 1 Hz stats line."""
from __future__ import annotations

import argparse
import queue
import sys
import threading
import time

import numpy as np

from .decoder import StreamDecoder
from .deproject import Deprojector
from .protocol import FLAG_DROPPED, FrameType, StreamId
from .sources import FileSource, SerialSource, pump


class Stats:
    def __init__(self):
        self.frames = 0
        self.seq_gaps = 0
        self.dropped_flags = 0
        self._last_seq = None

    def update(self, header):
        self.frames += 1
        if header.flags & FLAG_DROPPED:
            self.dropped_flags += 1
        if self._last_seq is not None and header.seq > self._last_seq + 1:
            self.seq_gaps += header.seq - self._last_seq - 1
        self._last_seq = header.seq


def _reader(source, decoder, slot: queue.Queue, stats: Stats, record):
    for frame in pump(source, decoder, record_path=record):
        if frame.header.frame_type != FrameType.DATA or frame.header.stream_id != StreamId.DEPTH_ZF32:
            continue
        stats.update(frame.header)
        try:
            slot.get_nowait()          # latest-wins: drop stale frame
        except queue.Empty:
            pass
        slot.put(frame)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="roomscan-view")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--replay")
    ap.add_argument("--record")
    ap.add_argument("--fov-h", type=float, default=60.0)
    ap.add_argument("--fov-v", type=float, default=45.0)
    args = ap.parse_args(argv)

    import open3d as o3d   # deferred: heavy import

    source = FileSource(args.replay) if args.replay else SerialSource(args.port, args.baud)
    decoder = StreamDecoder()
    stats = Stats()
    slot: queue.Queue = queue.Queue(maxsize=1)
    threading.Thread(target=_reader, args=(source, decoder, slot, stats, args.record),
                     daemon=True).start()

    vis = o3d.visualization.Visualizer()
    vis.create_window("roomscan", width=1280, height=800)
    pcd = o3d.geometry.PointCloud()
    added = False
    deproj = None
    shown = 0
    t_stat = time.monotonic()
    f_stat = 0

    while vis.poll_events():
        try:
            frame = slot.get(timeout=0.02)
        except queue.Empty:
            vis.update_renderer()
            continue
        h, w = frame.header.height, frame.header.width
        if deproj is None:
            deproj = Deprojector(w, h, args.fov_h, args.fov_v)
        depth = np.frombuffer(frame.payload, dtype="<f4").reshape(h, w)
        pts = deproj(depth)
        pcd.points = o3d.utility.Vector3dVector(pts)
        if len(pts):
            zn = (pts[:, 2] - pts[:, 2].min()) / max(float(np.ptp(pts[:, 2])), 1e-6)
            pcd.colors = o3d.utility.Vector3dVector(
                np.stack([zn, 0.6 * (1 - zn), 1 - zn], axis=1))
        if not added:
            vis.add_geometry(pcd)
            added = True
        else:
            vis.update_geometry(pcd)
        vis.update_renderer()
        shown += 1

        now = time.monotonic()
        if now - t_stat >= 1.0:
            fps = (shown - f_stat) / (now - t_stat)
            print(f"\r{fps:5.1f} fps | frames {stats.frames} | seq gaps {stats.seq_gaps} "
                  f"| crc fail {decoder.crc_failures} | skipped {decoder.bytes_skipped} B ",
                  end="", flush=True)
            t_stat, f_stat = now, shown
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Synthetic end-to-end check (no hardware)**

Build a fake capture and view it:

```powershell
cd host
.venv\Scripts\python -c "
import numpy as np, struct
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame
buf = b''
for i in range(100):
    z = 1500 + 400*np.sin(np.linspace(0, 6.28, 54*42) + i/5)
    payload = z.astype('<f4').tobytes()
    buf += pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, i, i*66000, 54, 42, len(payload)), payload)
open('synthetic.bin','wb').write(buf)"
.venv\Scripts\roomscan-view --replay synthetic.bin
```

Expected: window opens, animated wavy point cloud, stats line shows `crc fail 0`, `seq gaps 0`, ~replay-speed fps. Close window to exit cleanly.

- [ ] **Step 3: Commit**

```bash
git add host/src/roomscan/viewer.py
git commit -m "feat(host): live Open3D point-cloud viewer with fps/drop HUD"
```

---

### Task 6: Firmware fork `firmware/scanner-stream/` builds

**Files:**
- Create: `firmware/scanner-stream/**` (copy of `<APP>`, excluding IDE/build dirs)
- Modify: `firmware/scanner-stream/CMakeLists.txt` (repoint 5-levels-up paths)

**Interfaces:**
- Produces: a buildable app dir; all later firmware tasks edit only this copy. `<APP>` remains untouched. The `<APP>`-local `Drivers/` copy comes along, so `cmake/stm32cubemx/CMakeLists.txt` works unchanged; only the root `CMakeLists.txt`'s `../../../../../` references (package-root Utilities/Middlewares/BSP-Components) need repointing.

- [ ] **Step 1: Copy the app**

```powershell
robocopy "F:\git\personal\lidar\53L9A1\Projects\NUCLEO-H563ZI\Applications\53L9A1\53L9A1_PostprocessSingle" `
         "F:\git\personal\lidar\roomscanner\firmware\scanner-stream" `
         /E /XD build EWARM MDK-ARM STM32CubeIDE Binary
```

Expected: robocopy exit code < 8 (success). `firmware/scanner-stream/` contains `Src/`, `Inc/`, `Drivers/`, `cmake/`, `CMakeLists.txt`, `CMakePresets.json`, linker scripts, startup file, `.ioc`.

- [ ] **Step 2: Repoint package paths in the root CMakeLists**

In `firmware/scanner-stream/CMakeLists.txt`, insert directly after the `project(...)` line:

```cmake
# 53L9A1 reference package (read-only) providing shared Utilities/Middlewares/BSP components
set(PKG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../53L9A1)
```

then replace every occurrence of `../../../../../` with `${PKG_ROOT}/`:

```bash
cd F:/git/personal/lidar/roomscanner/firmware/scanner-stream
sed -i 's|\.\./\.\./\.\./\.\./\.\./|${PKG_ROOT}/|g' CMakeLists.txt
```

Also change the project name line: `set(CMAKE_PROJECT_NAME scanner_stream)`.

- [ ] **Step 3: Build**

```bash
cd F:/git/personal/lidar/roomscanner/firmware/scanner-stream
cmake --preset Debug
cmake --build build/Debug
```

Expected: link succeeds, `build/Debug/scanner_stream.bin` produced, `arm-none-eabi-size` output printed. If headers from the package aren't found, check the `target_include_directories` block also got the `${PKG_ROOT}/` rewrite.

- [ ] **Step 4: Commit**

```bash
git add firmware/scanner-stream
git commit -m "feat(firmware): fork reference app as scanner-stream, build against 53L9A1 package in place"
```

---

### Task 7: **[HW]** Capture `streams_inspect` / `controls_inspect` dump

**Files:**
- Create: `docs/transform-streams.md`

**Interfaces:**
- Produces: the definitive list of transform-library streams (depth / reflectance / confidence / XYZ?) and controls. **Downstream impact:** if an XYZ stream exists, Phase 1 stays depth-only but Phase 2 planning changes; the dump also supplies stream names for Phase 2 `stream_id` allocation and control names for Phase 3.

- [ ] **Step 1: Flash the freshly built fork (unmodified behavior = reference behavior)**

Follow the `firmware-loop` skill:

```sh
cd firmware/scanner-stream
STM32_Programmer_CLI -c port=SWD -w build/Debug/scanner_stream.bin 0x08000000 -rst
```

- [ ] **Step 2: Capture the boot output**

```powershell
python -m serial.tools.list_ports -v        # find the ST-Link VCOM port
python -m serial.tools.miniterm COM<N> 115200 --raw | Tee-Object -FilePath boot_dump.txt
```

Press the board's black RESET button; capture everything from reset through the first few ASCII frames, then Ctrl+] to exit.

- [ ] **Step 3: Write `docs/transform-streams.md`**

Paste the raw `streams_inspect` and `controls_inspect` sections verbatim into a fenced block, then add a short interpretation section answering, explicitly:
1. Which output streams exist beyond `depth`/ZF32 (names + formats)?
2. Is there an XYZ/point-cloud output stream? (→ note the Phase 2 implication either way)
3. Which controls exist (names/types) — flag the ones Phase 3 will target (usecase, binning, stream toggles)?
4. Observed ZF32 value range from the ASCII frames (sanity-check the millimetre assumption used by `Deprojector`).

- [ ] **Step 4: Commit**

```bash
git add docs/transform-streams.md
git commit -m "docs: capture transform library streams/controls inspection dump"
```

---

### Task 8: Firmware binary streaming over VCOM @ 921600 (milestone 1a)

**Files:**
- Create: `firmware/scanner-stream/Src/rs_protocol.h`
- Create: `firmware/scanner-stream/Src/rs_protocol.c`
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c`
- Modify: `firmware/scanner-stream/Src/main.c` (baud, USER CODE region)
- Modify: `firmware/scanner-stream/CMakeLists.txt` (add source)

**Interfaces:**
- Consumes: protocol layout from `docs/protocol.md` (Task 1); BSP UART handle `extern UART_HandleTypeDef hcom_uart[]` (index `COM1`).
- Produces: `rs_write_header(uint8_t out[32], uint8_t frame_type, uint8_t stream_id, uint8_t flags, uint32_t seq, uint64_t t_us, uint16_t width, uint16_t height, uint32_t payload_len)`; `uint32_t rs_crc32(uint32_t crc, const uint8_t *data, size_t len)` (zlib-chainable: pass previous result as `crc`, start with 0); `void rs_put_u32(uint8_t *p, uint32_t v)`. Task 10/11 reuse all of these unchanged.

- [ ] **Step 1: Write `rs_protocol.h`**

```c
/* Wire protocol v1 — single source of truth: roomscanner/docs/protocol.md.
 * HAL-free on purpose: host-compilable for cross-checking against the Python codec. */
#ifndef RS_PROTOCOL_H
#define RS_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define RS_PROTO_VERSION     (1u)
#define RS_HEADER_SIZE       (32u)
#define RS_FRAME_DATA        (1u)
#define RS_FRAME_EVENT       (2u)
#define RS_STREAM_DEPTH_ZF32 (0u)
#define RS_FLAG_DROPPED      (0x01u)

void rs_put_u32(uint8_t *p, uint32_t v);

/* IEEE 802.3 / zlib CRC-32. Chain calls by passing the previous return as crc (start 0). */
uint32_t rs_crc32(uint32_t crc, const uint8_t *data, size_t len);

void rs_write_header(uint8_t out[RS_HEADER_SIZE], uint8_t frame_type, uint8_t stream_id,
                     uint8_t flags, uint32_t seq, uint64_t t_us, uint16_t width,
                     uint16_t height, uint32_t payload_len);

#endif /* RS_PROTOCOL_H */
```

- [ ] **Step 2: Write `rs_protocol.c`**

```c
#include "rs_protocol.h"

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

void rs_put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void put_u64(uint8_t *p, uint64_t v) {
    rs_put_u32(p, (uint32_t)v);
    rs_put_u32(p + 4, (uint32_t)(v >> 32));
}

uint32_t rs_crc32(uint32_t crc, const uint8_t *data, size_t len) {
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

void rs_write_header(uint8_t out[RS_HEADER_SIZE], uint8_t frame_type, uint8_t stream_id,
                     uint8_t flags, uint32_t seq, uint64_t t_us, uint16_t width,
                     uint16_t height, uint32_t payload_len) {
    out[0] = 'R'; out[1] = 'S'; out[2] = 'C'; out[3] = 'N';
    out[4] = RS_PROTO_VERSION;
    out[5] = frame_type;
    out[6] = stream_id;
    out[7] = flags;
    rs_put_u32(out + 8, seq);
    put_u64(out + 12, t_us);
    put_u16(out + 20, width);
    put_u16(out + 22, height);
    rs_put_u32(out + 24, payload_len);
    rs_put_u32(out + 28, 0u); /* reserved */
}
```

Add to `CMakeLists.txt` `target_sources`, next to `Src/vl53l9_app.c`:

```cmake
    Src/rs_protocol.c
```

- [ ] **Step 3: Wire streaming into `vl53l9_app.c`**

Top of file — change the knob and add includes/support (after the existing `#define CONF_USECASE` line):

```c
#define CONF_PRINT_FRAME   (0) /**< ASCII art disabled in streaming builds */
#define CONF_STREAM_BINARY (1) /**< emit rs_protocol frames on the COM1 UART */

#include "rs_protocol.h"
#include "stm32h5xx_nucleo.h"

extern UART_HandleTypeDef hcom_uart[];

static uint64_t rs_time_us(void) {
    /* v1: HAL tick, 1 ms resolution widened to the u64 µs wire field.
     * Upgrade to a TIM-based µs clock when IMU fusion needs it (Phase 5). */
    return (uint64_t)HAL_GetTick() * 1000u;
}

static void rs_send_depth_uart(uint32_t seq, uint8_t flags, const uint8_t *payload,
                               uint32_t len, uint16_t w, uint16_t h) {
    uint8_t hdr[RS_HEADER_SIZE];
    uint8_t tail[4];
    rs_write_header(hdr, RS_FRAME_DATA, RS_STREAM_DEPTH_ZF32, flags, seq, rs_time_us(), w, h, len);
    uint32_t crc = rs_crc32(0u, hdr, RS_HEADER_SIZE);
    crc = rs_crc32(crc, payload, len);
    rs_put_u32(tail, crc);
    HAL_UART_Transmit(&hcom_uart[COM1], hdr, RS_HEADER_SIZE, 1000);
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)payload, (uint16_t)len, 1000);
    HAL_UART_Transmit(&hcom_uart[COM1], tail, 4, 1000);
}
```

Fix the reference return-value bug in the acquisition loop (`vl53l9_trigger_frame` call):

```c
        ret = vl53l9_trigger_frame(p_dev);
        if (ret) {
            handle_error();
        }
```

**Sequence-number correctness:** the loop processes frame N−1 while frame N is being DMA'd, so the depth buffer and the freshly parsed metadata are off by one. Track the previous counter. Add above the `while (1)`:

```c
    uint32_t rs_prev_counter = 0;
    bool rs_have_prev = false;
```

Inside the loop, replace the processing `else` branch body so the send happens right after processing, tagged with the *previous* frame's counter:

```c
        } else {
            /* TODO: find a better way to handle this, maybe leveraging mems list */
            in_raw_mems.items = &in_raw_mem[(raw_mem_index + 1) % 2];
            ret = transform_process_stream(p_transform, &stream_buffers);
            if (ret) {
                handle_error();
            }
#if CONF_STREAM_BINARY
            if (rs_have_prev) {
                rs_send_depth_uart(rs_prev_counter, 0u, (const uint8_t *)out_depth_mem.data,
                                   frame_buffer_size, out_width, out_height);
            }
#endif
        }
```

And after the existing metadata parse (`vl53l9_utils_parse_frame` success), record the counter:

```c
        rs_prev_counter = (uint32_t)frame.p_metadata->frame_counter;
        rs_have_prev = true;
```

Finally, guard the ASCII/fps print block so streaming builds emit no interleaved text:

```c
#if !CONF_STREAM_BINARY
        print_frame((float *)out_depth_mem.data, out_height, out_width);
        printf("Processed frame n. %lu @ %u fps\n", (unsigned long)frame.p_metadata->frame_counter,
               (unsigned int)frame_rate);
#endif
```

(The `streams_inspect`/`controls_inspect` printfs at startup stay — they run before streaming begins, and the host decoder resyncs past ASCII by design.)

- [ ] **Step 4: Raise the VCOM baud**

In `firmware/scanner-stream/Src/main.c`, in the COM1 init block (~line 117), change:

```c
  BspCOMInit.BaudRate   = 921600;
```

- [ ] **Step 5: Build**

Run: `cmake --build build/Debug` (from `firmware/scanner-stream/`)
Expected: clean link, `.bin` produced.

- [ ] **Step 6: [HW] Flash and verify end-to-end**

```sh
STM32_Programmer_CLI -c port=SWD -w build/Debug/scanner_stream.bin 0x08000000 -rst
cd ../../host
.venv\Scripts\roomscan-view --port COM<N> --baud 921600 --record ..\captures\first_stream.bin
```

Expected: live point cloud of whatever the sensor faces; stats line shows ~9–10 fps (921600 baud ceiling for 9108-byte frames), `crc fail 0`, `seq gaps 0` after startup. Report the actual numbers. Replay check: `roomscan-view --replay ..\captures\first_stream.bin` renders the identical sequence.

- [ ] **Step 7: Sanity-check ZF32 units against the capture**

Point the sensor at a wall at a hand-measured ~1 m; confirm the rendered cloud's z ≈ 1.0 m (i.e. payload is millimetres). If it's off by 1000× or is radial-not-perpendicular, update `docs/protocol.md` stream notes and `Deprojector` accordingly before proceeding.

- [ ] **Step 8: Commit**

```bash
git add firmware/scanner-stream docs/protocol.md
git commit -m "feat(firmware): binary depth streaming over VCOM at 921600 (milestone 1a)"
```

---

### Task 9: Cross-implementation golden check (C encoder vs Python decoder)

**Files:**
- Create: `host/tests/test_capture_regression.py`
- Create: `host/tests/fixtures/hw_capture_snippet.bin` (from Task 8's recording)

**Interfaces:**
- Consumes: `captures/first_stream.bin` (Task 8), decoder (Task 2).
- Produces: a checked-in, hardware-free regression proving real firmware bytes decode — the lockstep guarantee the `protocol-change` skill relies on.

- [ ] **Step 1: Extract a small snippet from the live capture**

```powershell
cd host
.venv\Scripts\python -c "
data = open('../captures/first_stream.bin','rb').read()
i = data.find(b'RSCN')
open('tests/fixtures/hw_capture_snippet.bin','wb').write(data[max(0, i-64): i + 3*9108 + 64])"
```

(≤ ~28 KB: three real frames plus surrounding bytes — small enough to check in.)

- [ ] **Step 2: Write the regression test**

`host/tests/test_capture_regression.py`:

```python
from pathlib import Path

from roomscan.decoder import StreamDecoder
from roomscan.protocol import StreamId

FIXTURE = Path(__file__).parent / "fixtures" / "hw_capture_snippet.bin"


def test_real_firmware_capture_decodes():
    d = StreamDecoder()
    frames = d.feed(FIXTURE.read_bytes())
    assert len(frames) >= 3
    assert d.crc_failures == 0
    for f in frames:
        assert f.header.stream_id == StreamId.DEPTH_ZF32
        assert f.header.payload_len == f.header.width * f.header.height * 4
    seqs = [f.header.seq for f in frames]
    assert seqs == sorted(seqs)
```

- [ ] **Step 3: Run tests, verify they pass**

Run: `.venv\Scripts\pytest tests/ -v`
Expected: all pass (if CRC fails here, the C and Python CRC implementations disagree — fix before anything else).

- [ ] **Step 4: Commit**

```bash
git add host/tests/test_capture_regression.py host/tests/fixtures/hw_capture_snippet.bin
git commit -m "test(host): real-firmware capture regression locks C/Python protocol parity"
```

---

### Task 10: TinyUSB CDC enumeration (milestone 1b, part 1)

**Files:**
- Create: `firmware/vendor/tinyusb/` (vendored, `src/` tree only)
- Create: `firmware/scanner-stream/Src/tusb_config.h`
- Create: `firmware/scanner-stream/Src/usb_descriptors.c`
- Modify: `firmware/scanner-stream/Src/main.c`, `Src/stm32h5xx_it.c`, `CMakeLists.txt`

**Interfaces:**
- Produces: board enumerates as CDC ACM `VID 0xCAFE / PID 0x4001`; `tud_task()` pump callable from the app loop; `tud_cdc_write*` available to Task 11.
- **Decision:** TinyUSB (MIT, no RTOS, single vendored lib) over ST's classic USB Device Library — the `53L9A1/` package ships **no** USB middleware and STM32CubeH5 itself moved to USBX, so something must be vendored either way; TinyUSB's `stm32_fsdev` port covers the H5's `USB_DRD_FS` IP. **Fallback if the support check in Step 1 fails:** vendor `STM32_USB_Device_Library` (Core + CDC) from STM32CubeU5 (same DRD IP) and write the standard `usbd_conf.c` glue against the already-initialized `hpcd_USB_DRD_FS`.

- [ ] **Step 1: Vendor TinyUSB and verify H5 support**

```bash
cd F:/git/personal/lidar/roomscanner
git clone --depth 1 https://github.com/hathach/tinyusb C:/Users/hello/AppData/Local/Temp/claude/tinyusb-clone
mkdir -p firmware/vendor/tinyusb
cp -r C:/Users/hello/AppData/Local/Temp/claude/tinyusb-clone/src firmware/vendor/tinyusb/src
cp C:/Users/hello/AppData/Local/Temp/claude/tinyusb-clone/LICENSE firmware/vendor/tinyusb/
grep -rn "OPT_MCU_STM32H5" firmware/vendor/tinyusb/src/tusb_option.h
grep -rln "STM32H5" firmware/vendor/tinyusb/src/portable/st/stm32_fsdev/
```

Expected: both greps hit (`OPT_MCU_STM32H5` defined; fsdev port handles H5). **If not: stop, use the fallback path above and re-plan this task.**

- [ ] **Step 2: `tusb_config.h`**

```c
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU              OPT_MCU_STM32H5
#define CFG_TUSB_OS               OPT_OS_NONE
#define CFG_TUSB_RHPORT0_MODE     OPT_MODE_DEVICE

#define CFG_TUD_ENABLED           1
#define CFG_TUD_ENDPOINT0_SIZE    64

#define CFG_TUD_CDC               1
#define CFG_TUD_CDC_RX_BUFSIZE    256
#define CFG_TUD_CDC_TX_BUFSIZE    2048
#define CFG_TUD_CDC_EP_BUFSIZE    64

#endif
```

- [ ] **Step 3: `usb_descriptors.c`**

```c
#include "tusb.h"

#define USB_VID 0xCAFE
#define USB_PID 0x4001

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04}, /* 0: English (US) */
    "roomscanner",              /* 1: manufacturer */
    "scanner-stream",           /* 2: product */
    "000001",                   /* 3: serial */
    "scanner-stream CDC",       /* 4: CDC interface */
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;
    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) _desc_str[1 + i] = str[i];
    }
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
```

(Add `#include <string.h>` at the top with the tusb include.)

- [ ] **Step 4: CMake integration**

Append to `target_sources` in `firmware/scanner-stream/CMakeLists.txt`:

```cmake
    Src/usb_descriptors.c

    # TinyUSB (vendored, MIT)
    ../vendor/tinyusb/src/tusb.c
    ../vendor/tinyusb/src/common/tusb_fifo.c
    ../vendor/tinyusb/src/device/usbd.c
    ../vendor/tinyusb/src/device/usbd_control.c
    ../vendor/tinyusb/src/class/cdc/cdc_device.c
    ../vendor/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
```

Append to `target_include_directories`:

```cmake
    ../vendor/tinyusb/src
```

Append to `target_compile_definitions`:

```cmake
    CFG_TUSB_MCU=OPT_MCU_STM32H5
```

- [ ] **Step 5: main.c + IRQ wiring**

TinyUSB drives the USB peripheral registers itself — the MX-generated `MX_USB_PCD_Init()` (HAL PCD) must not run alongside it. In `main.c`:

- In `USER CODE BEGIN Includes`: `#include "tusb.h"`
- Comment out the `MX_USB_PCD_Init();` call in `main()` with a note: `/* USB owned by TinyUSB (see USER CODE 2) */`
- In `USER CODE BEGIN 2` (after peripheral init, before the app loop):

```c
  HAL_PWREx_EnableVddUSB();
  __HAL_RCC_USB_CLK_ENABLE();
  HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);
  tud_init(BOARD_TUD_RHPORT);
```

(`BOARD_TUD_RHPORT` = 0; define it in `tusb_config.h` if the vendored version doesn't: `#define BOARD_TUD_RHPORT 0`.)

- In `Src/stm32h5xx_it.c`, add (USER CODE region):

```c
void USB_DRD_FS_IRQHandler(void) {
    extern void tud_int_handler(uint8_t rhport);
    tud_int_handler(0);
}
```

(First check the file — if MX already generated a `USB_DRD_FS_IRQHandler` calling `HAL_PCD_IRQHandler`, replace its body with `tud_int_handler(0);` inside the USER CODE guards.)

- [ ] **Step 6: Prove enumeration with a heartbeat**

Temporarily add to `vl53l9_app.c`'s loop (top of `while(1)`, marked clearly for removal in Task 11):

```c
        /* TASK10 TEMP: CDC heartbeat, removed in Task 11 */
        tud_task();
        if (tud_cdc_connected()) {
            tud_cdc_write_str("hb\r\n");
            tud_cdc_write_flush();
        }
```

(with `#include "tusb.h"` at top). Build: `cmake --build build/Debug` — expect clean link.

- [ ] **Step 7: [HW] Flash and verify enumeration**

Flash per `firmware-loop`. Connect a USB cable to the board's **native USB (USER) connector** — the ST-Link connector alone won't expose the CDC port. Then:

```powershell
python -m serial.tools.list_ports -v
```

Expected: a new COM port with `VID:PID=CAFE:4001`. `python -m serial.tools.miniterm COM<M> 115200` shows a stream of `hb` lines. Report the observed port.

- [ ] **Step 8: Commit**

```bash
git add firmware/vendor/tinyusb firmware/scanner-stream
git commit -m "feat(firmware): TinyUSB CDC ACM enumeration on native USB_DRD_FS"
```

---

### Task 11: Stream frames over CDC with drop policy (milestone 1b, part 2)

**Files:**
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c`

**Interfaces:**
- Consumes: `rs_*` (Task 8), `tud_*` (Task 10), `SerialSource.find_port()` CDC preference (Task 4 — already matches `0xCAFE:0x4001`).
- Produces: milestone 1b — full-rate depth streaming over CDC; VCOM keeps startup text only.

- [ ] **Step 1: Replace the heartbeat with a CDC frame writer**

Remove the TASK10 TEMP block. Add next to `rs_send_depth_uart`:

```c
/* Pump the CDC FIFO out. Returns false if the host stalled >100 ms (frame aborted:
 * the host decoder counts one CRC failure/resync and we set DROPPED on the next frame). */
static bool rs_cdc_send(const uint8_t *p, uint32_t n) {
    uint32_t t0 = HAL_GetTick();
    while (n) {
        uint32_t avail = tud_cdc_write_available();
        if (avail) {
            uint32_t k = MIN(avail, n);
            tud_cdc_write(p, k);
            p += k;
            n -= k;
        }
        tud_task();
        if ((HAL_GetTick() - t0) > 100u) {
            return false;
        }
    }
    tud_cdc_write_flush();
    return true;
}

static void rs_send_depth_cdc(uint32_t seq, uint8_t flags, const uint8_t *payload,
                              uint32_t len, uint16_t w, uint16_t h) {
    static uint8_t pending_dropped = 0;

    if (!tud_cdc_connected()) {   /* no host: don't burn 100 ms per frame */
        pending_dropped = 1;
        return;
    }
    flags |= pending_dropped ? RS_FLAG_DROPPED : 0u;

    uint8_t hdr[RS_HEADER_SIZE];
    uint8_t tail[4];
    rs_write_header(hdr, RS_FRAME_DATA, RS_STREAM_DEPTH_ZF32, flags, seq, rs_time_us(), w, h, len);
    uint32_t crc = rs_crc32(0u, hdr, RS_HEADER_SIZE);
    crc = rs_crc32(crc, payload, len);
    rs_put_u32(tail, crc);

    bool ok = rs_cdc_send(hdr, RS_HEADER_SIZE) && rs_cdc_send(payload, len) && rs_cdc_send(tail, 4);
    pending_dropped = ok ? 0u : 1u;
}
```

Switch the send call in the processing branch from `rs_send_depth_uart(...)` to `rs_send_depth_cdc(...)` (same arguments), and add a `tud_task();` call at the top of the `while (1)` loop so USB stays serviced even on frames that skip sending.

- [ ] **Step 2: Build**

Run: `cmake --build build/Debug`
Expected: clean link.

- [ ] **Step 3: [HW] Flash and validate throughput**

```powershell
cd host
.venv\Scripts\roomscan-view --record ..\captures\cdc_stream.bin      # auto-selects CAFE:4001
```

Expected: live cloud at the sensor's actual processed frame rate (bounded by transform time, not the link — CDC FS moves 9108-byte frames at 60+ fps). Acceptance: **fps ≥ 15, `crc fail 0`, `seq gaps 0`** with the host idle. Then stress it: grab/resize the viewer window for a few seconds — seq gaps may appear (drop policy working) but the stream must recover with no CRC failures afterward. Report all numbers.

- [ ] **Step 4: Update docs**

- `docs/protocol.md`: no layout change (no version bump) — but confirm the "USB identification" section matches reality (port VID/PID observed).
- `ROADMAP.md`: mark Phase 1 milestones 1a/1b complete with measured fps.

- [ ] **Step 5: Commit**

```bash
git add firmware/scanner-stream docs/protocol.md ROADMAP.md
git commit -m "feat(firmware): full-rate depth streaming over native USB CDC (milestone 1b)"
```

---

## Execution notes

- Tasks 1–6 are hardware-free and strictly ordered only where interfaces demand (1→2→4→5; 3 can go anytime after 1; 6 anytime). Tasks 7–11 need the board and are sequential.
- If Task 7's dump reveals an XYZ output stream, **do not change Phase 1 scope** — depth-only streaming stays the deliverable; record the finding in `docs/transform-streams.md` for Phase 2.
- If Task 8 shows persistent CRC failures at 921600 baud, drop to 460800 (`BspCOMInit.BaudRate` + `--baud`) — halves fps but milestone 1a only proves the chain; 1b is the performance milestone.
