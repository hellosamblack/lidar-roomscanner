# SLAM GPU Container Service Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the GPU-bound SLAM compute (`Mapper.step`) inside a `--gpus all` WSL container behind the existing `SlamWorker` interface, so the Windows panel keeps USB capture + rendering while a black-box CUDA service accelerates the live and processed render.

**Architecture:** A `RemoteSlamWorker` implements `SlamWorker`'s exact `submit()`/`latest()`/`start()`/`stop()` interface but ships per-frame tuples over a localhost TCP socket to a container-side `SlamService` that owns one `Mapper` on `CUDA:0`. Both ends use latest-wins slots. The panel selects the backend from config and falls back to the in-process CPU worker when the service is unreachable. The container is a stock `python:3.12-slim` + `pip install open3d` (0.19 wheel carries its own CUDA runtime; the driver comes via `--gpus all`).

**Tech Stack:** Python 3.12, Open3D 0.19 (Linux CUDA wheel in-container, CPU wheel on Windows), stdlib `socket`/`struct`/`threading`, NumPy, `wslc` (WSL Container CLI), PowerShell lifecycle scripts.

## Global Constraints

- Python: `>=3.11,<3.13` (repo pin; container uses **3.12**). Copy verbatim from `host/pyproject.toml`.
- Open3D: `>=0.18`; container installs **0.19.0** (verified `cuda_available=True`, `device_count=1` on `python:3.12-slim` + `libgl1 libgomp1 libx11-6`).
- **No new runtime dependencies** on the Windows host. Wire framing uses stdlib + NumPy only; mesh (de)serialization imports `open3d` lazily (already a dep).
- The GPU path is an **accelerator, never a hard dependency**: any connect/socket failure must fall back to the in-process CPU `SlamWorker` without raising.
- `backend` defaults to `"local"` — behavior is unchanged until the user opts in via `roomscan.toml`.
- Container is **localhost-only, single-client, single-GPU** — no auth, no multi-tenancy.
- All new files are **additive**; do not rewrite `worker.py`, `mapper.py`, or the panel's render path. The result payload is the same `(mesh, trajectory, FrameStep)` shape `SlamWorker` already publishes.
- Tests must run **GPU-free and container-free** (loopback on `CPU:0`, ephemeral ports); existing 458 tests stay green.

## Reference: exact existing signatures (do not change these)

```python
# slam/worker.py  -- the interface RemoteSlamWorker must mirror
SlamWorker(width, height, mesh_every=5, **mapper_kwargs)
  .submit(depth, quat, pressure, reflectance=None, confidence=None) -> None
  .latest() -> tuple(mesh, trajectory, FrameStep) | None
  .start() -> None
  .stop()  -> None
  .tracking_lost_count -> int          # @property

# slam/mapper.py
@dataclass
class FrameStep:
    pose: np.ndarray          # 4x4 float64, == report_pose (also the trajectory entry)
    fitness: float
    rmse: float
    tracking_lost: bool
    slam_ms: float
Mapper(...).mesh() -> o3d.t.geometry.TriangleMesh   # tensor mesh, lives on the mapper's device
Mapper(...).trajectory -> list[np.ndarray]          # list of 4x4

# slam/config.py
SlamConfig(dataclass) ... device: str = "CPU:0"     # add fields here
preferred_device() -> str                            # "CUDA:0" if o3d.core.cuda.is_available() else "CPU:0"

# panel.py -- two SlamWorker construction sites to reroute
#   ~1231:  self.slam_worker = SlamWorker(w, h, fov_h=..., fov_v=..., device=preferred_device())
#   ~1710:  self._showcase_preview_worker = SlamWorker(w, h, fov_h=..., fov_v=..., device=preferred_device())
```

The submit tuple carries: `depth float32[H,W]`, optional `reflectance`/`confidence float32[H,W]`, `quat` (len-4), `pressure float|None`, and (for the wire) a monotone frame id. The result carries a `FrameStep`, the full trajectory (list of 4x4), and a throttled mesh.

---

### Task 1: `wire` — message framing + mesh codec

**Files:**
- Create: `host/src/roomscan/slam/wire.py`
- Test: `host/tests/test_slam_wire.py`

**Interfaces:**
- Produces:
  - `encode_message(fields: dict) -> bytes` / `decode_message(buf: memoryview|bytes) -> dict` — a message is a dict whose values are either JSON scalars (int/float/bool/str/None) or `np.ndarray`. Framing = 4-byte big-endian total length, then a JSON header describing each field (`{"scalars": {...}, "arrays": {name: [dtype_str, [shape...]]}}`), then the raw array bytes concatenated in header order.
  - `send_message(sock, fields: dict) -> None` — length-prefixed write of `encode_message`.
  - `recv_message(sock) -> dict | None` — blocking read of one framed message; returns `None` on clean EOF.
  - `mesh_to_arrays(mesh) -> dict` — pulls a tensor `TriangleMesh` (any device) to CPU and returns `{"mesh_v": f32[N,3], "mesh_t": i32[M,3]}` plus `"mesh_c": f32[N,3]` when vertex colors exist. Empty mesh → zero-length arrays. Imports `open3d` lazily.
  - `arrays_to_mesh(d: dict) -> o3d.t.geometry.TriangleMesh` — inverse; rebuilds a CPU tensor mesh from `mesh_v`/`mesh_t`/optional `mesh_c`. Imports `open3d` lazily.

- [ ] **Step 1: Write the failing test**

```python
# host/tests/test_slam_wire.py
import io, socket, threading
import numpy as np
import pytest
from roomscan.slam import wire


def test_encode_decode_roundtrip_mixed_scalars_and_arrays():
    depth = np.arange(54 * 42, dtype=np.float32).reshape(42, 54)
    fields = {
        "fid": 7,
        "pressure": None,
        "tracking_lost": True,
        "slam_ms": 12.5,
        "depth": depth,
        "quat": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    }
    out = wire.decode_message(wire.encode_message(fields))
    assert out["fid"] == 7
    assert out["pressure"] is None
    assert out["tracking_lost"] is True
    assert out["slam_ms"] == pytest.approx(12.5)
    np.testing.assert_array_equal(out["depth"], depth)
    assert out["depth"].dtype == np.float32
    np.testing.assert_array_equal(out["quat"], fields["quat"])


def test_send_recv_over_socketpair():
    a, b = socket.socketpair()
    fields = {"fid": 3, "pose": np.eye(4, dtype=np.float32)}
    t = threading.Thread(target=wire.send_message, args=(a, fields))
    t.start()
    got = wire.recv_message(b)
    t.join()
    assert got["fid"] == 3
    np.testing.assert_array_equal(got["pose"], np.eye(4, dtype=np.float32))
    a.close(); b.close()


def test_recv_message_returns_none_on_eof():
    a, b = socket.socketpair()
    a.close()
    assert wire.recv_message(b) is None
    b.close()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_slam_wire.py -q`
Expected: FAIL — `ModuleNotFoundError: roomscan.slam.wire`.

> Note: `socket.socketpair()` works on Windows for AF_INET/AF_UNIX emulation in CPython 3.5+. If it is unavailable in the runner, substitute a loopback `AF_INET` pair bound to `127.0.0.1:0`.

- [ ] **Step 3: Write minimal implementation**

```python
# host/src/roomscan/slam/wire.py
"""Length-prefixed framing for the SLAM compute service (internal IPC, NOT the
device wire protocol -- deliberately independent of protocol.py/CRC).

A "message" is a dict of JSON scalars and numpy ndarrays. On the wire:
  [4-byte BE total-length][json header][raw array bytes in header order]
The header is {"scalars": {...}, "arrays": {name: [dtype_str, [shape...]]}}.
"""
from __future__ import annotations

import json
import struct
import numpy as np

_LEN = struct.Struct(">I")


def encode_message(fields: dict) -> bytes:
    scalars, arrays, blobs = {}, {}, []
    for k, v in fields.items():
        if isinstance(v, np.ndarray):
            v = np.ascontiguousarray(v)
            arrays[k] = [v.dtype.str, list(v.shape)]
            blobs.append(v.tobytes())
        else:
            scalars[k] = v
    header = json.dumps({"scalars": scalars, "arrays": arrays}).encode("utf-8")
    body = b"".join(blobs)
    return _LEN.pack(len(header)) + header + body


def decode_message(buf) -> dict:
    mv = memoryview(buf)
    (hlen,) = _LEN.unpack(mv[:4])
    header = json.loads(bytes(mv[4:4 + hlen]))
    out = dict(header["scalars"])
    off = 4 + hlen
    for name, (dtype_str, shape) in header["arrays"].items():
        dt = np.dtype(dtype_str)
        n = int(np.prod(shape)) if shape else 1
        nbytes = n * dt.itemsize
        out[name] = np.frombuffer(bytes(mv[off:off + nbytes]), dtype=dt).reshape(shape)
        off += nbytes
    return out


def send_message(sock, fields: dict) -> None:
    payload = encode_message(fields)
    sock.sendall(_LEN.pack(len(payload)) + payload)


def _recv_exactly(sock, n: int) -> bytes | None:
    chunks, got = [], 0
    while got < n:
        chunk = sock.recv(n - got)
        if not chunk:
            return None
        chunks.append(chunk); got += len(chunk)
    return b"".join(chunks)


def recv_message(sock) -> dict | None:
    head = _recv_exactly(sock, 4)
    if head is None:
        return None
    (total,) = _LEN.unpack(head)
    payload = _recv_exactly(sock, total)
    if payload is None:
        return None
    return decode_message(payload)


def mesh_to_arrays(mesh) -> dict:
    m = mesh.cpu()
    v = m.vertex["positions"].numpy().astype(np.float32) if "positions" in m.vertex else np.zeros((0, 3), np.float32)
    t = m.triangle["indices"].numpy().astype(np.int32) if "indices" in m.triangle else np.zeros((0, 3), np.int32)
    out = {"mesh_v": np.ascontiguousarray(v), "mesh_t": np.ascontiguousarray(t)}
    if "colors" in m.vertex:
        out["mesh_c"] = np.ascontiguousarray(m.vertex["colors"].numpy().astype(np.float32))
    return out


def arrays_to_mesh(d: dict):
    import open3d as o3d
    o3c = o3d.core
    m = o3d.t.geometry.TriangleMesh()
    m.vertex["positions"] = o3c.Tensor(np.asarray(d["mesh_v"], np.float32))
    m.triangle["indices"] = o3c.Tensor(np.asarray(d["mesh_t"], np.int32))
    if "mesh_c" in d:
        m.vertex["colors"] = o3c.Tensor(np.asarray(d["mesh_c"], np.float32))
    return m
```

- [ ] **Step 4: Add the mesh round-trip test and run the file**

```python
# append to host/tests/test_slam_wire.py
def test_mesh_arrays_roundtrip():
    o3d = pytest.importorskip("open3d")
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
    t = np.array([[0, 1, 2]], np.int32)
    src = o3d.t.geometry.TriangleMesh()
    src.vertex["positions"] = o3d.core.Tensor(v)
    src.triangle["indices"] = o3d.core.Tensor(t)
    d = wire.mesh_to_arrays(src)
    # survives an encode/decode round-trip too
    d = wire.decode_message(wire.encode_message(d))
    rebuilt = wire.arrays_to_mesh(d)
    np.testing.assert_array_equal(rebuilt.vertex["positions"].numpy(), v)
    np.testing.assert_array_equal(rebuilt.triangle["indices"].numpy(), t)
```

Run: `cd host && python -m pytest tests/test_slam_wire.py -q`
Expected: PASS (4 tests; the mesh test skips if open3d is absent).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/slam/wire.py host/tests/test_slam_wire.py
git commit -m "feat(slam): wire framing + mesh codec for the compute service"
```

---

### Task 2: `service` — container-side SLAM server

**Files:**
- Create: `host/src/roomscan/slam/service.py`
- Test: `host/tests/test_slam_service.py`

**Interfaces:**
- Consumes: `wire.recv_message`/`send_message`/`mesh_to_arrays` (Task 1); `SlamWorker` (existing).
- Produces:
  - `class SlamService` with `serve_client(conn)` — reads submit messages, drives a `SlamWorker` synchronously (`submit` → `run_once` → `latest`), sends a result message per frame. A per-connection `SlamWorker` is created lazily on the **first** submit (it carries the `H,W` from `depth.shape`) so no frame shape is needed up front, mirroring the panel. Mesh arrays are included **only when the published mesh object changed** (identity check); each result carries a monotone `mesh_seq` so the client can reuse its cached mesh when arrays are omitted.
  - `serve(host="0.0.0.0", port=5555, device="CUDA:0", **mapper_kwargs) -> None` — accept loop, one client at a time; a dropped client resets state and re-accepts.
  - `main(argv=None) -> int` — CLI entrypoint (`--host`, `--port`, `--device`, plus SLAM knobs from `SlamConfig`); exits non-zero if `device` starts with `CUDA` and `o3d.core.cuda.is_available()` is False (fail fast, visible in `wslc logs`). Wired as `python -m roomscan.slam.service`.

Submit message fields: `{"fid": int, "depth": f32[H,W], "quat": f32[4], "pressure": float|null, optional "reflectance": f32[H,W], optional "confidence": f32[H,W]}`.
Result message fields: `{"fid": int, "pose": f32[4,4], "fitness": float, "rmse": float, "tracking_lost": bool, "slam_ms": float, "traj": f32[K,4,4], "tracking_lost_count": int, "mesh_seq": int, optional "mesh_v"/"mesh_t"/"mesh_c"}`.

- [ ] **Step 1: Write the failing test** (in-process CPU service on an ephemeral port, driven with raw `wire` calls)

```python
# host/tests/test_slam_service.py
import socket, threading
import numpy as np
import pytest
from roomscan.slam import wire
from roomscan.slam.service import SlamService

pytest.importorskip("open3d")

H, W = 42, 54


def _synthetic_frame(fid):
    depth = np.full((H, W), 500.0, np.float32)     # 0.5 m plane, mm
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    return {"fid": fid, "depth": depth, "quat": quat, "pressure": None}


def test_service_returns_stepresult_per_frame():
    srv = SlamService(device="CPU:0", fov_h=55.0, fov_v=42.0)
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]

    def accept_once():
        conn, _ = lsock.accept()
        srv.serve_client(conn)
        conn.close()
    th = threading.Thread(target=accept_once, daemon=True); th.start()

    cli = socket.create_connection(("127.0.0.1", port))
    results = []
    for fid in range(4):
        wire.send_message(cli, _synthetic_frame(fid))
        results.append(wire.recv_message(cli))
    cli.close(); lsock.close(); th.join(timeout=2)

    assert [r["fid"] for r in results] == [0, 1, 2, 3]
    for r in results:
        assert r["pose"].shape == (4, 4)
        assert isinstance(r["tracking_lost"], bool)
        assert r["traj"].shape[1:] == (4, 4)
    # mesh sent at least once, and mesh_seq is monotone non-decreasing
    seqs = [r["mesh_seq"] for r in results]
    assert seqs == sorted(seqs)
    assert any("mesh_v" in r for r in results)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_slam_service.py -q`
Expected: FAIL — `ImportError: cannot import name 'SlamService'`.

- [ ] **Step 3: Write minimal implementation**

```python
# host/src/roomscan/slam/service.py
"""Container-side SLAM compute service. Owns one Mapper (via SlamWorker for its
mesh throttle + publish logic) on a chosen Open3D device (CUDA:0 in the GPU
container). One client at a time; localhost-only; no auth. See
docs/superpowers/specs/2026-07-13-slam-gpu-container-service-design.md."""
from __future__ import annotations

import argparse
import socket
import sys

import numpy as np

from . import wire
from .config import SlamConfig
from .worker import SlamWorker


class SlamService:
    def __init__(self, device="CUDA:0", mesh_every=5, **mapper_kwargs):
        self._device = device
        self._mesh_every = mesh_every
        self._mapper_kwargs = mapper_kwargs

    def serve_client(self, conn) -> None:
        worker = None
        last_mesh = object()          # sentinel; never equal to a real mesh
        mesh_seq = 0
        while True:
            msg = wire.recv_message(conn)
            if msg is None:
                break
            depth = np.asarray(msg["depth"], np.float32)
            if worker is None:
                h, w = depth.shape
                worker = SlamWorker(w, h, mesh_every=self._mesh_every,
                                    device=self._device, **self._mapper_kwargs)
            quat = np.asarray(msg["quat"], np.float32)
            pressure = msg.get("pressure")
            refl = msg.get("reflectance")
            conf = msg.get("confidence")
            worker.submit(depth, quat, pressure,
                          reflectance=None if refl is None else np.asarray(refl, np.float32),
                          confidence=None if conf is None else np.asarray(conf, np.float32))
            worker.run_once()
            mesh, traj, step = worker.latest()

            out = {
                "fid": int(msg["fid"]),
                "pose": np.asarray(step.pose, np.float32),
                "fitness": float(step.fitness),
                "rmse": float(step.rmse),
                "tracking_lost": bool(step.tracking_lost),
                "slam_ms": float(step.slam_ms),
                "traj": np.asarray(traj, np.float32) if traj else np.zeros((0, 4, 4), np.float32),
                "tracking_lost_count": int(worker.tracking_lost_count),
            }
            if mesh is not None and mesh is not last_mesh:
                out.update(wire.mesh_to_arrays(mesh))
                last_mesh = mesh
                mesh_seq += 1
            out["mesh_seq"] = mesh_seq
            wire.send_message(conn, out)


def serve(host="0.0.0.0", port=5555, device="CUDA:0", **mapper_kwargs) -> None:
    srv = SlamService(device=device, **mapper_kwargs)
    lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    lsock.bind((host, port)); lsock.listen(1)
    print(f"[slam-service] listening on {host}:{port} device={device}", flush=True)
    while True:
        conn, addr = lsock.accept()
        print(f"[slam-service] client {addr} connected", flush=True)
        try:
            srv.serve_client(conn)
        except (ConnectionError, OSError) as e:
            print(f"[slam-service] client dropped: {e}", flush=True)
        finally:
            conn.close()
            print("[slam-service] client disconnected; awaiting next", flush=True)


def main(argv=None) -> int:
    cfg = SlamConfig.load()
    ap = argparse.ArgumentParser(prog="roomscan-slam-service")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--device", default="CUDA:0")
    args = ap.parse_args(argv)

    if args.device.upper().startswith("CUDA"):
        import open3d as o3d
        if not o3d.core.cuda.is_available():
            print("[slam-service] CUDA requested but not available in this container",
                  file=sys.stderr)
            return 2

    serve(host=args.host, port=args.port, device=args.device,
          fov_h=cfg.fov_h, fov_v=cfg.fov_v, voxel_size=cfg.voxel_size,
          icp_mode=cfg.icp_mode, baro_weight=cfg.baro_weight, max_dist=cfg.max_dist,
          min_fitness=cfg.min_fitness, max_rmse=cfg.max_rmse,
          min_confidence=cfg.min_confidence, weight_threshold=cfg.weight_threshold,
          stationary_hold=cfg.stationary_hold, stationary_window=cfg.stationary_window,
          stationary_coherence=cfg.stationary_coherence,
          stationary_step_ceiling=cfg.stationary_step_ceiling,
          stationary_rot_ceiling=cfg.stationary_rot_ceiling)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_slam_service.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/slam/service.py host/tests/test_slam_service.py
git commit -m "feat(slam): container-side compute service over the wire framing"
```

---

### Task 3: `remote` — `RemoteSlamWorker` client

**Files:**
- Create: `host/src/roomscan/slam/remote.py`
- Test: `host/tests/test_slam_remote.py`

**Interfaces:**
- Consumes: `wire` (Task 1), `SlamService` (Task 2).
- Produces:
  - `class RemoteSlamWorker` mirroring `SlamWorker`: `__init__(width, height, addr="127.0.0.1:5555", mesh_every=5, connect_timeout=1.0, **mapper_kwargs)` (extra kwargs accepted and ignored so the panel can pass the same kwargs as the local worker), `.submit(depth, quat, pressure, reflectance=None, confidence=None)`, `.latest() -> (mesh, trajectory, FrameStep) | None`, `.start()`, `.stop()`, `.tracking_lost_count` (property).
  - `.connect() -> bool` — attempt the TCP connection; returns False on failure (caller uses this to decide fallback). `start()` calls it.
  - Internals: a send happens inline on `submit` (latest-wins: only the newest un-sent frame is kept if the socket is mid-write — a background sender thread drains a single slot). A background **receiver thread** reads result messages, rebuilds a `FrameStep` + trajectory (list of 4x4) + mesh (via `wire.arrays_to_mesh`, cached and reused when `mesh_seq` is unchanged), and stores `(mesh, trajectory, FrameStep)` in a lock-guarded latest-wins slot read by `latest()`. `FrameStep` is imported from `.mapper`.

- [ ] **Step 1: Write the failing test** (loopback: real `SlamService` on CPU, driven through `RemoteSlamWorker`; assert equivalence with a direct local `SlamWorker`)

```python
# host/tests/test_slam_remote.py
import socket, threading, time
import numpy as np
import pytest
from roomscan.slam.remote import RemoteSlamWorker
from roomscan.slam.service import SlamService

pytest.importorskip("open3d")

H, W = 42, 54


def _serve_on_ephemeral(device="CPU:0"):
    srv = SlamService(device=device, fov_h=55.0, fov_v=42.0)
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]

    def loop():
        conn, _ = lsock.accept()
        try:
            srv.serve_client(conn)
        except OSError:
            pass
        finally:
            conn.close()
    th = threading.Thread(target=loop, daemon=True); th.start()
    return port, lsock, th


def test_remote_worker_publishes_results():
    port, lsock, th = _serve_on_ephemeral()
    rw = RemoteSlamWorker(W, H, addr=f"127.0.0.1:{port}", fov_h=55.0, fov_v=42.0)
    assert rw.connect() is True
    rw.start()
    depth = np.full((H, W), 500.0, np.float32)
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    got = None
    for _ in range(200):                       # poll up to ~2 s for the first result
        rw.submit(depth, quat, None)
        time.sleep(0.01)
        got = rw.latest()
        if got is not None:
            break
    rw.stop(); lsock.close(); th.join(timeout=2)
    assert got is not None
    mesh, traj, step = got
    assert step.pose.shape == (4, 4)
    assert isinstance(traj, list)
    assert rw.tracking_lost_count >= 0


def test_connect_returns_false_when_no_server():
    rw = RemoteSlamWorker(W, H, addr="127.0.0.1:1", connect_timeout=0.3)
    assert rw.connect() is False
    assert rw.latest() is None       # never raises
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_slam_remote.py -q`
Expected: FAIL — `ModuleNotFoundError: roomscan.slam.remote`.

- [ ] **Step 3: Write minimal implementation**

```python
# host/src/roomscan/slam/remote.py
"""Client-side drop-in for SlamWorker that ships frames to a SlamService over a
localhost socket and republishes the returned (mesh, trajectory, FrameStep).
Same interface as slam.worker.SlamWorker, so panel.py is agnostic to which one
it holds. On any socket failure the caller falls back to the local worker."""
from __future__ import annotations

import socket
import threading
import time

import numpy as np

from . import wire
from .mapper import FrameStep

_IDLE_SLEEP_S = 0.005


class RemoteSlamWorker:
    def __init__(self, width, height, addr="127.0.0.1:5555", mesh_every=5,
                 connect_timeout=1.0, **mapper_kwargs):
        self._w, self._h = width, height
        host, _, port = addr.partition(":")
        self._host, self._port = host, int(port)
        self._connect_timeout = connect_timeout
        self._sock = None
        self._fid = 0

        self._in_lock = threading.Lock()
        self._in_slot = None
        self._out_lock = threading.Lock()
        self._out_slot = None
        self._tracking_lost_count = 0

        self._last_mesh_seq = -1
        self._last_mesh = None

        self._threads = []
        self._stop_evt = threading.Event()

    def connect(self) -> bool:
        try:
            self._sock = socket.create_connection(
                (self._host, self._port), timeout=self._connect_timeout)
            self._sock.settimeout(None)
            return True
        except OSError:
            self._sock = None
            return False

    def submit(self, depth, quat, pressure, reflectance=None, confidence=None) -> None:
        with self._in_lock:
            self._fid += 1
            msg = {"fid": self._fid,
                   "depth": np.asarray(depth, np.float32),
                   "quat": np.asarray(quat, np.float32),
                   "pressure": None if pressure is None else float(pressure)}
            if reflectance is not None:
                msg["reflectance"] = np.asarray(reflectance, np.float32)
            if confidence is not None:
                msg["confidence"] = np.asarray(confidence, np.float32)
            self._in_slot = msg

    def latest(self):
        with self._out_lock:
            return self._out_slot

    @property
    def tracking_lost_count(self) -> int:
        return self._tracking_lost_count

    def start(self) -> None:
        if self._sock is None and not self.connect():
            raise ConnectionError(f"slam-service unreachable at {self._host}:{self._port}")
        self._stop_evt.clear()
        self._threads = [threading.Thread(target=self._send_loop, daemon=True),
                         threading.Thread(target=self._recv_loop, daemon=True)]
        for t in self._threads:
            t.start()

    def _send_loop(self) -> None:
        while not self._stop_evt.is_set():
            with self._in_lock:
                msg, self._in_slot = self._in_slot, None
            if msg is None:
                time.sleep(_IDLE_SLEEP_S); continue
            try:
                wire.send_message(self._sock, msg)
            except OSError:
                break

    def _recv_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                res = wire.recv_message(self._sock)
            except OSError:
                break
            if res is None:
                break
            step = FrameStep(pose=np.asarray(res["pose"], np.float64),
                             fitness=res["fitness"], rmse=res["rmse"],
                             tracking_lost=res["tracking_lost"], slam_ms=res["slam_ms"])
            traj = [np.asarray(p, np.float64) for p in res["traj"]]
            self._tracking_lost_count = res["tracking_lost_count"]
            if res["mesh_seq"] != self._last_mesh_seq and "mesh_v" in res:
                self._last_mesh = wire.arrays_to_mesh(res)
                self._last_mesh_seq = res["mesh_seq"]
            with self._out_lock:
                self._out_slot = (self._last_mesh, traj, step)

    def stop(self) -> None:
        self._stop_evt.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None
        for t in self._threads:
            t.join(timeout=1.5)
        self._threads = []
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd host && python -m pytest tests/test_slam_remote.py -q`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/slam/remote.py host/tests/test_slam_remote.py
git commit -m "feat(slam): RemoteSlamWorker client mirroring SlamWorker's interface"
```

---

### Task 4: config fields + backend factory + panel wiring

**Files:**
- Modify: `host/src/roomscan/slam/config.py` (add two fields)
- Create: `host/src/roomscan/slam/backend.py` (factory + fallback)
- Modify: `host/src/roomscan/panel.py` (two `SlamWorker(...)` sites → `make_slam_worker(...)`)
- Test: `host/tests/test_slam_backend.py`

**Interfaces:**
- Consumes: `SlamWorker` (existing), `RemoteSlamWorker` (Task 3), `SlamConfig` (existing).
- Produces:
  - `SlamConfig.backend: str = "local"` and `SlamConfig.remote_addr: str = "127.0.0.1:5555"` (picked up automatically by `load()`'s field filter).
  - `make_slam_worker(width, height, cfg=None, **mapper_kwargs)` in `backend.py` — returns a started-capable worker: if `cfg.backend == "remote"`, build a `RemoteSlamWorker(addr=cfg.remote_addr, ...)` and try `connect()`; on failure log to stderr and **fall back** to a local `SlamWorker`. Otherwise return a local `SlamWorker`. `cfg` defaults to `SlamConfig.load()`. The returned object is not yet started (caller calls `.start()`, same as today).

- [ ] **Step 1: Write the failing test**

```python
# host/tests/test_slam_backend.py
import numpy as np
import pytest
from roomscan.slam.config import SlamConfig
from roomscan.slam.backend import make_slam_worker
from roomscan.slam.worker import SlamWorker

pytest.importorskip("open3d")


def test_config_has_backend_defaults():
    cfg = SlamConfig()
    assert cfg.backend == "local"
    assert cfg.remote_addr == "127.0.0.1:5555"


def test_local_backend_returns_local_worker():
    cfg = SlamConfig(backend="local")
    w = make_slam_worker(54, 42, cfg=cfg, fov_h=55.0, fov_v=42.0, device="CPU:0")
    assert isinstance(w, SlamWorker)


def test_remote_backend_falls_back_to_local_when_unreachable():
    cfg = SlamConfig(backend="remote", remote_addr="127.0.0.1:1")
    w = make_slam_worker(54, 42, cfg=cfg, fov_h=55.0, fov_v=42.0, device="CPU:0")
    # unreachable service -> silent fall back to the in-process CPU worker
    assert isinstance(w, SlamWorker)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd host && python -m pytest tests/test_slam_backend.py -q`
Expected: FAIL — `ModuleNotFoundError: roomscan.slam.backend` (and the config-defaults test fails until the fields are added).

- [ ] **Step 3: Add config fields**

```python
# host/src/roomscan/slam/config.py -- add after the `device` field (~line 69)
    # Compute backend for the live worker: "local" runs Mapper in-process
    # (default, unchanged behavior); "remote" ships frames to a SlamService
    # (GPU WSL container) at remote_addr, falling back to local if unreachable.
    backend: str = "local"
    remote_addr: str = "127.0.0.1:5555"
```

- [ ] **Step 4: Create the factory**

```python
# host/src/roomscan/slam/backend.py
"""Select the live SLAM worker backend from config: in-process CPU SlamWorker
(default) or a RemoteSlamWorker talking to the GPU container's SlamService,
with automatic fallback to local when the service is unreachable."""
from __future__ import annotations

import sys

from .config import SlamConfig
from .worker import SlamWorker


def make_slam_worker(width, height, cfg=None, **mapper_kwargs):
    cfg = cfg if cfg is not None else SlamConfig.load()
    if cfg.backend == "remote":
        from .remote import RemoteSlamWorker
        rw = RemoteSlamWorker(width, height, addr=cfg.remote_addr, **mapper_kwargs)
        if rw.connect():
            return rw
        print(f"[slam] remote backend at {cfg.remote_addr} unreachable; "
              f"falling back to local CPU worker", file=sys.stderr)
    return SlamWorker(width, height, **mapper_kwargs)
```

- [ ] **Step 5: Run the backend tests**

Run: `cd host && python -m pytest tests/test_slam_backend.py -q`
Expected: PASS (3 tests).

- [ ] **Step 6: Reroute the two panel construction sites**

At `panel.py:~1229` the code does `from .slam.config import preferred_device`; add the factory import alongside and replace the constructor. Change:

```python
            from .slam.config import preferred_device
            h, w = depth.shape
            self.slam_worker = SlamWorker(w, h, fov_h=self.args.fov_h, fov_v=self.args.fov_v,
                                          device=preferred_device())
            self.slam_worker.start()
```

to:

```python
            from .slam.config import preferred_device
            from .slam.backend import make_slam_worker
            h, w = depth.shape
            self.slam_worker = make_slam_worker(w, h, fov_h=self.args.fov_h,
                                                fov_v=self.args.fov_v,
                                                device=preferred_device())
            self.slam_worker.start()
```

And at `panel.py:~1708` change:

```python
            self._showcase_preview_worker = SlamWorker(w, h, fov_h=self.args.fov_h,
                                                        fov_v=self.args.fov_v,
                                                        device=preferred_device())
```

to:

```python
            from .slam.backend import make_slam_worker
            self._showcase_preview_worker = make_slam_worker(w, h, fov_h=self.args.fov_h,
                                                             fov_v=self.args.fov_v,
                                                             device=preferred_device())
```

> Leave the top-of-file `from .slam.worker import SlamWorker` import as-is if other code references the class; the factory does not remove it.

- [ ] **Step 7: Run the panel test suite to confirm no regression**

Run: `cd host && python -m pytest tests/test_panel_ux.py tests/test_panel_walls.py tests/test_slam_backend.py -q`
Expected: PASS (existing panel tests unchanged; new backend tests green).

- [ ] **Step 8: Commit**

```bash
git add host/src/roomscan/slam/config.py host/src/roomscan/slam/backend.py host/src/roomscan/panel.py host/tests/test_slam_backend.py
git commit -m "feat(slam): backend factory + remote/local config, panel routes through it"
```

---

### Task 5: container image + lifecycle scripts

**Files:**
- Create: `tools/slam-container/Dockerfile`
- Create: `tools/slam-container/build.ps1`
- Create: `tools/slam-container/start.ps1`
- Create: `tools/slam-container/stop.ps1`
- Create: `tools/slam-container/README.md`

**Interfaces:**
- Consumes: the `roomscan` package (installed into the image), `roomscan.slam.service:main` entrypoint (Task 2).
- Produces: image `roomscan-slam:cuda`; a detached container `roomscan-slam` listening on `127.0.0.1:5555`.

> This task's deliverable is validated by building the image and connecting to the running service from Windows, not by pytest.

- [ ] **Step 1: Write the Dockerfile**

```dockerfile
# tools/slam-container/Dockerfile
# GPU SLAM compute service. Open3D 0.19 Linux wheel bundles its CUDA runtime;
# the driver arrives via `wslc run --gpus all`. Only libgl1/libgomp1/libx11-6
# are needed for `import open3d` (verified on this box, 2026-07-13).
FROM python:3.12-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libgomp1 libx11-6 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir "open3d==0.19.0" "numpy>=1.26" "pyserial>=3.5" "psutil>=5.9" "Pillow>=10"

# Install the roomscan package (build context = repo root's host/).
COPY host/ /app/host/
RUN pip install --no-cache-dir --no-deps /app/host

EXPOSE 5555
ENTRYPOINT ["python", "-m", "roomscan.slam.service"]
CMD ["--host", "0.0.0.0", "--port", "5555", "--device", "CUDA:0"]
```

- [ ] **Step 2: Write build.ps1**

```powershell
# tools/slam-container/build.ps1
# Build roomscan-slam:cuda. Build context is the repo's host/ parent so the
# Dockerfile's `COPY host/` sees the package.
$ErrorActionPreference = "Stop"
$repo = Resolve-Path "$PSScriptRoot/../.."
Write-Host "Building roomscan-slam:cuda from $repo ..."
wslc build -t roomscan-slam:cuda -f "$PSScriptRoot/Dockerfile" "$repo"
Write-Host "Done. Run start.ps1 to launch the detached service."
```

- [ ] **Step 3: Write start.ps1 (idempotent, detached)**

```powershell
# tools/slam-container/start.ps1
# Launch the GPU SLAM service detached on 127.0.0.1:5555. Idempotent: if a
# container named roomscan-slam already runs, do nothing.
$ErrorActionPreference = "Stop"
$name = "roomscan-slam"
$running = (wslc list 2>$null) -match $name
if ($running) { Write-Host "$name already running."; exit 0 }
# remove any stopped leftover with the same name
wslc remove $name 2>$null | Out-Null
wslc run -d --name $name --gpus all --publish 5555:5555 roomscan-slam:cuda
Write-Host "Started $name. Check: wslc logs $name"
```

- [ ] **Step 4: Write stop.ps1**

```powershell
# tools/slam-container/stop.ps1
$ErrorActionPreference = "SilentlyContinue"
wslc stop roomscan-slam | Out-Null
wslc remove roomscan-slam | Out-Null
Write-Host "Stopped roomscan-slam."
```

- [ ] **Step 5: Build the image**

Run: `pwsh tools/slam-container/build.ps1`
Expected: image builds; final line "Done." (First build pulls `python:3.12-slim` + the Open3D wheel; several minutes.)

- [ ] **Step 6: Start the service and verify it is listening on the GPU**

Run:
```powershell
pwsh tools/slam-container/start.ps1
Start-Sleep -Seconds 3
wslc logs roomscan-slam
```
Expected logs contain: `[slam-service] listening on 0.0.0.0:5555 device=CUDA:0` and **no** "CUDA requested but not available". If the CUDA-unavailable line appears, the `--gpus all` passthrough or driver is the fault — re-run the Task 0/probe check from the plan intro.

- [ ] **Step 7: Verify Windows -> container reachability**

Run:
```powershell
Test-NetConnection 127.0.0.1 -Port 5555
```
Expected: `TcpTestSucceeded : True`. If False, WSL localhost forwarding is not mirroring the published port — record the container's WSL IP (`wslc inspect roomscan-slam`) and set `remote_addr` in `roomscan.toml` to `<wsl-ip>:5555` instead of `127.0.0.1:5555`.

- [ ] **Step 8: Write README.md and commit**

```markdown
# tools/slam-container

GPU SLAM compute service (Phase 6). Windows captures + renders; this container
runs Mapper on CUDA:0 behind SlamService. See
docs/superpowers/specs/2026-07-13-slam-gpu-container-service-design.md.

- `build.ps1`  — build image roomscan-slam:cuda
- `start.ps1`  — launch detached on 127.0.0.1:5555 (idempotent)
- `stop.ps1`   — stop + remove the container
- logs:  wslc logs roomscan-slam

Enable in roomscan.toml:
    [slam]
    backend = "remote"
    remote_addr = "127.0.0.1:5555"
```

```bash
git add tools/slam-container/
git commit -m "feat(slam): WSL GPU container image + build/start/stop scripts"
```

---

### Task 6: end-to-end GPU validation

**Files:**
- Create: `tools/slam-container/verify_e2e.py` (a manual validation harness, not a pytest test)

**Interfaces:**
- Consumes: `RemoteSlamWorker` (Task 3), the running container (Task 5), a recorded capture `.bin`.
- Produces: a printed comparison of remote-GPU vs local-CPU `slam_ms` over the same capture, proving the offload works and is faster.

- [ ] **Step 1: Write the harness**

```python
# tools/slam-container/verify_e2e.py
"""Replay a recorded capture through RemoteSlamWorker (GPU container) and a
local CPU SlamWorker, and compare median per-frame slam_ms. Run with the
container started (tools/slam-container/start.ps1).

Usage: python tools/slam-container/verify_e2e.py <capture.bin> [--addr 127.0.0.1:5555]
"""
import argparse, statistics, time
from roomscan.slam.cli import _load_frames
from roomscan.slam.remote import RemoteSlamWorker
from roomscan.slam.worker import SlamWorker


def drive(worker, frames):
    ms = []
    for depth, refl, conf, quat, pa, t in frames:
        worker.submit(depth, quat, pa, reflectance=refl, confidence=conf)
        # remote: poll latest until a new result arrives; local: run_once
        if hasattr(worker, "run_once"):
            worker.run_once()
            got = worker.latest()
        else:
            got = None
            for _ in range(200):
                time.sleep(0.005); got = worker.latest()
                if got is not None:
                    break
        if got is not None:
            ms.append(got[2].slam_ms)
    return ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--addr", default="127.0.0.1:5555")
    args = ap.parse_args()
    frames, w, h = _load_frames(args.capture)
    print(f"{len(frames)} frames {w}x{h}")

    rw = RemoteSlamWorker(w, h, addr=args.addr, fov_h=55.0, fov_v=42.0)
    assert rw.connect(), f"no service at {args.addr} -- run start.ps1"
    rw.start()
    remote_ms = drive(rw, frames); rw.stop()

    lw = SlamWorker(w, h, fov_h=55.0, fov_v=42.0, device="CPU:0")
    local_ms = drive(lw, frames)

    print(f"remote(GPU) median slam_ms = {statistics.median(remote_ms):.2f} (n={len(remote_ms)})")
    print(f"local(CPU)  median slam_ms = {statistics.median(local_ms):.2f} (n={len(local_ms)})")


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Run it against a recorded capture**

Run (container started first):
```powershell
pwsh tools/slam-container/start.ps1
cd host && python ../tools/slam-container/verify_e2e.py <path-to-capture>.bin
```
Expected: both lines print with sane frame counts; the remote/GPU median is reported. Confirm the container is doing GPU work while it runs: `wslc stats roomscan-slam` shows activity, and the startup log said `device=CUDA:0`.

- [ ] **Step 3: Commit**

```bash
git add tools/slam-container/verify_e2e.py
git commit -m "test(slam): end-to-end GPU-vs-CPU slam_ms validation harness"
```

---

## Self-Review

**Spec coverage:**
- Verified premises / modern-compatible stack → Task 5 Dockerfile (python:3.12-slim + open3d 0.19), plan intro. ✔
- `SlamWorker` seam / `RemoteSlamWorker` same interface → Task 3. ✔
- Container-side black-box service on CUDA:0 → Task 2. ✔
- Wire framing independent of protocol.py → Task 1. ✔
- Throttled mesh return (reuse publish shape) → Task 2 reuses `SlamWorker` + `mesh_seq` identity gate; Task 3 caches/reuses mesh. ✔
- Config `backend`/`remote_addr` + panel routing → Task 4. ✔
- Persistent detached lifecycle via scripts → Task 5. ✔
- Graceful fallback to local CPU worker → Task 4 `make_slam_worker`; Task 3 `connect()->bool`. ✔
- Transport / reachability verification + WSL-IP fixup → Task 5 Steps 6–7. ✔
- GPU-free / container-free tests → Tasks 1–4 (CPU loopback, ephemeral ports). ✔
- End-to-end GPU validation → Task 6. ✔

**Placeholder scan:** No TBD/TODO; every code step shows complete code; every command has an expected result. ✔

**Type consistency:** `submit(depth, quat, pressure, reflectance=None, confidence=None)` and `latest() -> (mesh, trajectory, FrameStep)` identical across `SlamWorker`, `SlamService` (server drives a `SlamWorker`), and `RemoteSlamWorker`. `FrameStep(pose, fitness, rmse, tracking_lost, slam_ms)` reconstructed with the exact field names in Task 3. Message field names (`fid`, `pose`, `traj`, `mesh_seq`, `mesh_v/t/c`, `tracking_lost_count`) match between Task 2 (producer) and Task 3 (consumer). `make_slam_worker(width, height, cfg=None, **mapper_kwargs)` signature consistent between Task 4 definition and the two panel call sites. ✔
