# SLAM GPU compute offloaded to a WSL container (black-box service)

**Date:** 2026-07-13
**Status:** Design approved, spec under review
**Owner:** hellosamblack
**Related:** `docs/superpowers/specs/2026-07-10-phase6-slam-design.md`, memory `gpu-cuda-build-blocker`

## Problem

Phase 6 SLAM runs on Open3D's tensor pipeline and is fully device-parameterized
(`--device CUDA:0`), but a CUDA-enabled Open3D **cannot be built on this Windows
box**: MSVC 14.51 is too new for CUDA 12.6/13.3's `nvcc` (memory
`gpu-cuda-build-blocker`). The SLAM therefore runs CPU-only, which caps the live
and processed render.

Open3D ships **prebuilt Linux x86_64 CUDA wheels**. WSL2 on this machine now
supports GPU-passthrough **containers** via `wslc` (WSL Container CLI, shipped
with WSL 2.9.4). We move the GPU-bound SLAM compute into a container and keep the
Windows host responsible for USB capture and rendering.

### Verified premises (2026-07-13, this box)

- `wslc run --rm --gpus all pytorch/pytorch:2.5.1-cuda12.4-cudnn9-runtime …` →
  `torch.cuda.is_available() == True`, device = *NVIDIA GeForce RTX 4080 Laptop GPU*.
  GPU passthrough works.
- Clean `python:3.12-slim` + `apt install libgl1 libgomp1 libx11-6` +
  `pip install open3d` → **open3d 0.19.0, `o3d.core.cuda.is_available() == True`,
  `device_count == 1`**. The stock wheel carries its own CUDA runtime; only the
  GPU **driver** (via `--gpus all`) and three system libs are needed. No CUDA
  base image required.
- Repo pins `python>=3.11,<3.13`; **Python 3.12 + Open3D 0.19** is the
  most-modern compatible pairing.

## Goal

Faster **live** and **processed** render by running `Mapper.step()` on the GPU,
with the container as a **black box**: the Windows app routes per-frame data to
it and receives SLAM results back. No USB passthrough, no in-container GUI. The
GPU path is an **accelerator, never a hard dependency** — if the service is down,
the panel falls back to today's in-process CPU worker.

### Non-goals

- Running the live GUI panel or USB CDC capture inside the container.
- Changing the SLAM algorithm, protocol wire format, or firmware.
- Multi-GPU, multi-client, or networked (non-localhost) operation.
- Replacing the existing `roomscan-slam` offline CLI (it already accepts
  `--device CUDA:0` and will simply run inside the container too, unchanged).

## Architecture

```
Windows (host process)                     WSL container  (roomscan-slam:cuda, --gpus all)
┌──────────────────────────┐               ┌───────────────────────────────────┐
│ USB CDC capture           │               │ slam-service  (TCP :5555)          │
│ panel.py  (live render)   │               │   owns ONE Mapper(device=CUDA:0)   │
│ RemoteSlamWorker          │──frames──────▶│   loop: decode → mapper.step()     │
│   .submit()/.latest()     │◀─results──────│   publish (mesh, traj, FrameStep)  │
│   (falls back to local)   │               │   throttled mesh (_MESH_EVERY)     │
└──────────────────────────┘               └───────────────────────────────────┘
                    localhost TCP  (WSL2 localhost relay / wslc --publish 5555)
```

### The seam: `SlamWorker`'s interface

`panel.py` talks to the SLAM subsystem **only** through `SlamWorker`'s methods
(`worker.py`): `submit(depth, quat, pressure, reflectance=None, confidence=None)`,
`latest() -> (mesh, trajectory, FrameStep) | None`, `start()`, `stop()`,
`tracking_lost_count`. Both use latest-wins single slots, so a slow consumer only
ever sees the newest item.

We introduce **`RemoteSlamWorker`** implementing that exact interface. `submit()`
serializes the tuple and sends it over the socket; a receive thread stores the
newest `(mesh, trajectory, FrameStep)` reply in the same kind of latest-wins
slot; `latest()` returns it. `panel.py` chooses the backend from config and is
otherwise **unchanged** — no new render path (we return the throttled mesh, the
identical shape `SlamWorker` already publishes).

### Components

| Unit | Location | Responsibility | Depends on |
|------|----------|----------------|------------|
| `wire` | `host/src/roomscan/slam/wire.py` | Length-prefixed framing of the submit tuple and the result tuple over a socket. Header carries shapes+dtypes; bodies are raw `ndarray.tobytes()`. Mesh serialized as vertex/triangle/vertex-color arrays. | stdlib, numpy |
| `service` | `host/src/roomscan/slam/service.py` | Container-side TCP server. Owns one `Mapper` (or reuses `SlamWorker` internally for the throttle/publish logic), receives submit frames, steps, sends results. Single client, latest-wins. | `mapper`/`worker`, `wire` |
| `remote` | `host/src/roomscan/slam/remote.py` | `RemoteSlamWorker`: client mirroring `SlamWorker`'s interface; send on `submit`, background recv thread, `latest()`. Connect/reconnect + health. | `wire` |
| container image | `tools/slam-container/Dockerfile` | `python:3.12-slim` + `libgl1 libgomp1 libx11-6` + `pip install open3d` + install the `roomscan` package; entrypoint runs `service`. | Open3D wheel |
| lifecycle scripts | `tools/slam-container/build.ps1`, `start.ps1`, `stop.ps1` | Build `roomscan-slam:cuda` once; start it **detached** with `--gpus all --publish 5555:5555`; stop it. `start.ps1` is idempotent (no-op if already running). | `wslc` |

### Data flow (per live frame)

1. Panel's reader decodes a device frame, runs `TransformStage` (as today) for
   its **own instant local view** — unchanged, keeps live render fast.
2. Panel `submit(depth, quat, pressure, reflectance, confidence)` → `RemoteSlamWorker`.
3. `RemoteSlamWorker` frames the tuple and sends over TCP.
4. `service` receives, `mapper.step(...)` on **CUDA:0**, throttles `mesh()` on the
   existing `_MESH_EVERY` counter, frames `(mesh, trajectory, FrameStep)` back.
5. `RemoteSlamWorker`'s recv thread stores it; panel `latest()` renders it — same
   code path as the local worker today.

### Wire sizes

- **→ container:** depth `float32[H×W]` (+ optional reflectance/confidence,
  quat(4), pressure, t). 54×42 ⇒ ~9–27 KB/frame; ~28 fps ⇒ <1 MB/s.
- **← Windows:** trajectory + `FrameStep` every frame (tiny); **mesh throttled**
  to every `_MESH_EVERY` (=5) integrated frames, a few hundred KB each. Localhost
  handles both with headroom. Latest-wins on both ends means backpressure just
  drops stale frames, never queues.

### Transport & networking

- Container publishes `5555` via `wslc run --publish 5555:5555`; panel connects to
  `127.0.0.1:5555`. WSL2's localhost relay (mirrored networking on 2.9.4) makes
  this transparent; the plan's first task **empirically verifies** Windows→
  container reachability and records the exact address if a fixup is needed.
- Single client, single GPU, single `Mapper`. No auth (localhost-only, internal).

### Config

`roomscan.toml` `[slam]` gains:
- `backend = "local" | "remote"` (default `"local"` — opt in).
- `remote_addr = "127.0.0.1:5555"`.

`panel.py` constructs `RemoteSlamWorker` when `backend == "remote"`, else the
existing `SlamWorker`.

### Error handling & fallback

- If `RemoteSlamWorker` cannot connect (or the socket drops), the panel logs and
  **falls back to the in-process CPU `SlamWorker`** — today's behavior, no crash.
- `service` handles client disconnect by resetting to accept the next connection;
  a fresh connection may reset the `Mapper` (new scan) — decided in the plan.
- The service's `Mapper` runs on `CUDA:0`; if CUDA is unavailable inside the
  container it exits non-zero at startup (fail fast, visible in `wslc logs`).

## Testing

- `wire` round-trip: submit tuple and result tuple (incl. a mesh) serialize →
  deserialize byte-identical (arrays `allclose`, dtypes/shapes preserved).
- `remote`↔`service` **loopback on CPU device**: run the service in-process on
  `CPU:0` bound to an ephemeral port, drive it through `RemoteSlamWorker`, assert
  the published `(mesh, trajectory, FrameStep)` matches a direct local
  `SlamWorker` over the same synthetic frames. Keeps CI **GPU-free**.
- Fallback: `RemoteSlamWorker` with no server → panel wiring falls back to local
  worker without raising.
- Existing 458 tests stay green (all new files are additive).

## Rollout

1. Verify container reachability + Open3D CUDA end-to-end (probe already green).
2. `wire` + tests.
3. `service` + `remote` + loopback tests.
4. Dockerfile + build/start/stop scripts; build image; run service detached.
5. Wire `backend`/`remote_addr` config into `panel.py`; fallback.
6. End-to-end: replay a recorded capture through the live panel against the
   container; confirm GPU utilization and compare per-frame `slam_ms` vs CPU.

## Open questions (resolved)

- Return payload → **throttled mesh** (reuse existing publish shape). ✔
- Lifecycle → **persistent detached service** via `start.ps1`; panel connects,
  falls back if absent. ✔
