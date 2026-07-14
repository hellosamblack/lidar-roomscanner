# Live-view fps: off-thread adaptive mesh rendering + pose/mesh transport split

**Date:** 2026-07-13
**Status:** Design approved, spec under review
**Owner:** hellosamblack
**Related:** `docs/superpowers/specs/2026-07-13-slam-gpu-container-service-design.md`, memory `cuda-at-scale-validation`

## Problem

The live SLAM view must feel snappy: **≥30 fps viewport minimum, ideally 120+**, smooth camera, **without sacrificing final-map quality** and with minimal live-preview-quality loss (owner directive, 2026-07-13). Two measurements confirm the bottleneck is **rendering/transport architecture, not compute**:

- Per-frame SLAM `step()` is ~9 ms on GPU (2.1× over CPU's ~19 ms) and **flat over the run** — plenty of headroom for the sensor's ~28 fps data rate (I3C ceiling). Compute is not the limiter.
- The stall is on the **GUI thread**: `panel._render_slam_frame` runs on Open3D's `set_on_tick_event`, and on each tick where a new mesh arrives it does wall-classification (`_wall_submesh` / `shading.wall_triangle_mask`), full-vertex `.cpu().numpy()` copies (`_update_floor_grid`), and an `add_geometry` **full-replace of the growing mesh** — all O(map size). As the map grows these hitch and worsen (the "gets worse over time" the owner observed).
- On the **remote/container path**, the service sends one combined `(mesh, trajectory, FrameStep)` per frame and the **full trajectory every frame**, so a growing mesh delays pose delivery and the trajectory transfer is O(map size).

The "120 fps" target is a **viewport-render** goal (smooth camera decoupled from data arrival), not a new-scan-data goal (that is sensor-limited to ~28 fps).

## Goal

Keep the viewport at ≥30 fps (target 120), flat as the map grows, on both backends, by getting the O(map-size) work off the per-tick critical path and decoupling pose from mesh on the wire — while preserving the exact final map.

### Non-goals

- The container's GPU-memory OOM on a full-scale scan (a separate GPU-hardening step). This spec is latency/decoupling only.
- Changing the SLAM algorithm, the map, the wire *framing* (`wire.py`), or the device protocol.
- Raising the sensor data rate (firmware/I3C — out of scope).

## Shared invariant

Both components preserve the `SlamWorker`/`RemoteSlamWorker` contract: `latest() -> (mesh, trajectory, FrameStep) | None`. Component A consumes that tuple and is therefore **backend-agnostic** — it improves the local (in-process) and remote (container) live view identically. Decimation is **display-only**; the saved/offline map always comes from the full-resolution `mapper.mesh()`, so the final artifact is byte-identical to today.

## Component A — Off-thread adaptive mesh rendering (panel)

### Units

| Unit | Location | Responsibility | Depends on |
|------|----------|----------------|------------|
| `MeshPrep` | `host/src/roomscan/slam/meshprep.py` (new) | Off-GUI-thread stage: take the newest worker mesh, adaptively decimate to a vertex budget, do wall/non-wall split + floor-grid summary, publish a ready-to-upload `MeshPacket` into a latest-wins slot. | `shading` (wall mask), open3d, numpy |
| `MeshPacket` | same | Plain data: non-wall verts/colors/tris, wall submesh, floor-grid points, source `mesh_seq`/vertex count, `decimated: bool`. No open3d handles that must be built on the GUI thread beyond what `add_geometry` needs. | — |
| panel tick wiring | `host/src/roomscan/panel.py` (`_render_slam_frame`, `_upload_slam_mesh`) | Per tick: cheap pose/camera/FOV/trajectory-head. At a throttled cadence: pull the latest `MeshPacket` and `add_geometry` it (+ full trajectory ribbon + floor grid). | `MeshPrep` |
| render-fps counter | `host/src/roomscan/panel.py` | Count viewport ticks/sec (distinct from the existing data-fps `self._fps`); expose to HUD + log. | — |

### Behavior

- **MeshPrep** runs on its own daemon thread with a latest-wins input slot (fed the worker's `mesh` when it changes, keyed by identity/`mesh_seq`) and a latest-wins output `MeshPacket` slot. It performs the O(size) work — decimation, wall-split, floor extraction, numpy materialization — **off the GUI thread**.
- **Adaptive decimation:** MeshPrep tracks the last GUI upload's measured wall-time (fed back from the tick). If the last full-res upload exceeded the **frame budget** (`fps_budget_ms`, default targeting ~120 fps ⇒ a small per-upload ceiling), it decimates the next packet to a **target vertex budget** (`live_vertex_budget`, e.g. 150k) via Open3D quadric decimation / vertex clustering; while uploads fit the budget it stays full-res (`decimated=False`). This is the "adaptive" behavior — full quality until it would cost fps.
- **GUI tick** (`_render_slam_frame`): unchanged cheap path (pose label, FOV, follow-camera, trajectory *head* dot) every tick. The heavy path — `add_geometry` of the packet's mesh, the full trajectory *ribbon*, and the floor grid — runs only (a) when a **new** packet is available and (b) at most once per **mesh cadence** (`mesh_upload_hz`, default 2–5 Hz). The tick measures the upload wall-time and feeds it back to MeshPrep's adaptive controller.
- **Floor grid** is computed inside MeshPrep from the (decimated) vertices, replacing the per-tick full `.cpu().numpy()` in `_update_floor_grid`.
- **Trajectory:** the *head* marker updates every tick (O(1)); the full *ribbon* re-uploads only at the mesh cadence.

### Config (`[slam]` or a new `[view]` table in `roomscan.toml`)

`mesh_upload_hz` (default 3.0), `live_vertex_budget` (default 150000), `fps_budget_ms` (default 8.0 ≈ 120 fps). Defaults chosen so a small map stays full-res and only large maps decimate.

## Component B — Pose/mesh transport split (remote path)

### Wire

Add a 1-byte message-type tag to each framed message (within the existing `wire.py` framing; no framing change):
- `pose` (every frame): `fid, pose f32[4,4], fitness, rmse, tracking_lost, slam_ms, tracking_lost_count`. Tiny (~fixed size).
- `mesh` (only when a new throttled mesh is ready): `mesh_seq, mesh_v/mesh_t/mesh_c`.

### Service (`slam/service.py`)

`serve_client` runs its `SlamWorker` threaded. Per received frame: `step()` → send a `pose` message **immediately**. When `worker.latest()` yields a **new** mesh object (identity/`mesh_seq` changed), interleave a `mesh` message on the same socket. Mesh extraction/transfer never delays a `pose`. Client-config forwarding (`cfg`) and `device` ownership unchanged.

### Client (`slam/remote.py`)

The recv thread dispatches by tag:
- `pose` → update the latest `FrameStep`; **append `pose` to a client-side trajectory list** (delta accumulation — no full-traj transfer); publish `(cached_mesh, trajectory, step)`.
- `mesh` → rebuild + cache the mesh via `wire.arrays_to_mesh` when `mesh_seq` changes.

`latest()` returns the same `(mesh, trajectory, FrameStep)` tuple, so the panel and Component A are unchanged. Trajectory now grows on the client from deltas rather than being resent each frame.

## Data flow (remote + adaptive render, per frame)

1. Panel tick → `RemoteSlamWorker.submit(depth, quat, …)` (cheap).
2. Service `step()` on CUDA → `pose` message back immediately; client appends to trajectory, publishes step.
3. Every ~5 integrated frames the service extracts a mesh → `mesh` message; client caches it.
4. `MeshPrep` (client side, off GUI thread) sees the new cached mesh → adaptively prepares a `MeshPacket`.
5. Panel tick: pose/camera/head every tick (smooth); mesh/ribbon/floor uploaded from the packet at `mesh_upload_hz`.

## Validation

- **render-fps counter**: viewport ticks/sec, logged periodically.
- **Replay drive**: `roomscan-panel --replay <capture>` (or the CLI replay entry) with SLAM on, over a map-growing capture; capture render-fps early-third vs late-third. **Success: render-fps ≥30 sustained (target 120) and flat (late/early ≈ 1.0)**, on both `backend=local` and `backend=remote`. Compare against the current code's render-fps on the same capture to quantify the win.
- **Unit tests** (GPU-free, GUI-free): `MeshPrep` decimation kicks in past the vertex budget and stays full-res below it; `MeshPacket` wall/non-wall split matches the current `_wall_submesh` output on a fixture mesh; wire `pose`/`mesh` tagged-message round-trip; `RemoteSlamWorker` trajectory-delta accumulation + mesh caching against a loopback `SlamService`. Existing panel unit tests (`test_panel_walls.py`, `test_panel_ux.py`) stay green.

## Rollout order

1. `MeshPrep` + `MeshPacket` + tests (pure, off-GUI).
2. Panel wiring: throttled-cadence upload + render-fps counter + adaptive feedback; keep cheap per-tick path.
3. Wire `pose`/`mesh` tag + round-trip tests.
4. Service pose/mesh split; client dispatch + trajectory delta + tests.
5. Replay validation on both backends; record before/after render-fps.

## Open questions (resolved)

- Scope → **both** render-loop and remote transport in one spec. ✔
- Live mesh detail → **adaptive** display-only decimation (full-res until it would cost fps; final map always full-res). ✔
