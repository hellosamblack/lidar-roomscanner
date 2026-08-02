# Web Phase 4 — SLAM mode (design)

Status: ✅ Complete (2026-07-16) — 69 backend tests (was 57), full host suite **637 passed / 1 skipped**,
driven end-to-end in headless Chrome against `captures/verify_slam.bin`: Real-Time↔SLAM switch, live mesh
reconstruction (Tracking OK, Fitness 0.85, RMSE ~11 mm, 330→1080 frames as it looped), trajectory + follow
camera, walls Split/Solid, and **Save** → `results/web_<ts>.ply` (+`.tum`) with a download link — all
confirmed on screen; SLAM ran on the local **CUDA:0** GPU. Predecessors: Web Phases 1 (core instrument),
2 (sensors), 3 (recording & playback). Same **host-only, reuse-don't-reimplement** discipline: confined to
`host/src/roomscan/web.py` + `host/src/roomscan/static/` + `host/tests/test_web.py`. **No edits to
`slam/`, `sensors.py`, `protocol.py`, `panel.py`, or firmware.** No device-wire change (the MESH message
is on the app `/ws` — see `docs/web-protocol.md`, which this phase extends).

**Owner decisions (2026-07-16):** (1) **GPU-accelerated** — run on the GPU, not CPU. (2) **Include a web
Save button** — write the reconstructed mesh + trajectory to disk from the browser (§8). Both are locked;
this spec reflects them.

**Compute is LOCAL GPU (discovered 2026-07-16).** The Proxmox host now passes an **RTX 2000 Ada** through
to this container (the `headless-host-deployment` memory's "no GPU" line is superseded): `.venv` Open3D
0.19 reports `cuda_available == True`, and the SLAM pipeline runs **in-process on `CUDA:0`** —
**verified** at **7.2 ms/frame** over `captures/verify_slam.bin` (329 frames, lost=0, a 29,921-vertex map).
So the owner's "GPU" decision is satisfied by the **local** worker with `device="CUDA:0"`, with **no remote
service and no container** — simpler and faster than the retired Windows WSL path. The `[slam]
backend=remote` SlamService remains available as a config-only fallback (`make_slam_worker` honors it), but
is not used here.

## 1. Goal & scope

Bring the desktop panel's **SLAM mode** (Phase 6 — live pose + reconstructed mesh + trajectory,
first-person follow camera, HUD) to the web app, as a top-bar **Real-Time ↔ SLAM** mode switch. In SLAM
mode the viewport shows the accumulating **TSDF mesh** (walls / floor-ceiling shaded) with the sensor's
**trajectory** and a **follow camera**, instead of the per-frame point cloud.

**Reuse target (the whole point).** `slam/` already has the off-thread pipeline the desktop uses:
`make_slam_worker` (`slam/backend.py:12`) → `SlamWorker.submit(depth,quat,pressure,reflectance,confidence)`
/ `.latest() → (mesh, trajectory, FrameStep)` (`slam/worker.py:30`), and `MeshPrep` (`slam/meshprep.py:118`)
→ a plain-numpy `MeshPacket` (wall/non-wall verts+colors+tris + floor grid). Web Phase 4 wires these into
the existing reader/broadcast plumbing and adds a wire format + a Three.js renderer. It edits **nothing**
in `slam/`.

Non-goals (deferred, unchanged from ROADMAP): **Showcase** (Web Phase 5 — `slam/showcase.py`), settings
persistence + retiring `panel.py` (Web Phase 6). No loop closure / relocalization beyond what `Mapper`
already does. (Mesh **save/export** IS in scope this phase — §8, owner decision.)

## 2. Compute: local GPU (`CUDA:0`), in-process

Per the discovery box above, the SLAM worker runs **in-process on the local RTX 2000 Ada** via
`make_slam_worker(width, height, cfg)` with `device` resolved by `slam.config.preferred_device()` (returns
`"CUDA:0"` when Open3D reports CUDA, which it does here). No remote service, no IPC. Design consequences:

- **Fast enough for live** — 7.2 ms/frame measured, well under the 28 Hz (~36 ms) sensor budget. The
  worker's latest-wins input slot still decouples mapper time from the reader, so even a transient slow
  frame never backs up the broadcaster; nothing waits on the mapper.
- **Deterministic verification via replay** — SLAM over `captures/verify_slam.bin` (recorded live this
  session: RAW + IMU_QUAT + ENV, 329 frames, lost=0, ~30k-vertex map) is the repeatable end-to-end check,
  and composes for free with Web Phase 3's `SessionController` (load capture → switch to SLAM → watch the
  map build). Live-device SLAM is the same path pointed at the live source.
- **Verification-data note** — the older `recordings/2026-07-08-room-scan.bin` predates IMU streaming (no
  stream 9), so the mapper gets no rotation prior and loses tracking → empty map. Always verify SLAM with a
  stream-9 capture; `verify_slam.bin` is that fixture.
- Backend stays `slam/config.py`'s job; a reachable `[slam] backend=remote` service would be used with no
  code change (web only ever touches the `make_slam_worker` interface), but it is not needed here.

## 3. Server plumbing — a SLAM stage beside the point-cloud stage

The reader already produces, per frame, everything the mapper needs: `depth`, the fused `quat` (Phase 2's
`SensorState`), and `reflectance`/`confidence` from the transform outputs. Phase 4 adds a **`SlamRunner`**
owned alongside the `SessionController`, fed from the same reader tick:

- **Feed.** A small hook in the broadcast/reader path calls `slam_runner.submit(depth, quat, pressure,
  reflectance, confidence)` **only while mode == "slam"** (no CPU burned in real-time mode). The worker's
  `submit` is the existing latest-wins drop — cheap, non-blocking, never on the event loop.
- **Drain + mesh.** A dedicated worker thread (the `SlamWorker`'s own `start()`) runs `Mapper.step`. A
  broadcaster-side poll takes `worker.latest()`; on a new `mesh_seq` it hands the tensor mesh to a
  **`MeshPrep`** instance (reused verbatim, off-thread, adaptive decimation) and reads back the
  `MeshPacket`. Trajectory + `FrameStep` (pose/fitness/rmse/tracking_lost) publish every frame regardless
  of the mesh throttle (`_MESH_EVERY = 5`), so the HUD stays live even while tracking-lost.
- **Lifecycle.** Entering SLAM mode constructs the worker (`make_slam_worker(width, height)`) + `MeshPrep`
  and `start()`s both; leaving SLAM stops them and frees the map. A source-swap (Phase 3 load_capture /
  go_live) resets the mapper (fresh map for a fresh source). All start/stop/reset runs off the event loop
  via `asyncio.to_thread`, serialized with the SessionController lock (SLAM and source-swap must not race).

`width`/`height` come from the transform output resolution the point-cloud path already knows.

## 4. Wire additions (all on the existing `/ws`, per `docs/web-protocol.md`)

### New binary tag — `MESH` (tag 3)

The `MeshPacket` is plain little-endian arrays; the placeholder reserved in Web Phase 1 is now `TAG_MESH = 3`.
A single self-describing frame (counts up front so the client allocates once):

```
u32 tag=3
u32 mesh_seq
u32 flags                 # bit0 decimated, bit1 walls_split
u32 n_nonwall_verts  u32 n_nonwall_tris
u32 n_wall_verts     u32 n_wall_tris
u32 n_floor_pts      u32 n_floor_lines
f32[3*n_nonwall_verts] nonwall_pos   f32[3*n_nonwall_verts] nonwall_col   u32[3*n_nonwall_tris] nonwall_idx
f32[3*n_wall_verts]    wall_pos       f32[3*n_wall_verts]    wall_col       u32[3*n_wall_tris]    wall_idx
f32[3*n_floor_pts]     floor_pos      u32[2*n_floor_lines]   floor_idx
```

Positions cast `f64→f32` (metres, Open3D CV world); colors `f64→f32` in [0,1]. Emitted on the mesh
cadence only (throttled), so bandwidth is a few hundred KB every ~5 integrated frames, not per frame.
A `pack_mesh(packet) -> bytes` builder in `web.py` mirrors `pack_point_cloud`/`pack_ir_image`.

### New JSON message — `slam` (server → browser)

Every processed frame (cheap, no mesh), on the broadcast cadence:

```
{type:"slam", mode:"slam",
 pose:[16 f32 row-major world<-camera],        # FrameStep.pose, Open3D CV convention
 traj_tail:[[x,y,z], …],                        # last ~256 trajectory positions (downsampled), for the line
 traj_len:int,                                  # full trajectory length (HUD)
 fitness:float, rmse:float, tracking_lost:bool, slam_ms:float,
 frames_integrated:int, mesh_seq:int}
```

Follow-camera math (eye/center from pose, matching `panel.follow_camera_target`) is computed **server-side**
and shipped as part of `pose` handling, per the web-protocol invariant "server-side math stays server-side"
— the browser positions its camera from the pose, it does not re-derive the follow transform's constants.
`traj_tail` is downsampled server-side to a bounded length so the JSON stays small on a long scan.

### New inbound — `set_mode`

```
{type:"set_mode", mode:"realtime"|"slam"}
{type:"slam_opt", trajectory?:bool, walls?:"solid"|"split", follow?:bool}   # display toggles -> state echo
{type:"save"}                                                              # write full-res map -> results/ (§8)
```

`set_mode` is routed in `_handle_inbound` (`web.py:942`); it flips `UiState.mode`, starts/stops the
`SlamRunner` off-loop, and echoes the authoritative mode in the `state` message (one-way flow — the client
never optimistically switches). `slam_opt` toggles piggyback on `set_ir`-style `state` echoes. `save` is
§8. A server → browser **`saved`** JSON (`{type:"saved", items:[{name,bytes,mtime}]}`, newest first — same
shape as `captures`) lists `results/*.ply`; broadcast on connect and after a save completes.

## 5. Frontend — new `slam.js` module (the 9th)

One vanilla ES module, constructed in `app.js` like the others, talking only through the hub. **Unlike
`sensors.js`/`capture.js` (2D canvas), `slam.js` renders 3D** — but it reuses the **existing `scene.js`
Three.js context and camera**, swapping what's in the scene by mode (no second WebGL context; keeps the
headless SwiftShader box cheap):

- **Mesh.** `MESH` binary → two `THREE.BufferGeometry` meshes (non-wall, wall) with vertex colors +
  `MeshStandardMaterial`, plus a `LineSegments` floor grid. Rebuilt on each `mesh_seq`; geometry disposed
  on replace. Wall mesh hidden when "walls solid" is off, matching the desktop.
- **Trajectory.** `traj_tail` → a `Line` (green ribbon, desktop parity); a glowing marker at the current
  pose head.
- **Follow camera.** When follow is on, the camera eye/center track `pose` each `slam` message; when off,
  the user orbits freely (OrbitControls, as in real-time mode).
- **Mode switch.** Top-bar segmented Real-Time / SLAM (`set_mode`); in real-time mode the point-cloud path
  renders as today and the SLAM group is hidden. Driven entirely from the server `state`/`slam` echo.
- **SLAM HUD.** Extends the metrics HUD with a SLAM row: tracking OK/LOST, fitness, RMSE, frames
  integrated, SLAM ms, mesh vertex count.
- **Save.** A Save button (enabled in SLAM mode once `frames_integrated > 0`) sends `{type:"save"}`; a
  saved-maps list (the `saved` message) shows `results/*.ply` with a download link per row (served from a
  `/results/<name>` static mount, basename-sanitized).

## 6. Save / export (owner decision)

Reuses the desktop Showcase FINAL-save shape (`panel._RESULTS_DIR = "results"`): on `{type:"save"}` the
`SlamRunner`, **off the event loop** (`asyncio.to_thread`), pulls the **full-resolution** `mapper.mesh()`
(never the decimated live `MeshPacket`) and the full `trajectory`, and writes:

- `results/web_<ts>.ply` — `o3d.io.write_triangle_mesh` of the full-res tensor mesh (`.cpu().to_legacy()`).
- `results/web_<ts>.tum` — trajectory in TUM format (reuse `slam.metrics`' TUM writer; timestamps synthetic
  monotonic, matching the CLI's `--out-traj`).

`<ts>` is passed in from the request-handling coroutine (scripts can't call `Date.now()`, but `web.py` can
use real wall-clock). On completion the server broadcasts a fresh `saved` list. Errors (empty map, disk)
publish an `error`-classified bus line → toast, no exception escapes. A `/results/<name>` GET (basename +
`.ply`/`.tum` allow-list, must-exist — same `sanitize` discipline as captures) lets the browser download the
artifact. Save is **disabled in real-time mode and on an empty map** (no silent no-op — the button is
greyed with a reason, per the one-way-echo state).

## 7. Testing

Pure/unit (no socket): `pack_mesh` round-trips a synthetic `MeshPacket` (counts + arrays byte-exact);
`slam` message shape (pose 16-float, traj downsample bound, tracking flags); `set_mode` gating (no worker
constructed until mode=slam); `SlamRunner` lifecycle (start/stop/reset, latest-wins submit).

Integration (real uvicorn + `websockets`, extends the Phase-1/3 harness): drive `set_mode → slam` over a
**recorded capture** and assert MESH frames + `slam` messages arrive and `frames_integrated` climbs;
`load_capture` mid-SLAM resets the map; `go_live`/`set_mode → realtime` tears the worker down.

End-to-end: headless Chrome (SwiftShader) per `docs/web-ui-testing.md` against a recorded capture — switch
to SLAM, watch mesh + trajectory build, toggle follow/walls, confirm the SLAM HUD, switch back. Target:
full host suite green (≥ 625 + new).

## 8. Risks

- **CPU SLAM is slow on the headless box** — mitigated by latest-wins frame dropping (mapper never backs
  up the reader) and by making **replay the primary validation path** (deterministic, fps-independent).
  Live-device SLAM is fps-bound; documented, not blocked.
- **Mesh bandwidth on a large map** — mitigated by `MeshPrep`'s adaptive quadric decimation (display-only,
  the full-res mesh is never sent) and the `_MESH_EVERY` throttle. A hard vertex cap in `pack_mesh` guards
  a pathological map; log when it clips (web-protocol "no silent caps").
- **SLAM ↔ source-swap races** (two tabs, or load_capture mid-SLAM) — serialized under the
  SessionController lock; mode/worker transitions are idempotent.
- **Reusing `scene.js`'s single context across modes** — geometry must be disposed on mode switch and on
  each mesh rebuild to avoid a GPU-memory leak on the software renderer (checklist item for review).
- **Open3D tensor teardown** — the worker's Open3D handles must be released on stop; mirror
  `slam.worker.SlamWorker.stop()`'s bounded join, don't leave a daemon holding a VoxelBlockGrid.
```
