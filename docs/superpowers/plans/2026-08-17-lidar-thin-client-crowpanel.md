# LiDAR thin-client render — implementation plan

**Status:** Ready to implement
**Date:** 2026-08-17
**Spec:** [`docs/superpowers/specs/2026-08-17-thin-client-render-design.md`](../specs/2026-08-17-thin-client-render-design.md) (approved)
**Companion:** `CrowPanelProp` repo, `docs/superpowers/specs/2026-08-17-lidar-thin-client-crowpanel-design.md`.
The **Protocol contract** section of the spec is duplicated verbatim in both repos — any change
to `THIN_FRAME` / `thin_*` messages made during implementation must be mirrored there.
**Governing issue:** [#194](https://github.com/hellosamblack/lidar-roomscanner/issues/194) —
*Thin-client render feed: /ws-thin, server-side raster for the CrowPanel* (`work-item`, `area/host-web`).

## Summary of the work

Add a server-rendered raster feed for a GPU-less embedded client: a new
`host/src/roomscan/thin_render.py` module (Open3D `OffscreenRenderer`, per-client orbit camera,
RGB565 conversion), a new `/ws-thin` WebSocket endpoint in `web.py` with its own paced broadcast
task, a curated 2 Hz `thin_telemetry` projection, `thin_orbit`/`thin_mode`/`thin_record` inbound
commands, and first-ever mDNS advertisement of the web server (`_roomscan._tcp.local.`).
The existing `/ws`/`/ws-mesh` protocol is untouched.

## Step 0 — De-risk spike: headless Open3D offscreen rendering (do first, gate everything on it)

The dev/runtime host is a GPU-less Proxmox LXC on llvmpipe. The spec chose Open3D
`OffscreenRenderer`, but nothing in this repo has ever created one here — SLAM meshing uses
Open3D's tensor pipeline, not its rendering stack.

- Throwaway script (not committed, results are): create an `OffscreenRenderer(480, 480)`,
  add a ~2 300-point cloud (real frame via `rig_playback`/`capture.py` decode of an existing
  recording), render, save PNG. Record: does context creation succeed (EGL vs OSMesa), and
  wall time per render at 480×480.
- Measure the render call's **GIL-hold**, not just wall time — Open3D C++ does not release the
  GIL (hard-won rule; BUG-063 methodology: watchdog-thread tick, judged by `tick_share`).
  A 10 fps render loop that holds the GIL 30 ms/tick is a 30% tax on *every* asyncio client.
- **Gate:** context creation works AND render ≤ ~40 ms. If either fails, fall back to a
  **numpy software projector** — at 54×42 (≈2.3 k points) a hand-rolled
  project-points-and-splat into a 480×480 buffer is trivial and GIL-friendly; the module API
  below is renderer-agnostic on purpose so only the backend swaps. Record the decision and
  numbers in this plan before proceeding.

### Step 0 result (2026-08-17) — **GATE PASSED, Open3D backend chosen**

Measured on this host (GPU-less Proxmox LXC, llvmpipe, `host/.venv`, Open3D 0.19.0):

| Measurement | Result |
| --- | --- |
| Context creation | **OK** — `[Open3D INFO] EGL headless mode enabled.`, EGL 1.5 / OpenGL 4.5, ~4.0–4.5 s one-time |
| Render 480×480, 2 300 points | mean **17.2 ms**, p50 16.7, p95 18.5, max 26.1 — well under the ~40 ms gate |
| Render + full geometry churn + camera move + RGB565 | **27.2 ms/tick** |
| Sustained 10 fps loop | **60/60 frames in 6.0 s** |
| GIL tax on the event loop (`tick_share`, BUG-063 method) | baseline 1.000 → **1.005 during render** (≈0%, within noise) |

Filament renders on its own threads and releases the GIL, so the feared 30%-tax scenario does
not materialise. **No numpy software-projector fallback is needed.** Visual confirmation: a
real 480×480 PNG with 23 599 distinct colours, not a blank buffer.

**Two hard Filament constraints the spike found — both abort the process (`utils::PreconditionPanic`,
`terminate called`), they do not raise a catchable exception, so the code must make them
unreachable rather than handle them:**

1. **One `OffscreenRenderer` per process.** Constructing a second one aborts immediately.
   → `ThinRenderer` is a hard process-wide singleton; the client cap shares one renderer.
2. **Every Filament call must run on the thread that created the renderer**
   (`JobSystem::getState(): This thread has not been adopted.`). Creating on the main thread and
   rendering from a worker aborts.
   → `ThinRenderer` owns a **dedicated render thread** that both creates the renderer and services
   every render job from a queue. Verified working: init + churn + render, all on one worker
   thread, 20/20 frames.

**Architecture consequence (amends steps 2–3):** the per-connection render task does *not* call
Open3D directly. Each connection owns only its `ThinCamera` + mode, and submits a job to the
single render thread, awaiting the result off the event loop. Renders are serialised across
clients — at the cap of 2 clients × 10 fps that is 20 renders/s ≈ 54 % of the render thread,
which fits. If a future client count needs more, the cap is the knob, not a second renderer.

## Step 1 — Pure functions: RGB565 conversion + `THIN_FRAME` packer

New `host/src/roomscan/thin_render.py`, bottom-up:

- `rgba_to_rgb565(img: np.ndarray) -> bytes` — vectorized (no per-pixel Python), little-endian,
  row-major.
- `pack_thin_frame(pixels_rgb565: bytes, width: int = 480, height: int = 480) -> bytes` —
  `u32 tag=1, u16 w, u16 h` header per the contract, all little-endian.
- Tests in `host/tests/test_thin_render.py`: exact header byte layout against hand-computed
  bytes (golden-vector style, like `test_protocol.py`), known-color RGB565 conversions
  (pure red/green/blue/white/black — catches bit-order and endianness), payload length =
  `w*h*2`. **Assert values, not types** (the BUG-050 lesson).

## Step 2 — `ThinRenderer` + per-connection state

Still in `thin_render.py`:

- `ThinCamera` dataclass: `yaw`, `pitch` (clamped ±89°), `zoom` (clamped to a sane range);
  `apply_orbit(dyaw, dpitch, dzoom)` accumulates deltas. Mode field:
  `"point_cloud" | "slam" | "ir"`, default `point_cloud`.
- `ThinRenderer`: lazy singleton owning the offscreen context. `render(source, camera) ->
  np.ndarray (RGBA)`. Init failure raises `ThinRenderUnavailable` once and caches the failure —
  `/ws-thin` turns it into the JSON-error-and-close path; `roomscan.web` never crashes on it.
- **Data taps — reuse, never recompute:**
  - *point_cloud*: the point-cloud broadcaster already computes `pts`/`colors` right before
    `pack_point_cloud` (`web.py:1085` callers, loop at `web.py:5593`). Stash a reference
    (`state.thin_latest_pc = (generation, pts, colors)`) at that spot — one assignment, no copy.
  - *slam*: `_cache_latest_mesh` (`web.py:5021`) retains packed MESH bytes (#186). Extend it to
    also retain the pre-pack Open3D geometry (or unpack once on demand in the thin loop —
    decide by measuring; unpacking 150 k-vertex MESH bytes per mode-switch is fine, per-tick
    is not).
  - *ir*: stash the latest reflectance frame beside where `pack_ir_image` is fed; colorize via
    the existing `ir_image.reflectance_to_rgb` + `ir_range`.
  - Every stash is **generation-tagged** and the thin loop drops mismatches — the #101
    source-generation barrier applies to this consumer like every other per-frame consumer
    (memory: "View source-generation barrier"). A live→replay switch must never show a stale
    thin frame.

## Step 3 — `/ws-thin` endpoint + broadcast task in `web.py`

- New `@app.websocket("/ws-thin")` beside `/ws` (`web.py:5909`) / `/ws-mesh` (`web.py:5973`).
  On accept: if `ThinRenderer` is unavailable, send `{"type":"error", ...}` and close cleanly.
- **Per-connection render task**, mirroring the point-cloud broadcaster's deadline pacing
  (`POINT_INTERVAL` pattern, `web.py:141` + resync-don't-burst at `web.py:5598`) with
  `THIN_INTERVAL = 1/10`. Per-connection camera + mode live in the task's scope, so disconnect
  teardown is automatic and multiple clients get independent orbits.
- **Backpressure = freshest-frame-wins, structurally.** `/ws` has none (BUG-061: `send_bytes`
  never blocks, queues unboundedly), so do not copy its send path blindly: skip the *render*
  (not just the send) while the previous `send_bytes` awaitable hasn't completed — a
  single-slot in-flight guard per connection. A slow client costs itself frames, never the
  render loop or other clients.
- **Cap concurrent thin clients** (constant, suggest 2) with a JSON error on excess.
  Connection count is a real performance variable on this server (BUG-060), and a thin
  client's per-connection cost is a full render, far above a deflate.
- Inbound handler: `thin_orbit` (clamp, accumulate), `thin_mode` (validate enum, ignore junk),
  `thin_record` — **factor the existing `record` branch (`web.py:6106`,
  `ctrl.start_record()/stop_record()` + `_broadcast_session`/`_broadcast_captures`) into a
  shared helper** used by both entry points; zero duplicated recording logic. Malformed JSON:
  log + ignore, never disconnect.
- `thin_telemetry` every 500 ms from the same task: a projection of the already-built
  `_ranging_message` (`web.py:1897`), `build_sensor_message` (`web.py:1492`),
  `build_metrics_message` (`web.py:1443`) fields — `fps`, `power_mode`, `i3c_airtime_pct`,
  `point_count`, plus authoritative `recording` (from `ctrl`) and this connection's `mode`.
  No new telemetry math.

## Step 4 — mDNS advertisement

- In `_lifespan` (`web.py:4354`): register `zeroconf.ServiceInfo` — type
  `_roomscan._tcp.local.`, instance `roomscan`, the actual bound port (8000 default) —
  and unregister on shutdown. `zeroconf` is already a dependency (`sources.py:13`).
- Inject a `zeroconf_factory` the way `sources.py:221` does, so tests exercise registration
  with a fake — the real startup path is otherwise untested (memory: "web startup path
  untested"), so this needs an explicit lifespan test, not an assumption.
- Verify live with the existing `host/tools/query_mdns.py` / `sniff_mdns.py`.

## Step 5 — Probe tool + MCP surface

- Logic as a pure function in the `roomscan` package: connect to `/ws-thin`, receive N frames,
  decode + save PNGs, round-trip `thin_orbit`/`thin_mode`/`thin_record`, return a dict
  (frames received, measured fps, telemetry seen, PNG paths).
- Per the MCP-first rule this **lands as an MCP tool** — `rig_thin_probe`, mirroring
  `rig_ws_probe` (`host/src/roomscan/mcp_server/`), docstring = agent-facing description —
  plus a thin CLI front end `host/tools/thin_client_probe.py`. One implementation, two fronts.
  (Alternative of CLI-only requires an `EXCLUDED` entry in `test_mcp_registry.py`; don't —
  agents will use this constantly to eyeball the feed.)
- Document in `docs/mcp-server.md`. Invariant holds: the tool talks to `roomscan-web` over
  `/ws-thin`; it never binds the device stream.

## Step 6 — Validation

- **Unit:** packer/RGB565 (step 1); orbit clamp + accumulation; telemetry projection built from
  hand-built state; the shared record helper (both entry points hit the same `ctrl` calls);
  generation-tag mismatch drops the stale stash; renderer-unavailable → error JSON + clean
  close. Integration tests use hand-built state + ASGI websocket client, `test_web.py` style
  (run from `host/`, cwd-relative fixtures).
- **Non-regression:** with a thin client connected and streaming, existing `/ws` point-cloud
  cadence is unchanged — judge by `tick_share` via `slam_stall_profile` methodology, never
  summed tick-lateness (BUG-063).
- **E2E on replay, no hardware:** `rig_up` against `--replay recordings/<capture>.bin`, run
  `rig_thin_probe`, inspect the saved PNGs (owner preference: show the visual result — attach
  frames for point_cloud, slam, and ir modes, plus a before/after orbit pair proving
  `thin_orbit` moved the camera). **Pixels are the observable** — a frame counter or a
  read-back of the camera state proves nothing (the #106 lesson); the orbit pair must be
  framed so a null result can't be explained away.
- **Close gate (operator-request table):** everything above runs on this host today against
  recorded captures, so the server-side issue can close on it. Interop with a *real* CrowPanel
  is the companion repo's acceptance, not a hold on this one — but say so explicitly in the
  closing comment and file the interop follow-up if the CrowPanel side isn't ready.

## Step 7 — Docs + landing

- `docs/thin-client.md`: the protocol contract (kept verbatim-identical to both specs), the
  renderer backend decision from step 0 with its numbers, and the client-cap/backpressure
  policy. Index it in `docs/README.md`. Note `/ws-thin`'s existence + pointer in
  `docs/web-protocol.md` without touching that protocol's own surface.
- `status-sync` at ship (docs + ROADMAP register row move in the same commit);
  `session-end` **before** the `Closes #NNN` commit (#169).

## Risks

| Risk | Mitigation |
| --- | --- |
| Open3D `OffscreenRenderer` can't get a headless context on llvmpipe, or is slow | Step 0 gates the whole plan; numpy software projector is the designed fallback (API keeps the backend swappable) |
| Render call holds the GIL and starves the event loop / other clients | Step 0 measures GIL-hold up front; render task only runs while a thin client is connected; drop cadence or switch backend if `tick_share` degrades — and **cost the mitigation against the failure** before adding complexity (BUG-052 lesson) |
| ~4.6 MB/s per client saturates the wireless uplink (FileHub / Pi bridge) | Freshest-frame-wins degrades gracefully to lower fps; deferred JPEG path (~5–10× smaller) is the real fix, already specced as future work |
| Stale frame shown across a live/replay source switch | Generation-tagged stashes, dropped on mismatch (#101 barrier), unit-tested |
| Contract drift vs the CrowPanel repo | Protocol section is duplicated verbatim by rule; any wire change during implementation updates both specs + `docs/thin-client.md` in the same change |

## Suggested implementation order & session shape

Steps 0→1→2 are one session (spike, then pure functions while the spike's numbers are fresh).
Step 3 is the big one and lands with step 6's unit/integration tests in the same commits.
Steps 4 and 5 are independent of each other after step 3 and could parallelize. Step 7 rides
the final commit via `status-sync`. Every session starts with `session-start` against the
governing issue.
