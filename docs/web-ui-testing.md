# Visually testing the web UI on the headless host

This box has **no display and no Chrome extension**, so the mcp browser tools,
VNC clicking, and any on-screen interaction are unavailable. To *see* and *drive*
the `roomscan-web` UI here, use **`host/tools/web_ui_shot.py`** — it launches
headless Chrome (software WebGL via SwiftShader), navigates the page over the
Chrome DevTools Protocol, runs JS to click/toggle/type, and captures PNGs you
then Read back. This is the standard way to verify web front-end work in this
repo (established 2026-07-16, Web Phase 1).

## The recipe

### 1. Start the server against a replay — DETACHED, sandbox off

The Bash sandbox kills network-listener processes (uvicorn exits 144), so the
server **must** run with `dangerouslyDisableSandbox`, and it must be *detached*
(`setsid … &`, stdin from `/dev/null`) so it survives after the launching Bash
call returns. Use `ROOMSCAN_NO_BROWSER=1` (nothing to open here):

```bash
ROOMSCAN_NO_BROWSER=1 setsid host/.venv/bin/python -m roomscan.web \
    --replay <capture.bin> --replay-fps 20 > /tmp/web.log 2>&1 < /dev/null &
sleep 4
curl -s -o /dev/null -w "%{http_code}\n" http://localhost:8000/static/index.html   # expect 200
```

Bash gotchas that cost time here:
- Run the launch in its **own** Bash call. The shell snapshot runs `set -e`, so a
  leading `pkill -f roomscan.web` that matches nothing exits 1 and aborts the rest
  of a compound command — kill in a separate call.
- Verify liveness with a *separate* `curl` call; don't chain it after the launch.

### 2. Screenshot + drive with `web_ui_shot.py` (sandbox off)

```bash
# default-load shot:
host/.venv/bin/python host/tools/web_ui_shot.py --out /tmp/01-load.png

# drive interactions — one PNG per step; `js` runs in the page, click by element id:
host/.venv/bin/python host/tools/web_ui_shot.py --out /tmp/01-load.png --steps '[
  {"js":"document.getElementById(\"btn-ping\").click()","wait":1.2,"out":"/tmp/02-toast.png"},
  {"js":"document.querySelector(\"#seg-color button[data-mode=depth]\").click()","wait":2,"out":"/tmp/03-depth.png"}
]'
```

Then **Read** each PNG to inspect it. The tool also prints the on-page `#diag-log`
tail — the fastest signal for a load failure (WS never opened, module 404, WebGL
context refused). It manages its own Chrome (launch + teardown); pass `--port` to
reuse an already-running remote-debugging instance instead.

Each step is `{"js": <expr, awaited>, "wait": <seconds>, "out": <png path>}`.
Because control is just JS in the page, you click real bindings (`element.click()`),
so this exercises `controls.js` → `ws.send` → server, not a synthetic shortcut.

**`wait` does not guarantee the replay advanced.** Stepping with `wait` seconds apart
to sample a capture at different points can silently return the *same* frame three
times — the replay may have already ended (it holds the last frame) or be paused,
and nothing in the step result says so (seen 2026-07-29: three shots meant to span a
roll sweep all read an identical `rotate(4.88deg)`). Assert the state you are relying
on inside the `js` — read `#seek`/the transport position, or log the value you are
sampling — rather than inferring progress from elapsed time. A per-step `__diag` line
echoing the quantity under test makes a stalled replay obvious instead of producing
three confidently identical "measurements".
Useful element ids live in `host/src/roomscan/static/index.html` (e.g. `btn-ping`,
`seg-color button[data-mode=…]`, `chk-ir-freeze`, `log-toggle`, the Web-Phase-3
capture controls `btn-record`, `btn-refresh-caps`, `#cap-list .cap-row`,
`btn-playpause`, `seg-speed button[data-fps=…]`, `chk-loop`, `seek`, and the
Web-Phase-4 SLAM controls `#seg-mode button[data-mode=realtime|slam]`,
`chk-slam-traj`, `chk-slam-follow`, `#seg-walls button[data-walls=split|solid]`,
`btn-save`, `#saved-list .cap-row a`, and the diagnostics toggle `diag-toggle`).

**The magnetometer-calibration modal has two renderers, and both must be shot.**
Open it with `sensor-mag-cal`; drive it with `magcal-start` / `magcal-stop` /
`magcal-clear`. The 3D "Shell & Steering" view (`magcal3d.js`) publishes a 1 Hz
diag line and `window.__magcal3d = {renderer, frames, cells, covered, lastPoseMs,
poseHz, reason}` — assert `renderer=="webgl"` and `poseHz≈30` rather than merely
that a canvas exists. **`?magcal2d=1` forces the 2D Lambert fallback**, so one run
covers both paths; `window.__magcal3d.renderer` reads `"2d"` and
`#magcal-fallback-note` says why. Context loss is reachable from a step:
`document.getElementById('magcal-hero').getContext('webgl2')
.getExtension('WEBGL_lose_context').loseContext()` — it must degrade to the 2D map,
not hang. **Reading the coverage number is the honesty check**: with the board
sitting still on the desk the shell must be almost entirely dashed hollow rings and
the gauge must say so (measured on-rig 2026-07-29: 2 / 92 cells after 600 samples,
plus a `STATIONARY` chip). If it looks flattering while nothing moves, the view is
lying. No capture in `captures/` contains a full tumble — `tilt_sweep_20260729.bin`
is a 1-D tilt and bins into 2 cells — so a *covered* shell shot needs a synthetic
fixture (rewrite a capture's stream-10 mag payloads along a continuous spherical
spiral; `roomscan.protocol.pack_frame` re-encodes the frames unchanged).

**SLAM verification needs a stream-9 capture.** SLAM builds nothing from a capture
with no IMU_QUAT (stream 9) — the mapper gets no rotation prior and loses tracking
(`recordings/2026-07-08-room-scan.bin` predates IMU → empty map). Use
`captures/verify_slam.bin` (has 9/10) or record a fresh one with
**`host/tools/capture.py --udp --seconds 12 --out captures/verify_slam.bin`** (the
headless host has no USB, so `--udp` grabs the Ethernet stream — streams 9/10 in the
decode-report confirm it's SLAM-capable). To
build the map, launch with `--replay <stream9.bin> --replay-fps 30`, click
`#seg-mode button[data-mode=slam]`, and enable Loop (`chk-loop`) so frames keep
feeding — SLAM is fed from the 30 Hz broadcaster only while in SLAM mode, so ~330
frames take ~11 s to integrate. `window.__gotMesh` and the diag line
`slam.js: first mesh: N non-wall verts` confirm the mesh path (the *first* emit is
an empty packet — N=0 — by design; later ones carry geometry).

## The dock layout (why nothing overlaps)

Every floating block lives in one of two **docks** — `#left-dock` (telemetry HUD,
sensors, SLAM HUD, IR monitor, diagnostics) and `#right-rail` (one card per
control group). A dock is a height-bounded **column-wrapping** flex container
spanning the band between the top bar and the event-log console, so a stack that
would run past the bottom of that band spills into a **new column** instead of
overflowing onto whatever is below. `layout.js` (a *classic* script, so it keeps
working when the ES module graph fails) keeps the band in sync with the measured
top-bar/console heights and resolves the one collision CSS can't see — the left
dock's columns marching into the right dock's — by degrading in order:

1. collapse Diagnostics → 2. scroll the right dock (one column) →
3. collapse the IR monitor body → 4. collapse the sensors body.

Consequences when adding UI: **put a new block inside a dock** (as a `.card`
child) rather than giving it its own `position: fixed` corner — a fixed block is
outside the layout and *will* overlap something at some viewport size. Give it an
explicit `width`, and mark any hideable body `.card-body` so the collapse
degradation can reach it.

Regression check — assert zero overlaps at whatever size you're testing (paste as
a `--steps` `js`; it reports through `__diag`, which the tool prints):

```js
(function(){function c(r,d){return {left:Math.max(r.left,d.left),right:Math.min(r.right,d.right),
top:Math.max(r.top,d.top),bottom:Math.min(r.bottom,d.bottom)}}var b=[];
['left-dock','right-rail'].forEach(function(id){var e=document.getElementById(id);if(!e)return;
var dr=e.getBoundingClientRect();[].slice.call(e.children).forEach(function(k){var r=c(k.getBoundingClientRect(),dr);
if(r.right-r.left>1&&r.bottom-r.top>1)b.push({n:k.id||k.className,r:r})})});
['topbar','log-console'].forEach(function(id){var e=document.getElementById(id);if(e)b.push({n:id,r:e.getBoundingClientRect()})});
var bad=[];for(var i=0;i<b.length;i++)for(var j=i+1;j<b.length;j++){var a=b[i].r,d=b[j].r;
if(Math.min(a.right,d.right)-Math.max(a.left,d.left)>1&&Math.min(a.bottom,d.bottom)-Math.max(a.top,d.top)>1)
bad.push(b[i].n+' X '+b[j].n)}window.__diag('OVERLAP '+innerWidth+'x'+innerHeight+' n='+b.length+
' overlaps='+bad.length+(bad.length?' :: '+bad.join(' | '):''));return bad.length})()
```

Verified 0 overlaps at 1600×1000, 1280×800, 1100×560 and 820×700, with the SLAM
cards shown and the event log expanded to its 28vh maximum.

## The diagnostics panel

`window.__diag`'s on-page sink is now the collapsible `#diag-card` (header
`#diag-toggle`, text sink still `#diag-log`, which `web_ui_shot.py` prints). It is
**collapsed by default**, shows a line count — or a red error count — in its
header, and **opens itself on the first error** so a silent module/import failure
is still visible without devtools. Clicking the header persists the choice in
`localStorage['roomscan.diag.collapsed']`, and an explicit choice wins over the
auto-open. If you're driving a run where you want the log visible regardless:
`document.getElementById('diag-card').classList.remove('collapsed')`.

Driving gotchas (cost time in Web Phase 3):
- **Wait for server-rendered lists before clicking them.** Rows built from a
  `captures`/`session` message (the capture library, any server-driven list) don't
  exist until that message arrives (~0.5–1.5 s after `list_captures`/connect). A step
  that does `[...cap-row].find(r=>r.dataset.name===X).click()` too soon calls `.click()`
  on `undefined` and the step throws — the action never fires and you debug a phantom.
  Give the prior step ≥1.5 s `wait`, or first emit the rendered rows via
  `window.__diag(...)` and confirm the target is present.
- **Don't interleave exploratory clicks across `web_ui_shot.py` runs.** Each run is a
  fresh browser but the **server state persists** (current source, pacer paused/loop).
  Ad-hoc clicking across runs leaves the server in a confusing mid-state that reads like
  a bug; drive a **clean, disciplined step sequence in one run**, and restart the server
  (`pkill -9 -f roomscan.web`) before a fresh scenario.
- **Closures hide module state.** To inspect what a module actually received (e.g. the
  last `session`), temporarily stash it on `window` inside the hub handler and read it via
  `__diag`; remove the hook before committing.

## Picking a replay capture

- **Depth-only view** (point cloud, metrics, commands): any capture works,
  including the small golden fixtures under `host/tests/fixtures/`.
- **IR pane / reflectance colour**: needs frames that carry a reflectance plane,
  i.e. RAW_3DMD + CALIB run through the transform. **Dual-stream recordings**
  (RAW_3DMD + a redundant DEPTH_ZF32 passthrough of the same seq, e.g.
  `recordings/2026-07-08-room-scan.bin`) intermittently fall IR/reflectance back
  to depth, because the DEPTH frame lands *last* in the latest-wins slot. Filter
  to RAW+CALIB first so the IR pane is exercised:

  ```python
  from roomscan.sources import FileSource, pump
  from roomscan.decoder import StreamDecoder
  from roomscan.protocol import pack_frame, StreamId, FrameType
  src, dec, out = FileSource("recordings/2026-07-08-room-scan.bin"), StreamDecoder(), bytearray()
  for f in pump(src, dec):
      if f.header.frame_type == FrameType.DATA and f.header.stream_id == StreamId.DEPTH_ZF32:
          continue                       # drop the redundant depth passthrough
      out += pack_frame(f.header, f.payload)
  open("/tmp/rawonly.bin", "wb").write(bytes(out))
  ```

  Live production streams are RAW-only, so this quirk is replay-data only.

## Verifying persisted settings (survives a restart)

A durable setting (Web Phase 5: color / IR colormap+freeze / SLAM toggles, in
`roomscan.toml` [viewer]`) is only proven if it survives a **server restart**,
not just a reconnect. A screenshot can't show that — the reusable pattern is two
real servers against the same config, checking the *first* `state` message:

1. Point `roomscan.config.config_path` at a temp file (so you don't stomp the
   real `~/roomscan/roomscan.toml`), and seed the app the way `main()` does:
   `cfg = ViewerConfig.load(); app.state.config = cfg; app.state.ui_state = web.ui_from_config(cfg)`.
2. Boot server A on a free port, connect a `websockets` client, send e.g.
   `{"type":"set_color","mode":"confidence"}`, await the echoed `state`.
3. Assert the temp `roomscan.toml` now has `color = "confidence"` (the
   `_persist_ui` write). Stop server A (`server.should_exit = True`).
4. Boot server B (fresh `app.state` seeded from the same file) and assert the
   **first** `state` message a new client receives already carries
   `color_mode == "confidence"` — that's the connect-time `_state_message`
   seeded from the file, i.e. a real restart survived.

A worked runner (paused reader, hermetic DEPTH capture, two servers) is the
Phase-5 verify script — grep the Web-Phase-5 commit (`db4fc65`) message for the
shape, or reuse `tests/test_web.py::_make_depth_capture` + `_free_port`. Key
gotcha: `_persist_ui`/`ViewerConfig.load()` resolve the path through
`config.config_path()` at call time, so monkeypatch that (not an already-imported
name) to redirect the write.

## Teardown

`pkill -9 -f roomscan.web` (and `-f remote-debugging-port` if you reused a Chrome).
Put temp replays/PNGs in the session scratchpad, not the repo.

See also `docs/headless-host-setup.md` (host bring-up) and
`host/tools/headless_doctor.py` (checks WebGL-capable browser is installed).
