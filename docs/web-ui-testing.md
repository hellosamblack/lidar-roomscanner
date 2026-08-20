# Visually testing the web UI on the headless host

This box has **no display**, so VNC clicking and any on-screen interaction are
unavailable. Web front-end work is verified by driving headless Chrome (software
WebGL via SwiftShader), screenshotting, and reading the PNGs back.

## Preferred: the `ui_*` MCP tools (2026-07-29)

`roomscan-mcp` (`docs/mcp-server.md`) holds **one browser open across calls**, so
only the first call pays the launch and settle. It supersedes the `web_ui_shot.py`
recipe below for day-to-day work:

```
rig_up(replay="captures/tilt_sweep_20260729.bin", replay_fps=20)
ui_screenshot(renavigate=True, settle=8)     # returns the PNG inline + #log-lines tail
ui_wait_for("parseInt(document.getElementById('pos-status').textContent.match(/\\d+/)[0]) > 50")
ui_eval("document.getElementById('hud-device-fps').textContent")
ui_reset()                                   # between independent scenarios
rig_down()
```

Why prefer it: the screenshot comes back as an image block (no `/tmp` file, no
separate Read), JS goes in as a plain string (no JSON-inside-shell-argument
quoting), and `ui_wait_for` waits on a **real condition** instead of a fixed sleep —
which this doc has always warned is the trap. Backed by Playwright
(`channel="chrome"`), with the raw-CDP path as fallback.

**From a git worktree these tools verify the WRONG CHECKOUT** (2026-08-11, #103).
The MCP server is launched from the session's original cwd (`.mcp.json`), so
`rig_up` serves the **main** checkout's `static/` and `run_tests` cannot see a test
you just added in the worktree (it reports `file or directory not found`). Front-end
work in a worktree can therefore be "verified" against code you did not edit. Start
your own server *from the worktree* and let the `ui_*` tools — which only drive a
browser at a URL — hit it:

```bash
# from <worktree>/, with dangerouslyDisableSandbox (a plain call exits 144)
ROOMSCAN_NO_BROWSER=1 \
ROOMSCAN_TRANSFORM_DLL=<main>/host/transform/build/libroomscan_transform.so \
PYTHONPATH=host/src:host \
  nohup setsid <main>/host/.venv/bin/python -m roomscan.web \
  --replay captures/<something>.bin --replay-fps 20 >/tmp/wt-web.log 2>&1 </dev/null &
```

A fresh worktree also needs `captures -> ../../../captures` and
`host/transform/build -> <main>/host/transform/build` symlinked in, or the library is
empty and 15 tests skip as "native transform DLL not built". Confirm which tree you
are actually serving with `ls -l /proc/<pid>/cwd` — **not** by hashing the served
file, which is identical to main's until your first edit. Run pytest directly:
`cd <worktree>/host && <main>/host/.venv/bin/python -m pytest`.

**`ui_screenshot`'s `width`/`height` now really do resize the viewport** (fixed
2026-08-12, #168, in two rounds). Both session paths — Playwright
(`page.set_viewport_size`) and the raw-CDP fallback
(`Emulation.setDeviceMetricsOverride`) — re-apply the requested size on *every*
call. There were two defects, and the second was **created by the fix for the
first**: an early return once the browser was already up made every call after
the first keep the launch size, so the three narrow sizes below "passed" three
times at 1600x1000; removing that early return then gave teeth to the argless
`self.start()` calls inside `goto`/`evaluate`/`wait_for`/`screenshot`, which
reasserted the 1600x1000 default microseconds *before* the pixels were captured.
`width`/`height` now default to `None`, meaning "keep the last explicitly
requested size", so a future argless caller cannot silently reintroduce this.

The first round was declared verified by driving `CdpSession.start()` directly
and reading back `innerWidth` — which is exactly why it could not see the
regression it had just introduced: `start()` is not the path `ui_screenshot`
takes to a pixel. **Assert through the tool, not through `session.py`.**
Confirmed 2026-08-12 by requesting 1280x800, 1100x560 and 820x700 through
`ui_screenshot` in a *fresh Python process* — the long-lived `roomscan-mcp`
pins the modules it booted with, so verifying a fix to the MCP layer *through*
the MCP layer just runs the old code — and reading back `innerWidth`/
`innerHeight` after each: three exact matches.

Still worth `ui_eval("(() => ({w: innerWidth, h: innerHeight}))()")` between shots
when a result surprises you — a check that cannot fail is what made this invisible
for a week. And to squeeze a sidebar card *without* a viewport change, expanding
the *other* cards in the same dock still works: the card under test drops to its
`min-height` floor, the same constraint a short viewport imposes, reached through
real in-app state.

**`renavigate=True` is a navigation, not a cache bypass** (2026-07-30). Chrome
happily re-served a just-edited `index.html` from its cache across two full
reloads, so a shipped copy change read as "not landed" until a `?cb=` query string
exposed it — a stale read that reports a *successful* load, and one you cannot see
in the pixels. Both browser backends (and `web_ui_shot.py`) now send
`Network.setCacheDisabled` at start, so this should not recur; if you ever suspect
it has, the one-line check is `ui_eval` on the text you just changed, and the
escape hatch is still a `?cb=<anything>` on the URL.

Assertable readouts: `#pos-status` ("frame N / total", replay only — it is empty on
a live server), `#hud-view-fps`, `#hud-device-fps`, `#record-status`, `#ir-frame`,
`#slam-frames`. **`window.__diag` is the page's logging sink *function*, not a state
object** — read what it logged from `ui_screenshot`'s event-log tail.

## Fallback recipe: `host/tools/web_ui_shot.py`

Still correct, and the only option in a client without MCP. It launches and tears
down its own Chrome per invocation.

### 1. Start the server against a replay — DETACHED, sandbox off

The server must be *detached* (`setsid … &`, stdin from `/dev/null`) so it survives
after the launching Bash call returns. Use `ROOMSCAN_NO_BROWSER=1` (nothing to open
here):

> **Stale claim, re-measure before relying on it (2026-07-29):** this doc used to
> say the Bash sandbox kills network listeners (uvicorn → exit 144) and that
> `dangerouslyDisableSandbox` was therefore mandatory. That did **not** reproduce:
> uvicorn bound `0.0.0.0`, served a request and exited 0 from a normal Bash call, as
> did plain listeners on loopback and `0.0.0.0`. It may have been true when written.
> `rig_up()` starts the server detached without needing the flag.

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

Then **Read** each PNG to inspect it. The tool also prints the on-page `#log-lines`
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
real-time view modes `#seg-view-mode button[data-viewmode=world|fpv|mirror]`
plus their camera framing `sl-cam-distance`/`sl-cam-height`/`sl-cam-rotation`
and `btn-cam-reset` (each edits the *selected* mode's framing) plus the
world-only auto-orbit `chk-orbit`/`sl-orbit-speed`,
the top-bar Record control `#topbar-record`/`btn-record` (Live page only; moved
out of its sidebar card by #118), the floating playback panel
`#playback-panel` (replaced the `#transport-card` sidebar card, #123 — fixed
position over the viewport bottom, replay only) with `btn-golive`,
`btn-playpause`, `btn-transport-restart`
(**not** `btn-restart`, which is the top bar's Restart Server — see BUG-047),
`seg-speed button[data-fps=…]`, `chk-loop`, `seek`, `pos-status`, the §12
View-page capture browser `#browser-card` with `#cap-grid .cap-tile[data-name]`,
its per-tile `input[data-check]`, `#seg-browser-sort button[data-sort=…]`,
`#seg-browser-view button[data-view=…]`, `chk-browser-thumbs`,
`btn-browser-refresh`, `btn-browser-delete` and the confirm modal
`#delete-modal` / `chk-delete-sidecars` / `btn-delete-confirm`, the
selected-capture action drawer `#browser-selected-detail` with
`btn-preview-load`/`btn-preview-rename`/`btn-preview-build` (**not**
`#preview-card`, the standalone card it replaced; and since #103 it sits *before*
`#cap-grid` in the card's fixed chrome — after the self-scrolling grid it was
clipped out of the card and those three buttons were unclickable),
and the
Web-Phase-4 SLAM controls `#seg-mode button[data-mode=realtime|slam]`,
`chk-slam-traj`, `chk-slam-follow`, `#seg-walls button[data-walls=split|solid]`,
`btn-save`, `#saved-list .cap-row a`, and the event-log toggle `log-toggle`
(diagnostics fold into that console now — the old `diag-card`/`diag-toggle` are gone)).

**The Sensors card's numerics live in a collapsed `<details>`.** Since the
2026-07-29 declutter, only the gizmo/compass, the selected orientation readout,
the fusion state and Environment are visible by default; the full-precision raw
ZYX values (`sensor-roll`/`…-quat`), the jitter table (`jitter-<signal>` for p95
and `jitter-<signal>-mean` for mean) and the yaw-offset controls sit inside
`#sensor-diag`. Set `document.getElementById('sensor-diag').open = true` in a
step before shooting or asserting them — they are in the DOM and updating either
way, but a screenshot won't show them. Opened, the drawer is its own scroll box
(the card is capped at the dock band), so `#sensor-diag.scrollTop = 1e4` brings
the jitter table into frame.

**Beware `const` leaking between steps.** Each step's `js` is evaluated in the
same page global scope, so a second step that re-declares `const s = …` throws
`Identifier 's' has already been declared` — and the step's *whole* body then
never runs. The tool prints `JS error in step …`, but the symptom reads exactly
like the app ignoring your click (cost ~20 min on 2026-07-29: three "the mode
select is broken" reproductions were all this). Wrap each step's body in an IIFE
(`(()=>{ … })()`) or use distinct names.

**The magnetometer-calibration modal has one 3D canvas and a 2D fallback, and
both must be shot.** (It had *two* WebGL canvases until 2026-07-31 — a body-fixed
"hero" and a small world-fixed "steering" widget. They are merged: one
`#magcal-hero` canvas, world-fixed framing with the full coverage styling, ghost
included. There is no `#magcal-steer`.) Open it with `sensor-mag-cal`; drive it
with `magcal-start` / `magcal-stop` / `magcal-clear`. `magcal3d.js` publishes a
1 Hz diag line and `window.__magcal3d = {renderer, framing, frames, cells,
covered, ghosted, lastPoseMs, poseHz, behindRecomputes, refreshMs, staticBehind,
reason}` — assert `renderer=="webgl"` and `poseHz≈30` rather than merely that a
canvas exists.

Two fields are new and worth asserting:

- **`framing`** is `"world"` (the merged view) or `"body"` (the no-orientation
  fallback, camera parked on the boresight). Forcing `body` is the only way to
  screenshot the degradation path on a rig that *has* an IMU: load a
  stream-9-less capture, or in a step set the modal's renderer handle's pose
  flags without `POSE_HAVE_QUAT` (bit 3). When it is `body`, `#magcal-hero-note`
  must be visible and saying why.
- **`behindRecomputes` / `refreshMs`** measure the per-frame near/far cell split
  (the merged camera is room-fixed, so which cells are *behind the eye* changes
  as the device turns; it used to be a constant). Attempted at most every 100 ms
  and skipped entirely while the view direction has moved < 1.5°.
  **`?magcalstatic=1` freezes it at the old body-fixed value**, which is how you
  get a same-build baseline — compare `frames` over a fixed wall-clock window
  with and without it.

  Measured 2026-07-31 (llvmpipe, live rig, modal open, 30 s windows):

  | arm | fps | recomputes/s | `refreshMs` |
  |---|---|---|---|
  | `?magcalstatic=1` (frozen) | 5.08 | 0 (1 total) | — |
  | default (per-frame) | 5.87 | 0 (2 total) | — |

  **A stationary rig does not exercise it at all** — that is what the 1.5° gate
  is for, and it is why the fps difference here is only run-to-run noise on this
  box (the "slower" arm measured faster). To get the cost under motion, build an
  isolated instance on a throwaway canvas and pump synthetic poses; `createMagcal3d`
  is exported and takes any canvas. At a **90 °/s tumble**: 2.5 recomputes/s
  (bounded by the render rate, since `updateBehind` runs from `frame()`) at
  **0.062 ms each** — 0.16 ms/s, and **0.062 % of wall clock even at the 10 Hz
  ceiling**. Ignore any `refreshMs` read after only one or two calls: the first
  call is cold and reads ~0.4 ms, 6× the warmed value.

**`?magcal2d=1` forces the 2D Lambert fallback**, so one run
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

**Record (and anything else gated on `has_live`) needs a live source, not `--replay`.**
Launching with `--replay` sets `has_live=False`, and `#btn-record` is disabled whenever
`!session.has_live` (`capture.js`) — the standard replay recipe above can't exercise
Record, Go Live, or anything else live-only. On this headless box there's no real
device to plug in, so fake one: monkeypatch `roomscan.web.get_best_source` to return an
object with `.read()/.write()/.close()` that replays a real capture's raw bytes in a
loop (`.read()` returning chunks + a small `time.sleep` to mimic pacing), then call the
**real** `web.main()` so every other app-state field is wired exactly like production
(`client` stays `None` since a `FakeLive` isn't a `SerialSource`/`UdpSource`, so `cmd`
dispatch won't round-trip, but recording/session/captures all work — that's the tee
path, not the command path). Verified 2026-07-29 for the post-recording naming modal:

```python
# /tmp/webtest/launch.py — NOT committed, scratch only
import sys, time
from pathlib import Path
sys.path.insert(0, "/path/to/roomscanner/host/src")
import roomscan.web as web

RAW = Path("/tmp/webtest/src.bin").read_bytes()   # any real capture, e.g. captures/verify_slam.bin

class FakeLive:
    def __init__(self, data): self.data, self.pos = data, 0
    def read(self):
        time.sleep(0.03)
        chunk = self.data[self.pos:self.pos + 4096]
        self.pos = (self.pos + 4096) % len(self.data) if self.pos + 4096 < len(self.data) else 0
        return chunk or self.data[:4096]
    def write(self, d): pass
    def close(self): pass

web.get_best_source = lambda *a, **kw: FakeLive(RAW)
sys.argv = ["roomscan-web"]
sys.exit(web.main())
```

Run it exactly like the normal recipe (`setsid …/python /tmp/webtest/launch.py`, sandbox
off, from a scratch cwd so `captures/`/`results/` land in `/tmp/webtest/` and never touch
the repo's `host/captures/`). Click `#btn-record` to start/stop from a step same as any
other button — the disabled check only reads `session.has_live`, which is now `True`.
Teardown: `kill -9` the pid (not `pkill -f roomscan.web` — the process is literally
running `launch.py`, that pattern won't match) and `rm -rf` the scratch dir.

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

## The diagnostics feed

`window.__diag` no longer has its own pane (removed 2026-08-05). It writes
**straight into the bottom event-log console** (`#log-lines`), as `diag` rows
(cyan) or `error` rows (red), alongside device `event`/`cmd`/`log` lines — one
place for both client-side faults and device events. It writes to `#log-lines`
directly (a plain DOM append, no ES module), so it stays visible even when the
modules fail to load — the exact case the sink exists for. `ui_screenshot` /
`web_ui_shot.py` print the `#log-lines` tail (was `#diag-log`).

The console is **collapsed by default** and **opens itself on the first error**
so a silent module/import failure is still visible without devtools; clicking its
header persists the choice in `localStorage['roomscan.card.log.collapsed']`, and
an explicit choice wins over the auto-open. To force it open in a driven run:
`document.getElementById('log-console').classList.remove('collapsed')`.

Driving gotchas (cost time in Web Phase 3):
- **Wait for server-rendered lists before clicking them.** Tiles built from a
  `captures`/`session` message (the capture browser, any server-driven list) don't
  exist until that message arrives (~0.5–1.5 s after `list_captures`/connect). A step
  that does `[...cap-tile].find(r=>r.dataset.name===X).click()` too soon calls `.click()`
  on `undefined` and the step throws — the action never fires and you debug a phantom.
- **The browser and preview cards only exist on the View page.** They carry
  `.hidden` until a `state` echo says `source === "view"`, so a screenshot taken
  on Live shows neither. Send `{"type":"set_source","source":"view"}` (or use
  `rig_view(source="view")`) and let the echo land before asserting on them.
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
