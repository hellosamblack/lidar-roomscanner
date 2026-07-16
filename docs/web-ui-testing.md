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
Useful element ids live in `host/src/roomscan/static/index.html` (e.g. `btn-ping`,
`seg-color button[data-mode=…]`, `chk-ir-freeze`, `log-toggle`, the Web-Phase-3
capture controls `btn-record`, `btn-refresh-caps`, `#cap-list .cap-row`,
`btn-playpause`, `seg-speed button[data-fps=…]`, `chk-loop`, `seek`, and the
Web-Phase-4 SLAM controls `#seg-mode button[data-mode=realtime|slam]`,
`chk-slam-traj`, `chk-slam-follow`, `#seg-walls button[data-walls=split|solid]`,
`btn-save`, `#saved-list .cap-row a`).

**SLAM verification needs a stream-9 capture.** SLAM builds nothing from a capture
with no IMU_QUAT (stream 9) — the mapper gets no rotation prior and loses tracking
(`recordings/2026-07-08-room-scan.bin` predates IMU → empty map). Use
`captures/verify_slam.bin` (has 9/10) or record a fresh one from the live board. To
build the map, launch with `--replay <stream9.bin> --replay-fps 30`, click
`#seg-mode button[data-mode=slam]`, and enable Loop (`chk-loop`) so frames keep
feeding — SLAM is fed from the 30 Hz broadcaster only while in SLAM mode, so ~330
frames take ~11 s to integrate. `window.__gotMesh` and the diag line
`slam.js: first mesh: N non-wall verts` confirm the mesh path (the *first* emit is
an empty packet — N=0 — by design; later ones carry geometry).

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

## Teardown

`pkill -9 -f roomscan.web` (and `-f remote-debugging-port` if you reused a Chrome).
Put temp replays/PNGs in the session scratchpad, not the repo.

See also `docs/headless-host-setup.md` (host bring-up) and
`host/tools/headless_doctor.py` (checks WebGL-capable browser is installed).
