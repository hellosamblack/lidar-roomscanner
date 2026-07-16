# Web Phase 3 — Recording & Playback (design)

Status: ✅ Complete (2026-07-16) — 45 backend tests, full host suite 625 passed / 1 skipped, driven
end-to-end in headless Chrome (record-disabled-in-replay, capture library, runtime load/swap,
pause/resume, speed ×0.5/×1/×2/Max, loop, seek — all confirmed on screen).
Predecessors: Web Phase 1 (core instrument), Web Phase 2 (sensors). Same host-only,
reuse-don't-reimplement discipline. Confined to `host/src/roomscan/web.py` +
`host/src/roomscan/static/` + `host/src/roomscan/sources.py` (one additive param) +
`host/tests/test_web.py`. No wire-protocol or firmware change.

## 1. Goal & scope

The web app runs **remotely on the headless box**, so the desktop panel's model —
`Record` on live + `Pause`/`replay-fps` only when *launched* with `--replay` — is a poor
fit: an operator on the far side of Tailscale can't re-launch with `--replay`. Owner
picked **Full remote** (2026-07-16):

1. **Record** a live session to `captures/web_<ts>.bin`, with live status (elapsed, bytes).
2. **Capture library** — the browser lists `captures/*.bin` and the operator picks one.
3. **Load & play at runtime** — selecting a capture *swaps the reader* to replay that file;
   a **Go Live** control swaps back to the device. No relaunch.
4. **Transport** in playback — Pause/Resume, speed, Loop, and a **seekable progress bar**.

Non-goals (deferred, unchanged from ROADMAP): SLAM trajectory/mesh (Web Phase 4),
showcase (5), settings persistence + retiring `panel.py` (6).

## 2. The one hard part: runtime source-swap

Phases 1/2 built the source + reader thread **once** in `main()`. Phase 3 makes the reader
lifecycle owned by a **`SessionController`** so it can be stopped and restarted against a
different source without touching the single broadcaster or the shared `slot`.

- **Persistent live source.** The live source (serial/UDP) is created once at startup (unless
  launched with `--replay`, then there is none). It is **never handed to `pump()` directly**,
  because `pump()`'s `finally: source.close()` would close it. Instead it is wrapped in a
  **`_NoCloseSource`** proxy (delegates `read`/`write`, `close()` is a no-op) so a swap to
  replay leaves the live device untouched and **Go Live re-uses it instantly** (no 5 s UDP
  re-probe). Real teardown closes the underlying source explicitly at process exit.
- **Reader wrapper.** `SessionController._run()` loops: build a fresh `StreamDecoder`, open the
  current source (live proxy, or `FileSource(path, start=offset)` for replay), call the
  **unchanged** `panel._run_reader(..., is_stopped=self._stop.is_set, recorder=self.recorder,
  state=sensor_state, metrics=metrics)`. On return it distinguishes **manual stop** (`_stop`
  set → break) from **natural EOF** (replay finished): if `loop` and replay → restart from the
  top; else publish `replay finished` and park.
- **Swaps run off the event loop.** `load_capture`/`go_live`/`seek` call
  `await asyncio.to_thread(controller.<op>)`; the controller serializes them under a
  `threading.Lock`. A swap = set `_stop`, `join(timeout)`, flip mode/target, clear `_stop`,
  start a new reader thread. Live→replay keeps the live proxy open; replay→live best-effort
  flushes a serial RX buffer (`reset_input_buffer`) so Go Live doesn't replay stale bytes.

### Commands during replay
The `CommandDispatcher` stays bound to the live client. The inbound `cmd` handler gates on
`controller.mode`: in replay it publishes `"<label> -> not available in replay"` on the bus
(classified `error` by the existing `classify_bus_line` → toast), so no device round-trip
happens and no dispatcher change is needed.

## 3. Capture index (position + seek)

On `load_capture`, the controller scans the file once (in `to_thread`) to build an index —
frames are self-delimiting (`HEADER_SIZE + payload_len + 4`, magic-anchored), so this is a
cheap linear walk:

```
CaptureIndex = {
  n_frames:   int,              # DATA frames only (RAW_3DMD / DEPTH_ZF32)
  offsets:    list[int],        # byte offset of each DATA frame
  seqs:       list[int],        # header.seq of each DATA frame (monotonic within a capture)
  calib_spans: list[(off,end)], # byte spans of CALIB frames, in file order
}
```

- **Position (read-only progress).** The broadcaster already holds the latest `header.seq`;
  `position = (seq - seqs[0]) / (seqs[-1] - seqs[0])` in `[0,1]`, reported in the `session`
  message with `total_frames`. No reader edit.
- **Seek.** `seek(frac)` → `i = round(frac*(n-1))` → `off = offsets[i]`. RAW frames need their
  governing CALIB (pipeline: "first CALIB wins"), so the controller **pre-feeds every
  `calib_span` at or before `off`** into the fresh decoder, then opens `FileSource(path,
  start=off)`. DEPTH_ZF32 captures have no CALIB spans → nothing to pre-feed, still correct.
  Seek is a stop+restart (~50 ms); the client debounces (seek on pointer-release).

`sources.FileSource` gains one additive optional param `start: int = 0` (`self._f.seek(start)`
after open) — desktop-compatible, the only edit to a shared module.

## 4. Wire additions (all on the existing `/ws`)

Inbound (client → server) JSON:

| type        | fields                                   | effect |
|-------------|------------------------------------------|--------|
| `record`    | `on: bool`                               | start/stop recording (no-op+error log in replay) |
| `list_captures` | —                                    | server broadcasts a fresh `captures` message |
| `load_capture`  | `name: str`                          | swap reader → replay `captures/<basename>` |
| `go_live`   | —                                        | swap reader → live (error if no live source) |
| `transport` | `action: pause\|resume\|speed\|loop\|seek\|restart`, `value: number` | playback control |

Outbound (server → client) JSON:

- **`session`** — broadcast on change **and** on the metrics cadence (so timer/position tick):
  ```
  {type:"session", mode:"live"|"replay", source_label, has_live,
   recording:{active, path, elapsed_s, bytes},
   playback:{is_replay, capture_name, paused, speed_fps, loop, position, total_frames}}
  ```
- **`captures`** — `{type:"captures", items:[{name, bytes, mtime}]}` (newest first). Broadcast
  on connect, on `list_captures`, and after a recording stops.

`speed_fps`: `0` = as-fast-as-decoded (interval 0), else the pacer interval `1/fps`. The UI
offers ×0.5/×1/×2/Max over a base of 30 fps → `{15, 30, 60, 0}`.

## 5. Frontend — new `capture.js` module

One vanilla ES module (the 8th), constructed in `app.js` like the others; talks only through
the hub. Drives all active/disabled state **from the server's `session`/`captures` echo**
(one-way flow, per Phases 1/2) so multiple tabs stay in sync.

New right-rail group **"Capture & Playback"** in `index.html`:
- **Record** toggle — visible when `has_live`; shows `● Rec 00:14 · 2.3 MB` while active;
  disabled in replay mode.
- **Source** — a `● Live` row plus one row per capture (name · size); click swaps. A ⟳ button
  fires `list_captures`. Active row = current source.
- **Transport** (shown only when `is_replay`): Pause/Resume, ×0.5/×1/×2/Max segmented, Loop
  toggle, and a progress `<input type=range>` (seek on release; live-updated from `position`).

No second WebGL context, no build step — consistent with Phase 2's 2D-canvas choice.

## 6. Testing

Pure/unit (no socket): capture-index builder (offsets/seqs/calib spans on a synthetic mixed
capture), `session`/`captures` message shape, `sanitize_capture_name` (basename-only, `.bin`,
must-exist), transport→pacer effects, record gating in replay.

Integration (real uvicorn + `websockets`, extends the Phase-1 harness): record a live-ish
stream to a temp dir and confirm bytes land; `load_capture` swaps a running server from one
capture to another and both clients see the new stream; `go_live`/loop/seek smoke.

End-to-end: driven in headless Chrome (SwiftShader) per `docs/web-ui-testing.md` against a
recorded capture — record, list, load, pause, speed, loop, seek all confirmed on screen; server
log clean. Target: full host suite green (≥ 610 + new).

## 7. Risks

- **Serial staleness on Go Live** — mitigated by best-effort `reset_input_buffer`; UDP is
  self-healing via keepalive.
- **Seek into a RAW capture without re-feeding CALIB** → blank frames. Mitigated by the
  `calib_spans` pre-feed; covered by a test on a RAW+CALIB synthetic.
- **Swap races (two tabs)** — serialized by the controller lock; idempotent (`load_capture`
  of the current file is cheap).
- **Recording a replay** — disallowed (Record disabled/ignored in replay mode).
