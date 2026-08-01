# The `roomscan` MCP server

`roomscan-mcp` exposes the dev loop as typed tools with structured results. It is the
primary agent-facing surface; the `host/tools/` CLIs remain for humans and for
clients without MCP.

**Why it exists.** Two reasons, both measured rather than assumed:

1. **The structure already existed and was being thrown away.** `analyze_capture`
   built anomaly dicts with byte offsets, `Doctor` accumulated per-check verdicts,
   `orientation_probe` returned summary dicts -- and each one flattened that into
   prose at the `print()` boundary for an agent to re-parse. Every wrapped tool now
   returns the dict directly.
2. **One process can hold state across many tool calls.** Each Bash call is a fresh
   process, so UI checks relaunched Chrome and paid an ~8 s settle every time. The
   server keeps one browser and one `/ws` connection warm.

**Not a reason:** sandbox escape. An earlier draft of this work claimed the agent
Bash sandbox kills network listeners (uvicorn → exit 144). Measured 2026-07-29:
uvicorn binds `0.0.0.0`, serves a request and exits 0 from inside a normal Bash
call, as do plain listeners on loopback and `0.0.0.0`. See "Stale claim" below.

## Running it

Registered for this repo in `.mcp.json` (project scope, checked in, so the
`.agents/` runtime picks it up too):

```jsonc
{"mcpServers": {"roomscan": {
  "command": "host/.venv/bin/python", "args": ["-m", "roomscan.mcp_server"],
  "env": {"PYTHONPATH": "host/src:host"}}}}
```

Install with `pip install -e "host[mcp]"` (pulls `mcp>=2.0`, `playwright`, and the
`[web]` extra). Note SDK 2.x renamed `FastMCP` to `mcp.server.MCPServer`.

**stdio vs HTTP.** The default stdio server is a child of its MCP client and dies
with it, so the browser and `/ws` connection are rebuilt each session. For
cross-session warmth run it standalone instead:

```sh
host/.venv/bin/python -m roomscan.mcp_server --http --port 8765
claude mcp add --transport http roomscan http://127.0.0.1:8765/mcp -s project
```

Both transports serve the same tool definitions.

⚠ **A running server pins the code from when it started.** Add or change a tool and the connected
server keeps serving the old one — `capture_analyze` went on returning results with no `continuity`
block for the rest of the session that added it (2026-07-31). There is no reload. In the session that
writes a tool, drive it through its `host/tools/` CLI from Bash, and tell subagents to do the same;
the MCP surface catches up on the next server start. Same failure shape as a stale `roomscan-web`
(ROADMAP 6.D's "SLAM gave up", which was a process running week-old code).

## Tools

**rig_\*** — control a running `roomscan-web` over `/ws` (`docs/web-protocol.md`).

| tool | notes |
|---|---|
| `rig_status()` | start here: server up?, source, fps, playback frame, streams |
| `rig_up(replay?, replay_fps?)` | starts the server detached; `replay` for deterministic checks |
| `rig_down()` | terminates it and drops the `/ws` connection |
| `rig_command(name, param?)` | device COMMAND/ACK; `ok=false` when the ACK is an error |
| `rig_record(on)` | records via the server, returning the capture path |
| `rig_set(...)` | legacy display options; verifies the echo before reporting success |
| `rig_view(source?, display?, regenerate?)` | authoritative Live/View and Point cloud/Preview/SLAM/Detailed control |
| `rig_playback(action, value?)` | `go_live` / `load_capture` / transport |
| `rig_save()` | export the **Live** SLAM map (`.ply` + `.tum`). Live SLAM only — a live scan is unrepeatable, so its one-shot export stays; for a recorded capture the persistent artifact is the sidecar from `rig_view(display="detailed", regenerate=True)` |

**ui_\*** — headless Chrome, held open across calls.

| tool | notes |
|---|---|
| `ui_screenshot(...)` | returns the PNG as an image block plus the `#diag-log` tail |
| `ui_eval(js)` | JS result as JSON; JS errors come back as `ok=false` |
| `ui_wait_for(predicate_js)` | wait on a real condition instead of sleeping |
| `ui_reset(relaunch?)` | clean page between scenarios -- server state persists |

Useful readouts: `#pos-status` ("frame N / total", replay only), `#hud-view-fps`,
`#hud-device-fps`, `#record-status`, `#ir-frame`, `#slam-frames`. Element ids live in
`host/src/roomscan/static/index.html`. `window.__diag` is the page's *logging sink
function*, not a state object -- read what it logged via `ui_screenshot`'s tail.

**data** — `capture_list()` (includes `has_stream_9`, which SLAM and orientation work
ask constantly), `capture_analyze(path)`, `capture_magcheck(path, cal_path?, compare?)`,
`capture_skew(path, window_s?)`, `capture_motion(path)`,
`slam_rerender(capture, voxel_size?, block_count?, device?, max_frames?)`,
`slam_ensemble(capture, n?, device?, voxel_size?, block_count?)`,
`doctor()`, `orientation_probe(mode)`.

`slam_loop_closure_gate(baseline, loop_closure)` applies the pre-registered paired 95% confidence gate to
the two matched circuit ensembles; global loop closure stays disabled unless both
circuits pass it without a tracking regression.

`slam_ensemble` is what produces those matched ensembles, and is the right tool
whenever a drift number is going to be **quoted or compared** — `slam_rerender` runs
once, and one run is not a measurement. Its own validation pass spread 0.477–0.966 m
of closure across ten numerically innocuous perturbations of the *same* capture
(mean 0.670 ± 0.154 m, reproducing the recorded 0.74 ± 0.19 m baseline), which is
BUG-037's chaos made visible. It reports `summary.horizontal_closure_m` — the gate's
required input, which nothing else in the repo computed — split from
`vertical_error_m` on the real world-up axis (**−Y**, not −Z). Check `runs_died` and
`any_saturated` before quoting anything; and remember closure is only *drift* if the
operator actually returned to the start pose. Budget ≈ frames × n × 7 ms on CUDA:0.

`capture_skew` measures where a depth frame actually sits on the IMU's clock, from stream 13
`IMU_SYNC` (BUG-031). ⚠ Its number is **window-dependent** — 18/38/150 µs RMS at `window_s`
2/5/20 — because what survives the fix is the two oscillators drifting apart, not per-frame skew
(lag-1 autocorrelation 0.992). Quote the window alongside the figure, or use the window-free
10–11 µs. It also reports the quaternion's phase offset, which is a **+7.8 ms lead**, not a lag.

`capture_analyze` answers **two** questions that are easy to conflate. `clean` means the
bytes that arrived decode end to end; `continuity.complete` means everything the device
sent actually arrived. A capture can be `clean: true` and still be missing seconds of
frames — the three 2026-07-31 multi-room captures were byte-perfect while losing
2.3 % / 4.3 % / 9.4 % of RAW frames, one in a single 215-frame (7.1 s) hole, which is why
the census exists. `whole_group_lost` (a seq absent from every stream ⇒ link outage) and
`partial_group_lost` (absent only from RAW_3DMD ⇒ fragment loss on the ~15 KB datagram)
separate two different faults, and `device_fps` against `received_fps` shows what the
device produced versus what the recorder kept. **Check it before quoting any SLAM result
computed over a capture.**

`capture_motion` reports what the operator physically did — `segments` alternating
`hold`/`move` runs, `takes`, `fast_events`, bookend flags, per-hold tilt. Several
data-collection gates are conditions on motion, not on data (DC-F's stationary bookends,
DC-E's 15 s tilt holds, DC-D's "panning the whole time", DC-A's fast whips), and this
checks them directly. Rate uses the measured `dt`, never a nominal 1/30 s — with frames
being lost, a two-frame step would otherwise read as a phantom doubling of speed — and
long gaps are reported as `unmeasured_frac` rather than interpolated across. Tilt is
degrees from straight up in the SFLP quaternion's Z-up world (0 = ceiling, 90 =
horizontal), matching `capture_magcheck`'s tilt table; deriving it in the renderer's Y-up
frame instead turns a pan into an apparent 90° tilt sweep.

`slam_rerender` is the offline high-detail pass. A capture stores raw ToF frames, not a
map, so the live scan is only a preview and the pipeline can be re-run at any resolution
afterwards — `voxel_size=0.005` roughly doubles detail over the 10 mm default. Two limits
worth knowing: the sensor samples ~36 mm between rays at 2 m, so below ~5 mm the extra
detail comes only from multi-view fusion (dense back-and-forth sweeping), never from one
view; and blocks scale as 1/voxel², so halving the voxel wants ~4× the `block_count`.
**Always read `map.saturated`** — a scan that ran near its configured capacity is exactly
where map growth stalled and tracking collapsed (BUG-035). Past ~6 GiB of grid use
`device="CPU:0"`, where system RAM rather than VRAM is the limit.

**And read `tracking.died` before quoting `start_end_gap_m` as drift.** A lost frame
freezes the pose and nothing relocalizes, so a dead run still reports a plausible gap that
is only where the estimate stopped — one real circuit reported 2.05 m of "drift" whose last
22% was fabricated (BUG-036). `tracking.trailing_lost` / `longest_lost_run` bound how much
of the tail to distrust; `icp_escalations` counts frames the tight ICP radius could not
handle alone (~0 on a clean scan). Both this and saturation raise a top-level `warning`.

`baro.correction_m` reports how much of the height came from the barometer rather than
from ICP (expect ~10 mm). **`trajectory.path_length_m` is not comparable across
2026-07-30**: before BUG-037 the height constraint fed the barometer's per-frame noise
straight into the pose, so ~35% of reported path was vertical motion that never happened —
and every "% of path" drift figure divided by it came out flatteringly small.

Unlike its neighbours this one shells out to the `roomscan-slam` console script rather than
calling in-process: the job runs for many minutes and would otherwise block the event loop
and pull CUDA into the server. It reads that run's `--json` report instead of scraping
stdout, so prose and structured output stay one implementation with two front ends. Bound
exploratory runs with `max_frames`; the default `timeout_s` is 1800.

`capture_magcheck` scores a magnetometer calibration against a capture it never saw — the
BUG-030 closing test. Read `verdict`, which is the worse of two deliberately different
measurements: `attitude.attitude_locked_pct` (|B| error left after detrending the room's
own slowly-varying field, a **lower** bound — an attitude held longer than the window is
absorbed into the trend) and `tilt_ramp.ratio` (|B| max/min across boresight-tilt bins,
detrend-free, so it catches exactly what the first one hides). `field` is
`magsweep.field_consistency`, correct for a stationary tumble but it under-rates a good
calibration on a walk. `compare=[...]` scores several fits against one capture.

**build** — `fw_build()`, `fw_flash()`, `run_tests()`. These encode the host facts
that bite every session: Ninja comes from the venv, the packaged stlink 1.8.0 cannot
identify an H5 (a locally built `develop` is used, expect chipid `0x484`), and pytest
must run with cwd=`host/`.

**On-rig verified 2026-07-29.** `fw_build()` returned `text=148044 data=13231
bss=54232`, `.bin` 161279 B (`size` is `null` on a no-op build — `ninja: no work to
do` prints no size line, which is honest rather than a parser bug). `fw_flash()`
probed `chipid 0x484`, wrote and verified. It was flashed against a board that had
gone unresponsive — no ping, no ACK, `device_hz: None` — and the flash revived it:
ping 0% loss, streams 7/9/10/11 all at **30.5 Hz, 0 drops, 0 gaps**, `ping` →
`OK applied=1`, and a `rig_record` → `capture_analyze` round trip gave 614 frames,
0 CRC failures. `run_tests()` was checked in both directions: 22 passed on a `-k`
selection, and `ok=False` with the failing test id against a deliberate probe.

## Two invariants

**Client, never competitor.** The server must never bind the device UDP/CDC stream --
`roomscan-web` owns it, and `capture.py --udp` starves it. This is why raw
`capture.py` stays CLI-only and recording goes through `rig_record()`: the rule is
structural rather than something to remember. Verified with `ss -uanp`: only
`roomscan.web` binds `0.0.0.0:5000`.

**Report what happened, not what was requested.** roomscan-web broadcasts `state` and
`session` on a timer as well as on change, so the first one after a request can
predate it -- which reads as success while reporting the old value, or (worse) as
failure while the thing actually worked. Five real bugs of this shape were caught
during verification, every one of them returning a confident wrong answer:

| tool | symptom | fix |
|---|---|---|
| `rig_set` | reported the *old* colour after a successful set | poll until the echoed field matches |
| `rig_playback("go_live")` | `ok` on a `--replay` server, which has no live source | check `is_replay` actually cleared |
| `rig_record(on=True)` | `ok` in replay, where `start_record()` is a no-op | check `active`, explain why |
| `rig_record(on=True)` | **`ok=False` while 734 clean frames were recorded** | poll `session` for `active == on` |
| `rig_command` | `ok` for `status="timeout"` — device never answered | only `status == "ok"` is success |

The last two were found on real hardware, against a board that had stopped
responding. `_await_state` / `_await_session` exist for exactly this: a broadcast
arriving is not evidence that anything changed. A sync test pins `web._cmd_status`'s
vocabulary (`ok | error | busy | timeout`) so a new status cannot silently read as
success.

## Adding a tool

1. Write the logic as a **pure function returning structured data**, in the
   `host/tools/` script if one exists. Keep its `argparse` `main()` as a prose
   printer over that same function -- one implementation, two front ends.
2. Register a thin wrapper in the matching `roomscan/mcp_server/tools_*.py`. The
   **docstring is the description the agent sees**; write it for someone who has
   never seen the tool. Type the parameters -- the schema comes from the signature.
3. Lazy-import anything heavy (`open3d`, `numpy`) inside the function body. A test
   asserts the server builds without importing them.
4. Update `EXPOSED`/`EXCLUDED` in `host/tests/test_mcp_registry.py` and this file.

Not everything should be wrapped. The scratch tier, the deprecated-panel tools, and
the rare one-shot rigs stay CLI-only; each is listed in `EXCLUDED` with its reason,
and the test fails on any script that is neither exposed nor excluded.

## Browser backend

`session.py` defines the browser as an interface (`goto`, `evaluate`, `wait_for`,
`screenshot`, `diag_tail`) with two implementations. **Playwright is the default**
(`channel="chrome"` drives the system Chrome, so there is no browser download);
`CdpSession`, the raw-CDP plumbing lifted from `web_ui_shot.py`, is the fallback when
playwright is not installed.

**Spike result (2026-07-29): Playwright passes on this GPU-less host.** With
`--enable-unsafe-swiftshader --use-gl=angle --use-angle=swiftshader`, it obtained a
WebGL context, logged `first point cloud: 2256 pts`, and produced a screenshot
equivalent to the CDP one. It was adopted for native waiting: `wait_for_function` is
driven by the page's own event loop rather than a 4 Hz poll, so `ui_wait_for` cannot
miss a transient condition between polls.

## Stale claim: the Bash sandbox and listeners

`docs/headless-host-setup.md` and the `agent-sandbox-port-binding` memory state that
the agent Bash sandbox kills network listeners (uvicorn → exit 144), and prescribe
working around it. **That did not reproduce on 2026-07-29**: uvicorn on `0.0.0.0`
served a request and exited 0 from a normal Bash call; `$HOME` writes and external
HTTPS were also allowed. It may have been accurate when written. Re-verify before
relying on either the claim or its workarounds.
