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
   server keeps one browser and its websockets warm — since BUG-061 that is **two**
   sockets, `/ws` plus the credit-gated `/ws-mesh`, which `RigSession` opens
   best-effort and **auto-acks**. The ack matters: an agent client that never acked
   would hold its mesh credit and be throttled to the legacy 1 mesh / 5 s, and
   `rig_status()`'s `binary_tags_seen` would stop reporting MESH (tag 3) entirely,
   because tag 3 no longer travels on `/ws` at all. A server predating `/ws-mesh`
   simply leaves `mesh_connect_error` set and everything else works.

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

⚠ **The client's tool *list* can lag the server's code independently.** Observed 2026-08-04: after a
restart, `rig_status()` returned fields that only exist in the newly-added code — so the server was
demonstrably running it — while three tools added in the same commit were absent from the client's
tool list and could not be called. Code freshness and surface freshness are two questions; a tool
"not existing" is not evidence the server is stale. Check by calling a tool that already exists and
looking for a field the new code adds, and drive the new one through its `host/tools/` CLI or an
in-process call meanwhile.

## Tools

**rig_\*** — control a running `roomscan-web` over `/ws` (`docs/web-protocol.md`).

| tool | notes |
|---|---|
| `rig_status()` | start here: server up?, source, fps, playback frame, streams |
| `rig_up(replay?, replay_fps?)` | starts the server detached; `replay` for deterministic checks |
| `rig_down()` | terminates it and drops the `/ws` connection |
| `rig_command(name, param?, timeout?)` | device COMMAND/ACK, matched to the command sent; `ok=false` when the ACK is an error. `timeout` defaults to 5 s, except `standby` at 20 s — it powers the ToF laser down/up and does not ACK inside 5 s (#171) |
| `rig_idle(auto_idle?, level?, wake?)` | wake the ToF laser and/or control the laser-wear auto-idle; disable `auto_idle` before recording a static scene (else the laser parks mid-capture and only IMU is recorded), restore it after |
| `rig_record(on)` | records via the server, returning the capture path |
| `rig_set(...)` | legacy display options; verifies the echo before reporting success |
| `rig_view(source?, display?, regenerate?)` | authoritative Live/View and Point cloud/Preview/SLAM/Detailed control. With `regenerate=True` it awaits the server's `detailed` broadcast (the same way `rig_save` awaits `saved`) and returns it as `detailed` (`{"started": bool, "reason": ...}` plus progress fields); `ok=False` with the refusal reason as `error` when the rebuild did not start (#189) |
| `rig_profile(profile?, ranging_mode?, fps?, exposure_ms?, power_mode?, force?)` | read or set the ranging profile; `ok=true` only once the **device** reads back the requested config (see below) |
| `rig_imu_env_rate(rate_hz?, coupled?, require_full_env?)` | read or set the IMU/env poll rate (streams 9/10/11) — a second, independent command from `rig_profile` |
| `rig_playback(action, value?)` | `go_live` / `load_capture` / transport. **`go_live`, `load_capture`, `seek` and `restart` discard the ephemeral SLAM map** — they begin a new replay timeline, so the worker/TSDF/trajectory/cached mesh and all sensor state reset (BUG-091, `docs/web-protocol.md` → "Timeline discontinuities"). Seeking mid-scan throws the scan away and restarts from the seek point; `pause`/`resume`/`speed`/`loop` keep the map |
| `rig_save()` | export the **Live** SLAM map (`.ply` + `.tum`). Live SLAM only — a live scan is unrepeatable, so its one-shot export stays; for a recorded capture the persistent artifact is the sidecar from `rig_view(display="detailed", regenerate=True)` |
| `rig_ws_probe(seconds?, url?)` | splits "nothing rendered" into server-computing / transport-delivering / payload-well-formed. Acks every mesh (`/ws-mesh` is credit-gated, so a silent client sees one mesh then a 1-per-5-s trickle) and re-parses one MESH with `slam.js`'s exact layout — `slack_bytes != 0` means packer and reader have drifted. Keep `seconds` small: connection count is a performance variable here (BUG-060/061) |
| `rig_thin_probe(frames?, url?, out_dir?, orbit_yaw?, modes?, record?, timeout?, format?, fps?, width?, height?, quality?, v2?, credits?, max_frame_bytes?)` | what `/ws-thin` is actually drawing, as PNGs — stands in for the CrowPanel thin client (#194). Decodes `THIN_FRAME` (`<IHH` header, tag 1, `w*h*2` RGB565 bytes) and, after negotiating via `thin_hello` (`format="jpeg"` etc., #197), `THIN_FRAME_JPEG` (`<IHHII` header, tag 2 — `tag, w, h, seq, payload_len`, #202 — baseline 4:2:0 JFIF payload) into `results/thin_probe/<timestamp>/*.png`; with a hello requested, `ok` additionally requires the ack arrived AND the recorded frame tags match the negotiated format (`hello.degraded` reports a jpeg→raw downgrade), served at `/results/…`. `measured_fps` and `measured_mbps` cover the initial decoded receive batch and are timestamped before per-frame statistics and PNG writes, so the probe's observability cost cannot throttle the credit loop and then masquerade as server/network latency. `v2=True` sends the CrowPanel spec's proto-2 hello (`accept` list + `credits` 1–8 + `max_frame_bytes`) and auto-grants a `thin_ready` per consumed frame, exercising credit flow control end to end — watch the per-client `tx_fps`/`tx_bytes_per_s`/`dropped` telemetry vs the rig-internal `fps` (#202). **The observable is pixels:** `thin_orbit` is scored by `orbit.changed_frac` against `orbit.control_changed_frac` — the same measure on two consecutive frames with no command — so scene motion is subtracted by construction and a null can't be explained away (the #106 lesson); `moved_pixels` needs ≥10% of pixels changed *and* 3× the control. `thin_mode` is confirmed by the server's own `thin_telemetry`, then requires a frame → `modes_with_frames`; a confirmed mode with no frame is a finding (the loop sends nothing for a stale/empty generation-tagged stash, #101). Check `distinct_colors` before believing a frame — a failed render is a uniform buffer. `record=False` by default: `thin_record` starts a REAL capture. Server down / `thin_client_limit` / `thin_render_unavailable` come back in `server_error`, never as an exception |

**ui_\*** — headless Chrome, held open across calls.

| tool | notes |
|---|---|
| `ui_screenshot(...)` | returns the PNG as an image block plus the `#log-lines` (event-log) tail |
| `ui_eval(js)` | JS result as JSON; JS errors come back as `ok=false` |
| `ui_wait_for(predicate_js)` | wait on a real condition instead of sleeping |
| `ui_reset(relaunch?)` | clean page between scenarios -- server state persists |

Useful readouts: `#pos-status` ("frame N / total", replay only), `#hud-view-fps`,
`#hud-device-fps`, `#record-status`, `#ir-frame`, `#slam-frames`. Element ids live in
`host/src/roomscan/static/index.html`. `window.__diag` is the page's *logging sink
function*, not a state object -- read what it logged via `ui_screenshot`'s tail.

### Ranging control is effect-verified, not fire-and-forget

`rig_profile()` and `rig_imu_env_rate()` change what the sensor physically does, so
neither reports success on anything less than the device's own readback. Both wait for
their half of the `ranging` broadcast to go **pending and then settle onto the requested
configuration** — the server re-broadcasts that message on its ~4 Hz metrics tick, so a
merely newly-arrived one (never mind the cached one, or the log line the UI prints) is
compatible with nothing having happened at all. They are **two independent pending
commands sharing one message**: each waiter reads only its own half, so a profile change
in flight is neither failed by an IMU-rate error nor confirmed by an IMU-rate settle.

`ok=false` covers a busy server, a replay source, host-side validation failure, a device
error (BUSY/BAD_PARAM/timeout), an unsupported CDC rate, and an applied-vs-requested
mismatch — and the result still carries what the device *actually* has applied. Requests
the host model rejects never reach the device. Two deliberate refusals with escape
hatches: above 60 fps over USB CDC needs `force=True`, and `require_full_env=True` makes
an IMU/env rate above the 60 Hz sensor-hub cycle an error rather than a warning (stream
10 sub-samples there; 9 and 11 do not). Leave ≥2 s between reconfigurations — a faster
one can produce no ACK at all (BUG-073).

Four tools, four different questions, all over one `roomscan.profiles` model:
`profile_estimate()` is offline — what a configuration *would* do, safe to call with no
rig at all; `rig_profile()` is what the device *is* doing now; `capture_profile_probe()`
is what a recording *actually delivered*; and `profile_tuning()` sets our model beside
**ST's** ProfileTuning planning model and names every place they disagree rather than
picking the friendlier number (ST's flat timing model under-rates this hardware, and its
DSS-off row assumes a 106-byte frame our I3C path has never been shown to produce). All
four report an estimate in the same shape, so they can be compared field by field.

Read `expected_delivered_fps`, never the requested `fps`: above an exposure's measured
1× ceiling the sensor accepts the request and delivers period-multiples. **It is a model
output, and the model has a known hole** — BUG-075: a 50 Hz / 2 ms request delivers a
near-even bimodal 20/40 ms alternation instead of a clean multiple, so for
short-period/short-exposure combinations treat the number as indicative and measure the
capture (`capture_profile_probe()`) before quoting a rate. `measured_fps` on the same
result is the link's own observation and carries no such caveat. `rig_status()`
carries the same ranging state (`ranging_profile`, `ranging_measured_fps`,
`imu_env_rate_hz`, `imu_env_coupled`, plus the whole `ranging` message).

**data** — `capture_list()` (includes `has_stream_9`, which SLAM and orientation work
ask constantly), `capture_analyze(path)`, `capture_magcheck(path, cal_path?, compare?)`,
`capture_skew(path, window_s?)`, `capture_motion(path)`, `capture_heading(path, cal_path?)`,
`capture_meta(path)`, `capture_profile_probe(path, requested_fps?, udp_stats?)`,
`profile_estimate(profile?, ranging_mode?, fps?, exposure_ms?, power_mode?, imu_env_rate_hz?, transport?)`,
`profile_tuning(ranging_mode?, power_config?, resolution?, dss?, output_interface?, fps?, exposure_ms?, ambient_lux?)`,
`slam_rerender(capture, voxel_size?, block_count?, device?, max_frames?, icp_trace_start_s?, icp_trace_end_s?)`,
`slam_ensemble(capture, n?, device?, voxel_size?, block_count?, icp_mode?, max_frames?, apply_quat_phase?, quat_interp_mode?)`,
`slam_stall_profile(capture, frames?, device?, decimate?)`,
`slam_icp_bench(capture?, what?, frames?, raycast_frames?, ensemble_n?, device?, ab_pairs?, ab_frames?, baseline_icp_device?, candidate_icp_device?)`,
`splat_list()`, `splat_build(video, name, force?, fps?, iters?, max_gaussians?)`,
`splat_compare(capture, reference?, opacity_min?, voxel?, allow_scale?)`,
`splat_vram_sweep(video?, model_dir?, image_dir?, budget_gib?, margin_gib?, reserve_gib?, ladder?, sh_degree?, long_edge?, depth_lambda?, worst_k?)`,
`splat_sfm_probe(video, configs?, fps?, max_frames?, long_edge?)`,
`splat_render(ply, out, transform?, azimuth?, elevation?, opacity_min?, core_pct?, iso_scale?, width?, height?, views?)`,
`doctor()`, `orientation_probe(mode)`.

**bridge** — the Raspberry Pi 3 bridge node (issue #191), which replaces the RavPower
FileHub as the scanner's wireless uplink: `bridge_status(host?)`,
`bridge_logs(unit?, lines?, host?)`, `bridge_wifi_update(ssid, psk, profile?, host?)`,
`bridge_update(host?, secrets?)`, `bridge_tee_list(host?)`,
`bridge_tee_fetch(name, out?, convert?, host?)`, `bridge_reboot(host?)`,
`bridge_image_build(secrets?, release?, xz?, skip_debs?)`,
`bridge_pcap_convert(path, out?)`.

The FileHub's defect was never that it broke -- it was that nothing about it was
observable when it did, so a capture that lost 2.3-9.4 % of its frames looked exactly
like a clean one (#60). These tools exist to make that hop legible, so each reports the
state it read back rather than the action it requested: `bridge_wifi_update` returns the
association state after the radio settles (not nmcli's exit code), `bridge_status`
returns the driver's actual `power_save` and the nftables counters that prove traffic is
moving, and `bridge_update` reports the unit states afterwards. An unreachable Pi returns
`{"ok": false, "error": ...}` -- it does not raise.

`bridge_tee_fetch` is the recovery path: the Pi tees every scanner packet to a bounded
2 GB pcap ring, and the fetched pcap converts to a capture `.bin` that is
byte-format-identical to `capture.py`'s output, so every `capture_*` tool reads it
unchanged and its `frame_loss_pct` is directly comparable with what the host recorded for
the same take. Frames the Wi-Fi hop dropped are still in the pcap.

Target resolution for all of them: `$ROOMSCAN_BRIDGE_HOST`, else mDNS
`_roomscan-bridge._tcp`, else `roomscan-bridge.local`; ssh key `~/.ssh/roomscan-bridge`,
generated by `bridge_image_build` and baked into the image. Build and flashing procedure:
`docs/pi-bridge-runbook.md`.

`splat_build` reconstructs a navigable Gaussian splat from a video (offline
frames -> COLMAP SfM -> gsplat 3DGS), landing it under `results/splats/<slug>/`
where it becomes the third Live/View/Splat source in `roomscan-web`. It is a long,
GPU-heavy, host-only job, so it shells out to `roomscan-splat` rather than pulling
CUDA into the server; `splat_list` is the light, torch-free listing behind the
web picker. This is the *standalone* Phase 7 subset -- a rough room from video
alone, no ToF fusion / hand-eye calibration (that remains blocked on DC-I).

`splat_compare` reverses Phase 7: it treats a metric ground-truth splat (an
imported Scaniverse export, by default) as truth and diffs a capture's Detailed-SLAM
mesh against it to expose where our lidar got the room *shape* wrong (e.g. BUG-084's
mid-scan ceiling fork). It rigidly aligns the two -- both are metric, so a
scale/extent mismatch is a finding, not fit away -- and reports alignment
fitness/RMSE, per-axis extents + ratio, floor footprint, bidirectional cloud-to-cloud
distance, and a vertical/ceiling-fork analysis, writing overlay/heatmap PLYs and
floor-plan + elevation PNGs under `results/compare/<stem>__vs__<ref>/`. Torch-free
(open3d + plyfile), shelled out to `roomscan-splat compare` like the other heavy jobs.

`splat_vram_sweep` and `splat_sfm_probe` answer "how dense a splat can we get away
with on THIS capture?" — density has two independent ceilings and each tool measures
one. `splat_vram_sweep` finds the largest MCMC `cap_max` (gaussian count) that fits
VRAM, by measuring peak on the *real* COLMAP scene + frames at forced counts, cycling
every view to catch the worst-case frame's backward. It is the honest replacement for
the discredited synthetic probe, which built a uniform cube and under-reported the true
peak ~2×: the count is forced to exactly N (never grown by MCMC, which under-measures)
using the real cloud's untrained knn scales (a conservative over-estimate), and the fit
is decided on reserved / device-wide NVML, never `max_memory_allocated`. A
`capture_limited` result (low `registered_ratio`) means the VRAM cap is *not* binding.
`splat_sfm_probe` measures the other ceiling — registration — by extracting frames once
and running SfM under several configs (sequential/exhaustive, overlap, SIFT feature
count, loop closure) on the same frames, reporting per config `largest_ratio`,
`total_placed_ratio`, and `n_submodels` so sub-model *fragmentation* (COLMAP splitting a
walkthrough and the build keeping only the largest piece) is visible; it recommends the
config that registers the most single-connected frames. Both run as subprocesses (GPU /
CPU minutes) and never bind the device.

`splat_render` rasterizes a splat `.ply` to a PNG on the GPU (gsplat, base/DC color) so
a build's result can be SEEN on this display-less, llvmpipe box where the browser splat
viewer can't cope at these counts. It auto-frames the cloud (pass the manifest's
`transform` for a levelled camera; `azimuth`/`elevation`/`views` orbit it), and
`iso_scale`+`opacity_min`+`core_pct` turn it into a clean colored *coverage* cloud
(needles/floaters removed) for before/after comparisons.

`slam_stall_profile` answers "why does the live view feel frozen?" with a number.
It replays a capture through the real Live-SLAM pipeline and reports, per stage,
both wall time and the **GIL starvation** a watchdog thread measured — read the
starvation, not the wall time. These are native calls and Open3D holds the GIL,
so a stage's starvation *is* how long the asyncio loop could not run, and the two
differ by an order of magnitude: `prepare_packet` costs 178 ms p50 but only 11.9%
of wall starved (mostly numpy, which releases the GIL), while the same stage with
`decimate=True` costs 2440 ms p50 and **94.3%**. Run it at scale — 1200 frames of
a near-static capture showed zero stalls on code that freezes 1261 ms on a real
room sweep. It runs in a subprocess and never binds the device, so it is safe
beside a live server (both just get slower). Found BUG-060.

Judge that starvation by **`tick_share`**, not by `starved_pct_of_wall` (#74).
Each `gil_starvation[<stage>]` entry, and the report's top level, now carry
`ticks` / `expected_ticks` / `tick_share` (= ticks landed ÷ `stage_wall_s /
period`) beside the legacy fields, matching `slam_icp_bench`'s watchdog. The
legacy percentage is self-defeating at the extreme: a stage that holds the GIL
almost completely leaves the watchdog almost no chance to tick, so there is
almost nothing to sum, and it can read *lower* than a stage that ran freely — 1
tick landed of ~2186 due. A low `starved_pct_of_wall` is only evidence of health
when `tick_share` is near 1; when `tick_share` is low it means the instrument
barely sampled.

`slam_icp_bench` answers "would moving this on (or off) the GPU help?" with
measurements instead of intuition. `what="api"` is the one to run **first** before
writing any device-resident code: it probes each Open3D 0.19 tensor op against a
deep CUDA queue and reports which ones force a hidden host synchronization — on
the installed build `sum(dim=)`, boolean-mask indexing, `nonzero()`, `.item()`,
every linalg entry point *and* `nns.hybrid_search` all sync, while elementwise
ops, `matmul`, gathers, `concatenate` and uploads do not. A call returning a
device tensor is **not** evidence that no sync happened. `what="icp"` races four
solvers over identical recorded inputs; `what="raycast"` costs the
download/mask/re-upload round trip; `what="ensemble"` scores accuracy with a
matched perturbation ensemble and a **non-inferiority** gate whose tolerance is
one standard deviation of the baseline ensemble's own closure. Its `gil` block
reports `tick_share` — read that, not `starved_pct`: code that holds the GIL
completely starves the watchdog itself, so the percentage *under*-reads exactly
when it matters (measured: 1 tick in 10.93 s). Findings in
`docs/superpowers/plans/2026-08-02-cuda-icp-study.md`.

`what="ab"` is the pass to use when a **change** has to be sized, rather than two
implementations compared: an interleaved, paired, whole-pipeline A/B of
`Mapper.icp_device` over the shipped code, alternating which arm runs first in
each pair. Use it instead of `what="icp"` for that job — the isolated
microbenchmark swung **43%** between sessions on identical inputs, which is far
larger than the effect being measured. It reports the per-pair spread (never one
number), a whole-trajectory equivalence check against the other arm (expected
exactly `0.0`, with `frac_frames_with_misses` printed because a full-match scene
has no power over the correspondence handling at all), `tick_share`, and a **CPU
load + GPU** sample around every arm. That last part is not decoration: a
sibling session's headless Chrome at 1200% CPU is invisible to `nvidia-smi` and
slows both arms ~2.3×, and the variant under test is the CPU-bound one.

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
`apply_quat_phase=True` enables #155's timestamp alignment (report field `quat_interp`
says what it *did* — applied/eligible), and `quat_interp_mode="reflected"` runs the
validation-only wrong-direction null arm; a "win" that the reflected arm matches is
not sub-frame-phase evidence (see the 2026-08-12 #155 session ledger).

`capture_skew` measures where a depth frame actually sits on the IMU's clock, from stream 13
`IMU_SYNC` (BUG-031). ⚠ Its number is **window-dependent** — 18/38/150 µs RMS at `window_s`
2/5/20 — because what survives the fix is the two oscillators drifting apart, not per-frame skew
(lag-1 autocorrelation 0.992). Quote the window alongside the figure, or use the window-free
10–11 µs. It also reports `quat_lead_us`, the **signed** distribution of the stream-9
quaternion's batch-midpoint lead over the frame-ready edge — measure it per capture rather than
assuming the number on record: +7.76 ms on the 2026-07 golden capture, +5.1 ± 0.7 ms on 2026-08
rigs, and **−3.9 ms (opposite sign) on officeFullScanAug6** — the sign flip is what killed the
constant-offset plan (#126 → #155).

`capture_meta` decodes the 100-byte `vl53l9_meta_t` tail every RAW_3DMD frame carries (and the host
has always archived but never read). Use it to recover what a capture *actually* ran at — the
exposure (inverted from `nb_shot_step` via AN6522 ratios, so a `...8ms...`-named file that was only
ever a guess is now answerable), the ToF **die temperature** (a few °C over ambient from laser
self-heating; datasheet ±0.1 mm/°C ranging drift), the per-frame `error_status`/`error_code` health
flag, the DSS/binning/nb_step config readback, and the reference-SPAD channels. No firmware or wire
change — it reads bytes already in the file. `die_temp_c` is a raw u16 that reads as °C empirically.

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
afterwards — `voxel_size=0.005` roughly doubles detail over the 10 mm default. Three limits
worth knowing: the sensor samples ~36 mm between rays at 2 m, so below ~5 mm the extra
detail comes only from multi-view fusion (dense back-and-forth sweeping), never from one
view; blocks scale as 1/voxel², so halving the voxel produces ~4× the blocks; and — the
one that bites — **Open3D cannot extract a mesh from more than ~260k active blocks at all**
(BUG-053). ~~Halving the voxel wants ~4× the `block_count`~~: **raising `block_count` does
not help**, because the ceiling is the absolute block count, not the load factor (the same
273,521 blocks crash at 68.4% load in a 400k grid, on both CPU and CUDA, and they crash the
*process* — host segfault, CUDA illegal memory access). A room-sized capture at 5 mm simply
cannot be meshed on this Open3D build; that is why the Detailed preset default moved to
0.01 (BUG-054). `TsdfMap` now refuses to extract above 250,000 blocks and raises
`TsdfCapacityError` rather than making the call.
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

For a suspected tracking-collapse interval, set both `icp_trace_start_s` and
`icp_trace_end_s` (seconds relative to the first depth frame). The chosen mode then returns
an `icp_trace` containing every frame in that window: fitness, RMSE, the **exact** final ICP
correspondence count and source-point denominator, tracking state, reconstructed height, and
vertical step. Output is windowed, computation is not: frame-to-model ICP must still replay
the prefix before the requested window to build the map it registered against. Leaving both
arguments at their defaults adds no trace or retention cost.

`capture_magcheck` scores a magnetometer calibration against a capture it never saw — the
BUG-030 closing test. Read `verdict`, which is the worse of two deliberately different
measurements: `attitude.attitude_locked_pct` (|B| error left after detrending the room's
own slowly-varying field, a **lower** bound — an attitude held longer than the window is
absorbed into the trend) and `tilt_ramp.ratio` (|B| max/min across boresight-tilt bins,
detrend-free, so it catches exactly what the first one hides). `field` is
`magsweep.field_consistency`, correct for a stationary tumble but it under-rates a good
calibration on a walk. `compare=[...]` scores several fits against one capture.

`capture_heading` asks the different question that BUG-058 needed and `capture_magcheck`
structurally cannot answer: not "is the magnetometer calibrated" but "is the number labelled
heading actually a heading". It regresses `absolute_heading` on the quat's own boresight
bearing **and** its roll together — they differ only by the SFLP frame's arbitrary datum, so
the coefficients must be 1 and 0. On `NorthFacingRoll.bin` today's heading scores +0.016 and
the pre-BUG-058 one **−0.984**, on a capture `capture_magcheck` calls excellent (0.18%).
Read the per-axis `verdict`, never the bare coefficient: each is judged against a
block-bootstrap 95% interval — blocks, because the residual is yaw drift and room field that
wander over seconds, and treating that as white noise reported a real circuit's roll axis as
`bad` at 0.181 when the honest answer was "36° of roll and 5.8° of drift cannot resolve it".
Most captures certify one axis and leave the other `inconclusive`. It cannot see absolute
direction — both estimates share the quaternion and the calibration, so a rotated fit
(DT0103) moves them together; that is still ROADMAP DC-E's braced sweep.

`capture_profile_probe` is the measurement half of the ranging-profile contract in
`docs/superpowers/plans/completed/2026-07-31-high-framerate-and-manual-ranging-modes.md` —
`roomscan.profiles` says what a requested configuration should do, this says what a
recorded capture actually did. `fps_within_tolerance` applies the plan's own +/-2%
acceptance gate to the measured vs. requested rate; `stream_pairing` still assumes
coupled 1:1 emission (a seq-keyed set intersection in `host/tools/profile_probe.py`) and is a
**known lower bound now that Task 7 has shipped decoupled IMU/env draining** — unlike
`skew_check.py`'s `collect_frames()`, which Task 7 explicitly reworked to retain every send
sharing one frozen `seq` instead of last-write-wins, `profile_probe.py`'s `stream_pairing` was not
reworked, so its paired-percentage figure undercounts a decoupled capture's true sample count.
**UDP fragment health cannot be recovered from the capture file** — a datagram that lost a
fragment is dropped before the recorder ever sees it — so pass `udp_stats` from `rig_status()`'s
own metrics if that field matters; omitted, it reports `None` rather than a guess.

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

**fleet\_\*** — planners for the `issue-fleet` skill, which runs several subagent
workers across open issues in separate worktrees. Both are **read-only**: neither
creates a worktree, claims an issue, spawns an agent, or touches git. Those stay in
the orchestrator's own Bash calls, because everything in this server runs from
`paths.REPO` — always the *main* checkout, whichever worktree the caller believes it
is in.

| tool | notes |
|---|---|
| `fleet_plan(max_agents?, priorities?, exclude_areas?, include_unknown?, triage?)` | ranks open issues (priority, ×2 for prior work, bonus for gating others) and returns the highest-scoring batch whose file footprints do not collide. Read `notes`: soft conflicts and prose-inferred dependencies are surfaced for your judgement, never applied silently. Each selected issue carries a bounded `triage` digest — latest plan excerpt, latest comment + `kind`, `acceptance_hint`, `chars_elided` — **read that instead of running `gh issue view`**; this call already fetched every body and comment thread. Held issues are in `excluded`, vetoed outright, so no `operator_queue()` cross-check is needed to filter the batch |
| `fleet_budget(ceiling_pct?, observed_week_pct?, observed_block_pct?, limit_basis?, limit_tokens?, forecast_agents?, forecast_minutes?, source?, session_id?, project_dir?)` | current 5h block and **rolling** 168h window against a declared ceiling, plus an orchestrator-context check, folded into one verdict on the *projected* load of the **worst** constraint. **Ask the owner for their percentages and pass them** — without `observed_week_pct` the weekly figure is `None` and the verdict covers the 5h block only. `verdict` may be `rotate`/`rotate_hard` (hand off to a fresh session — not advice); `budget_verdict` preserves the original `go`/`reduce`/`stop`/`unknown` and `binding_constraint` says which of budget or context drove it. `rotation` carries weighted-token **deltas**, never percentages; pin `session_id` after the first call. Read `seven_day.by_seat` — delegating and rotating both move spend between the orchestrator and worker seats, so neither shows up in a total. Read `binding_window`, `limit_basis`, `seven_day.pct_basis` and `coverage.includes_subagents` before trusting anything |
| `operator_page(out?, repo?)` | the same queue as a **plan** rather than a list, written as a self-contained HTML page (default `host/src/roomscan/static/operator.html`, so it is reachable at `/static/operator.html` beside the app the runbooks already tell the owner to open). Scrapes every runbook and works out what the queue actually costs: aliases that say "covered by the request on #NNN" become `riders` and appear nowhere else, near-duplicate setup steps cluster so a shared power-up happens once per sitting, and issues needing the same **venue** (one metal-free spot, one blank wall) group into one setup — that last one catches savings step-text clustering cannot see, since two runbooks can share a venue and no wording at all. `missing` lists holds with no runbook, the same dead end `operator_queue().problems` reports. Regenerate after posting or revising any runbook, or after any `needs/*` label changes on any issue (including a close) — the page is a snapshot, and a label change with no runbook edit is easy to miss (#183, 2026-08-14; `status-sync` checklist item 9 is the backstop) |
| `operator_queue(detailed?)` | open issues held for an owner action (`needs/operator`), with subtype, age, status labels and the parsed footer of the latest `## 🔧 Operator Request` comment — the artifact expected and the gate that scores it. Read `problems`: it names held issues whose runbook is missing or unparseable, which the issue list cannot show you. `pending[i]["parked"]` marks the one legitimate exception — a `priority/later` hold deliberately labelled before its runbook was written, reported per-issue and kept **out** of `problems` (#175); `priority/now`/`priority/next` with no runbook, a malformed footer, and a comment-read failure are never excused by `priority/later` and remain problems. Input to the `operator-request` skill's batch mode. `detailed=False` skips the per-issue comment fetch (so `parked` stays `False` — that path never reads comments) |
| `tracker_lint(repo?)` | mechanical lint of open-issue labels against the tracker invariants (#179): `needs/operator` requires a `needs/*` subtype **and** a `status/fix-unverified`\|`status/blocked` hold, every open issue carries exactly one `priority/now\|next\|later` tier and exactly one of `bug`/`work-item`/`data-collection`, plus an advisory check that an `area/*` label is present. Returns `violations` by rule with issue numbers; `ok` is `True` when no **hard** violations — advisory rule-4 findings land in `advisory_count` without failing `ok`. CLI front end: `host/tools/tracker_lint.py --json` |

Three things `fleet_plan` encodes that a prose rubric kept getting wrong:

- **The median issue names exactly one file.** Intersecting raw footprints therefore
  finds almost no conflicts — a check with no power. Seeds are expanded through a
  co-edit graph from `git log --name-only` (`web.py` and `test_web.py` co-occur in 37
  commits) before conflict is computed, and `conflicts()` distinguishes a **hard**
  overlap on stated files from a **soft** one on inferred paths.
- **The collisions that bite are not files.** One Playwright browser is shared across
  every `ui_*` call, and port 8000, the rig and the GPU are each a singleton — so
  `resources` is modelled separately from footprints, with a per-wave cap of one each.
- **Prior work and extractable paths are the same signal**, so rewarding prior work
  while deferring unknown footprints would starve the cold tail permanently. One
  explicit exploration slot per wave goes to the highest-scoring path-less issue, and
  says so in `notes`.

`fleet_budget` is an estimate and is built to admit it: no quota API is reachable from
an agent here (the `claude` CLI has no usage subcommand; the real figures live in CLI
process memory and are never written to disk). It prefers `ccusage` and falls back to
reading `~/.claude/projects/**` directly. Two facts the fallback reader had to encode,
both measured: worker spend lives one level deeper than the main transcripts
(`<session>/subagents/agent-*.jsonl`, invisible to a flat `projects/*/*.jsonl` glob),
and the store repeats records, so summing without deduplicating on
`(message.id, requestId)` overcounts by ~80%. With both fixed, a native read
reproduced `ccusage blocks --json` exactly on entry count and on three of its four
token fields.

**It is a relative meter, not an absolute one (#172, 2026-08-12).** Calibrating it
against the owner's own reported figures is what established this: two readings of the
same 5h window on the same day implied allowances **1.6x apart** (96.2M read as 85% by
the owner → 113M; 40.9M read as 22% → 186M), because the weighted token this module
computes is its own invention (`CACHE_READ_WEIGHT`, `OUTPUT_WEIGHT`, a per-model table)
and does not track however the real limit counts. Change the model/cache/output mix, as
one wave does versus another, and the conversion moves. So percentages are **anchored**
on `observed_week_pct` / `observed_block_pct` rather than derived, and the weekly figure
comes back `None` — with a loud note — when there is no anchor, instead of guessing.

What *is* reliable is the delta, so the forecast projects forward from the owner's
figure at a measured **~25M weighted tokens per weekly point** (the conservative bound;
a 3-worker Sonnet wave with review, one full suite and browser checks ran ~40M, a
2-worker wave ~10.5M). Two things this replaced, both wrong since the module was
written: the weekly denominator was `peak_block * 168/5` — 33.6 back-to-back peak
blocks, i.e. the assumption that no weekly cap exists — which reported 12.4% against an
owner-read 72%; and `decide()` took only the 5h numbers, so an entire multi-wave run
returned `go` at every check while the weekly ceiling was nearly spent. The 5h window
resets every five hours and is nearly always cheap right afterwards; the weekly one is
what actually runs out, which is why `binding_window` now names the window that drove
the verdict — a `stop` on the 5h clears in hours, a `stop` on the weekly does not.

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
| `rig_command` | **`ok` carrying the *previous* command's late ACK** (#171, fixed 2026-08-12) | now matched on `resolve_command()`'s label; a non-matching ACK seen while waiting is surfaced as `unmatched_acks`, never accepted as the answer |

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

**`fleet_run.py` is the structural exclusion, not a judgement call.** It chains
orchestrator sessions across a rotating fleet run, so it has to outlive every session it
spawns — and an MCP tool runs *inside* the session being rotated, dying with it. The
lifetime is the argument: anything that must survive a session boundary is a CLI the owner
runs, never a tool the session calls. It is documented in the `issue-fleet` skill, and its
per-link record lands in `.fleet/<run-id>.chain.json`.

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
