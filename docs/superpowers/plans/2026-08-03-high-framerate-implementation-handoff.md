# Handoff — High Frame-Rate & Manual Ranging Modes implementation (2026-08-04, rev 5)

**Plan:** `docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md` (status
header there summarizes; this doc is the authoritative resume state, superseding rev 3 of this file).
**Working model:** orchestrator + Sonnet subagents (owner directive: preserve usage limits); agents
edit + test, never commit — the orchestrator verifies (focused tests + ruff on touched files) and
commits per task. Continue that pattern.

**Tasks 1–11 are ALL DONE.** **Task 12 (final: validation/docs/land/retro) is IN PROGRESS** as of
this revision — see the new Task 12 section below. (Task 11 had been delegated to Codex; it had not
landed by 2026-08-04, so a Claude session implemented it — see the Task 11 section.)

## DSS adjudication — RESOLVED, Position A upheld, HIGH confidence. Task 12 is UNBLOCKED.

Rev 3 recorded the DSS finding as contested: a concurrent Codex session working the same
AN6522/ProfileTuning comparison was reported to have found the vendored driver's CSI-2 path supports
DSS-off with full per-zone data, which would have meant the `91b9eac` 46 Hz ceiling amendment was
premature. **That contest is now resolved in favor of the original finding (Position A):**

1. `platform_start_csi_pipe`/`platform_stop_csi_pipe` are `return -1` stubs, never called anywhere
   in the vendored tree.
2. The board schematic has **zero** CSI/MIPI occurrences — only Arduino/Zio headers are wired.
3. The transform library's only input shapes are four full-per-zone 3DMD layouts; no DSS-off-specific
   shape exists on any interface the driver or transform library expose.
4. AN6522 ties >100 fps explicitly to "using a MIPI interface" and never mentions DSS as the enabler.
5. Task 4/5 already toggled the real DSS register on hardware directly — neither frame size nor the
   frame-rate ceiling moved.
6. The "Codex found CSI-2 supports DSS-off" framing traced to a **transcript-summary
   mischaracterization** — Codex's own uncommitted notes conclude the same as Position A verbatim
   ("cannot demonstrate 66 fps full-map I3C… measured floor stands").

**The `91b9eac` ceiling amendment (High Frame-Rate preset = 46 Hz, hardware-confirmed 45.86–45.56 fps)
stands as final. Do not reopen this question** absent new hardware evidence — the six points above
are independent and all point the same direction.

## Landed on `main` (each verified at commit time)

| Commit | What |
|---|---|
| `d2c4148` | Task 1 — profiles contract, `profile_probe` + `capture_profile_probe` MCP, spec reconciled vs DS14879 |
| `2b8a9ee` | Task 2 — protocol v2: cmds 8–12, typed ACKs, C cross-check, v1 replay pinned |
| `414adaa` | Task 3 — typed `CommandClient` + `roomscan-ctl` (incl. `imu-rate` pair); enum mapping BY NAME |
| `7598fde` | Task 4 — firmware atomic profile apply, readback ACKs, DSS setter; HW gate passed |
| `b10f44d` | Task 5 — autonomous sync; BUG-072 fixed; honest >30 Hz shortfall measured (stop point) |
| `7e560a2` | BUG-073/074 filed |
| `91b9eac` | Ceiling amendment — HFR preset 46 Hz everywhere; `expected_delivered_fps` quantization model. |
| `5c23270` | Task 9 — SLAM ingest split remnants: `transform_fps`/`browser_fps`, invariant tests, 90 Hz CUDA bench (p99 9.05 ms < 11.1) |
| `8cc9239` | Task 10 — web ranging UI: `RangingState`, set_profile/set_manual_params/set_imu_env_rate, I3C bar, CDC warning |
| `896c7a8` | Model refinement — I3C 11.8736 ms (10 Mbps effective), >8 ms floor line `16.5+exposure`, DSS spec section, ST's exact power model (5/5 anchors, 0.01%) |
| `5c90da6` | Task 6 firmware+host — applied-period TX pacing, EVENT 7 `TX_QUEUE_STATS`, CDC isolation, ACK routing to originating transport |
| `10966c5` | Task 6 — EVENT 7 TX_QUEUE_STATS golden vector + C cross-check (closes the protocol-change checklist gap from `5c90da6`) |
| `91a5dfe` | Task 6 — firmware TX queue telemetry surfaced in web authoritative state (steps 4–5; step 4 needed no new code, `8cc9239`'s `_transport_kind` already did it) |
| `56ee9ba` | fix(tools): `analyze_capture` accepts protocol v2 (BUG-076) |
| — | **Task 6 hardware gate: run and PASSED 2026-08-04** — bridge-measured, not wired; see below |
| `3f4b307` | Task 7 — shared `rs_lsm_service_tick()`, firmware cmds 11/12, decoupled TIM2-paced drains, skew_check N:1 grouping + 4 tests; **plus BUG-074 determination and fix** (real defect: wake-path hardware failures acked nothing before `handle_error()`, not the hypothesized shadow-write bug) |
| — | **Task 7 hardware gate: run, initially failed on BUG-077, re-run and PASSED 2026-08-04** — see below |
| `d732f3a` | Task 8 — applied IMU/env rate scalar flows web → worker → `Mapper.set_imu_rate_hz`; `baro_tau_frames` pinned in real seconds; `ImuFusion` explicit reference-rate input; replay uses each stream's own measured cadence |
| `17a27d8` | BUG-077 fix — abort off-cycle IMU drains when FRAME_READY is pending (landed same session as Task 7, cited separately here per the task instruction) |

Suite progression this session: 1817 → 1828 → 1832 → 1858 passed, always exactly 1 skip. **Final:
1858 passed / 1 skipped.** 2+ skips means the wrong interpreter (cwd must be `host/` with
`host/.venv/bin/python`).

## Task 6 — DONE, hardware gate PASSED 2026-08-04, bridge-measured (wired gate still outstanding)

Firmware built and flashed: `.bin` 165,259 B, text 152,020 / data 13,235 / bss 177,328. Rig
untethered, on the FileHub Wi-Fi bridge (172.17.2.58 that session — resolve via mDNS, it moves).

Four 60 s operating points, fps derived from `t_us` cadence (device-clock intervals):

| Op point | Applied readback | fps (bridge) | CRC | whole-group loss |
|---|---|---|---|---|
| (a) 30 Hz Room Mapping | ambient/30fps/6ms/ULP | 29.88 | 0 | 0 |
| (b) 46 Hz HFR preset | precision/46fps/4ms/regular | 45.56 | 0 | 0 |
| (c) 50 Hz manual, 2 ms | precision/50fps/2ms/regular | 33.52 (BUG-075 bimodal) | 0 | 0 |
| (d) 90 Hz oversubscribed, 2 ms | precision/90fps/2ms/regular | 44.79 | 0 | 0 |

Captures: `captures/web_20260804_065504.bin` (a), `070325.bin` (b), `070618.bin` (c), `070811.bin` (d).

**ZERO-DROP GATE: PASS.** `fw_tx_enqueue_drops`/`fw_tx_stack_stalls` were 0 at start AND end of all
four runs (cumulative-since-boot counters — zero-at-start-and-end is the correct proof, not merely
zero-at-end); verified as real counters incremented at genuine failure sites in
`firmware/scanner-stream/Src/ethernet_transport.c`. `fw_tx_emitted_bytes` climbed monotonically every
poll (3.0M → 367M), proving the counters are actually written.

**CDC ISOLATION: untested, with reason.** This host cannot use ST-Link VCOM unprivileged, and the rig
runs untethered with no USB cable attached. Task 6's CDC-isolation path (DATA to Ethernet-only above
60 fps, ACK routed to originating transport) is code-complete and unit-tested but **unverified on
hardware**. Do not upgrade this to "passed" in any later doc.

**Wired-Ethernet release gate: still outstanding** — the plan's step 6 target list (60/90/100 Hz
wired) predates the `91b9eac` ceiling amendment; the operating points actually run
(30/46/50/90-oversubscribed) supersede that list rather than leaving it unmet, but no wired run has
happened this program.

## Task 7 — DONE (implementation `3f4b307` + gate + BUG-077 fix `17a27d8`)

Shared `rs_lsm_service_tick()` extracted from the idle loop and reused by both idle and active paths;
firmware cmds 11 (`SET_IMU_ENV_RATE`)/12 (`GET_IMU_ENV_RATE`); decoupled mode paces drains off TIM2
against a software-tracked next-due timestamp, sends 9/10/11 with frozen `seq`, skips stream 13 on
non-coincident iterations; `skew_check.py`'s `collect_frames()` reworked from a `seq`-keyed dict (which
silently overwrote multiple decoupled sends sharing one frozen `seq`) to a grouping that retains all of
them, plus 4 new tests.

**Gate results (bridge-measured):** coupled-mode regression PASS (29.89 fps, intervals matched Task
6's baseline to 0.01 ms, `n_imu_raw_sends=1`, stream 13 every frame); idle regression PASS (exactly
18.182 Hz IMU/IMUraw/Env with ToF parked); decoupled independence PASS (30 Hz held across Room
Mapping/Precision/HFR; 90 Hz → 85.2 Hz with stream 10 sub-sampled to 56.4 Hz as the readback warns);
stream-13 discipline PASS (100% coverage at 90/30, ~3 sends per ToF frame, frozen `seq` verified);
cmds 11/12 live with truthful readback; BUG-074 wake path sanity-passed via a natural auto-idle cycle
(full fault-injection retest still outstanding).

**Item 5 ("no measurable perturbation of ToF frame cadence") initially FAILED → BUG-077:** decoupled
drains doubled ToF intervals under precision ranging mode (Precision/30: 4.91%, HFR/30: 14.94%,
Ambient clean) — root cause: the off-cycle drain ran before checking FRAME_READY, and each FIFO word
is a blocking I3C transaction on the shared bus. Asymmetry explained by period-minus-exposure margin
(ambient's 27.3 ms period absorbs it; HFR's 17.7 ms cannot). **Fixed** by returning on
FRAME_READY-first with a per-word abortable drain. Re-gate: Precision/30 0.09%, HFR/30 0.88%, all
within ~0.3 points of coupled controls.

**TRADE (measured, honest, not fully eliminated):** decoupled-90 IMU/env on Room Mapping dropped
85.2 → 77.2 Hz (~9%); 30 Hz combos unaffected. Fix captures
`web_20260804_085237.bin`–`090419.bin`.

## Task 8 — DONE (`d732f3a`, host-only, no hardware gate required)

Applied IMU/env rate scalar flows web → worker → `Mapper.set_imu_rate_hz`. `baro_tau_frames` pinned
in real seconds (900 @ 30 Hz / 2700 @ 90 Hz — the same 30-second time constant regardless of applied
rate). `ImuFusion` takes an explicit `quat_ref_rate_hz` input (previously an inert derivation constant
with zero runtime effect); the rate-derived yaw crossover term scales as
`TAU_YAW_S * sqrt(30 / rate)`; coupled-30 Hz behavior is bit-identical to before. Remote/service SLAM
backends carry the same scalar. Replay falls back to stream 9's own measured cadence, never an
assumed 30 Hz.

## Task 11 — DONE (host-only; live round trips run on the rig)

`rig_profile()` / `rig_imu_env_rate()` (`mcp_server/tools_rig.py`) and the offline
`profile_estimate()` (`tools_data.py`). Both rig tools verify by **effect**: they wait for their own
half of the `ranging` broadcast to go pending and then settle onto the requested config. That
message also rides the ~4 Hz metrics tick, so neither the cached copy nor a merely newly-arrived one
counts — and the two halves are independent pending commands sharing one message, so each waiter
reads only its own half (an IMU-rate error must not fail a profile change, and an IMU-rate settle
must not confirm one). One further staleness guard: a tick can slip between the send and the server
acting on it carrying the *previous* command's error, so an unchanged error string is ignored until
the pending echo proves our command started.

`ok=false` for busy / replay / host-side validation failure / device error / unsupported CDC rate /
applied mismatch, and the result still reports what the device actually has applied. Two deliberate
refusals with escape hatches: >60 fps over CDC needs `force=True` (Task 12 step 5 needs to send it),
and `require_full_env=True` turns the >60 Hz env sub-sampling warning into a pre-send error.
`rig_status()` now carries the ranging state (`ranging_profile`, `ranging_measured_fps`,
`imu_env_rate_hz`, `imu_env_coupled`, plus the whole `ranging` message).

**Single-owner cleanup:** the JSON vocabulary (`PROFILE_ID_TO_STR` etc.) and `estimate_to_json()`
moved out of `web.py` into `roomscan.profiles`, which `web.py` now aliases — so the browser and an
agent cannot be told different things about one configuration (BUG-076's lesson applied before it
could happen again).

**Live verification on the rig (2026-08-04, server on :8000, device untethered):** `high_framerate`
applied and read back 46/4 ms/precision/regular, measured **45.35 fps**; restored to `room_mapping`
(30/6 ms/ambient/ulp), measured **29.9 fps**. IMU/env decoupled to **30 Hz** with truthful readback
and restored to coupled. The first attempt returned `SET_RANGING_PROFILE -> BUSY` because the rig was
idle-parked — surfaced as `ok=false` with the device's own error, which is the failure path working.
Rig left in its documented end state. 32 new tests (suite 1889 passed / 1 skipped).

**Not done here:** the browser-side VISUAL pass and every hardware sweep stay Task 12's; these tools
are what Task 12 should drive it with.

## Task 12 — IN PROGRESS (2026-08-04): end-to-end validation, docs, landing, retro

**Host suite:** 1895 passed / 1 skipped (the 1 skip is the expected Windows-only `test_logfilter`
case — 2+ skips means the wrong interpreter, see the memory note on this). **ruff:** 4 findings, all
pre-existing and in `host/tools/`, unrelated to this feature (`headless_doctor.py` unused `os`
import; `query_mdns.py` unused import + f-string-without-placeholder; `test_send.py` multi-import
line). **Firmware:** production raw-only Debug build is clean — `.bin` 166,107 B (text 152,868 /
data 13,235 / bss 175,288).

**Finding (plan-accuracy gap, not a blocker):** the plan's step 2 assumes a build-time knob selects
the onboard-transform config; no such CMake knob exists. `CONF_TRANSFORM_ONBOARD` is an unguarded
`#define ... (0)` in `firmware/scanner-stream/Src/vl53l9_app.c` (~line 36) with no
`target_compile_definitions`/preset wiring in `CMakeLists.txt`, so a `-D` override on the command
line is a silent no-op — only the raw-only config is buildable without a source edit. Filed as
BUG-078.

**De-scoped by owner directive (2026-08-04):** *"We will never use ethernet or CDC for data
transfer"* — read as never a physically wired link in production. The plan's **wired-Ethernet
release gate** (Task 6 step 6) and **CDC-isolation hardware proof** (Task 6 steps 3/4) are
DE-SCOPED, not outstanding. The Wi-Fi bridge IS the transport and it's proven: four 60 s Task 6
captures ran 0 CRC / 0 seq gaps / 0 incomplete frames / 0 firmware TX drops at
30/46/50/90-oversubscribed Hz, peaking ~9.9 Mbit/s against the 100 Mbit/s port — ~10x headroom, so
bandwidth is definitively not the bottleneck and a wired test would prove nothing new.

**Still genuinely outstanding** (not failures, tracked as open bugs): BUG-074's fault-injection
retest, BUG-075's root cause (50 Hz/2 ms bimodal cadence), BUG-073 (reconfig <2 s can drop an ACK).

**Live rig sweep** (all profile/manual/IMU-rate combinations from plan step 3) **and the browser-UI
visual pass** (plan step 6): Live rig sweep (over the Wi-Fi bridge; device restored to boot default
afterward): all four presets + manual 60/61/90/100 fps verified — Room Mapping 29.8 fps, Precision
29.9 fps (precision/30/10 ms/ulp), High Frame-Rate 45.4 fps (46 preset, precision/46/4 ms/regular),
and every over-ceiling manual request ACKed but delivered a truthful period-multiple (60→29.9,
61→30.4, 90→44.8, 100→49.2 fps), each within ~1.5% of the model's `expected_delivered_fps`; 0 CRC
across all; DSS auto-disables at fps≥61; I3C airtime reaches 100% at 90/100. IMU/env decoupling
verified: 30 Hz and 90 Hz both apply, 90 Hz warns and sub-samples env to 53.5 Hz while quat/raw run
76.7 Hz, stream-13 coincidence per design (39% at 30/30 unlocked, 100% at 90/30); restored to
coupled. The 30 Hz default matches the documented baseline `captures/web_20260803_121735.bin` within
±2% — no regression. 3–4 initial SET_* commands returned SENSOR_ERROR and cleared on a single retry
(BUG-073). Browser-UI visual pass: four-way selector, preset apply with one-way (non-optimistic)
state flow, manual-panel disclosure, invalid-combo rejection (validation error shown, no bad command
sent), the "ToF bus airtime" I3C bar with three-state color escalation (blue→amber→red/100%), the
>60 fps quantization-consequence warning, the IMU/env poll-rate control with its >60 Hz
env-sub-sampling warning, and title-tooltip coverage on every new control all PASS; server-side
second-client sync-on-connect verified. Two checks were tool-limited and honestly not performed: a
truly rendered second tab, and a narrow-viewport resize (the ui_* screenshot tool would not change
the viewport; narrow-width has a prior verification in `docs/web-ui-testing.md`). **One real defect
found by the pass and filed as BUG-079:** the Manual fps/exposure number/slider widgets cannot apply
a changed value — `web.py`'s 250 ms periodic `ranging` re-broadcast outruns `controls.js`'s 300 ms
`MANUAL_DEBOUNCE_MS` and reverts the field before it sends (presets, the IMU/env control, and
`set_manual_params` over `/ws` are all unaffected).

**Docs status-sync:** ROADMAP.md, `docs/protocol.md` (stale cmd 8-12 caveat rows), `docs/web-protocol.md`
(stale Task-7-not-landed note on `set_imu_env_rate`), `docs/mcp-server.md` (`stream_pairing` N:1
caveat tense/accuracy), `BUGS.md` (BUG-078 added) all updated in the landing commit(s). `CLAUDE.md`
needs no change — no existing statement there is now wrong (see the CLAUDE.md section of the docs
plan for the reasoning).

## Load-bearing findings (carry forward; do not re-derive)

- **The frame-rate ceiling is sensor-intrinsic.** Floors 20.0 / 21.739 / 23.529 ms at ≤2 / 4 / 8 ms
  exposure; shorter applied periods deliver integer multiples (2×/3×) with the applied-period
  readback exact. HFR preset = 46 Hz / 4 ms / Precision / Regular / DSS on, hardware-confirmed
  45.86–45.56 fps (two independent bridge runs), 0 frame loss by `t_us` cadence. **BUG-075 shows this
  "clean integer multiple" story is not the whole picture at 50 Hz/2 ms** — treat the ceiling itself
  as solid but the quantization model as incomplete.
- **DSS: RESOLVED, Position A upheld, HIGH confidence** (see the dedicated section above). Our
  `STANDBY_DSS_MODE` toggle is ranging-quality only, the vendored I3C driver always fetches the DSS
  LUT, ST's 100+ fps "DSS Disable" mode is unimplemented in the vendored driver and incompatible with
  our 14,842 B transform. Not contested; do not reopen.
- **Off-cycle firmware work must yield to FRAME_READY, or it steals the shared bus (BUG-077).** Any
  service tick that shares an I3C bus with the primary acquisition trigger — not just the IMU/env
  drain — must check for a pending FRAME_READY edge *before* issuing another blocking bus transaction,
  and be structured so each unit of work (here, one FIFO word) is independently abortable. The
  available headroom for such off-cycle work is `period − exposure` (the margin BUG-077's asymmetry
  traced to), not the nominal period — a short-period/short-exposure ranging mode can have near-zero
  slack even though its overall frame rate looks unremarkable.
- **ProfileTuning.exe** (`references/software/53L9A1/`, support-gated ST tool, untracked, keep) is
  PyInstaller; decompiled model at `/tmp/pt_extract/ProfileTuning_decompiled.py` (regenerable with
  pyinstxtractor-ng + decompyle3 in a /tmp venv). Its timing model reproduces the owner's tool runs
  bit-exactly but is WRONG about our hardware (flat 26.9 ms floor vs measured 20–23.5) — measured
  brackets outrank it. Its power model is exact and now ships in `profiles.py` (`ambient_lux` param).
- **profiles.py ↔ wire enums differ in numbering; map by NAME** (`control.py`
  `_manual_params_to_wire`). Cmds 9/10 ACK = 16-byte readback shape regardless of result.
- **TX queue telemetry (Task 6) is real, not decorative** — verified against genuine failure sites in
  `ethernet_transport.c`, and `fw_tx_emitted_bytes` climbing monotonically under load is the proof a
  counter is actually wired, not just present. Reuse this pattern for any future firmware counter.
- **A second, independently-maintained parser of the same wire format drifts silently** (BUG-076) —
  extend the golden-vector/cross-check discipline `protocol.py` already has to any other code that
  parses the wire format.
- **Rig etiquette:** resolve the device via mDNS `roomscanner.local` EVERY time (IP changes across
  bridge resets). `rig_down` before driving the device from /tmp scripts; `rig_up` same port +
  `rig_status` healthy after. Restart the server after host-code/protocol changes (stale-process
  trap). FileHub bridge recovery is OWNER-ONLY. ≥2 s settle between reconfigs (BUG-073).

## Open bugs from this push

- **BUG-073** (open): reconfig <2 s after a prior one can produce no ACK/no BUSY.
- **BUG-074** (fixed, `3f4b307`): SET_STANDBY wake path acked nothing before `handle_error()` on
  hardware failure (not the originally hypothesized shadow-write bug). Sanity-passed via a natural
  auto-idle cycle; **a full fault-injection retest is still outstanding.**
- **BUG-075** (open): 50 Hz/2 ms manual request delivers a near-even bimodal 20/40 ms alternation
  instead of a clean floor; mechanism unexplained; not a TX-pacing defect. **The
  `expected_delivered_fps` model is unreliable for short-period/short-exposure combos until this is
  root-caused.**
- **BUG-076** (fixed, `56ee9ba`): `analyze_capture` was blind to protocol v2 since `2b8a9ee`.
- **BUG-077** (fixed, `17a27d8`): off-cycle IMU/env drain didn't yield to FRAME_READY, doubling ToF
  intervals under short-margin ranging modes. See the Task 7 section and the load-bearing finding
  above.
- BUG-072 fixed (`b10f44d`).

## Remaining work

1. **Task 11 — DONE** (`1663556`; see its section above). Codex's separate **profile-tuning
   comparison** also landed (`a418131`): `host/tools/profile_tuning.py` + the `profile_tuning` MCP
   tool, `docs/vl53l9cx-datasheet-notes.md` §7, and AN6522 in-tree. Two integration defects were
   fixed on the way in — `ambient_lux` was a dead knob (parsed by the CLI, then `del`'d, because no
   estimate could carry it; now threaded through `estimate_profile`/`_preset`/`_manual`), and
   `roomscanner_full_map` was `dataclasses.asdict`, emitting raw enum ints and silently dropping
   `ok`; it now uses `profiles.estimate_to_json` like the rest of the surface. ST's 89 MB
   support-gated package stays untracked, now with a `.gitignore` entry recording why.
2. **Task 12** — end-to-end validation (incl. Task 10's deferred VISUAL pass: four profile buttons,
   invalid manual combos, debounce/pending, bus-bar colors, CDC warning, second-tab sync, narrow-width
   dock-band check), re-run the 30 Hz baseline vs `captures/web_20260803_121735.bin`, full docs
   status-sync (incl. `protocol.md`'s stale cmd-8-12 caveat rows), land, then **milestone-retro**
   (mandatory). DSS is resolved so it no longer blocks this task.
3. **De-scoped by owner directive (2026-08-04): "we will never use ethernet or CDC for [a wired]
   data transfer [link]."** The **wired-Ethernet release gate** (Task 6's bridge-only numbers
   needing a wired confirmation run) and the **CDC isolation test** (needs privileged VCOM or a
   tethered session) are therefore DE-SCOPED, not outstanding — the Wi-Fi bridge is the production
   transport and its four Task 6 operating points already proved 0 CRC / 0 seq gaps / 0 incomplete
   frames / 0 firmware TX drops at 30/46/50/90-oversubscribed Hz, peaking ~9.9 Mbit/s against the
   100 Mbit/s port (~10x headroom) — a wired run would prove nothing new about the bottleneck.
   Genuinely still outstanding: **BUG-074 fault-injection retest**, **BUG-075 root cause** (50 Hz/
   2 ms bimodal cadence), and **BUG-073** (reconfig <2 s can drop an ACK). Restart the MCP server
   before trusting `capture_analyze`/`capture_profile_probe` output from it if it still predates
   BUG-076's fix and Task 1's tools — the `roomscan-web` server WAS restarted this session onto
   current code (new PID; the prior instance was stale from 08:13 and predated Tasks 8/11/BUG-077),
   and this session's MCP server is fresh (its `capture_profile_probe`/`capture_analyze` tools were
   exercised successfully throughout the Task 12 sweep), so no further restart is needed.

## Rig end state (2026-08-04, end of this session)

Device flashed with `17a27d8` firmware, streaming. Boot default Room Mapping, coupled IMU/env mode,
idle auto-standby enabled (`idle_enabled: true`). `roomscan-web` server up on :8000, `rig_status`
healthy. Device reachable at 172.17.2.58 this session — **re-resolve via mDNS
(`roomscanner.local`) at the start of the next session**, it moves across bridge resets.

## Working model note (unchanged)

Orchestrator + Sonnet subagents (owner directive: preserve usage limits). Agents edit + test, never
commit — the orchestrator verifies (focused tests + ruff on touched files) and commits per task.
Continue that pattern for Task 12.
