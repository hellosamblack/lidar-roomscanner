# Handoff — High Frame-Rate & Manual Ranging Modes implementation (2026-08-04, rev 3)

**Plan:** `docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md` (status
header there summarizes; this doc is the authoritative resume state, superseding rev 2 of this file).
**Working model:** orchestrator + Sonnet subagents (owner directive: preserve usage limits); agents
edit + test, never commit — the orchestrator verifies (focused tests + ruff on touched files) and
commits per task. Continue that pattern.

**Task 6 is now DONE (code + hardware gate).** Resume starts at **Task 7**.

## ⚠ BLOCKING FOR TASK 12 — the "DSS story (settled)" finding below is now CONTESTED

Rev 2 of this doc (and `ROADMAP.md`) recorded the DSS finding as settled: our `STANDBY_DSS_MODE`
toggle is ranging-quality only, and ST's 100+ fps "DSS Disable" mode is unimplemented in the vendored
I3C driver and incompatible with our 14,842-byte transform. **A concurrent Codex session working the
same AN6522/ProfileTuning comparison found the vendored driver's CSI-2 path DOES support DSS-off with
full per-zone data.** If that holds up, the `91b9eac` ceiling amendment (HFR preset = 46 Hz
everywhere, "DSS off isn't reachable on our path") is wrong, and the original 90/100 Hz gate in the
plan may be reachable after all.

**The owner's decision:** finish Task 6 first (now done — see below), then **adjudicate DSS before
Task 12 writes any ceiling number into ROADMAP.md.** Do not treat the 46 Hz ceiling, the DSS section
of the spec, or any of this doc's "settled" language about DSS as final until that adjudication
happens. This is a hard blocker on Task 12, not a nice-to-have follow-up.

## Landed on `main` (each verified at commit time)

| Commit | What |
|---|---|
| `d2c4148` | Task 1 — profiles contract, `profile_probe` + `capture_profile_probe` MCP, spec reconciled vs DS14879 |
| `2b8a9ee` | Task 2 — protocol v2: cmds 8–12, typed ACKs, C cross-check, v1 replay pinned |
| `414adaa` | Task 3 — typed `CommandClient` + `roomscan-ctl` (incl. `imu-rate` pair); enum mapping BY NAME |
| `7598fde` | Task 4 — firmware atomic profile apply, readback ACKs, DSS setter; HW gate passed |
| `b10f44d` | Task 5 — autonomous sync; BUG-072 fixed; honest >30 Hz shortfall measured (stop point) |
| `7e560a2` | BUG-073/074 filed |
| `91b9eac` | Ceiling amendment — HFR preset 46 Hz everywhere; `expected_delivered_fps` quantization model. **⚠ Now contested — see above.** |
| `5c23270` | Task 9 — SLAM ingest split remnants: `transform_fps`/`browser_fps`, invariant tests, 90 Hz CUDA bench (p99 9.05 ms < 11.1) |
| `8cc9239` | Task 10 — web ranging UI: `RangingState`, set_profile/set_manual_params/set_imu_env_rate, I3C bar, CDC warning |
| `896c7a8` | Model refinement — I3C 11.8736 ms (10 Mbps effective), >8 ms floor line `16.5+exposure`, DSS spec section, ST's exact power model (5/5 anchors, 0.01%). **DSS spec section also now contested.** |
| `5c90da6` | Task 6 firmware+host — applied-period TX pacing, EVENT 7 `TX_QUEUE_STATS`, CDC isolation, ACK routing to originating transport |
| `10966c5` | Task 6 — EVENT 7 TX_QUEUE_STATS golden vector + C cross-check (closes the protocol-change checklist gap from `5c90da6`) |
| `91a5dfe` | Task 6 — firmware TX queue telemetry surfaced in web authoritative state (steps 4–5; step 4 needed no new code, `8cc9239`'s `_transport_kind` already did it) |
| `56ee9ba` | fix(tools): `analyze_capture` accepts protocol v2 (BUG-076) |
| — | **Task 6 hardware gate: run and PASSED 2026-08-04** — bridge-measured, not wired; see below |

Suite at handoff: last known full run 1791 passed / 1 skip (pre-`896c7a8`/`5c90da6`); those and every
commit since ran green on focused suites at commit time (test_profiles 140, test_sources 38,
protocol+crosscheck 67, plus `test_web`/`test_control` for the Task 6 telemetry work). **A fresh
FULL-suite run is still the first thing the next resuming session should do** — it has not been
re-run in full since `896c7a8`. Exactly 1 skip expected; 2+ = wrong interpreter.

## Task 6 — DONE, with two caveats (2026-08-04)

Firmware built and flashed: `.bin` 165,259 B, text 152,020 / data 13,235 / bss 177,328 — this was the
**first time Task 6's firmware (built atop `5c90da6`) reached hardware**, and no previous binary was
retained, so these are absolute figures, not deltas. Boot health was confirmed **indirectly**
(streaming resumed within seconds of flashing); nobody had eyes on the LD1/LD2/LD3 boot-progress LEDs
this pass.

**Rig state:** untethered, on the FileHub Wi-Fi bridge (172.17.2.58 this session — resolve via mDNS,
it moves). **Caveat 1 — every number below is bridge-measured, not wired.** The plan's release gate
is wired Ethernet; no wired link was available, so the wired gate remains **outstanding**, not
satisfied. Do not let bridge fragment loss read as a sensor or pacer limit — it tracked a UDP
`frags_lost` counter climbing at ~1 fragment/70 s independent of fps, i.e. link-layer loss.

Four 60 s operating points, fps derived from `t_us` cadence (device-clock intervals, independently
re-verified from the raw capture bytes by the orchestrator — treat these as confirmed):

| Op point | Applied readback | fps (bridge) | interval median/p05/p95 (ms) | interval modes (ms:count) | CRC | RAW loss | whole-group loss |
|---|---|---|---|---|---|---|---|
| (a) 30 Hz Room Mapping | ambient/30fps/6ms/ULP | 29.88 | 33.45 / 33.44 / 33.46 | 33.5:4727 | 0 | 2/4740 (0.042%) | 0 |
| (b) 46 Hz HFR preset | precision/46fps/4ms/regular | 45.56 | 21.81 / 21.80 / 21.82 | 22.0:3573, 43.5:12 | 0 | 4/3622 (0.11%) | 0 |
| (c) 50 Hz manual, 2 ms | precision/50fps/2ms/regular | 33.52 | 22.48 / 20.05 / 40.15 | **20.0:1211, 40.0:1146** | 0 | 2/2504 (0.08%) | 0 |
| (d) 90 Hz oversubscribed, 2 ms | precision/90fps/2ms/regular | 44.79 | 22.30 / 22.29 / 22.30 | 22.5:3444 | 0 | 3/3469 (0.086%) | 0 |

Captures: `captures/web_20260804_065504.bin` (a), `070325.bin` (b), `070618.bin` (c), `070811.bin` (d).

**ZERO-DROP GATE: PASS.** `fw_tx_enqueue_drops` and `fw_tx_stack_stalls` were 0 at start AND end of
all four runs — these are cumulative-since-boot counters never reset per run, so zero-at-start and
zero-at-end is the correct proof, not merely zero-at-end. They were verified to be real counters
incremented at genuine failure sites in `firmware/scanner-stream/Src/ethernet_transport.c`, not
decorative. `fw_tx_queue_high_water` stayed 5–7 (no pressure). `fw_tx_emitted_bytes` climbed
monotonically every poll (3.0M → 367M), proving the counters are actually written.

**CDC ISOLATION: untested, with reason.** This host cannot use ST-Link VCOM unprivileged, and the rig
runs untethered with no USB cable attached. Task 6's CDC-isolation path (DATA to Ethernet-only above
60 fps, ACK routed to originating transport) is code-complete and unit-tested but **unverified on
hardware**. Recorded as untested — do not upgrade this to "passed" in any later doc.

**Caveat 2 — the plan's step 6 target list (60/90/100 Hz wired) predates the `91b9eac` ceiling
amendment** (sensor tops out ~46 Hz). The operating points actually run (30/46/50/90-oversubscribed)
supersede that list rather than leaving it unmet; the plan doc is annotated in place.

**BUG-075 filed (open):** (c) 50 Hz/2 ms alternates near-evenly between the 20.0 ms floor and its 2×
(1211 vs 1146 intervals, averaging 33.52 fps) even though the applied-period readback and the
`profiles.py` `expected_delivered_fps` model both predict a clean 1× 50 fps. This is a ranging-engine
/ profile-model gap, **not** a TX-pacing defect — zero TX drops and zero whole-group loss throughout.
Honestly unexplained: (d), same 2 ms exposure, holds a rock-steady 22.30 ms and **never** touches the
20.0 ms floor that (c) hits half the time, so a simple "DSS costs time" story does not fit the data
either. Do not trust `expected_delivered_fps` for short-period/short-exposure combos until this is
root-caused.

**BUG-076 fixed (`56ee9ba`):** `analyze_capture`'s hardcoded `ver != 1` made the forensics scanner and
the `capture_analyze` MCP tool report `frames_decoded=0`/whole-file SKIP_RUN for every protocol-v2
capture since `2b8a9ee`. Survived because the real streaming decoder was never affected, and a
SKIP_RUN reads exactly like a corrupt capture rather than a broken tool. Lesson: a second,
independently-maintained parser of the same wire format drifts silently — extend the
golden-vector/cross-check discipline `protocol.py` already has to any other code that parses the wire
format.

## Load-bearing findings (do not re-derive, do not re-open — except DSS, see the blocking warning above)

- **The frame-rate ceiling is sensor-intrinsic.** Floors 20.0 / 21.739 / 23.529 ms at ≤2 / 4 / 8 ms
  exposure (measured 2026-08-03); shorter applied periods deliver integer multiples (2×/3×) with the
  applied-period readback exact. HFR preset = 46 Hz / 4 ms / Precision / Regular / DSS on,
  hardware-confirmed 45.86–45.56 fps (two independent bridge runs), 0 frame loss by `t_us` cadence.
  **BUG-075 (above) shows this "clean integer multiple" story is not the whole picture at 50 Hz/2
  ms** — treat the ceiling itself as solid but the quantization model as incomplete.
- **DSS story: ⚠ CONTESTED, not settled** (see the blocking section at the top of this doc). Rev 2's
  claim — our `STANDBY_DSS_MODE` toggle is ranging-quality only, the vendored I3C driver always
  fetches the DSS LUT, ST's 100+ fps "DSS Disable" mode is unimplemented/incompatible with our
  14,842 B transform, spec §3.2.3 — is disputed by a concurrent investigation into the driver's
  CSI-2 path. Do not cite this as fact until Task 12 adjudicates it.
- **ProfileTuning.exe** (`references/software/53L9A1/`, support-gated ST tool, untracked, keep) is
  PyInstaller; decompiled model at `/tmp/pt_extract/ProfileTuning_decompiled.py` (regenerable with
  pyinstxtractor-ng + decompyle3 in a /tmp venv). Its timing model reproduces the owner's tool runs
  bit-exactly but is WRONG about our hardware (flat 26.9 ms floor vs measured 20–23.5) — measured
  brackets outrank it. Its power model is exact and now ships in `profiles.py` (`ambient_lux` param).
  Note: this tool is also the subject of the DSS dispute above — the same file, two readings.
- **profiles.py ↔ wire enums differ in numbering; map by NAME** (`control.py`
  `_manual_params_to_wire`). Cmds 9/10 ACK = 16-byte readback shape regardless of result.
- **TX queue telemetry (Task 6) is real, not decorative** — verified against genuine failure sites in
  `ethernet_transport.c`, and `fw_tx_emitted_bytes` climbing monotonically under load is the proof a
  counter is actually wired, not just present. Reuse this pattern for any future firmware counter.
- **Rig etiquette:** resolve the device via mDNS `roomscanner.local` EVERY time (IP changes across
  bridge resets — was 172.17.2.58 ↔ 172.31.253.1 in the previous session, 172.17.2.58 this one).
  `rig_down` before driving the device from /tmp scripts; `rig_up` same port + `rig_status` healthy
  after. Restart the server after host-code/protocol changes (stale-process trap). FileHub bridge
  recovery is OWNER-ONLY. ≥2 s settle between reconfigs (BUG-073).
- Device firmware currently flashed = Task 6's build (first Task 6 firmware to reach hardware,
  2026-08-04). Boot default Room Mapping; end state verified healthy indirectly (streaming resumed,
  server up) — no direct LED observation this pass.

## Remaining work, in order (Task 6 is done — start here)

1. **Task 7** — IMU/env decoupled poll rate: shared `rs_lsm_service_tick()` (idle loop must stay
   ~18.2 Hz unchanged), firmware cmds 11/12 (protocol/host/CLI/UI already landed and tested against
   UNKNOWN_CMD), stream 13 only on coincident FRAME_READY, TIM2-paced software tick, skew_check N:1
   grouping fix, **plus the BUG-074 SET_STANDBY wake-path shadow check** (same file). Gate: coupled
   mode byte-identical; decoupled rates hold independent of ToF profile; no ToF cadence
   perturbation.
2. **Task 8** — rate-aware filters: `baro_tau_frames` and `ImuFusion` reference rate from the
   APPLIED IMU/env rate (Task 7 readback), never the ToF profile; replay uses capture timestamps.
3. **Task 11** — MCP `rig_profile`/`rig_imu_env_rate`/`profile_estimate` with verified-readback
   waits. NOTE: the running MCP server predates Task 1 — `capture_profile_probe` wasn't resolvable
   in an earlier session (stale snapshot); a server restart fixes it — check this is still current.
4. **Task 12** — end-to-end validation (incl. Task 10's deferred VISUAL pass: four profile buttons,
   invalid manual combos, debounce/pending, bus-bar colors, CDC warning, second-tab sync,
   narrow-width dock-band check), re-run the 30 Hz baseline vs a known-good capture,
   **adjudicate the contested DSS finding BEFORE writing any ceiling number into ROADMAP.md**
   (see the blocking section at the top of this doc), status-sync (protocol/web/MCP docs + ROADMAP
   with actual numbers), land, then **milestone-retro** (mandatory).

Parallelism learned: safe pairs were (1‖2), (3‖4), (5‖9), (10‖amendment), (6‖refinement). Task 7
touches `vl53l9_app.c` (same file Task 6 touched — now clear); 8/10 share `web.py` — serialize
those. Hardware users serialize always.

## Open bugs from this push
- **BUG-073** (open): reconfig <2 s after a prior one can produce no ACK/no BUSY.
- **BUG-074** (open, unconfirmed): SET_STANDBY wake path likely missing BUG-072's shadow repair.
- **BUG-075** (open): 50 Hz/2 ms manual request delivers a near-even bimodal 20/40 ms alternation
  instead of a clean floor; mechanism unexplained; not a TX-pacing defect.
- BUG-072 fixed (`b10f44d`).
- BUG-076 fixed (`56ee9ba`): `analyze_capture` blind to protocol v2 since `2b8a9ee`.
