# Handoff — High Frame-Rate & Manual Ranging Modes implementation (2026-08-03)

**Plan being executed:** `docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md`
(committed/frozen in `d2c4148`, including the Task 7 IMU/env-decoupling additions).
**Working model:** orchestrator session delegating each task to a Sonnet subagent
(owner directive: preserve usage limits); agents edit + test but never commit — the
orchestrator verifies (focused tests + ruff on touched files, full suite at gates)
and commits per-task. Continue that pattern.

## Landed on `main` (all verified, full suite green at each commit)

| Commit | Task | What |
|---|---|---|
| `d2c4148` | 1 | `profiles.py` pure model + 104 tests; `profile_probe.py` + `capture_profile_probe` MCP tool; spec reconciled against DS14879 (pdftotext of `references/datasheets/NUCLEO-VL53L9CX/datasheet.pdf`) |
| `2b8a9ee` | 2 | Protocol v2 lockstep: commands 8–12 in C+Python, typed ACKs, bounded `rs_parsed_command_t`, host-compiled C cross-check (`test_protocol_c_crosscheck.py`), v2 golden vectors; v1 replay pinned by frozen `golden_depth_2x2.bin` |
| `414adaa` | 3 | `CommandClient.send_profile/send_manual_params/get_ranging_config/send_imu_env_rate/get_imu_env_rate`; `roomscan-ctl profile/manual/profile-status/imu-rate/imu-rate-status`; old usecase/period/exposure deprecated |
| `7598fde` | 4 | Firmware `rs_ranging` layer: atomic apply at safe point, readback-built ACKs, app-local DSS setter, Room Mapping boot default, REINIT/standby survival. **Hardware gate passed**; DSS-off stop-point cleared (raw stays 14 842 B, transforms 8/8) |

Baseline counts after `414adaa`: **1712 passed, 1 skipped** (the permanent Windows-only
skip), ruff clean on every touched file (138 pre-existing errors elsewhere are NOT ours).

## Key decisions / findings already made (do not re-litigate)

- **Spec numbers reconciled (Task 1):** power = duty-cycle model fit from DS14879
  Table 36/9 anchors (Precision preset 220→267 mW, HFR 420→415 mW @90 fps). Range
  formula **dropped** for a categorical `(ranging_mode, dss)` Table 9 lookup — the
  datasheet confounds exposure with ambient per row, no continuous formula is
  defensible. Exposure step is a truthful **1 ms** (`EXPOSURE_STEP_MS`); sub-ms is
  PENDING a proven local setter (Task 4/5 scope if ever). Blanking margin is a
  labelled placeholder `BLANKING_MARGIN_US_PENDING_HW = 500` — **Task 5 was to
  measure it**; check its report.
- **`profiles.py` enums and wire enums differ in numbering by design.** Conversion is
  by NAME (`_manual_params_to_wire` in `control.py`) with a per-member regression
  test. Never pass `.value` across that boundary (BUG-039/048/051/058 class).
- **Firmware validates exposure vs the driver's real 1–30 ms ceiling**, host UI caps
  at 16 ms — firmware is deliberately less restrictive, never rejects a compliant host.
- **ACK for cmds 9/10 is the 16-byte ranging-config readback shape regardless of
  result**; layouts documented in `docs/protocol.md` (v2 section is authoritative).
- **CDC is untested** — no `CAFE:4001` endpoint exists on this host; reported
  honestly, not faked. Ethernet/UDP is the acceptance path (per plan).
- **Stale-server trap (bit us already, planned around):** the running `roomscan-web`
  pins the code from when it started. After flashing v2 firmware or landing host
  changes, restart it via MCP `rig_down`/`rig_up` (NEVER change the port) and verify
  `rig_status`. Task 4's agent already did one such restart cycle.
- 60 s Task 1 baseline capture: `captures/web_20260803_121735.bin` — 30.222 fps,
  0 CRC, 0.197 % RAW loss, 100 % 9/10/13 pairing, stream-11 466.2 Hz. This is the
  non-regression comparator for Tasks 5/6/12.
- Anomaly to watch: **one-off 5 s timeout on the very first command after a fresh
  flash+reset** (Task 4, reproduced once, retry succeeded). Recheck in Task 5/6/12 runs.
- Untracked `package.json`/`package-lock.json` at repo root are another session's —
  leave them alone.

## In flight at handoff time (uncommitted work in the tree)

Two background Sonnet agents were still running; their edits land as uncommitted
working-tree changes. **Verify the tree directly — do not trust or bulk-read agent
transcripts** (`~/.claude` JSONL bulk-reads OOM; the tree + tests are the truth).

1. **Task 5 — autonomous acquisition / real target FPS** (was mid-edit:
   `firmware/scanner-stream/Src/vl53l9_app.c`, `rs_ranging.{h,c}`, plus
   `host/tests/bench_commands.py` allowed). Its gate: applied-period readback first,
   then 60 s captures at 30/60/90/100 Hz; sensor cadence measured from `t_us`
   intervals (±2 %), transport losses reported separately (Ethernet pacing is Task 6,
   `ETH_TX_WINDOW_MS` still assumes 33 ms frames); 90↔30 live switches bounded;
   `vl53l9_stop()` safety proven via EVENT/ACK (no printf); standby/wake at 30 and
   90 Hz; I3C bus-work margin at 90/100 Hz. Plan stop-point: if the SENSOR can't hold
   cadence, stop — don't relabel. Expected end state: server up + streaming, device
   on Room Mapping. Commit as `feat(firmware): run high-rate profiles in autonomous sync`.
2. **Task 9 — SLAM ingest/presentation split** (files: `reader.py`, `web.py`,
   `slam/worker.py`, `metrics.py` + their tests). Briefed that **BUG-060/061 already
   did part of this** (reader-thread submission, `/ws-mesh`) — it audits current code
   first, adds the six-way metrics split (RAW received / transform completed / SLAM
   submitted / processed / latest-wins replacements / browser frames sent), pins
   presentation cadences rate-independent, and benchmarks the CUDA worker vs the
   11.1 ms budget via uncapped replay (labelled approximation; real 90 Hz check is
   Task 12). Commit as `refactor(web): separate sensor ingest from render cadence`.

If an agent died mid-task: `git diff` shows how far it got; finish or revert
per-file. If both landed cleanly: run the full suite + ruff on touched files,
verify firmware builds both knob configs, then commit each task separately.

## Remaining tasks and ordering (see plan for full text)

```
[in flight] Task 5  autonomous sync            [HW]  — firmware
[in flight] Task 9  SLAM ingest split                — host
Task 6  Ethernet pacing + queue telemetry     [HW]  — after 5 (same firmware files)
Task 7  IMU/env poll-rate decoupling          [HW]  — after 5/6 (vl53l9_app.c conflicts);
        firmware side of cmds 11/12 (host CLI + codec already landed in Tasks 2–3);
        shared rs_lsm_service_tick with the idle loop (idle stays ~18.2 Hz);
        stream 13 only on genuinely coincident FRAME_READY iterations
Task 8  rate-aware filters (imufusion/baro tau)      — after 7 (needs applied-rate
        readback) and after 9 lands (web.py/worker.py overlap)
Task 10 web UI profile controls + IMU-rate control   — after 7 (needs cmds 11/12 live);
        server-authoritative RangingState via GET_RANGING_CONFIG
Task 11 MCP rig_profile/rig_imu_env_rate/profile_estimate — after 10
Task 12 end-to-end validation + status-sync + milestone-retro  [HW] — last
```

Dependency cautions for parallelism: 6 and 7 both edit `vl53l9_app.c`/
`ethernet_transport.c` — sequential. 8 and 9 overlap in `web.py`/`slam/worker.py` —
sequential. 10 and 8 both edit `web.py` — sequential. Safe pairs were (1‖2), (3‖4),
(5‖9).

## Standing rules for the resuming orchestrator

- Tests: `cd host && .venv/bin/python -m pytest -q` — expect exactly 1 skip; 2+ means
  wrong interpreter. Firmware: `cmake --preset Debug && cmake --build build/Debug`
  from `firmware/scanner-stream` with the venv on PATH (ninja lives there); both
  `CONF_TRANSFORM_ONBOARD` knob configs must compile.
- Flash/observe via MCP `fw_build`/`fw_flash`/`rig_*`; never bind the device stream
  while `roomscan-web` owns it (agents used `/tmp` scripts with the server stopped,
  then restored it). Wi-Fi-bridge recovery is OWNER-ONLY — report, don't attempt.
- Commit per task with the plan's commit messages; stage only in-scope files
  (other sessions share this tree). Protocol changes stay lockstep
  (`protocol-change` skill). `status-sync` before landing docs claims; the plan
  mandates `milestone-retro` after Task 12.
- Honest reporting is a plan invariant: applied/readback over requested, measured
  over predicted, "blocked/untested" over silence.
