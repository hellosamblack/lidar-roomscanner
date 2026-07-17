---
name: milestone-retro
description: Use after completing a milestone or phase (post-merge, before starting the next phase) — a retrospective that converts the push's friction into skills, scripts, and references so the next push is easier. Mandated by docs/engineering-practices.md "Self-improvement after milestones".
---

# Milestone retrospective → reusable tooling

A milestone isn't done until the next one got easier. Run this after every merge that closes a phase or
major effort, before the next phase's plan executes.

## Procedure

1. **Mine the evidence** (don't rely on memory): read the phase's section of
   `.superpowers/sdd/progress.md` and skim the task reports (`.superpowers/sdd/*-report.md`). Look for:
   - Rituals rebuilt from prose by multiple subagents (capture scripts, reset/retry loops, bench
     harnesses, flash-and-measure sequences).
   - Environment facts learned the hard way (tool locations, port behaviors, timing windows, race
     conditions, library quirks) that live only in reports or dispatch prompts.
   - Review findings that repeated across tasks (the same class of bug caught twice = a checklist gap).
   - Anything a dispatch prompt had to explain at length that a skill/reference should own.
2. **Convert, don't summarize.** For each finding pick the durable form:
   - **Script** (`host/tools/*.py`, or `scripts/` for non-Python): parameterized, documented, tested at
     least once for real. Rule: a hardware ritual performed from prose by >2 subagents MUST become one.
   - **Skill update**: new facts/steps into the governing skill (`firmware-loop`, `protocol-change`, …);
     a genuinely new activity gets a new skill (follow superpowers:writing-skills). Put long supporting
     material in the skill's `references/`, runnable helpers in its `scripts/`.
   - **Doc fix**: corrections to `docs/engineering-practices.md`, `ROADMAP.md` risk lists, etc.
3. **Prune while you're there**: stale skill lines (changed baud rates, moved files, dead knobs) get
   corrected — a skill that lies is worse than none.
4. **Commit** the retro output as part of closing the milestone (docs/skills/scripts commit, pushed with
   the merge or immediately after). One line in the ledger: what was extracted, what was deliberately
   skipped (and why).

## Candidate checklist (per milestone)

- [ ] Capture/measure tooling used by [HW] tasks — script-worthy?
- [ ] Flash/reset/port rituals — already in `firmware-loop`? Still accurate?
- [ ] New protocol/wire facts — in `protocol-change` + `docs/protocol.md`?
- [ ] Vendor library discoveries — reference doc'd with file:line citations?
- [ ] Repeated review findings — checklist line added to the governing skill?
- [ ] Dispatch-prompt boilerplate that grew — fold into a skill so prompts shrink?
- [ ] **Review scrutiny — docs cite evidence that exists.** Any doc claim citing a prior result, a
  measured number, or a root cause must be checked against the artifact it cites (caught twice: false
  precedent in Phase 2 Task 7, false `conf_scaling` citation in Phase 2.5 Task 5).
- [ ] **Status currency — ROADMAP.md/CLAUDE.md/memory match reality.** Every phase status block,
  open-item list, and memory index line checked against what actually merged (drift caught 2026-07-10:
  two phases complete but unmarked, stale predictions, memory contradicting hardware-verified fixes).
  The `status-sync` skill is the per-merge mechanism; this line is the backstop.
- [ ] **Review scrutiny — burst/adversarial input case for any parser of untrusted bytes.** Any new or
  touched parser of host↔device bytes needs a same-write burst (data larger than the parser's buffer)
  and a corrupted/malformed-input case exercised on hardware, not just a single well-formed sample
  (caught in Phase 3 Task 2's parse-while-draining rework).

## Executed 2026-07-16 (Web Phase 4 — SLAM mode retro)

Single-session milestone committed straight to `main` (`a0314e4`), host-only (no firmware, no
device-wire change — MESH is app-protocol), verified end-to-end in headless Chrome on the local **CUDA:0**
GPU. Most friction was converted *during* the push (design spec, `docs/web-protocol.md` updated with tag 3
+ the new messages, memories + ROADMAP/CLAUDE synced in the feature commit). This retro's two extractions:

- **Status-currency correction — "Showcase" collapsed (owner clarification).** The web plan carried a stale
  6th "Showcase mode" phase, but Showcase was only ever **another name for SLAM mapping** (record→build→save)
  — and the desktop panel redesign had *already* dissolved it ("SLAM absorbs the former Showcase flow — no
  separate Showcase concept in the UI", ROADMAP Phase 6). The web app already delivers it via Web Phases 3
  (record + load/replay) + 4 (SLAM build + **Save** full-res). Trued up: the plan is now **5 phases, not 6**
  (`ROADMAP.md` header + the Phase-1/3/4 deferred lines, `CLAUDE.md` bullet, `web-panel-replacement` +
  MEMORY.md). Only **Web Phase 5 (settings + retire `panel.py`)** remains; the desktop `slam/showcase.py`
  engine (offline full-quality re-run + reveal) is retired with `panel.py` then — its one edge over web SLAM
  (guaranteed-every-frame offline) is at most a small SLAM-mode option, not a phase.
- **`host/tools/capture.py --udp` — THE tool extraction.** The headless host has no USB (board streams over
  UDP), so `capture.py`'s serial-CDC + SWD path didn't apply; recording the Phase-4 SLAM fixture meant
  hand-rolling a UDP dump inline. Added an `acquire_udp()` path (via `get_best_source()`, no SWD/port-cycle,
  keepalive self-heal) reusing the exact same decode-and-report. **Verified live** (`--udp --seconds 8`:
  3.6 MB, streams 9/10/RAW/CALIB, 30.3 fps, 0 CRC, 0 gaps). Closes the `milestone-self-improvement` seeded
  backlog item.
- **Deliberately skipped**: no `firmware-loop`/`protocol-change` edits (host-only, no device-wire change);
  no new checklist line (the new inbound handlers `set_mode`/`slam_opt`/`save` validate untrusted input and
  are test-covered; `pack_mesh`/`build_slam_message` have round-trip tests — no repeated-finding gap); no
  edit to `slam/` (reuse-only, as designed). `verify_slam.bin` stays gitignored (regenerate via `--udp`).

## Executed 2026-07-16 (Web Phase 3 — recording & playback retro)

Not an SDD subagent phase — a single-session milestone committed straight to `main` (`e935063`),
host-only (no firmware, **no binary-wire change**), verified end-to-end in headless Chrome. Two of the
usual friction sources were already **converted during the push**: the `web_ui_shot.py` driving gotchas
landed in `docs/web-ui-testing.md` (`81d1963` — wait for server-rendered lists before clicking; don't
interleave exploratory clicks across runs since server state persists; the window-stash trick for
closure-held state), and status was synced in the feature commit (ROADMAP/CLAUDE/`web-panel-replacement`
memory all current — backstop re-checked, no drift).

- **`docs/web-protocol.md` created** — THE extraction. The `/ws` *app* protocol (browser ↔ FastAPI,
  distinct from the device wire protocol) has grown one message at a time across Web Phases 1–3 and lives
  only in scattered `web.py` builder functions + the inbound dispatcher — there is **no enum registry**
  the way the binary wire has `docs/protocol.md`. Web Phase 4 (SLAM) adds trajectory+mesh messages onto
  this same socket, so it would have had to reverse-engineer the whole contract from three specs + code.
  The new doc indexes every binary tag + JSON message (in/out) with `file:line` citations, plus the four
  invariants a new message must hold (one-way echo, validate untrusted inbound, server-side math stays
  server-side, off-loop blocking work). Pointers wired into CLAUDE.md's docs list + ROADMAP Web Phase 3.
- **Deliberately skipped**: no `firmware-loop`/`protocol-change`/`capture.py` edits (host-only, no wire or
  HW ritual — the retro's [HW]/parser-burst checklist lines don't apply; the new inbound handlers *are*
  parsers of untrusted client JSON, but each already validates + drops, covered by the 45 backend tests);
  no new script (`web_ui_shot.py` already covers the headless-drive ritual); no new memory (the
  `web-panel-replacement` memory already tracks the phase-by-phase state — `docs/web-protocol.md` is the
  durable form, not an index line).

## Executed 2026-07-15 (headless-host bring-up retro)

Not an SDD phase — a bug-fix session bringing the repo up on a fresh **headless
Linux host** (Proxmox/LXC, no GPU, Ethernet-only). Four migration gaps, all the
same shape (implicit on the Windows dev box, absent on the fresh host):
BUG-020 (native transform loader Windows-only), UDP keepalive self-heal,
BUG-021 (three.js vendored off the unpkg CDN), BUG-022 (software-WebGL Chrome
flag) — all in `BUGS.md`, merged `71f145e`, pushed. Each gap cost a manual
diagnosis dig, so the extraction is a **doctor script + reference doc** (like the
07-10 host-side retro's coordinate-frames doc, not SDD tooling):

- **`host/tools/headless_doctor.py`** — THE extraction. Runs the whole fresh-host
  diagnosis in ~5 s: vendored 53L9A1 sources present → native `.so` built+loadable
  (`--build` builds it) → board reachable + actually streaming (wake→frames) →
  viewer assets self-contained (no unpkg) → browser+WebGL. Each failure prints the
  exact fix; exit code = failures. Verified live (all pass on the current host).
- **`docs/headless-host-setup.md`** — 5-minute checklist: the four gaps, the
  doctor one-liner, and an "Offline"-diagnosis table keyed to the in-browser diag
  panel added this session. Pointer wired into CLAUDE.md's docs list.
- **Memories**: `headless-host-deployment` (host is GPU-less/Ethernet-only, the
  four gaps + fixes) and `agent-sandbox-port-binding` (Bash sandbox kills
  uvicorn/exit-144; use `dangerouslyDisableSandbox`, verify data path direct +
  browser via headless-Chrome screenshot). Both indexed in MEMORY.md.
- **Deliberately skipped**: no `firmware-loop`/`protocol-change` edits (no
  firmware or wire change — host + browser only); the stale "USB CDC is production
  link" line in the `mapping-pipeline-plan` memory is noted/superseded by the new
  headless memory but not rewritten (predates the headless move; a bigger truing-up
  best done when transport docs are next touched).

## Executed 2026-07-14 (Phase 5 Ethernet retro)

- **ROADMAP.md / CLAUDE.md trued up**: Phase 5 marked complete (no longer shelved). The architecture decision was updated to reflect USB CDC OR Ethernet UDP as the transport links.
- **Memory updated**: `recent.md` logging the descriptor exhaustion and initialization wait-loop fixes.
- **Scripts**: Promoted `test_send.py` and `test_udp_receive.py` in `host/tools/` for UDP network testing and headless device streaming validation.
- **Skill update**: Noted that the headless wait loop must check `ETH_HasTarget()` alongside USB connection to prevent hanging Ethernet-only boots.
- **Deliberately skipped**: No new project rules required.

## Executed 2026-07-10 (post-Phase-4 follow-up / pre-Phase-6 retro)

Covers the merges that landed *after* the Phase 4 retro below: PR #7 (camera-panel world accumulation),
the IMU-axis-mapping fixes (`14f6a4b`/`cb2b01c`/`55108ec`/`3c6c93d`), BUG-001/002/004/007/008, on-rig
mag calibration, and PR #8 (Phase 6 SLAM decision doc). This work was host-side viz/bug fixes on
worktrees, not an SDD phase — so the extraction is a **reference doc**, not tooling.

- **`docs/coordinate-frames.md` created** — THE extraction. Frame conventions (ToF/CV, SFLP body, SFLP
  Z-up world, Open3D Y-up CV world), the two structural transforms (`T_CV_TO_BODY`, `T_WORLD_TO_CV`), the
  shared body→world sandwich `T_WORLD_TO_CV @ R @ T_CV_TO_BODY` (reused by `gizmo_pose` and panel
  accumulation), the mag `AXIS_CONVENTION`, and gravity/down handling were scattered across `sensors.py`
  + `panel.py` and re-derived wrong repeatedly (BUG-004 yaw-as-roll, the IMU-axis commits). Consolidated
  with `file:line` citations. Phase 6 SLAM operates entirely in these frames — this directly makes the
  next milestone easier (the retro rule's whole point). Wired a pointer into ROADMAP.md Phase 6.
- **Deliberately skipped**: no new scripts (the bug fixes were one-off host edits, no repeated HW ritual);
  no `protocol-change` edits (no wire change since streams 9/10); BUG-005/006 remain open/anomaly as
  tracked in `BUGS.md`, not retro material.

## Executed 2026-07-10 (Phase 4 / IKS4A1 retro)

Distinct from earlier retros: most friction was converted **during** the push, not after it — the
`stack-electrical` skill, the vendored IKS4A1 drivers/datasheets/board model (`references/`), the I3C
bench-probe diag tools, and the shared percentile-clip IR/cloud normalization helper all landed as part
of the milestone commits (evidence the 07-09 retro's convert-don't-summarize rule took). This retro
therefore extracted mostly guidance, not tooling:

- **ROADMAP.md / CLAUDE.md trued up**: Phases 3.5 + 4 marked complete with status blocks; Ethernet
  (Phase 5) **shelved** (owner, 2026-07-10 — I3C readout, not USB, is the bandwidth wall) with explicit
  revival triggers; the top-level transport decision rewritten to match measured reality; two Phase 4
  predictions marked superseded (quat shipped 4×float32, not fp16; IMU/ENV ride at per-ToF-frame
  cadence, not independent native-rate frames); reference-bug ledger statuses added (#1 fixed, #4
  still inherited — `allocate_memory` still `uint16_t`, #6 addressed).
- **`firmware-loop` skill**: stacked-board capture expectations (streams 9/10 presence as the health
  signal; ENTDAA/env-sensor failure signatures point at `docs/iks4a1-stacking.md`, not at hardware);
  `roomscan-panel` noted as the preferred live surface.
- **Windows path-length fix**: vendored datasheet PDFs with ~180-char paths made `git worktree add`
  (and any fresh clone without `core.longpaths`) fail on Windows — renamed short (`dt0064-*`,
  `dt0106-*`), `core.longpaths=true` set in the local repo config. Rule extracted: keep repo paths
  ≤150 chars so worktree prefixes fit under Windows' 260-char limit.
- **Repo-hygiene finding**: the metrics-HUD work existed both as unpushed commits on local `main` and
  as draft PR #1, byte-identical (a worktree-subagent cwd slip — see the worktree-subagent-gotchas
  memory); resolution = merge PR #1, then a plain `git pull` reconciles local main.
- **Deliberately skipped**: no new scripts (capture.py / bench_commands.py covered every [HW] ritual
  this push); no protocol-change edits (streams 9/10 already followed the checklist when they landed).

## Executed 2026-07-09 (Phase 3 retro)

The 2026-07-08 known backlog (below) was executed in full at this retro:

- `host/tools/capture.py` shipped: SWD reset (`STM32_Programmer_CLI -c port=SWD -rst`, path
  overridable via `ROOMSCAN_PROGRAMMER`) + stale-port wait, CDC port discovery via
  `roomscan.sources.SerialSource`, boot-hang retry (≤3, documented as belt-and-braces now that
  firmware self-heals boot hangs internally — 10/10 soak, Phase 3 Task 5), timed raw capture, and a
  decode-and-report (frames by stream, fps under both conventions, CRC failures, seq gaps with the
  connect transient broken out separately, CALIB cadence check, EVENT decode). Verified live
  (`--reset --seconds 15`): 426 RAW + 7 CALIB frames, 28.4-28.5 fps, 0 CRC failures, 0 seq gaps, CALIB
  cadence exactly 64.
- `host/tools/bench_commands.py` shipped: promoted from `host/tests/bench_commands.py` (kept for
  history), rebased on `roomscan.control.CommandClient`; subcommands `ping`/`calib`/`burst N`/
  `corrupted-frame`/`mixed-burst`/`all`, per-window stream-continuity accounting, and a
  `CalibClassifier` that fixes the cadence-vs-on-demand CALIB ambiguity (discriminates by seq residue,
  falls back to send-time correlation only for genuine coincidences). Verified live: `ping`, `burst 3`,
  `corrupted-frame`, and the full `all` sequence (including `calib` and `mixed-burst`) all passed.
- `firmware-loop` skill rewritten: pruned the stale "Phase 1 Task creates it" / bare-VCOM-only framing,
  added the SWD reset one-liner, replaced the monitor section's prose with a pointer to `capture.py`,
  documented that boot-hang retry now lives IN firmware (the old external ~1-in-5 retry workaround is
  obsolete — don't reintroduce it), added the fps-convention note (print both, label which), documented
  `roomscan-ctl` and the viewer's P/C/R/1/2 command keys, and downgraded COM15/COM14 to "typically,
  not guaranteed — resolve by VID/PID."
- `protocol-change` skill: one-line addition — the lockstep artifact set includes the enum registries
  (`CommandCode`/`ResultCode`/`EventCode`/`StreamId`) across spec/py/h, same one-commit discipline.
- The two repeated review findings above were promoted from this file's throwaway backlog into the
  permanent Candidate checklist (this section, above) so they survive past this one retro.

Nothing from the 2026-07-08 backlog was skipped or deferred.

<details>
<summary>2026-07-08 backlog (superseded — kept for the record)</summary>

- **`host/tools/capture.py`**: THE top offender — every [HW] task reimplemented CDC capture from prose:
  find port by VID/PID (CAFE:4001), wait-for-stale-port-to-vanish after SWD reset, DTR-gated open,
  retry-on-boot-hang (≤3 resets), timed raw capture to file, decode-and-report (fps by stated convention,
  crc, gaps, CALIB cadence, seq contiguity). One script, flags for duration/output/reset. Fold the
  `firmware-loop` skill's monitor section onto it.
- **`host/tools/bench_commands.py`**: Phase 3 Task 2's command bench (ping/calib/burst/corrupted) —
  promote from throwaway to tool once Task 3's CommandClient lands (rebase it on that).
- **`firmware-loop` skill**: add the SWD reset one-liner, the boot-flake retry protocol, and the
  fps-convention note ((N−1)/t_us-span vs frames/wall-clock — name which one when reporting).
- **Review-checklist candidates**: "docs cite evidence that exists" (caught twice: false precedent in P2
  Task 7, false conf_scaling citation in P2.5 Task 5); "burst/adversarial input case for any parser of
  untrusted bytes" (P3 Task 2 drain-before-parse).

</details>
