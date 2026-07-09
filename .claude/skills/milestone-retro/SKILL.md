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
- [ ] **Review scrutiny — burst/adversarial input case for any parser of untrusted bytes.** Any new or
  touched parser of host↔device bytes needs a same-write burst (data larger than the parser's buffer)
  and a corrupted/malformed-input case exercised on hardware, not just a single well-formed sample
  (caught in Phase 3 Task 2's parse-while-draining rework).

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
