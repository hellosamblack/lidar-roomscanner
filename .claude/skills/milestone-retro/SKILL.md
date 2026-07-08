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

## Known backlog (seeded 2026-07-08, from Phases 1-2.5 — execute at the Phase 3 retro if not sooner)

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
