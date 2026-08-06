---
name: status-sync
description: Use when landing any work — committing a completed feature/fix/phase to main, or being told to "ship it", "land it", or "wrap it up" — in the roomscanner repo, before declaring the work done.
---

# Status sync — docs move with the code

The repo's planning docs are load-bearing: every session plans against `ROADMAP.md` and `AGENTS.md`.
A commit that changes what's true about the project but not the docs creates drift that a later
session pays to rediscover (the 2026-07-10 retro burned a full session correcting exactly this).

**The unit of "done" is: code + the doc deltas it implies, in the same commit.**

## The checklist (fill every slot; write "n/a — <why>" where truly not applicable)

1. **ROADMAP.md — work-item register.** Does this work advance/close a work item
   (`SLAM-`/`SENS-`/`XPORT-`/`FW-`/`OFFLINE-`/`TOOL-`/`DC-`), clear a deferred item, change a measured
   number, or invalidate a prediction? Update the register entry **in this commit**, with measured
   numbers in their convention (interval vs wall-clock fps). Completed-work narratives belong in
   `docs/roadmap-history.md` (keep the `Phase N` names) with only a stub left behind — don't let
   ROADMAP.md grow back into a history dump.
2. **BUGS.md + `bugs/`.** New defect → next free `BUG-NNN`, write the full entry as `bugs/BUG-NNN.md`,
   and add a row to the `BUGS.md` index table. Closing one → flip the status in its index row; the
   entry file stays as the record. Post-mortems live in the entry file, never inline in the index.
3. **Superseded content — annotate while open, move on close.** While an item is *open*, annotate what
   you proved wrong in place (strikethrough + what shipped). On *close*, the record moves to its
   archive (the bug's entry file / `docs/roadmap-history.md`) instead of accreting strikethrough
   forever. Never silently delete it; never leave it stale.
4. **Ledgers** — Reference-firmware bug list, "Considered and rejected", the data-collection queue:
   move or annotate affected entries; don't create a duplicate entry elsewhere.
5. **AGENTS.md** — Only if a phase status or an architecture decision changed; keep the summary
   consistent with ROADMAP.md.
6. **Memory** — Any auto-memory file (and its `MEMORY.md` index line) whose description states a
   now-changed status ("STILL OPEN", "blocked", "draft PR") gets reconciled.
7. **New files** — repo-relative paths ≤150 chars (longer breaks `git worktree add` and fresh
   clones on Windows).
8. **MCP surface** — did this work add an agent-facing capability, or a script under `host/tools/`?
   Then it ships as an MCP tool in the same commit: wrapper registered in
   `host/src/roomscan/mcp_server/`, `EXPOSED`/`EXCLUDED` updated in
   `host/tests/test_mcp_registry.py`, `docs/mcp-server.md` updated. Did it change a `/ws` message
   the `rig_*` tools read or send? Then check those tools still verify the *effect* rather than
   just the echo (`docs/mcp-server.md` → "Two invariants").

## Branch discipline (owner workflow, 2026-07-16)

The PR flow is retired. **Land work by committing straight to `main`, no PR.**

- Do the work on a short-lived branch (or a worktree) for isolation if you like, but finish by
  getting the commit onto local `main` and **closing the branch without a PR** — either commit
  directly on `main`, or merge/fast-forward the branch into `main` and delete it
  (`git branch -d <branch>`). No `gh pr create`, no `gh pr merge`.
- The doc deltas (checklist above) ride in the **same commit** as the code — that's what "docs
  move with the code" now means without a PR to bundle them.
- Landing = commit on `main` + branch closed. Pushing to `origin` is a separate step; only push
  when the owner asks (they may want to review the local commit first).
- Subagents don't commit; the controlling session commits (subagent cwd defaults to the main
  checkout — the mis-commit that caused the 2026-07-10 main/PR divergence). This still holds.
- **When you MERGE a subagent's feature commit, run this checklist yourself against it.** Subagents
  are briefed on the feature, not on the planning docs, so their commit will land the code with no
  ROADMAP/AGENTS.md delta and you inherit the drift. On 2026-07-29 a 3D calibration view merged and
  *none* of `ROADMAP.md`, `AGENTS.md`, or the task's resume doc mentioned it — the resume doc would
  have sent a fresh session hunting for a 2D-only modal. Caught only by a second wrap-up. Either brief
  the subagent to include the doc deltas, or do the checklist as part of the merge commit.

## Rationalizations (all mean: do the checklist now)

| Excuse | Reality |
|---|---|
| "Docs can be a follow-up commit" | Follow-ups don't happen; that's how the drift occurred. |
| "I only touched code, not the plan" | Closing an open item or changing a measured number *is* changing the plan's truth. |
| "The milestone retro will catch it" | The retro is a backstop, not the mechanism — and it costs a session. |
| "Owner said ship it quickly" | The checklist is minutes; correcting drift is a session. Quick = this list, once. |
| "I'll commit the docs right after the code" | Same commit, or it won't happen — that's how the drift occurred. |

## Red flags — stop and run the checklist

- A commit message saying "closes/completes/fixes" an item that still reads open in ROADMAP.md.
- Landing code with no doc delta in the same commit when a phase status / measured number changed.
- A memory description contradicting what you just verified.
- `gh pr create` / `gh pr merge` in your plan — the PR flow is retired; commit to `main` instead.
