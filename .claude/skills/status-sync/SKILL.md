---
name: status-sync
description: Use when landing any work — opening/updating a PR, committing a completed feature/fix/phase, or being told to "ship it", "land it", or "wrap it up" — in the roomscanner repo, before declaring the work done.
---

# Status sync — docs move with the code

The repo's planning docs are load-bearing: every session plans against `ROADMAP.md` and `CLAUDE.md`.
A merge that changes what's true about the project but not the docs creates drift that a later
session pays to rediscover (the 2026-07-10 retro burned a full session correcting exactly this).

**The unit of "done" is: code + the doc deltas it implies, in the same PR.**

## The checklist (fill every slot; write "n/a — <why>" where truly not applicable)

1. **ROADMAP.md** — Does this work complete/advance a phase, clear a deferred/open item, change a
   measured number, or invalidate a prediction? Update the phase's status block **in this PR**.
   State measured numbers with their convention (interval vs wall-clock fps).
2. **Superseded content** — Anything the work proved wrong (a predicted encoding, a planned
   approach) gets **annotated as superseded in place** (strikethrough + what shipped instead).
   Never silently delete it; never leave it stale.
3. **Ledgers** — Reference-firmware bug list, deferred/open lists, "Considered and rejected":
   move or annotate affected entries; don't create a duplicate entry elsewhere.
4. **CLAUDE.md** — Only if a phase status or an architecture decision changed; keep the summary
   consistent with ROADMAP.md.
5. **Memory** — Any auto-memory file (and its `MEMORY.md` index line) whose description states a
   now-changed status ("STILL OPEN", "blocked", "draft PR") gets reconciled.
6. **New files** — repo-relative paths ≤150 chars (longer breaks `git worktree add` and fresh
   clones on Windows).

## Branch discipline (hard rules)

- Work rides a worktree branch → push → **draft PR**. **NEVER commit to local `main`**, even
  doc-only changes; never `git merge` a feature branch into local main.
- **NEVER merge your own PR** (`gh pr merge` is owner-only). Landing = draft PR open + branch
  pushed. Baseline testing showed models under "ship it tonight" pressure append
  `gh pr merge --squash --delete-branch` — that line is the violation, delete it.
- Subagents don't commit; the controlling session commits (subagent cwd defaults to the main
  checkout — the mis-commit that caused the 2026-07-10 main/PR divergence).

## Rationalizations (all mean: do the checklist now)

| Excuse | Reality |
|---|---|
| "Docs can be a follow-up commit" | Follow-ups don't happen; that's how the drift occurred. |
| "I only touched code, not the plan" | Closing an open item or changing a measured number *is* changing the plan's truth. |
| "The milestone retro will catch it" | The retro is a backstop, not the mechanism — and it costs a session. |
| "Owner said ship it quickly" | The checklist is minutes; correcting drift is a session. Quick = this list, once. |
| "Merging my PR finishes the job" | Merging is the owner's decision. Draft PR = finished. |

## Red flags — stop and run the checklist

- A commit message saying "closes/completes/fixes" an item that still reads open in ROADMAP.md.
- `gh pr merge` in your own plan; any commit while `git status -sb` says `## main`.
- A memory description contradicting what you just verified.
