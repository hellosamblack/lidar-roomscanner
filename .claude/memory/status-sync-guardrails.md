---
name: status-sync-guardrails
description: "Owner directive (2026-07-10): prevent roadmap/doc/status drift from lesser-model sessions — status-sync skill is MANDATORY at ship time; wrap-up no longer merges to main"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 8936dba6-2b29-435b-b6d0-e6c8a657e216
---

After the 2026-07-10 retro corrected accumulated drift (two phases complete but unmarked in
ROADMAP.md/CLAUDE.md, stale predictions, memory contradicting hardware-verified fixes, metrics-HUD
commits on local main duplicating draft PR #1), the owner directed: harden guidance so this can't
recur.

**Why:** planning docs are load-bearing here — every session plans against ROADMAP.md/CLAUDE.md;
drift costs a full session to rediscover and correct. Smaller models under "ship it" pressure
demonstrably (haiku baseline, 2/2 reps) auto-merge their own PRs and skip doc/memory sync.

**How to apply:**
- Run the `status-sync` project skill (.claude/skills/status-sync/) whenever landing work; the doc
  deltas belong in the SAME PR as the code. [[milestone-self-improvement]] retro is the backstop.
- Hard rules now in docs/engineering-practices.md: never commit to local main, never merge your own
  PR (draft PR = done; merging is the owner's call), controller commits — not subagents
  ([[worktree-subagent-gotchas]]), repo paths ≤150 chars.
- The `wrap-up` skill was rewritten (2026-07-10) from merge-into-main to draft-PR flow — do not
  reintroduce the old Phase 1 behavior.
- OPEN follow-up: GREEN-phase verification of the status-sync skill wording (re-run the ship-it
  scenario with the skill in context, per superpowers:writing-skills) was cut short by session
  limits — do it before trusting the skill as bulletproof.
