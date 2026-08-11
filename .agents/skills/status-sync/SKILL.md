---
name: status-sync
description: Use when landing any work — committing a completed feature/fix/phase to main, or being told to "ship it", "land it", or "wrap it up" — in the roomscanner repo, before declaring the work done.
---

# Status sync — docs move with the code

The repo's planning docs are load-bearing: every session plans against `ROADMAP.md` and `CLAUDE.md`.
A commit that changes what's true about the project but not the docs creates drift that a later
session pays to rediscover (the 2026-07-10 retro burned a full session correcting exactly this).

**The unit of "done" is: code + the doc deltas it implies, in the same commit.**

## The checklist (fill every slot; write "n/a — <why>" where truly not applicable)

1. **GitHub Issues — work items.** Does this work advance/close a `work-item`-labeled issue, clear a
   deferred item, change a measured number, or invalidate a prediction? Update the issue **in this
   commit's** neighborhood (`gh issue comment <n> --body "..."` with measured numbers in their
   convention — interval vs wall-clock fps — and `gh issue close <n> --reason completed` if it's
   done). Completed-work narratives that deserve a durable write-up belong in
   `docs/roadmap-history.md` (keep the `Phase N` names); a closed issue's own comment thread is
   usually enough on its own.
2. **GitHub Issues — defects.** New defect → `gh issue create --label bug --label area/<area>`
   (`docs/engineering-practices.md` has the area list). Closing one → `gh issue close <n> --reason
   completed` (or `"not planned"` for by-design/anomaly/investigated, with the matching `status/*`
   label). The issue body is the permanent record — add a comment for a later addendum rather than
   editing the original report away. **Titles carry no `BUG-NNN:` prefix** — the `bug` label is the
   type and `#NNN` is the only ID (the legacy prefixes were stripped 2026-08-11; see the
   `session-start` skill's title convention).
3. **Superseded content — annotate while open, move on close.** While an item is *open*, annotate what
   you proved wrong (a follow-up comment on the issue, or strikethrough in `ROADMAP.md` prose for
   standing-decision content). On *close*, the record is the closed issue (or
   `docs/roadmap-history.md` for phase-level work) — don't accrete strikethrough forever. Never
   silently delete it; never leave it stale.
4. **Ledgers** — Reference-firmware bug list, "Considered and rejected" (both stay in `ROADMAP.md`
   prose, not GitHub Issues — vendor-package bugs we don't own, and rejected proposals aren't tracked
   work): move or annotate affected entries; don't create a duplicate entry elsewhere.
5. **`AGENTS.md`** (repo root — `CLAUDE.md` and `.agents/AGENTS.md` are symlinks to it) — Only if a
   phase status or an architecture decision changed; keep the summary consistent with `ROADMAP.md`
   and the open GitHub Issues.
6. **Memory** — Any auto-memory file whose description states a now-changed status ("STILL OPEN",
   "blocked", "draft PR") gets reconciled, along with its index line. These live **outside the
   repo**, at `~/.claude/projects/-home-sam-git-personal-lidar-roomscanner/memory/` with the
   `MEMORY.md` index in that same directory — so they are never part of the commit, and reconciling
   them is a separate write you must not skip just because `git status` looks clean.
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
  ROADMAP/CLAUDE.md delta and you inherit the drift. On 2026-07-29 a 3D calibration view merged and
  *none* of `ROADMAP.md`, `CLAUDE.md`, or the task's resume doc mentioned it — the resume doc would
  have sent a fresh session hunting for a 2D-only modal. Caught only by a second session-end. Either brief
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

## After the final commit — hand off to `session-end`

If the commit you just landed **closes the session's governing issue** — its message contains
`Closes #NNN` — that is the session's final commit. **Continue directly into the `session-end`
skill in this same turn**; do not stop and wait to be told to wrap up. Mid-session landings that
only `Refs #NNN` do not trigger this — run this checklist and keep working. (A `Stop`-hook backstop,
`.claude/hooks/session-end-guard.sh`, will block the turn and remind you if you forget — but don't
rely on it.)
