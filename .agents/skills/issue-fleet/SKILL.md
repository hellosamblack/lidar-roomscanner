---
name: issue-fleet
description: Run several subagent workers in parallel across open GitHub Issues, each in its own
  worktree, under a declared usage ceiling. Use when the ask is to burn down the backlog rather than
  fix one thing — "work the backlog", "run a fleet", "parallel agents on issues", "knock out several
  issues", "spend the next few hours on open issues". Picks the highest-impact issues, prefers ones
  that already have prior work, keeps workers off each other's files, and delegates to the cheapest
  model that will not cost quality.
---

# Issue fleet — you orchestrate, workers implement

You are the orchestrator. You own **every commit boundary, every `gh` call, and every doc delta**;
workers own only the code inside their own worktree. That split is not fastidiousness, it is the
scar tissue from four incidents. On 2026-07-10 an implementer subagent wrote files into a worktree
and ran `git commit` at the main checkout, landing its task straight onto `main`; recovery cost a
full cycle. On 2026-07-29 a worker told to "prefer port 8001" killed the owner's `:8000` server and
relaunched it from a worktree about to be deleted — so the live server served unmerged code. Around
the same date another session swept an entire uncommitted working tree into its own commit. And on
2026-08-11 (#103) a worktree session verified a UI fix through `ui_*`, which drives the **main**
checkout's server — it was testing code it had not written.

Every one of those is a worker doing something outside its own worktree. So: workers never run
`gh`, never touch `main`, never touch shared docs, never touch `:8000`, and never call an MCP tool.

Two planners do the arithmetic — `fleet_plan()` and `fleet_budget()`, documented in
`docs/mcp-server.md` and implemented in `host/tools/fleet_plan.py` and
`host/tools/fleet_budget.py`. Read their `notes`, `limit_basis` and `coverage` fields rather than
just their numbers. The worker contract they are dispatched with lives in
`.agents/skills/issue-fleet/references/worker-brief.md`.

## Step 0 — Reconcile before planning

```bash
git -C . worktree list                       # stale worktrees from a crashed run?
git -C . status --porcelain                  # the owner may have uncommitted work RIGHT NOW
git -C . rev-parse HEAD                      # snapshot: compared again after every wave
gh issue list --repo hellosamblack/lidar-roomscanner --label "status/in-progress" \
  --state open --json number,title,updatedAt
```

Record HEAD and the porcelain output — they are the tripwire in Step 7. A `status/in-progress` issue
older than ~2 h whose last comment is a fleet session-start with no outcome is a **stale claim from a
dead run**: read the thread, then clear the label with a comment saying why. Never silently reuse a
claim that is not yours.

## Step 1 — Get the ceiling, then check it

Ask the owner for a not-to-exceed figure if they have not given one ("80% of the 5-hour", "60% of the
weekly"). Then:

```
fleet_budget(ceiling_pct=80, forecast_agents=3, forecast_minutes=45)
```

**Print `limit_basis` and `coverage.includes_subagents` verbatim in your reply.** There is no quota
API on this box — the `claude` CLI has no usage subcommand and the real figures never reach disk — so
every number is an estimate against a denominator that is either the owner's or this account's own
historical peak. A percentage without its denominator is a fabricated number.

`verdict: "unknown"` means stop and ask. It never means proceed.

## Step 2 — Plan the batch

```
fleet_plan(max_agents=3, priorities="now,next")
```

Cap the wave at **3**. Beyond that, pytest and headless Chrome contend for CPU and the review queue
becomes the bottleneck anyway.

The planner already enforces what it can: no two selected issues collide on stated files or hot
files, at most one worker per singleton resource (browser / device / GPU), and one reserved
exploration slot for an issue with no discoverable footprint. What it hands to **you**:

- `notes` — soft conflicts (issues meeting only on *inferred* files) and prose-inferred dependency
  edges. Veto anything you know really overlaps. GitHub's native `blockedBy` is empty across this
  repo, so the dependency spine exists only in sentences and is advisory.
- `suggested_model` — advisory. `orchestrator-only` means firmware or transform work: it needs the
  rig, and its error paths spin forever instead of returning, so a worker cannot recover unattended.

Model tiers: **Haiku 4.5** for mechanical work (a declared footprint of ≤3 files, doc-shaped or
test-shaped changes, grep-checkable rules); **Sonnet 5** for ordinary implementation; **you (Opus)**
for orchestration and the review gate. Do not put a cheap tier on SLAM numerics — a wrong fix there
reads as plausible.

## Step 3 — Claim serially, before spawning anything

For each selected issue, in one pass, before any worker starts. Write the comment to a file with the
Write tool and pass `--body-file`: backticks inside a double-quoted `--body` are command-substituted
before `gh` sees them, and a footprint list is nothing but backticks.

```bash
gh issue edit NNN --repo hellosamblack/lidar-roomscanner --add-label "status/in-progress"
gh issue comment NNN --repo hellosamblack/lidar-roomscanner --body-file /tmp/claim-NNN.md
```

The comment is a `**Session start**` block tagged with a run id (`fleet-<YYYYMMDD-HHMM>`) so the
owner's own sessions can tell a fleet claim from a human one, and carries `Files in scope:` from the
plan's footprint — which is also what makes the *next* run's footprint extraction `declared` rather
than guessed.

Then **read back**. Labels are last-write-wins with no compare-and-swap, so another session can claim
the same issue in the same second:

```bash
gh issue view NNN --repo hellosamblack/lidar-roomscanner --comments
```

If someone else's `**Session start**` on that issue is *older* than yours, yield it — earliest
timestamp wins, drop your label, re-plan. A deterministic rule needs no negotiation.

## Step 4 — One worktree per issue

Prefer spawning the worker with `isolation: "worktree"`. The harness then **structurally** refuses
that session's git operations against the shared checkout, which is worth more than any instruction:
the 2026-07-10 mis-commit happened to an agent that had been told not to.

When you need a specific branch name or pre-made symlinks, create it yourself instead:

```bash
git -C . worktree add .claude/worktrees/issue-NNN-<slug> -b issue-NNN main
ln -s ../../../captures .claude/worktrees/issue-NNN-<slug>/captures
ln -s "$PWD/host/transform/build" .claude/worktrees/issue-NNN-<slug>/host/transform/build
```

Both symlinks matter. Without `host/transform/build`, 15 tests skip as "native transform DLL not
built" — which reads as a regression against the 0-skips rule rather than a setup gap. Firmware
worktrees need two more junctions; see the `worktree-subagent-gotchas` memory rather than guessing.

`fleet_plan` sizes `worktree_name` to keep the repo-relative path under 150 characters, past which
`git worktree add` breaks.

## Step 5 — Brief and spawn

Use the `fleet-worker` (Sonnet) or `fleet-worker-mech` (Haiku) agent type, defined in
`.claude/agents/fleet-worker.md` and `.claude/agents/fleet-worker-mech.md`. Give each worker its
issue number, its worktree **absolute** path, its assigned port, and the plan's footprint as a
starting map — not as a limit.

The contract, restated in the spawn prompt because it is load-bearing:

- **Absolute paths for everything.** A subagent's cwd is the main checkout even when its worktree is
  elsewhere; a relative path silently edits the wrong tree.
- `git -C <abs worktree>` for every git call. `cd <abs worktree>/host` for pytest.
- Commit `Refs #NNN` on the worktree branch. **Never `Closes`** — fast-forwarded to HEAD it trips the
  Stop hook mid-fleet.
- **Explicit pathspecs, never `git add -A`.** The two symlinks show as untracked, and a multi-path
  `git add` aborts entirely and silently if one pathspec matches nothing.
- Forbidden: `ROADMAP.md`, `AGENTS.md`/`CLAUDE.md`, `BUGS.md`, `docs/**`, `.remember/**`, the issue
  migration map. Report intended doc changes as a `doc_deltas` block; you apply them once, at the end.
- **Do not touch :8000.** Your port is the assigned one, or none at all.
- No `gh`. No MCP tools — every one of them runs from the main checkout, so `run_tests` cannot see a
  test the worker just wrote and `ui_*` serves code the worker did not edit (`docs/web-ui-testing.md`
  has the recipe for a worktree-local server if front-end verification is genuinely needed).
- `docs/engineering-practices.md` is binding: TDD below the viewer, assert quantities not types,
  prove a regression test by reintroducing the defect.
- Finish with a structured report: `files_changed`, `tests_run`, `doc_deltas`,
  `unexpected_files_touched`, `blocked_on`.

## Step 6 — Monitor by side effects

**Do not infer liveness from transcript size.** On 2026-07-29 a healthy agent that had already
produced a 600-line module looked stalled at 144 bytes and was re-dispatched, nearly putting two
agents on the same files. Look at `git -C <wt> status`, `find <wt> -newermt`, or just wait for the
completion notification. Re-check `fleet_budget` against `projected_pct` at each wave boundary.

## Step 7 — Review gate, per branch

```bash
git -C <wt> log main..HEAD --pretty=%s          # no "Closes" anywhere
git -C <wt> diff --name-only main...HEAD        # no SHARED_DOCS, no surprises vs the footprint
git -C . rev-parse HEAD && git -C . status --porcelain   # unchanged vs Step 0?
```

Main's HEAD or dirty set moving during a wave means a worker committed to the shared checkout — stop
and fix that before merging anything. Then run the `code-review` skill inline on the branch diff. Send
findings back to the **same** worker, which still holds the context; spawn a fresh reviewer only for
diffs over roughly 800 lines, to protect your own context.

## Step 8 — Land serially, largest footprint first

```bash
git -C . diff --name-only main...issue-NNN                 # incoming
# ABORT if incoming intersects the owner's uncommitted files
git -C <wt> rebase main                                    # conflicts stay in the private worktree
cd <wt>/host && /path/to/main/host/.venv/bin/python -m pytest -q --no-header -k <narrow>
git -C . merge --ff-only issue-NNN
rm <wt>/captures <wt>/host/transform/build                 # or `worktree remove` refuses
git -C . worktree remove <wt> && git -C . branch -d issue-NNN
```

Rebase inside the worktree is the point: a conflict never leaves the shared checkout in a broken
state. **No `merge --squash`, no `cherry-pick`** — both write into the shared index that the owner's
other sessions are using, and both destroy the per-commit `Refs #NNN` attribution you paid for.

Re-verify before each merge: another session can land two commits mid-wave, so what was a
fast-forward when you planned it may have diverged.

## Step 9 — Close out once, at the end

Run the full suite once after all merges, not per branch. Then run `session-end` **once** for the
whole fleet: it records memory, posts one comment per claimed issue, runs `status-sync` over the
union of every worker's `doc_deltas`, and lands a single commit carrying every `Closes #NNN`. One
closing commit at HEAD, one marker write, and `session-end`'s own invariants hold unmodified.

Finally, drop `status/in-progress` from every issue you claimed — including the ones you yielded or
deferred mid-run.

## Red flags

- `git add -A` anywhere near a worktree.
- A worker briefed with `gh`, or calling `run_tests` / `ui_*`.
- A percentage reported without its `limit_basis`, or `verdict: "unknown"` treated as `go`.
- `merge --squash` or `cherry-pick` into `main`.
- Two browser workers, or two device workers, in one wave.
- `Closes #NNN` on a worker branch.
- `main`'s HEAD moved during a wave and you did not stop.
- A batch where every issue has prior work — the cold tail is starving; check the exploration slot
  actually fired.
