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

**Always pass BOTH `observed_week_pct` and `observed_block_pct`.** The weekly is almost always the
binding window, and without its anchor the verdict silently covers the 5h block only.

**If the `roomscan` MCP tools are not exposed in your session, drive the CLI front ends instead** —
verified 2026-08-12, when a whole fleet run had *no* MCP tools available (no `fleet_*`, no `run_tests`,
no `ui_*`), though `.mcp.json` was correctly configured. Same code, same fields:

```bash
host/.venv/bin/python host/tools/fleet_budget.py --ceiling 80 --agents 3 --minutes 45 \
    --observed-week-pct 73 --observed-block-pct 33 --json
host/.venv/bin/python host/tools/fleet_plan.py --max-agents 3 --priorities "now,next" --json
```

Check this at Step 1 rather than discovering it at Step 8.5 — **whether the MCP tools exist decides
which issues you may claim at all**, because they are how the orchestrator does its own verification.

**Re-checking the budget mid-run re-forecasts; it does not re-measure.** `seven_day.pct` is pinned to
the `observed_week_pct` you passed, so it will read the same at every wave boundary. The measured
quantity is `seven_day.weighted_tokens` — diff it between invocations to see what the fleet actually
spent (2026-08-12: a 2-worker wave with one review round-trip and one full suite cost 8.0M weighted
tokens ≈ 0.32 weekly points). Report the percentage as *the owner's figure* and the delta as *the
measurement*; presenting a re-read of the anchor as evidence the run is on budget is circular.

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

**Cross-check the batch against `operator_queue()` before anything else — the planner does not.**
`fleet_plan` has no idea an issue is already parked on the owner, so a `needs/operator` issue ranks
exactly as if it were actionable. On 2026-08-12 that put **two of its three picks** (#60 at 105, the
top score, and #57 at 85) into a wave when both were already in the operator queue, out of nine held
issues repo-wide. One `operator_queue(detailed=False)` call at Step 2 removes the whole class. Treat
an outstanding `needs/operator` as a **veto** — somebody is already waiting on the owner for it.

**Distrust `prior_work` credit; open the thread.** The planner doubles an issue's score for prior
work, and on the same run credited #158 with *"prior work x2 (implementation plan comment)"* when
**the issue had no comments at all** — a phantom signal promoting it into the wave. Confirm the
comments exist before letting that multiplier decide anything. (Both defects are filed as #177.)

**Check the data the issue needs actually exists on this box.** #158 also wanted RTAB-Map exports:
the rtabmap checkout's `data/samples.zip` is bag-of-words vocabulary imagery, not an export, and the
paired capture (#161) is `status/blocked`. Its coordinate-convention half — which its own body flags
as the BUG-051/058 failure mode — had nothing to validate against. `ls` the directory before
claiming.

**Read the body of every issue you are about to claim. A high score is not actionability.** The
planner ranks on labels, prior work and footprint — it cannot see a sentence. On 2026-08-12 two of
the three top-ranked issues were un-workable for reasons only their prose states: #60 (scored 105,
`priority/now`, handed the exploration slot twice) names **owner action** as its next step — a
physical experiment flexing the Ethernet patch cable to discriminate link-bounce from bridge roaming
scans from a firmware TX-pacer stall — and #84 came back `orchestrator-only`. Treat
`suggested_model: orchestrator-only` as a **veto, not advice**: it means firmware or transform work
that needs the rig, whose error paths spin forever instead of returning. Veto by hand, say why, and
leave the issue unclaimed rather than half-claimed.

**Veto anything whose acceptance needs a verification THIS session cannot perform.** Workers drive no
browser and call no MCP tools, so the check always falls to you (Step 8.5) — and if your own session
lacks the tools, nobody can do it. On 2026-08-12 the second-ranked cluster was four `area/host-web`
issues (#106–#109: splat transparency slider, oscillate orbit, camera eye-level, floorplan preview)
whose acceptance is *entirely* "does it look right in the viewport", in a session with no `ui_*`
tools. Landing that code unverified would have shipped a green diff nobody had ever seen run. Also
veto issues the body itself parks on someone else: #127 says **blocked on orientation ground truth
(DC-F)**, and the `data-collection` issues (#128, #142–#144) are owner captures, not code.

Ask of each candidate: *what would prove this fixed, and can I run it today?* If the honest answer is
no, leave it and say so — a run that lands three unverifiable diffs is worse than a run that lands one
verified fix. Prefer issues whose acceptance is **pytest-shaped**: a named function, a stated
mechanism, and a test the body already specifies.

That question is the `operator-request` skill's close-or-hold table, applied at planning time
instead of at closing time — it is the same judgement, and that skill carries the full version plus
the rationalizations. **A veto is not a dead end: it is an operator request waiting to be written.**
Rather than leaving a vetoed issue silently unclaimed, run `operator-request` on it — post the
runbook, apply `needs/operator` + a subtype — so the next wave finds it actionable instead of
re-deriving the same veto. `operator_queue()` lists what is already waiting, so you can skip
issues whose request is outstanding.

Where the planner reports a soft conflict, **assign the contested file exclusively** instead of
hoping. Name it in both spawn prompts — "X owns `foo.py`; you stop and report rather than edit it" —
and in the claim comments. Cheap, and it converts a gamble into a rule. Note the footprint model
does not see **shared test files**: two workers appending to the same `test_*.py` is common and only
shows up at the second one's rebase.

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

**If the agent type is not found, you are in the session that created it.** Agent definitions are
loaded at session start, exactly like the MCP server pinning the code it started with — so a
definition written this session is invisible until the next one. Fall back to `subagent_type:
"claude"` with an explicit `model:` and paste the contract into the spawn prompt; the contract is the
load-bearing part, the agent file is only how it stops being retyped. Verified 2026-08-12.

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

Read the diff against **the issue's plan**, not just against itself. The failure mode a clean diff
cannot show you is a step that was never done: plans here gate steps on a condition (*"if the
existing test harness makes it cheap"*), and a worker that skips one tends to omit it from the
report entirely rather than decline it. Check each conditional step yourself — the condition is
often cheaper to evaluate than the worker judged. Also confirm `git stash list` is empty: `refs/stash`
is shared across every worktree of the clone, so a worker that stashes to prove a regression and dies
mid-proof leaves it on the owner's shared checkout.

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

## Step 8.5 — The verifications only you can run

Workers call no MCP tools and drive no browser, so every issue that needs the rig, the GPU or the
one shared browser arrives with that row in `blocked_on`. Run those yourself after the merge — and
before you do, check what the tool you are about to verify with is actually running:

- **A fix TO the MCP layer cannot be verified THROUGH the MCP layer.** `roomscan-mcp` is long-lived
  and pins the modules it booted with, so after landing a `tools_*.py` / `session.py` fix, calling
  that same tool executes the *pre-fix* code and reads as the fix not working. Restarting
  `roomscan-web` does not help — different process. **Drive the merged module in a fresh `python`
  process instead** (that is how #168's viewport fix was confirmed: three requested sizes, three
  exact `innerWidth`/`innerHeight` matches). If the check also needs the device *through* the stale
  server, you cannot do it at all — say so on the issue rather than assuming, as #171 did.
- **Restart `roomscan-web` before any browser check**, or you are testing the code it started with.
  Confirm the restart took by looking for a field the new code adds, not by the process being up.
- Prefer a check whose failure is visible: for #101 the proof was the viewport empty and Device FPS
  `-` *while the event log showed the device still emitting*, then data returning on capture load.
- **A worker's "could not reproduce — the code looks correct" on a *behavioural* claim is not a
  negative result. It is a request for this step.** Verified 2026-08-12 on #107 ("Oscillate does
  nothing"): the worker traced the whole path, checked every known trap in this repo, and reported
  the implementation complete and internally consistent. It was right about every line and the bug
  was still real — the defect was a wrong assumption about *three.js*, not about our code
  (a positive `autoRotateSpeed` **decreases** the azimuth, so both reversal branches re-asserted the
  direction the wave was already travelling and it orbited forever). Static reading cannot see a
  library's sign convention. Measure it: 80.1° of travel in 9 s at an 18° amplitude with 0 reversals
  before, 1 reversal bounded to ±21° after. Do not let a confident static "cannot reproduce" close
  the row — and do not re-read the source harder, which is the same reasoning that produced the bug.
- Splitting "never runs" from "runs and decides wrong" is one probe, not a bisect: find a value the
  loop **re-asserts every tick**, force it to something the loop would never choose, and see whether
  it snaps back. On #107 that collapsed the search space in a single reading and proved the wave was
  live before any code was changed.

## Step 9 — Close out once, at the end

Run the full suite once after all merges, not per branch. Then run `session-end` **once** for the
whole fleet: it records memory, posts one comment per claimed issue, runs `status-sync` over the
union of every worker's `doc_deltas`, and lands a single commit carrying every `Closes #NNN`. One
closing commit at HEAD, one marker write, and `session-end`'s own invariants hold unmodified.

Finally, drop `status/in-progress` from every issue you claimed — including the ones you yielded or
deferred mid-run.

## Verified end to end, 2026-08-12 (issue #170)

One full round trip, one Haiku worker, on this repo:

- `git worktree add` + both symlinks → the **full suite from inside the worktree passed 2081 tests
  with 0 skips** in 6:29. Without the `host/transform/build` symlink 15 of those skip.
- The worker committed `Refs #170` on its own branch using `git -C <abs worktree>`. The Step-0
  tripwire held: main's HEAD and porcelain were byte-identical before and after.
- The review gate **earned its place** — the worker's first commit carried a comment claiming
  `AREA_GLOBS_EXCLUDED` was "used by `expand_footprint()`" when nothing referenced it. Sent back to
  the same worker, corrected in one round. A cheap tier will write a plausible false comment; read
  the diff, do not read the report.
- `rebase` (already up to date) → `merge --ff-only` → symlinks removed → `worktree remove` →
  `branch -d`, clean.
- `fleet_plan()` against the live tracker: 65 open, 17 candidates, 3 selected, and it correctly
  excluded #170 itself as `claimed by another session` — the claim written in Step 3.
- `fleet_budget()` cross-checked: ccusage and the native reader agreed to 1.6 points on the 5-hour
  window (51.2% vs 52.8%) and 0.7 on the rolling week, with the same reset time and verdict.

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
- A worker report that is **silent** about a conditional step of its issue's plan.
- A non-empty `git stash list` at the review gate.
