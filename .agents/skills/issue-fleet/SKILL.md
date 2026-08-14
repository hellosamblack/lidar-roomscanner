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

**An MCP tool can also be present but *stale in its signature*, which reads like a bug in the tool.**
On 2026-08-13 `fleet_plan(max_agents=3, priorities="now,next")` failed with *"plan_fleet_live() got
an unexpected keyword argument 'triage'"*. Nothing was broken: the wrapper on disk passes `triage=`,
and the long-lived `roomscan-mcp` process had booted a `fleet_plan.py` from before the triage-digest
feature landed. The tell is that the rejected parameter is one a **recent commit added**. **Do not
"fix" the signature** — that edits working code to match a stale process, and it is code you are
forbidden to author anyway. Fall through to the same CLI front end above, which reads disk on every
invocation. Absent, stale-in-behaviour and stale-in-signature are one root cause with three faces;
only the first announces itself.

**Re-checking the budget mid-run re-forecasts; it does not re-measure.** `seven_day.pct` is pinned to
the `observed_week_pct` you passed, so it will read the same at every wave boundary. The measured
quantity is `seven_day.weighted_tokens` — diff it between invocations to see what the fleet actually
spent (2026-08-12: a 2-worker wave with one review round-trip and one full suite cost 8.0M weighted
tokens ≈ 0.32 weekly points). Report the percentage as *the owner's figure* and the delta as *the
measurement*; presenting a re-read of the anchor as evidence the run is on budget is circular.

## Step 1.5 — Session rotation

`fleet_budget()` also measures **your own context**, and returns `rotate` / `rotate_hard` when
carrying it another wave costs more than handing off. **These are instructions, not advice.**

- **`rotate`** (context ≥ 300K): finish the wave in flight — merge it, run its Step 8.5 — then write
  the handoff and **stop**. Do not start another wave. Under `fleet_run.py` your successor starts
  itself; see "Supervised rotation" below.
- **`rotate_hard`** (≥ 450K): do not start the next *issue* either. Land whatever has a branch, park
  the rest, hand off.
- Pin `session_id` from the first call's `rotation.session_id` so later calls do not re-infer it.
- If `rotation.ok` is false the verdict silently covers the budget windows only and says so in
  `notes`. Fall back to turn count and say you are doing that.

**Rotating beats shrinking the wave.** A smaller wave inside a 400K context still pays 400K on every
turn; wave size attacks the growth term, which is the part rotation cannot touch and the part you
are paying for anyway. That is why `rotate` outranks `reduce`.

**The arithmetic is one-sided.** A fresh session re-caches ~45K of system prompt and reads a ~5K
handoff — about 250K weighted. Continuing at 300K costs ~20M weighted more over the next wave; at
477K it was 43M, and at 660K, 62M. **The reason not to rotate is never cost. It is lost state — and
that is exactly what the handoff file exists to carry.**

Measured before any of this existed: orchestrator cache-read outweighed output 160–380x, context ran
45K → 478–660K in a single session, the last quartile of turns cost ~4x per turn what the first did,
and **94.3%** of a week's weighted spend sat in the orchestrator seat rather than in its workers.

### Supervised rotation — the chain runs itself

If `host/tools/fleet_run.py` spawned you, **rotation is automatic and you must not ask the owner to
restart the run.** Your prompt says so, names your link number, and pins your `session_id`. Write the
handoff, end your turn, and the supervisor starts your successor. It also stops the chain — on a
permission denial, a token ceiling, `--max-sessions`, or no progress — so a link that quietly achieves
nothing does not get repeated six times.

Pass the `session_id` from your prompt to **every** `fleet_budget()` call. At your turn 1 the newest
top-level records on disk are still your *predecessor's*; an unpinned call reads that 400K context and
rotates you before you have done anything.

You may also be started by hand, with no supervisor. Then Step 1.5 ends the way it always did: write
the handoff and stop.

**Under a supervisor, "end your turn" means end the LINK — so never do it while workers are still
running.** `fleet_run.py` waits on your process exit, so ending your turn mid-wave ends the link with
its workers' output unread and, if `waves_done`/`issues_landed` have not moved, stops the chain as
no-progress. You must therefore stay in-turn until the wave is reviewed and merged. Two tempting ways
to wait are unavailable: `Monitor` is **not** in `DEFAULT_ALLOWED_TOOLS` (calling it is a denial,
which halts the chain), and foreground `sleep` is blocked by the harness. What works is the granted
venv interpreter as a blocking wait, issued as its own Bash call:

```
<abs venv python> -c "__import__('time').sleep(480)"
```

No `;` (a semicolon even inside the quotes risks reading as a compound command), no redirection, one
command. Between blocks, check side effects — `git -C <wt> status --porcelain` and the Step 0
tripwire — rather than inferring liveness from transcript size.

### Bash calls under supervision — no compound commands, no ops outside the repo

Verified live 2026-08-14 (`permcheck-20260814`/`-b`/`-c`: three real `claude -p` probes run under
`fleet_run.py`'s own allowlist, not a reading of documentation) — two mechanics that cost real runs
before they were pinned down, both fixed the same way: **split into separate Bash calls.**

- **A compound command (`&&`, `;`) is denied even when every piece is individually allowed.**
  `mkdir -p X && rm -rf X` was denied though both `mkdir` and `rm` are granted on their own; so was
  `cd <dir> && <command>` — the exact pattern this skill used to recommend for pytest, until today.
  No allowlist entry fixes this; there is no `Bash(&&:*)` to grant. Issue `cd <dir>` and the
  following command as **two separate Bash calls** instead. Working directory persists between
  them — verified: a `.venv/bin/python` call issued right after a bare `cd host` ran correctly in
  `host/`, with no error. A single command with a pipe (`| cat`, `| tail`) or a stderr merge
  (`2>&1`) is fine on its own; it is specifically chaining with `&&`/`;` — or `cd` as half of
  one — that gets denied.
- **A destructive or output-redirecting op targeting a path outside the repo is denied regardless
  of the allowlist.** `rm -f /tmp/x` was denied even with `Bash(rm:*)` granted, while
  `rm -f <path inside the repo>` succeeded in the same probe. File redirection via `>` was denied
  both outside the repo (`/tmp/x`) and inside it (`.fleet/x`) — `>` redirection is unreliable
  everywhere; use the `Write` tool for anything that needs to persist, not shell redirection. If
  you scratch a file under `/tmp`, do not plan to `rm` it afterward under supervision — leave it
  (harmless, untracked) or scratch inside the repo instead, where cleanup actually works.

### The handoff file — `.fleet/<run-id>.md`

Gitignored, local to this machine. **Deliberately outside `.claude/`**, where this lived until
2026-08-13: an unattended session cannot `Write` anywhere under that tree even when its allowlist
grants `Write`, and no path-scoped rule lifts it — measured three ways, recorded in `fleet_run.py`'s
`FLEET_DIR` note. A handoff the next session cannot write is not a handoff.

**Carry only what cannot be reconstructed** — Step 0's four
commands already recover the claims (`gh issue list --label status/in-progress`), the worktrees and
branches (`git worktree list`), and what merged (`git branch --merged main`). So the file holds:

```
run_id, session_chain[]              # append your session id on every rotation
ceiling_pct / observed_week_pct / observed_block_pct    # the anchors as declared at Step 1
budget_at_rotation                   # five_hour + seven_day weighted_tokens, for the run's delta
mcp_tools_available                  # which fleet_*/run_tests/ui_* this session actually had
waves_done
claims[]        {issue, worktree, branch, state: claimed|merged|yielded|parked, model}
vetoes[]        {issue, reason}      # so the next session does not re-derive them
step_8_5_pending[]  {issue, check, tool_required}
doc_deltas_collected[]               # union so far; Step 9 applies them once
memory_candidates[]                  # REQUIRED -- see below
orchestrator_code_edits              # count, for the ledger
```

**Plus one fenced `run-state` block, which is the only part a supervisor reads.** Write it even when
no supervisor started you — it costs four lines and it is what makes the run resumable by one.

````
```run-state
run_id: fleet-20260813-1600
run_state: rotated        # rotated -> spawn my successor | complete -> stop | halted -> stop
link: 2                   # your position in the chain
session_chain: [<uuid>, <uuid>]
waves_done: 3
issues_landed: [170, 171]
issues_open: [178]
halt_reason:              # required when run_state is halted; name the tool or the person
```
````

Two ways a chain stops that are easy to trip by accident:

- **No block at all** stops it, because a finished run and a crashed one are indistinguishable from
  outside the session.
- **`waves_done` and `issues_landed` both unchanged** stops it as no-progress. If you deliberately
  spend a link on something that advances neither — a long review round-trip, a doc sweep — set
  `run_state: halted` and say so, rather than leaving a successor to repeat you.

`run_state: complete` is a claim that the run is done **and** that you ran `session-end`. It is the
only end state the supervisor treats as success.

**`memory_candidates` is not optional.** `session-end` records *this* session's memory, so a rotated
fleet loses every earlier wave's lessons unless the handoff carries them forward. Rotation without
that field actively destroys what `session-end` exists to preserve.

**Only the last session runs `session-end`.** Intermediate rotations write the handoff and stop — no
memory write, no commit, no label drop. The `Stop` hook does not interfere: it fires only when HEAD
carries `Closes #NNN`, and an intermediate session ends at a worker's `Refs #NNN` ff-merge. Do not
"fix" that.

**If the fresh session lacks an MCP tool its claims depended on**, it must not land them unverified
(Step 1 already says tool availability decides what may be claimed at all). Park those issues via
`operator-request` with `needs/eyes` or `needs/hardware`, drop `status/in-progress`, and record them
in `step_8_5_pending`. Rotation *helps* in the other direction: an agent definition written in one
session is live in the next.

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

**Held issues are already excluded — do not cross-check `operator_queue()` to filter the batch.**
`excluded_reason()` vetoes `needs/operator` before scoring (#177), so held issues arrive in
`plan["excluded"]` labelled with their subtype. The instruction that used to live here — *"the
planner does not"* — described the tracker before that fix and is no longer true;
`test_needs_operator_is_excluded_so_the_skill_need_not_cross_check` pins the two together. Call
`operator_queue()` for its **`problems`** field, which the planner genuinely cannot see (held issues
whose runbook comment is missing or unparseable), or when you are about to *write* a request.

**If you do run a `gh` probe, use `--json`.** `gh issue view N --comments -q '...'` fails with
*"cannot use `--jq` without specifying `--json`"*, and with `2>/dev/null` on the end it prints
nothing at all — indistinguishable from "no comments," which is how a planner defect got wrongly
filed against #158. Use `--json comments -q '.comments | length'`, and never `2>/dev/null` a probe
whose emptiness is your evidence. The full story is in `has_prior_work()`'s docstring in
`host/tools/fleet_plan.py` — the file anyone hunting that bug will already have open.

**A worker's "I could not reproduce this" is data, not an obstacle.** That worker refused to invent
a mechanism, hardened the invariant instead, and said plainly it could not confirm the root cause.
It was right and the orchestrator was wrong. Re-run your own probe before overriding it.

**Check the data the issue needs actually exists on this box.** #158 also wanted RTAB-Map exports:
the rtabmap checkout's `data/samples.zip` is bag-of-words vocabulary imagery, not an export, and the
paired capture (#161) is `status/blocked`. Its coordinate-convention half — which its own body flags
as the BUG-051/058 failure mode — had nothing to validate against. `ls` the directory before
claiming. **And the mirror image: a recorded veto goes stale when the plan is superseded** — that
same #158 veto was correct on 2026-08-12, then a newer plan re-scoped the issue to fixture-level
validation and it landed cleanly on 2026-08-13. Re-triage against the *latest* plan comment before
re-applying a veto from a prior run or from this file's examples.

**Read the `triage` digest of every issue you are about to claim. A high score is not
actionability.** The planner ranks on labels, prior work and footprint — it cannot see a sentence,
so each selected issue now carries a bounded digest that brings the sentence to you: `plan_excerpt`
(the **latest** plan comment, not the first — plans get superseded), `latest_comment` with its
`kind`, `body_excerpt`, `acceptance_hint` and `chars_elided`. **Read that instead of running
`gh issue view`.** `fleet_plan` already fetched every open issue's full body and comment thread in
one call, so a per-issue `gh` loop re-pays for text you are holding — measured on the live tracker,
974 tokens carried where the loop pulled ~19,300, and a token spent here is re-sent on every
remaining turn of the run. On 2026-08-12 two of
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

`triage.acceptance_hint` pre-sorts exactly this: `visual` needs the browser, `hardware` needs the
rig, `pytest` is the shape you want. If your session lacks the tool the hint names, that is the
veto — and you know it before spawning anything, rather than at Step 8.5 with a branch already
merged.

**But confirm the hint before honouring it — it false-positives, and in the expensive direction.**
On 2026-08-14 #81 came back `acceptance_hint: "hardware"` and is **replay-only**: the capture it
needs (`captures/imuTranslationError.bin`) was already on disk and `host/tools/slam_ensemble.py`
runs it with no rig. The hint fired on prose that *mentions* the rig, not on what the acceptance
requires. That tier held three issues, so honouring it would have vetoed the run's entire workable
pool and produced an empty wave. **Before accepting a `hardware` or `capture` hint, `ls captures/`
for the file the issue names — if the capture already exists, the work is replay and the hint is
wrong.** This is the same shape as the co-edit false conflict below: a heuristic that suppresses
workable issues silently costs you more than one that lets a bad issue through, because a suppressed
issue never reaches your review.

**And an ungranted MCP tool is not an absent capability.** Under `fleet_run.py`'s scoped allowlist a
link typically gets only `fleet_plan`, `fleet_budget`, `run_tests`, `operator_queue` and `doctor`.
That does **not** veto every issue whose verification is normally an MCP call: most of these tools
are a thin wrapper over a pure function that also has a CLI under `host/tools/` (`AGENTS.md`'s "one
implementation, two front ends"), and the allowlist grants the venv interpreter. `slam_ensemble` the
**tool** was ungranted while `host/tools/slam_ensemble.py` ran fine and produced both workers' n=8
ensembles. `ls host/tools/` before vetoing. The veto genuinely holds only for `ui_*` browser checks
and anything that must go through the long-lived `roomscan-web` process, which binds the device.
Keep using `run_tests()` as MCP even so — it keeps a 2500-test transcript out of your context.

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

**A `deferred` file conflict is a hypothesis, not a fact — check it before accepting it.** The
conflict test runs on the **expanded** footprint, and expansion is a co-edit correlation over
`git log`, not a statement about the change in front of you. On 2026-08-13 #175 was deferred with
`conflicts_with: [159]` when the two share **zero** files: #175 is `host/tools/operator_queue.py`,
#159's stated seeds are all `host/src/roomscan/splat/**`, and #159's expansion had ballooned to 32
files, sweeping in unrelated `host/src/roomscan/mcp_server/` modules that merely co-occurred in old
commits. #175 then
landed alone, clean, first try. Each `footprint` entry is tagged `seed` / `sibling` / `coedit`, and
**only `seed` is a claim about the actual change** — so compare the two issues' seed-tier files, and
overrule a conflict that rests only on `coedit` entries. This is the mirror of the veto advice
below, and the more expensive direction: a wrong veto you can see in the batch, while a wrongly
deferred issue never reaches your review at all.

Where the planner reports a soft conflict, **assign the contested file exclusively** instead of
hoping. Name it in both spawn prompts — "X owns `foo.py`; you stop and report rather than edit it" —
and in the claim comments. Cheap, and it converts a gamble into a rule. Note the footprint model
does not see **shared test files**: two workers appending to the same `test_*.py` is common and only
shows up at the second one's rebase.

## Step 2.5 — Delegate the deep read; never do it yourself

When `triage.chars_elided` is large and the digest has not settled whether an issue is workable,
**spawn an `Explore` agent** rather than reading the thread. Its context is discarded when it
returns; yours is re-sent on every remaining turn of the run, so the same paragraphs cost you a
hundred times what they cost it.

Give it the issue number, the digest you already hold, and this response contract:

> At most **120 words per issue**. Exactly these fields: `verdict` (`workable` | `veto` | `unclear`),
> a one-sentence reason, the acceptance check and whether it can be run **today** with the tools this
> session has, and citations as comment dates. **Do not paste issue text.**

Use the built-in `Explore` type, not a new `fleet-triage` agent definition:

- **Agent definitions load at session start**, so a definition written now is invisible until the
  next session — including to the session that wrote it. `Explore` works on the first run.
- Workers "never run `gh` and never call an MCP tool" is a four-incident invariant enforced by
  `test_agents_are_denied_gh_and_mcp`. A triage worker would need `Bash` for `gh`, which either
  carves an exception into that rule or forks the test. `Explore` is not a fleet worker, so the
  sentence stays literally true.
- `Explore` is read-only **by construction** — no `Edit`, `Write`, or `Agent` — rather than by a
  prose promise of the kind that failed on 2026-07-10.

If `Explore` is unavailable in this session, fall back to `subagent_type: "claude"` with
`model: sonnet` and paste the contract into the prompt. The contract is the load-bearing part.

**Expect the total not to move much.** Explore's spend lands in the `subagent` seat; what drops is
the `top_level` seat, which is the one multiplied by every remaining turn. Read
`seven_day.by_seat`, not `seven_day.weighted_tokens`, or this will look like it did nothing.

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
gh issue view NNN --repo hellosamblack/lidar-roomscanner --json comments \
  -q '[.comments[] | select(.body | test("\\*\\*Session start\\*\\*")) | {createdAt, login: .author.login}]'
```

**This one stays** — it is the only `gh issue view` the triage digest cannot replace, because it
must read the tracker *after* your write to see a racing claim. But it is scoped to the claim
timestamps: ~100 bytes instead of the whole thread.

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
- `git -C <abs worktree>` for every git call. For pytest: `cd <abs worktree>/host` as its own Bash
  call, then the pytest invocation as a **separate** call — never `cd X && pytest`. See "Bash calls
  under supervision" above.
- Commit `Refs #NNN` on the worktree branch. **Never `Closes`** — fast-forwarded to HEAD it trips the
  Stop hook mid-fleet.
- **Explicit pathspecs, never `git add -A`.** The two symlinks show as untracked, and a multi-path
  `git add` aborts entirely and silently if one pathspec matches nothing.
- Forbidden: `ROADMAP.md`, `AGENTS.md`/`CLAUDE.md`, `BUGS.md`, `docs/**`, `.remember/**`,
  `.claude/**`, the issue migration map. Report intended doc changes as a `doc_deltas` block; you
  apply them once, at the end. `.claude/**` is on that list because it holds the agent definitions —
  including `fleet-worker.md`, a worker's own contract. Since definitions load at session start, a
  worker rewriting it would take effect on the *next* wave rather than failing visibly.
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
cd <wt>/host                                                # its OWN Bash call -- see below
/path/to/main/host/.venv/bin/python -m pytest -q --no-header --tb=line -k <narrow> 2>&1 | tail -n 20
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

Run the full suite once after all merges, not per branch — **via `run_tests()`, not Bash.** It
already returns `counts`, `failures[:50]`, a one-line `summary`, and `tail` only when the run failed;
piping 2000+ tests through Bash puts the whole transcript in your context, where it is re-sent on
every remaining turn. (`--tb=line | tail -n 20` is the Bash fallback when MCP tools are absent —
`--tb=line` is the load-bearing half, turning multi-KB tracebacks into one line each.) `run_tests`
always means the **main** checkout; that flatness is deliberate, so do not reach for a worktree
parameter. Then run `session-end` **once** for the
whole fleet: it records memory, posts one comment per claimed issue, runs `status-sync` over the
union of every worker's `doc_deltas`, and lands a single commit carrying every `Closes #NNN`. One
closing commit at HEAD, one marker write, and `session-end`'s own invariants hold unmodified.

Finally, drop `status/in-progress` from every issue you claimed — including the ones you yielded or
deferred mid-run.

Then append one row to **`docs/fleet-ledger.md`**, differencing the `fleet_budget()` call you made at
Step 1 against one now. `orch_share` (`weighted_top_level / weighted_total`) is the headline; fill in
`confounds` honestly, because a blank there is a claim that nothing changed. Read that file's header
before drawing any conclusion from it — the per-run spread in this project is ~7x, so it supports a
within-arm trend and paired comparisons, and **not** a model-vs-model A/B.

## What you may write

**You coordinate; you do not author.** Your legitimate `Edit`/`Write` targets are exactly:

- comment/claim body files for `gh --body-file`,
- the run handoff file,
- the union of workers' `doc_deltas` at Step 9 — the files workers are *forbidden*,
- the ledger row,
- whatever `session-end` writes in its own phases,
- conflict hunks during `git rebase` inside a worktree. That is landing, not authoring.

**Nothing under `host/src/`, `host/tests/`, `host/tools/`, `firmware/` or `host/transform/` is
yours.** A review-gate finding goes **back to the same worker** — it still holds the context, and
re-deriving it in your session pays for that context on every remaining turn. If no worker holds it,
that is a new claim and a new spawn, not an inline fix. `suggested_model: orchestrator-only` is a
veto, not a licence to write the code yourself.

One session made **57** such edits. They are counted into the ledger's `orch_code_edits`, so if this
rule is not working the ledger will say so.

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
- An `Edit` or `Write` from **you** onto a file under `host/src/`, `host/tests/`, `host/tools/` or
  `firmware/`. Count them: one session ran 57.
- `verdict: "rotate"` treated as advice, or a new wave started after one.
- A `gh issue view --comments` you ran when the issue's `triage` digest was sitting in the plan you
  already had.
- A full-suite run whose output landed in your context instead of `run_tests()`'s counts.
- Judging this change by `seven_day.weighted_tokens`. Delegating and rotating move spend *between*
  seats; only `by_seat` can show they worked.
