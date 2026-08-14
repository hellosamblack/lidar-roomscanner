# Fleet run ledger

One row per `issue-fleet` run, appended by the orchestrator at Step 9. Produced by
differencing two `fleet_budget()` calls — one at Step 1, one at Step 9 — so every number
here is a **measured delta**, not a re-forecast.

Started 2026-08-13 (#182), when the orchestrator seat first became measurable.

## What this can and cannot tell you

**Read this section before drawing a conclusion from the table.** A ledger that implies a
comparison it cannot support is the failure mode `fleet_budget.py`'s own calibration note
warns about — a fabricated number that reads as authoritative.

**It cannot deliver a clean model-vs-model A/B.** The measured per-session spread in this
project is ~7x (23.5M–166.9M prompt tokens across six sessions), and `fleet_budget`'s own
notes record ~4x between a 3-worker and a 2-worker wave. With a coefficient of variation
near 1, detecting a 30% difference between two orchestrator models at any useful
confidence needs *n* in the dozens. **Five runs per arm will not do it, and neither will
ten.** Do not read two rows with different `orchestrator_model` values as a result.

**It can deliver three things:**

1. **A within-arm trend on `orch_share`** (`weighted_top_level / weighted_total`). This
   needs no control group, and it is exactly the quantity #182 set out to move. Usable
   after ~3 runs. Baseline measured 2026-08-13, before any of #182's changes were
   exercised by a real run: **94.3%** of trailing-7d weighted spend was the orchestrator
   seat; workers were 5.7%.
2. **A paired comparison** — but only between runs with the same wave size, the same
   rotation threshold and the same triage path. That is what `confounds` is for. Fill it
   in honestly; a blank `confounds` on a run that changed two things at once is worse
   than no row.
3. **`weighted_orch_per_landed_issue`** as the normalised metric. It removes the largest
   single variance source — how much work a run happened to do — and is the fairest
   number in the table.

**A caution that applies to every row.** Delegating work and rotating sessions both *move*
spend between seats as well as reducing it: an `Explore` agent's tokens land in
`weighted_subagent`, and rotation's saving is concentrated in `weighted_top_level`. Anyone
reading `weighted_total` alone will see a much smaller drop than expected and conclude the
change failed. `orch_share` is the column that answers the question.

## Columns

| column | meaning |
|---|---|
| `run_id` | the `fleet-YYYYMMDD-HHMM` id from Step 1 |
| `orchestrator_model` | the model the orchestrating session ran on |
| `sessions` | how many orchestrator sessions the run spanned (>1 means it rotated) |
| `waves` | worker waves dispatched |
| `claimed` / `landed` | issues claimed vs actually merged to `main` |
| `weighted_total` | Step 9 minus Step 1, `seven_day.weighted_tokens` |
| `weighted_top_level` | the orchestrator seat, from `seven_day.by_seat` |
| `weighted_subagent` | every worker and Explore agent |
| `orch_share` | `weighted_top_level / weighted_total` — **the headline** |
| `max_context` | highest `rotation.context_tokens` seen at any wave boundary |
| `orch_code_edits` | orchestrator `Edit`/`Write` calls onto `host/src`, `host/tests`, `host/tools`, `firmware/` — should be 0 |
| `confounds` | anything that makes this row non-comparable. Blank means "nothing changed" and is a claim |

## Supervised runs (`fleet_run.py`, #183)

When a run is chained by `host/tools/fleet_run.py`, most of the row is already written for
you: `.fleet/<run-id>.chain.json` holds one record per link — session id, turns, duration,
permission denials, the weighted delta measured across that link, and the decision that
ended it. `sessions` is `len(links)`, and `weighted_total` is the supervisor's own sum.

**Rotation is not free, and this table is where that shows up.** Every link re-caches a
fresh system prompt — the measured floor is ~45K raw, so an Opus link starts ~250K weighted
in debt before it does anything, and a 4-link run pays that four times. The saving it buys
is `turns × (C − B)`, which at a 300K rotation point is ~20M weighted over the following
wave. The ratio is one-sided at the thresholds we use, but it is a ratio, not a law: a
policy that rotated every 30 turns would spend more on fresh contexts than it recovered.
Record `sessions` honestly so the trend can be read against `orch_share`.

Two entries in `confounds` matter for chained runs specifically, because both change what
a link *is*: the `--max-sessions` value the run was launched with, and any `--allow-add`
granted after a `denied` stop. A run that halted on a denial and was relaunched is two
partial runs sharing a `run_id`, not one run — say so.

Measured overhead, 2026-08-13 (`smoke-chain`, two Haiku links, chain mechanics only, no
fleet work): 8,980 and 8,834 weighted tokens per link, 23s and 24s, 3 turns each. That is
the floor a link costs before it does anything — the supervisor's own tax, on the cheapest
model.

## Runs

| run_id | date | orchestrator_model | sessions | waves | claimed | landed | weighted_total | weighted_top_level | weighted_subagent | orch_share | max_context | orch_code_edits | confounds |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| _(baseline, pre-#182)_ | 2026-08-13 | claude-opus-5 | — | — | — | — | 593.8M | 560.1M | 33.7M | **94.3%** | 660K | 57 (one session) | trailing-7d snapshot across all sessions, not a single run; no rotation gate, no triage digest, orchestrator authored code freely |
| fleet-20260813-1845 | 2026-08-13 | claude-fable-5 | 1 | 1 | 2 (#158, #178) | 2 | ~3.4M | ~0.95M | ~2.44M | **~28%** | 141K | 0 | totals are per-model 7d deltas (fable=orch, sonnet=workers/reviewer/triage), because the 5h block aggregate was polluted by a concurrent opus-5 session; rolling-window roll-off may understate; weekly pct unanchored (no owner figure); one review round-trip on #158; full suite once via run_tests |
| fleet-20260813-2118 | 2026-08-13 | claude-opus-5 | 1 | 1 | 1 (#175); #159 held, not claimed | 1 | 3.53M | n/a | +0.54M | **n/a** | 106K | 0 | **seat split unusable this run — do not read the blanks as zero.** The two `fleet_budget()` calls returned a *negative* `five_hour.by_seat.top_level` delta (4.79M → 4.03M) and a negative `by_model` opus-5 delta, i.e. the composition report was recomputed differently between calls (`duplicate_records_skipped` moved 21,718 → 21,800); only `weighted_tokens` differenced coherently. The 18:00 block also held 21.8M of pre-run activity, so its composition snapshot is not this run's either. Owner-directed 2-issue run, not a backlog sweep, so the batch was not planner-selected. 3 subagents (2× Explore triage, 1× Sonnet worker), 0 review round-trips, full suite once via run_tests (2473 passed, 0 skipped) + one targeted re-run after a skill-guard failure |
| fleet-20260814-0157 | 2026-08-14 | inherit (session default) | 1 | 0 | 0 | 0 | 179,915 | 179,915 | 0 | 100% | n/a | 0 | **first real supervised chain run (#183), max-weighted 40M / max-sessions 3.** Link 1 halted on **denied** before claiming anything — `gh` came up unauthenticated because the supervisor was launched as root (`$HOME=/root`) while `gh` was only ever authenticated for `sam`; the link burned its budget diagnosing that (`env`, `printenv`, `ls ~/.config/gh` — outside the allowlist, correctly denied) rather than doing fleet work. Chain-mechanics-only spend; not comparable to a landed-work row. |
| fleet-20260814-0322 | 2026-08-14 | inherit (session default) | 1 | 0 | 0 | 0 | 303,648 | 303,648 | 0 | 100% | n/a | 0 | Retry with `GH_CONFIG_DIR` exported as a manual patch. 0 denials this time, but link 1 halted on a *new* stop reason — **scope-emptiness**: the only two `priority/now` issues (#95, #60) are both `status/blocked` + `needs/operator`, `fleet_plan` confirmed 0 candidates. Correct behavior (no pointless rotation), but only 1 session ran, so **#183's core claim — 2+ sessions chaining unattended on real work — is still unverified**. Also surfaced a second `$HOME` split from the same root-launch cause: auto-memory landed under `/root/.claude/projects/.../memory/`, a path `sam` cannot even read back. Fixed same day: `resolve_run_env()` in `fleet_run.py` now corrects `$HOME` to the repo checkout's owner for every spawned link (root, sam, or anyone), which fixes both the `gh` auth split and the memory split in one place and retires the manual `GH_CONFIG_DIR` workaround. |
| fleet-20260814-1204 | 2026-08-14 | inherit (session default) | 1 | 1 | 1 (#81, not landed) | 0 | 0* | n/a | n/a | n/a | n/a | 0 | **`$HOME` fix in place, `priority/next` brief — the first run to actually claim an issue and spawn a worker.** Link 1 claimed #81, spawned a Sonnet worker (Agent()), and ran 755s / 27 turns before halting on **denied**. `*weighted_delta reads 0.0 despite real cost (cost_usd 0.53) — a rolling-window artifact like fleet-20260813-2118's negative delta, not a free run; do not read as zero spend.` Two distinct denial causes: (1) a genuine allowlist gap — the worker's cwd is its own worktree, so once it `cd`'d into `host/` its relative `.venv/bin/python` didn't match the `host/.venv/bin/python:*` rule, and it burned turns hunting a fallback (`python3 --version`, `-c`, `-m pytest`, all also denied) that never landed; fixed same day (`Bash(.venv/bin/python:*)` added). (2) at least one denial (`rm /tmp/probe_graft.py`) looks like the auto-mode classifier acting independently of the allowlist — `Bash(rm:*)` was already granted, so no `--allow-add` entry explains it; the worker adapted by neutering the file instead of deleting it. Separately, the run left the worktree **owned by root** (supervisor launched as root) — `$HOME` correction fixes config resolution, not file ownership, so a follow-on `sam`-run session can't operate on it (`dubious ownership`). Owner decided (2026-08-14) to run future links as `sam` rather than keep patching root's side effects. Cleanup: `status/in-progress` dropped from #81 manually; worktree required a root shell to remove (owned by the process that created it). No committed work — nothing lost. |
| fleet-20260814-1503 | 2026-08-14 | inherit (session default) | 1 | 0 | 0 | 0 | 153,674 | 153,674 | 0 | 100% | 48.3K | 0 | **First run launched as `sam`, per the owner's 2026-08-14 decision.** Root-vs-sam file ownership was gone, but a *new* `$HOME`-adjacent split appeared: the owner's global `~/.claude/settings.json` installs a `PreToolUse` hook (`rtk`, a token-optimization CLI) that transparently rewrites every Bash command (`git status` → `rtk git status`) before the allowlist checks it — so every `Bash(git:*)`/`Bash(gh:*)`/etc rule in `DEFAULT_ALLOWED_TOOLS` silently stopped matching, and even trivial reconnaissance (`git worktree list`) was denied. Root-launched links never hit this because root's `$HOME` has no such hook. Fixed: `build_argv()` now passes `--setting-sources project,local`, excluding "user" (where the hook lives) while keeping what a link needs — `orch-edit-count.sh`, `session-end-guard.sh`, the repo's own `gh issue *` grants (all "project"/"local"). Link 1 also hit a **real, pre-existing code bug**, not a process-staleness artifact as two prior sessions had assumed: the MCP wrapper `tools_fleet.fleet_plan()` always calls `plan_fleet_live(..., triage=triage)`, but `plan_fleet_live()` neither accepted nor forwarded `triage` — every live call raised `TypeError`. No test called the live wrapper (every `fleet_plan` test in `test_fleet_plan.py` calls `plan_fleet` directly), so this shipped uncaught across fleet-20260813-2118 and fleet-20260814-1204's CLI-fallback attempts, both of which reported it "unpatched" rather than fixed. Fixed here with a regression test at the live-wrapper level. Link 1 halted at Step 2 with nothing claimed, per design — correctly wrote `run_state: halted` rather than guessing around the denial. |
| fleet-20260814-1552 | 2026-08-14 | inherit (session default) | 1 | 1 | 2 (#81, #180; both not landed) | 0 | 3,499,142 | 3,499,142 | 0 | 100% | n/a | 0 | **Owner uninstalled the `rtk` hook entirely (`~/.claude/settings.json` now `"PreToolUse": []`); this run's own `--setting-sources` fix was already redundant with it but harmless.** Furthest yet: `fleet_plan` worked (triage fix confirmed live), planned a thin batch (3 of 58 open), correctly vetoed #84 as orchestrator-only (posted as a comment, left unclaimed — see the issue), claimed #81 and #180, spawned two parallel workers, ran 956s/35 turns/$3.64 — **all of it burned re-discovering the same allowlist gap in new forms.** `Bash(host/.venv/bin/python:*)`/`Bash(.venv/bin/python:*)` name one binary in one of its invocation forms each; this run hit a *third* form (an absolute path, used because a worker's cwd wasn't always the venv's own worktree) and a *fourth* (`host/.venv/bin/pytest`, a different console script in the same directory) — neither covered by a `python`-specific rule. **Root cause of the repeat cost: the allowlist was enumerating invocation forms of one binary instead of trusting the directory.** Fixed by anchoring three rules on the `.venv/bin/` *directory* (relative-from-root, relative-from-`host/`, absolute via `REPO`) rather than the `python` binary specifically — covers every script in the repo's own venv regardless of path form. Both worktrees held zero commits and zero real diffs (only the two symlinks) when discarded — nothing lost, but $3.64 and 956s of worker time spent proving the same category of gap three separate times across two runs is the real cost of narrow, literal-prefix allowlist entries. Cleanup (worktrees, branches, `status/in-progress` on #81/#180) done directly as `sam` — no root-ownership friction this time. |
| permcheck-20260814 / -b / -c / -d | 2026-08-14 | claude-haiku-4-5 | 4 (1 each, `--task` diagnostic probes, not real fleet work) | 0 | 0 | 0 | ~65,000 total | ~65,000 | 0 | 100% | n/a | 0 | **Not fleet work — a live permission-matcher probe, run after the owner asked for proof this class of failure was actually fixed, not just re-patched.** Cheap (Haiku, `--task` override, `--max-sessions 1`) and decisive: found the last two fixes were themselves wrong. (a) `Bash(host/.venv/bin/:*)` — the directory-only form from fleet-20260814-1552's fix — matched **nothing live**; even the previously-working `host/.venv/bin/python -c "print(1)"` was denied under it. Reverted to naming each script explicitly across all three anchor forms (`VENV_BIN_SCRIPTS` × 3 anchors, 6 rules) — verified GRANTED in probe `-b`. (b) `mkdir -p X && rm -rf X` was denied though both halves are individually allowed — confirmed **compound `&&`/`;` commands are denied independent of the allowlist**, no rule can grant this; root-caused to `.agents/skills/issue-fleet/SKILL.md` (2 places) and `references/worker-brief.md` + both `.claude/agents/fleet-worker*.md` (3 more places) all instructing exactly this shape (`cd <wt>/host && pytest ...`) — the actual source of most of `-1204`'s and `-1552`'s wasted turns, not the allowlist at all. Fixed all 5 locations to split into two Bash calls; probe `-b` confirmed cwd persists across the split (`cd host` then a separate `.venv/bin/python` call, GRANTED, correct cwd). (c) Also found and corrected a wrong prior inference: `-1204`'s `rm /tmp/probe_graft.py` denial had been attributed to "the auto-mode classifier acting independently" — probe `-c` shows the simpler, confirmed explanation: **any op (rm, `>` redirect) targeting a path OUTSIDE the repo is denied regardless of the allowlist**, no separate classifier needed to explain it. Probe `-d`: the corrected split-call pattern, run end-to-end with a real (cheap) pytest selection — **0 denials.** First run in this whole investigation with zero surprises. |
| fleet-20260814-1716 | 2026-08-14 | claude-opus-5 | 1 | 1 | 2 (#81, #180) | 1 (#81's test; #180 no commit) | 23.96M | 6.59M | 2.75M | **27.5% (see confounds — do not compare)** | 187K | 0 | **The seat split accounts for only 9.34M of the 23.96M total, so this row's `orch_share` is not comparable to any other row.** `weighted_total` is the ccusage block aggregate; the seats are the record-level composition report, and `fleet_budget`'s own `basis` field says the two do not reconcile. Read literally (top_level/total) it is 27.5%; read composition-internally (top/(top+sub)) it is **70.5%** — a 43-point spread on the headline column, so this run supports neither a win nor a regression on `orch_share`. Real, non-ambiguous measurements from this run: subagent seat 2.75M (identical under both the 5h and 7d bases), max context 187K, `rotation.call` stayed `hold` throughout. Other confounds: the 7d rolling window's start moved 46 min between the two calls (roll-off understates); the weekly percentage was **unanchored** (owner left `observed_week_pct` unset at launch, so every verdict covered the 5h block only); wave of 2, not 3, because the `priority/next` tier held only 3 candidates and one was an `orchestrator-only` veto; **both** workers returned negative results, so this row's cost bought diagnosis rather than merged code, and `weighted_orch_per_landed_issue` is misleading here for that reason. First supervised run with the corrected allowlist: **0 permission denials end to end** (the `-1204`/`-1552` failure mode is fixed). Chaining still unexercised — the link ended `complete`, not `rotated`, because the tier was exhausted in one wave. |
