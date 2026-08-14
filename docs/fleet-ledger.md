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
