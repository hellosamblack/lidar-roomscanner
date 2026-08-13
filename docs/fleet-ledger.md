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

## Runs

| run_id | date | orchestrator_model | sessions | waves | claimed | landed | weighted_total | weighted_top_level | weighted_subagent | orch_share | max_context | orch_code_edits | confounds |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| _(baseline, pre-#182)_ | 2026-08-13 | claude-opus-5 | — | — | — | — | 593.8M | 560.1M | 33.7M | **94.3%** | 660K | 57 (one session) | trailing-7d snapshot across all sessions, not a single run; no rotation gate, no triage digest, orchestrator authored code freely |
