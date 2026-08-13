#!/usr/bin/env python3
"""Estimate Claude usage against an owner-declared ceiling, for pacing an agent fleet.

There is no quota API available to an agent on this box. The `claude` CLI has no
`usage`/`limits` subcommand; the authoritative source -- `GET /api/oauth/usage` and
the `anthropic-ratelimit-unified-*` response headers -- lives in CLI process memory
and is never written to disk. What *is* on disk is per-message token accounting in
`~/.claude/projects/**/*.jsonl`, which is what `ccusage` parses and what this module
falls back to.

So every number here is an estimate, and the module is built so that it says so:

* **`limit_basis` is non-empty on every return path.** A percentage whose denominator
  is invented is a fabricated number. Max 5x publishes no token budget, so the
  denominator is either owner-supplied or this account's own historical peak block --
  and the caller is told which.
* **Failure returns `verdict: "unknown"`, never a number and never `"go"`.** A
  renamed key in ccusage's output must not read as a healthy 0%.

Two measurements pin the implementation, both verified against the live store:

1. **Subagent spend lives one directory deeper.** Worker transcripts are written to
   `projects/<proj>/<session-uuid>/subagents/agent-*.jsonl`; a flat `projects/*/*.jsonl`
   glob misses all of it. For a fleet -- where the workers *are* the spend -- that is
   not a rounding error. ccusage does cover it; this module's own reader must too, and
   `coverage` reports which files were actually read.
2. **The store contains duplicate records.** The same `(message.id, requestId)` pair
   recurs, and summing naively overcounts by ~80% (185.8M vs 102.8M weighted tokens on
   one 5h block). With dedup and both globs, a native read reproduced ccusage exactly
   on entry count and on three of four token fields.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import Counter
from datetime import UTC, datetime, timedelta
from pathlib import Path

#: Pinned so a ccusage release cannot silently change the schema under a running fleet.
#: Verified against this version on 2026-08-12; `parse_ccusage_blocks` raises rather
#: than guessing if a bump renames a field, which degrades to verdict "unknown".
CCUSAGE_PKG = "ccusage@20.0.19"

PROJECTS_ROOT = Path.home() / ".claude" / "projects"
TOP_LEVEL_GLOB = "*/*.jsonl"
SUBAGENT_GLOB = "*/*/subagents/*.jsonl"

BLOCK_HOURS = 5
WEEK_HOURS = 168  # rolling, not calendar -- see `rolling_window`

#: THIS MODULE IS A RELATIVE METER, NOT AN ABSOLUTE ONE. Calibrated against the owner's
#: own reported figures on 2026-08-12, and the attempt is what proves the point:
#:
#:   moment          weighted tokens   this tool   owner   => implied 5h allowance
#:   session start        96,234,076       63.4%     85%            113,000,000
#:   ~8.5h later          40,938,630       27.0%     22%            186,100,000
#:
#: Two readings of the SAME window, 1.6x apart. No single denominator reconciles them,
#: so the error is not in the denominator -- it is in the numerator's UNITS. The
#: weighted token below (`CACHE_READ_WEIGHT`, `OUTPUT_WEIGHT`, the per-model table) is
#: this module's own invention and does not track however the real limit counts; change
#: the model/cache/output mix, as one wave does versus another, and the conversion
#: moves. Do not "fix" this by hardcoding an owner-calibrated limit: an absolute number
#: fitted to one work mix is wrong for the next, and it will read as authoritative.
#:
#: What IS sound is the DELTA. Over that same session the rolling-7d sum rose 581.6M ->
#: 632.1M (+50.5M) while the owner's weekly moved 70% -> 72%. A rolling window only ever
#: drops records as it slides, so a net rise is a LOWER BOUND on tokens added -- which
#: also refutes any weekly allowance small enough to make 2 points cost less than 50.5M.
#: Measured per-wave, on this project, with subagents counted:
#:
#:   a 3-worker Sonnet wave + review + one full suite + browser checks   ~= 40M weighted
#:   a 2-worker wave (one Haiku), same overheads                        ~= 10.5M weighted
#:
#: Two estimates of the cost of one weekly point disagree, and the disagreement is the
#: honest range: the whole session gives >=25.2M/point (50.5M over ~2 points, a lower
#: bound because of window slide), while wave 2 alone gives ~40M/point (~40M over the
#: 71%->72% step). We adopt the LOWER bound deliberately -- it converts a given wave into
#: MORE weekly points, so the tool errs toward stopping early rather than overrunning a
#: ceiling. At 25M/point the 3-worker wave above bills as ~1.6 points, not 1.
#:
#: So: anchor on a percentage the OWNER reports, then use this module's delta to project
#: forward from it. That is what `observed_week_pct` / `observed_block_pct` are for, and
#: why a weekly percentage is reported as `None` rather than guessed when absent.
#: Re-measure `TOKENS_PER_WEEK_POINT` whenever the plan or the typical work mix changes.
TOKENS_PER_WEEK_POINT = 25_000_000.0
CALIBRATED_ON = "2026-08-12"

#: Cache reads are an order of magnitude cheaper than fresh input, so counting them at
#: face value makes a long cached session look like a runaway. Named and testable
#: rather than buried in an expression; it is a heuristic, not a measured constant.
CACHE_READ_WEIGHT = 0.1
#: Output is materially more expensive than input across every current model.
OUTPUT_WEIGHT = 5.0

#: Relative to Sonnet. The real unified rate limit weights models differently and does
#: not publish how; these track published price ratios, which is the closest public
#: proxy. Because the default denominator is this account's own historical peak, a
#: consistent weighting largely cancels in the ratio.
MODEL_WEIGHT = {
    "claude-opus-5": 5.0, "claude-opus-4-8": 5.0, "claude-opus-4-6": 5.0,
    "claude-sonnet-5": 1.0, "claude-sonnet-4-6": 1.0, "claude-fable-5": 1.0,
    "claude-haiku-4-5-20251001": 0.27,
}
DEFAULT_MODEL_WEIGHT = 1.0

VERDICTS = ("go", "reduce", "stop", "unknown")


# --------------------------------------------------------------------------------
# Pure token arithmetic
# --------------------------------------------------------------------------------

def weighted_tokens(usage: dict, model: str | None = None) -> float:
    """One comparable number from a `message.usage` block.

    Applies the cache-read discount and the output premium, then scales by the model
    weight, so an Opus turn and a Haiku turn of equal raw size are not treated as
    equal load.
    """
    raw = (
        usage.get("input_tokens", 0)
        + usage.get("output_tokens", 0) * OUTPUT_WEIGHT
        + usage.get("cache_creation_input_tokens", 0)
        + usage.get("cache_read_input_tokens", 0) * CACHE_READ_WEIGHT
    )
    return raw * MODEL_WEIGHT.get(model or "", DEFAULT_MODEL_WEIGHT)


def _parse_ts(value: str | None) -> datetime | None:
    if not value:
        return None
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None


def read_transcripts(root: Path = PROJECTS_ROOT) -> tuple[list[dict], dict]:
    """Every usage record under `root`, deduplicated, with a coverage report.

    Reads **both** the top-level session transcripts and the nested
    `<session>/subagents/agent-*.jsonl` files, and deduplicates on
    `(message.id, requestId)` -- the store repeats records, and without dedup the
    total roughly doubles. Malformed lines and records without usage are skipped.
    """
    records: list[dict] = []
    seen: set[tuple[str, str]] = set()
    stats = Counter()

    for label, pattern in (("top_level", TOP_LEVEL_GLOB), ("subagent", SUBAGENT_GLOB)):
        for path in sorted(root.glob(pattern)):
            stats[f"{label}_files"] += 1
            try:
                text = path.read_text(errors="ignore")
            except OSError:
                stats["unreadable_files"] += 1
                continue
            for line in text.splitlines():
                if '"usage"' not in line:
                    continue
                try:
                    doc = json.loads(line)
                except (ValueError, TypeError):
                    stats["malformed_lines"] += 1
                    continue
                message = doc.get("message") or {}
                usage = message.get("usage") or {}
                if not usage:
                    continue
                ts = _parse_ts(doc.get("timestamp"))
                if ts is None:
                    stats["undated_records"] += 1
                    continue
                key = (message.get("id") or "", doc.get("requestId") or "")
                if key != ("", "") and key in seen:
                    stats["duplicate_records"] += 1
                    continue
                if key != ("", ""):
                    seen.add(key)
                # `context` is the RAW live-context size at this turn -- what the model was
                # actually handed. `weighted` cannot substitute: it discounts cache reads
                # by CACHE_READ_WEIGHT and scales by the model, which is right for load
                # accounting and destroys the growth signal. Keep both.
                rel = path.relative_to(root)
                records.append({
                    "ts": ts,
                    "model": message.get("model"),
                    "weighted": weighted_tokens(usage, message.get("model")),
                    "source": label,
                    # A subagent transcript carries its PARENT's sessionId, which is what
                    # attribution wants. The filename fallback must mirror that: a
                    # subagent's own stem is `agent-<id>`, so walk up to the session dir.
                    "session_id": (doc.get("sessionId")
                                   or (rel.parts[1] if label == "subagent" else path.stem)),
                    "project_dir": rel.parts[0],
                    "cache_read": usage.get("cache_read_input_tokens", 0),
                    "context": (usage.get("input_tokens", 0)
                                + usage.get("cache_creation_input_tokens", 0)
                                + usage.get("cache_read_input_tokens", 0)),
                })
                stats[f"{label}_records"] += 1

    coverage = {
        "top_level_files": stats["top_level_files"],
        "subagent_files": stats["subagent_files"],
        "top_level_records": stats["top_level_records"],
        "subagent_records": stats["subagent_records"],
        "duplicate_records_skipped": stats["duplicate_records"],
        "malformed_lines_skipped": stats["malformed_lines"],
        # The property that matters for a fleet: did we actually look where workers write?
        "includes_subagents": stats["subagent_files"] > 0,
    }
    return records, coverage


def rolling_window(records: list[dict], hours: int, now: datetime) -> float:
    """Weighted tokens in the trailing `hours`.

    Deliberately rolling. `ccusage weekly` buckets by *calendar* week, which reports a
    fraction of the true load every Monday morning; the limit it is meant to track is
    a rolling seven days.
    """
    cutoff = now - timedelta(hours=hours)
    return sum(r["weighted"] for r in records if cutoff <= r["ts"] <= now)


def rolling_window_blocks(blocks: list[dict], hours: int, now: datetime) -> float:
    """Same rolling window, over pre-aggregated blocks rather than raw records.

    ccusage hands back per-block totals with no per-message detail, so a block that
    straddles the window edge is prorated by the fraction of its span that falls
    inside. Coarser than the record-level path, and the reason the two sources can
    differ by a few percent at the boundary.
    """
    cutoff = now - timedelta(hours=hours)
    total = 0.0
    for b in blocks:
        start, end = b["start"], min(b["end"], now)
        if end <= cutoff or start >= now:
            continue
        span = (end - start).total_seconds()
        if span <= 0:
            continue
        overlap = (end - max(start, cutoff)).total_seconds()
        total += b["weighted"] * max(0.0, min(1.0, overlap / span))
    return total


def identify_blocks(records: list[dict], block_hours: int = BLOCK_HOURS) -> list[dict]:
    """Group records into usage blocks the way the CLI does.

    Blocks are anchored to **first activity**, not to an absolute clock grid: a block
    opens at the first record (floored to the hour) and closes after `block_hours`, or
    early if the gap to the next record exceeds `block_hours`. Flooring `now` to a
    fixed `hour % 5` grid instead would misplace the boundary whenever a session did
    not happen to start on that grid -- the live block observed while building this
    module opened at 23:00, which a mod-5 grid would call 00:00, silently dropping the
    first hour of load from the reading that gates the fleet.
    """
    if not records:
        return []
    span = timedelta(hours=block_hours)
    blocks: list[dict] = []
    start: datetime | None = None
    prev: datetime | None = None
    total = 0.0

    for r in sorted(records, key=lambda x: x["ts"]):
        ts = r["ts"]
        new_block = (start is None
                     or ts - start >= span
                     or (prev is not None and ts - prev >= span))
        if new_block:
            if start is not None:
                blocks.append({"start": start, "end": start + span, "weighted": total})
            start = ts.replace(minute=0, second=0, microsecond=0)
            total = 0.0
        total += r["weighted"]
        prev = ts
    if start is not None:
        blocks.append({"start": start, "end": start + span, "weighted": total})
    return blocks


def current_block(records: list[dict], now: datetime,
                  block_hours: int = BLOCK_HOURS) -> dict:
    """The block `now` falls in, derived from raw records."""
    return current_block_from(identify_blocks(records, block_hours), now, block_hours)


def current_block_from(blocks: list[dict], now: datetime,
                       block_hours: int = BLOCK_HOURS) -> dict:
    """The block `now` falls in, or an empty block anchored at `now` if none does."""
    live = next((b for b in reversed(blocks) if b["start"] <= now < b["end"]), None)
    if live is None:
        anchor = now.replace(minute=0, second=0, microsecond=0)
        live = {"start": anchor, "end": anchor + timedelta(hours=block_hours), "weighted": 0.0}
    return {
        "weighted_tokens": round(live["weighted"], 1),
        "block_start": live["start"].isoformat(),
        "resets_at": live["end"].isoformat(),
        "minutes_remaining": max(0, int((live["end"] - now).total_seconds() // 60)),
    }


def historical_peak_block(blocks: list[dict], now: datetime) -> float:
    """Heaviest *completed* block this account has ever produced.

    The self-calibrating denominator: absent a published limit, the largest block the
    account has actually sustained is the most defensible stand-in, and it is derived
    from the same weighting as the numerator so the ratio is internally consistent.
    The in-flight block is excluded -- comparing a partial block against itself would
    report 100% the moment it became the maximum.
    """
    if not blocks:
        return 0.0
    completed = [b["weighted"] for b in blocks if not (b["start"] <= now < b["end"])]
    return round(max(completed or [b["weighted"] for b in blocks]), 1)


def forecast(current: float, *, agents: int, minutes: int,
             burn_per_minute: float) -> float:
    """Projected weighted tokens once `agents` run for `minutes` at the recent rate.

    The reason the go/no-go compares this rather than the current reading: parallel
    workers are invisible until they return, so a between-waves check on current usage
    can wave through a batch that lands at 130% of the ceiling.
    """
    return round(current + max(0, agents) * max(0, minutes) * max(0.0, burn_per_minute), 1)


def recent_burn_rate(records: list[dict], now: datetime, minutes: int = 60) -> float:
    """Weighted tokens per minute over the trailing `minutes`, from raw records."""
    if minutes <= 0:
        return 0.0
    cutoff = now - timedelta(minutes=minutes)
    total = sum(r["weighted"] for r in records if cutoff <= r["ts"] <= now)
    return total / minutes


def burn_rate_from_block(block: dict, now: datetime) -> float:
    """Burn rate inferred from the in-flight block when per-message detail is absent.

    ccusage reports only a block total, so the trailing-60-minute rate cannot be
    recovered; the block's average over its elapsed span is the honest substitute.
    Dividing a whole block's total by 60 instead -- which is what treating the block
    as one timestamped record does -- overstates the rate by up to 5x.
    """
    start = _parse_ts(block["block_start"]) if isinstance(block["block_start"], str) \
        else block["block_start"]
    elapsed = max(1.0, (now - start).total_seconds() / 60.0)
    return block["weighted_tokens"] / elapsed


def decide(pct: float, projected_pct: float, ceiling_pct: float,
           week_pct: float | None = None,
           projected_week_pct: float | None = None) -> str:
    """`go` / `reduce` / `stop` from the projected load of the WORST window.

    The weekly arguments are not optional decoration. Until 2026-08-12 this function
    took only the 5h numbers, so an entire multi-wave fleet run returned `go` at every
    check while the owner sat at ~70-72% of a **weekly** ceiling declared as 80% -- the
    binding constraint was computed, returned in the payload, and then never consulted.
    The 5h window resets every five hours and is nearly always cheap just after a reset;
    the weekly one is the one that actually runs out. They are `None` only when no
    weekly anchor was supplied, in which case there is genuinely nothing to compare.
    """
    worst = max([v for v in (pct, projected_pct, week_pct, projected_week_pct)
                 if v is not None])
    if worst >= ceiling_pct:
        return "stop"
    if worst >= ceiling_pct * 0.75:
        return "reduce"
    return "go"


def binding_window(pct: float, projected_pct: float,
                   week_pct: float | None, projected_week_pct: float | None) -> str:
    """Which window drives the verdict -- so a `stop` says what to wait for."""
    five = max(pct, projected_pct)
    week = max([v for v in (week_pct, projected_week_pct) if v is not None] or [-1.0])
    return "seven_day" if week > five else "five_hour"


# --------------------------------------------------------------------------------
# Seat and model composition
# --------------------------------------------------------------------------------

def breakdown(records: list[dict], hours: int, now: datetime) -> dict:
    """Weighted tokens in the trailing `hours`, split by seat and by model.

    `by_seat` is the number this module exists to expose for a fleet: `top_level` is the
    orchestrator's own turns, `subagent` is every worker and Explore agent it spawned.
    Delegating work moves spend from the first bucket to the second, and rotating a
    session shrinks the first -- so a change that does either reads as "no improvement"
    against a total, and only shows up here.

    Deliberately NOT a second opinion on the total. On the ccusage path the block
    aggregates are authoritative and these record-level sums will not match them; the
    returned `basis` says so, because the first reader who notices the mismatch will
    otherwise "fix" one of the two.
    """
    cutoff = now - timedelta(hours=hours)
    window = [r for r in records if cutoff <= r["ts"] <= now]
    by_seat: dict[str, float] = {}
    by_model: dict[str, float] = {}
    for r in window:
        by_seat[r["source"]] = by_seat.get(r["source"], 0.0) + r["weighted"]
        by_model[r["model"] or "unknown"] = by_model.get(r["model"] or "unknown", 0.0) + r["weighted"]
    return {
        "by_seat": {k: round(v, 1) for k, v in sorted(by_seat.items())},
        "by_model": {k: round(v, 1) for k, v in sorted(by_model.items())},
        "records_in_window": len(window),
        "basis": ("record-level sums over the raw transcript store; these are a COMPOSITION "
                  "report and will not add up to the reported total when source=='ccusage', "
                  "where block aggregates are authoritative"),
    }


# --------------------------------------------------------------------------------
# Orchestrator context growth
# --------------------------------------------------------------------------------

#: Measured floor of a fresh session's context across six real sessions in this project
#: (42.3K-46.7K): the system prompt, AGENTS.md, the memory index and the tool schemas.
#: Note this is `input + cache_creation + cache_read`, not cache_read alone -- at turn 1
#: the system prompt is being cache-*created*, so reading cache_read alone reports ~17K
#: and understates the floor by 28K.
CONTEXT_BASE_TOKENS = 45_000
#: Rotate at the point where continuing costs at least one weekly point more than
#: restarting. At WAVE_TURNS=200 and an Opus effective cache-read weight of
#: CACHE_READ_WEIGHT * MODEL_WEIGHT = 0.1 * 5.0 = 0.5:
#:     C = CONTEXT_BASE_TOKENS + TOKENS_PER_WEEK_POINT / (200 * 0.5) = 45K + 250K = 295K
#: Below that the claimed saving sits inside this module's own admitted 1.6x calibration
#: uncertainty, and claiming it would be exactly the over-precision the header forbids.
ROTATE_AT_CONTEXT = 300_000
#: At 450K one orchestrator turn bills ~225K weighted; 200 more turns is ~45M, which
#: exceeds the ~40M a whole 3-worker wave with review, suite and browser checks costs.
#: The orchestrator's own memory is then more expensive than the work it coordinates.
HARD_ROTATE_AT_CONTEXT = 450_000
#: The horizon the saving is quoted over -- roughly one wave of orchestrator turns.
WAVE_TURNS = 200
#: Beyond this, the newest top-level record is probably not the calling session.
CONTEXT_STALENESS_S = 180

ROTATIONS = ("hold", "rotate", "rotate_hard", "unknown")


def identify_orchestrator(records: list[dict], now: datetime,
                          session_id: str | None = None,
                          project_dir: str | None = None) -> dict:
    """Which top-level session is asking.

    With no `session_id`, take the newest `top_level` record. That is self-verifying in
    the normal case: a tool call is written to the transcript before its result comes
    back, so at the moment this runs the caller *is* the newest top-level record. It is
    a heuristic all the same, so it is fenced by `CONTEXT_STALENESS_S` and reports its
    own `confidence` rather than presenting a guess as a measurement.
    """
    top = [r for r in records if r["source"] == "top_level"]
    if project_dir:
        top = [r for r in top if r["project_dir"] == project_dir] or top
    if not top:
        return {"ok": False, "reason": "no top-level records found", "confidence": "none"}

    if session_id:
        mine = [r for r in top if r["session_id"] == session_id]
        if not mine:
            return {"ok": False, "confidence": "none",
                    "reason": f"session_id {session_id!r} matched no top-level record"}
        return {"ok": True, "session_id": session_id, "confidence": "pinned",
                "records": mine}

    newest = max(top, key=lambda r: r["ts"])
    age = (now - newest["ts"]).total_seconds()
    if age > CONTEXT_STALENESS_S:
        return {"ok": False, "confidence": "none",
                "reason": (f"newest top-level record is {age:,.0f}s old (> "
                           f"{CONTEXT_STALENESS_S}s); this is probably not the calling "
                           f"session, so its context is not reported")}
    return {"ok": True, "session_id": newest["session_id"], "confidence": "inferred",
            "records": [r for r in top if r["session_id"] == newest["session_id"]]}


def session_context(records: list[dict], now: datetime,
                    session_id: str | None = None,
                    project_dir: str | None = None,
                    wave_turns: int = WAVE_TURNS) -> dict:
    """Current context size of the calling orchestrator, and what rotating would save.

    The saving is `turns * (C - B)` cache-read tokens: the growth term `s*n^2/2` is
    identical whether or not you rotate -- it is the cost of doing the work -- so only
    the flat carry of an already-large context is recoverable. Reported as a weighted
    token DELTA, never a percentage: this module has no trustworthy denominator.
    """
    who = identify_orchestrator(records, now, session_id, project_dir)
    if not who["ok"]:
        return {"ok": False, "reason": who["reason"], "confidence": who["confidence"]}

    mine = sorted(who["records"], key=lambda r: r["ts"])
    ctx = mine[-1]["context"]
    base = min((r["context"] for r in mine), default=CONTEXT_BASE_TOKENS) or CONTEXT_BASE_TOKENS
    base = min(base, CONTEXT_BASE_TOKENS)
    turns = len(mine)
    model = mine[-1]["model"]
    weight = CACHE_READ_WEIGHT * MODEL_WEIGHT.get(model or "", DEFAULT_MODEL_WEIGHT)
    saving = max(0.0, (ctx - base)) * wave_turns * weight
    # One fresh system-prompt cache_creation plus reading the handoff file back.
    cost = (CONTEXT_BASE_TOKENS + 5_000) * MODEL_WEIGHT.get(model or "", DEFAULT_MODEL_WEIGHT)
    return {
        "ok": True,
        "session_id": who["session_id"],
        "confidence": who["confidence"],
        "model": model,
        "turns": turns,
        "context_tokens": ctx,
        "base_tokens": base,
        "growth_per_turn": round((ctx - base) / turns, 1) if turns else 0.0,
        "last_record_age_s": round((now - mine[-1]["ts"]).total_seconds(), 1),
        "rotation_saving_weighted": round(saving, 1),
        "saving_basis": (f"turns*(C-B) cache-read carried over the next ~{wave_turns} turns, at "
                         f"CACHE_READ_WEIGHT={CACHE_READ_WEIGHT} and "
                         f"MODEL_WEIGHT[{model}]={MODEL_WEIGHT.get(model or '', DEFAULT_MODEL_WEIGHT)}; "
                         f"the s*n^2/2 growth term is identical either way and is excluded"),
        "rotation_cost_weighted": round(cost, 1),
        "cost_basis": ("one fresh ~45K system-prompt cache_creation plus a ~5K handoff read, "
                       "at the same model weight"),
    }


def decide_rotation(ctx: dict,
                    at: int = ROTATE_AT_CONTEXT,
                    hard: int = HARD_ROTATE_AT_CONTEXT) -> str:
    """`hold` / `rotate` / `rotate_hard` / `unknown` from raw context size.

    The trigger is RAW context tokens, deliberately not weighted. A weighted threshold
    would scale with `MODEL_WEIGHT`, so the same one-weekly-point rule would sit at
    ~1.3M tokens for a Sonnet- or Fable-class orchestrator, i.e. never fire -- one model
    would run 600-turn sessions while another rotated at 200, and any subsequent
    model-cost comparison would be measuring the rotation policy rather than the model.
    The weighted saving is *reported* as justification; the trigger stays raw.
    """
    if not ctx.get("ok"):
        return "unknown"
    if ctx["context_tokens"] >= hard:
        return "rotate_hard"
    if ctx["context_tokens"] >= at:
        return "rotate"
    return "hold"


#: Worst-first. `rotate` outranks `reduce` because rotating IS how you reduce: a smaller
#: wave inside a 400K context still pays 400K on every turn -- shrinking the wave attacks
#: the growth term, which is the one rotation cannot touch. `stop` outranks `rotate`
#: because a rotated session with no budget has burned a fresh 45K only to be told to stop.
_VERDICT_RANK = {"go": 0, "reduce": 1, "rotate": 2, "rotate_hard": 3, "stop": 4, "unknown": 5}


def combine_verdicts(budget_verdict: str, rotation: str) -> str:
    """Fold the rotation call into the budget call, worst-first."""
    if rotation in ("unknown", "hold"):
        return budget_verdict
    return max((budget_verdict, rotation), key=lambda v: _VERDICT_RANK.get(v, 0))


# --------------------------------------------------------------------------------
# ccusage bridge
# --------------------------------------------------------------------------------

def parse_ccusage_blocks(payload: dict) -> list[dict]:
    """Normalise `ccusage blocks --json` into comparable records.

    Raises `ValueError` on any shape it does not recognise, which is what turns a
    renamed key into `verdict: "unknown"` upstream instead of a silent zero.
    """
    if not isinstance(payload, dict) or "blocks" not in payload:
        raise ValueError("ccusage payload has no 'blocks' key")
    blocks = payload["blocks"]
    if not isinstance(blocks, list) or not blocks:
        raise ValueError("ccusage 'blocks' is empty or not a list")

    out = []
    for b in blocks:
        if not isinstance(b, dict):
            raise ValueError("ccusage block is not an object")
        counts = b.get("tokenCounts")
        start = _parse_ts(b.get("startTime"))
        if not isinstance(counts, dict) or start is None:
            raise ValueError("ccusage block is missing tokenCounts or a parseable startTime")
        models = b.get("models") or []
        # ccusage aggregates a block across models, so per-model weighting is not
        # recoverable here; use the heaviest model present, which is the conservative read.
        weight = max((MODEL_WEIGHT.get(m, DEFAULT_MODEL_WEIGHT) for m in models),
                     default=DEFAULT_MODEL_WEIGHT)
        usage = {
            "input_tokens": counts.get("inputTokens", 0),
            "output_tokens": counts.get("outputTokens", 0),
            "cache_creation_input_tokens": counts.get("cacheCreationInputTokens", 0),
            "cache_read_input_tokens": counts.get("cacheReadInputTokens", 0),
        }
        if b.get("isGap"):
            continue
        end = _parse_ts(b.get("endTime")) or (start + timedelta(hours=BLOCK_HOURS))
        out.append({
            "start": start,
            "end": end,
            "last_activity": _parse_ts(b.get("actualEndTime")) or start,
            "is_active": bool(b.get("isActive")),
            "weighted": weighted_tokens(usage, None) * weight,
            "cost_usd": b.get("costUSD"),
        })
    if not out:
        raise ValueError("ccusage returned only gap blocks")
    return out


def run_ccusage(timeout: int = 180, pkg: str = CCUSAGE_PKG) -> dict:
    """`npx -y <pinned ccusage> blocks --json`. Raises on any failure."""
    proc = subprocess.run(["npx", "-y", pkg, "blocks", "--json"],
                          capture_output=True, text=True, timeout=timeout)
    if proc.returncode != 0:
        raise RuntimeError(f"ccusage exited {proc.returncode}: {proc.stderr.strip()[:400]}")
    return json.loads(proc.stdout)


# --------------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------------

def fleet_budget(ceiling_pct: float = 80.0,
                 limit_basis: str = "peak",
                 limit_tokens: float | None = None,
                 forecast_agents: int = 0,
                 forecast_minutes: int = 0,
                 observed_week_pct: float | None = None,
                 observed_block_pct: float | None = None,
                 source: str = "auto",
                 now: datetime | None = None,
                 root: Path = PROJECTS_ROOT,
                 ccusage_payload: dict | None = None,
                 session_id: str | None = None,
                 project_dir: str | None = None,
                 rotate_at_context: int = ROTATE_AT_CONTEXT,
                 wave_turns: int = WAVE_TURNS) -> dict:
    """Current 5h and rolling-7d load against `ceiling_pct`, plus an orchestrator-context
    check, folded into one verdict.

    `limit_basis` is `"peak"` (this account's heaviest completed 5h block) or
    `"owner"` (requires `limit_tokens`). `source` is `"auto"`, `"ccusage"`, or
    `"transcripts"`.

    `verdict` may now be `"rotate"`/`"rotate_hard"` as well as `go`/`reduce`/`stop`/
    `unknown`; `budget_verdict` always carries the original four so older callers keep
    working. `session_id` pins which top-level session is measured (take it from the
    first call's `rotation.session_id`); without it the newest top-level record is used
    and `rotation.confidence` reports `"inferred"`.
    """
    now = now or datetime.now(UTC)
    unknown = {
        "ok": False, "verdict": "unknown", "source": "none",
        "coverage": {"includes_subagents": False},
        "limit_basis": "none — no usage data could be read",
        "ceiling_pct": ceiling_pct,
        "five_hour": None, "seven_day": None,
        "projected_pct": None, "headroom_pct": None,
    }

    # ccusage hands back per-block aggregates; the transcript reader hands back per-message
    # records. Keeping the two representations distinct is load-bearing: treating a block
    # total as if it were a single timestamped message made the trailing-60-minute burn rate
    # read 1.28M tokens/min and flipped a healthy run to STOP.
    blocks: list[dict] = []
    records: list[dict] = []
    coverage = {"includes_subagents": False}
    used_source, notes = "none", []

    if source in ("auto", "ccusage"):
        try:
            payload = ccusage_payload if ccusage_payload is not None else run_ccusage()
            blocks = parse_ccusage_blocks(payload)
            used_source = "ccusage"
            # ccusage's own file coverage is not observable from its payload, and whether
            # subagent transcripts were counted is the one fact a fleet must not guess at.
            # Read the store directly for that, cheaply, and report it either way.
            #
            # The records are KEPT (they used to be dropped on the floor here) because the
            # per-seat, per-model and context breakdowns below are record-level facts that
            # block aggregates cannot express. They are deliberately NOT used for any
            # headline total on this path -- see the gating note at `week_tokens`.
            records, coverage = read_transcripts(root)
        except Exception as exc:  # noqa: BLE001 -- any failure must degrade, not raise
            notes.append(f"ccusage unavailable: {type(exc).__name__}: {exc}")
            if source == "ccusage":
                return {**unknown, "reason": notes[0], "notes": notes}

    if used_source == "none" and source in ("auto", "transcripts"):
        try:
            records, coverage = read_transcripts(root)
            blocks = identify_blocks(records)
            used_source = "transcripts"
        except Exception as exc:  # noqa: BLE001
            notes.append(f"transcript read failed: {type(exc).__name__}: {exc}")
            return {**unknown, "reason": notes[-1], "notes": notes}

    if not blocks:
        return {**unknown, "reason": "no usage records found", "notes": notes}

    block = current_block_from(blocks, now)
    # Records are for DETAIL (seat, model, context); blocks are for TOTALS. Never mix.
    #
    # This used to read `if records`, which was equivalent only because the ccusage path
    # discarded its records. Now that it keeps them -- the seat and context breakdowns are
    # record-level facts -- truthiness would silently switch the weekly total from the
    # block-prorated derivation to the record derivation, changing the number the owner
    # has been anchoring `observed_week_pct` against. Gate on the source, not on whether
    # the list happens to be populated.
    week_tokens = round(
        rolling_window_blocks(blocks, WEEK_HOURS, now) if used_source == "ccusage"
        else rolling_window(records, WEEK_HOURS, now), 1)

    if limit_basis == "owner":
        if not limit_tokens or limit_tokens <= 0:
            return {**unknown,
                    "reason": "limit_basis='owner' requires a positive limit_tokens",
                    "notes": notes}
        denominator = float(limit_tokens)
        basis_text = (f"owner-supplied: {denominator:,.0f} weighted tokens per {BLOCK_HOURS}h block")
    else:
        denominator = historical_peak_block(blocks, now)
        if denominator <= 0:
            return {**unknown,
                    "reason": "no completed block to calibrate against; pass limit_basis='owner'",
                    "notes": notes}
        basis_text = (f"this account's heaviest completed {BLOCK_HOURS}h block "
                      f"({denominator:,.0f} weighted tokens) — an observed high-water mark, "
                      f"not a published limit")

    if observed_block_pct and observed_block_pct > 0:
        # Anchor on what the owner can actually see. Locally valid: it converts THIS
        # reading's tokens at THIS work mix, which is the only regime the weighting
        # heuristic is trustworthy over.
        denominator = block["weighted_tokens"] / (observed_block_pct / 100.0)
        basis_text = (f"anchored on owner-reported {observed_block_pct:g}% of the {BLOCK_HOURS}h "
                      f"block at this reading ({denominator:,.0f} weighted tokens implied)")
    pct = round(block["weighted_tokens"] / denominator * 100, 1)

    # Same gating rule as `week_tokens`: on the ccusage path the block is authoritative.
    burn = (burn_rate_from_block(block, now) if used_source == "ccusage"
            else recent_burn_rate(records, now))
    projected = forecast(block["weighted_tokens"], agents=forecast_agents,
                         minutes=forecast_minutes, burn_per_minute=burn)
    projected_pct = round(projected / denominator * 100, 1)
    added = max(0.0, projected - block["weighted_tokens"])

    # The weekly percentage is ANCHORED, never derived. The old code computed it as
    # `week_tokens / (peak_block * 168/5)` -- 33.6 back-to-back peak blocks, i.e. the
    # assumption that no weekly cap exists -- and reported 12.4% against an owner-read
    # 72%. Rather than swap one invented denominator for another (see the calibration
    # note at the top of this file: two readings of the same window implied allowances
    # 1.6x apart), report `None` when there is no anchor and say so.
    if observed_week_pct and observed_week_pct > 0:
        week_pct = round(float(observed_week_pct), 1)
        projected_week_pct = round(week_pct + added / TOKENS_PER_WEEK_POINT, 1)
        week_basis = (f"anchored on owner-reported {observed_week_pct:g}%; the forecast adds "
                      f"{added:,.0f} weighted tokens at {TOKENS_PER_WEEK_POINT:,.0f}/point "
                      f"(measured {CALIBRATED_ON}, deliberately the conservative bound)")
    else:
        week_pct = None
        projected_week_pct = None
        week_basis = ("no anchor — pass `observed_week_pct` with the figure the owner reads "
                      "off their client. This module's weighted token does not convert to a "
                      "real weekly percentage (see the calibration note in fleet_budget.py); "
                      "the token count below is a real measurement, the percentage is not "
                      "derivable from it")
        notes.append("WEEKLY LOAD IS UNKNOWN: the verdict reflects the 5h block only. The "
                     "weekly window is usually the binding one — ask the owner for their "
                     "current weekly percentage and pass it as `observed_week_pct`.")

    budget_verdict = decide(pct, projected_pct, ceiling_pct, week_pct, projected_week_pct)
    binding = binding_window(pct, projected_pct, week_pct, projected_week_pct)
    if not coverage.get("includes_subagents"):
        notes.append("no subagent transcripts were found — if a fleet is running, this "
                     "number is orchestrator-only and understates the real load")

    # Context growth is measured from records, which exist on every path now. A context
    # that cannot be measured must NOT become `verdict: "unknown"` -- that keyword means
    # "stop and ask" to every caller, and firing it whenever the newest-record heuristic
    # misfires is how a useful guard gets deleted. Degrade to a note instead.
    ctx = session_context(records, now, session_id=session_id,
                          project_dir=project_dir, wave_turns=wave_turns)
    rotation_call = decide_rotation(ctx, at=rotate_at_context)
    verdict = combine_verdicts(budget_verdict, rotation_call)
    if not ctx.get("ok"):
        notes.append(f"context growth not measured: {ctx.get('reason')}; the verdict "
                     f"covers the budget windows only")
    elif rotation_call.startswith("rotate"):
        notes.append(
            f"ROTATE: this session is carrying {ctx['context_tokens']:,} tokens of context "
            f"over {ctx['turns']} turns (fresh is ~{ctx['base_tokens']:,}). Continuing costs "
            f"~{ctx['rotation_saving_weighted']:,.0f} weighted tokens more than handing off "
            f"and starting fresh, over the next ~{wave_turns} turns; the handoff itself costs "
            f"~{ctx['rotation_cost_weighted']:,.0f}. Finish the wave in flight, write the "
            f"handoff, then stop — do not start another wave.")

    binding_constraint = ("context" if rotation_call.startswith("rotate")
                          and _VERDICT_RANK.get(rotation_call, 0) >= _VERDICT_RANK.get(budget_verdict, 0)
                          else binding)

    return {
        "ok": True,
        "source": used_source,
        "coverage": coverage,
        "limit_basis": basis_text,
        "ceiling_pct": ceiling_pct,
        "five_hour": {**block, "pct": pct,
                      **breakdown(records, BLOCK_HOURS, now)},
        "seven_day": {"weighted_tokens": week_tokens, "pct": week_pct,
                      "projected_pct": projected_week_pct,
                      "pct_basis": week_basis,
                      "window_start": (now - timedelta(hours=WEEK_HOURS)).isoformat(),
                      "rolling": True,
                      **breakdown(records, WEEK_HOURS, now)},
        "burn_per_minute": round(burn, 1),
        "projected_pct": projected_pct,
        "headroom_pct": round(ceiling_pct - max(
            [v for v in (pct, projected_pct, week_pct, projected_week_pct)
             if v is not None]), 1),
        "binding_window": binding,
        # `binding_window` keeps its exact meaning -- WHICH WINDOW drives the verdict.
        # Context is not a window, so it gets its own field rather than overloading that one.
        "binding_constraint": binding_constraint,
        "rotation": {**ctx, "call": rotation_call},
        # The unmodified go/reduce/stop/unknown, so a caller that only knows those four
        # values keeps working when `verdict` is `rotate`.
        "budget_verdict": budget_verdict,
        "verdict": verdict,
        "reason": _verdict_reason(budget_verdict, pct, projected_pct, ceiling_pct,
                                  week_pct, projected_week_pct, binding),
        "notes": notes,
    }


def _verdict_reason(verdict: str, pct: float, projected: float, ceiling: float,
                    week_pct: float | None = None,
                    projected_week_pct: float | None = None,
                    binding: str = "five_hour") -> str:
    """Name the WINDOW the number came from. A bare "projected 81%" gave no clue
    whether to wait five hours or five days, which is the only actionable part."""
    worst = max([v for v in (pct, projected, week_pct, projected_week_pct)
                 if v is not None])
    where = "weekly" if binding == "seven_day" else "5h block"
    if week_pct is None:
        where += ", weekly UNKNOWN"
    if verdict == "stop":
        return (f"projected {worst:.1f}% of the declared ceiling ({ceiling:.0f}%) on the "
                f"{where} — do not start another wave")
    if verdict == "reduce":
        return (f"projected {worst:.1f}% against a {ceiling:.0f}% ceiling on the {where} — "
                f"shrink the wave or drop to cheaper tiers")
    return f"projected {worst:.1f}% against a {ceiling:.0f}% ceiling on the {where}"


def _render(r: dict) -> str:
    if not r.get("ok"):
        return (f"verdict : {r['verdict'].upper()}\n"
                f"reason  : {r.get('reason', 'unknown')}\n"
                f"basis   : {r['limit_basis']}")
    fh, sd, cov = r["five_hour"], r["seven_day"], r["coverage"]
    return "\n".join([
        f"verdict : {r['verdict'].upper()} — {r['reason']}",
        f"basis   : {r['limit_basis']}",
        f"source  : {r['source']}  (subagent transcripts read: "
        f"{cov.get('subagent_files', 0)} files, included={cov.get('includes_subagents')})",
        f"5-hour  : {fh['pct']:.1f}% of ceiling basis · {fh['weighted_tokens']:,.0f} weighted "
        f"· resets {fh['resets_at']} ({fh['minutes_remaining']} min)",
        f"7-day   : {sd['pct']:.1f}% · {sd['weighted_tokens']:,.0f} weighted (rolling 168h)",
        f"burn    : {r['burn_per_minute']:,.0f} weighted tokens/min (trailing 60 min)",
        f"headroom: {r['headroom_pct']:.1f} points",
        f"seats   : {_render_seats(sd.get('by_seat', {}))}",
        f"context : {_render_context(r.get('rotation', {}))}",
        *([f"note    : {n}" for n in r.get("notes", [])]),
    ])


def _render_seats(by_seat: dict) -> str:
    """Orchestrator share is the actionable half: delegating and rotating both move
    spend out of `top_level`, and neither shows up in a total."""
    if not by_seat:
        return "not measured"
    top, sub = by_seat.get("top_level", 0.0), by_seat.get("subagent", 0.0)
    total = top + sub
    share = f" ({100 * top / total:.0f}% orchestrator)" if total else ""
    return f"orchestrator {top:,.0f} · workers {sub:,.0f} weighted{share}"


def _render_context(rot: dict) -> str:
    if not rot.get("ok"):
        return f"not measured — {rot.get('reason', 'unknown')}"
    return (f"{rot['context_tokens']:,} tokens over {rot['turns']} turns "
            f"[{rot['call']}] · rotating would save ~{rot['rotation_saving_weighted']:,.0f} "
            f"weighted vs ~{rot['rotation_cost_weighted']:,.0f} to hand off "
            f"({rot['confidence']} session id)")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ceiling", type=float, default=80.0,
                    help="not-to-exceed percentage of the limit basis (default: 80)")
    ap.add_argument("--basis", choices=("peak", "owner"), default="peak")
    ap.add_argument("--limit-tokens", type=float, default=None,
                    help="required with --basis owner: weighted tokens per 5h block")
    ap.add_argument("--agents", type=int, default=0, help="agents to forecast")
    ap.add_argument("--minutes", type=int, default=0, help="minutes to forecast them running")
    ap.add_argument("--observed-week-pct", type=float, default=None,
                    help="the weekly percentage the owner reads off their client — without "
                         "it the weekly number is None and the verdict covers the 5h block "
                         "only, which is usually NOT the binding window")
    ap.add_argument("--observed-block-pct", type=float, default=None,
                    help="the 5h-block percentage the owner reads off their client")
    ap.add_argument("--source", choices=("auto", "ccusage", "transcripts"), default="auto")
    ap.add_argument("--session-id", default=None,
                    help="pin which top-level session's context is measured; without it "
                         "the newest top-level record is used (reported as 'inferred')")
    ap.add_argument("--project-dir", default=None,
                    help="restrict session identification to one project directory")
    ap.add_argument("--rotate-at", type=int, default=ROTATE_AT_CONTEXT,
                    help=f"context tokens at which to advise rotation (default: "
                         f"{ROTATE_AT_CONTEXT:,})")
    ap.add_argument("--wave-turns", type=int, default=WAVE_TURNS,
                    help=f"horizon the rotation saving is quoted over (default: {WAVE_TURNS})")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    result = fleet_budget(ceiling_pct=args.ceiling, limit_basis=args.basis,
                          limit_tokens=args.limit_tokens, forecast_agents=args.agents,
                          forecast_minutes=args.minutes,
                          observed_week_pct=args.observed_week_pct,
                          observed_block_pct=args.observed_block_pct,
                          source=args.source,
                          session_id=args.session_id, project_dir=args.project_dir,
                          rotate_at_context=args.rotate_at, wave_turns=args.wave_turns)
    print(json.dumps(result, indent=2) if args.json else _render(result))
    return 0 if result.get("verdict") != "unknown" else 1


if __name__ == "__main__":
    sys.exit(main())
