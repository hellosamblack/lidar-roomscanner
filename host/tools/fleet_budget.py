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
                records.append({
                    "ts": ts,
                    "model": message.get("model"),
                    "weighted": weighted_tokens(usage, message.get("model")),
                    "source": label,
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


def decide(pct: float, projected_pct: float, ceiling_pct: float) -> str:
    """`go` / `reduce` / `stop` from the projected load, not the current load."""
    worst = max(pct, projected_pct)
    if worst >= ceiling_pct:
        return "stop"
    if worst >= ceiling_pct * 0.75:
        return "reduce"
    return "go"


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
                 source: str = "auto",
                 now: datetime | None = None,
                 root: Path = PROJECTS_ROOT,
                 ccusage_payload: dict | None = None) -> dict:
    """Current 5h and rolling-7d load against `ceiling_pct`, with a go/reduce/stop call.

    `limit_basis` is `"peak"` (this account's heaviest completed 5h block) or
    `"owner"` (requires `limit_tokens`). `source` is `"auto"`, `"ccusage"`, or
    `"transcripts"`.
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
            _, coverage = read_transcripts(root)
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
    week_tokens = round(
        rolling_window(records, WEEK_HOURS, now) if records
        else rolling_window_blocks(blocks, WEEK_HOURS, now), 1)

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

    pct = round(block["weighted_tokens"] / denominator * 100, 1)
    week_denominator = denominator * (WEEK_HOURS / BLOCK_HOURS)
    week_pct = round(week_tokens / week_denominator * 100, 1)

    burn = recent_burn_rate(records, now) if records else burn_rate_from_block(block, now)
    projected = forecast(block["weighted_tokens"], agents=forecast_agents,
                         minutes=forecast_minutes, burn_per_minute=burn)
    projected_pct = round(projected / denominator * 100, 1)

    verdict = decide(pct, projected_pct, ceiling_pct)
    if not coverage.get("includes_subagents"):
        notes.append("no subagent transcripts were found — if a fleet is running, this "
                     "number is orchestrator-only and understates the real load")

    return {
        "ok": True,
        "source": used_source,
        "coverage": coverage,
        "limit_basis": basis_text,
        "ceiling_pct": ceiling_pct,
        "five_hour": {**block, "pct": pct},
        "seven_day": {"weighted_tokens": week_tokens, "pct": week_pct,
                      "window_start": (now - timedelta(hours=WEEK_HOURS)).isoformat(),
                      "rolling": True},
        "burn_per_minute": round(burn, 1),
        "projected_pct": projected_pct,
        "headroom_pct": round(ceiling_pct - max(pct, projected_pct), 1),
        "verdict": verdict,
        "reason": _verdict_reason(verdict, pct, projected_pct, ceiling_pct),
        "notes": notes,
    }


def _verdict_reason(verdict: str, pct: float, projected: float, ceiling: float) -> str:
    if verdict == "stop":
        return (f"projected {max(pct, projected):.1f}% of the declared ceiling "
                f"({ceiling:.0f}%) — do not start another wave")
    if verdict == "reduce":
        return (f"projected {max(pct, projected):.1f}% against a {ceiling:.0f}% ceiling — "
                f"shrink the wave or drop to cheaper tiers")
    return f"projected {max(pct, projected):.1f}% against a {ceiling:.0f}% ceiling"


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
        *([f"note    : {n}" for n in r.get("notes", [])]),
    ])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ceiling", type=float, default=80.0,
                    help="not-to-exceed percentage of the limit basis (default: 80)")
    ap.add_argument("--basis", choices=("peak", "owner"), default="peak")
    ap.add_argument("--limit-tokens", type=float, default=None,
                    help="required with --basis owner: weighted tokens per 5h block")
    ap.add_argument("--agents", type=int, default=0, help="agents to forecast")
    ap.add_argument("--minutes", type=int, default=0, help="minutes to forecast them running")
    ap.add_argument("--source", choices=("auto", "ccusage", "transcripts"), default="auto")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    result = fleet_budget(ceiling_pct=args.ceiling, limit_basis=args.basis,
                          limit_tokens=args.limit_tokens, forecast_agents=args.agents,
                          forecast_minutes=args.minutes, source=args.source)
    print(json.dumps(result, indent=2) if args.json else _render(result))
    return 0 if result.get("verdict") != "unknown" else 1


if __name__ == "__main__":
    sys.exit(main())
