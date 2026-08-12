"""Guards on `fleet_budget`'s windowing, deduplication, and refusal to guess.

Offline: reads the synthetic transcript tree and captured ccusage payloads under
`fixtures/fleet/`. Reference `now` for the transcript tree is 2026-08-12T03:00:00Z,
and the fixture README records what each record encodes.

The theme of most of these is that a usage estimator's failure mode is *confidence*.
A renamed key that reads as zero, a flat glob that misses the workers, or a
percentage of an invented denominator all produce a healthy-looking number that is
wrong, which is worse than no number at all.
"""
import importlib.util
import json
import sys
from datetime import UTC, datetime, timedelta
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location("fleet_budget", _ROOT / "tools" / "fleet_budget.py")
fb = importlib.util.module_from_spec(_SPEC)
sys.modules["fleet_budget"] = fb
_SPEC.loader.exec_module(fb)

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "fleet"
TRANSCRIPTS = FIXTURES / "transcripts"
NOW = datetime(2026, 8, 12, 3, 0, tzinfo=UTC)


@pytest.fixture(scope="module")
def read():
    return fb.read_transcripts(TRANSCRIPTS)


@pytest.fixture(scope="module")
def blocks_payload():
    return json.loads((FIXTURES / "ccusage_blocks.json").read_text())


# --------------------------------------------------------------------------------
# Coverage: the nested subagent path
# --------------------------------------------------------------------------------

def test_reader_finds_the_nested_subagent_transcripts(read):
    """Worker spend lives at `<session>/subagents/agent-*.jsonl`, one level below the
    main transcripts. A flat `projects/*/*.jsonl` glob misses all of it -- and in a
    fleet the workers are most of the spend, so this is the difference between a
    budget and a decoration."""
    _, coverage = read
    assert coverage["subagent_files"] == 1
    assert coverage["subagent_records"] > 0
    assert coverage["includes_subagents"] is True


def test_subagent_tokens_actually_enter_the_total(read):
    records, _ = read
    assert any(r["source"] == "subagent" for r in records)
    subagent_weight = sum(r["weighted"] for r in records if r["source"] == "subagent")
    assert subagent_weight > 0


def test_flat_glob_would_have_missed_them():
    """Pins the reason the reader uses two globs, so a future simplification fails here."""
    flat = sorted(TRANSCRIPTS.glob(fb.TOP_LEVEL_GLOB))
    nested = sorted(TRANSCRIPTS.glob(fb.SUBAGENT_GLOB))
    assert nested, "fixture lost its nested subagent transcript"
    assert not set(flat) & set(nested)


def test_coverage_reports_absent_subagents_rather_than_assuming(tmp_path):
    proj = tmp_path / "-proj"
    proj.mkdir()
    (proj / "s.jsonl").write_text("")
    _, coverage = fb.read_transcripts(tmp_path)
    assert coverage["includes_subagents"] is False


# --------------------------------------------------------------------------------
# Deduplication
# --------------------------------------------------------------------------------

def test_duplicate_records_are_counted_once(read):
    """The store repeats `(message.id, requestId)` pairs. Summing naively overcounted a
    real 5h block by ~80% (185.8M vs 102.8M weighted tokens)."""
    _, coverage = read
    assert coverage["duplicate_records_skipped"] == 2


def test_dedup_changes_the_answer(read):
    records, _ = read
    deduped = sum(r["weighted"] for r in records)
    naive = 0.0
    for path in list(TRANSCRIPTS.glob(fb.TOP_LEVEL_GLOB)) + list(TRANSCRIPTS.glob(fb.SUBAGENT_GLOB)):
        for line in path.read_text().splitlines():
            if '"usage"' not in line:
                continue
            doc = json.loads(line)
            msg = doc["message"]
            naive += fb.weighted_tokens(msg["usage"], msg.get("model"))
    assert naive > deduped, "the fixture must contain duplicates for this to mean anything"


def test_malformed_and_usage_free_lines_are_skipped_not_raised(read):
    records, _ = read
    assert records, "a malformed line must not abort the whole read"


# --------------------------------------------------------------------------------
# Windowing: rolling vs calendar
# --------------------------------------------------------------------------------

def test_seven_day_window_is_rolling_not_calendar(read):
    """A record on 2026-08-06 is in ISO week 32 while `now` is in week 33, but it is
    inside the trailing 168 hours (which open at 2026-08-05T03:00Z). `ccusage weekly`
    buckets by calendar week and would drop it; the limit it tracks does not."""
    records, _ = read
    target = next(r for r in records if r["ts"] == datetime(2026, 8, 6, 9, tzinfo=UTC))
    assert target["ts"].isocalendar()[1] != NOW.isocalendar()[1], "fixture drifted"
    included = fb.rolling_window(records, fb.WEEK_HOURS, NOW)
    excluding = fb.rolling_window([r for r in records if r is not target], fb.WEEK_HOURS, NOW)
    assert included > excluding


def test_records_older_than_the_window_are_excluded(read):
    records, _ = read
    stale = datetime(2026, 8, 4, 12, tzinfo=UTC)
    assert stale < NOW - timedelta(hours=fb.WEEK_HOURS)
    assert any(r["ts"] == stale for r in records), "fixture lost its out-of-window record"
    total = fb.rolling_window(records, fb.WEEK_HOURS, NOW)
    assert total == pytest.approx(
        sum(r["weighted"] for r in records if r["ts"] >= NOW - timedelta(hours=fb.WEEK_HOURS)))


def test_blocks_are_anchored_to_activity_not_a_clock_grid(read):
    """The live block observed while building this module opened at 23:00. Flooring
    `now` onto an absolute `hour % 5` grid would call that 00:00 and silently drop the
    first hour of load from the reading that gates the fleet."""
    records, _ = read
    blocks = fb.identify_blocks(records)
    assert any(b["start"] == datetime(2026, 8, 4, 12, tzinfo=UTC) for b in blocks)
    assert all(b["start"].minute == 0 for b in blocks)
    assert any(b["start"].hour % fb.BLOCK_HOURS != 0 for b in blocks), (
        "fixture no longer distinguishes activity-anchored blocks from a mod-5 grid")


def test_a_gap_longer_than_a_block_starts_a_new_block(read):
    records, _ = read
    starts = [b["start"] for b in fb.identify_blocks(records)]
    assert len(starts) == len(set(starts)) > 1


def test_current_block_reports_when_it_resets(read):
    records, _ = read
    block = fb.current_block(records, NOW)
    assert block["weighted_tokens"] > 0
    assert fb._parse_ts(block["resets_at"]) > NOW
    assert block["minutes_remaining"] > 0


def test_historical_peak_excludes_the_in_flight_block(read):
    """Comparing a partial block against itself would report 100% the instant it became
    the maximum, turning a healthy run into a spurious STOP."""
    records, _ = read
    peak = fb.historical_peak_block(fb.identify_blocks(records), NOW)
    live = fb.current_block(records, NOW)["weighted_tokens"]
    assert peak > 0
    assert peak != live


# --------------------------------------------------------------------------------
# Weighting
# --------------------------------------------------------------------------------

def test_cache_reads_are_discounted():
    read_heavy = fb.weighted_tokens({"cache_read_input_tokens": 1000})
    create_heavy = fb.weighted_tokens({"cache_creation_input_tokens": 1000})
    assert read_heavy == pytest.approx(create_heavy * fb.CACHE_READ_WEIGHT)
    assert read_heavy < create_heavy


def test_output_costs_more_than_input():
    assert fb.weighted_tokens({"output_tokens": 100}) > fb.weighted_tokens({"input_tokens": 100})


def test_model_weight_separates_opus_from_haiku():
    usage = {"output_tokens": 1000}
    assert (fb.weighted_tokens(usage, "claude-opus-5")
            > fb.weighted_tokens(usage, "claude-sonnet-5")
            > fb.weighted_tokens(usage, "claude-haiku-4-5-20251001"))


def test_unknown_model_does_not_vanish():
    assert fb.weighted_tokens({"output_tokens": 100}, "claude-future-9") > 0


# --------------------------------------------------------------------------------
# Refusing to guess
# --------------------------------------------------------------------------------

def test_mangled_ccusage_payload_yields_unknown_not_zero():
    """A renamed key must not read as a healthy 0%."""
    payload = json.loads((FIXTURES / "ccusage_blocks_mangled.json").read_text())
    result = fb.fleet_budget(source="ccusage", ccusage_payload=payload, now=NOW,
                             root=TRANSCRIPTS)
    assert result["ok"] is False
    assert result["verdict"] == "unknown"
    assert result["five_hour"] is None
    assert result["projected_pct"] is None
    assert result["reason"]


@pytest.mark.parametrize("kwargs", [
    {"source": "transcripts", "limit_basis": "peak"},
    {"source": "transcripts", "limit_basis": "owner", "limit_tokens": 1_000_000},
    {"source": "transcripts", "limit_basis": "owner"},                      # invalid: no tokens
    {"source": "ccusage", "ccusage_payload": {"blocks": []}},               # invalid payload
])
def test_limit_basis_is_never_empty(kwargs):
    """A percentage whose denominator is invented is a fabricated number, so every
    return path -- including every failure path -- has to name its denominator."""
    result = fb.fleet_budget(now=NOW, root=TRANSCRIPTS, **kwargs)
    assert result["limit_basis"].strip()


def test_owner_basis_requires_a_positive_limit():
    result = fb.fleet_budget(source="transcripts", limit_basis="owner", limit_tokens=0,
                             now=NOW, root=TRANSCRIPTS)
    assert result["verdict"] == "unknown"
    assert "limit_tokens" in result["reason"]


def test_peak_basis_names_itself_as_an_observed_high_water_mark():
    result = fb.fleet_budget(source="transcripts", limit_basis="peak", now=NOW,
                             root=TRANSCRIPTS)
    assert "not a published limit" in result["limit_basis"]


def test_empty_store_is_unknown_rather_than_zero(tmp_path):
    result = fb.fleet_budget(source="transcripts", now=NOW, root=tmp_path)
    assert result["verdict"] == "unknown"
    assert result["five_hour"] is None


def test_parse_ccusage_rejects_a_renamed_field(blocks_payload):
    broken = json.loads(json.dumps(blocks_payload))
    broken["blocks"][0]["tokenTotals"] = broken["blocks"][0].pop("tokenCounts")
    with pytest.raises(ValueError):
        fb.parse_ccusage_blocks(broken)


def test_parse_ccusage_accepts_the_captured_shape(blocks_payload):
    parsed = fb.parse_ccusage_blocks(blocks_payload)
    assert parsed
    assert all(b["end"] > b["start"] for b in parsed)
    assert all(b["weighted"] >= 0 for b in parsed)


# --------------------------------------------------------------------------------
# Verdicts and forecasting
# --------------------------------------------------------------------------------

def test_verdict_flips_exactly_at_the_ceiling():
    assert fb.decide(79.9, 0, 80.0) == "reduce"
    assert fb.decide(80.0, 0, 80.0) == "stop"
    assert fb.decide(10.0, 0, 80.0) == "go"


def test_verdict_uses_the_projection_not_just_the_present():
    """Parallel workers are invisible until they return, so judging on current load
    alone waves through a batch that lands well past the ceiling."""
    assert fb.decide(10.0, 95.0, 80.0) == "stop"


def test_verdict_stops_on_the_weekly_window_even_when_the_5h_block_is_idle():
    """#172, the defect this whole family exists to prevent: `decide()` used to take
    only the 5h numbers, so a run just after a block reset looked completely free while
    the WEEKLY ceiling was nearly spent. Measured on 2026-08-12 -- the tool returned
    `go` at every check across a multi-wave run while the owner sat at 70-72% of a
    weekly ceiling declared as 80%. The 5h window resets every five hours; the weekly
    one is what actually runs out."""
    assert fb.decide(2.0, 3.0, 80.0, week_pct=85.0, projected_week_pct=85.0) == "stop"
    # ... and the projection over the weekly window counts too, not just its present.
    assert fb.decide(2.0, 3.0, 80.0, week_pct=70.0, projected_week_pct=95.0) == "stop"
    # With no weekly anchor there is nothing to compare, and 5h alone still decides.
    assert fb.decide(2.0, 3.0, 80.0, week_pct=None, projected_week_pct=None) == "go"


def test_binding_window_names_the_window_that_drove_the_verdict():
    """A bare "projected 81%" does not say whether to wait five hours or five days."""
    assert fb.binding_window(2.0, 3.0, 85.0, 85.0) == "seven_day"
    assert fb.binding_window(90.0, 91.0, 10.0, 10.0) == "five_hour"
    assert fb.binding_window(2.0, 3.0, None, None) == "five_hour"


def test_weekly_percentage_is_none_without_an_anchor_and_says_so():
    """Rather than swap one invented denominator for another. The old code derived it
    as `peak_block * 168/5` -- 33.6 back-to-back peak blocks, i.e. no weekly cap at all
    -- and reported 12.4% against an owner-read 72%."""
    r = fb.fleet_budget(source="transcripts", limit_basis="owner", limit_tokens=1_000_000,
                        now=NOW, root=TRANSCRIPTS)
    assert r["seven_day"]["pct"] is None
    assert r["seven_day"]["weighted_tokens"] > 0, "the token count is still a real measurement"
    assert "no anchor" in r["seven_day"]["pct_basis"]
    assert any("WEEKLY LOAD IS UNKNOWN" in n for n in r["notes"])


def test_weekly_anchor_is_reported_verbatim_and_the_forecast_adds_to_it():
    """The anchor is the owner's figure, not something we recompute; the forecast moves
    it by measured tokens-per-point rather than by a re-derived percentage."""
    r = fb.fleet_budget(source="transcripts", limit_basis="owner", limit_tokens=1_000_000,
                        observed_week_pct=72.0, forecast_agents=3, forecast_minutes=45,
                        now=NOW, root=TRANSCRIPTS)
    assert r["seven_day"]["pct"] == 72.0
    assert r["seven_day"]["projected_pct"] >= 72.0
    assert "anchored on owner-reported 72%" in r["seven_day"]["pct_basis"]


def test_observed_block_pct_overrides_the_peak_denominator():
    """Anchoring the 5h block on a reported percentage must actually move the number,
    and must name itself in `limit_basis` -- the standing rule that a percentage never
    appears without its denominator."""
    anchored = fb.fleet_budget(source="transcripts", observed_block_pct=22.0,
                               now=NOW, root=TRANSCRIPTS)
    assert anchored["five_hour"]["pct"] == pytest.approx(22.0, abs=0.1)
    assert "anchored on owner-reported 22%" in anchored["limit_basis"]


def test_forecast_grows_with_agents_and_minutes():
    base = fb.forecast(100, agents=0, minutes=30, burn_per_minute=10)
    more = fb.forecast(100, agents=3, minutes=30, burn_per_minute=10)
    assert more > base == 100


def test_forecast_ignores_negative_inputs():
    assert fb.forecast(100, agents=-5, minutes=-5, burn_per_minute=-5) == 100


def test_projection_raises_the_percentage():
    plain = fb.fleet_budget(source="transcripts", limit_basis="owner", limit_tokens=1_000_000,
                            now=NOW, root=TRANSCRIPTS)
    projected = fb.fleet_budget(source="transcripts", limit_basis="owner",
                                limit_tokens=1_000_000, forecast_agents=5,
                                forecast_minutes=60, now=NOW, root=TRANSCRIPTS)
    assert projected["projected_pct"] > plain["projected_pct"]
    assert projected["headroom_pct"] < plain["headroom_pct"]


def test_block_average_burn_rate_is_not_a_per_hour_total():
    """ccusage reports only block totals, so dividing one by 60 minutes -- which is what
    treating a block as a single timestamped record does -- overstates the rate by up
    to 5x and flipped a healthy run to STOP."""
    block = {"weighted_tokens": 300.0, "block_start": NOW - timedelta(hours=5)}
    assert fb.burn_rate_from_block(block, NOW) == pytest.approx(1.0)


def test_rolling_window_over_blocks_prorates_the_edge():
    blocks = [{"start": NOW - timedelta(hours=10), "end": NOW - timedelta(hours=5),
               "weighted": 1000.0}]
    whole = fb.rolling_window_blocks(blocks, 24, NOW)
    half = fb.rolling_window_blocks(blocks, 7.5, NOW)
    assert whole == pytest.approx(1000.0)
    assert half == pytest.approx(500.0)


def test_full_result_is_json_serialisable():
    result = fb.fleet_budget(source="transcripts", now=NOW, root=TRANSCRIPTS)
    json.dumps(result)
