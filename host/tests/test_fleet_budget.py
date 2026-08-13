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
# Records-for-detail / blocks-for-totals
# --------------------------------------------------------------------------------

def test_the_two_weekly_derivations_actually_disagree(read, blocks_payload):
    """Power check for the guard below: if block-prorated and record-summed weekly
    totals happened to agree, `test_ccusage_path_prorates_the_weekly_over_blocks`
    would pass no matter which one the code picked, and would be worth nothing.

    On the captured fixtures they disagree by ~2600x -- the ccusage payload is a real
    account's week, the synthetic transcript tree is eight hand-written records."""
    records, _ = read
    blocks = fb.parse_ccusage_blocks(blocks_payload)
    from_blocks = fb.rolling_window_blocks(blocks, fb.WEEK_HOURS, NOW)
    from_records = fb.rolling_window(records, fb.WEEK_HOURS, NOW)
    assert from_blocks != from_records
    assert from_blocks > from_records * 100


def test_ccusage_path_prorates_the_weekly_over_blocks(blocks_payload):
    """The regression guard for keeping `records` on the ccusage path.

    `read_transcripts` is now called for its records as well as its coverage, because
    the seat/model/context breakdowns are record-level facts. The headline totals must
    not notice. This previously read `if records`, which was equivalent only while the
    ccusage path threw its records away -- flipping it to the record derivation would
    have reported ~104K where the block derivation reports ~271M, i.e. near-infinite
    headroom, on every call the skill makes."""
    r = fb.fleet_budget(source="ccusage", ccusage_payload=blocks_payload,
                        limit_basis="owner", limit_tokens=1_000_000,
                        now=NOW, root=TRANSCRIPTS)
    blocks = fb.parse_ccusage_blocks(blocks_payload)
    assert r["seven_day"]["weighted_tokens"] == round(
        fb.rolling_window_blocks(blocks, fb.WEEK_HOURS, NOW), 1)


def test_ccusage_path_still_reports_subagent_coverage(blocks_payload):
    """The reason `read_transcripts` was called here in the first place: ccusage's own
    payload cannot say whether worker transcripts were counted."""
    r = fb.fleet_budget(source="ccusage", ccusage_payload=blocks_payload,
                        limit_basis="owner", limit_tokens=1_000_000,
                        now=NOW, root=TRANSCRIPTS)
    assert r["coverage"]["includes_subagents"] is True


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


# --------------------------------------------------------------------------------
# Record identity and the context proxy
# --------------------------------------------------------------------------------

GROWTH_PROJECT = "-proj-ctxgrowth"
GROWTH_SESSION = "sess-grow0001"


def test_records_carry_session_identity(read):
    records, _ = read
    assert all(r["session_id"] for r in records)
    assert all(r["project_dir"] for r in records)


def test_a_subagent_is_attributed_to_its_PARENT_session(read):
    """A worker's spend belongs to the run that spawned it. The transcript carries the
    parent's `sessionId`, and the filename fallback has to mirror that -- a subagent's
    own stem is `agent-<id>`, so a naive `path.stem` would invent a session per worker
    and scatter one run's spend across as many buckets as it had agents."""
    records, _ = read
    subs = [r for r in records if r["source"] == "subagent"]
    assert subs, "fixture must contain subagent records"
    assert all(not r["session_id"].startswith("agent-") for r in subs)
    assert all(r["session_id"] == "sess-aaaa1111" for r in subs)


def test_context_proxy_is_not_the_weighted_token(read):
    """`weighted` discounts cache reads by CACHE_READ_WEIGHT and scales by model, which
    is right for load and destroys the growth signal. A large-context record must show
    a context far above its weighted cost, or the rotation metric is measuring spend
    again under a different name."""
    records, _ = read
    grown = [r for r in records if r["session_id"] == GROWTH_SESSION]
    biggest = max(grown, key=lambda r: r["context"])
    assert biggest["context"] >= 400_000
    assert biggest["context"] > biggest["weighted"] * 10


# --------------------------------------------------------------------------------
# Seat and model composition
# --------------------------------------------------------------------------------

def test_by_seat_splits_the_orchestrator_from_its_workers(read):
    records, _ = read
    b = fb.breakdown(records, fb.WEEK_HOURS, NOW)
    assert b["by_seat"]["top_level"] > 0
    assert b["by_seat"]["subagent"] > 0


def test_by_model_separates_the_tiers(read):
    records, _ = read
    b = fb.breakdown(records, fb.WEEK_HOURS, NOW)
    assert "claude-opus-5" in b["by_model"]
    assert "claude-haiku-4-5-20251001" in b["by_model"]


def test_breakdown_declares_it_is_not_a_second_opinion_on_the_total(read):
    """On the ccusage path the block aggregates are authoritative and these record-level
    sums will not match them. Without the disclaimer the first reader who notices the
    mismatch 'fixes' one of the two numbers."""
    records, _ = read
    b = fb.breakdown(records, fb.WEEK_HOURS, NOW)
    assert "will not add up" in b["basis"]
    assert "ccusage" in b["basis"]


def test_seat_breakdown_is_attached_to_both_windows():
    r = fb.fleet_budget(source="transcripts", limit_basis="owner", limit_tokens=1_000_000,
                        now=NOW, root=TRANSCRIPTS)
    for window in ("five_hour", "seven_day"):
        assert "by_seat" in r[window]
        assert "by_model" in r[window]


# --------------------------------------------------------------------------------
# Context growth and the rotate verdict
# --------------------------------------------------------------------------------

def _grown(records, ctx_tokens):
    """One synthetic top-level session sitting at `ctx_tokens` of context."""
    return [{"ts": NOW - timedelta(seconds=10), "model": "claude-opus-5",
             "weighted": 1.0, "source": "top_level", "session_id": "synthetic",
             "project_dir": "-proj-x", "cache_read": ctx_tokens, "context": ctx_tokens}]


def test_rotate_fires_at_the_threshold_and_not_below():
    below = fb.session_context(_grown(None, fb.ROTATE_AT_CONTEXT - 1), NOW)
    at = fb.session_context(_grown(None, fb.ROTATE_AT_CONTEXT + 1), NOW)
    assert fb.decide_rotation(below) == "hold"
    assert fb.decide_rotation(at) == "rotate"


def test_hard_rotate_fires_above_the_hard_threshold():
    ctx = fb.session_context(_grown(None, fb.HARD_ROTATE_AT_CONTEXT + 1), NOW)
    assert fb.decide_rotation(ctx) == "rotate_hard"


def test_the_rotation_trigger_is_raw_tokens_not_weighted():
    """A weighted trigger would scale with MODEL_WEIGHT, so the same one-weekly-point
    rule would sit near 1.3M for a Sonnet/Fable-class orchestrator and never fire --
    one model would run 600-turn sessions while another rotated at 200, and any later
    model comparison would be measuring the rotation policy instead of the model."""
    big = fb.ROTATE_AT_CONTEXT + 50_000
    opus = _grown(None, big)
    cheap = [{**opus[0], "model": "claude-sonnet-5"}]
    assert fb.decide_rotation(fb.session_context(opus, NOW)) == "rotate"
    assert fb.decide_rotation(fb.session_context(cheap, NOW)) == "rotate"


def test_rotation_saving_is_a_weighted_delta_not_a_percentage():
    """This module has no trustworthy denominator; a percentage here would be exactly
    the fabricated-authority failure its own calibration note warns about."""
    ctx = fb.session_context(_grown(None, 400_000), NOW)
    assert isinstance(ctx["rotation_saving_weighted"], float)
    assert "%" not in ctx["saving_basis"]
    assert "%" not in ctx["cost_basis"]


def test_the_saving_reflects_the_model_weight_even_though_the_trigger_does_not():
    opus = fb.session_context(_grown(None, 400_000), NOW)
    cheap = fb.session_context([{**_grown(None, 400_000)[0], "model": "claude-sonnet-5"}], NOW)
    assert opus["rotation_saving_weighted"] > cheap["rotation_saving_weighted"]


def test_a_stale_newest_record_refuses_to_identify_a_session():
    """The newest-top-level-record heuristic only holds while the caller is actually
    live. Guessing on a day-old record would attribute a stranger's context to you."""
    stale = _grown(None, 400_000)
    stale[0]["ts"] = NOW - timedelta(hours=3)
    ctx = fb.session_context(stale, NOW)
    assert ctx["ok"] is False
    assert fb.decide_rotation(ctx) == "unknown"


def test_a_pinned_session_id_beats_recency():
    older = {**_grown(None, 400_000)[0], "session_id": "wanted",
             "ts": NOW - timedelta(minutes=30)}
    newer = {**_grown(None, 50_000)[0], "session_id": "noise"}
    ctx = fb.session_context([older, newer], NOW, session_id="wanted")
    assert ctx["confidence"] == "pinned"
    assert ctx["context_tokens"] == 400_000


def test_a_pinned_session_that_does_not_exist_is_not_silently_inferred():
    ctx = fb.session_context(_grown(None, 400_000), NOW, session_id="absent")
    assert ctx["ok"] is False


# --------------------------------------------------------------------------------
# Verdict composition
# --------------------------------------------------------------------------------

def test_stop_outranks_rotate_and_rotate_outranks_reduce():
    """`rotate` beats `reduce` because rotating IS how you reduce -- a smaller wave in a
    400K context still pays 400K on every turn. `stop` beats `rotate` because a rotated
    session with no budget has burned a fresh 45K only to be told to stop."""
    assert fb.combine_verdicts("stop", "rotate") == "stop"
    assert fb.combine_verdicts("reduce", "rotate") == "rotate"
    assert fb.combine_verdicts("go", "rotate_hard") == "rotate_hard"
    assert fb.combine_verdicts("reduce", "hold") == "reduce"


def test_the_original_budget_verdict_survives_alongside_the_combined_one():
    """Callers that only know go/reduce/stop/unknown must not break when the combined
    verdict becomes `rotate`."""
    r = fb.fleet_budget(source="transcripts", limit_basis="owner", limit_tokens=1_000_000,
                        now=NOW, root=TRANSCRIPTS)
    assert r["budget_verdict"] in fb.VERDICTS


def test_unmeasurable_context_does_not_make_the_verdict_unknown():
    """`unknown` means 'stop and ask' to every caller. Firing it whenever the
    newest-record heuristic misfires is how a useful guard gets deleted."""
    r = fb.fleet_budget(source="transcripts", limit_basis="owner", limit_tokens=1_000_000,
                        now=NOW + timedelta(days=30), root=TRANSCRIPTS,
                        session_id="does-not-exist")
    assert r["rotation"]["ok"] is False
    assert r["verdict"] == r["budget_verdict"]
    assert r["verdict"] != "unknown"
    assert any("context growth not measured" in n for n in r["notes"])


def test_binding_window_is_not_overloaded_with_context():
    """`binding_window` answers 'which WINDOW drives the verdict'. Context is not a
    window, so it gets its own field rather than corrupting that one's meaning."""
    r = fb.fleet_budget(source="transcripts", limit_basis="owner", limit_tokens=1_000_000,
                        now=NOW, root=TRANSCRIPTS)
    assert r["binding_window"] in ("five_hour", "seven_day")
    assert "binding_constraint" in r
