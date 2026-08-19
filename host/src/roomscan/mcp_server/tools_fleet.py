"""Fleet orchestration: which issues to work in parallel, and whether there is budget.

Both tools are read-only planners. Neither creates a worktree, spawns an agent,
claims an issue, or touches git state -- those stay in the orchestrator's own Bash
calls, one visible permission-gated command at a time. Burying them here would run
them from `paths.REPO`, which is always the *main* checkout no matter which worktree
the caller thinks it is in, and that is exactly how a subagent once landed a task
straight onto `main`.
"""
from __future__ import annotations

import sys

from .paths import HOST
from .server import mcp

sys.path.insert(0, str(HOST))  # `tools` is a top-level package rooted at host/


@mcp.tool()
def fleet_plan(max_agents: int = 3, priorities: str = "now,next",
               exclude_areas: str = "", include_unknown: bool = True,
               triage: bool = True) -> dict:
    """Choose a conflict-free batch of open issues for parallel workers.

    Ranks open issues by priority, doubles the score of any issue that already has
    prior work (an implementation-plan comment or a declared file scope), and adds a
    bonus for issues that gate others. Then it extracts each issue's file footprint,
    expands it through a co-edit graph built from `git log`, and selects the highest
    scorers whose footprints do not collide -- also respecting singleton runtime
    resources (one browser, one device, one GPU per wave) that no file footprint can
    express.

    Returns `batch` (each with footprint, confidence tier, resources, assigned port,
    suggested model tier and worktree name), plus `deferred`, `excluded` and `notes`.
    Read `notes`: soft conflicts and prose-inferred dependencies are surfaced there
    for your judgement rather than applied silently.

    **Read each selected issue's `triage` digest instead of running `gh issue view`.**
    This call already fetched every open issue's full body and comment thread, so a
    per-issue `gh` loop re-pays for text you are holding. The digest is bounded and
    carries: `plan_excerpt` (the LATEST implementation-plan comment, not the first --
    plans get superseded), `latest_comment` with its `kind` (`session_start` /
    `implementation_plan` / `operator_request` / `worker_report` / `other`),
    `body_excerpt`, `comment_count`, `acceptance_hint`, and `chars_elided`.

    `acceptance_hint` pre-sorts the verification veto: `visual` needs the browser,
    `hardware` needs the rig. If this session lacks the tool the hint names, that is
    the veto -- and you know it before spawning anything.

    When `chars_elided` is large and the digest has not settled the question, spawn an
    `Explore` agent to read the thread and report back a short verdict. Do not read it
    yourself: a thread you read stays in your context for every remaining turn of the
    run, while a subagent's context is discarded when it returns.

    Held issues arrive in `excluded` labelled `needs/operator (<subtype>)`; the planner
    vetoes them outright, so no `operator_queue()` cross-check is needed to filter the
    batch. Call that tool for its `problems` field, or when writing an operator request.

    `priorities` and `exclude_areas` are comma-separated (`"now,next"`,
    `"firmware,host-slam"`). Set `include_unknown=False` to skip issues whose
    footprint cannot be determined from their text. `triage=False` drops the digests.
    """
    from tools.fleet_plan import plan_fleet_live
    return plan_fleet_live(
        max_agents=max_agents,
        include_priorities=tuple(p.strip() for p in priorities.split(",") if p.strip()),
        exclude_areas=tuple(a.strip() for a in exclude_areas.split(",") if a.strip()),
        include_unknown_footprint=include_unknown,
        triage=triage)


@mcp.tool()
def fleet_budget(ceiling_pct: float = 80.0, limit_basis: str = "peak",
                 limit_tokens: float = 0.0, forecast_agents: int = 0,
                 forecast_minutes: int = 0, observed_week_pct: float = 0.0,
                 observed_block_pct: float = 0.0, source: str = "auto",
                 session_id: str = "", project_dir: str = "") -> dict:
    """Estimate Claude usage against a not-to-exceed ceiling before starting a wave.

    **ASK THE OWNER FOR THEIR CURRENT PERCENTAGES AND PASS THEM.** `observed_week_pct`
    and `observed_block_pct` are the figures they read off their own client. Without
    `observed_week_pct` the weekly percentage comes back `None` and the verdict covers
    the 5-hour block ONLY -- which is nearly always the *non*-binding window, because it
    resets every five hours. On 2026-08-12 this tool returned `go` at every check across
    a whole multi-wave run while the owner sat at ~70-72% of a weekly ceiling declared
    as 80%; the weekly number was computed, returned, and never consulted by the verdict.

    Percentages are ANCHORED on those observations, not derived. This module's
    "weighted token" is its own invention, and calibration proved it does not convert to
    a real percentage: two readings of the same 5-hour window on the same day implied
    allowances 1.6x apart, because the model/cache/output mix differed. What IS reliable
    is the *delta* -- so the forecast projects forward from the owner's figure at a
    measured ~25M weighted tokens per weekly point (the conservative bound; a 3-worker
    Sonnet wave with review, one full suite and browser checks ran ~40M).

    Read `binding_window` to know which window drives the verdict -- a `stop` on the
    5-hour clears in hours, a `stop` on the weekly does not. `limit_basis` and
    `seven_day.pct_basis` name every denominator, and `coverage.includes_subagents`
    says whether worker transcripts were counted (for a fleet they are most of the
    spend). `forecast_agents`/`forecast_minutes` matter because parallel workers are
    invisible until they return.

    **`verdict` can be `"rotate"` or `"rotate_hard"`, and neither is advice.** They mean
    your own context has grown expensive enough that continuing costs more than handing
    off: finish the wave in flight, write the handoff file, and stop. `rotate_hard` means
    do not start the next issue either. `budget_verdict` always carries the original
    `go`/`reduce`/`stop`/`unknown`, and `binding_constraint` says whether the budget or
    your context drove the call. Rotating beats shrinking the wave -- a smaller wave in a
    400K context still pays 400K on every turn.

    `rotation` carries the evidence: `context_tokens`, `turns`, and
    `rotation_saving_weighted` versus `rotation_cost_weighted`. Those are weighted-token
    DELTAS, not percentages -- this module has no trustworthy denominator, and the saving
    is what a fresh session avoids re-sending, not a share of anything.

    `rotation.confidence` is `"inferred"` when the session was identified by recency
    (the newest top-level transcript record, which is normally you). Pass
    `session_id` from the first call's `rotation.session_id` to pin it for the rest of
    the run. If the context cannot be measured the verdict silently covers the budget
    windows only and says so in `notes` -- it does NOT become `unknown`.

    Read `seven_day.by_seat`: `top_level` is your own turns, `subagent` is every worker
    you spawned. Delegating moves spend between those two buckets and rotating shrinks
    the first, so both changes look like nothing against a total and only show up here.

    Any failure returns `verdict: "unknown"` with a reason, never a number.
    """
    from tools.fleet_budget import fleet_budget as _budget
    return _budget(ceiling_pct=ceiling_pct, limit_basis=limit_basis,
                   limit_tokens=limit_tokens or None,
                   forecast_agents=forecast_agents, forecast_minutes=forecast_minutes,
                   observed_week_pct=observed_week_pct or None,
                   observed_block_pct=observed_block_pct or None,
                   source=source,
                   session_id=session_id or None, project_dir=project_dir or None)


@mcp.tool()
def operator_queue(detailed: bool = True) -> dict:
    """What is waiting on the owner: open issues held for a physical or human action.

    Answers "what do you need from me?" in one call. Returns every open issue labelled
    `needs/operator` with its subtype (`capture`, `network`, `hardware`, `eyes`,
    `decision`), age, status labels, and the parsed footer of its latest
    `## 🔧 Operator Request` comment -- which names the artifact expected and the gate
    that will score it.

    Read `problems`. It reports held issues whose runbook comment is missing or has no
    parseable footer, or whose comments could not be read: the label is a promise that
    the instructions exist, and an issue carrying it with nothing to act on is a dead
    end the issue list cannot show you. One exception: a `priority/later` hold with no
    runbook yet is a deliberately *parked* hold, not a broken one -- it is labelled
    `needs/operator` early, before its runbook is written, so it stays findable while
    queued behind the now/next tiers. That case is reported per-issue as
    `pending[i]["parked"] == True`, not folded into `problems` -- nobody is waiting on
    it yet. A malformed footer or a comment-read failure is still a problem even on a
    `priority/later` issue, since either means a runbook exists (or was attempted) and
    is broken, which `priority/later` cannot excuse.

    This is also the input to batch mode -- group the result by `kind` and by shared
    setup so the owner powers the rig up once rather than once per issue. See the
    `operator-request` skill.

    `detailed=False` skips the per-issue comment fetch for a fast label-only listing.
    """
    from tools.operator_queue import collect
    return collect(include_comments=detailed)


@mcp.tool()
def operator_page(out: str = "", repo: str = "") -> dict:
    """Generate the owner-facing HTML page of everything waiting on the owner.

    `operator_queue()` is the flat list; this is the *plan*. It scrapes every
    `## 🔧 Operator Request` runbook and works out what the queue actually costs the
    owner, which is not one trip per issue: it resolves the aliases that say "covered by
    the request on #NNN", clusters near-duplicate setup steps so a shared power-up is
    done once per sitting rather than once per issue, and groups issues that need the
    same *venue* -- the same metal-free spot, the same blank wall -- into one setup.

    Returns the plan (`sittings`, each with `legs` in the order to do them, plus
    `riders` and `missing`) and writes a self-contained page the owner can tick through
    while holding the hardware. Written by default to the web server's static dir, so it
    is reachable at <http://localhost:8000/static/operator.html> -- next to the app the
    runbooks already tell the owner to open. Pass `out` to write elsewhere.

    Two things worth reading in the result. `missing` lists issues held with no runbook:
    they cannot be planned, and they are the same dead end `operator_queue().problems`
    reports. `riders` lists issues that close for free with another issue's action --
    they appear nowhere else on the page, so the trip count stays honest.

    Regenerate after posting or revising any runbook, or after any `needs/*` label changes on
    any issue (including a close) -- the page is a snapshot, not live, and a label change with
    no runbook edit is the case that is easy to miss (#183, 2026-08-14).
    """
    from tools.operator_page import DEFAULT_OUT, build_page
    from tools.operator_queue import REPO
    return build_page(repo=repo or REPO, out=out or DEFAULT_OUT)


@mcp.tool()
def tracker_lint(repo: str = "") -> dict:
    """Lint GitHub issue labels against mechanical invariants.

    Reports violations of label rules that must hold for the tracker to stay coherent:
    1. `needs/operator` requires a needs/* subtype AND a status hold
    2. every open issue has exactly one priority/now|next|later
    3. exactly one of bug/work-item/data-collection per issue
    4. (advisory) area/* should be present

    Returns `violations` (a list of violation dicts), `summary` (counts per rule),
    and `ok` (True if no violations found). Violations are identified by rule name
    and a human-readable message for manual triage.
    """
    from tools.tracker_lint import lint_issues, fetch_issues, REPO as DEFAULT_REPO
    issues = fetch_issues(repo=repo or DEFAULT_REPO)
    return lint_issues(issues)
