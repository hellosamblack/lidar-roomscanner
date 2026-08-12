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
               exclude_areas: str = "", include_unknown: bool = True) -> dict:
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

    `priorities` and `exclude_areas` are comma-separated (`"now,next"`,
    `"firmware,host-slam"`). Set `include_unknown=False` to skip issues whose
    footprint cannot be determined from their text.
    """
    from tools.fleet_plan import plan_fleet_live
    return plan_fleet_live(
        max_agents=max_agents,
        include_priorities=tuple(p.strip() for p in priorities.split(",") if p.strip()),
        exclude_areas=tuple(a.strip() for a in exclude_areas.split(",") if a.strip()),
        include_unknown_footprint=include_unknown)


@mcp.tool()
def fleet_budget(ceiling_pct: float = 80.0, limit_basis: str = "peak",
                 limit_tokens: float = 0.0, forecast_agents: int = 0,
                 forecast_minutes: int = 0, observed_week_pct: float = 0.0,
                 observed_block_pct: float = 0.0, source: str = "auto") -> dict:
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

    Any failure returns `verdict: "unknown"` with a reason, never a number.
    """
    from tools.fleet_budget import fleet_budget as _budget
    return _budget(ceiling_pct=ceiling_pct, limit_basis=limit_basis,
                   limit_tokens=limit_tokens or None,
                   forecast_agents=forecast_agents, forecast_minutes=forecast_minutes,
                   observed_week_pct=observed_week_pct or None,
                   observed_block_pct=observed_block_pct or None,
                   source=source)
