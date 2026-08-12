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
                 forecast_minutes: int = 0, source: str = "auto") -> dict:
    """Estimate Claude usage against a not-to-exceed ceiling before starting a wave.

    Reports the current 5-hour block and a **rolling** 168-hour window (not a calendar
    week), plus a `go` / `reduce` / `stop` verdict compared against the *projected*
    load once `forecast_agents` run for `forecast_minutes` -- parallel workers are
    invisible until they return, so judging on current load alone can wave through a
    batch that lands well past the ceiling.

    Every reading is an estimate: no quota API is reachable from an agent on this box.
    Read two fields before trusting a number. `limit_basis` names the denominator --
    either this account's heaviest completed block (`"peak"`) or an owner-supplied
    figure (`limit_basis="owner"` with `limit_tokens`), because a percentage of an
    invented denominator is a fabricated number. `coverage.includes_subagents` says
    whether worker transcripts were actually counted; they live one directory deeper
    than the main ones, and for a fleet they are most of the spend.

    Any failure returns `verdict: "unknown"` with a reason, never a number.
    """
    from tools.fleet_budget import fleet_budget as _budget
    return _budget(ceiling_pct=ceiling_pct, limit_basis=limit_basis,
                   limit_tokens=limit_tokens or None,
                   forecast_agents=forecast_agents, forecast_minutes=forecast_minutes,
                   source=source)
