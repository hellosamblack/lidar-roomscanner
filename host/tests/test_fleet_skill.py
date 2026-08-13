"""Execution guards on the `issue-fleet` skill and its agent definitions.

A skill mechanism can be silently dead: this repo has already shipped a skill whose
label did not exist and another whose path broke inside a worktree, and both read
perfectly. So these tests check the things a careful reader cannot: that every path
the skill names exists, every label it uses is real, every agent it dispatches is
defined, and that its `gh` invocations avoid the quoting trap that mangles a body.
"""
import json
import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
SKILL_DIR = REPO / ".agents" / "skills" / "issue-fleet"
SKILL = SKILL_DIR / "SKILL.md"
BRIEF = SKILL_DIR / "references" / "worker-brief.md"
AGENTS_DIR = REPO / ".claude" / "agents"
FLEET_AGENTS = ("fleet-worker", "fleet-worker-mech")


@pytest.fixture(scope="module")
def skill_text():
    return SKILL.read_text()


@pytest.fixture(scope="module")
def bash_blocks(skill_text):
    return re.findall(r"```bash\n(.*?)```", skill_text, re.DOTALL)


# --------------------------------------------------------------------------------
# Shape
# --------------------------------------------------------------------------------

def test_skill_and_brief_exist():
    assert SKILL.is_file()
    assert BRIEF.is_file()


def test_frontmatter_has_only_the_two_fields_this_repo_uses(skill_text):
    assert skill_text.startswith("---\n")
    front = skill_text.split("---\n", 2)[1]
    keys = {ln.split(":", 1)[0] for ln in front.splitlines() if ln and not ln.startswith(" ")}
    assert keys == {"name", "description"}, keys
    assert re.search(r"^name: issue-fleet$", front, re.MULTILINE)


def test_description_carries_trigger_phrases(skill_text):
    front = skill_text.split("---\n", 2)[1]
    assert '"' in front, "description should quote the phrases a user would actually type"


# --------------------------------------------------------------------------------
# The `gh` quoting trap
# --------------------------------------------------------------------------------

@pytest.mark.parametrize("doc", [SKILL, BRIEF])
def test_gh_comment_and_create_always_use_body_file(doc):
    """Backticks inside a double-quoted `--body` are command-substituted before `gh`
    sees them, and a footprint list is nothing but backticks. `--body-file` is the only
    safe form, and a heredoc has the same hazard."""
    text = doc.read_text()
    for line in text.splitlines():
        if re.search(r"gh issue (comment|create)", line):
            assert '--body "' not in line, line
            assert "--body '" not in line, line
    assert "<<EOF" not in text and "<<'EOF'" not in text


def test_skill_shows_the_body_file_form(skill_text):
    assert "--body-file" in skill_text


# --------------------------------------------------------------------------------
# Every referenced artefact actually exists
# --------------------------------------------------------------------------------

def _referenced_paths(text: str) -> set[str]:
    """Backticked repo paths — files with a known extension, and directories."""
    files = re.findall(r"`([A-Za-z0-9_.][A-Za-z0-9_./-]*\.(?:py|md|json|html|js|toml|sh))`", text)
    dirs = re.findall(r"`([A-Za-z0-9_.][A-Za-z0-9_./-]*/)`", text)
    return {p for p in set(files) | set(dirs) if "/" in p and "*" not in p}


@pytest.mark.parametrize("doc,minimum", [(SKILL, 4), (BRIEF, 4)])
def test_named_repo_files_exist(doc, minimum):
    """A skill that names a moved file is worse than one that names none.

    The `minimum` is the point of the parametrisation: if the extraction regex ever
    stops matching, this test would otherwise pass by checking nothing at all -- which
    is the exact shape of a guard that verifies nothing.
    """
    candidates = _referenced_paths(doc.read_text())
    assert len(candidates) >= minimum, (
        f"only {len(candidates)} paths extracted from {doc.name}; the regex has probably "
        f"stopped matching, so this guard is checking nothing")
    missing = [c for c in candidates if not (REPO / c).exists()]
    assert not missing, f"{doc.name} names paths that do not exist: {sorted(missing)}"


def test_skill_points_at_tools_that_are_registered(skill_text):
    for tool in ("fleet_plan(", "fleet_budget("):
        assert tool in skill_text
    registry = (REPO / "host" / "tests" / "test_mcp_registry.py").read_text()
    assert '"fleet_plan.py"' in registry and '"fleet_budget.py"' in registry


def test_referenced_skills_exist(skill_text):
    for name in re.findall(r"`(session-end|status-sync|code-review|session-start)`", skill_text):
        if name == "code-review":
            continue  # a built-in CLI skill, not a repo one
        assert (REPO / ".agents" / "skills" / name / "SKILL.md").is_file(), name


# --------------------------------------------------------------------------------
# Labels
# --------------------------------------------------------------------------------

def test_labels_named_by_the_skill_are_real(skill_text):
    """Checked against the captured tracker slice rather than a live `gh` call, so the
    test stays offline; the fixture is regenerated from the real repo."""
    fixture = REPO / "host" / "tests" / "fixtures" / "fleet" / "issues_open.json"
    known = {lb["name"] for issue in json.loads(fixture.read_text()) for lb in issue["labels"]}
    used = set(re.findall(r"`(status/[a-z-]+|priority/[a-z-]+|area/[a-z-]+)`", skill_text))
    used |= set(re.findall(r'"(status/[a-z-]+)"', skill_text))
    assert used, "the skill should name at least one label"
    assert used <= known, f"labels not present in the tracker: {sorted(used - known)}"


# --------------------------------------------------------------------------------
# Agent definitions
# --------------------------------------------------------------------------------

@pytest.mark.parametrize("name", FLEET_AGENTS)
def test_agent_definition_exists_and_declares_a_model(name):
    path = AGENTS_DIR / f"{name}.md"
    assert path.is_file(), f"{path} is missing; the skill dispatches to it"
    front = path.read_text().split("---\n", 2)[1]
    assert re.search(r"^name: " + re.escape(name) + r"$", front, re.MULTILINE)
    model = re.search(r"^model: (\w+)$", front, re.MULTILINE)
    assert model and model.group(1) in {"haiku", "sonnet", "opus"}, front


@pytest.mark.parametrize("name", FLEET_AGENTS)
def test_agents_are_denied_gh_and_mcp(name):
    """Workers must not reach the tracker or the MCP server: every MCP tool runs from
    the main checkout, so a worker verifying through one is verifying the wrong tree."""
    front = (AGENTS_DIR / f"{name}.md").read_text().split("---\n", 2)[1]
    tools = re.search(r"^tools: (.+)$", front, re.MULTILINE)
    assert tools, "an unrestricted worker inherits every tool, including MCP"
    granted = {t.strip() for t in tools.group(1).split(",")}
    assert not {t for t in granted if t.startswith("mcp__")}
    assert "Task" not in granted and "Agent" not in granted, "workers do not spawn workers"


@pytest.mark.parametrize("name", FLEET_AGENTS)
def test_agent_body_states_the_load_bearing_prohibitions(name):
    body = (AGENTS_DIR / f"{name}.md").read_text()
    for phrase in ("git -C", "Closes", "git add -A", "8000", "MAIN checkout"):
        assert phrase in body, f"{name} does not mention {phrase!r}"


def test_the_skill_dispatches_to_the_agents_that_exist(skill_text):
    named = set(re.findall(r"`(fleet-worker(?:-mech)?)`", skill_text))
    assert named, "the skill should name the agent types it spawns"
    assert named <= set(FLEET_AGENTS)


# --------------------------------------------------------------------------------
# Consistency between the skill and the planner it drives
# --------------------------------------------------------------------------------

def test_shared_doc_prohibition_matches_the_planner():
    """The skill forbids workers from editing exactly the files the planner drops from
    conflict detection. If those two lists drift, the planner stops seeing a real
    conflict class."""
    import importlib.util
    import sys
    spec = importlib.util.spec_from_file_location(
        "fleet_plan_consistency", REPO / "host" / "tools" / "fleet_plan.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["fleet_plan_consistency"] = mod
    spec.loader.exec_module(mod)

    brief = BRIEF.read_text() + SKILL.read_text()
    for name in ("ROADMAP.md", "AGENTS.md", "BUGS.md"):
        assert name in mod.SHARED_DOC_FILES
        assert name in brief, f"{name} is dropped by the planner but not forbidden to workers"
    assert "docs/" in mod.SHARED_DOC_PREFIXES
    assert "docs/**" in brief


# --------------------------------------------------------------------------------
# Orchestrator context cost (#182)
# --------------------------------------------------------------------------------

def test_dot_claude_is_forbidden_to_workers_as_well_as_to_the_planner(skill_text):
    """`.claude/agents/fleet-worker.md` is a worker's own contract, and definitions load
    at session start -- so an edit there would not fail visibly, it would silently
    change how the next wave behaves."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "fp_x", REPO / "host" / "tools" / "fleet_plan.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert ".claude/" in mod.SHARED_DOC_PREFIXES
    assert ".claude/**" in (BRIEF.read_text() + skill_text)


def test_the_skill_states_the_rotation_policy(skill_text):
    """A tooling-enforced gate nobody is told to obey is just a field in a dict."""
    assert "rotate" in skill_text
    assert "rotation.context_tokens" in skill_text or "rotation.session_id" in skill_text
    assert "instructions, not advice" in skill_text


def test_the_skill_names_the_handoff_file_and_requires_memory_candidates(skill_text):
    """Rotation without `memory_candidates` destroys every pre-rotation wave's lessons,
    because `session-end` only ever records the session it runs in."""
    assert ".fleet/" in skill_text
    assert "memory_candidates" in skill_text


def test_the_skill_keeps_the_handoff_out_of_the_unwritable_dot_claude_tree(skill_text):
    """A headless link cannot Write under `.claude/` even with `Write` allowlisted (#183,
    measured three ways against CLI 2.1.228), so a handoff there is one an automated
    successor can never produce. The path is load-bearing, not cosmetic."""
    stale = [ln for ln in skill_text.splitlines() if ".claude/fleet" in ln]
    assert not stale, f"handoff moved to .fleet/, but the skill still names: {stale}"


def test_the_skill_documents_the_run_state_block_a_supervisor_reads(skill_text):
    """`fleet_run.py` parses exactly this block. If the skill stops describing it, links
    write prose the supervisor reads as a crash and the chain stops after one link."""
    assert "```run-state" in skill_text
    for field in ("run_state:", "session_chain:", "waves_done:", "issues_landed:", "halt_reason:"):
        assert field in skill_text, field
    for state in ("rotated", "complete", "halted"):
        assert state in skill_text, state
    assert "no-progress" in skill_text


def test_the_skill_no_longer_mandates_an_inline_multi_issue_gh_triage_loop(skill_text):
    """The planner already fetched every body and comment thread, so the orchestrator
    should never pull a whole thread again.

    Counts *unbounded invocations* -- a `gh issue view` whose output is not narrowed by
    `--json` -- rather than every mention of the string. Prose that tells you NOT to run
    the loop necessarily names it, and a test that cannot tell an instruction from a
    prohibition would force the guidance to be deleted in order to pass.
    """
    unbounded = [ln.strip() for ln in skill_text.splitlines()
                 if "gh issue view" in ln and "--json" not in ln and ln.lstrip().startswith("gh ")]
    assert not unbounded, f"unbounded `gh issue view` invocations remain: {unbounded}"
    assert "triage" in skill_text
    assert "instead of running" in skill_text


def test_the_claim_race_read_back_survives(skill_text):
    """The one `gh issue view` that must NOT be optimised away: it reads the tracker
    *after* your own write, to catch a session that claimed the same issue in the same
    second. Labels are last-write-wins with no compare-and-swap, so deleting this
    re-opens the double-claim window the earliest-timestamp rule closes."""
    assert "Session start" in skill_text
    assert "earliest\ntimestamp wins" in skill_text or "earliest timestamp wins" in skill_text
    assert re.search(r"gh issue view NNN .*--json comments", skill_text, re.DOTALL)


def test_the_skill_tells_the_orchestrator_not_to_author_code(skill_text):
    assert "You coordinate; you do not author" in skill_text
    for path in ("host/src/", "host/tests/", "host/tools/"):
        assert path in skill_text


def test_the_skill_warns_that_a_total_will_not_show_the_win(skill_text):
    """Delegating and rotating move spend BETWEEN seats. A reader checking
    `seven_day.weighted_tokens` will see almost no change and conclude it failed."""
    assert "by_seat" in skill_text


def test_the_ledger_exists_and_disclaims_the_comparison_it_cannot_support():
    """`test_named_repo_files_exist` already requires the file; this requires it to be
    honest. The per-run spread here is ~7x, so no realistic number of runs supports a
    model-vs-model A/B, and a ledger implying one is worse than no ledger."""
    ledger = (REPO / "docs" / "fleet-ledger.md").read_text()
    assert "orch_share" in ledger
    assert "cannot" in ledger
    assert "A/B" in ledger
