"""Execution guards on the `operator-request` skill.

A skill mechanism can be silently dead and still read perfectly -- this repo has already
shipped one whose label did not exist and another whose path broke inside a worktree, and
`status/fix-unverified` sat in the tracker for weeks with zero references in any skill,
which is the exact failure this skill was written to end. So these tests check the things
a careful reader cannot: that every label is real, every path exists, every MCP tool is
registered, the runbook rules are mechanically enforceable, and -- most important -- that
the bookend skills actually route here. A close-or-hold gate nobody consults is worth
nothing.
"""
from __future__ import annotations

import asyncio
import json
import re
from pathlib import Path

import pytest

from roomscan.mcp_server.server import build

REPO = Path(__file__).resolve().parents[2]
SKILLS = REPO / ".agents" / "skills"
SKILL_DIR = SKILLS / "operator-request"
SKILL = SKILL_DIR / "SKILL.md"
LIBRARY = SKILL_DIR / "references" / "step-library.md"
TEMPLATE = SKILL_DIR / "references" / "runbook-template.md"
LABELS_FIXTURE = REPO / "host" / "tests" / "fixtures" / "labels.json"

DOCS = (SKILL, LIBRARY, TEMPLATE)


@pytest.fixture(scope="module")
def skill_text():
    return SKILL.read_text()


@pytest.fixture(scope="module")
def library_text():
    return LIBRARY.read_text()


@pytest.fixture(scope="module")
def template_text():
    return TEMPLATE.read_text()


@pytest.fixture(scope="module")
def known_labels():
    return {lb["name"] for lb in json.loads(LABELS_FIXTURE.read_text())}


@pytest.fixture(scope="module")
def tool_names():
    return {t.name for t in asyncio.run(build().list_tools())}


# --------------------------------------------------------------------------------
# Shape
# --------------------------------------------------------------------------------

def test_skill_and_references_exist():
    for path in DOCS:
        assert path.is_file(), f"{path} is missing"


def test_frontmatter_has_only_the_two_fields_this_repo_uses(skill_text):
    assert skill_text.startswith("---\n")
    front = skill_text.split("---\n", 2)[1]
    keys = {ln.split(":", 1)[0] for ln in front.splitlines() if ln and not ln.startswith(" ")}
    assert keys == {"name", "description"}, keys
    assert re.search(r"^name: operator-request$", front, re.MULTILINE)


def test_description_carries_trigger_phrases(skill_text):
    front = skill_text.split("---\n", 2)[1]
    assert '"' in front, "description should quote the phrases a user would actually type"


# --------------------------------------------------------------------------------
# The `gh` quoting trap -- backticks in a double-quoted --body are substituted by the
# shell before gh sees them, and a runbook is nothing but backticks.
# --------------------------------------------------------------------------------

@pytest.mark.parametrize("doc", DOCS, ids=lambda p: p.name)
def test_gh_write_calls_always_use_body_file(doc):
    text = doc.read_text()
    for line in text.splitlines():
        if re.search(r"gh issue (comment|create|edit)", line):
            assert '--body "' not in line, line
            assert "--body '" not in line, line
    assert "<<EOF" not in text and "<<'EOF'" not in text


def test_skill_shows_the_body_file_form(skill_text, template_text):
    assert "--body-file" in skill_text
    assert "--body-file" in template_text


# --------------------------------------------------------------------------------
# Every referenced artefact actually exists
# --------------------------------------------------------------------------------

def _referenced_paths(text: str) -> set[str]:
    files = re.findall(r"`([A-Za-z0-9_.][A-Za-z0-9_./-]*\.(?:py|md|json|html|js|toml|sh))`", text)
    dirs = re.findall(r"`([A-Za-z0-9_.][A-Za-z0-9_./-]*/)`", text)
    return {p for p in set(files) | set(dirs) if "/" in p and "*" not in p}


@pytest.mark.parametrize("doc,minimum", [(SKILL, 4), (TEMPLATE, 1)], ids=["SKILL", "TEMPLATE"])
def test_named_repo_files_exist(doc, minimum):
    """The `minimum` is the point of the parametrisation: if the extraction regex ever
    stops matching, this guard would otherwise pass by checking nothing at all -- which
    is the exact shape of a check that verifies nothing."""
    candidates = _referenced_paths(doc.read_text())
    assert len(candidates) >= minimum, (
        f"only {len(candidates)} paths extracted from {doc.name}; the regex has probably "
        f"stopped matching, so this guard is checking nothing")
    missing = [c for c in candidates
               if not (REPO / c).exists() and not (SKILL_DIR / c).exists()]
    assert not missing, f"{doc.name} names paths that do not exist: {sorted(missing)}"


def test_referenced_skills_exist(skill_text):
    named = set(re.findall(r"`(session-end|status-sync|session-start|issue-fleet|"
                           r"tof-scan-diagnosis|firmware-loop)`", skill_text))
    assert named, "the skill should name the skills it hands off to"
    for name in named:
        assert (SKILLS / name / "SKILL.md").is_file(), name


# --------------------------------------------------------------------------------
# Labels
# --------------------------------------------------------------------------------

def test_labels_named_by_the_skill_are_real(skill_text, known_labels):
    """Checked against a captured tracker slice so the test stays offline; regenerate
    with `gh label list --repo <repo> --limit 200 --json name,description`."""
    used = set(re.findall(r"`(needs/[a-z-]+|status/[a-z-]+|priority/[a-z-]+)`", skill_text))
    assert used, "the skill should name at least one label"
    assert used <= known_labels, f"labels not present in the tracker: {sorted(used - known_labels)}"


def test_the_whole_needs_family_exists(known_labels):
    from tools.operator_queue import SUBTYPES, UMBRELLA
    expected = {UMBRELLA} | {f"needs/{s}" for s in SUBTYPES}
    assert expected <= known_labels, f"missing from the tracker: {sorted(expected - known_labels)}"


def test_skill_documents_every_subtype_the_code_classifies(skill_text):
    """Drift guard. `operator_queue` routes on these; if the skill and the code disagree
    an agent applies a label the queue silently cannot classify."""
    from tools.operator_queue import SUBTYPES
    documented = set(re.findall(r"`needs/([a-z-]+)`", skill_text)) - {"operator"}
    assert documented == set(SUBTYPES), (
        f"skill documents {sorted(documented)}, code classifies {sorted(SUBTYPES)}")


def test_the_orphaned_label_now_has_a_driver(skill_text):
    """`status/fix-unverified` existed unused for weeks. This skill is what drives it;
    if that reference disappears the label is orphaned again."""
    assert "status/fix-unverified" in skill_text


# --------------------------------------------------------------------------------
# The runbook rules, mechanically enforced
# --------------------------------------------------------------------------------

_FENCED = re.compile(r"```\w*\n(.*?)```", re.DOTALL)
_STEP = re.compile(r"^\s*(?:\d+|N)\.\s+(.*)$")


def _block_steps(block: str) -> list[str]:
    """Steps in one fenced block, each joined with its wrapped continuation lines.

    Steps wrap; the sentence that parks the operator is often on the second line. A
    parser that reads only the first line silently checks half of each step.
    """
    out: list[str] = []
    for line in block.splitlines():
        m = _STEP.match(line)
        if m:
            out.append(m.group(1))
        elif out and line.strip():
            out[-1] += " " + line.strip()
    return out


def _steps(text: str) -> list[str]:
    return [s for block in _FENCED.findall(text) for s in _block_steps(block)]


def test_every_step_in_the_library_declares_whose_turn_it_is(library_text):
    """The zero-ambiguity invariant: the operator must never wonder whether a step is
    theirs. An untagged step is exactly the ambiguity this skill exists to remove."""
    steps = _steps(library_text)
    assert len(steps) >= 25, (
        f"only {len(steps)} steps extracted; the parser has probably stopped matching, "
        f"so this guard is checking nothing")
    untagged = [s for s in steps if not s.startswith(("[Claude]", "[You]"))]
    assert not untagged, f"steps missing a [Claude]/[You] tag: {untagged}"


_PARKS = re.compile(r"[Ww]ait for me|I will check|I will confirm")
_RELEASES = re.compile(r"[Nn]othing more is needed from you")


def test_every_claude_step_says_what_the_operator_does_next(library_text):
    """Blocks get composed in any order, so "is a [You] step next?" cannot be answered
    from one block. The invariant that survives composition is simpler: a [Claude] step
    either parks the operator ("wait for me") or explicitly releases them ("nothing more
    is needed from you"). Saying neither leaves them guessing, which is the one thing
    this skill exists to prevent."""
    claude = [s for s in _steps(library_text) if s.startswith("[Claude]")]
    assert len(claude) >= 3, (
        f"only {len(claude)} [Claude] steps found; the parser has probably stopped matching")
    silent = [s for s in claude if not (_PARKS.search(s) or _RELEASES.search(s))]
    assert not silent, f"[Claude] steps that leave the operator guessing: {silent}"


def test_operator_steps_never_name_a_tool(library_text, tool_names):
    """Tool names belong in [Claude] steps. A `[You]` step naming one is jargon leaking
    into an instruction a non-technical operator has to execute."""
    offenders = []
    for step in _steps(library_text):
        if not step.startswith("[You]"):
            continue
        for tool in tool_names:
            if re.search(rf"\b{re.escape(tool)}\b", step):
                offenders.append((tool, step))
    assert not offenders, f"[You] steps naming an MCP tool: {offenders}"


def test_the_static_scene_trap_is_covered(library_text):
    """The laser parks mid-take on a still scene and the file still reports clean. If
    this block loses its guard the failure is silent and the take is wasted."""
    assert "rig_idle(auto_idle=False)" in library_text
    assert "rig_idle(auto_idle=True)" in library_text, "the guard must also be restored"


def test_the_bridge_ordering_trap_keeps_its_order(library_text):
    """Out of order the FileHub claims its port as WAN and the board never links."""
    block = next(b for b in _FENCED.findall(library_text) if "Unplug" in b)
    order = [i for i in (block.find("Unplug"), block.find("Power-cycle"),
                         block.find("Bridge Mode"), block.find("Plug the network cable back"))]
    assert all(i >= 0 for i in order), block
    assert order == sorted(order), "the four bridge steps are out of order"


# --------------------------------------------------------------------------------
# Template <-> code agreement
# --------------------------------------------------------------------------------

def test_template_footer_keys_match_what_the_parser_reads(template_text):
    from tools.operator_queue import parse_footer
    example = re.search(r"<!--\s*operator-request:.*?-->", template_text, re.DOTALL)
    assert example, "the template must carry a literal footer example"
    parsed = parse_footer(example.group(0))
    assert set(parsed) == {"issue", "kind", "artifact", "gate"}, parsed


def test_request_heading_is_identical_everywhere(skill_text, template_text):
    """The fulfilment half finds its way back by grepping for this heading. If the
    template drifts from the constant, every posted runbook becomes unfindable."""
    from tools.operator_queue import REQUEST_HEADING
    assert REQUEST_HEADING in template_text
    assert REQUEST_HEADING in skill_text


def test_result_headings_are_identical_everywhere(skill_text):
    from tools.operator_queue import RESULT_HEADING, RESULT_HEADING_FAIL
    assert RESULT_HEADING in skill_text
    assert RESULT_HEADING_FAIL in skill_text


# --------------------------------------------------------------------------------
# MCP surface
# --------------------------------------------------------------------------------

def test_mcp_tools_cited_by_the_skill_are_registered(skill_text, library_text, tool_names):
    cited = set(re.findall(r"`((?:rig|capture|ui|slam|operator)_[a-z_]+)\(", skill_text + library_text))
    assert cited, "the skill should name the tools it drives"
    missing = cited - tool_names
    assert not missing, f"skill cites unregistered MCP tools: {sorted(missing)}"


def test_operator_queue_is_registered(tool_names):
    assert "operator_queue" in tool_names


# --------------------------------------------------------------------------------
# Wiring -- the guard that would have caught the `status/fix-unverified` gap
# --------------------------------------------------------------------------------

@pytest.mark.parametrize("skill", ["status-sync", "session-end", "session-start", "issue-fleet"])
def test_the_bookend_skills_route_here(skill):
    """A close-or-hold gate nobody consults is worth nothing. `status/fix-unverified`
    sat in the tracker for weeks precisely because no skill referenced it."""
    text = (SKILLS / skill / "SKILL.md").read_text()
    assert "operator-request" in text, (
        f"{skill} does not reference the operator-request skill, so the gate never fires "
        f"from it")


def test_status_sync_gates_closing_on_the_decision_table():
    text = (SKILLS / "status-sync" / "SKILL.md").read_text()
    assert "operator-request" in text
    assert re.search(r"needs/operator", text), (
        "status-sync should name the hold labels it may have to apply instead of closing")


def test_agents_md_documents_the_skill():
    text = (REPO / "AGENTS.md").read_text()
    assert "operator-request" in text, "the canonical agent guidance should list the skill"
