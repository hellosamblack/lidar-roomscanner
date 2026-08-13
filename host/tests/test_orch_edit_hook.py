"""Execution tests for the orchestrator-edit counting hook.

These run the hook the way Claude Code runs it — as a subprocess, with a real JSON payload
on stdin — rather than asserting on its source. Two mechanisms in this repo were once
silently dead because nothing ever executed them, and a hook is the easiest thing in the
tree to break invisibly: it fails, exits 0, and everything downstream reads its silence as
"no orchestrator edits happened", which is also what success looks like.

The registration test matters as much as the behaviour ones. A hook script that is not
wired into `.claude/settings.json` produces exactly the same (empty) log as a hook that
ran and found nothing to count.
"""
import json
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
HOOK = REPO / ".claude" / "hooks" / "orch-edit-count.sh"
SETTINGS = REPO / ".claude" / "settings.json"


def fire(project_dir: Path, *, tool="Edit", rel_path="host/src/roomscan/web.py",
         session="sess-1", raw_path=None):
    payload = {"session_id": session, "tool_name": tool,
               "tool_input": {"file_path": raw_path or str(project_dir / rel_path)}}
    proc = subprocess.run([str(HOOK)], input=json.dumps(payload), text=True,
                          capture_output=True,
                          env={"PATH": "/usr/bin:/bin", "CLAUDE_PROJECT_DIR": str(project_dir)})
    assert proc.returncode == 0, proc.stderr  # a counter must never fail a tool call
    log = project_dir / ".fleet" / "orch-edits.log"
    return log.read_text(encoding="utf-8").splitlines() if log.exists() else []


def test_counts_an_orchestrator_edit_to_code(tmp_path):
    lines = fire(tmp_path)
    assert len(lines) == 1
    ts, session, tool, rel = lines[0].split("\t")
    assert (session, tool, rel) == ("sess-1", "Edit", "host/src/roomscan/web.py")
    assert ts.endswith("Z")


@pytest.mark.parametrize("rel", ["host/src/roomscan/web.py", "host/tests/test_web.py",
                                 "host/tools/fleet_plan.py", "firmware/scanner-stream/main.c",
                                 "host/transform/CMakeLists.txt"])
def test_every_code_prefix_is_counted(tmp_path, rel):
    assert len(fire(tmp_path, rel_path=rel)) == 1


@pytest.mark.parametrize("rel", [
    ".claude/worktrees/issue-170-slug/host/src/roomscan/web.py",
    ".worktrees/issue-170-slug/host/tools/fleet_plan.py",
])
def test_a_worker_editing_inside_its_own_worktree_is_not_counted(tmp_path, rel):
    # This is the correct division of labour. Counting it would make the metric measure
    # the opposite of what it claims -- and the exclusion is by PATH, so it holds however
    # a subagent's tool call happens to be attributed.
    assert fire(tmp_path, rel_path=rel) == []


@pytest.mark.parametrize("rel", ["docs/fleet-ledger.md", "ROADMAP.md",
                                 ".fleet/fleet-20260813.md", "AGENTS.md"])
def test_the_orchestrators_legitimate_writes_are_not_counted(tmp_path, rel):
    # Doc deltas, the ledger row and the handoff are exactly what the orchestrator IS
    # allowed to write; a counter that flagged them would be noise nobody reads.
    assert fire(tmp_path, rel_path=rel) == []


def test_a_path_outside_the_repo_is_not_counted(tmp_path):
    assert fire(tmp_path, raw_path="/tmp/issue-body.md") == []


def test_non_editing_tools_are_ignored(tmp_path):
    assert fire(tmp_path, tool="Bash") == []
    assert fire(tmp_path, tool="Read") == []


def test_malformed_payload_exits_clean(tmp_path):
    proc = subprocess.run([str(HOOK)], input="not json at all", text=True, capture_output=True,
                          env={"PATH": "/usr/bin:/bin", "CLAUDE_PROJECT_DIR": str(tmp_path)})
    assert proc.returncode == 0
    assert not (tmp_path / ".fleet" / "orch-edits.log").exists()


def test_counts_accumulate_across_calls_and_keep_session_identity(tmp_path):
    fire(tmp_path, session="orch-a")
    fire(tmp_path, session="orch-a", rel_path="host/tools/x.py")
    lines = fire(tmp_path, session="orch-b", rel_path="firmware/y.c")
    assert len(lines) == 3
    assert sum(1 for ln in lines if "orch-a" in ln) == 2  # per-link attribution needs this


def test_the_hook_is_executable_and_registered(tmp_path):
    # A hook that exists but is not wired produces the same empty log as one that ran and
    # found nothing -- the failure is invisible from the data alone.
    import os
    assert os.access(HOOK, os.X_OK), f"{HOOK} is not executable"
    hooks = json.loads(SETTINGS.read_text(encoding="utf-8"))["hooks"]["PostToolUse"]
    commands = [h["command"] for entry in hooks for h in entry["hooks"]]
    assert any("orch-edit-count.sh" in c for c in commands)
    matchers = [entry.get("matcher", "") for entry in hooks]
    assert any("Edit" in m and "Write" in m for m in matchers)
