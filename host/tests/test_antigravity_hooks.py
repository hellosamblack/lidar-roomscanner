"""Tests for Antigravity-specific hooks and configuration.

Ensures .agents/hooks.json and .agents/hooks/ scripts are functional and adhere to
Antigravity's lifecycle hooks contract without interfering with Claude Code's .claude/ setup.
"""
import json
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
AGENTS_HOOKS_JSON = REPO / ".agents" / "hooks.json"
GUARD_HOOK = REPO / ".agents" / "hooks" / "session-end-guard.sh"
ORCH_HOOK = REPO / ".agents" / "hooks" / "orch-edit-count.sh"


def test_agents_hooks_json_is_valid():
    assert AGENTS_HOOKS_JSON.exists(), ".agents/hooks.json must exist"
    data = json.loads(AGENTS_HOOKS_JSON.read_text(encoding="utf-8"))
    assert "session-end-guard" in data
    assert "Stop" in data["session-end-guard"]
    assert "orch-edit-count" in data
    assert "PostToolUse" in data["orch-edit-count"]


def test_hook_scripts_are_executable():
    assert os.access(GUARD_HOOK, os.X_OK), f"{GUARD_HOOK} must be executable"
    assert os.access(ORCH_HOOK, os.X_OK), f"{ORCH_HOOK} must be executable"


def test_orch_hook_counts_antigravity_replace_file_content(tmp_path):
    log = tmp_path / ".fleet" / "orch-edits.log"
    payload = {
        "conversationId": "conv-test-1",
        "toolCall": {
            "name": "replace_file_content",
            "args": {"TargetFile": str(tmp_path / "host/src/roomscan/web.py")},
        },
    }
    proc = subprocess.run(
        [str(ORCH_HOOK)],
        input=json.dumps(payload),
        text=True,
        capture_output=True,
        cwd=str(tmp_path),
    )
    assert proc.returncode == 0
    assert proc.stdout.strip() == "{}"
    assert log.exists()
    lines = log.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    ts, session, tool, rel = lines[0].split("\t")
    assert (session, tool, rel) == ("conv-test-1", "replace_file_content", "host/src/roomscan/web.py")


def test_orch_hook_ignores_worktrees_and_docs(tmp_path):
    log = tmp_path / ".fleet" / "orch-edits.log"
    for path in [".claude/worktrees/issue-1/host/src/web.py", "docs/fleet-ledger.md", "README.md"]:
        payload = {
            "conversationId": "conv-test-1",
            "toolCall": {
                "name": "write_to_file",
                "args": {"TargetFile": str(tmp_path / path)},
            },
        }
        proc = subprocess.run(
            [str(ORCH_HOOK)],
            input=json.dumps(payload),
            text=True,
            capture_output=True,
            cwd=str(tmp_path),
        )
        assert proc.returncode == 0
    assert not log.exists()


def test_session_end_guard_allows_non_closing_or_matching_marker(tmp_path):
    # In a non-git directory or repo without closing commit, outputs {}
    proc = subprocess.run(
        [str(GUARD_HOOK)],
        input=json.dumps({"terminationReason": "model_stop"}),
        text=True,
        capture_output=True,
        cwd=str(tmp_path),
    )
    assert proc.returncode == 0
    assert proc.stdout.strip() == "{}"
