#!/usr/bin/env bash
# PostToolUse counter for the "you coordinate, you do not author" rule.
# Measures off-allowlist orchestrator edits to code files and logs them to .fleet/orch-edits.log.

input=$(cat)

RS_HOOK_PAYLOAD="$input" python3 - "$PWD" <<'PY' 2>/dev/null
import json, os, sys, time
from pathlib import Path

try:
    payload = json.loads(os.environ.get("RS_HOOK_PAYLOAD") or "")
except Exception:
    sys.exit(0)

# Extract tool name and target file path across both Antigravity and Claude formats
tool = ""
raw = ""
session = payload.get("conversationId") or payload.get("session_id") or "?"

if "toolCall" in payload:
    tc = payload.get("toolCall") or {}
    tool = tc.get("name") or ""
    args = tc.get("args") or {}
    raw = args.get("TargetFile") or args.get("file_path") or args.get("path") or ""
else:
    tool = payload.get("tool_name") or ""
    raw = (payload.get("tool_input") or {}).get("file_path") or (payload.get("tool_input") or {}).get("TargetFile") or ""

if tool not in ("replace_file_content", "write_to_file", "Edit", "Write", "MultiEdit", "NotebookEdit"):
    sys.exit(0)

if not raw:
    sys.exit(0)

root = Path(sys.argv[1])
try:
    rel = Path(raw).resolve().relative_to(root.resolve()).as_posix()
except Exception:
    sys.exit(0)

# Exclude worker worktrees
if rel.startswith(".claude/worktrees/") or rel.startswith(".worktrees/"):
    sys.exit(0)

CODE = ("host/src/", "host/tests/", "host/tools/", "host/transform/", "firmware/")
if not rel.startswith(CODE):
    sys.exit(0)

log = root / ".fleet" / "orch-edits.log"
try:
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("a", encoding="utf-8") as fh:
        fh.write("\t".join([
            time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            str(session),
            tool,
            rel,
        ]) + "\n")
except Exception:
    pass
PY

echo "{}"
exit 0
