#!/usr/bin/env bash
# PostToolUse counter for the "you coordinate, you do not author" rule (#182, #183).
#
# The rule is unenforceable by construction: the orchestrator's legitimate writes (comment
# bodies, the handoff, doc deltas, the ledger row) and its illegitimate ones (authoring
# code that belonged to a worker) use the same Edit/Write tools. One session made 57 of the
# second kind. So this does not block anything -- it MEASURES, appending one line per
# off-allowlist edit to `.fleet/orch-edits.log`, which the ledger's `orch_code_edits`
# column and `fleet_run.py`'s per-link record both read. If the count does not fall
# run-over-run, the rule is not working and the ledger says so.
#
# Two design points, both load-bearing:
#
# * **It writes a sidecar, not the handoff.** The plan called for appending into the
#   handoff's `orchestrator_code_edits` field. That cannot work: a link rewrites the whole
#   handoff with `Write`, so anything a hook appended between rewrites is destroyed by the
#   next one. Append-only sidecar, counted by session id, survives.
# * **Worker edits are excluded by PATH, not by trusting attribution.** Workers edit inside
#   `.claude/worktrees/<name>/host/src/...`; the orchestrator edits `host/src/...` in the
#   main checkout. Discriminating on the path holds however a subagent's tool call is
#   attributed -- and a worker editing host/src IS the correct division of labour, so
#   counting it would make the metric measure the opposite of what it claims.
#
# Never blocks, never fails a tool call: every path exits 0.

cd "${CLAUDE_PROJECT_DIR:-.}" 2>/dev/null || exit 0

# The payload goes through the environment, NOT stdin: `python3 - <<'PY'` feeds the heredoc
# to the interpreter on stdin, so a `json.load(sys.stdin)` inside it parses the hook's own
# source and every edit silently goes uncounted. Caught by a positive-control test; the
# negative cases all still "passed" while nothing worked at all.
input=$(cat)

RS_HOOK_PAYLOAD="$input" python3 - "$PWD" <<'PY' 2>/dev/null
import json, os, sys, time
from pathlib import Path

try:
    payload = json.loads(os.environ.get("RS_HOOK_PAYLOAD") or "")
except Exception:
    sys.exit(0)

tool = payload.get("tool_name", "")
if tool not in ("Edit", "Write", "MultiEdit", "NotebookEdit"):
    sys.exit(0)

root = Path(sys.argv[1])
raw = (payload.get("tool_input") or {}).get("file_path") or ""
if not raw:
    sys.exit(0)
try:
    rel = Path(raw).resolve().relative_to(root.resolve()).as_posix()
except Exception:
    sys.exit(0)   # outside the repo: not the orchestrator authoring this project's code

# A worker's own worktree is where code SHOULD be written. Excluded first, so a nested
# `host/src` under a worktree can never match the prefixes below.
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
            str(payload.get("session_id") or "?"),
            tool,
            rel,
        ]) + "\n")
except Exception:
    pass
PY
exit 0
