#!/usr/bin/env bash
# Stop-hook backstop for the session-end skill.
#
# When the turn is about to end and HEAD carries the session's closing commit
# (message contains `Closes #NNN`, per the session-start convention) but
# session-end has not yet run for that HEAD, block the stop and remind Claude to
# run it. The reminder is fed back into the same turn, so session-end runs
# without a new user message.
#
# Idempotent + loop-safe via two guards:
#   - `stop_hook_active`: once we've blocked and Claude has continued, the next
#     Stop carries this flag -> we let it stop.
#   - `$(git rev-parse --git-dir)/session-end-done`: session-end writes HEAD's sha
#     here as its last step, so a later Stop on the same HEAD does not re-fire.
#     The git dir is per-clone and untracked -> correct home for local state.
#     Resolve it with `--git-dir`, never as a literal `.git/`: in a linked
#     worktree `.git` is a *file*, so a hardcoded redirect would overwrite the
#     worktree pointer itself.
#
# The hook only ever *reminds*; it never edits files or commits.

cd "${CLAUDE_PROJECT_DIR:-.}" 2>/dev/null || exit 0

input=$(cat)

# --- loop guard: already continued from a stop hook this turn ---
active=$(printf '%s' "$input" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    d = {}
print("1" if d.get("stop_hook_active") else "0")
' 2>/dev/null)
[ "$active" = "1" ] && exit 0

# --- only meaningful inside a git repo ---
git rev-parse --git-dir >/dev/null 2>&1 || exit 0

msg=$(git log -1 --pretty=%B 2>/dev/null)
# does HEAD close a governing issue?
printf '%s' "$msg" | grep -Eqi 'closes[[:space:]]+#[0-9]+' || exit 0

head_sha=$(git rev-parse HEAD 2>/dev/null)
marker="$(git rev-parse --git-dir 2>/dev/null)/session-end-done"
if [ -f "$marker" ] && [ "$(cat "$marker" 2>/dev/null)" = "$head_sha" ]; then
    exit 0  # session-end already handled this HEAD
fi

num=$(printf '%s' "$msg" | grep -Eoi 'closes[[:space:]]+#[0-9]+' | grep -Eo '[0-9]+' | head -1)
reason="HEAD closes a governing issue (Closes #${num:-?}) but session-end has not run for this commit. Run the session-end skill now — ship checks, memory, self-improvement, handoff — which records \$(git rev-parse --git-dir)/session-end-done as its final step."

# Emit a Stop-hook block decision; Claude Code feeds `reason` back into the turn.
python3 -c '
import json, sys
print(json.dumps({"decision": "block", "reason": sys.argv[1]}))
' "$reason"
exit 0
