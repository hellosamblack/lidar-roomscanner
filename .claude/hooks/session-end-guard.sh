#!/usr/bin/env bash
# Stop-hook backstop for the session-end skill — the DEGRADED path.
#
# Normally session-end runs *before* the closing commit and lands it itself, so
# the session's memory and self-improvements ride with `Closes #NNN` (issue
# #169). This hook exists for when that didn't happen: the turn is about to end,
# HEAD carries the closing commit (message contains `Closes #NNN`, per the
# session-start convention), and session-end has not run for that HEAD. Block the
# stop and remind Claude. The reminder is fed back into the same turn, so
# session-end runs without a new user message — its Phase 3 then lands the
# improvements as a follow-up `Refs #NNN` commit instead of bundling them.
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
reason="HEAD closes a governing issue (Closes #${num:-?}) but session-end has not run for this commit. session-end should have run BEFORE this commit so its memory + self-improvement edits rode with it (issue #169) — you are now on the degraded path. Run the session-end skill now: Phase 1 memory (auto memory AND a comment on #${num:-NNN}), Phase 2 improvements, then Phase 3 lands those as a follow-up 'Refs #${num:-NNN}' commit, Phase 4 handoff. It records \$(git rev-parse --git-dir)/session-end-done as its final step."

# Emit a Stop-hook block decision; Claude Code feeds `reason` back into the turn.
python3 -c '
import json, sys
print(json.dumps({"decision": "block", "reason": sys.argv[1]}))
' "$reason"
exit 0
