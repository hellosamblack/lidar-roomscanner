#!/usr/bin/env bash
# Antigravity Stop hook backstop for the session-end skill.
#
# Normally session-end runs before the closing commit and lands it itself.
# If a commit landed with `Closes #NNN` and session-end marker is missing,
# block the stop and remind the agent to finish session-end phases.

input=$(cat)

# Only meaningful inside a git repo
git rev-parse --git-dir >/dev/null 2>&1 || { echo "{}"; exit 0; }

msg=$(git log -1 --pretty=%B 2>/dev/null)
# does HEAD close a governing issue?
if ! printf '%s' "$msg" | grep -Eqi 'closes[[:space:]]+#[0-9]+'; then
    echo "{}"
    exit 0
fi

head_sha=$(git rev-parse HEAD 2>/dev/null)
marker="$(git rev-parse --git-dir 2>/dev/null)/session-end-done"
if [ -f "$marker" ] && [ "$(cat "$marker" 2>/dev/null)" = "$head_sha" ]; then
    echo "{}"
    exit 0
fi

num=$(printf '%s' "$msg" | grep -Eoi 'closes[[:space:]]+#[0-9]+' | grep -Eo '[0-9]+' | head -1)
reason="HEAD closes a governing issue (Closes #${num:-?}) but session-end has not run for this commit. session-end should have run BEFORE this commit so its memory + self-improvement edits rode with it. Run the session-end skill now: Phase 1 memory (auto memory AND a comment on #${num:-NNN}), Phase 2 improvements, then Phase 3 lands those as a follow-up 'Refs #${num:-NNN}' commit, Phase 4 handoff. It records \$(git rev-parse --git-dir)/session-end-done as its final step."

python3 -c '
import json, sys
print(json.dumps({"decision": "continue", "reason": sys.argv[1]}))
' "$reason"
exit 0
