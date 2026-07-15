---
name: commit-signing-1password
description: Git commit signing goes through 1Password and intermittently fails from agent shells; user authorized unsigned commits per-session
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ceb4f6a8-c1d1-4a8b-878a-7b6aee0b5cfe
---

Git commits in this workspace are signed via 1Password, which intermittently fails from
non-interactive shells with `error: 1Password: failed to fill whole buffer` (works after the user
unlocks 1Password, then locks again mid-session).

**Why:** the 1Password agent needs an unlocked vault / interactive approval that agent shells can't
trigger.

**How to apply:** on the first signing failure, ask the user to unlock 1Password or authorize unsigned
commits; on 2026-07-08 they said "you can commit without signing" → use
`git -c commit.gpgsign=false commit ...` for that session. Re-ask each session rather than assuming
the authorization is standing. See [[workspace-and-repo]].
