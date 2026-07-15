---
name: milestone-self-improvement
description: "Owner directive — after every milestone, run the milestone-retro skill to convert friction into skills/scripts/references before starting the next phase"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ceb4f6a8-c1d1-4a8b-878a-7b6aee0b5cfe
---

Owner directive (2026-07-08): **after every milestone** (phase completion or major merge to main),
create/modify skills — with references and scripts — capturing whatever would have made that development
push easier.

**Why:** hardware rituals and env facts were being re-derived from prose by every subagent (capture
tooling was rebuilt ~6 times across Phases 1-2.5).

**How to apply:** run the repo's `milestone-retro` skill (`.claude/skills/milestone-retro/SKILL.md`)
before starting the next phase; it has the procedure, hard rules (>2 subagents repeating a ritual → make
it a script), and a seeded backlog (host/tools/capture.py is the top item, due at the Phase 3 retro).
Encoded in repo: CLAUDE.md self-improvement rule + docs/engineering-practices.md section. See
[[mapping-pipeline-plan]].
