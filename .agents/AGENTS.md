# Claude-aligned agent guidance

The canonical project instructions are [`CLAUDE.md`](../CLAUDE.md); follow that file
instead of maintaining a second rule set here.

The shared project memory is Claude's
`/home/sam/.claude/projects/-home-sam-git-personal-lidar-roomscanner/memory/MEMORY.md`.
Read its index at task start, then only the notes relevant to the task. Also read the
gitignored `.remember/now.md` and `.remember/recent.md` for current state. Update the
canonical memory (and its index) for durable lessons or decisions; keep current state
and milestones in `.remember/`; use links to primary project docs rather than
duplicate their content.

Shared skills are registered in [`skills.json`](skills.json) and resolve to
`.claude/skills`. Use those sources directly.
