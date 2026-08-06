# Claude-aligned Codex guidance

`CLAUDE.md` at this repository root is the canonical project instruction set. Read
and follow it before taking project actions. Do not copy, paraphrase, or maintain a
second version of its rules here; changes to durable project guidance belong in
`CLAUDE.md`.

## Shared Claude memory

The canonical project memory is outside the checkout at:

`/home/sam/.claude/projects/-home-sam-git-personal-lidar-roomscanner/memory/MEMORY.md`

At the start of a task, read this index and only the linked memory notes relevant to
the work. Also read `.remember/now.md` and `.remember/recent.md`: they are gitignored
but shared, short-horizon operational state maintained by Claude. Treat these as the
shared working history with Claude; do not create a Codex copy in the repository.

When a task produces a durable decision, measured result, operational constraint, or
reusable lesson, update the relevant canonical memory note and its index entry. Keep
immediate current state and next steps in `.remember/now.md`; record a completed
milestone in `.remember/recent.md` (and a dated note when useful). Avoid duplicating
information already captured in project docs, `CLAUDE.md`, `ROADMAP.md`, or `BUGS.md`:
memory notes should link to those primary sources.

## Shared skills

Project skills are registered through `.agents/skills.json`, which points to
`.claude/skills`. Use the applicable shared skill exactly as discovered. Do not fork
or duplicate these skills for Codex.

## Source ownership

- Durable operating rules: `CLAUDE.md`
- Project history / decisions: Claude memory index and its linked notes
- Immediate shared state: `.remember/now.md` and `.remember/recent.md` (gitignored)
- Product plans and current status: `ROADMAP.md` (standing decisions + the forward-looking Work-item
  register; completed-phase history in `docs/roadmap-history.md`), `BUGS.md` (an index over per-bug
  files in `bugs/`), and `docs/`
- Reusable workflows: `.claude/skills/`
