---
name: session-end
description: The end-of-session bookend to session-start — runs the checklist for
  shipping, memory, and self-improvement. Trigger when the user says "wrap up",
  "close session", "end session", "wrap things up", "close out this task";
  AND automatically, in the same turn, right after landing the
  commit that closes the session's governing issue (message contains `Closes #NNN`).
---

# Session End

The close-side bookend to `session-start`. Run the four phases in order. All phases
auto-apply without asking; present a consolidated report at the end.

## When this runs automatically

If you have just landed a commit whose message contains `Closes #NNN` — the session's
final commit, per the `session-start` convention — **continue straight into this skill
in the same turn**. Do not stop and wait to be told to wrap up: the closing commit *is*
the signal that the session is ending. (A `Stop`-hook backstop,
`.claude/hooks/session-end-guard.sh`, will block the turn and remind you if you miss it —
but the point is not to need it.)

`status-sync` remains its own skill for the per-commit doc-sync you run on *every* landing,
including mid-session ones that only `Refs #NNN`; this skill calls it (Phase 1) and adds the
once-per-session Remember / Review / Handoff phases on top.

## Phase 1: Ship It

**Commit & land (commit-to-`main`, no PR — owner workflow 2026-07-16):**
1. Run `git status` and `git branch` to see current state
2. Run the `status-sync` skill checklist — the commit must include the doc
   deltas the work implies (ROADMAP.md status, superseded annotations, memory)
3. **Stage only what this session's task actually touched — never a blanket
   `git add -A`/`git add .`** (added 2026-07-29 after a session wrap-up almost swept a
   concurrently-edited, unrelated feature into the same commit as the session's
   fix). `git status` can show modified files that predate this session or —
   on a box the owner is also actively using — are being edited *right now* in
   another terminal. Before staging:
   - Diff each modified file and ask whether the session's own work produced
     that hunk. A file can be **partially** in scope: use `git apply --cached`
     with a hand-built hunk (or `git add -p`) to stage only the relevant hunks,
     leaving the rest of the file unstaged.
   - A fast tell that a file is still moving: re-run the test suite right
     before committing. New test names or failures that weren't present
     earlier in the session mean someone is mid-edit on that file *right now*
     — leave it alone regardless of how the diff looks.
   - If most of `git status`'s modified list turns out to be out of scope,
     that's expected — commit the small in-scope set and say so in the report
     (see the closing note below), don't pull the rest in "to be tidy."
4. Commit the in-scope files with a descriptive message.
5. Land the commit on local `main` and **close any feature/worktree branch
   without a PR**: either commit directly on `main`, or fast-forward/merge the
   branch into `main` and `git branch -d` it. No `gh pr create`, no
   `gh pr merge`. Do NOT push — pushing to `origin` is a separate step the
   owner asks for explicitly (they may want to review the local commit first).
6. **If in-scope status-sync doc deltas (ROADMAP.md/CLAUDE.md) sit next to
   out-of-scope code in the same file, edit and stage only the in-scope prose**
   — don't document someone else's still-moving feature on their behalf; note
   in the closing report that it's undocumented and out of scope.

**File placement check:**
7. If any files were created or saved during this session:
   - Verify they follow the project naming convention
   - Auto-fix naming violations (rename the file)
   - Verify they're in the correct subfolder per project structure
   - Auto-move misplaced files to their correct location
8. If any document-type files (.md, .docx, .pdf, .xlsx, .pptx) were created
   at the workspace root or in code directories, move them to the docs folder
   if they belong there

**Deploy:**
9. Check if the project has a deploy skill or script
10. If one exists, run it
11. If not, skip deployment entirely — do not ask about manual deployment

**Restore what you stopped (added 2026-07-29 after leaving the owner's viewer dead a whole session):**
11a. Did this session stop, replace, or repoint an **owner-facing service**? On this project that is
    almost always `roomscan-web` on port 8000 — the address the owner has open in a browser — but the
    same applies to anything you `pkill`ed, reflashed, or pointed at a replay instead of the device.
11b. Restore it to its pre-session state and **verify** (`pgrep -af roomscan.web`, then
    `curl -s -o /dev/null -w '%{http_code}' http://localhost:8000/static/index.html` → 200, and check
    the log's `[source]` line says the device, not a `--replay`). Say in the report that you stopped
    it and that it is back — the owner cannot see your `pkill`. If a process you didn't start now
    holds the port (wrong cwd, args you didn't pass), that's likely the owner driving their own test —
    say what you see instead of killing or "restoring" over it.
11c. Whoever needs it: `ROOMSCAN_NO_BROWSER=1 setsid host/.venv/bin/python -m roomscan.web
    > /tmp/web-live.log 2>&1 < /dev/null &` with `dangerouslyDisableSandbox` (the sandbox kills
    listeners — see the `agent-sandbox-port-binding` memory).

**Release the issue claim:**
12. Drop the `status/in-progress` label `session-start` added, on every issue this session claimed —
    whether or not the work finished. A stale claim makes the next session either duplicate the
    check or steer around work nobody is doing:

    ```bash
    gh issue edit NNN --repo hellosamblack/lidar-roomscanner --remove-label "status/in-progress"
    ```

    If the session is ending with the issue still open and unfinished, say so in the outcome comment
    (Phase 4's handoff) — the comment is the handoff, the label is not.

**Task cleanup:**
13. Check the task list for in-progress or stale items
14. Mark completed tasks as done, flag orphaned ones

## Phase 2: Remember It

Review what was learned during the session. Decide where each piece of
knowledge belongs in the memory hierarchy:

**Memory placement guide** — this project has three durable destinations, plus one
short-horizon one:
- **Auto memory** (Claude writes for itself, one fact per file) — debugging insights,
  patterns discovered during the session, project quirks. Lives **outside the repo**, at
  `~/.claude/projects/-home-sam-git-personal-lidar-roomscanner/memory/`, with a one-line
  pointer per file in that directory's `MEMORY.md` index. Before writing, check for an
  existing file that already covers it and update that instead of duplicating.
- **`AGENTS.md`** (repo root; instructions every agent reads) — permanent project rules,
  conventions, architecture decisions, owner directives. `CLAUDE.md` and `.agents/AGENTS.md`
  are symlinks to it, so edit the one file. Anything binding and long-form belongs in
  `docs/engineering-practices.md` instead, which `AGENTS.md` points at.
- **A skill under `.agents/skills/`** — when the knowledge is procedural (a checklist, a
  build/flash ritual, an environment fact needed at a specific moment) rather than a fact.
  This is the `milestone-retro` rule's preferred destination.
- **`.remember/now.md` / `.remember/recent.md`** (gitignored, shared with Codex) —
  short-horizon operational state: what is mid-flight right now. Not durable memory.

**Decision framework:**
- Is it a permanent project convention or owner directive? → `AGENTS.md` (or
  `docs/engineering-practices.md` if it needs more than a paragraph)
- Is it procedural — something to do at a particular moment? → the governing skill
- Is it a pattern or insight discovered this session? → auto memory (+ its `MEMORY.md` line)
- Is it in-flight state for the next few hours? → `.remember/now.md`
- Is it already recorded by the code, tests, or git history? → don't write it anywhere

Note anything important in the appropriate location.

## Phase 3: Review & Apply

Analyze the conversation for self-improvement findings. If the session was
short or routine with nothing notable, say "Nothing to improve" and proceed
to finish.

**Auto-apply all actionable findings immediately** — do not ask for approval
on each one. Apply the changes, commit them, then present a summary of what
was done.

**Finding categories:**
- **Skill gap** — Things Claude struggled with, got wrong, or needed multiple
  attempts
- **Friction** — Repeated manual steps, things user had to ask for explicitly
  that should have been automatic
- **Knowledge** — Facts about projects, preferences, or setup that Claude
  didn't know but should have
- **Automation** — Repetitive patterns that could become skills, hooks, or
  scripts

**Action types** (same destinations as Phase 2):
- **`AGENTS.md`** — Edit the canonical project guidance (repo root)
- **`docs/engineering-practices.md`** — Add a binding convention that needs more than a paragraph
- **Skill** — Create or update a skill under `.agents/skills/`; add a hook under `.claude/hooks/`
  (and register it in `.claude/settings.json`) when the behavior must fire without being remembered
- **Auto memory** — Save an insight for future sessions, with its `MEMORY.md` index line
- **Tool** — Land the capability as an MCP tool under `host/src/roomscan/mcp_server/`
  (the `AGENTS.md` "New tools land in the MCP server" rule), not only as a `host/tools/` script

Present a summary after applying, in two sections — applied items first,
then no-action items. **If Phase 1 step 3 found out-of-scope changes left
uncommitted on purpose (someone else's WIP, a concurrently-edited feature),
say so explicitly here** — name the files/feature and that they're
untouched and undocumented, so the owner knows it's still theirs to land:

## Phase 4: Next Steps & Handoff

Provide the owner with a clear forward-looking summary covering three areas:

**Next Development Steps:**
- List the logical next tasks/features that follow from this session's work
- Reference ROADMAP.md phase/milestone context where applicable
- Estimate scope (small/medium/large) and any blockers or dependencies
- Flag any architectural decisions that unlock or constrain future work

**Recordings & Verifications Needed from Owner:**
- Capture sessions or hardware tests that depend on physical actions (moving
  devices, changing board settings, rescanning a space, etc.)
- Validation runs needed to gate the next milestone or confirm a fix works
  in production (e.g., "full-room sweep with the current firmware to verify
  tracking doesn't lose frames in this layout")
- UI/UX validation steps that require human eyes or a real handheld use case
- Any data the session assumed but didn't verify (e.g., "assumes the mag cal
  is valid — run `capture_magcheck` on the latest capture")

**Risks & Opportunities (to be recorded in docs):**
- **Risks** — Known edge cases, second-order failure modes, or performance
  cliffs that the next session should watch for or test against
- **Opportunities** — Nice-to-have optimizations, follow-up improvements, or
  architectural cleanup that would pay off if time allows
- For each item: suggest a documentation home (a GitHub issue — `gh issue create --label bug`
  for a defect, `--label work-item` for forward-looking work — a CLAUDE.md note, or a new
  memory file) so it persists across sessions

All three sections can be brief if the session was a small bug fix or
maintenance task with no forward-looking implications. If none apply, say
"No forward-looking items" and proceed to the closing summary.

## Closing action — mark this session done (idempotency)

As the very last step, record the current HEAD so the `Stop`-hook backstop does not
re-prompt on later turns:

```bash
git rev-parse HEAD > "$(git rev-parse --git-dir)/session-end-done"
```

Derive the path with `git rev-parse --git-dir`, never hardcode `.git/` — in a linked worktree
(which `status-sync` explicitly sanctions) `.git` is a *file*, so a hardcoded redirect fails and
the hook, which resolves the same way to `.git/worktrees/<name>/`, would keep re-prompting.

The git dir is per-clone and untracked, so this is local state, never committed. The hook
(`.claude/hooks/session-end-guard.sh`) fires only when HEAD carries `Closes #NNN` **and**
this marker does not already equal HEAD — so writing it here is what closes the loop.

---

Worked example of Phases 3–4 output:

Findings (applied):

1. ✅ Skill gap: Cost estimates were wrong multiple times
   → [CLAUDE.md] Added token counting reference table

2. ✅ Knowledge: Worker crashes on 429/400 instead of retrying
   → [Rules] Added error-handling rules for worker

---
No action needed:

4. Knowledge: Discovered X works this way
   Already documented in CLAUDE.md

---

## Phase 4 Summary: Next Steps & Handoff

**Next development steps:**
1. Implement worker retry logic for 429/400 errors (medium scope, blocks Release 2.0)
2. Refactor cost estimation module to use new token table (small scope, technical debt)

**Owner recordings & verifications needed:**
- Run a production load test under 10k concurrent requests to validate retry behavior
- Benchmark cost accuracy on 3 representative workloads before next release

**Risks & opportunities:**
- **Risk**: Token counting off-by-one on edge cases (e.g., tool calls in system prompts)
  → Document in [CLAUDE.md] "Known limitations" section, add test coverage
- **Opportunity**: Cost estimation could feed a real-time budget dashboard
  → Proposed as Phase 3 follow-up if UX validates demand