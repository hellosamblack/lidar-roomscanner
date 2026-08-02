---
name: wrap-up
description: Use when user says "wrap up", "close session", "end session",
  "wrap things up", "close out this task", or invokes /wrap-up — runs
  end-of-session checklist for shipping, memory, and self-improvement
---

# Session Wrap-Up

Run three phases in order. All phases auto-apply without asking; present a
consolidated report at the end.

## Phase 1: Ship It

**Commit & land (commit-to-`main`, no PR — owner workflow 2026-07-16):**
1. Run `git status` and `git branch` to see current state
2. Run the `status-sync` skill checklist — the commit must include the doc
   deltas the work implies (ROADMAP.md status, superseded annotations, memory)
3. **Stage only what this session's task actually touched — never a blanket
   `git add -A`/`git add .`** (added 2026-07-29 after a wrap-up almost swept a
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

**Task cleanup:**
12. Check the task list for in-progress or stale items
13. Mark completed tasks as done, flag orphaned ones

## Phase 2: Remember It

Review what was learned during the session. Decide where each piece of
knowledge belongs in the memory hierarchy:

**Memory placement guide:**
- **Auto memory** (Claude writes for itself) — Debugging insights, patterns
  discovered during the session, project quirks. Tell Claude to save these:
  "remember that..." or "save to memory that..."
- **CLAUDE.md** (instructions for Claude) — Permanent project rules,
  conventions, commands, architecture decisions that should guide all future
  sessions
- **`.claude/rules/`** (modular project rules) — Topic-specific instructions
  that apply to certain file types or areas. Use `paths:` frontmatter to scope
  rules to relevant files (e.g., testing rules scoped to `tests/**`)
- **`CLAUDE.local.md`** (private per-project notes) — Personal WIP context,
  local URLs, sandbox credentials, current focus areas that shouldn't be
  committed
- **`@import` references** — When a CLAUDE.md would benefit from referencing
  another file rather than duplicating its content

**Decision framework:**
- Is it a permanent project convention? → CLAUDE.md or `.claude/rules/`
- Is it scoped to specific file types? → `.claude/rules/` with `paths:`
  frontmatter
- Is it a pattern or insight Claude discovered? → Auto memory
- Is it personal/ephemeral context? → `CLAUDE.local.md`
- Is it duplicating content from another file? → Use `@import` instead

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

**Action types:**
- **CLAUDE.md** — Edit the relevant project or global CLAUDE.md
- **Rules** — Create or update a `.claude/rules/` file
- **Auto memory** — Save an insight for future sessions
- **Skill / Hook** — Document a new skill or hook spec for implementation
- **CLAUDE.local.md** — Create or update per-project local memory

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
- For each item: suggest a documentation home (ROADMAP.md sub-section,
  CLAUDE.md note, or a new memory file) so it persists across sessions

All three sections can be brief if the session was a small bug fix or
maintenance task with no forward-looking implications. If none apply, say
"No forward-looking items" and proceed to the closing summary.

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