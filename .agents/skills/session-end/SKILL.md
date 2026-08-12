---
name: session-end
description: The end-of-session bookend to session-start — runs the checklist for
  memory, self-improvement, shipping, and handoff. Trigger when the user says "wrap up",
  "close session", "end session", "wrap things up", "close out this task";
  AND automatically, in the same turn, as soon as the session's work is done and
  **before** you write the commit that closes the governing issue (`Closes #NNN`).
---

# Session End

The close-side bookend to `session-start`. Run the four phases in order. All phases
auto-apply without asking; present a consolidated report at the end.

The order is load-bearing: **Remember → Improve → Ship → Hand off.** The first two phases
write files; the third is the only one that stages and commits.

## When this runs automatically

**Run this skill when the session's work is finished and verified, before you write the
closing commit.** The closing commit — the one whose message contains `Closes #NNN`, the
session's final commit per the `session-start` convention — is landed *by* this skill, in
Phase 3. Do not commit it first and then wrap up: Phases 1–2 produce real repository edits
(skills, hooks, MCP tools, `AGENTS.md`, `docs/`), and those must be authored **before**
anything is staged, so they go through the same `status-sync` checklist as the session's own
work and so `Closes #NNN` stays the last commit of the session.

*(Ordering fixed 2026-08-11, issue #169. Previously this skill fired only after the closing
commit already existed and put Review & Apply after Ship It, which made it structurally
impossible for a session's self-improvements to ride with — or before — its closing commit;
they always landed as a trailing commit that `status-sync` never saw.)*

**If you did commit first anyway** — a `Stop`-hook backstop, `.claude/hooks/session-end-guard.sh`,
blocks the turn and reminds you when HEAD carries `Closes #NNN` and this skill has not run for it.
Run the skill then, in the same turn: work through the phases as written, but Phase 3 lands the
Phase 1–2 output as a **follow-up commit** (`Refs #NNN`) instead of bundling it. Say so in the
report — a trailing improvement commit is the degraded path, not the intended one.

`status-sync` remains its own skill for the per-commit doc-sync you run on *every* landing,
including mid-session ones that only `Refs #NNN`; this skill calls it (Phase 3) and adds the
once-per-session Remember / Review / Handoff phases around it.

## Phase 1: Remember It

Review what was learned during the session. Decide where each piece of
knowledge belongs in the memory hierarchy. **Write the files now — do not defer them to
after the commit**; in-repo destinations (`AGENTS.md`, `docs/`, skills) must exist before
Phase 3 stages anything.

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

**Then log the memories on the governing issue (owner, 2026-08-11).** Auto memory lives
*outside* the repo, in a per-machine directory no human, no Codex session, and no agent
running elsewhere can read — so a fact recorded only there is invisible to everyone but a
future Claude session on this box that happens to recall it. The issue thread is the shared
record. Post one comment per issue this session claimed:

```bash
gh issue comment NNN --repo hellosamblack/lidar-roomscanner --body-file /tmp/mem.md
```

Write `/tmp/mem.md` with `Write`, never inline `--body "..."` — backticks inside a
double-quoted shell argument are command-substituted before `gh` sees them
(`backticks-in-quoted-commit-message` memory). Structure it as:

```markdown
**Memories recorded** — <UTC timestamp>

- `<memory-slug>` (auto memory, new|updated) — <the fact itself, one or two sentences.
  Reproduce the substance, not just the title: this comment is the only copy anyone
  outside this machine can read.>
- `AGENTS.md` → <section> — <what rule was added/changed>
- `.agents/skills/<name>/SKILL.md` — <what procedure changed and why>
```

Skip the comment only if nothing durable was learned — and say "nothing durable learned"
in the consolidated report rather than staying silent, so the absence is a decision on the
record and not an omission.

## Phase 2: Review & Apply

Analyze the conversation for self-improvement findings. If the session was
short or routine with nothing notable, say "Nothing to improve" and proceed
to Phase 3.

**Auto-apply all actionable findings immediately** — do not ask for approval
on each one. Apply the changes now, in the working tree; Phase 3 commits them
alongside the session's work. Do **not** commit here.

**Finding categories:**
- **Skill gap** — Things Claude struggled with, got wrong, or needed multiple
  attempts
- **Friction** — Repeated manual steps, things user had to ask for explicitly
  that should have been automatic
- **Knowledge** — Facts about projects, preferences, or setup that Claude
  didn't know but should have
- **Automation** — Repetitive patterns that could become skills, hooks, or
  scripts

**Action types** (same destinations as Phase 1):
- **`AGENTS.md`** — Edit the canonical project guidance (repo root)
- **`docs/engineering-practices.md`** — Add a binding convention that needs more than a paragraph
- **Skill** — Create or update a skill under `.agents/skills/`; add a hook under `.claude/hooks/`
  (and register it in `.claude/settings.json`) when the behavior must fire without being remembered
- **Auto memory** — Save an insight for future sessions, with its `MEMORY.md` index line
- **Tool** — Land the capability as an MCP tool under `host/src/roomscan/mcp_server/`
  (the `AGENTS.md` "New tools land in the MCP server" rule), not only as a `host/tools/` script

**Execute what you changed before moving on.** A skill mechanism can be silently dead — a
nonexistent label, a path that breaks in a worktree — and reading it back proves nothing
(`skill-mechanisms-need-execution-tests` memory). Run the commands you added or edited; for an
MCP tool, call it. For a hook, exercise its trigger condition. Carry the result into Phase 3:
a change that has not been executed is not ready to commit.

Because these edits are now in scope for the session's commit, they are also in scope for
`status-sync` — Phase 3 runs that checklist over them, not around them.

## Phase 3: Ship It

**Commit & land (commit-to-`main`, no PR — owner workflow 2026-07-16):**
0. **Decide whether the issue may close at all.** Run the `operator-request` close-or-hold table
   (`status-sync` gates on it too). If the acceptance needs a capture, a link, a hardware change or
   a human's eyes that you did not exercise this session, the commit says **`Refs #NNN`, not
   `Closes #NNN`** — post the operator runbook, apply `needs/operator` + a subtype, and leave the
   issue open. Everything below still runs; only the closing keyword and the `gh issue close`
   change. Say plainly in the summary that the work is landed but unverified.
1. Run `git status` and `git branch` to see current state
2. Run the `status-sync` skill checklist — the commit must include the doc
   deltas the work implies (ROADMAP.md status, superseded annotations, memory).
   This now covers the Phase 1–2 output too: a new skill, hook, or MCP tool authored
   above gets the same doc-sync treatment as the session's feature work (an MCP tool
   means `host/tests/test_mcp_registry.py` and `docs/mcp-server.md` in the same commit).
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
   - The Phase 1–2 edits **are** in scope — you wrote them minutes ago. They are the
     one category of change you never have to attribute.
4. **Order the commits so `Closes #NNN` is last.** Two shapes are correct:
   - *Bundled* — the improvement is inseparable from the work (an MCP tool the task
     added, a doc rule the fix establishes): one commit, `Closes #NNN`.
   - *Split* — the improvement is generic meta-work unrelated to the feature: land it
     first as its own `Refs #NNN` commit, then the session's closing commit. Never the
     reverse; a commit after `Closes #NNN` means the session did not actually close.
5. Commit the in-scope files with a descriptive message.
6. Land the commit on local `main` and **close any feature/worktree branch
   without a PR**: either commit directly on `main`, or fast-forward/merge the
   branch into `main` and `git branch -d` it. No `gh pr create`, no
   `gh pr merge`. Do NOT push — pushing to `origin` is a separate step the
   owner asks for explicitly (they may want to review the local commit first).
7. **If in-scope status-sync doc deltas (ROADMAP.md/CLAUDE.md) sit next to
   out-of-scope code in the same file, edit and stage only the in-scope prose**
   — don't document someone else's still-moving feature on their behalf; note
   in the closing report that it's undocumented and out of scope.

**File placement check:**
8. If any files were created or saved during this session:
   - Verify they follow the project naming convention
   - Auto-fix naming violations (rename the file)
   - Verify they're in the correct subfolder per project structure
   - Auto-move misplaced files to their correct location
9. If any document-type files (.md, .docx, .pdf, .xlsx, .pptx) were created
   at the workspace root or in code directories, move them to the docs folder
   if they belong there

**Deploy:**
10. Check if the project has a deploy skill or script
11. If one exists, run it
12. If not, skip deployment entirely — do not ask about manual deployment

**Restore what you stopped (added 2026-07-29 after leaving the owner's viewer dead a whole session):**
13a. Did this session stop, replace, or repoint an **owner-facing service**? On this project that is
    almost always `roomscan-web` on port 8000 — the address the owner has open in a browser — but the
    same applies to anything you `pkill`ed, reflashed, or pointed at a replay instead of the device.
13b. Restore it to its pre-session state and **verify** (`pgrep -af roomscan.web`, then
    `curl -s -o /dev/null -w '%{http_code}' http://localhost:8000/static/index.html` → 200, and check
    the log's `[source]` line says the device, not a `--replay`). Say in the report that you stopped
    it and that it is back — the owner cannot see your `pkill`. If a process you didn't start now
    holds the port (wrong cwd, args you didn't pass), that's likely the owner driving their own test —
    say what you see instead of killing or "restoring" over it.
13c. Whoever needs it: `ROOMSCAN_NO_BROWSER=1 setsid host/.venv/bin/python -m roomscan.web
    > /tmp/web-live.log 2>&1 < /dev/null &` with `dangerouslyDisableSandbox` (the sandbox kills
    listeners — see the `agent-sandbox-port-binding` memory).

**Release the issue claim:**
14. Drop the `status/in-progress` label `session-start` added, on every issue this session claimed —
    whether or not the work finished. A stale claim makes the next session either duplicate the
    check or steer around work nobody is doing:

    ```bash
    gh issue edit NNN --repo hellosamblack/lidar-roomscanner --remove-label "status/in-progress"
    ```

    If the session is ending with the issue still open and unfinished, say so in the outcome comment
    (Phase 4's handoff) — the comment is the handoff, the label is not.

**Task cleanup:**
15. Check the task list for in-progress or stale items
16. Mark completed tasks as done, flag orphaned ones

**Report.** Present the consolidated summary in two sections — applied items first, then
no-action items. **If step 3 found out-of-scope changes left uncommitted on purpose (someone
else's WIP, a concurrently-edited feature), say so explicitly** — name the files/feature and
that they're untouched and undocumented, so the owner knows it's still theirs to land.

## Phase 4: Next Steps & Handoff

Provide the owner with a clear forward-looking summary covering three areas:

**Next Development Steps:**
- List the logical next tasks/features that follow from this session's work
- Reference ROADMAP.md phase/milestone context where applicable
- Estimate scope (small/medium/large) and any blockers or dependencies
- Flag any architectural decisions that unlock or constrain future work

**Recordings & Verifications Needed from Owner:**

**Each item here is an action, not a note.** Run the `operator-request` skill for every one: post
the runbook as a comment on its issue and apply `needs/operator` + a subtype. A verification listed
only as prose here is a verification that never happens — that was this section's failure mode, and
it is why issues closed while their real acceptance sat unrun.

- Capture sessions or hardware tests that depend on physical actions (moving
  devices, changing board settings, rescanning a space, etc.)
- Validation runs needed to gate the next milestone or confirm a fix works
  in production (e.g., "full-room sweep with the current firmware to verify
  tracking doesn't lose frames in this layout")
- UI/UX validation steps that require human eyes or a real handheld use case
- Any data the session assumed but didn't verify (e.g., "assumes the mag cal
  is valid — run `capture_magcheck` on the latest capture")

If several are pending, use `operator-request`'s batch mode — `operator_queue()`, then one combined
runbook — so the owner powers the rig up once rather than once per issue.

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

Run this **after** Phase 3 has landed the closing commit, so the sha you record is the one the
hook will compare against. The git dir is per-clone and untracked, so this is local state, never
committed. The hook (`.claude/hooks/session-end-guard.sh`) fires only when HEAD carries
`Closes #NNN` **and** this marker does not already equal HEAD — so writing it here is what
closes the loop.

---

Worked example of Phases 2–4 output:

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
