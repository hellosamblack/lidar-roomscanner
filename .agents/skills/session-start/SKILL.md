---
name: session-start
description: Run at the start of any session that will make code changes — finds the governing
  GitHub Issue(s), flags in-progress conflicts, surfaces cross-cutting concerns, posts a
  session-start comment for traceability, and emits the commit-prefix template. Trigger on any
  intent to write/edit/delete code, or when the user says "let's work on X" / "fix Y" /
  "implement Z" and no governing issue has been established yet.
---

# Session start — anchor code changes to a GitHub Issue

Every session that writes code must be anchored to a GitHub Issue before the first edit.
This is what makes work traceable across sessions (human and agent) and prevents two sessions
from unknowingly covering the same ground or contradicting each other.

**Do this before writing the first line of code. If you're already mid-session without having
done it, do it now before the next change.**

## Step 1 — Map the task to an area label

| Work involves | Label |
|---|---|
| STM32 firmware, general | `area/firmware` |
| Firmware build / CMake | `area/firmware-build` |
| The `firmware/scanner-stream/` fork | `area/firmware-scanner-stream` |
| Firmware Ethernet / lwIP | `area/firmware-eth` |
| The firmware↔host boundary (protocol, joint fixes) | `area/firmware-host` |
| SLAM pipeline (`roomscan.slam`) | `area/host-slam` |
| Web UI (`roomscan-web`, `*.js`, `*.html`) | `area/host-web` |
| USB / Ethernet transport (sources, protocol decode) | `area/host-transport` |
| IMU / sensor fusion / mag cal | `area/host-sensors` |
| Offline splat pipeline (`roomscan.splat`) | `area/host-splat` |
| Offline post-processing (COLMAP / 3DGS / ARCore) | `area/host-offline` |
| MCP server, `host/tools/` scripts | `area/host-tools` |
| Viewer (`roomscan` desktop viewer) | `area/host-viewer` |
| The deprecated desktop panel (`panel.py`) | `area/host-panel` |
| The `vl53l9-transform-c` pipeline | `area/transform-lib` |
| An operating-environment / procedure issue, not code | `area/environment` |

`gh label list` is the live vocabulary if this table drifts; `docs/engineering-practices.md` carries
the same list. If the task spans multiple areas, list all of them — they all get a `Refs #NNN` in
the commit.

Also set a **priority** label on any issue you create: `priority/now` (critical path or
owner-critical), `priority/next` (queued, or one unblock away), `priority/later` (parked/proposed).

## Step 2 — List open issues in the area

```bash
gh issue list --repo hellosamblack/lidar-roomscanner \
  --label "area/<area>" --state open \
  --json number,title,labels,updatedAt \
  --limit 20
```

Also sweep for open bugs that might overlap the same files:

```bash
gh issue list --repo hellosamblack/lidar-roomscanner \
  --label bug --state open \
  --json number,title,labels,updatedAt \
  --limit 20
```

## Step 3 — Identify the governing issue

Pick the best match:

- **Exact match** — an open issue whose title/scope is exactly this task. Use it.
- **Partial match** — an open issue that would be partially advanced. Use it; note the partial
  scope in your session-start comment so the issue's history stays honest.
- **No match** — create one (now unblocked in auto-mode):

```bash
gh issue create \
  --repo hellosamblack/lidar-roomscanner \
  --title "<verb>: <what>" \
  --label "work-item" --label "area/<area>" --label "priority/<now|next|later>" \
  --body "## What

<one paragraph — what this does and why>

## Acceptance

- [ ] <first verifiable outcome>
- [ ] Tests green"
```

Record the number — this is `#NNN` for the rest of the session.

**Title convention — no ID prefix.** A title is a plain `<verb>: <what>` (e.g. `Strip
legacy-ID prefixes from issue titles`). It carries **no** `BUG-NNN:`/`SLAM-N:`/`DC-<letter>:`-style
prefix — that legacy scheme was stripped (2026-08-11) because it collided with GitHub's own
`#NNN`. The issue **type** is denoted by a label — `bug`, `work-item`, or `data-collection` —
and the **area** by `area/*`; never re-encode either in the title. The only identifier is
GitHub's `#NNN`. (Historical legacy IDs still resolve via `docs/issue-migration-map.md` — the
human-readable table to cite, generated from `docs/issue-migration-map.json`, which is the
machine-readable source of truth the tooling and `host/tests/test_doc_links.py` read — and via the
`Legacy ID:` line preserved in each migrated body; regenerate title cleanup with
`host/tools/migrate_issues.py strip-prefixes`.)

## Step 4 — Check for in-progress conflicts

Other sessions (human and agent) share this checkout. Find anything actively being worked:

```bash
gh issue list --repo hellosamblack/lidar-roomscanner \
  --label "status/in-progress" --state open \
  --json number,title,labels,updatedAt
```

For any hit in the same area — and for any issue in the area whose most recent comment is a
**Session start** with no matching outcome comment — **stop and read it**:

```bash
gh issue view NNN --repo hellosamblack/lidar-roomscanner --comments
```

Check whether it covers overlapping files or logic. If it does, coordinate before
proceeding — don't write on top of active work.

Then claim your own governing issue, so the next session sees you the same way:

```bash
gh issue edit NNN --repo hellosamblack/lidar-roomscanner --add-label "status/in-progress"
```

`session-end` removes the label when it closes out the session. If a session dies without
cleaning up, the stale label is exactly the signal you want — read the thread, then clear it.

## Step 5 — Post a session-start comment

```bash
gh issue comment NNN --repo hellosamblack/lidar-roomscanner \
  --body "**Session start** — $(date -u '+%Y-%m-%dT%H:%MZ')

Task: <one sentence>
Files in scope: \`<file1>\`, \`<file2>\` (or 'TBD if exploratory')
Related open issues checked: #X (<title>), #Y (<title>) — or 'none'"
```

This is the traceability record. The next session (human or agent) can read the issue thread
and know exactly what was attempted, what was found in scope, and which related issues were
checked at the time.

## Step 6 — Emit the session context block

Output this at the top of your first substantive response:

```
── Session context ─────────────────────────────────────────
Governing issue : #NNN — <title>
Related open    : #X (area/...) — <why it might cross-cut>
                  [or 'none checked']
Commit prefix   : <type>(area): ... [Refs #NNN]
────────────────────────────────────────────────────────────
```

Use `Refs #NNN` in every commit that advances the issue without closing it. Use
`Closes #NNN` only in the final commit that fully resolves it.

## At session end

The `session-end` skill handles the close side: outcome comment on the issue and `gh issue close`
if the work is done (via the `status-sync` checklist it runs). You do not close the issue in this
skill. `session-end` fires automatically in the same turn once you land the `Closes #NNN` commit.

## Special cases

**Multiple issues touched by one session** — name all of them in the session-start comment
and in `Refs` lines of each commit. `session-end` will prompt you to update each at session end.

**Data-collection task** — these require the hardware physically present. Read the issue body
for capture protocol requirements before starting; they are often blocking (`status/blocked`).

**Exploratory / diagnostic session** — if the session is read-only investigation with no
planned code change, skip this skill. If the investigation reveals something that *does*
require a code change, run this skill before making it.

**Work that advances a closed issue** — reopen it (`gh issue reopen NNN`) rather than
creating a duplicate; add a comment explaining what changed.
