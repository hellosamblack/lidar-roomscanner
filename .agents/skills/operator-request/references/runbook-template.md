# Runbook template — the Operator Request comment

Copy this shape exactly. The section order is fixed and `host/tests/test_operator_skill.py` asserts
that the headings here match the list `SKILL.md` states, so the two cannot drift apart.

Write it to a file and post with `gh issue comment NNN --body-file <file>`. **Never** inline
`--body "..."` — a runbook is full of backticks and they are command-substituted by the shell
before `gh` ever sees them.

---

## The template

```markdown
## 🔧 Operator Request — <plain-language title, no jargon>

**Why this matters**

<One or two sentences. What we do not know, and what this will tell us. No acronyms, no tool
names, no issue-speak. If a smart person who has never seen this project could not follow it,
rewrite it.>

**What you need, and how long**

- <physical item>
- <physical item>
- About **<N> minutes**.

**Steps**

<Numbered, alternating [Claude]/[You], one action each, composed from the step library.>

1. [You] ...
2. [Claude] ... **Wait for me to say ready.**
3. [You] ...

**While it is running, note down**            ← include ONLY if there is a live-only observable

- <thing>: ______
- <thing>: ______

These are not saved in the file. If you do not write them down, the take has to be redone.

**When you are done, say**

> **done with #<NNN>**

**What I will do with it**

<The check, in plain language, and what counts as a pass. State the number and the threshold so
the operator knows what success looks like — e.g. "I will check the recording covered enough
angles; we need at least 60 of the 92 patches filled in.">

**If something looks wrong**

- **<symptom in plain words>** → <what to do>
- **<symptom in plain words>** → <what to do>
- **Anything else** → stop, tell me what you see, and do not redo the take yet. A confusing take
  is more useful to me intact than repeated.

<!-- operator-request: issue=<NNN> kind=<capture|network|hardware|eyes|decision> artifact=<capture|observation|photo|answer> gate=<tool.field or "operator-report"> -->
```

---

## The footer

The HTML comment on the last line is invisible on GitHub and is how the fulfilment half finds
its way back without guessing. `operator_queue()` parses it. All five keys are required.

| Key | Values |
|---|---|
| `issue` | the issue number, digits only |
| `kind` | matches the `needs/<kind>` subtype label applied |
| `artifact` | `capture` (a `.bin`), `observation` (numbers the operator wrote down), `photo`, `answer` |
| `gate` | the check to run — `tool.field` (e.g. `capture_analyze.continuity.complete`) or the literal `operator-report` when the operator's own notes are the result |

---

## Rules for the prose

- **No acronyms without expansion.** Not "ToF", "IMU", "SFLP", "mag", "fps", "CRC". Say "the depth
  sensor", "the motion sensor", "the compass".
- **No tool names in `[You]` steps.** `rig_idle`, `capture_analyze` and friends belong in
  `[Claude]` steps and in "What I will do with it" — never in an instruction the human executes.
- **Name what they will see, not what it is called internally.** "the button turns red and reads
  ■ Stop", not "the record toggle asserts".
- **Every `[You]` step is one action.** "Walk the loop and return to the start" is two.
- **Give the pass threshold a number.** "Good coverage" is not followable; "at least 60 of the 92
  patches" is.
- **Say what to do when it goes wrong**, including the case where nothing appears to happen — that
  is often the actual finding.
- **Never name a control from memory.** Grep `host/src/roomscan/static/index.html` for it, and check
  whether anything the runbook opens covers it — a modal backdrop makes the sidebars unclickable and
  silently reorders the whole procedure.
