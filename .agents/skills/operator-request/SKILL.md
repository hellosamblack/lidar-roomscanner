---
name: operator-request
description: Use when an issue cannot be finished or verified without the owner physically doing something — recording a capture with the rig, re-cabling or fixing the network bridge, changing a jumper, or looking at the viewport and judging. Also use before closing ANY issue, to decide whether the work is genuinely verified or must stay open pending an operator action. Triggers on "I can't verify this without hardware", "needs a capture", "ask the owner to record", "what do you need from me", "is this ready to close".
---

# Operator Request — work only a human can finish

Two jobs, and the second one is the reason the first exists:

1. **Ask well.** Turn "a human has to do something" into a runbook a non-technical person can
   follow exactly, posted as a comment on the issue.
2. **Judge honestly.** Decide whether an issue is actually verified or only *believed* to work. If
   it is only believed, it does not close.

## Part 1 — The close-or-hold judgement

**Run this before every `gh issue close`, and before writing any commit message containing
`Closes #NNN`.** `status-sync` and `session-end` both route here.

One question:

> **What would prove this is fixed — and did I actually run it, today, against real data?**

| Close outright | Hold open with `needs/*` |
|---|---|
| A test covers the changed behaviour and it passes | The claim needs sensor data that does not exist yet |
| An MCP tool run against a **real existing** capture reproduces the claimed number | The path cannot be exercised on this host (USB CDC is dead here — #16, #145) |
| `ui_screenshot` / `ui_eval` confirms a visual claim | The observable is not stored in the capture file (#60) |
| Docs, refactor, or tooling that makes no behavioural claim | It needs a human aesthetic or usability judgement (#106–#109) |
| | It needs a physical configuration that is not currently in place |

The asymmetry is deliberate. Closing an issue that is not fixed costs a future session a full
rediscovery; leaving one open costs a label.

### Rationalizations — all of them mean hold

| Excuse | Reality |
|---|---|
| "The code is obviously right" | Obviousness is not a measurement. Four separate shipped defects in this repo were obvious code. |
| "Tests pass" | That verifies the test. If the claim is about hardware, the test cannot see it. |
| "It worked before the refactor" | Then the claim is that nothing changed — which is itself unmeasured. |
| "The owner can verify later" | That is exactly what this skill is for. Post the request; don't close and hope. |
| "It's a small fix" | #16 is a small fix. It has been `status/fix-unverified` since 2026-07-30. |
| "Re-opening is cheap" | Nobody re-opens. A closed issue is invisible. |
| "I said in the closing comment that it isn't verified" | **The most common one, and the hardest to see.** An accurate note is not a hold. The #174 audit reopened #57, #168 and #171 — every one closed with a candid paragraph naming the exact unrun check, #171's even spelling out the command. Prose does not appear in `operator_queue()`, does not carry a label, and does not stop `Closes #NNN`. If you are writing that paragraph, you have already made the judgement: change the keyword and the labels, not just the wording. |

### What "hold" looks like

Apply **the umbrella plus one subtype**:

| Label | For |
|---|---|
| `needs/operator` | **Always** — the umbrella. This is what `operator_queue()` and the owner filter on |
| `needs/capture` | A recording made with the rig |
| `needs/network` | A link, bridge, transport or cabling action |
| `needs/hardware` | A jumper, solder bridge, mount, cable swap, board-stack change |
| `needs/eyes` | A human visual or usability judgement |
| `needs/decision` | An owner decision; no physical action |

Then pair with the **existing** status vocabulary — do not invent a new one:

- **`status/fix-unverified`** — the code landed; only verification remains. (This label existed
  unused for weeks. It is now driven from here.)
- **`status/blocked`** — nothing can proceed until the data arrives.

```bash
gh issue edit NNN --repo hellosamblack/lidar-roomscanner \
  --add-label "needs/operator" --add-label "needs/capture" --add-label "status/fix-unverified"
```

Then say so plainly in chat: what landed, what is unverified, and that the issue is waiting on the
owner. **Do not describe held work as "done".**

## Part 2 — Writing the request

Read `references/step-library.md` and `references/runbook-template.md` before writing. Do not
compose from memory — the wording in the library is load-bearing.

1. **Pick the subtype** from the table above. It sets `kind` in the footer.
2. **Decide what the artifact is** — a recording, numbers the operator writes down, a photo, or an
   answer. If the thing you need is not saved into the capture file (#60 is the canonical case),
   the artifact is `observation` and the runbook **must** include an L-block.
3. **Name the gate before you write the steps.** If you cannot state the threshold as a number,
   you do not yet know what you are asking for — work that out first, or you will get a take like
   #142: collected exactly to spec, and still unable to answer the question.
4. **Compose the steps** from the library blocks, in this order: setup → configure → record →
   motion → hand back. Renumber consecutively.
5. **Check every trap that applies:**
   - Still or tripod-mounted scene → **S5 static-scene guard is mandatory** (else the laser parks
     mid-take and the file still reports clean).
   - Magnetometer work → M4, hand-held, never on the tripod.
   - Anything where they might reach for `capture.py` → the do-not block.
6. **Write it to a file and post it:**

```bash
gh issue comment NNN --repo hellosamblack/lidar-roomscanner --body-file /tmp/operator-request.md
```

Use `--body-file` always. A runbook is full of backticks; inside a double-quoted `--body` the shell
substitutes them before `gh` sees them.

7. **Verify every UI element you named, in the DOM.** A human-facing instruction that names a
   button is a claim about the interface — check it the way you would check code:

   ```bash
   grep -noE 'Calibrate Mag|id="btn-record"' host/src/roomscan/static/index.html
   ```

   Reading the element's own `title=` tooltip is the cheapest way to describe it accurately. Then
   ask **where it is relative to everything else the runbook uses**: a `.modal-backdrop` is
   `position: fixed; inset: 0`, so while any modal is open the sidebars are unreachable and the
   step order has to change. Writing this from memory produced a runbook telling the owner to
   watch a coverage ball that only exists inside a modal covering the button they needed next.

   **Then grep the handler, not just the element.** Existing is not the same as working: opening a
   panel is rarely what arms it. `#magcal-modal`'s open button only calls `setOpen(true)`
   (`magcal.js:542`); sampling starts on `#magcal-start` (`:547`), and until it is pressed the
   coverage ball stays empty and **Stop & Fit** is `disabled`. The #144 runbook shipped without that
   click and would have wasted the owner's trip — caught by the #174 audit, not by writing it.

   ```bash
   grep -n "btnStart\|addEventListener" host/src/roomscan/static/magcal.js
   ```

   For every control the runbook names, ask: *does this render, or does this do the thing?*

8. **Re-read it once as the operator.** Every `[You]` step must be one action. No acronyms. If a
   step says "and then", split it. If you cannot picture doing it, neither can they.
9. **Regenerate `/static/operator.html`.** Not optional, not batch-mode-only — every runbook post
   changes what the page should show. `operator_page()` (or, if the MCP tool is unavailable,
   `host/.venv/bin/python host/tools/operator_page.py`). This step went missing for exactly one
   issue's worth of the session (#183, 2026-08-14) because it previously lived only as a
   prose aside inside "Batch mode" below — a single-runbook post never read that far and the page
   sat stale a full day, still telling the owner a *closed* issue needed a decision from them.

### Batch mode

When several issues are waiting, the setup is identical for all of them — do not make the owner
power the rig up three times. Call `operator_queue()`, group by `kind`, and post **one** combined
runbook: shared setup once, then each take as its own numbered section with its own name and its
own hand-back phrase. Cross-link it from each issue.

Order the takes so the fussiest configuration comes last, and put anything needing the tripod
adjacent — recomposing the rig between takes is the slow part.

**Batching the whole standing queue is a tool, not a judgement call.** Batch mode above is for
requests you are writing *together*; the queue also accumulates runbooks written weeks apart, each
with its own power-up preamble, and read literally it asks for one trip per issue. `operator_page()`
scrapes all of them and answers what the queue actually costs — it resolves the cross-reference
requests below into free riders, clusters near-duplicate setup steps so a shared power-up happens
once per sitting, and groups issues needing the same **venue** into one setup. That last grouping is
the one to look at when writing a *new* runbook: two runbooks can share a venue and no wording at
all (#142's recording is started by Claude, so it shares no step text with #144, yet both need the
owner stood in the same metal-free spot), and the page is where that shows up. It writes
`/static/operator.html` beside the app the runbooks already tell the owner to open — see step 9
above for when to regenerate it; it is a snapshot, not a live view.

### When another issue already asked for the same artifact

Batch mode is for requests you are writing together. The commoner case is that the artifact you
need **has already been requested on a different issue** — then you must not write a second
runbook for the same physical action. Two live runbooks for one recording is precisely the
ambiguity the red flag below forbids, and the duplicate is the one the owner will do twice.

Post a **cross-reference request** instead: a real `## 🔧 Operator Request` comment, titled
`Covered by the request on #MMM (<why it is the same take>)`, whose steps are "do the runbook on
#MMM; nothing here", carrying the hand-back phrase **of the other issue** and its own footer with
**this** issue's number and *its own* gate. #16 is the standing example ("Covered by the request on
#145 — same cable, same five recordings"); #159 → #161 was written this way on 2026-08-13.

This keeps three things true at once: the label stays a promise that instructions exist, the issue
appears in `operator_queue()` with a parseable footer instead of landing in `problems`, and the
owner is asked for the recording exactly once. Say plainly in the comment that there is nothing
extra for them to do.

## Part 3 — Processing the result

Triggered when the owner says the hand-back phrase, or anything like it.

1. **Find the request.** The newest `## 🔧 Operator Request` comment on the issue; parse its
   footer for `artifact` and `gate`.

   ```bash
   gh issue view NNN --repo hellosamblack/lidar-roomscanner --comments
   ```

2. **Find the artifact.** `capture_list()` is newest-first. Match the name the runbook asked for
   (`i<NNN>-<slug>`). If more than one plausibly matches, or the name does not appear, **ask which
   file** — never guess. Scoring the wrong take produces a confident wrong answer.

3. **Validate the artifact before scoring it.** This step is not optional and is the direct lesson
   of #141, #142 and #143 — all three are `status/partial` because a take was scored that could not
   answer the question.

   - `capture_analyze(path)` → check **`continuity.complete`**, not just `clean`. They are
     deliberately different properties: a byte-clean file can be missing 9% of its frames.
   - `capture_motion(path)` → confirm the operator's motion actually met the protocol — the holds,
     the bookends, the pan rate, the return to start.
   - Any depth-dependent claim → confirm depth frames are actually present. If S5 was needed and
     missed, the laser parked and you have motion data only.

   **If the take does not meet the protocol, stop.** Say exactly which clause failed and by how
   much, then post a revised runbook fixing the ambiguity that caused it. Do not score it anyway
   and do not report a number from it — a number from a take that could not answer the question is
   worse than no number, because it looks like an answer.

4. **Run the gate** named in the footer. For `gate=operator-report`, the operator's own notes are
   the result; quote them back verbatim.

5. **Post the result.** A comment headed `## ✅ Operator Result` or `## ❌ Operator Result`,
   stating: the artifact used, what was measured, the gate and whether it passed, and the verdict.

6. **Then, and only then:**
   - **Pass** → drop `needs/operator`, the subtype, and `status/fix-unverified`; close via
     `status-sync`.
   - **Fail** → the issue stays open. Record what was learned. If the take was good but the gate
     failed, that is a real finding — write it up. If the take was unusable, post a revised runbook.

```bash
gh issue edit NNN --repo hellosamblack/lidar-roomscanner \
  --remove-label "needs/operator" --remove-label "needs/capture" \
  --remove-label "status/fix-unverified"
```

7. **Regenerate `/static/operator.html`.** Same rule as posting (Part 2 step 9) — a resolved hold
   is exactly as stale as an unposted one until the page catches up. This applies even when an
   issue closes **without** going through this Part's numbered flow at all — e.g. a session re-runs
   the close-or-hold judgement itself and closes directly, as #183 did on 2026-08-14, dropping
   `needs/operator`/`needs/decision` via `status-sync` rather than an "Operator Result" hand-back.
   The page went stale a full day because nothing regenerated it either way. **The trigger is "a
   `needs/*` label changed on any issue," not "Part 3 ran."**

## Red flags

- You are about to write `Closes #NNN` for something you did not personally exercise.
- You are about to close an issue whose acceptance says "verify on the rig".
- A runbook step contains a tool name, an acronym, or the word "just".
- You named a button, card or readout **from memory** without grepping `index.html` for it — or you
  named two controls without checking whether one covers the other.
- You told the operator to **open** something without checking what **starts** it.
- You are writing "this was not verified on hardware" into a comment that also says `Closes #NNN`.
- You wrote "good coverage", "a reasonable amount", or "until it looks right" instead of a number.
- You are scoring a capture without having checked `continuity.complete` and `capture_motion`.
- An issue carries `needs/operator` but has no `## 🔧 Operator Request` comment — the label is a
  promise that the instructions exist. **One exception, and only one:** a `priority/later` hold may
  be labelled before its runbook is written, deliberately, so it stays findable while queued behind
  the now/next tiers. `operator_queue()` reports that as `pending[i]["parked"] == True` and keeps it
  out of `problems` (#175). Nothing else is excused — `priority/now`/`priority/next`, a malformed
  footer, and a comment-read failure are all still problems, because each means someone is waiting
  or a runbook exists and is broken.
- You are posting a second request to an issue that already has an unanswered one. Revise the
  existing one instead; two live runbooks on one issue is exactly the ambiguity this skill exists
  to remove.

## Related

- `references/step-library.md` — the canonical steps. Compose, do not reword.
- `references/runbook-template.md` — the comment format and its machine-readable footer.
- `status-sync` — routes here before any close; the doc-delta checklist for landing the work.
- `session-end` — routes here for each item it would otherwise have listed as owner-verification
  prose.
- `issue-fleet` — its veto rule is Part 1 of this skill applied at planning time.
- `tof-scan-diagnosis` — what to run when a returned capture reconstructs wrongly.
- `docs/mcp-server.md` — the `rig_*` and `capture_*` tools these runbooks drive.
- `host/tools/operator_page.py` — the owner-facing page behind `operator_page()`. Its `TAG_RULES`
  lexicon is the one hand-maintained part: edit it when a runbook introduces a genuinely new kind of
  constraint. Everything else (aliases, shared setup, venue grouping) is derived from the runbook
  text, so it cannot drift when a runbook is revised.
- `host/tools/operator_queue.py` — the queue behind `operator_queue()`; its `SUBTYPES` and
  heading constants are the source of truth the label table above must agree with.
- `host/tests/test_operator_skill.py` — the execution guards on all of the above, including the
  wiring check that keeps this skill from becoming another unused mechanism.
