# Step library — the canonical operator steps

Compose runbooks from these blocks. **Do not reword them.** Copy the block, fill the `<angle
brackets>`, delete what does not apply. The wording is deliberate: every sentence here was written
against the actual UI strings, the actual tool signatures, and a specific way a take has been
ruined before.

Two rules that make a runbook unambiguous:

1. **Every step is tagged `[Claude]` or `[You]`, and they alternate.** The operator must never
   wonder whose turn it is. A `[Claude]` step always ends by telling them to wait.
2. **A `[You]` step is one physical action or one click.** If a step contains "and then", split it.

Renumber the steps consecutively when you assemble them — the numbering below is local to each block.

---

## SETUP blocks

### S1 — Power up the rig

Use when the rig may be off, or at the start of any session after it has been idle.

```
1. [You] Press the power button on the **battery pack** (the RavPower box the scanner is strapped
   to). Its battery lights should come on.
2. [You] Look at the small square **network socket** on the scanner board where the flat cable
   plugs in. There should be a **steady green light** next to it, and usually a second blinking
   one. That means it is talking to the network.
   - **No green light?** The cable is loose, or the battery pack is not in the right mode — go to
     "Fix the bridge" below.
3. [You] Tell me "**powered up**" and wait — I will check whether it has appeared on the network.
```

### S2 — Fix the bridge (the ordering trap)

Use **only** when S1 shows no link, or when I tell you the scanner is not on the network. This is
the single most common first-connection-of-the-day failure.

> **The order matters and cannot be shortcut.** If the network cable is plugged in while the
> battery pack boots, it decides that socket is its *uplink* rather than its *output*, and the
> scanner will never get a connection. Running the repair script on its own does not fix that — the
> pack has to be restarted with the cable out.

```
1. [You] **Unplug** the flat network cable from the battery pack.
2. [You] **Power-cycle the battery pack** — hold its power button until it switches off, wait five
   seconds, switch it back on. Wait until its lights stop changing (about 30 seconds).
3. [You] In the app's top bar, click **Bridge Mode**. (I can also run this for you — say the word.)
4. [You] **Plug the network cable back in.**
5. [You] Tell me "**bridge redone**" and wait — I will check for the scanner on the network.
```

### S3 — Confirm the scanner is on the network

```
1. [Claude] I check that the scanner is reachable and the viewer is running (`rig_status()`, and
   `rig_up()` if it is not). **Wait for me to say ready.**
2. [You] Open <http://localhost:8000/static/index.html> in your browser. (From another machine,
   swap `localhost` for this host's address.)
3. [You] Look at the **top-right corner**. You should see a coloured dot and a set of small
   **status chips**. All chips green, and no "Offline" text, means everything is connected.
   - **Says "Offline"?** Look at the **log strip along the bottom** of the page and read me the
     last line. Do not guess — that line names the actual problem.
```

### S4 — Set the scanner up for this take

Use when the take needs a specific ranging profile or sample rate.

```
1. [Claude] I set the scanner to <profile / rate> and confirm the scanner itself reports it back
   (`rig_profile()` / `rig_imu_env_rate()`). **Wait for me to say ready.**
```

### S5 — Static-scene guard

**Required whenever the scanner will be still, or on a tripod, or pointed at an unchanging scene.**

> **Why this exists.** To spare the laser, the scanner parks it when nothing is moving, and wakes
> it again only on real movement. On a still scene it therefore parks *mid-recording* and the file
> ends up with motion data but no depth — and the automatic file check still calls that file
> "clean". This has silently ruined takes. I turn the power-saving off first.

```
1. [Claude] I disable the automatic laser idle (`rig_idle(auto_idle=False)`) so the laser stays on
   for the whole recording. **Wait for me to say ready.**
```

And at the very end of the runbook:

```
N. [Claude] I turn the automatic laser idle back on (`rig_idle(auto_idle=True)`). **Nothing more
   is needed from you.**
```

---

## RECORDING blocks

### R1 — Start recording

```
1. [You] In the top bar, make sure **Source** is set to **Live**. (The Record card is hidden
   otherwise.)
2. [You] In the right-hand panel, find the card titled **Record** and click the **● Record**
   button once.
3. [You] Check the button turned **red** and now reads **■ Stop**, with a counter underneath it
   like `Rec 0:12 · 4.2 MB` that is climbing. If that counter is not climbing, nothing is being
   recorded — stop and tell me.
```

> If you prefer, I can start and stop the recording for you (`rig_record`) so your hands stay on
> the scanner. Say so and I will drive it; the rest of the steps are unchanged.

### R2 — Stop recording and name the file

```
1. [You] Click **■ Stop**.
2. [You] A box pops up asking you to name the recording. Type exactly:

       <i173-exampleslug>

   then click **Save**. (The `.bin` on the end is added automatically.)
   - If it says the name is taken, add `-2` to the end and save again — then tell me the name you
     actually used.
   - The recording is already safely on disk either way. **Skip** is never a disaster.
```

The name is always `i<issue-number>-<short-slug>`, no spaces or capitals — e.g. `i60-cableflex`.
That is what ties the file to the issue; without it I have to guess which file you meant.

### R3 — Hand back

Always the last `[You]` step.

```
N. [You] Tell me exactly: **done with #<NNN>**
```

---

## MOTION blocks

Pick exactly one, unless the take deliberately has several segments. Each states the pace and
duration because the check I run afterwards (`capture_motion`) scores **what you physically did**,
not just the data — a take with the right length and the wrong motion fails.

### M1 — Walk the loop and come back

```
1. [You] Stand at your starting spot and pick a landmark to remember it by — a specific floor
   tile, a table corner. You must be able to return to **the same spot facing the same way**.
2. [You] Walk your route slowly and steadily, about **one slow pace per second** — much slower
   than normal walking. Keep the scanner roughly level and pointed ahead.
3. [You] Walk the full loop and **return to your exact starting spot, facing the same direction**.
   Stand still there for **ten seconds** before stopping.
   - Returning to the start is the whole point: the check measures how far the map thinks you
     drifted, and that is only meaningful if you really came back.
```

### M2 — Slow pan

```
1. [You] Stand still. Hold the scanner level.
2. [You] Turn on the spot at a steady <slow / medium / brisk> pace — about <N> seconds for a full
   turn — **without stopping**, for the whole <N>-second recording.
3. [You] Keep turning until I tell you the recording has stopped. Pausing partway invalidates it.
```

### M3 — Tilt sweep at one fixed bearing

```
1. [You] Hold the scanner **in your hands, off any tripod or mount.**
2. [You] Pick one thing across the room to point at — a door, a window — and **keep pointing at it
   the whole time**. Do not rotate left or right at any point.
3. [You] Hold the scanner **level** for a slow count of fifteen.
4. [You] Tilt it up to about **45°** — halfway to straight up — and hold for a slow count of
   fifteen.
5. [You] Tilt it to point **straight up** and hold for a slow count of fifteen.
6. [You] Repeat steps 3–5 once more, so you have done two full cycles.
   - Holding still is what makes this work. Sweeping smoothly through the angles gives me nothing
     to measure.
```

### M4 — Free tumble

> **Ordering trap.** The live coverage ball is inside the **Calibrate Mag** window, and that window
> is a full-screen overlay — while it is open the Record button cannot be clicked. So recording
> must be **started first**, and the window **closed again before stopping**. Getting this backwards
> means the operator either cannot start the take or cannot end it. The instrument keeps streaming
> and recording the whole time the window is open.
>
> **Arming trap (added #174).** Opening that window does not begin collecting — `magcal.js:542`
> only calls `setOpen(true)`; the sweep starts on `#magcal-start` (`:547`). Step 3 below is not
> optional padding: without it the ball never fills, the operator has no gap guidance to steer by,
> and **Stop & Fit** never enables. The first version of the #144 runbook omitted it.

```
1. [You] Hold the scanner **in your hands, well away from the tripod, any metal furniture, and
   your laptop.** Metal nearby bends the reading and gets baked in permanently.
2. [You] In the **left-hand panel**, find the **Calibrate Mag** button and click it. A window opens
   showing a ball.
3. [You] In that window, click **Start**. Only now does the ball begin filling in as you cover
   angles — opening the window on its own collects nothing, and the **Stop & Fit** button stays
   greyed out until you have pressed **Start**.
4. [You] Slowly turn the scanner through **every orientation you can** — like slowly rolling a ball
   in your hands. Upside down, on each side, nose up, nose down.
5. [You] Keep going for <N> seconds. **Aim at the gaps the ball shows you** rather than repeating
   a motion you have already done — slow and varied beats fast and repetitive.
6. [You] Close the window with the **×** in its corner (or press **Esc**). You must close it before
   you can stop the recording.
```

### M5 — Stationary with bookends

```
1. [You] Set the scanner down somewhere stable, or hold it as still as you can.
2. [You] Keep it **completely still** for the first ten seconds.
3. [You] <the middle action — e.g. an M-block, or a LIVE block below>
4. [You] Keep it **completely still** for the last ten seconds.
   - Those still stretches at each end are the reference the whole take is measured against.
```

---

## LIVE-OBSERVATION blocks

Use when the thing being measured is **not saved into the file** — the answer only exists on screen
while it runs, so what you write down *is* the deliverable.

### L1 — Watch a number on screen

```
1. [You] Before starting, find **<the named readout>** in the left-hand panel and keep it visible.
2. [You] Note down its value **before** you begin: ______
3. [You] While recording, glance at it every 15 seconds or so. If it **jumps**, note down roughly
   when (e.g. "about 40 seconds in") and the new value.
4. [You] Note down its value at the end: ______
5. [You] Read me the numbers you wrote down when you hand back. **This is the actual result — it
   is not saved in the file, so if you do not write it down it is gone.**
```

### L2 — Physically disturb something mid-recording

```
1. [You] About <halfway> through the recording, <the disturbance — e.g. gently flex the network
   cable back and forth near each end plug for about ten seconds>.
2. [You] Note down roughly **when** you did it: ______
3. [You] Note down whether **anything on screen changed** at that moment: ______
   - Both answers matter, including "nothing changed" — that is a real result, not a failure.
```

---

## HARDWARE blocks

### H1 — Change a jumper or link on the boards

```
1. [You] **Switch the power off** at the battery pack and unplug the network cable.
2. [You] <the exact physical change — board name, silkscreen label, from-position to-position>
3. [You] Take a photo of the changed area and send it to me, so I can confirm it before we power
   back on.
4. [You] Wait — I will check the photo before you power up. Powering up with this wrong can damage
   parts.
```

---

## Do-not blocks

Fold these in as warnings only where they apply.

- **Do not run `capture.py` while the viewer is open.** They both grab the scanner's data stream
  and starve each other. Recording goes through the app (or `rig_record`), always.
- **Do not calibrate the magnetometer on the tripod, or with the laptop close by.** Metal that is
  not bolted to the scanner cannot be calibrated out; it corrupts the fit (BUG-034).
- **Do not change the ranging profile more than once every couple of seconds.** Rapid changes can
  lose the acknowledgement (BUG-073/#84).
- **Do not re-plug the scanner's network cable while a recording is running** unless the runbook
  explicitly asks for it.
