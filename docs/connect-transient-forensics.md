# Connect-time transient forensics (Phase 3 Task 6)

Root-causing the "1 CRC failure + 1 `FLAG_DROPPED` + some skipped bytes" transient
observed at stream start since Phase 2, tracked in `ROADMAP.md`'s Phase 2 deferred list.
Two recorded instances exist: `captures/e2e_p2.bin` (Phase 2 Task 7) and
`captures/e2e_p25.bin` (Phase 2.5 Task 5), both captured via
`roomscan.viewer --record` immediately after an SWD reset.

## Method

A standalone byte-exact decoder (same resync/CRC logic as
`host/src/roomscan/decoder.StreamDecoder`, reimplemented so it could log absolute file
offsets for every anomaly) was run over both captures in full. It reports, for every
CRC-failing/skipped byte run: the file offset where it starts and ends, the frame
immediately before and after it (type/stream/seq/flags), and a hexdump of the
boundary. Kept in the session scratchpad, not committed (one-off analysis, not a
reusable tool — see "What was not committed" below).

## Byte evidence

Both files decode to **exactly one** CRC failure and **exactly one** skip run,
byte-for-byte identical in position and length:

| | `e2e_p2.bin` | `e2e_p25.bin` |
|---|---|---|
| File size | 24,395,776 B | 27,598,848 B |
| Frames decoded | 1660 | 1878 |
| CRC failures | 1 | 1 |
| Bytes skipped (mid-run) | 12,064 | 12,064 |
| First frame (offset 0) | `CALIB` seq=1, plen=2332 | `CALIB` seq=1, plen=2332 |
| Bad header (offset 2368) | `DATA/RAW_3DMD` seq=1, plen=14842 (declared), CRC mismatch | same |
| Next good frame (offset 14432) | `DATA/RAW_3DMD` seq=2, **flags=0x01 (DROPPED)** | same |

Detail on the bad region (`e2e_p2.bin`; `e2e_p25.bin` is identical down to the offsets):

- The header at file offset 2368 is **perfectly well-formed**: magic, version,
  `frame_type=DATA`, `stream_id=RAW_3DMD`, `w=54`, `h=42`, `plen=14842`, `seq=1`,
  `t_us` one HAL tick (1 ms) after the preceding CALIB's `t_us` — i.e. this is
  genuinely the device's real seq=1 RAW frame header, not garbage that happens to
  spell "RSCN".
- A full frame at that offset would end at `2368 + 32 + 14842 + 4 = 17246`. The next
  real, CRC-valid frame instead starts at **14432** — 2814 bytes short of that. Only
  `14432 - 2368 - 32 = 12032` payload bytes ever arrived for this frame; no CRC tail
  was ever received for it.
- Those 12,032 bytes are **not filler**: all 256 byte values appear, and the content
  visually matches the same zone-intensity pattern seen in every neighboring good
  RAW_3DMD payload (checked by hexdump comparison). This is the **front of a real,
  in-progress payload send that stopped partway through**, not stale/garbage FIFO
  content and not a decoder false-positive.
- The very next successfully-decoded frame (seq=2) carries `flags=0x01`
  (`RS_FLAG_DROPPED`) — exactly the flag firmware sets on the frame after one it
  failed to fully transmit.
- No other CRC failure or skip occurs anywhere else in either file (1660/1878 frames
  decode cleanly for the rest of each run) — this is a one-time, non-recurring event,
  confirming the "first-occurrence transient" characterization from prior reports.

### Answers to the forensics questions

- **(a) Start or mid-stream?** Capture **start** — the very first RAW frame (seq=1),
  immediately after the very first CALIB (seq=1). Both files.
- **(b) Tail of a truncated RAW frame?** Not the tail — the **front** of it. The header
  and the first ~12 KB of payload arrived intact; the last ~2.8 KB of payload + the
  4-byte CRC never arrived. The frame was cut off mid-*send*, not mid-*capture*.
- **(c) `FLAG_DROPPED` on the next frame?** Yes, in both captures, on seq=2.
- **(d) Same pattern in both captures?** Yes — identical offset (2368), identical run
  length (12,064 B), identical frame-2 flag. Not just "the same class of event": the
  same byte-for-byte shape twice, from two independently captured sessions weeks apart
  in this project's timeline.

## Root cause

The firmware's raw-only send helper `rs_cdc_send()`
(`firmware/scanner-stream/Src/vl53l9_app.c:67-84`) already has a documented abort
policy: it pumps `tud_task()` while writing, and if the host hasn't drained enough of
the CDC IN endpoint to make room within **100 ms**, it gives up and returns `false`
*mid-payload* — whatever bytes it had already handed to TinyUSB stay sent, the rest of
the frame (including its CRC) is simply never written. `rs_send_frame_cdc()`
(`vl53l9_app.c:106-118`) treats that as "this frame was dropped" and sets
`RS_FLAG_DROPPED` on the *next* successful send. This exact mechanism is what the
Phase 2 Task 7 / Phase 2.5 Task 5 stall/recover experiments deliberately triggered by
pausing the host's reader for 5 s — and it reproduced there with the identical
signature (one CRC failure from the mid-frame abort, one `FLAG_DROPPED` on recovery).

The connect-time transient is **the same mechanism, firing once, for free** — not from
an artificial pause, but from ordinary host-side startup latency:

1. Firmware's DTR gate (`vl53l9_app.c:1100`, `while (!tud_cdc_connected()) tud_task();`)
   releases the instant the host asserts DTR (which pyserial's `Serial.__init__` does as
   part of opening the port — `host/src/roomscan/sources.py:33`, `SerialSource.__init__`
   calls `serial.Serial(port, baud, timeout=...)`).
2. Firmware then does a fixed `HAL_Delay(50)` "let the host's reader thread settle"
   grace period (`vl53l9_app.c:1103`), triggers frame 1's ranging, and as soon as it's
   ready sends CALIB then RAW frame 1 at full speed — all before the *caller* of
   `SerialSource(...)` (e.g. `roomscan.viewer`, which does further setup — decoder
   construction, `TransformStage`/DLL load, window/thread startup — before its read loop
   in `sources.pump()` ever calls `.read()`) has necessarily started pulling bytes off
   the wire.
3. If that host-side gap between "DTR asserted" and "first `.read()` call actually
   draining the endpoint" exceeds the ~100 ms `rs_cdc_send()` budget, frame 1's send
   aborts partway — exactly the byte-for-byte identical result seen in both captures,
   because both captures used the same `roomscan.viewer --record` code path with
   essentially the same fixed startup-latency profile on this machine.

This is **not** any of the brief's leading hypotheses:
- **Not stale TX FIFO residue** — the truncated bytes are live, freshly-varying sensor
  data matching the surrounding good frames' content, not stale/repeated bytes.
- **Not the "attach to an already-streaming board" bug** — that scenario (separately
  tracked, ledger-observed live) would show a **large** CALIB `seq` (reflecting a
  frame counter that has been running since boot) and `raw-skip` climbing toward the
  64-frame cadence ceiling. Both captures instead open with `CALIB seq=1` and an
  immediately-following `RAW seq=1` at an early boot timestamp (`t_us` ≈ 11.7 s /
  16.2 s since boot) — proof these two captures really are from fresh boots, exactly
  as their methodology claimed. `raw-skip` is correspondingly absent for the rest of
  both runs.
- **Not a DTR *signal* race** in the sense of ambiguous/bouncing DTR — it is a
  straightforward throughput/timing race between "firmware starts sending" and "host
  starts reading," using the same 100 ms budget the stall/recover tests already
  exercise deliberately.

## DTR-gate one-shot question

Confirmed by reading `vl53l9_app.c`: the `while (!tud_cdc_connected())` gate
(`vl53l9_app.c:1100`) sits **once**, before the raw-only `while (1)` acquisition loop
(`vl53l9_app.c:1326`) begins, and is never re-entered. After the first host connection
of a boot, a host disconnect/reconnect does **not** re-block acquisition — the loop
keeps ranging and calling `rs_send_frame_cdc()`, which just marks `pending_dropped` and
returns immediately while `tud_cdc_connected()` is false (`vl53l9_app.c:110-113`). So a
reconnect **can** land mid-stream, with no CALIB-first guarantee — this is real and is
the mechanism behind the ledger's separately-reported "attach to an already-streaming
board" artifact. It is architecturally distinct from what's in `e2e_p2.bin`/
`e2e_p25.bin`.

**Old-capture methodology check** (`p2-task-7-report.md`, `p25-task-5-report.md`): both
reports describe resetting via `STM32_Programmer_CLI -c port=SWD -rst` and starting the
capture "immediately after reset," without a documented explicit wait for the old COM
port to vanish before opening the new one. That imprecision doesn't matter here: the
byte evidence (CALIB `seq=1`, early `t_us`) independently proves both captures are
genuinely fresh-boot streams regardless of the exact human/script timing around the
reset — the frame counter can't be spoofed by the host-side protocol. **Conclusion:**
the "stale reconnect" alternative explanation does not apply to either analyzed
capture; the transient is inherent to a genuinely-fresh connect, not a methodology gap.

## Verdict and disposition

**CHARACTERIZED-COSMETIC.** The connect-time transient is the pre-existing,
already-validated `rs_cdc_send()` 100 ms-stall / `FLAG_DROPPED` self-heal mechanism,
triggered once per connection by ordinary host-side startup latency between DTR-assert
and the first live read. It:

- costs exactly one RAW frame (never more, in either recorded instance),
- self-heals with no seq gap (the sensor's frame counter is untouched — only the CDC
  write was aborted) and no recurrence for the rest of the session,
- is byte-for-byte reproducible, so it is not host-scheduling noise; it is a
  deterministic property of this firmware/host pair's fixed startup timings.

No wire-protocol or decoder change is needed — `docs/protocol.md`'s existing decoder
requirements (resync on CRC failure, DROPPED-flag semantics) already describe exactly
this behavior as correct, designed operation.

### Why no hardware round was run

The task brief allows "one hardware round at most," conditioned on the offline evidence
leaving a live question. It doesn't, here: two independent, real hardware captures
already agree byte-for-byte on offset, run length, and the following DROPPED flag, and
the firmware code path that explains them (`rs_cdc_send`'s 100 ms abort) is the same
one already hardware-validated by two prior stall/recover experiments (Phase 2 Task 7,
Phase 2.5 Task 5). A fresh round of SWD-reset captures would, per this analysis, be
expected to reproduce the *same* one-frame artifact again (not zero, as the brief's
"attach-to-already-streaming" hypothesis would have predicted for a truly fresh boot) —
which would confirm, not further discriminate, what's already shown here. Spending a
hardware round to reconfirm an already-dispositive, deterministic byte match did not
seem justified; this is flagged explicitly rather than silently skipped, per the
brief's evidence-before-assertions expectation.

### The separate mid-stream-reattach item (CALIB-on-DTR-connect)

The ledger's live-observed "attach to an already-streaming board" case (large CALIB
`seq`, `raw-skip` climbing to the ≤63-frame ceiling) is real but architecturally
distinct from the transient analyzed above (see "DTR-gate one-shot question"). It
remains tracked in `ROADMAP.md` as the "CALIB-on-DTR-connect" open item. Two things
changed its status this task:

- **Partially mitigated already:** Phase 3 Task 2 shipped `SEND_CALIB`
  (`RS_CMD_SEND_CALIB`) — a host can now request an immediate CALIB frame on connect
  (`roomscan-ctl calib`) instead of waiting up to 63 RAW frames. This is a manual,
  not automatic, fix.
- **Automatic fix evaluated, not implemented.** The brief's proposed cheap fix — use
  `tud_cdc_line_state_cb` (DTR rising) to abort any in-flight frame and restart at a
  frame boundary with an immediate CALIB — was evaluated and is **not** small/safe
  enough to land in this task: TinyUSB invokes that callback from the USB stack's own
  context, concurrently with the main acquisition loop's send/trigger state machine
  (`raw_mem_index`, the static `rs_calib_countdown`, and any in-progress
  `rs_cdc_send()` byte loop). Safely tearing down and restarting that state from a
  callback without a new synchronization primitive is a real firmware design task, not
  a one-line change — **specced as a Phase 3/4 follow-up**, not implemented here.

## What shipped this task

- `docs/protocol.md`: one-clause fix to the CALIB registry row (deferred from Phase 3
  Task 5) — recovery/REINIT-triggered CALIB retransmits carry the *last-captured* seq
  (EVENT-frame convention), distinct from the periodic/stream-start retransmit's
  *next-frame* seq. Version-history entry `(f)` added.
- `ROADMAP.md`: the Phase 2 "Open — connect-time CRC/DROPPED transient" item marked
  resolved with a cross-reference to this document; the "Open — CALIB-on-DTR-connect"
  item updated with the `SEND_CALIB` mitigation and the follow-up spec for the
  DTR-callback auto-fix.
- This document.
- No firmware or host code changes (root cause is in already-shipped, already-correct
  behavior; no fix needed).

## What was not committed

- The standalone forensics script used to produce the offsets/hexdumps above (session
  scratchpad only) — one-off analysis over two fixed capture files, not a reusable
  tool; the same resync/CRC logic already lives in `host/src/roomscan/decoder.py` for
  production use.
