# Phase 3: Runtime Configuration + Device Robustness — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. Checkbox steps.

**Goal:** Bidirectional protocol: a host→device **control channel** (profile/exposure/frame-period at runtime, CALIB on request) and the device-robustness half — EVENT-frame emission, `handle_error` → bounded re-init recovery (kills the 1-in-5 boot hang as a user-visible failure), and the connect-time-transient investigation. Owner scope calls: **binning stays fixed at 2 (full 54×42)** — no resolution knob; robustness **is** in scope.

**Architecture:** COMMAND (3) / ACK (4) frame types — additive values on the existing 32-byte header, still protocol v1. Device gains a CDC RX path (TinyUSB) with a minimal fixed-size frame parser (reuses `rs_crc32`); commands apply at a safe point between frames; reconfig = stop ranging → re-profile → restart (no transform on the MCU anymore — Phase 2 made this cheap). Host gains `SerialSource.write`, a `CommandClient` (send + await ACK), a `roomscan-ctl` CLI, and minimal viewer key bindings. EVENTs flow device→host on every fault path instead of silent spins.

## Global Constraints

- All standing conventions: 53L9A1 read-only; commits `git -c commit.gpgsign=false` + session trailers; ARM toolchain/flash/measure tooling per prior task reports; board COM14 / CDC CAFE:4001; suite baseline 51 passing; equivalence test stays green throughout.
- Protocol stays **v1** (new frame_type/command VALUES only; header layout, CRC, endianness unchanged). Wire changes follow the `protocol-change` skill: spec + protocol.py + rs_protocol.h + tests in one commit each time.
- Binning fixed at 2. Usecase/exposure/frame-period changes must preserve full 54×42 (reject any usecase whose profile implies a different binning — check `g_ranging_profiles[]` and enforce device-side with an error ACK).
- Firmware knob default remains the raw-only production config; dual-stream golden mode stays untouched.
- Device must never block acquisition on RX (poll, don't wait) and never trust host input: bounded command payloads, CRC-checked, unknown commands → error ACK, malformed frames dropped + counted.

---

### Task 1: Protocol — COMMAND/ACK frame types + command registry

**Files:** `docs/protocol.md`, `host/src/roomscan/protocol.py`, `firmware/scanner-stream/Src/rs_protocol.h`; tests in `host/tests/test_protocol.py`.

- Spec: `frame_type` 3 = COMMAND (host→device), 4 = ACK (device→host). COMMAND payload: `u32 cmd`, `u32 param` (LE). ACK payload: `u32 cmd` (echo), `u32 result` (0 = OK; nonzero = error registry), `u32 applied` (cmd-specific: applied value / info). Header `seq` on COMMAND = host-chosen token; the ACK echoes the same token in its header `seq` (its own frame counter does NOT apply — document). `width`/`height`/`flags` = 0 on both. CRC/magic as ever.
- Command registry v1 (spec table + `CommandCode` IntEnum + `RS_CMD_*` defines): 1 PING (ack.applied = firmware protocol version), 2 SEND_CALIB (device transmits a CALIB frame immediately; closes the ≤63-frame blind start — cross-ref the ROADMAP open item), 3 SET_USECASE (param = usecase id; validated against binning-2-preserving profiles), 4 SET_FRAME_PERIOD_US, 5 SET_EXPOSURE_MS, 6 REINIT (full sensor re-init cycle). Result-code registry: 0 OK, 1 UNKNOWN_CMD, 2 BAD_PARAM, 3 REJECTED_BINNING, 4 SENSOR_ERROR (detail in applied), 5 BUSY.
- Python helpers: `pack_command(cmd, param, token) -> bytes` (full wire frame) and `parse_ack(payload) -> (cmd, result, applied)`; tests: golden bytes for one command frame (hand-checkable prefix like the Phase 1 golden test), ack roundtrip, decoder passthrough of frame_type 3/4 (already generic — pin it).
- Version history entry (additive, no bump). Firmware compiles (defines only). ONE commit.

### Task 2: Firmware — CDC RX + command parser + PING/SEND_CALIB **[HW]**

**Files:** `firmware/scanner-stream/Src/vl53l9_app.c` (+ `rs_protocol.h/.c` if a parse helper fits there — keep HAL-free).

- RX plumbing: TinyUSB CDC RX (`tud_cdc_available`/`tud_cdc_read` — check whether the vendored version needs `CFG_TUD_CDC_RX_BUFSIZE` raised; commands are ≤44 B, current 256 is fine). Accumulate into a small static ring/buffer; a minimal C parser finds magic, validates header (frame_type == COMMAND, payload_len == 8, bounded), CRC-checks, extracts (cmd, param, token). Malformed input: drop + count (expose count via PING's ack.applied? keep simple: a static counter reported in the task's bench notes).
- Poll point: once per acquisition-loop iteration (after the send, before parse — anywhere USB is already live); do NOT poll inside rs_wait_event_usb slices (keep the wait primitive single-purpose).
- Implement PING (ack immediately) and SEND_CALIB (send CALIB frame then ack) — the two no-reconfig commands. ACK via `rs_send_frame_cdc`-style path with frame_type ACK (generalize the sender's frame_type or add a sibling — smallest correct change).
- Bench verify [HW]: flash; use a throwaway python snippet (pyserial + `pack_command`) to send PING and SEND_CALIB while streaming; confirm ACKs decode with echoed tokens, CALIB arrives on demand, and the RAW stream never hiccups (no gaps during command handling). ONE commit.

### Task 3: Host — SerialSource.write + CommandClient + roomscan-ctl

**Files:** `host/src/roomscan/sources.py` (add `write(data: bytes)`), new `host/src/roomscan/control.py`, new console script `roomscan-ctl` (pyproject entry), tests `host/tests/test_control.py`.

- `CommandClient(source, decoder)`: `send(cmd, param=0, timeout=2.0) -> (result, applied)` — writes the frame, scans decoded frames for the ACK with the matching token (tokens increment from a random start), raises `TimeoutError` with counts on silence; tolerates interleaved DATA/EVENT frames (they keep flowing). Testable hardware-free with a loopback stub source (test feeds pre-built ACK bytes).
- `roomscan-ctl` CLI: `ping | calib | usecase N | period US | exposure MS | reinit` — connects to the CDC port (reuse find_port), prints result/applied, exit code from result. Viewer keeps working while ctl is NOT running (single port owner — document: stop the viewer or use its keys from Task 6).
- Tests: token matching, timeout, interleave tolerance, CLI arg parsing (invoke main with argv). [HW] smoke: `roomscan-ctl ping` and `calib` against the live board. ONE commit.

### Task 4: Firmware — runtime reconfig (SET_USECASE / FRAME_PERIOD / EXPOSURE / REINIT) **[HW]**

**Files:** `firmware/scanner-stream/Src/vl53l9_app.c`.

- Safe-point application: command sets a pending-config struct; the loop applies it at the top of the next iteration: stop ranging (find the correct BSP call — `vl53l9_stop` or standby; `vl53l9_utils_set_profile` requires standby per its header note — verify in the read-only driver) → apply profile fields (usecase swap from `g_ranging_profiles[]` with a LOCAL copy so overrides don't touch the shared table; enforce binning==2 else error-ACK REJECTED_BINNING without touching the sensor) → restart → re-trigger. ACK only after the sensor accepted (result carries SENSOR_ERROR + status on failure, device attempts to restore the previous profile — if restore also fails, fall into Task 5's recovery path).
- Frame-period floor: values below the sensor's capability get whatever the driver does (clamp or error) — discover on hardware, ACK the APPLIED value either way (that's what the `applied` field is for).
- [HW] verify: cycle all four usecases live while the viewer streams (expect a visible sub-second pause per switch, seq continuity or a documented gap+DROPPED, then clean streaming at the new profile's rate — measure fps per usecase and record the table). SET_FRAME_PERIOD to 50000 (20 fps) and back. REINIT round-trip. ONE commit.

### Task 5: Firmware robustness — EVENT emission + recovery **[HW]**

**Files:** `firmware/scanner-stream/Src/vl53l9_app.c` (+ `rs_protocol` helpers if needed).

- `rs_send_event(code, detail, msg)` per the long-specced EVENT payload (docs/protocol.md): emit at trigger-retry exhaustion (TRIGGER_TIMEOUT), DMA timeout (DMA_TIMEOUT), sensor error status (SENSOR_ERROR_STATUS), init failure (SENSOR_INIT_FAIL). Not-connected sends drop silently (existing policy) — fine.
- `handle_error` rework (raw-only path; keep the legacy spin for on-board-transform mode or share — judge): emit EVENT → bounded recovery: full sensor re-init cycle (reset → I3C address → init → calib re-read → profile → start), up to 5 attempts with increasing backoff (100 ms → 1.6 s), EVENT per attempt; success → resume streaming (CALIB retransmits, seq restarts — hosts already tolerate seq restarts); exhaustion → tud_disconnect + terminal spin (unchanged last resort).
- Boot bring-up: wrap the pre-loop sensor init in the same bounded retry (this converts the 1-in-5 boot hang into a self-healing delay). [HW] verify: (a) soak 10 consecutive SWD-reset boots — expect 10/10 reach streaming (vs historical ~80%); (b) induce a fault if feasible (REINIT storm or physically occlude/reset mid-stream is NOT reliable — acceptable evidence: code-path review + the boot soak + EVENT visible on at least one natural occurrence or a forced one via a temporary test hook, removed before commit; document what was actually exercised honestly). Viewer already prints EVENTs (Phase 1 work) — confirm live rendering of one real EVENT.
- ONE commit.

### Task 6: Connect-time transient forensics (offline first)

**Files:** analysis script (scratch or `host/tests/` if it earns keep), findings → `docs/` note or ROADMAP update.

- Both e2e captures (`captures/e2e_p2.bin`, `captures/e2e_p25.bin`) contain the transient in their recorded bytes. Byte-level forensics: locate the CRC-failing region — is it at stream start (pre-connect FIFO residue / partial frame from the DTR race), and does the corrupted fragment look like a truncated RAW mid-frame? Hypothesis menu: stale TX FIFO contents flushed on connect; DTR race sending a partial frame; host-side first-read artifact.
- If root cause is identified and the fix is small (e.g. `tud_cdc_write_clear()` on connect transition, or discard-until-first-magic device-side), implement + [HW] verify (3 fresh connects, zero CRC at connect). If not conclusively identified, write up the evidence and CLOSE the item as "characterized, cosmetic (1 frame at connect, self-healing)" with the analysis attached — either outcome is acceptable; no rabbit-holing beyond the captures + one hardware experiment round. ONE commit (fix or docs).

### Task 7: Viewer keys + config persistence + E2E + docs **[HW]**

**Files:** `host/src/roomscan/viewer.py`, new `host/src/roomscan/config.py`, ROADMAP, tests.

- Viewer key bindings (switch to `VisualizerWithKeyCallback`): `P` ping (prints ack), `C` request calib, `1-4` usecase, `R` reinit — commands ride the SAME open serial port via a CommandClient wired into the reader thread's source (single-owner constraint solved; replay mode: keys print "not available in replay"). Status line already shows events.
- Config persistence: `host/src/roomscan/config.py` — load/save `roomscan.toml` (repo root or `%APPDATA%`, pick and document): viewer defaults (color, fov, replay-fps, port override). CLI flags still win. Tests for load/save/priority.
- E2E [HW]: live session exercising keys (usecase cycle + calib + ping) with HUD evidence; 60 s soak at default profile confirming no regression (fps, gaps); suite green; ROADMAP Phase 3 status block (measured per-usecase fps table from Task 4, robustness soak result, transient verdict from Task 6, remaining open items honestly). ONE commit.

## Execution notes

- Order: 1 → 2 → 3 → 4 → 5 → 6 → 7 (3 can slot before 2's hardware step if convenient, but keep one implementer at a time; 6 is independent of 4-5 and can fill hardware-free gaps).
- The corrected Phase 3 premise (no transform-lib runtime controls; sensor re-init path instead) is already in ROADMAP — Task 4 is the embodiment. The transform-side `bypass-*` controls discovered in Phase 1 are NOT in scope (they live in the PC transform now — host-side config, trivially settable later).
- Tripwires: equivalence test green after every task; dual-stream mode compiles throughout (knob matrix).
