# High Frame-Rate Ranging Profiles & Manual Sensor Control — Implementation Plan

> **Source specification:** `docs/superpowers/specs/2026-07-31-high-framerate-and-manual-ranging-modes.md`
>
> **Required project skills while executing:** `protocol-change` for Tasks 2–3,
> `firmware-loop` for every task marked **[HW]**, `status-sync` before landing, and
> `milestone-retro` after the feature lands.

**Goal:** Ship Room Mapping, Precision, High Frame-Rate, and Manual ranging modes
whose controls reflect the configuration the sensor actually accepted; sustain a
measured 90 Hz full sensor stream over Ethernet; preserve the existing 30 Hz path;
let live SLAM consume high-rate frames without forcing the browser to render at
the acquisition rate; and make the LSM6DSV16X-derived IMU/env streams (9/10/11)
independently pollable at their own configured rate rather than permanently
coupled 1:1 to the ToF trigger.

**Architecture:** Replace the app's implicit dependency on the vendor usecase table
with a scanner-owned profile layer. Profiles are applied atomically at the existing
post-readout/pre-next-frame safe point. Frame-rate-controlled operation uses the
VL53L9CX autonomous synchronization mode; the existing manual-trigger loop cannot
meet the 11.1 ms period because raw readout plus its proven trigger-settle delay
already exceeds that budget. The device protocol moves to v2 because the requested
manual parameter block does not fit v1's fixed `cmd + u32` payload. On the host, one
pure profile-model module owns validation and consequence estimates, while a
server-side profile controller owns COMMAND/ACK state. The reader submits every
transformed frame to the latest-wins SLAM worker; the WebSocket broadcaster remains
a presentation-rate consumer. IMU/env draining moves onto its own service tick,
generalized from the pattern the sensor-idle loop already proves (it free-runs
streams 9/10/11 with a frozen `seq` while parked, with no ToF frame in flight at
all) and reused, not duplicated, by the active acquisition loop; no spare hardware
timer exists for this (TIM3 is already PWM, TIM2 is already the wall clock), so
pacing is a software tick against TIM2 reads, matching the idle loop's own
approach.

## Non-negotiable findings from the current tree

These are implementation constraints, not optional refinements:

1. **`SET_MANUAL_PARAMS` requires protocol v2.** Its fields occupy eight bytes
   (`u8 + u32 + u16 + u8`) in addition to the four-byte command code. Protocol v1
   permits exactly eight payload bytes total. This is a payload-layout change and
   therefore requires a version bump, updated C/Python codecs, and independent
   golden vectors in one commit.
2. **Target FPS is currently inert.** `vl53l9_set_frame_period()` only affects
   autonomous sync, while `vl53l9_app.c` unconditionally reasserts
   `VL53L9_SYNC_MANUAL`. The current loop's 2 ms `HAL_Delay` is about 3 ms wall time;
   combined with the approximately 9.5 ms raw readout, it cannot produce a true
   11.1 ms cadence even before exposure is counted.
3. **The vendor driver always enables DSS.** `vl53l9_set_binning()` writes
   `DSS_SHORT` or `DSS_LONG`; there is no public DSS setter. The scanner must add a
   local extension using the vendor's public register I/O, never edit
   `firmware/vendor/53L9A1/`.
4. **The host transform requires the 14,842-byte 3DMD shape.** A DSS-disabled sensor
   configuration must still supply the fixed-map LUT bytes expected by the existing
   readout and `roomscan.native.Transform`. A hardware/transform equivalence gate
   precedes the 90 Hz work.
5. **High-rate SLAM is presently capped at 30 submissions/s.** `SlamRunner.submit()`
   is called only from `_broadcaster()`, whose `POINT_INTERVAL` is `1/30`. Ingest and
   presentation must be split or the new sensor mode does not improve tracking.
6. **Ethernet pacing assumes a 33 ms frame.** `ETH_TX_WINDOW_MS` is 25 ms and its
   comments/budgets are explicitly 30 Hz assumptions. The full group includes RAW,
   quaternion, environment, raw IMU, and sync frames—not only the 14,842-byte ToF
   payload used by the UI's simple bandwidth estimate.
7. **The spec's numerical models conflict with its preset table.** For example,
   `50 + 5*6*30` is 950 mW, not the stated 200 mW; the ambient range equation gives
   8.5 m at 6 ms, not 8.0 m; and the precision equation gives about 7.9 m at 10 ms,
   not 8.8 m. The UI must not ship contradictory numbers.
8. **The requested 0.5 ms exposure step is not supported by the current driver.**
   `vl53l9_set_exposure()` accepts integer milliseconds. Half-millisecond control is
   not accepted until a local low-level setter is proven on hardware; otherwise the
   contract and UI use a truthful 1 ms step.
9. **Streams 9/10/11 are coupled to the ToF trigger only by convention, not by
   protocol or hardware.** The LSM6DSV16X's SFLP/sensor-hub blocks already free-run
   at their own fixed internal ODR (480 Hz XL/GY+SFLP, 60 Hz sensor-hub cycle)
   independent of when firmware drains them (`rs_lsm.c`); the wire protocol carries
   no frame-grouping structure, and every DATA frame is independently timestamped
   (`docs/protocol.md`); and the sensor-idle loop already proves the decoupled-
   emission pattern end to end — its own free-running tick drains and sends
   9/10/11 with a frozen `seq` and skips stream 13 entirely ("no ToF frame to
   describe"). Decoupling in the *active* streaming loop is new work only because
   that loop currently has no independent wake source — it blocks on the ToF
   FRAME_READY event with no service tick in between — and no spare hardware timer
   exists for one (TIM3 is PWM, TIM2 is the wall clock); pacing must reuse TIM2
   reads against a software-tracked next-due time, following the idle loop's own
   approach rather than a hardware IRQ.

## Global constraints and acceptance gates

- The default becomes **Room Mapping** only after its 30 Hz output is compared with
  the current AR_PRECISION baseline for rate, CRC/gaps, transform success, and depth
  plausibility.
- Binning remains 2 (54×42). Onboard-transform builds remain a golden-vector path
  and reject profiles above 30 Hz at compile time or command validation time.
- Ambient maps to `VL53L9_CONTEXT_LONG`; Precision maps to
  `VL53L9_CONTEXT_SHORT`. DSS is enabled at 60 Hz and below and forced off above
  60 Hz; above 60 Hz is valid only with Precision context.
- Manual FPS is an integer 1–100 and maps to
  `frame_period_us = round(1_000_000 / fps)`. Validation rejects combinations the
  sensor cannot schedule, including exposure/blanking that does not fit the frame
  period. The exact blanking margin is measured in Task 1, not guessed.
- Profile state is server-authoritative and changes only after a successful ACK
  containing device readback. No optimistic client state.
- The high-frame-rate preset remains selectable over CDC because the specification
  asks for a warning rather than a hard ban, but both UI and MCP results must report
  that the transport is unsupported above 60 Hz. Ethernet is the only 90 Hz
  acceptance path.
- The 90 Hz gate is: 60 s on Ethernet with RAW at 90 Hz within ±2%, zero CRC
  failures, zero sequence gaps, zero incomplete UDP frames, zero firmware TX queue
  drops, stream 9/10/13 paired to every RAW frame, and stream-11 samples averaging
  the configured 480 Hz ODR. If this fails, do not label a lower measured rate as
  “90 FPS”; fix the bottleneck or amend the specification with the measured ceiling.
- Power and max-range values remain labelled **estimated**. No measured claim is
  made without a power meter/range target measurement.
- IMU/env poll rate defaults to **coupled** (rate 0 = "one sample per ToF
  trigger," today's behavior, byte-identical framing) so existing captures and
  behavior are a strict subset of the new state space. An explicit nonzero rate
  decouples streams 9/10/11 onto their own tick; stream 13 (IMU_SYNC) is sent only
  on iterations where the drain is actually coincident with a ToF FRAME_READY
  edge, and is silently absent otherwise — never approximated or backfilled.
- Decoupled poll rate is an integer 1–480 Hz (the XL/GY/SFLP ODR ceiling).
  Validation reports, rather than silently drops, the case where a requested rate
  exceeds the 60 Hz sensor-hub cycle: stream 10 (env) sub-samples from the
  requested rate while stream 9 (quat)/11 (raw) can still hit it.
  Decoupled draining must never delay `rs_trigger_next()`/FRAME_READY servicing —
  a decoupled IMU/env rate does not get to steal cycles from the ranging
  profile's own timing budget.
- The IMU/env service tick is shared code between the active-streaming loop and
  the existing sensor-idle loop — one implementation of "drain LSM, emit 9/10/11
  on a frozen `seq`," not two divergent ones.
- `imufusion.py`'s `QUAT_REF_RATE_HZ` and the mapper's `baro_tau_frames` derive
  from the **actually applied IMU/env rate**, not the ToF profile rate, once the
  two are no longer assumed equal.
- Diagnostics that key sensor data by ToF `seq` (`host/tools/skew_check.py`'s
  `collect_frames()`) must handle N:1 — multiple independent IMU/env sends
  sharing one frozen `seq` — without silently dropping samples to
  last-write-wins.

---

### Task 1: Freeze the truthful profile contract and measurement baseline **[HW]**

**Files:**

- Modify: `docs/superpowers/specs/2026-07-31-high-framerate-and-manual-ranging-modes.md`
- Create: `host/src/roomscan/profiles.py`
- Create: `host/tests/test_profiles.py`
- Create: `host/tools/profile_probe.py`
- Modify: `host/src/roomscan/mcp_server/tools_data.py`
- Modify: `host/tests/test_mcp_registry.py`
- Modify: `docs/mcp-server.md`

1. Write failing table-driven tests for all four profile definitions, enum values,
   FPS↔period conversion, DSS/context rules, transport warning threshold, and invalid
   manual combinations. Include boundary cases at 1, 60, 61, 90, and 100 Hz and at
   1 ms and 16 ms exposure.
2. Add a pure `profiles.py` model with `ProfileId`, `RangingMode`, `PowerMode`,
   `ManualParams`, `ProfileConfig`, and `ProfileEstimate`. This module has no device,
   web, or Open3D imports and is the single host-side owner of range/power/bus math.
3. Reconcile the power/range formula conflicts against DS14879 Tables 9 and 36.
   Either correct the coefficients so the equations reproduce the documented anchors,
   or correct the anchors and explain the interpolation domain. Do not special-case
   preset labels over a different manual formula.
4. Resolve exposure granularity by proving whether a 500 µs local setter is possible.
   If not, change the specification and UI contract to a 1 ms step. Record the sensor's
   minimum schedulable blanking margin for each context so validation can reject
   exposure/period combinations before touching hardware.
5. Build `profile_probe.py` as a pure analyzer over a server-owned recording or decoded
   frame timestamps. Report requested FPS, measured median/p05/p95 interval, RAW rate,
   per-stream pairing, CRC/gaps, UDP incomplete/lost/reordered fragments, and stream-11
   effective sample rate. Never bind UDP/CDC directly while `roomscan-web` owns it.
6. Expose the analyzer through a thin MCP wrapper. It will be reused by every hardware
   task below and must report the observed configuration/rates, not the request.
7. Record a 60 s current-firmware baseline before changing sync/DSS. Pin actual RAW and
   sensor stream rates, I3C readout time, transform latency, link bytes/s, and TX queue
   health. This is the non-regression comparator for Tasks 4–6.

**Gate:** The amended spec, model tests, and probe agree on units and anchors. Any
unresolved model coefficient or exposure resolution blocks the consequence UI, but not
the protocol codec work in Task 2.

**Commit:** `test(profiles): pin ranging modes and measurement contract`

---

### Task 2: Protocol v2 codec, typed commands, and golden compatibility

**Files:**

- Modify: `docs/protocol.md`
- Modify: `firmware/scanner-stream/Src/rs_protocol.h`
- Modify: `firmware/scanner-stream/Src/rs_protocol.c`
- Modify: `host/src/roomscan/protocol.py`
- Modify: `host/src/roomscan/decoder.py`
- Modify: `host/tests/make_fixtures.py`
- Modify: `host/tests/test_protocol.py`
- Add/update: `host/tests/fixtures/*.bin`

1. Write failing tests for decoding an existing v1 capture after the host moves to v2,
   emitting ordinary v2 `cmd + u32` commands, emitting/parsing the 12-byte manual
   command payload, exact CRC coverage, partial reads at every byte boundary, garbage
   resynchronization, invalid enum/reserved values, and oversize rejection.
2. Bump `RS_PROTO_VERSION`/host `VERSION` to 2. Keep the 32-byte frame header unchanged;
   host `FrameHeader.unpack()` accepts versions 1 and 2 while new encodes use v2.
   Preserve v1 recording replay as a hard test.
3. Add synchronized registries:

   - command 8: `SET_RANGING_PROFILE`, legacy payload `<II>` (`cmd`, profile enum);
     preset IDs 0–2 apply immediately, while MANUAL (3) reapplies the last accepted
     manual candidate and is rejected until one exists,
   - command 9: `SET_MANUAL_PARAMS`, payload `<IBIHB>` (`cmd`, ranging mode,
     `frame_period_us`, exposure field with the Task-1-approved unit, power mode),
   - command 10: `GET_RANGING_CONFIG`, no parameter, needed to restore authoritative
     state after a web-server restart or a second client attaches,
   - command 11: `SET_IMU_ENV_RATE`, payload `<II>` (`cmd`, `rate_hz`; 0 means
     "coupled to the ToF trigger," the default and today's behavior; 1–480
     decouples streams 9/10/11 onto their own service tick — see Task 7),
   - command 12: `GET_IMU_ENV_RATE`, no parameter, returns applied rate and
     coupled/decoupled state; needed for the same restart/second-client
     restoration as command 10.

4. Make ACK parsing typed rather than a raw three-tuple. Commands 1–8 retain the legacy
   12-byte `<III>` ACK. Commands 9, 10, 11, and 12 return the complete applied/readback
   configuration after `cmd` and `result` (eight bytes for 9/10; four bytes — applied
   `rate_hz` — for 11/12), so the host can prove what the device applied instead of
   echoing the request. Document exact offsets, units, reserved bits, and failure
   semantics.
5. Replace `rs_parse_command(..., cmd, param, token)` with a bounded parsed-command
   struct carrying version, token, command, payload length, and decoded union. Derive
   total length from the validated header instead of one global 44-byte constant. Keep
   the parser HAL-free.
6. Add a host-compiled C parser cross-check driven from pytest: feed Python-generated v2
   vectors to `rs_protocol.c`, compare every decoded field, then feed independently
   generated fixture bytes to Python. Include concatenated 44/48-byte commands and magic
   split across reads.
7. Regenerate fixtures through `make_fixtures.py`, which must continue building bytes
   independently of `protocol.py`. Append the v2 entry to the protocol version history.

**Verification:**

```bash
cd host
.venv/bin/python -m pytest tests/test_protocol.py -q
.venv/bin/python -m pytest tests/test_decoder.py -q
```

**Commit:** One lockstep protocol commit:
`feat(protocol): v2 — ranging profiles and atomic manual parameters`

---

### Task 3: Host command client and CLI support for typed payloads

**Files:**

- Modify: `host/src/roomscan/control.py`
- Modify: `host/tests/test_control.py`
- Modify: `host/pyproject.toml` only if console metadata/help changes

1. Write failing tests for `CommandClient.send_profile()`,
   `send_manual_params()`, and `get_ranging_config()`: token matching, typed readback,
   interleaved DATA/EVENT frames, BUSY/error results, timeout, malformed extended ACK,
   and concurrent callers.
2. Generalize `CommandClient.send()` to accept a packed command payload while retaining
   the existing convenience path for commands 1–8. Preserve its crucial thread contract:
   writes never happen on the reader thread and ACKs are offered by that thread.
3. Extend `roomscan-ctl` with:

   ```text
   profile room_mapping|precision|high_framerate
   manual --ranging ambient|precision --fps N --exposure N --power ulp|lp|regular
   profile-status
   ```

   Print requested and applied/read-back values separately and return nonzero on timeout,
   protocol mismatch, unsupported transport, or device rejection.
4. Keep the old `usecase`, `period`, and `exposure` actions for one release as explicit
   low-level diagnostics; mark them deprecated in help because they are non-atomic and
   `period` is inert under old manual-sync firmware.

**Verification:**

```bash
cd host
.venv/bin/python -m pytest tests/test_control.py tests/test_protocol.py -q
```

**Commit:** `feat(host): add typed ranging profile controls`

---

### Task 4: Scanner-owned profile application, DSS control, and readback **[HW]**

**Files:**

- Create: `firmware/scanner-stream/Src/rs_ranging.h`
- Create: `firmware/scanner-stream/Src/rs_ranging.c`
- Modify: `firmware/scanner-stream/CMakeLists.txt`
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c`
- Modify: `firmware/scanner-stream/Src/rs_protocol.h`
- Modify: `firmware/scanner-stream/Src/rs_protocol.c`

1. Define scanner-owned preset constants matching the amended spec. Do not modify
   `g_ranging_profiles[]` in the vendor tree. Wrap `vl53l9_profile_t` with scanner fields
   for public profile ID and DSS state; keep binning fixed at 2.
2. Add an app-local DSS setter using `vl53l9_write8()` and
   `VL53L9_REGADDR_STANDBY_DSS_MODE(context)`. Apply it immediately after the vendor
   profile helper, because `vl53l9_set_binning()` always rewrites DSS.
3. Read back sync, context, power, frame period, exposure, binning, and DSS while the
   device is in standby. Build ACKs for commands 9/10 from these readbacks; never from
   the requested candidate.
4. Expand `rs_pending_cmd_t` to hold the typed candidate and originating transport. Keep
   one in-flight reconfiguration and return BUSY for a second. Validate all fields before
   stopping the sensor; apply the whole candidate or restore the previous whole profile.
5. Route commands 8/9/10 through the existing post-readout/pre-trigger safe point. ACK
   only after readback matches. A failed restore enters the existing bounded recovery;
   a failed request that restores cleanly returns `SENSOR_ERROR` without adopting the
   candidate.
6. Make Room Mapping the boot default. Reinit, hard-standby wake, soft-standby wake, and
   recovery must all preserve `g_active_profile`, sync mode, and DSS state.
7. Keep the raw buffer at 14,842 bytes when DSS is disabled. On hardware, confirm the
   fixed LUT is still readable and `Transform.process()` accepts the frame. Record one
   matched DSS-on/off static scene and compare transform success, valid-zone count, and
   depth distribution. If the driver returns a shorter/invalid raw frame, stop and solve
   the fixed-map LUT encoding before continuing.
8. Compile both production raw-only and onboard-transform knob configurations. The latter
   rejects high-rate/DSS-off commands rather than entering an unverified transform path.

**Hardware gate:** Commands 8/9/10 work over both CDC and UDP, readback is exact, every
profile survives REINIT and hard standby, and DSS-off raw frames transform without a
size/error regression.

**Commit:** `feat(firmware): apply ranging profiles atomically`

---

### Task 5: Autonomous acquisition loop and real target-FPS behavior **[HW]**

**Files:**

- Modify: `firmware/scanner-stream/Src/vl53l9_app.c`
- Modify: `firmware/scanner-stream/Src/rs_ranging.h`
- Modify: `firmware/scanner-stream/Src/rs_ranging.c`
- Modify: `host/tests/bench_commands.py`

1. Introduce explicit acquisition state (`MANUAL_TRIGGER` only for the legacy
   onboard-transform path; `AUTONOMOUS` for the production profile path). Remove every
   unconditional `VL53L9_SYNC_MANUAL` override from production boot/reinit/apply.
2. In autonomous mode, start the sensor once and consume FRAME_READY → DMA read → ack at
   the configured period. Do not call `rs_trigger_next()` or pay its settle delay.
   Preserve the existing event timestamps, stream grouping, command safe point, standby,
   and bounded recovery semantics.
3. Prove that `vl53l9_stop()` is safe at the post-readout command point in autonomous
   mode. Instrument status/readback through an EVENT or protocol diagnostic because
   ST-Link VCOM may be unavailable on this Linux host. Never rely on an unobservable
   `printf` for the acceptance result.
4. Test upward in controlled steps: 30 → 60 → 90 → 100 Hz, with compatible exposure and
   DSS settings. At each step assert the applied period first, then record 60 s and run
   `profile_probe`; a byte-identical or unchanged-rate result is a failed plumbing check,
   not evidence that period is irrelevant.
5. Size timeouts from the applied frame period with a bounded margin; retain recovery for
   missing FRAME_READY and DMA completion. Ensure changing 90→30 and 30→90 while streaming
   produces one bounded discontinuity, a successful ACK, and stable subsequent cadence.
6. At 90/100 Hz, measure I3C transfer duration and remaining scheduling margin including
   the LSM timestamp latch and FIFO drain. The UI model remains a ToF-airtime estimate,
   while the hardware report states total observed bus work.

**Hardware gate:** Default Room Mapping and Precision each meet 30 Hz without regression;
High Frame-Rate meets the global 90 Hz acceptance gate before transport/UI work calls it
supported.

**Commit:** `feat(firmware): run high-rate profiles in autonomous sync`

---

### Task 6: Ethernet high-rate pacing, queue telemetry, and CDC isolation **[HW]**

**Files:**

- Modify: `firmware/scanner-stream/Src/ethernet_transport.c`
- Modify: `firmware/scanner-stream/Inc/ethernet_transport.h`
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c`
- Modify: `host/src/roomscan/sources.py`
- Modify: `host/src/roomscan/web.py`
- Modify: `host/tests/test_sources.py`
- Modify: `host/tests/test_web.py`

1. Add tests around a pure pacing-budget helper: it derives the drain deadline from the
   active frame period, drains enough fragments at the actual `ETH_Process()` cadence,
   never interleaves frames, never abandons a partial frame, and handles the periodic
   CALIB/IMU_CAL burst.
2. Replace the fixed 25 ms/33 ms assumption with an applied-period-aware deadline. Record
   queue high-water, pending fragments, enqueue drops, stack stalls, and emitted bytes.
   Expose the counters through an existing diagnostic response or a documented additive
   EVENT/ACK field so the hardware gate can prove zero rather than infer it.
3. Define active data transport truthfully. When Ethernet has a target and applied FPS is
   above 60, DATA goes to Ethernet only; an open but non-draining CDC endpoint must not
   inject 100 ms stalls into the Ethernet acquisition path. Control ACKs return to the
   command's originating transport.
4. Preserve automatic CDC fallback when Ethernet has no target. The host surfaces
   `udp`, `cdc`, `replay`, or `none` in authoritative state rather than deriving it in
   browser JavaScript.
5. Extend UDP source metrics with the firmware queue counters and total link rate when
   available. Keep fragment reassembly compatible with reordered datagrams.
6. Run wired-Ethernet 60 s captures at 60, 90, and 100 Hz, then repeat 90 Hz through the
   actual Wi-Fi bridge. The release gate is wired Ethernet; bridge loss is reported
   separately rather than allowed to masquerade as a sensor ceiling.

**Hardware gate:** 90 Hz satisfies the global gate and an attached/open CDC port does not
change Ethernet cadence, gaps, or TX queue health.

**Commit:** `feat(transport): pace full-rate sensor streams over Ethernet`

---

### Task 7: Independent IMU/env poll-rate control **[HW]**

**Files:**

- Modify: `firmware/scanner-stream/Src/vl53l9_app.c`
- Modify: `firmware/scanner-stream/Src/rs_lsm.c`
- Modify: `firmware/scanner-stream/Src/rs_lsm.h`
- Modify: `firmware/scanner-stream/Src/rs_protocol.h`
- Modify: `firmware/scanner-stream/Src/rs_protocol.c`
- Modify: `firmware/scanner-stream/Src/ethernet_transport.c`
- Modify: `host/src/roomscan/control.py`
- Modify: `host/src/roomscan/protocol.py`
- Modify: `host/src/roomscan/profiles.py`
- Modify: `host/tools/skew_check.py`
- Modify: `docs/protocol.md`
- Modify: `host/tests/test_protocol.py`
- Modify: `host/tests/test_control.py`
- Modify: `host/tests/test_profiles.py`
- Create/modify: `host/tests/test_skew_check.py` (add one if none exists)

1. Extract the sensor-idle loop's existing drain/emit logic (`vl53l9_app.c`'s idle
   path, `rs_idle_lsm_tick`) into a shared `rs_lsm_service_tick()` helper in
   `rs_lsm.c`/`rs_lsm.h`, parameterized by coupled-vs-free-running-at-rate-X, used
   by both the idle loop and the new active-loop path below. Preserve idle-loop
   behavior exactly — regression: idle-loop measured rate stays ~18.2 Hz, unchanged.
2. Implement commands 11 (`SET_IMU_ENV_RATE`) and 12 (`GET_IMU_ENV_RATE`) against
   the protocol v2 codec added in Task 2. If Task 2 has already landed without
   them, add them there first as a follow-on lockstep protocol commit under the
   `protocol-change` checklist — golden vectors, host codec, C cross-check, exact
   ACK offsets.
3. In the active acquisition loop, replace the current unconditional per-ToF-frame
   drain with a call into the shared service tick: coupled mode (rate 0, the
   default) keeps today's exact behavior — drain once per ToF frame, stream 13
   always paired. Decoupled mode paces drains off TIM2 against a
   software-tracked next-due timestamp, independent of the FRAME_READY wait,
   sending 9/10/11 with `seq = g_last_seq` (frozen) and skipping stream 13 on
   iterations with no coincident FRAME_READY edge — the same convention the idle
   loop already uses.
4. Validate/report the rate-vs-hub-cycle mismatch: a requested rate above the
   60 Hz sensor-hub cycle sub-samples stream 10 specifically (quat/raw can still
   hit the requested rate); reject requests above 480 Hz (the XL/GY/SFLP ODR
   ceiling).
5. Route commands 11/12 through the existing post-readout/pre-next-frame safe
   point and BUSY/ACK discipline established by Task 4, independent of
   ranging-profile changes — an IMU/env rate change must not require a ToF
   profile reapplication, and vice versa.
6. Update Task 6's Ethernet pacing/queue-budget helper: decoupled IMU/env sends
   add TX load off the ToF cadence, so the budget can no longer be derived solely
   from the applied ToF frame period.
7. Rework `host/tools/skew_check.py`'s `collect_frames()` from a `seq`-keyed dict
   to a `seq`-keyed list (or a time-windowed grouping) so multiple decoupled
   sends sharing one frozen `seq` are all retained instead of last-write-wins
   overwritten. Add a test covering 2+ IMU/env sends between two ToF frames.
8. Extend `host/src/roomscan/control.py`/`roomscan-ctl` with
   `imu-rate <hz|coupled>` and `imu-rate-status`, mirroring the typed-readback
   pattern from Task 3. Extend `profiles.py` with an `imu_env_rate_hz: int | None`
   field (`None`/0 = coupled) so it participates in the same validation/estimate
   model as ranging profiles.
9. On hardware, verify: coupled mode is byte-identical to pre-feature firmware
   (same rate, same stream-13 pairing); decoupled 30 Hz IMU/env holds steady
   while ToF runs Room Mapping, Precision, and High Frame-Rate in turn; decoupled
   90 Hz IMU/env holds steady while ToF runs at its slowest profile; stream 13 is
   present only on genuinely coincident iterations at every combination; and
   draining never delays FRAME_READY servicing (ToF frame timing is unaffected by
   the IMU/env rate in force).

**Hardware gate:** Coupled mode regresses nothing. Decoupled mode holds the
requested IMU/env rate within measured tolerance independent of the concurrent
ToF rate in all combinations above, with zero silently-dropped `skew_check`
samples, stream 13 correctly absent off-edge, and no measurable perturbation of
ToF frame cadence.

**Commit:** `feat(firmware): decouple IMU/env poll rate from ToF acquisition cadence`

---

### Task 8: Rate-aware IMU and barometer behavior

**Files:**

- Modify: `host/src/roomscan/imufusion.py`
- Modify: `host/src/roomscan/slam/mapper.py`
- Modify: `host/src/roomscan/slam/worker.py`
- Modify: `host/src/roomscan/slam/remote.py`
- Modify: `host/src/roomscan/slam/service.py`
- Modify: `host/src/roomscan/web.py`
- Modify: `host/tests/test_imu_fusion.py`
- Modify: `host/tests/test_slam_mapper.py`
- Modify: `host/tests/test_slam_service.py`

1. Write tests proving a 30-second barometer time constant is equivalent at 30 and
   90 Hz **applied IMU/env rate** (`900` versus `2700` sample counts), including a live
   rate change that does not reset the map or introduce a correction step, and including
   the case where the applied IMU/env rate differs from the concurrent ToF rate (Task 7).
2. Pass the applied IMU/env poll rate — not the ToF target FPS — with each SLAM
   submission and update the mapper's `baro_tau_frames = round(30.0 * imu_rate_hz)` under
   the worker thread. In coupled mode (Task 7's default) this equals the ToF rate as
   before; in decoupled mode it must track streams 9/10's own rate, sourced from Task 7's
   `GET_IMU_ENV_RATE` readback/applied config, never derived from the ranging profile.
   Carry the same scalar through the remote-worker message so local and service backends
   match.
3. Keep replay truthful: use each stream's own capture timestamp rate when no live
   profile target exists. Never assume the ToF rate for streams 9/10/11, and never assume
   30 Hz for a 90 Hz recording.
4. Do not merely assign `QUAT_REF_RATE_HZ`: it is currently a derivation/comment constant
   and changing it would have zero runtime effect. Make reference rate an explicit
   `ImuFusion` input sourced from the applied/decoupled IMU rate (Task 7), recompute only
   the rate-derived yaw crossover term selected in Task 1, and leave the existing
   timestamp-based propagation/tilt gains in seconds. Tests must show the effective gain
   changes while coupled 30 Hz behavior stays numerically unchanged.
5. On a stationary replay resampled to 30 and 90 reference updates — including a
   decoupled combination where the IMU rate does not match the ToF rate — compare
   orientation noise and phase response in seconds, not frames. Reject a change that
   merely produces a different number without preserving the filter's intended
   time-domain behavior.

**Commit:** `feat(slam): preserve filter time constants across sensor rates`

---

### Task 9: Decouple full-rate SLAM ingest from browser presentation

**Files:**

- Modify: `host/src/roomscan/reader.py`
- Modify: `host/src/roomscan/web.py`
- Modify: `host/src/roomscan/slam/worker.py`
- Modify: `host/src/roomscan/metrics.py`
- Modify: `host/tests/test_viewer_reader.py`
- Modify: `host/tests/test_web.py`
- Modify: `host/tests/test_slam_panel.py`

1. Add a tested optional transformed-frame consumer to `_run_reader()`. It is invoked for
   every transformed RAW frame after statistics update and must remain non-blocking.
2. When live SLAM is active, the consumer snapshots the matching orientation/environment
   state and calls the worker's latest-wins `submit()`. Move submission out of
   `_broadcaster()`; leave `poll()` and MESH/SLAM WebSocket publication there.
3. Keep point-cloud/IR/browser rendering at explicit presentation cadences (30 Hz point
   cloud, 15 Hz IR/sensors unless a measured browser reason changes them). The browser
   should never be asked to render 90 point clouds/s merely because ingest is 90 Hz.
4. Add separate metrics for RAW received, transform completed, SLAM submitted, SLAM
   processed, latest-wins replacements, and browser frames sent. A 90 Hz source with a
   30 Hz browser must report deliberate presentation decimation separately from data or
   SLAM loss.
5. Verify the callback is inert in point-cloud mode, replay pacing remains correct, source
   swaps reset the SLAM worker once, and command writes still never run on the reader
   thread.
6. Benchmark the real CUDA worker at 90 Hz on an idle host. Require p99 processing below
   the 11.1 ms input period and no sustained latest-wins replacements before claiming the
   high-rate preset improves tracking. If compute cannot keep up, report processed rate
   and keep acquisition/render behavior honest.

**Commit:** `refactor(web): separate sensor ingest from render cadence`

---

### Task 10: Authoritative web profile state, controls, and bus visualization

**Files:**

- Modify: `docs/web-protocol.md`
- Modify: `host/src/roomscan/web.py`
- Modify: `host/src/roomscan/control.py`
- Modify: `host/src/roomscan/static/index.html`
- Modify: `host/src/roomscan/static/controls.js`
- Modify: `host/tests/test_web.py`
- Modify: `host/tests/test_static_ui.py`

1. Add a server-owned `RangingState` containing transport, current/applied profile,
   applied manual values, target FPS, measured FPS, consequence estimate, pending token,
   validation warning, and last error. Initialize it through `GET_RANGING_CONFIG`, not an
   assumed firmware default.
2. Add dedicated inbound messages:

   ```json
   {"type":"set_profile","profile":"high_framerate"}
   {"type":"set_manual_params","ranging_mode":"precision","fps":90,
    "exposure_ms":4,"power_mode":"regular"}
   ```

   The server validates through `profiles.py`, dispatches off the event loop, commits
   state only from successful device readback, and broadcasts a `ranging` message plus
   updated `state`. Replay returns an explicit unavailable result.
3. Ensure switching to Manual is one atomic command. Do not send profile, period,
   exposure, and power as four independently applied operations. Preset selection uses
   command 8; manual uses command 9.
4. Replace the old two-button Usecase area with the four-way selector. Add the manual
   ranging/FPS/exposure/power controls, paired range+number inputs, applied/requested
   status, and the consequence readouts. Debounce slider input and allow only one pending
   device command.
5. Render the I3C bar immediately beneath the selector from the server's estimate. Use
   the specified green/yellow/red thresholds and text that says **ToF bus airtime**, not
   total bus use. Include transfer time, frame period, and remaining airtime.
6. Show a prominent CDC warning whenever applied or pending FPS is above 60. Use the
   server-provided transport field; never guess from URL, browser location, or link rate.
7. Preserve one-way state flow: clicks do not set active segments locally. A failed,
   BUSY, or timed-out command leaves the previous applied state visible and announces the
   error. A second tab receives the same state immediately.
8. Add an IMU/env poll-rate control beside the ranging selector: a coupled/auto default
   (matches the ToF profile, today's behavior) plus an explicit Hz field, driven by the
   `set_imu_env_rate`/`RangingState`-analogous state from Task 7's commands 11/12. Show
   applied-vs-requested rate and a visible warning when the requested rate forces stream
   10 (env) sub-sampling above 60 Hz. Same one-way state flow and atomic-command
   discipline as the ranging controls — this is a second, independent pending command,
   not a fifth field bolted onto Manual ranging.
9. Add a `title` to every new control; extend static tests for tooltip coverage,
   duplicate IDs, min/max/step agreement with `profiles.py`, and the Device card's
   reachability. Test the consequence text against known numeric values, not only types.
10. Visually verify at desktop and narrow widths with the headless UI tools. Confirm the
    manual panel and the new IMU/env rate control do not make the Device card exceed its
    dock band; use progressive disclosure rather than deleting readouts.

**Commit:** `feat(web): add ranging modes and live bus consequences`

---

### Task 11: MCP controls and effect-verified automation

**Files:**

- Modify: `host/src/roomscan/mcp_server/tools_rig.py`
- Modify: `host/src/roomscan/mcp_server/tools_data.py`
- Modify: `host/src/roomscan/mcp_server/session.py`
- Modify: `host/tests/test_mcp_tools.py`
- Modify: `host/tests/test_mcp_registry.py`
- Modify: `docs/mcp-server.md`

1. Add `profile_estimate(...)` over the pure model and `rig_profile(...)` for preset,
   manual, and query operations. The latter talks only to roomscan-web; it never binds the
   device stream.
2. Wait until the echoed `ranging.applied` state matches the requested/read-back config.
   A command log line or any newly arrived state message is insufficient proof. Return
   `ok=false` for BUSY, timeout, replay, unsupported CDC rate, validation failure, or an
   applied mismatch.
3. Include requested, applied, estimated, measured FPS, transport, and warning fields in
   the result. Make `rig_status()` include the latest ranging state.
4. Add `rig_imu_env_rate(...)` mirroring `rig_profile(...)`'s verified-readback pattern:
   accepts `coupled` or an explicit Hz value, waits for the echoed applied rate (not a log
   line), and returns `ok=false` for BUSY/timeout/replay/an unreachable rate (above 480 Hz,
   or above 60 Hz with a caller expecting unsampled env). Include it in `rig_status()`
   alongside the ranging state.
5. Test stale broadcasts, timeout after an unchanged state, successful exact readback,
   CDC warning, replay rejection, and two sequential manual changes — for both
   `rig_profile()` and `rig_imu_env_rate()`, independently and interleaved (a profile
   change in flight while an IMU-rate change lands, and vice versa).

**Commit:** `feat(mcp): expose verified ranging profile control`

---

### Task 12: End-to-end validation, documentation, and landing **[HW]**

**Files:**

- Modify: `docs/protocol.md`
- Modify: `docs/web-protocol.md`
- Modify: `docs/mcp-server.md`
- Modify: `ROADMAP.md`
- Modify: `BUGS.md` if validation finds a defect or leaves a measured limitation
- Modify: `CLAUDE.md` only if a durable architecture/status statement changes
- Modify: `.remember/now.md`
- Modify: `.remember/recent.md`
- Modify: canonical memory only for durable measured findings not already owned by docs

1. Run focused suites after their tasks, then the complete host suite:

   ```bash
   cd host
   .venv/bin/python -m pytest -q
   .venv/bin/python -m ruff check src tests
   ```

2. Build both firmware knob configurations. For production:

   ```bash
   export PATH="$PWD/host/.venv/bin:$PATH"
   cd firmware/scanner-stream
   cmake --preset Debug
   cmake --build build/Debug
   ```

3. Use `firmware-loop`/MCP firmware tools to flash and observe over Ethernet. Run the
   Task-1 probe for Room Mapping 30, Precision 30, Manual 1/60/61/90/100 boundaries, and
   High Frame-Rate 90. Exercise profile changes, REINIT, soft standby, hard standby, and
   wake at least once each. Additionally exercise Task 7's IMU/env rate independent of
   ranging profile: coupled (default), decoupled 30 Hz against every ToF profile in turn,
   decoupled 90 Hz against the slowest ToF profile, and the >60 Hz env-sub-sampling case.
4. For each acceptance capture record requested/applied config, measured interval
   percentiles, RAW and sensor rates, CRC, device seq gaps, UDP incomplete/lost/reordered,
   firmware TX drops/high-water, link bytes/s, IMU sample rate (and, for decoupled runs,
   applied vs. measured IMU/env rate independent of the ToF rate, plus stream-13
   presence/absence correctness), transform failures, SLAM submissions/processed/
   replacements, and browser render FPS.
5. Exercise CDC at 60 Hz and request >60 Hz. Verify the warning is visible and returned
   by MCP; do not require a clean 90 Hz CDC stream. Return to Ethernet and prove recovery
   without reboot.
6. Drive the real browser UI: all four profile buttons, invalid manual combinations,
   slider debounce, pending state, successful readback, bus-bar colors at 60/90/100,
   CDC warning, reconnect/new-tab synchronization, narrow layout, tooltips, and the new
   IMU/env rate control (coupled default, an explicit decoupled rate, and the env
   sub-sampling warning above 60 Hz).
7. Re-run the 30 Hz baseline scene/capture and compare rate, transform output, orientation
   health, and SLAM tracking. The new default is accepted only if the intended profile
   difference is documented and no transport/protocol regression appears.
8. Follow `status-sync`: update protocol/web/MCP docs and ROADMAP with actual numbers in
   the same landing commits. Do not retain predicted 90 Hz or power/range numbers where
   measurement corrected them.
9. Land completed work straight to `main` in small verified commits. Keep the Task-2
   protocol artifacts in their single lockstep commit. Do not push unless the owner asks.
10. Run `milestone-retro` before beginning the next milestone; convert any repeated probe,
    pacing, or hardware ritual into the shared skill/MCP surface.

## Recommended execution order and stop points

```text
Task 1 contract/baseline
  -> Task 2 protocol v2
  -> Task 3 host typed control
  -> Task 4 atomic firmware profiles + DSS proof
  -> Task 5 autonomous 30/60/90 hardware gate
  -> Task 6 Ethernet/CDC transport gate
  -> Task 7 independent IMU/env poll-rate control
  -> Task 8 rate-aware filters
  -> Task 9 full-rate SLAM ingest
  -> Task 10 web UI
  -> Task 11 MCP
  -> Task 12 full validation/status sync/retro
```

- Tasks 2–3 may continue while Task 1 resolves estimate coefficients, but the profile
  model/UI cannot ship until Task 1 is closed.
- Stop after Task 4 if DSS-off frames cannot be transformed as 14,842-byte 3DMD.
- Stop after Task 5 if autonomous acquisition cannot sustain the requested cadence; do
  not compensate by relabelling measured FPS or by dropping paired sensor streams.
- Stop after Task 6 if the firmware TX queue or UDP receiver loses frames at 90 Hz. SLAM
  and UI work must be validated against a clean source, not an already lossy stream.
- Stop after Task 7 if decoupled IMU/env draining measurably perturbs ToF frame timing,
  or if the shared service tick cannot reproduce the idle loop's existing ~18.2 Hz
  behavior unchanged. Task 8's rate-aware filters depend on Task 7's applied-rate
  readback and cannot ship against an assumed rate.
- The deprecated `panel.py` is out of scope. Its low-level controls remain for diagnostics,
  but the supported four-mode experience lands in `roomscan-web` and MCP.
