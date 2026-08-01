# Roadmap — 53L9A1 3D Room Mapping

Product goal: a **tethered handheld 3D room scanner**. The STM32H563 streams timestamped sensor
frames to a PC running real-time SLAM (Open3D tensor ICP + TSDF); an offline pass fuses 4K phone
video into a ToF-seeded 3D Gaussian Splat. Full design + critical review:
[`references/roadmapResearch.md`](./references/roadmapResearch.md).

Active development happens in this `roomscanner/` workspace. The existing STM32 firmware is **read-only
reference** in the vendored-in-repo `firmware/vendor/53L9A1/` package; firmware paths below (`Src/…`) are relative to
`firmware/vendor/53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/` (aka `<APP>`).
Engineering conventions live in [`docs/engineering-practices.md`](./docs/engineering-practices.md).

## Overriding architecture decisions

- **Transport: native USB CDC OR Ethernet UDP (Phase 5).**
  *(Revises the 2026-07-10 "Ethernet is shelved" decision.)* The device now streams flawlessly over either USB CDC or Ethernet (UDP unicast). If Ethernet is plugged in, the device acts as a DHCP client (or falls back to a self-assigned IP server) and streams via UDP to the host when a packet is received. This removes the USB cable length limit and prepares the plumbing for Phase 6's hardware time-sync (PTP) requirements. USB CDC is still supported and automatically falls back if Ethernet is not connected. *(2026-07-21: the host→device COMMAND channel now works over Ethernet too — previously CDC-only, the ETH transport discarded inbound datagrams; see the Phase 3 command-channel addendum.)*
- **Sensors: X-NUCLEO-IKS4A1** adds IMU (LSM6DSV16X, hardware SFLP orientation), magnetometer (yaw-drift
  correction), barometer (Z-drift constraint), temp/humidity (thermal comp). **Integrated as of Phase 4
  (2026-07-10)** — LSM6DSV16X as a native I3C target sharing I3C1 with the ToF (HUB1-only routing,
  multi-device ENTDAA), SFLP orientation on stream 9, sensor-hub env (baro/mag/temp) on stream 10;
  stacking recipe + resolution history in `docs/iks4a1-stacking.md`. *(The original shared-bus
  legacy-I2C plan failed at speed once stacked — see the Phase 4 status block.)*
- **Sequencing rule (owner):** mature the visualizer + UI/config on the **ToF sensor alone** before adding
  the IKS4A1 board. *(Satisfied as of Phase 3, 2026-07-09 — visualizer, runtime config, and robustness
  are done; owner swapped IKS4A1 up to Phase 4, ahead of Ethernet.)*
- **Protocol rule:** design the frame protocol transport-agnostic from day one —
  `magic + version + seq + timestamp + payload + CRC32`, multi-stream, little-endian — so an eventual
  Ethernet cutover (Phase 5 — since shipped, vindicating the rule) is plumbing, not a redesign. Spec lives in `docs/protocol.md`; any
  wire change bumps the version and follows the `protocol-change` skill checklist.
- **Firmware fork rule:** our firmware lives in `roomscanner/firmware/` as a copy of `<APP>` that
  references the `53L9A1/` package in place for shared Drivers/Middlewares/Utilities. `<APP>` itself is
  never edited. Our copy is hand-maintained (we accept divorcing from CubeMX regeneration; keep the
  `USER CODE` guards anyway so a future regen remains possible).
- **Post-processing runs on the PC (owner decision, 2026-07-08).** The `vl53l9-transform-c` pipeline is
  the throughput wall on the M33 (~37-40 ms/frame ≈ 25 fps ceiling at full 54×42 — a hard requirement;
  see `docs/h563-optimization-notes.md`: the M33 has no vector FPU, CORDIC/FMAC don't fit this workload,
  and fidelity-neutral micro-optimizations buy only ~5-10%). The MCU becomes a thin bridge: raw `3DMD`
  frames (14,842 B at full res, per `docs/vl53l9cx-datasheet-notes.md` p.20) + the calibration blob once
  at startup stream to the PC, which runs the same portable-C transform bit-exact at desktop speed.
  Raw at 30 Hz ≈ 445 KB/s fits USB CDC today; ~100 Hz ≈ 1.5 MB/s fits the Ethernet UDP link.
  But I3C readout at 12.5 MHz makes 100 Hz raw unreachable on this board anyway (realistic I3C
  ceiling ~60-80 Hz, estimate; the sensor's CSI-2 output is its true 100 Hz path but the H5 has no CSI-2
  receiver). Ethernet was implemented in Phase 5 to remove cable limits and prep for PTP sync.
- **Deferred on-device optimizations** (recorded in case the on-MCU transform path is ever revived):
  `powf(x, const)` → multiplies in `ratenorm.c`/`sharpener.c` shadowed copies (verified `powf` survives
  in the ELF; est. 0.3-2 ms/frame), `-flto` (est. low single-digit %), SRAM bank placement for
  DMA-vs-CPU contention (speculative), acquisition/processing overlap via autonomous trigger mode +
  GPDMA2-driven async TX (est. → ~20-25 fps on-device). Full analysis: `docs/h563-optimization-notes.md`.

## Considered and rejected

- **HDR exposure-bracketing (2026-07-09).** Proposal: sweep `SET_EXPOSURE_MS` and per-pixel fuse the
  best-conditioned return to widen depth/IR dynamic range. **Rejected — redundant with the sensor's on-chip
  Dynamic SPAD Selection (DSS).** Per ST engineer: DSS is per-zone hardware auto-gain (all SPADs for
  dull/far, down to 1–2 for bright/near; 16 steps/zone, visible in the raw frame's 4-bit/zone DSS map),
  applied before accumulation; the sensor also dual-ranges (two PRIs, radar-aliasing rejection) and returns a
  fully-processed depth we can't reprocess host-side. DSS trades collection *area*; exposure trades
  integration *time* — so host HDR would only add range at DSS's extreme tails (retroreflector past min-SPAD,
  or very dark/far past all-SPAD), a corner case not worth a subsystem. Owner shelved it, trusting DSS. If
  ever revisited: a firmware `DISABLE_DSS` command would be the enabling prerequisite.

## Reference-firmware bugs — do not inherit

Found during review of `<APP>/Src/vl53l9_app.c`; fix these in our fork, leave the reference untouched:

1. **`vl53l9_trigger_frame` return value never checked** (`vl53l9_app.c:203-206`): the call's result is
   discarded and the stale `ret` from `vl53l9_start` is tested — trigger failures pass silently.
   **✅ Fixed in our fork** — the trigger's return is captured and checked
   (`firmware/scanner-stream/Src/vl53l9_app.c:1537` and the `:464` wrapper).
2. **`handle_error()` spins forever** (`vl53l9_app.c:317-322`): fine for a demo, wrong for a scanner. Our
   firmware must emit an error/event frame to the host and attempt sensor re-init before giving up.
   **✅ Fixed in our fork, Phase 3 Task 5** (raw-only build): EVENT emission + bounded re-init recovery
   (5 attempts, 100 ms→1.6 s backoff), boot bring-up wrapped the same way (10/10 boot soak, was ~80%) —
   see the Phase 3 status block below.
3. **`print_frame` divide-by-zero on flat scenes** (`vl53l9_app.c:296`): `(max - min)` is the divisor; a
   uniform depth field makes it 0. Also `min - average` underflows `uint32_t` when `average > min`
   (`vl53l9_app.c:288`). Moot once ASCII printing is replaced, but don't copy the pattern.
4. **`allocate_memory(uint16_t size)`** caps buffers at 64 KB — silent truncation risk if a future
   profile/stream needs more. Widen to `size_t` in our fork. **⚠ Still inherited as of 2026-07-10**
   (`firmware/scanner-stream/Src/vl53l9_app.c:1223`/`:1969` still take `uint16_t`) — safe today (largest
   allocation is the 14,842 B raw buffer) but widen it before adding any larger buffer.
5. **Blocking `printf` throttles the loop**: all output shares the 115200-baud VCOM. Any streaming path
   must be measured for TX-time vs frame-time and must drop frames rather than stall acquisition.
6. **Resource frees commented out** (`vl53l9_app.c:263-269`): acceptable in a never-exiting loop, but our
   app gains stop/reconfigure paths in Phase 3 — the teardown sequence must actually work by then.
   **✅ Addressed in our fork, Phase 3** — the raw-only build has no on-MCU transform to free. The
   sensor stop → re-profile → restart cycle is exercised inline by SET_USECASE, while rs_sensor_reinit()
   is exercised live by REINIT and the recovery path.

## Cross-cutting risks (watch continuously)

- **Struct packing / endianness**: Cortex-M33 and x86 are both little-endian, but never wire-cast packed
  structs across the link without a golden-vector test proving C encoder and Python decoder agree
  (`docs/protocol.md` defines the vectors).
- **Timestamp wraparound**: the platform profiler timestamp is 32-bit; extended to 64-bit µs on the MCU
  before it enters a frame header (wraps at ~71 min otherwise).
- **Backpressure**: on every transport (UART, CDC, UDP), a stalled host must cost frames, not sensor
  cadence. Sequence numbers increment per *captured* frame so the host can quantify drops.
- **Windows COM enumeration**: the board will expose two serial ports (ST-Link VCOM + native CDC). The
  host app selects by USB VID/PID, never by "first port found".
- **`-Ofast` on float depth data**: implies `-ffast-math` (no NaN semantics). Any NaN/invalid-depth
  sentinel handling must live host-side or use explicit sentinel values, not NaN checks, in firmware.

## Agent tooling — `roomscan-mcp` ✅ (2026-07-29)

The agent-facing surface is an MCP server (`host/src/roomscan/mcp_server/`, `.mcp.json`,
`docs/mcp-server.md`) exposing 19 typed tools: `rig_*` (drive a running `roomscan-web` over `/ws`),
`ui_*` (headless Chrome held open across calls, screenshots returned as image blocks), `capture_*` /
`doctor` / `orientation_probe`, and `fw_build` / `fw_flash` / `run_tests`. Thin layer only — each
wrapped script keeps its `argparse` `main()` as a prose printer over the same pure function
(`analyze_capture.scan()`, `Doctor(quiet=True).results`), verified by byte-identical CLI output.

Two decisions worth keeping:
- **Client, never competitor.** The server never binds the device stream; `capture.py` stays
  CLI-only and recording goes through `rig_record()`, so the contention rule is structural.
- **12 scripts are deliberately CLI-only** (scratch tier, deprecated panel, one-shot rigs), each with
  a recorded reason. `host/tests/test_mcp_registry.py` fails on any `host/tools/` script that is
  neither exposed nor excluded — verified to go red on a probe script.

Measured along the way, both correcting the repo: the **Bash sandbox no longer kills network
listeners** (uvicorn bound `0.0.0.0`, served, exited 0 — `docs/headless-host-setup.md` and the
`agent-sandbox-port-binding` memory are stale), and **Playwright renders the Three.js scene on this
GPU-less host** via `channel="chrome"` + SwiftShader, so it is now the `ui_*` backend (raw CDP kept
as fallback) for native `wait_for_function`. **Five** "confidently wrong answer" bugs were found and
fixed during verification, all from trusting a timer-driven broadcast as proof of effect — including
one that reported failure while 734 clean frames *were* recorded, and a `rig_command` that called a
device timeout a success. See `docs/mcp-server.md` → "Two invariants".

**Firmware tools verified on-rig 2026-07-29.** `fw_build` (text 148044 / data 13231 / bss 54232) and
`fw_flash` (chipid `0x484`, written + verified) were run against a board that had **stopped
responding entirely** — no ping, no command ACK, `device_hz: None`, though SWD saw it fine at
3291 mV. Reflashing revived it: ping 0% loss, streams 7/9/10/11 at **30.5 Hz, 0 drops, 0 gaps**,
`ping` → `OK applied=1`, and 614 recorded frames with 0 CRC failures. Cause of the original hang not
investigated — worth watching for a recurrence.

## Data-collection queue — owner-collected captures (2026-07-31)

Everything on this roadmap and in `BUGS.md` that is blocked on **data only the owner can collect**.
Stable IDs: refer to these as **DC-A … DC-I** in later sessions. Update the Status column when one
lands; do not renumber.

**Session prep for all of them.** Restart `roomscan-web` first — a long-lived server pins the code
from whenever it *started*, which is what faked the 2026-07-31 "SLAM gave up" (see 6.D). Record
through the web UI's Record button (`/ws {"type":"record"}`), **not** `capture.py --udp` — both bind
the device stream. Stay **off the tripod** for anything touching heading (BUG-034: +15–27 µT).
Ceiling bookends are welcome when a capture happens to have them and are **never** a required
protocol (walking to park the device puts the operator in the FOV — owner, 2026-07-30).

| # | Capture | Unblocks | Protocol | Acceptance gate | Status |
|---|---|---|---|---|---|
| **DC-A** | **Brisk-motion handheld scan** — one room, 60–90 s | Phase 6.D: BUG-036's escalating ICP retry has **never run in the field** (the run that exposed it was executing pre-fix code). Also probes the open "no relocalization" hole | Normal-pace sweep with 3–4 deliberate **fast** whips (~1.5 m/s ≈ 50 mm between frames, vs the 16.9 mm median that BUG-036 measured). Don't be gentle — the point is to trigger the retry | 0 CRC, stream 9 present. On replay: `tracking_stats` shows **escalations > 0**, `died == false`, no frozen-translation segment in the `.tum` | ✅ **PASS** (`DebugCapA.bin`, scored 2026-07-31) — escalations 7–47/run, 0/5 died, longest freeze 28 < 30. **First field exercise of BUG-036's retry, and it worked** |
| **DC-B** | **Multi-room closed loop ×2** — 2–3 rooms or a corridor circuit, 3–5 min each, two takes of the *same* route | Phase 6.D items 3–4, the loop-closure go/no-go. The baseline is single-room only (~3% over 24 m) and the paired 95%-CI gate has only the two coffee-room circuits to score. Loop closure is supposed to earn its keep on **multi-room** trajectories — nothing in `captures/` tests that | Start parked on a marked pose, walk the route, return to the **same** marked pose. Revisit at least one area mid-route (that is what a pose-proximity edge needs). Two takes so the gate is paired | 0 lost frames, stream 9, byte-clean. ~135 MB per 5 min | ❌ **collected ×3, gate FAILS on "0 lost frames"** (`DebugCapB1/B2/B3`, 2026-07-31) — 2.29/4.28/9.35% transport loss, **BUG-049**. B1 usable as a provisional baseline (**2.71 ± 0.58 m over 75.3 m = 3.6%**); B3 is 9.6% on the *same route*. **Re-record after BUG-049** — see below |
| **DC-C** | **Tracking-loss stress scan** — one room, ~90 s | Relocalization (Phase 6.D, explicitly open: the retry survives a bad *frame*, not a bad *second*). No capture contains a real tracking-kill event | Mid-scan, kill tracking deliberately 2–3×: palm over the sensor ~2 s, or point at a blank surface <20 cm away — then **return to already-mapped geometry** and continue | Byte-clean; the kill events identifiable on replay. This is a fixture, not a good scan — it should look bad | ✅ **PASS as a fixture** (`DebugCapC.bin`, 2026-07-31) — 0 CRC, 376–467 lost/run, freezes of 82–120 frames (2.7–4.0 s), 70–133 escalations. **Every run recovered** (0 died) because the protocol's "return to already-mapped geometry" is what relocalization would otherwise be for |
| **DC-D** | **Flat-field pan** — 20 s over a uniform matte surface | Phase 2.5 follow-up: reflectance carries ~18% per-zone FPN and the correction is **built and shipped-disabled** waiting only on this (`docs/flatfield-calibration.md`) | Blank painted wall / foam board / grey card at ~0.5–1 m, roughly perpendicular, **slowly panning the whole time**. A static capture is invalid — it bakes scene texture into the "correction" | ≥100 panned frames; `build_flatfield` residual in the low tens of percent, gains comfortably inside [0.5, 1.6]. Gains near the [0.33, 3.0] clip bounds ⇒ recapture | ✅ **PASS** (`DebugCapD.bin`, 2026-07-31) — residual **7.3%**, gains 0.754–1.224 (mean 1.000, sd 7.1%), **all 2268 zones** inside [0.5, 1.6], none near the clip bounds. ~490 panned frames at 14.9 °/s. Flat-field is ready to enable |
| **DC-E** | **Braced fixed-heading tilt sweep** — ~2 min | `docs/superpowers/plans/2026-07-29-orientation-resume.md` §4.6: BUG-030's closure proved the calibration's **magnitude**, not its **direction**. An ellipsoid fit is ambiguous up to a rotation (DT0103) — every sample on a perfect sphere while the field vector is systematically rotated. Bounded at ~2.5° by the near-spherical soft iron; measured at nothing | Hand-held, off the tripod. Pick **one fixed compass bearing** and keep pointing at it. Sweep tilt level → 45° → vertical, **holding each ~15 s**. Two cycles | `mag_check` tilt table flat (expected) **and** `absolute_heading` agreeing across every hold. Disagreement ⇒ implement DT0103's accelerometer-assisted fit | ⚠️ **collected to spec, gate FAILS on the tilt-ramp clause** (`DebugCapE.bin`, 2026-07-31) — 7 holds of 15–18 s at tilt 3.9/46.1/90.3/50.3/4.1/47.8/90.9°, exactly two cycles. Ramp **1.72×** (GOOD < 1.10). ⚠️ The heading clause is **partly artifact** — see BUG-048; re-score it after the singularity fix before sizing DT0103. **The singularity fix landed 2026-07-31 (BUG-051, `yaw_twist_deg`) — DC-E's heading clause is now re-scorable and that re-score is the next step here.** Note the artifact was larger than BUG-048 estimated: an 18.4° systematic bias at the operating pose, not only noise near lock |
| **DC-F** | **Controlled pan set** — 3 takes × ~60 s | The two claims currently inferred from stationary data plus arithmetic: applying the measured **+7.76 ms quat phase lead** (on the wire since BUG-031, nothing consumes it — now the largest motion-error term), and the `imufusion` A/B (built, gated off, no capture carries orientation ground truth). Also resume-doc §4.5 | Brace against a repeatable start (a corner, taped marks), pan to a repeatable end, hold. One take each at roughly **slow ~20 °/s / medium ~50 °/s / fast ~100 °/s**. 10 s stationary at both ends of every take | Endpoints repeatable enough that A→B is the same rotation across takes; 0 CRC; stream 11 present. The stationary bookends give the noise floor for free | ⚠️ **gate PASSES, purpose NOT unblocked** (`DebugCapF.bin`, 2026-07-31) — collected above spec: **4** pans at 19/25/36/89 °/s with 5 bookend holds of ~11 s. Pans agree to **1.42°** of nominal 90°. But the bookends' own noise floor (0.3–1.7°) **exceeds** the predicted phase-lead effect (0.15–0.69°), so the rate-vs-error test is underpowered, not falsifying. Phase offset re-measured here as **+5.13 ms** (sign confirmed *lead*), vs the +7.76 ms on record |
| **DC-G** | **Recorded magnetometer tumble** — 30–45 s | Magcal regression fixtures. The tumble that closed BUG-030 went straight through the modal, so **no capture contains one** — covered-shell tests still use a synthetic fixture (`tilt_sweep_20260729.bin` fills 2 of 92 cells, `web_20260729_061440.bin` fills 6) | Open the calibration modal, hit Record, free-tumble to good coverage, stop | ≥60 of 92 shell cells covered. Low priority — test data, not a decision | ⬜ open |
| **DC-H** | **USB CDC connect transient** — 5 × 15 s | **BUG-005**: fix implemented 2026-07-30, **the code path has never executed**. `CAFE:4001` does not enumerate on the headless host (USB_USER is powered from the battery bridge) and `/dev/ttyACM*` return `root:root` mode 0 after every replug | Needs the board's USB_USER cable into a machine that can open the port (udev rule or run as root). Fresh connect, `host/tools/capture.py --seconds 15`, five times | `capture_analyze` reports **0** CRC failures in the connect region (today: exactly 1) and the first frame after connect is CALIB | ⬜ open |
| **DC-I** | **Phase 7 seed set** | COLMAP pose priors + depth-regularized 3DGS | **Do not collect yet** — needs a rigid phone/webcam mount and a hand-eye extrinsic calibration, neither of which is designed. Listed so it is not a surprise when Phase 6 closes | — | ⬜ blocked on design |
| **DC-J** | **Specular / mirror behaviour** — 42 s, room scan then dwell on a large mirror | Nothing in the repo analysed specular surfaces; "mirror" elsewhere means the **UI view mode**. Unplanned — the owner recorded it to see what would happen | Normal room scan, then point at a large mirror and dwell | *(added retroactively)* Does the map gain phantom geometry; does tracking survive | ✅ **characterized, no defect** (`DebugCapMirror.bin`, 2026-07-31) — see below |

### DC-B — the multi-room result, and why 6.D is now blocked on the transport (2026-07-31)

Three takes of the same route, each ~3 min, scored with 10-run innocuous-perturbation ensembles
(`slam_ensemble`, `--device CUDA:0`). **All three fail the "0 lost frames" gate** on transport loss
(BUG-049), so these are provisional, not the baseline the phase wanted.

| take | transport loss | horizontal closure | path | drift | tracking |
|---|---|---|---|---|---|
| `DebugCapB1` | 2.29% | **2.71 ± 0.58 m** | 75.32 ± 0.53 m | **3.6%** | 0/10 died, worst lost 15, no freeze |
| `DebugCapB2` | 4.28% | *excluded* | — | — | **628-frame (21.2 s) mid-run freeze** |
| `DebugCapB3` | 9.35% | **7.28 ± 0.71 m** | 75.60 ± 0.52 m | **9.6%** | 0/10 died, worst lost 92, 67-frame freeze |

**1. The multi-room drift *rate* is not worse than single-room.** B1's 3.6% against the single-room
baseline's ~3.1% (0.74 ± 0.19 m over 23.9 m) says frame-to-model drift scales with **path length**, and
does not compound at room transitions. On this evidence the case for loop closure earning its
complexity indoors gets **weaker**, not stronger — which is the opposite of what 6.D expected to find.

**2. Transport loss, not the SLAM algorithm, dominates multi-room error.** B1 and B3 are the same route
walked to within 0.4% of the same path length, and differ 4× in transport loss. Paired bootstrap over
the matched perturbation set: **4.569 m mean difference, 95% CI [3.950, 5.168] m** — decisively
non-zero. The mechanism is directly observed on B2, whose 628-frame tracking collapse begins **70 ms
after** its 2.4 s transport outage ends (BUG-049).

> **Therefore 6.D's loop-closure evaluation cannot proceed on this data.** Roughly 4.6 m of B3's 7.3 m
> drift is caused by dropped packets. No pose-graph result measured against that is interpretable, and
> the paired gate would be scoring the network. **Fix BUG-049, re-record DC-B, then evaluate.**

**3. Vertical drift is the weak axis and wants its own look.** B1 ends **−1449 ± 826 mm** below its
start on a single-floor route, with the barometer contributing only +109 mm — so this is ICP vertical
drift. Scaling the single-room circuit's 125 ± 76 mm by path length predicts ~390 mm, so vertical
appears to grow faster than linearly. The ±826 mm spread makes that ~1.8 sd, i.e. **suggestive, not
established** — worth a targeted measurement rather than a conclusion.

**4. `died` does not catch a mid-run freeze.** B2 reports `died == false` (its tail is healthy) while
15.5% of its trajectory is a frozen dead-reckoned segment, and still produces a plausible-looking
closure. `tracking_stats.died` is trailing-only by construction; **`longest_lost_run` is the field that
catches this class**, and `slam_ensemble` now surfaces it as `worst_longest_lost_run` with a warning.

### DC-J — what a mirror actually does to this sensor (2026-07-31)

Measured against the capture's own room-only segments as baseline (the scan starts normal, which is
what makes it useful). Mirror dwell isolated to **t = 39.6–56.3 s** by no-return-sentinel fraction.

- **What comes back is multipath, not a virtual room.** Sentinel (12000 mm) fraction rises 0.07% →
  **12.1%**, but the *non*-sentinel returns spread diffusely over 600–5800 mm with no sharp peak at
  twice the mirror distance. The feared failure — a coherent phantom room integrated behind the wall —
  **does not occur**.
- **The map is not corrupted.** Block consumption *during* the dwell runs at **17.2 blocks/frame vs
  28.9 pre-dwell**: the mirror contributes *less* geometry, not phantom geometry. Mesh bbox grew
  proportionally with path length, with no blow-up to sentinel or 2×-distance scale.
- **Tracking is essentially unaffected** — 4–12 lost frames of 1270 across a 5-run ensemble, longest
  freeze 8, 0 died, 23 ICP escalations.
- **A real specular glint exists but is tiny**: reflectance max 4543 during dwell vs 192 in the room
  (24×), confined to **0.015%** of pixels. That is a plausible cheap specular *detector* if one is
  ever wanted.
- **Confidence does not flag it** — mirror returns are dampened monotonically with range rather than
  marked, so there is no per-pixel validity signal. A concrete new data point for **BUG-007**.

Conclusion: **no bug, no mitigation warranted.** TSDF/ICP already discard sentinel and multipath
returns structurally. Recorded here so the question is not re-opened from first principles.

**Explicitly NOT owner-data-blocked** (do not add these to the queue): ~~compression go/no-go and the
pacer measurement (Phase 5.5 — the link measures zero loss over ~567k frames and loss must **not** be
manufactured)~~ **⚠️ that exclusion is WITHDRAWN as of 2026-07-31: the link no longer measures zero
loss.** The three DC-B takes lost **2.29% / 4.28% / 9.35%** of RAW frames while byte-clean, in
multi-second whole-group outages — see BUG-049. Loss no longer has to be manufactured; it is in the
captures. The remaining exclusions stand: Detailed-SLAM iteration calibration and the track-at-10 mm/reconstruct-at-5 mm
question (6.D item 2 — runs against `coffeeRoomCircuit*.bin`); Allan-variance characterisation
(`captures/stationary_stream11_20260728_190311.bin`, 900 s, already recorded); SHT40 humidity
(firmware work gated on a consumer existing). One non-data blocker: 6.D's end-to-end browser/server
verification needs an agent permission profile that allows local port binding — a settings change,
not a capture.

## Phases

### Phase 0 — ✅ Complete
On-device transform pipeline + ASCII depth map over ST-Link VCOM.
Enabled by `CONF_PRINT_FRAME = 1` in `Src/vl53l9_app.c:31`.

### Phase 1 — Real-time 3D visualizer  ← **✅ Complete** (plan: `docs/superpowers/plans/2026-07-07-phase1-binary-protocol-visualizer.md`)

> **Status 2026-07-08:** both milestones verified on hardware.
>
> **1a** (ST-Link VCOM @921600): 0 CRC failures, 0 seq gaps, ~5.9 fps (sensor frame time + blocking
> UART co-limit; see `.superpowers/sdd/task-8-report.md`).
>
> **1b** (native USB CDC, TinyUSB, VID:PID `0xCAFE:0x4001`): 0 CRC failures, 0 seq gaps, 0 drops over a
> 20 s continuous capture (273 frames) — **13.65 fps**. Stall/recovery test (2 s read → 5 s host stops
> reading, port held open → 5 s resume) behaved exactly per the drop-policy design: 1 transient CRC
> failure from the mid-frame abort, one seq gap of 29 frames (dropped while the host wasn't draining,
> correctly marked with `FLAG_DROPPED` on the next successfully-sent frame), then clean contiguous
> decoding resumed with no further loss — see `.superpowers/sdd/task-11-report.md`.
>
> **The plan's "fps ≥ 15" figure is stale, not a miss**: a per-frame breakdown (`HAL_GetTick` deltas,
> 20-frame samples with an active CDC host draining) shows `transform_process_stream` ≈ 37-40 ms,
> the CDC send itself ≈ 8-9 ms, and sensor trigger/I3C readout/event-wait ≈ 26-29 ms — total ≈ 74 ms/frame
> (13.5 fps). The CDC link is *not* the bottleneck (send is ~12% of the frame budget and headroom is
> large — CDC FS moves the 9108 B frame in a fraction of that 8-9 ms of wall time budget, the rest is
> host-driven FIFO pump/schedule slack); the ceiling is sensor + on-MCU transform time, unchanged from
> milestone 1a's finding. Speeding this up (binning, usecase, or moving processing off the acquisition
> loop) is Phase 3+ scope, not a Phase 1 blocker.
>
> Known follow-up (unchanged): ~1-in-5 boots hang in sensor bring-up before frame 1 → needs EVENT-frame
> reporting + re-init recovery (wire contract for EVENT frames is already specced in `docs/protocol.md`).

Replace ASCII printing with a **versioned binary frame protocol** and a PC app that deprojects depth into
a live-rendered point cloud.

**Deliverables**
- `docs/protocol.md` — wire spec v1 (32-byte header, depth/ZF32 stream, CRC32) + golden test vectors.
- `host/` — Python package `roomscan`: streaming decoder (resyncs on corruption), depth→XYZ deprojection,
  serial + file-replay sources, raw-capture recorder, Open3D live viewer with fps/drop HUD.
- `firmware/scanner-stream/` — fork of `<APP>` that emits binary frames. Two milestones, both ✅:
  **1a** over ST-Link VCOM at 921600 baud (~5.9 fps — proved the whole chain with zero new
  middleware), then **1b** over native USB CDC FS (13.65 fps — full sensor+transform rate; link
  itself has ample headroom, see status note above).
- `docs/transform-streams.md` — captured `streams_inspect` / `controls_inspect` startup dump
  (`vl53l9_app.c:91-98`). **Capture this at first flash** — it enumerates what the transform library can
  emit (depth / reflectance / confidence / possibly XYZ) and settles on-MCU vs PC-side deprojection, and
  it scopes Phase 2/3.

**Acceptance** — ✅ met
- Live point cloud renders on the PC at the sensor's native frame rate over CDC; seq-gap counter proves
  zero drops with the host idle; recorder + replay reproduce identical clouds.

**Risks / bugs to watch**
- **No ST USB Device middleware in the `53L9A1/` package** — superseded: milestone 1b vendored
  **TinyUSB** instead of `STM32_USB_Device_Library` (see the `firmware/vendor/tinyusb` commits); CDC
  ACM enumerates on `hpcd_USB_DRD_FS` with HSI48 as the USB kernel clock, confirmed on hardware.
- **CDC TX re-entrancy** — resolved: `rs_cdc_send` pumps `tud_task()` while draining
  `tud_cdc_write_available()` and aborts (drop, not retry-spin) after a 100 ms stall; verified on
  hardware by a stall/resume test (see status note above).
- **ZF32 units and range unverified** — believed float millimetres of perpendicular Z
  (`radial_to_perp.c` exists in the algo set). Confirm empirically at capture time before hardcoding the
  mm→m conversion.
- **FoV constants for deprojection** — resolved in Phase 2.5: datasheet-derived defaults (55.0°H/42.0°V,
  `docs/vl53l9cx-fov-notes.md`), independently confirmed by a ZAPC least-squares best-fit (54.65°/42.50°,
  `docs/deprojector-validation.md`) within 0.35°/0.50° — no XYZ output stream exists (ZAPC is the closest
  equivalent; see Phase 2's stream facts below).
- ST-Link VCP at 921600: V3EC supports it, but verify clean reception (frame CRC failures at rate 0)
  before trusting milestone 1a numbers.

### Phase 2 — Raw streaming + PC-side transform (revised 2026-07-08) ← **✅ Complete**

> **Status 2026-07-08:** verified end-to-end on hardware. Firmware streams raw sensor frames only; the
> `vl53l9-transform-c` pipeline runs natively on the PC via a ctypes-wrapped DLL.
>
> **Equivalence gate** (Task 4 — the go/no-go everything else was gated behind): PC-side transform
> output vs. the same raw input processed on-MCU, compared over the full 731-pair hardware capture
> (`captures/golden_pairs.bin`, seq 1..731, one continuous 65 s run) — **731/731 pairs within the
> 0.01 mm tolerance**, max abs diff **0.000854 mm** (p50 0.000366, p90 0.000488, p99 0.000610 mm).
> **0/731 pairs are bit-exact** — reported honestly: the PC build (`/fp:precise`) and the MCU build
> (`-Ofast`) diverge slightly from float instruction reordering/reassociation, not a correctness bug;
> the divergence is over an order of magnitude below the gate. **PASS** — the on-MCU transform is
> retired.
>
> **Raw-only firmware** (Task 5): `CONF_TRANSFORM_ONBOARD=0` — the sensor streams RAW_3DMD (14,842 B)
> plus periodic CALIB (2,332 B, every 64 RAW frames) over native USB CDC. Measured **24.6 fps** (491
> frames / 19.921 s), just under the 25 fps target — CDC send-time serialization on top of the
> mandatory 5 ms settle + sensor ranging time, not a sensor limit (frame-time breakdown in the Task 5
> report). Confirmed again live in Task 7's soak runs (steady 23.3-27.0 fps across 1600+ frames).
>
> **Host pipeline** (Task 6): `TransformStage` bridges RAW/CALIB frames to depth arrays via the native
> DLL, lazily constructed on the first CALIB frame (depth-only replays never touch the DLL); viewer HUD
> gained `raw`/`raw-skip` counters. 39/39 tests passing.
>
> **Live end-to-end** (Task 7): `roomscan-view` against the live board, raw-only firmware, multiple
> supervised soaks (~55-113 s each): steady **~24-25 fps**, **0 seq gaps**, `raw` climbing 1:1 with
> `frames` throughout (1620 frames in the recorded run, `captures/e2e_p2.bin`). One CRC failure and
> one `FLAG_DROPPED` appeared at connection time — a **first occurrence**, not previously seen at
> connect: Phase 1 Task 11's 20 s soak and Phase 2's Task 2 (1471 frames) / Task 5 (499 frames)
> connect-time captures were all clean (Task 11's single CRC event came from its deliberate stall
> test, a different mechanism). The transient is one frame, does not recur within the run, and
> reproduces identically on replay of the same capture (i.e. it's in the recorded bytes, not decoder
> nondeterminism). Cause unexplained — observed once and now tracked in the deferred list below
> (candidate common root with the 1-in-5 boot hang: sensor bring-up timing). **`raw-skip` behavior, now
> documented**: on a **freshly SWD-reset** board, `raw-skip` stays **absent (0)** for the whole run —
> CALIB arrives before any RAW, as designed. On a board that had already been streaming since an
> earlier session, a host attaching mid-cycle sees a transient `raw-skip` (observed: 31, stable, never
> grows) because CALIB is retransmitted only every 64 RAW frames, not re-sent on every new host
> connection — a real, benign behavior, not a bug.
>
> **Stall/recovery** (Task 7): mid-run 5 s host-stops-reading (port stays open, same procedure as
> Phase 1's Task 11) — one transient CRC failure from the mid-frame abort, one seq gap (37 frames, seq
> 49→87), exactly one `FLAG_DROPPED` on the recovery frame, then clean contiguous decoding resumed
> (292 further frames, 0 further gaps/failures). **New for Phase 2**: `TransformStage` was fed straight
> through the gap — all 292 post-recovery RAW frames transformed to valid depth (`depth_ok=292,
> depth_bad=0`, no NaN/negative values), confirming the pipeline stays numerically sane across a
> dropped-frame boundary. The on-MCU TNR (temporal noise reduction) filter's state continuity *is*
> broken by the gap (its internal history assumes contiguous frames) — expected to show up as a
> one-time transient in the depth output's noise characteristics right after the gap, not as invalid
> data; this is expected live behavior given the drop policy, not a defect.
>
> **Replay identity** (Task 7): `captures/e2e_p2.bin` (the live run's RAW+CALIB recording) replayed
> through the same viewer/pipeline path at `--replay-fps 25` — `raw` climbing at the paced rate, 0 seq
> gaps, the same 1 CRC failure / 1 dropped-flag baked into the recording reproduced identically, no
> traceback. Confirms replay exercises the full PC-transform path on the exact recorded bytes.
> Replay identity is guaranteed only for recordings started from a device boot (frame 1): a mid-session `--record` starts at an arbitrary point, so its replay re-runs the transform with fresh TNR state after the next CALIB — a brief filter transient vs the live render, below sensor noise.
>
> **Deferred / follow-up** (not blockers for calling Phase 2 done) — **resolved in Phase 2.5 except where
> noted still-open:**
> - ✅ **Reflectance/confidence/ambient/`--color` viewer support** — shipped Phase 2.5 Task 2: the shim
>   grew a mask-selection API (`rst_create2`/`rst_process2`, `DEPTH|REFLECTANCE|CONFIDENCE|AMBIENT|ZAPC`)
>   and the viewer gained `--color {depth,reflectance,confidence}` (default changed to `reflectance` in bug tracker branch, falls back to `depth` if absent)
>   with a one-time stderr fallback notice if the requested plane is absent from the stream. Verified live
>   on hardware (Task 5, this doc's Phase 2.5 note below): IR-shaded cloud renders, no fallback warning,
>   no traceback.
> - ✅ **Trigger-early overlap** — shipped Phase 2.5 Task 4: the raw-only loop now triggers frame N+1
>   before sending frame N over CDC, hiding the ~15 ms send inside the sensor's ranging window. Measured
>   **27.76 fps** (up from 24.6), 2 ms settle (down from 5 ms; the one bounded experiment the task allowed),
>   0 crc, 0 gaps, re-confirmed by this task's 60 s live soak (below). **Strategic implication**: the CDC
>   send is no longer on the critical path at all (fully hidden inside ranging). Ethernet's value going forward is what the Phase 4 section already
>   says: 100 Hz-class rates, hardware PTP timestamping, and zero-config direct-link — not a fps lift for
>   this loop.
> - ✅ **ZAPC Deprojector validation** — done Phase 2.5 Task 3: ZAPC's z is bit-identical to ZF32 depth
>   (hard-asserted, 0.0 mm diff); best-fit FoV 54.65°H/42.50°V agrees with the datasheet defaults within
>   0.35°/0.50°; worst-case linear-model displacement is corner-concentrated (127 mm / 6.36% of z at
>   row 0, col 53, vs. 12-20 mm center-region) and doesn't improve with a global FoV tweak, so the linear
>   defaults stand and an **optional per-zone tan-table path** was added to `Deprojector` (constructor arg,
>   linear stays default) for future consumers needing corner accuracy. Full numbers, conventions, and the
>   decision writeup: `docs/deprojector-validation.md`. ~~**Vendor-bug note**: ZAPC's 4th (confidence)
>   channel is structurally ~1.0 on every zone including no-return sentinels — not usable as a validity
>   gate. Root cause (uninitialized `conf_scaling` divisor, never assigned anywhere in the `53L9A1/`
>   tree — the channel is structurally constant, no capture can change it; the sentinel zones'
>   1e-6-digit micro-variation is actually a packed filter-status code, not a confidence score) is
>   documented in `docs/deprojector-validation.md`'s confidence-channel section.~~ **Resolved**: The uninitialized `conf_scaling` divisor in the reference transform library was fixed by initializing it to `1.0f` in `radial_to_perp.c`, allowing the ZAPC confidence channel to dynamically vary and discriminate correctly (verified via `validate_deprojector_zapc.py`). Depth-sentinel gating remains a robust fallback mechanism.
> - **✅ Resolved — connect-time CRC/DROPPED transient** (first observed Phase 2 Task 7): root-caused
>   Phase 3 Task 6 by byte-exact forensics on both recorded instances (`captures/e2e_p2.bin`,
>   `captures/e2e_p25.bin`) — full writeup `docs/connect-transient-forensics.md`. Both captures show the
>   *identical* signature down to the byte: a perfectly well-formed RAW_3DMD seq=1 header immediately
>   after CALIB seq=1, truncated ~2.8 KB short of its declared payload+CRC, followed by `FLAG_DROPPED` on
>   seq=2. This is the pre-existing `rs_cdc_send()` 100 ms mid-frame-abort/DROPPED-flag mechanism (the
>   same one the stall/recover experiments deliberately trigger) firing once, for free, because the
>   host's own startup latency between DTR-assert (on port open) and its first live `.read()` can exceed
>   the firmware's 100 ms per-write budget on frame 1. **Characterized-cosmetic**: costs exactly one RAW
>   frame, self-heals with no seq gap, never recurs within a session, no wire/decoder change needed. Not
>   the mid-stream-reattach mechanism (see the CALIB-on-DTR-connect item below) — the CALIB `seq=1` and
>   early `t_us` in both captures prove these are genuinely fresh boots, not stale reconnects.
> - **Open — CALIB-on-DTR-connect** (mid-stream reattach, architecturally distinct from the item above —
>   see `docs/connect-transient-forensics.md`'s "DTR-gate one-shot" section): CALIB retransmit cadence
>   means a host attaching mid-cycle discards up to 63 RAW frames (~2.3 s blind start at 27.76 fps).
>   **Partially mitigated**: Phase 3 Task 2 shipped `SEND_CALIB` (`roomscan-ctl calib`) — a host can now
>   request CALIB on demand instead of waiting out the cadence. An automatic fix (device aborts any
>   in-flight frame and sends CALIB immediately on DTR rising, via `tud_cdc_line_state_cb`) was evaluated
>   Task 6 and found **not** small/safe enough to land there — it needs new synchronization between a
>   TinyUSB callback context and the main loop's send/trigger state (`raw_mem_index`, `rs_calib_countdown`,
>   in-flight `rs_cdc_send()`); specced as a Phase 3/4 follow-up, not implemented. Live evidence of the
>   blind start this fix addresses: Phase 2.5 Task 5's `--color` run attached mid-cycle and observed
>   `raw-skip 37` (stable, within the documented ≤63 ceiling).

Migrate post-processing to the PC per the architecture decision above. This **absorbs the original
Phase 2** (IR + additional streams): once the transform runs host-side, every output stream — depth,
reflectance, confidence, ambient, amplitude, status, and the ZAPC point cloud — is available on the PC
for free; multi-stream firmware plumbing is no longer needed.

- Firmware: new RAW stream over the existing protocol (`stream_id` from the registry; raw `3DMD`
  payload + a one-time calibration/EVENT frame at startup carrying `calib_data`); acquisition loop
  simplifies (no transform, no output buffer) — target the sensor's characterized 30 Hz profile.
- Host: build `vl53l9-transform-c` as a native library (portable C; needs a thin platform shim),
  wrap for the `roomscan` pipeline (raw frame + calib in → chosen output streams out), golden-test
  bit-exactness against an on-MCU-produced depth capture from Phase 1 (we have `captures/` +
  `hw_capture_snippet.bin` as ground truth).
- Viewer: colorize the cloud by IR reflectance/confidence (original Phase 2 UI goals), stream toggles.
- Acceptance — **met, with honest caveats**: full 54×42 raw streaming over USB CDC at 24.6 fps
  (target was ~30 fps — see the fps note above for why; not a blocker), PC-transform output within
  0.01 mm of the Phase 1 on-MCU output for the same raw input (not bit-identical — 0% exact-match rate,
  documented above; equivalence here means "within tolerance").

- **Stream facts** (Task 7 capture, `docs/transform-streams.md`): the transform library exposes `depth`,
  `ambient`, `amplitude`, `confidence`, `reflectance`, `status` outputs; wire stream IDs 0-6 are
  allocated in `docs/protocol.md`'s registry. With the transform host-side these are PC-config choices,
  not firmware features. The `ZAPC` point-cloud format now also runs on the PC and was used (Phase 2.5
  Task 3) to validate the host `Deprojector`'s linear-FoV model against calibrated intrinsics — datasheet
  defaults confirmed, optional per-zone tan-table added for corner accuracy (`docs/deprojector-validation.md`).
  ~~**Vendor bug**: ZAPC's per-zone confidence channel is structurally ~1.0 everywhere (uninitialized
  `conf_scaling` divisor in the library, never assigned) and does not discriminate valid/invalid zones —
  don't gate on it; use the depth sentinel instead (root cause + measurements in
  `docs/deprojector-validation.md`'s confidence-channel section; see also the Phase 2.5 deferred-list
  entry above).~~ **Fixed**: The uninitialized `conf_scaling` divisor has been set to `1.0f` in the library; the confidence channel now varies dynamically.
- Bandwidth: only the raw stream crosses the wire (14,842 B/frame — 1.63× the old depth payload,
  regardless of how many output streams the PC computes). 30 Hz ≈ 445 KB/s fits CDC FS.

### Phase 2.5 (interlude) — Multi-stream color, calibrated FoV, 30 fps overlap ← **✅ Complete**

Plan: `docs/superpowers/plans/2026-07-08-phase2.5-color-fov-overlap.md`. Cleared the top three items
from Phase 2's deferred list (all detailed above, inline, where each topic is discussed): datasheet +
ZAPC-calibrated Deprojector FoV, host-side reflectance/confidence/ambient/ZAPC outputs with viewer
`--color`, and a trigger-early restructure of the raw-only firmware loop (24.6 → 27.76 fps, target ≥28
missed by 0.24 fps — sensor-serial, not hideable; see the overlap bullet above for the honest budget
breakdown). Re-verified end-to-end on hardware (this task): 60 s live soak steady 26.6-28.0 fps, 0 seq
gaps, 1 crc fail + 1 dropped flag (both connect-time, same tracked transient as Phase 2 — no new
failure mode, 2 ms settle stability tripwire passed with no stall/gap bursts across the full soak);
`--color reflectance` 15 s live check rendered the IR-shaded cloud with no fallback warning and no
traceback; stall/recover quick check (2 s read → 5 s not-reading, port held open → 10 s resume)
reproduced the established drop-policy behavior exactly (one seq gap, one `FLAG_DROPPED` on the
recovery frame, one transient CRC failure, then clean contiguous decoding with all post-recovery depth
frames finite/non-negative). `docs/protocol.md` verified unchanged — no wire change in this phase, as
planned. Left open at the time: the connect-time transient and CALIB-on-DTR-connect (both above),
carried forward unchanged from Phase 2 — the connect-time transient was later root-caused and resolved
in Phase 3 Task 6 (see the updated bullet above).

**Data-quality follow-up — reflectance fixed-pattern (FPN) + flat-field correction (2026-07-16).**
Characterized a real, sensor-locked per-zone response non-uniformity in the reflectance plane: on a
flat wall it measures **~18% of signal**, rock-stable frame-to-frame (SNR ~9), aligned to the sensor
row/column axes — not display moiré, not scene texture (per-zone SPAD sensitivity / DSS). It
contaminates the IR view, reflectance SLAM coloring, and would be *amplified* by any reflectance-based
enhancement (multi-frame super-res / relief shading). **Host correction built + shipped-disabled:**
`roomscan.flatfield` (multiplicative unit-mean per-zone gain map) applied to the reflectance plane in
`pipeline.TransformStage` (so web/panel/viewer/SLAM all get it), gated by `[viewer] flatfield_path`,
off by default; builder `tools/build_flatfield.py`; 11 tests (656 suite green); doc
`docs/flatfield-calibration.md`. **STILL OPEN — needs on-rig action:** capture a real flat-field
reference by *slowly panning* the sensor across a uniform matte wall (panning is mandatory — it
averages out scene texture; a static capture is invalid: `verify_slam.bin` → 44%-residual garbage
map), then `build_flatfield` → set `flatfield_path` → verify the grid collapses. Prerequisite for the
reflectance super-resolution / sensor-fusion-overlay work (both scoped, not yet built).

### Phase 3 — UI & runtime configuration ← **✅ Complete** (plan: `docs/superpowers/plans/2026-07-08-phase3-runtime-config-robustness.md`)

> **Status 2026-07-08:** verified end-to-end on hardware, branch `phase3-runtime-config`, 7 tasks.
>
> **Protocol** (Task 1): `frame_type` 3 = COMMAND (host→device), 4 = ACK (device→host) — additive to v1,
> no version bump. Command registry 1-6 (PING, SEND_CALIB, SET_USECASE, SET_FRAME_PERIOD_US,
> SET_EXPOSURE_MS, REINIT), result registry 0-5 (OK, UNKNOWN_CMD, BAD_PARAM, REJECTED_BINNING,
> SENSOR_ERROR, BUSY). Full spec + version-history entries in `docs/protocol.md`.
>
> **2026-07-21 addition — SET_STANDBY (cmd 7) + Ethernet command RX** (on-rig verified over UDP):
> `SET_STANDBY` (param 0=wake/1=soft/2=hard) idles the ToF VCSEL for laser-wear reduction — soft =
> `vl53l9_stop()` → FSM STANDBY (instant resume), hard = `+ platform_power_disable()` (full re-init to
> wake). Applied at the same `rs_pending` safe point as the reconfig commands; a new loop-top idle branch
> services transport + polls for the wake instead of blocking on a frame event. The host `roomscan-web`
> drives it automatically off its viewer count (debounced, `[viewer] sensor_idle_*`). **Also found + fixed:
> the Phase 5 Ethernet transport was stream-out-only — `udp_receive_callback` discarded all inbound
> payloads, so NO command (ping/usecase/reinit/standby) had ever reached the board over Ethernet.** Inbound
> UDP now feeds the same `rs_parse_command`/`rs_pending` path as CDC (`rs_poll_commands_from` factored to
> take a byte-reader; `rs_poll_eth_commands` drains an `ETH_ReadCommands` buffer). Verified: soft+hard drop
> RAW to 0 frames/3s, wake resumes ~90 fps, PING+all standby ACK'd over UDP, 0 CRC.
>
> **Firmware command channel** (Tasks 2, 4): TinyUSB CDC RX + a bounded fixed-size frame parser
> (magic/CRC-checked, malformed input dropped and counted, polled once per acquisition-loop iteration —
> never blocks acquisition). PING/SEND_CALIB need no reconfig; usecase/exposure/period/REINIT
> reconfigure the sensor at a safe point (stop → re-profile → restart) via a factored-out
> `rs_sensor_reinit()` that Task 5's recovery path reuses directly. **Binning stays fixed at 2**
> (owner scope) — `SET_USECASE` rejects any binning-4 profile with `REJECTED_BINNING` without ever
> touching the sensor.
>
> **Measured per-usecase fps** (Task 4, [HW], board reset between measurements):
>
> | usecase | id | binning | result | measured fps |
> |---|---|---|---|---|
> | AR_RANGE | 0 | 2 | OK | **32.1-32.3** |
> | AR_PRECISION (shipped compile-time default) | 1 | 2 | OK | **27.8-28.6** |
> | AF_RANGE | 2 | 4 | **REJECTED_BINNING** | n/a — no full-res (binning-2) profile exists for this usecase |
> | AF | 3 | 4 | **REJECTED_BINNING** | n/a — no full-res (binning-2) profile exists for this usecase |
>
> `SET_FRAME_PERIOD_US` applies and reads back faithfully (e.g. `50000` → ack `applied=50000`) but has
> **no observable effect on fps** in this app's always-`VL53L9_SYNC_MANUAL` design — the driver's own doc
> comment (`vl53l9.h:248`) says the field only governs autonomous sync mode. Documented as a spec-honest
> no-op (the ACK contract — apply + read back — is still met), not a bug; would need
> `VL53L9_SYNC_AUTONOMOUS` (a bigger, unattempted change) to actually govern fps. `SET_EXPOSURE_MS` *does*
> change fps measurably (5 ms → 28.6 fps, 15 ms → 25.6 fps).
>
> **Planned 2026-07-31 follow-up — high-rate/manual ranging:** The reviewed
> [implementation plan](docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md)
> turns that autonomous-sync requirement into a gated firmware/protocol/host/UI/MCP sequence. It is
> **not implemented**: work begins by reconciling the source spec's inconsistent power/range anchors,
> proving exposure granularity and fixed-map DSS-off transform compatibility, and recording the current
> 30 Hz hardware baseline. The manual payload requires protocol v2; 90 Hz is accepted only after a
> 60-second full-stream Ethernet soak with zero CRC, sequence, reassembly, or firmware-queue loss.
>
> **Device robustness** (Task 5): `rs_send_event()` emits EVENT frames
> (`SENSOR_INIT_FAIL`/`TRIGGER_TIMEOUT`/`DMA_TIMEOUT`/`SENSOR_ERROR_STATUS`) on every fault path,
> replacing reference-firmware bug #2's silent infinite spin — **bug #2 (above) is now fixed in our
> fork** for the raw-only build. Bounded recovery: up to 5 re-init attempts, 100 ms→1.6 s backoff,
> shared by both the boot path and runtime `handle_error()`; a successful recovery retransmits CALIB and
> resumes streaming (seq restarts — a documented, host-tolerated discontinuity, not an error). **Boot
> soak: 10/10** consecutive SWD resets reached streaming, both before and after the final commit
> (historical baseline ~80% first-attempt success). Live recovery exercised via a temporary,
> since-removed fault-injection hook across 9 forced faults plus 1 hook-independent natural fault, all
> recovering within ~2 s (EVENT → CALIB retransmit → seq restart → clean streaming). One anomalous
> ~100 s hang on the very first post-flash boot did not reproduce in any of the 9 subsequent runs
> (including one with an identical fault signature) — disclosed honestly, not root-caused, tracked below.
>
> **Connect-time transient — root-caused, characterized-cosmetic** (Task 6,
> `docs/connect-transient-forensics.md`): byte-exact forensics over both e2e captures found the
> *identical* signature in each — a well-formed `RAW_3DMD seq=1` header truncated ~2.8 KB short by the
> pre-existing `rs_cdc_send()` 100 ms mid-frame-abort policy, racing host-startup latency on connect (not
> stale TX FIFO residue, not a DTR race, not the separate mid-stream-reattach bug). Costs exactly one RAW
> frame, self-heals with no seq gap, never recurs — no wire or firmware fix needed. The **CALIB-on-DTR-
> connect** item (mid-stream reattach discarding up to 63 blind-start RAW frames) remains open — see the
> deferred list below.
>
> **Host — viewer keys + config persistence** (Task 7, this entry): `roomscan-view` now opens an
> `o3d.visualization.VisualizerWithKeyCallback` window wired to a `CommandClient` on the same open serial
> port (live mode only — `--replay` prints "not available in replay" for every key press, verified). Each
> key press runs on a fire-and-forget worker thread so the render loop never blocks on `send()`'s
> up-to-2 s timeout, guarded by a single busy flag that rejects a second press while one command is still
> in flight (prints `busy, command already in flight`, verified live with a rapid double `R` press).
>
> | key | command | live-session result observed |
> |---|---|---|
> | `P` | PING | `ping -> OK applied=1` |
> | `C` | SEND_CALIB | `calib -> OK applied=0` |
> | `1` | SET_USECASE 0 (AR_RANGE) | `usecase 0 -> OK applied=0`; HUD fps rose into the ~32 fps band |
> | `2` | SET_USECASE 1 (AR_PRECISION) | `usecase 1 -> OK applied=1`; HUD fps returned to the ~28 fps band |
> | `R` | REINIT | `reinit -> OK applied=0`; brief fps dip (~21 fps) then clean resume; a second `R` pressed before the first completed correctly printed the busy line instead of double-sending |
>
> `roomscan.toml` (`%APPDATA%/roomscan/roomscan.toml`, one `[viewer]` table) persists `color`/`fov_h`/
> `fov_v`/`replay_fps`/`port`. Read with stdlib `tomllib`; written by a small hand-rolled TOML emitter (no
> third-party TOML-writer dependency taken — see `host/src/roomscan/config.py`). `--save-config` writes
> the effective settings. Priority: **CLI flag > config file > built-in default**, implemented via
> argparse's `None` sentinel + `apply_config_defaults()`.
>
> **60 s soak** (Task 7, [HW], immediately following the key session above, board left on the default
> AR_PRECISION profile): observed for 131 consecutive 1 Hz HUD samples (>2× the required window) —
> steady **27.6-29.1 fps** (one transient 21.0 fps sample during the preceding REINIT's settle, not part
> of the steady-state band), **0 new seq gaps** and **0 new CRC failures** for the entire session (the
> single CRC failure and the stable `raw-skip 44` present throughout are the pre-existing, already-tracked
> connect-time transient and mid-cycle-attach behavior, not new occurrences).
>
> Suite: **97 passed** (73 baseline + 15 `config.py` tests + 9 viewer-key/command-routing tests).
>
> **Deferred / honestly open** (not blockers for calling Phase 3 done):
> - **CALIB-on-DTR-connect auto-fix** (device aborts any in-flight frame and sends CALIB immediately on
>   DTR rising, via `tud_cdc_line_state_cb`): evaluated Task 6, needs new synchronization between a
>   TinyUSB callback context and the main loop's send/trigger state — not small/safe enough to land in
>   Phase 3. Specced as a Phase 3/4 follow-up. `SEND_CALIB` (the `C` key / `roomscan-ctl calib`) is the
>   shipped manual mitigation for the same blind-start problem.
>   **Implemented 2026-07-30, UNVERIFIED (BUG-005).** The synchronization turned out to be reentrancy,
>   not concurrency: TinyUSB dispatches class callbacks from `tud_task()`, not the ISR (verified in the
>   vendored `usbd.c`), so one volatile flag set in the callback and consumed at the loop's existing
>   per-frame safe point is enough. Cannot be exercised on the current headless host — the board's
>   USB_USER port is not attached to it (`lsusb` shows only the ST-LINK), so `CAFE:4001` never
>   enumerates and there is no DTR to raise. Verified only that it leaves the Ethernet path intact
>   (30.3 fps, 0 CRC, 0 gaps); see BUGS.md BUG-005 for the verification recipe on a USB-attached box.
> - **`SET_FRAME_PERIOD_US` is a spec-compliant no-op** in this app's always-manual-sync design (see the
>   fps table above) — the command does exactly what the protocol promises (apply + read back), it just
>   doesn't control fps here; would need an autonomous-sync redesign to matter.
> - **One 100 s post-flash boot-recovery hang** (Task 5) was observed once, did not reproduce in 9
>   subsequent identical-scenario runs, and was not root-caused — tracked as a low-confidence anomaly,
>   not a confirmed defect.
> - **AF_RANGE / AF usecases are unusable** at the project's fixed full-resolution binning-2 constraint —
>   an owner-scoped design decision (binning stays fixed at 2 for all of Phase 3), not a bug.

Host→device **control channel** to set usecase / binning / active streams at runtime. Recording/playback
and config persistence host-side.

- **Assumption corrected by the Task 7 capture:** the transform library's `controls` are only
  `bypass-*` algorithm toggles + `calib-buffer` + `cover-glass` — there is **no runtime usecase or
  binning control**. Usecase/binning are sensor-profile settings applied before init
  (`vl53l9_utils_set_profile`), so runtime reconfiguration means a full stop → re-profile →
  re-prepare → restart cycle on the device (which also forces the teardown path of reference bug #6 to
  work). Plan Phase 3 around that, not around a transform control write.

- Control frames reuse the same header (`frame_type` = command / ack); device replies with an ack frame
  carrying the applied config — the host never assumes.
- Reconfiguration path forces the teardown/re-prepare sequence to actually work (see reference bug #6):
  stop ranging → free/resize buffers (binning changes both raw and output sizes) → re-set capabilities →
  `transform_prepare` → restart. Watch for leaks in the opaque transform handle across cycles.
- CDC RX side appears here for the first time — until now the device only transmits.

### Phase 3.5 (interlude) — GUI control panel + 2D IR monitor ← **✅ Complete**

Plan: `docs/superpowers/plans/2026-07-09-phase3.5-gui-panel.md`, branch `phase3.5-gui-panel`. Owner
elected this next (2026-07-09), deferring Phase 4 (IKS4A1). Replaces the classic keyboard-only Open3D
window with an `open3d.visualization.gui` control panel — `roomscan-panel` (or `roomscan-view --panel`);
the classic `roomscan-view` window stays the default. Presentation layer only: `TransformStage`,
`CommandClient`/`CommandDispatcher`, `Deprojector`, `sources`/`pump`, `config`, `Stats`/`StreamDecoder`
are all reused unchanged (no wire change; `docs/protocol.md` untouched).

- **Panel groups:** Status (fps/frames/gaps/drops/crc/raw, usecase+color), Device (Ping/CALIB/Reinit
  buttons, usecase combobox, debounced exposure slider — all via the shared `CommandDispatcher`, so keys
  and buttons run one busy-guarded off-thread dispatch path), View (color mode, point size, background,
  reset-view), **IR Monitor** (scope addition — a live 2D reflectance image, nearest-neighbor upscaled
  from the 54×42 zone grid, gray/turbo, per-frame auto-range with a freeze toggle, "IR unavailable"
  placeholder on depth-only replay), Capture (mid-stream Record via a `Recorder` tee; replay pause +
  fps slider), Events (scrolling device-EVENT / command-result log via an in-process `LogBus`).
- **Threading:** render on the GUI main thread via `Window.set_on_tick_event` (polls the reader's
  latest-wins slot; labels/IR/log at ≤4 Hz); reader thread + command worker threads keep serial writes
  off the reader per the standing contract.
- **Support layer:** five file-disjoint TDD'd modules — `ir_image.py` (reflectance→RGB), `logbus.py`,
  `config.py` (+`point_size`/`ir_colormap`/`ir_freeze_range`/`panel_width`), `sources.py` `Recorder`,
  `control.py` `CommandDispatcher`.
- **Owner-requested follow-ups (2026-07-09):** (1) **Near contrast** (`roomscan/shading.py`) — for
  the person-in-front-of-wall setup, spends more of the colormap on close targets so facial relief
  stands out: `window` (default, greys past a cutoff), `emphasis` (near gamma), `equalize` (histogram),
  `off`; View-group combobox + adaptive slider. (2) **Point size** slider widened 1→20, default 3→5, to
  close the inter-zone gaps. (3) **Modal help** (`Help`/`H` → `gui.Dialog`). (4) A **headless snapshotter**
  `tools/panel_view.py` (Pillow, CPU) that renders the panel to a PNG — Open3D Filament offscreen fails
  on a locked box (`EGL Headless not supported`) — so the panel can be *seen* without a display.
- **Verified:** host suite 162 passed, ruff clean; headless `run_one_tick` smoke against
  `captures/e2e_p2.bin` rendered 194 frames (2257-pt cloud), reflectance present, IR auto-range + freeze,
  all callbacks functional, reader thread joins clean. The formerly-open live on-hardware run is
  **✅ closed** — merged 2026-07-09 (branch `phase3.5-gui-panel`, since deleted) and the panel has been
  the primary live surface for all Phase 4 hardware work since (IKS4A1 bring-up, sensors group,
  yaw-fusion checks), including per-frame IR rendering (`7006dc5`-era perf fix on the metrics-HUD
  branch). Known cosmetic: an Open3D filament-teardown "Fatal Python error" can print at interpreter
  exit (post-functional).

### Phase 3.6 (interlude) — Web UI Migration (FastAPI + Three.js) ← **✅ Complete**

Plan: `web_app_migration_plan.md`. Owner elected to deploy the visualizer to a headless server accessed remotely via Tailscale. The native Open3D UI fails in a locked/headless environment without a display.

- **Architecture (Option A: True Single Codebase):** `roomscan-web` entry point spins up a FastAPI/Uvicorn server running the exact same `TransformStage` reader thread as the desktop viewer. The server exposes a `/ws` WebSocket endpoint that streams the transformed point cloud and color data to the browser as a packed `Float32Array`. 
- **Frontend:** ~~A premium glassmorphic web UI (`index.html`, `app.js`) running Three.js handles the point cloud rendering on the local browser's GPU. Control elements (Ping, Reinit, Usecase selection) send JSON commands back through the WebSocket to the `CommandKeyState` dispatcher.~~ **Superseded by Web Phase 1 (below, 2026-07-16):** the monolithic `app.js` is retired in favour of 7 vanilla ES modules, and the single binary point-cloud message is now a multiplexed, tagged protocol. The single-`app.js` / single-message shape described here was the minimal first cut.
- **Bandwidth:** The WebSocket streams processed points rather than RAW data. With decimation running, bandwidth sits comfortably below Tailscale's limits.
- **Verified:** Dependencies added to `[web]` optional group. Ran headless test successfully against `synthetic.bin`; the server boots on port 8000 and serves the static Three.js payload.

### Web replacement of `panel.py` — a 5-phase program (Three.js web app supplants the Open3D desktop panel)

Owner direction (2026-07-15): the web app **fully replaces** the ~3600-line Open3D `panel.py`, delivered in
phases — (1) core real-time instrument, (2) sensors (IMU/env streams 9/10), (3) recording & playback,
(4) SLAM mode, (5) settings persistence + retire `panel.py`. **All five phases are done (2026-07-16):**
`roomscan-web` is now the primary, supported UI and `panel.py` is **deprecated in place** — kept only as
legacy for a local-display box (it can't run on the GPU-less headless host), no longer imported by the web
server, and it prints a deprecation notice on launch.

**Parity gap found after the fact (2026-07-28, BUG-026):** "fully replaces" was not quite true — **gravity
alignment of the live view was never ported**. It existed only in `panel.py` (IR pane via `ir_gravity_rot`,
orbit-mode cloud via `T_WORLD_TO_CV @ R @ T_CV_TO_BODY`), so booting the board upside down rendered both the
IR pane and the point cloud upside down in the web app. The orientation matrix was already on the wire as the
`sensor` message's `rot`, but only the 2D gizmo consumed it. Now fixed with continuous full alignment
(desktop-panel parity, owner's choice), plus a coherence-gated **display-only** smoother, because rotating
the cloud by the raw quat swings sensor noise on the scene's lever arm. That in turn surfaced BUG-027
(firmware aliasing) — the two are worth reading together. Wire semantics in `docs/web-protocol.md`; when
auditing the rest of the panel's feature set for other unported behaviour, this is the precedent that it can
happen silently. **And the precedent has a second lesson (2026-07-29):** porting the panel's behaviour
*verbatim* was itself not enough — the panel rolled the IR pane in 90° snaps, so once the cloud got
continuous alignment the two disagreed by up to 45° and the pane ignored moderate tilts entirely. Parity
with a deprecated implementation is a starting point, not the specification; check that the ported
behaviour is still coherent with whatever the new code does alongside it. Fixed by splitting the roll into
the server's quarter-turn snap plus a client-side `ir_roll_deg` residual (BUG-026 follow-up).

**"Showcase" is not a separate phase (owner clarification, 2026-07-16):** the earlier plan listed a 6th
"showcase mode" phase, but Showcase was only ever **another name for SLAM mapping** — the record → build →
save flow — a naming artifact from earlier in the project. The desktop panel already dissolved it ("SLAM
absorbs the former Showcase record→process→reveal flow — no separate Showcase concept in the UI", Phase 6
below), and the web app **already delivers it** across Web Phases 3 (record + load/replay a capture) and 4
(SLAM builds the full map + **Save** the full-res mesh/trajectory). So the web plan is **5 phases, not 6**;
the only remaining work is settings persistence + retiring `panel.py`. (The lone desktop-Showcase nicety
not yet in the web app is a *guaranteed-every-frame* offline post-process with a sharpening "reveal";
Web Phase 4's replay-fed SLAM already processes every frame when a capture is replayed at ≤30 fps, so this
is at most a small option inside SLAM mode, not a phase — fold it into Phase 5 only if the owner wants it.)

#### Web Phase 1 — Core Real-Time Web Instrument  ← **✅ Complete (2026-07-16)**

Spec: `docs/superpowers/specs/2026-07-15-web-phase1-core-instrument-design.md`. Host-side only — no wire-protocol
or firmware change. Confined to `host/src/roomscan/web.py` + `host/src/roomscan/static/` + `host/tests/test_web.py`.

- **Frame-stealing bug fixed:** the old per-connection `slot.get_nowait()` loops (two tabs stole each other's
  frames) are replaced by a **single asyncio broadcast task** fanning identical frames to all clients; reuses
  `panel._run_reader` (no forked reader). Regression-tested with two concurrent `websockets` clients.
- **Multiplexed `/ws` protocol:** tagged little-endian binary — POINT_CLOUD (tag 1), IR_IMAGE (tag 2) — plus
  `metrics`/`event`/`log`/`cmd`/`state` JSON, split client-side by `typeof event.data`.
- **Four user-facing features:** working device controls with visible feedback (toast + event-log console),
  runtime color modes (depth/reflectance/confidence — `stage` computes all three, switch is pure server state),
  live IR monitor pane, metrics HUD (VIEW fps client-side + Device fps + per-stream rate/jitter + link bandwidth).
- **Frontend:** 7 vanilla ES modules (`ws`/`scene`/`ir`/`hud`/`log`/`controls`/`app`), no build step, importmap +
  vendored three.js; one-way state flow through the `ws.js` pub/sub hub keeps multi-tab state in sync.
- **Verified:** 26 backend tests (`test_web.py`); full host suite **606 passed, 1 skipped**. Driven end-to-end in
  headless Chrome (SwiftShader) against a room-scan replay — all four features confirmed on screen.
- **Caveat (data, not code):** dual-stream recordings (RAW_3DMD + redundant DEPTH_ZF32 passthrough) intermittently
  fall the IR pane / reflectance colour back to depth, because the DEPTH frame lands last in the latest-wins slot.
  Live production streams are RAW-only, so unaffected; a "prefer-richest-frame" tweak is a future option.
- **Deferred to Web Phases ~~2~~ 3–5:** recording/playback UI, SLAM trajectory+mesh
  (adds a MESH binary type + a top-bar mode switch, placeholder reserved), settings persistence, and retiring
  `panel.py`. Also not yet carried over: exposure slider, rotate-90 / near-contrast view options. *(Showcase,
  once listed here, was a misnomer for SLAM mapping — see the plan header; delivered by Phases 3–4.)*

#### Web Phase 2 — Sensors (IMU/env streams 9/10)  ← **✅ Complete (2026-07-16)**

Spec: `docs/superpowers/specs/2026-07-16-web-phase2-sensors-design.md`. Host-side only — no wire-protocol
or firmware change. Confined to `web.py` + `static/` (new `sensors.js`, extended `app.js`/`index.html`) +
`test_web.py`.

- **Reuse, don't reimplement:** `web.py` now builds the same `SensorState` + `YawFusion` + `MagCalibration`
  the desktop panel does (`panel.py:525-541`) and feeds it through the shared reader by filling the
  `_run_reader(state=…)` slot Web Phase 1 left as `None`. The message builder calls the existing `sensors.py`
  math (`quat_to_matrix`, `T_WORLD_TO_CV`, `T_CV_TO_BODY`, `absolute_heading`, `AXIS_CONVENTION`) — nothing in
  `sensors.py`/`magcal.py`/`protocol.py`/`panel.py` was edited.
- **Protocol:** one new JSON message `{"type":"sensor", …}` on the existing `/ws` (no new binary tag — the
  payload is tiny): server-computed display-rotation `rot` (9-float row-major `T_WORLD_TO_CV @ R @ T_CV_TO_BODY`),
  `heading` (drift-free `absolute_heading`, calibrated mag when `mag_cal.json` present), pressure/temp/mag,
  `fusion` status, and 256-sample pressure/temp history. Broadcast at 15 Hz from the single broadcaster task;
  **silent until a 9/10 frame arrives** (`build_sensor_message` returns `None`), so ToF-only sessions add no traffic.
- **Frontend:** new `sensors.js` module (2D-canvas — no second WebGL context, keeps the headless SwiftShader box
  cheap) draws an orientation gizmo, a tilt-compensated compass (0=N clockwise), and pressure/temp sparklines,
  appended to the left rail per the Phase-1 layout plan. Streams 9/10 also light up the metrics HUD's IMU/Env
  rows for free (`metrics.py` already labels them).
- **Verified:** 4 new backend tests (`build_sensor_message` shape/units + display-transform equivalence + reader
  integration); full host suite **610 passed, 1 skipped**. Driven end-to-end in headless Chrome against a
  synthetic depth+IMU+ENV replay — gizmo, compass (heading tracked frame-exact), and sparklines all confirmed on
  screen; server log clean.
- **Not carried over (deferred):** world-frame point accumulation + baseline-yaw reset (revisit with SLAM,
  Web Phase 4), SHT40 humidity (unstreamed), on-rig mag-recalibration UI.

#### Web Phase 3 — Recording & Playback (full-remote)  ← **✅ Complete (2026-07-16)**

Spec: `docs/superpowers/specs/2026-07-16-web-phase3-recording-playback-design.md`. Host-side only — no
wire-protocol or firmware change. Confined to `web.py` + `static/` (new `capture.js`) + one additive
`FileSource(start=)` param in `sources.py` + `test_web.py`. Owner picked **Full remote** over
desktop-parity: the app runs remotely on the headless box, so the operator browses and loads captures
**from the browser**, not by relaunching with `--replay`.

- **Runtime source-swap (the hard part):** a new `SessionController` owns the reader-thread lifecycle so
  the source can be swapped **live↔replay at runtime** without disturbing the single broadcaster or the
  shared slot. The live device is opened once and kept behind a `_NoCloseSource` proxy (pump's
  `finally: close()` can't kill it), so **Go Live re-uses it instantly** (no 5 s UDP re-probe). Swaps run
  off the event loop via `asyncio.to_thread`, serialized by a lock; the reader body is the **unchanged**
  `panel._run_reader`.
- **Four capabilities:** (1) **Record** a live session → `captures/web_<ts>.bin` with live elapsed/bytes
  status (disabled in replay); (2) **capture library** — server lists `captures/*.bin`, browser picks;
  (3) **load at runtime** → reader swaps to replay, `Go Live` returns; (4) **transport** — Pause/Resume,
  speed ×0.5/×1/×2/Max, Loop, and a **seekable progress bar**. Seek re-injects the governing CALIB (from a
  CRC-verified capture index of frame offsets/seqs/calib-spans) so scrubbing into a RAW capture isn't blank.
- **Protocol:** two new `/ws` JSON messages — `session` (mode/source/recording/playback, broadcast on change
  + metrics cadence) and `captures` (library list) — plus inbound `record`/`list_captures`/`load_capture`/
  `go_live`/`transport`. No new binary tag. Commands in replay report "not available in replay" (no device
  round-trip). Frontend: new `capture.js` (8th ES module), all state driven from the server echo (one-way
  flow), no build step.
- **Verified:** 20 new backend tests (index/sanitize/list/session helpers + `SessionController` swap, record
  gating, live-record tee, seek); full host suite **625 passed, 1 skipped**. Driven end-to-end in headless
  Chrome against a synthetic capture library — record-disabled-in-replay, library listing, runtime load/swap,
  pause (position frozen), speed (Device FPS tracked the ×-setting), loop, and seek all confirmed on screen.
- **Deferred (at the time — Web Phases 4–5):** SLAM trajectory+mesh (**done, Phase 4**), then settings
  persistence + retiring `panel.py` (Phase 5, final). Serial-staleness on Go Live is mitigated by a
  best-effort `reset_input_buffer`; UDP self-heals via keepalive.
- **`/ws` protocol reference:** the full app protocol (binary tags + JSON messages, in/out) across Web Phases 1–3
  is now indexed in `docs/web-protocol.md` — Phase 4's trajectory/mesh messages hook in there (it also lists the
  invariants: one-way echo, validate untrusted inbound, server-side math, off-loop blocking work).
- **Post-recording naming (owner ask, 2026-07-29):** every stopped take now pops a skippable "Name Recording"
  modal in `capture.js`, prefilled with the auto `web_<ts>.bin` name. Save sends a new inbound `rename_capture`
  (`SessionController.rename_last_recording` / `sanitize_new_capture_name` in `web.py`); success/failure is read
  back off a new `session.recording.last_name` field rather than a dedicated ack, since the server already owns
  collision/validity checks. Skip/Esc/backdrop-click leaves the auto name — never blocking, the file is on disk
  either way. Verified end-to-end in headless Chrome (open-on-stop, save, collision rejection, skip); full host
  suite 961 passed, 1 skipped.
- **Discoverable Go Live (owner ask, 2026-07-29):** returning to live view only ever worked by clicking the
  "● Live device" row buried in the Source list — easy to lose once scrolled past mid-playback, and the owner
  hit exactly that dead end. Added a dedicated **"● Go Live"** button to the transport panel itself (`capture.js`
  / `index.html`), sending the same existing `go_live` message the row already did (`SessionController.switch_to_live`,
  unchanged); gated on `session.has_live` like the row. Verified in headless Chrome (renders, correctly disabled
  with no live source); full host suite 971 passed, 1 skipped.

#### Web Phase 4 — SLAM mode  ← **✅ Complete (2026-07-16)**

Spec: `docs/superpowers/specs/2026-07-16-web-phase4-slam-design.md`. Host-side only — no wire-protocol or
firmware change, **and no edits to `slam/`**. Confined to `web.py` + `static/` (new `slam.js`, extended
`scene.js`/`app.js`/`index.html`) + `test_web.py`. Owner decisions: **GPU-accelerated** and **include a web
Save button**.

- **Reuse, don't reimplement:** a new `SlamRunner` in `web.py` wraps the desktop's own off-thread pipeline —
  `make_slam_worker` (local **CUDA:0** worker; remote `SlamService` if `[slam] backend=remote`) + `MeshPrep` —
  **unchanged**. Fed from the broadcaster only while `mode == "slam"` (no GPU burned in real-time), latest-wins
  so a slow frame never backs up the loop; all enter/leave/reset/save run off the event loop.
- **Compute is the LOCAL GPU** (discovered this session): the Proxmox host passes an **RTX 2000 Ada** through
  to the container, Open3D 0.19 reports CUDA, SLAM runs in-process at **~7 ms/frame** — no remote container
  needed. (The `headless-host-deployment` "no GPU" note is superseded for compute; X/VNC GL + the test-Chrome
  WebGL are still software.)
- **Protocol:** new binary **MESH (tag 3)** — a flattened `MeshPacket` (wall/non-wall verts+colors+tris + floor
  grid, throttled) — plus a `slam` JSON per frame (pose, server-computed follow eye/center/up, downsampled
  trajectory tail, fitness/rmse/tracking/frames) and a `saved` list; inbound `set_mode`/`slam_opt`/`save`.
  Save writes the **full-res** `mapper.mesh()` + TUM trajectory to `results/web_<ts>.ply`/`.tum`, downloadable
  from a `/results` mount. All in `docs/web-protocol.md`.
- **Frontend:** new `slam.js` (9th module) renders the mesh (unlit vertex-colored — shading is baked
  server-side by `MeshPrep`) + trajectory + head marker into `scene.js`'s **single** Three.js context, and
  drives a follow camera; top-bar Real-Time↔SLAM switch, SLAM control group (trajectory/walls/follow + Save +
  saved-maps list), and a SLAM HUD row. All state driven from the server `state`/`slam`/`saved` echo.
- **Verified:** 12 new backend tests (`pack_mesh` round-trip, `slam` shape + traj bound, `sanitize_result_name`,
  `list_results`, `SlamRunner` lifecycle with fake worker/meshprep, save + empty-map raise); full host suite
  **637 passed, 1 skipped**. SLAM pipeline de-risked on GPU (`roomscan-slam --device CUDA:0`, 329-frame stream-9
  capture → lost=0, ~30k-vertex map) and driven end-to-end in headless Chrome against `captures/verify_slam.bin`:
  mode switch, live mesh build (Tracking OK, Fitness 0.85, RMSE ~11 mm, frames climbing), follow camera, walls
  Split/Solid, and Save → a downloadable `.ply`/`.tum` — all confirmed on screen.
- **Verification-data note:** SLAM needs a **stream-9 (IMU quat) capture** — the older
  `recordings/2026-07-08-room-scan.bin` predates IMU and loses tracking (empty map). `captures/verify_slam.bin`
  (recorded live this session, gitignored) is the fixture.
- **Remaining web work (Web Phase 5, final):** ✅ **done** — see below. (Showcase is **not** a separate phase — it
  was a misnomer for SLAM mapping, already delivered by Phases 3–4; see the plan header.)

#### Web Phase 5 — settings persistence + retire `panel.py`  ← **✅ Complete (2026-07-16)**

Host-side only — no wire-protocol, firmware, or `/ws`-message change. Owner decisions (2026-07-16):
**deprecate `panel.py` in place** (don't delete) and **persist to the shared `roomscan.toml` [viewer]` table**
(one config across web + desktop).

- **Settings persistence.** The web UI's six display prefs — `color`, `ir_colormap`, IR `freeze`, and the three
  SLAM display toggles (`trajectory`/`walls`/`follow`) — now live in the same `roomscan.toml` [viewer]` table the
  desktop viewer/panel already used. Three new flat fields were added to `ViewerConfig` (`slam_trajectory`,
  `slam_walls`, `slam_follow`); `web.ui_from_config` seeds `UiState` on boot (validating each value, falling back
  to the UiState default on anything unrecognized), and `web._persist_ui` writes each runtime change straight back
  — it **re-loads the file first** so a concurrent editor's non-web fields survive, and swallows write errors with
  a warning (a color click must never crash on an unwritable config dir). Persistence is a no-op when no config is
  attached, so the socket-free unit tests are unaffected.
- **`mode` is deliberately NOT restored.** The SLAM worker arms lazily on the first `set_mode slam` (no GPU burned
  until then), so a server restart always comes up in real-time regardless of the last session — restoring into
  SLAM would silently spin up the GPU on launch. The web app never writes the [viewer]` `mode` field; the desktop
  panel keeps owning it.
- **Behaviour note:** a fresh web install (no config file) now adopts the shared `color` **default**, which is
  `reflectance` (the desktop default, falls back to depth when the plane is absent) — not the old web-only `depth`.
- **`panel.py` deprecated in place.** The three GUI-free helpers the web server borrowed from the panel
  (`_run_reader`, `_Pacer`, `follow_camera_target`) plus their follow-camera constants moved to a new neutral
  **`reader.py`**; `panel.py` re-imports them (so `panel._run_reader` and its tests still resolve) and `web.py`
  now imports from `reader.py` and no longer imports the panel module at all. `roomscan-panel` and
  `roomscan-view --panel` print a one-line deprecation notice on launch; the panel is kept only for a
  local-display box (it can't run on the GPU-less headless host). `roomscan-web` is the primary, supported UI.
- **Verified:** 8 new backend tests (config field defaults + TOML round-trip, `ui_from_config` valid/invalid/mode
  mapping, `apply_ui_to_config` preserves desktop-only fields, `_persist_ui` write + no-op, `set_color` handler
  end-to-end); full host suite **645 passed, 1 skipped**. Driven end-to-end against two real `uvicorn` servers: a
  `/ws` `set_color` wrote `roomscan.toml`, and a **restarted** server seeded a fresh client's very first `state`
  message from it. No frontend change — the existing `state` echo already drives the UI, so persisted values reach
  the browser through the unchanged connect handshake.

#### Web Phase 6 — UI corrections pass  ← **✅ Complete (2026-07-31)**

Owner-driven round of UI fixes after living with the Live/View consolidation. Six commits
(`df28686`, `2399afa`, `61b96ac`, `66ea675`, `2dbcbcb`, `7d5ffe6`), **1327 passed / 1 skipped**.

- **Explain the instrument.** Drops/Gaps/FPS/Bandwidth and every per-stream row now carry
  tooltips (keyed on `stream_id`, not `label` — `metrics.py` maps two ids onto "ToF"). The Fusion
  row names a remedy for each `fusion_key`, not just the fault. `body { user-select: none }` was
  making every diagnostic uncopyable; `#diag-log`, the event log, the quat and the jitter table opt
  back in.
- **Declutter.** The right-rail IR group duplicated controls the IR card already had inline — gone;
  its × now collapses into the squircle rail rather than hiding with no way back. The orientation
  decomposition picker is gone and the card is pinned to **World**, which also fixed the axis
  labels: World's slots are triad roll, boresight tilt from horizontal, and absolute magnetic
  heading, so "Pitch"/"Yaw" named the wrong quantities. A one-time migration adopts the new labels
  for configs still carrying the old default (`_persist_ui` writes every field on any change, so the
  stale default would otherwise shadow the new one forever).
- **The squircle rail** is now a map of every panel — permanent buttons, bright when open and dim
  when collapsed, with matching icons injected into each card title from the same `CARD_ICONS` table.
- **A real device model.** `devicemodel.js` is the single source for the owner's 5.5 × 3 × 2.5 in
  block (dark grey / white / blue through the depth, camera on the blue face), shared by the mag-cal
  3D view and a new 2D painter for the Sensors gizmo (2D canvas, not a third WebGL context — this
  host is llvmpipe). `MOUNT_ROTATION` is a 180° turn about the boresight, derived from World roll
  being `triad_roll_deg` of body +X and corroborated by the live rig reading Roll 179.66°.
- **Mag-cal is one view, not two** — the world-fixed steering framing carrying the hero's cell
  styling. This *re-accepts* a tradeoff `magcal3d.js` had explicitly rejected (a body-fixed shell
  under a room camera has its holes orbit at hand speed); the mitigation is that the ghost and the
  leader line become the aiming instrument. `heroCam` is kept as the no-quaternion fallback, or a
  ToF-only session renders an empty canvas. The behind-camera split is now per-pose: 0.062 ms per
  recompute under a synthetic 90 °/s tumble.
- **Elevation replaces pressure** — feet, with hPa beside it and a Δ datum button (persisted).
  Sea-level reference from Open-Meteo, the app's **first outbound third-party request**: stdlib
  `urllib` in a thread, one GET per 30 min, cached, with a visible `msl_source` so a fallback is
  reported rather than hidden. EMA'd at 6 s because the barometer is 267 mm RMS per frame (BUG-037);
  the sparkline is left unsmoothed on purpose.
- **Resource headroom during Live SLAM.** `build_metrics_message` had hardcoded `resources: None`.
  Now wired, plus system-wide RAM/CPU (psutil) and **device-wide** VRAM via the ctypes NVML in
  `slam.gpumem` — `pynvml` is not installed, so every per-process GPU field would have been null.
  The VRAM figure deliberately exceeds `nvidia-smi memory.used` (370 vs 18 MiB) because NVML counts
  driver-reserved memory, which for a headroom gauge is spoken for. Also surfaces the **TSDF block
  gauge** — the ceiling that silently killed 18% of a real sweep (BUG-035) — sampled at the metrics
  rate, since `block_usage()` is a device sync on CUDA.
- **Auto-record on entering Live SLAM**, because a live scan is unrepeatable (the same reasoning
  that kept its one-shot Save, BUG-043). A manually started take is tracked separately and never
  stopped by a display switch.
- **Oscillate orbit** — a triangle wave sweeping ±N about the start azimuth. Two traps: the azimuth
  wraps at ±π (unwrapped per-frame steps, or any amplitude ≥ 180° latches — measured 0 reversals in
  1200 frames), and direction must not live in the sign of `autoRotateSpeed`, which the `state` echo
  overwrites on every unrelated setting change (measured: the return leg vanished entirely).
- **Live is one Record button; View is a real capture browser** — thumbnails, metadata, rename,
  multi-select delete, and a first-class **Preview** display mode. The Build action now opens a
  confirmation dialog with capture/frame/compute details and the active Detailed estimate (honestly
  marked uncalibrated until CUDA timing is measured); it then loads the capture, starts the build,
  and enters Detailed. Playback starts at **1×** by default. Thumbnails are a top-down sketch from
  ~40 sampled frames rotated by
  stream 9; **it is not a map** (no translation estimate, so every frame shares one origin) and the
  tooltip says so. 51 ms/file across the real 1.49 GiB library, reading 0.161% of a 408 MB capture.
  Served over `GET /thumb/{name}` rather than a `/ws` tag so `<img loading="lazy">` gives paced
  fetching and caching for free. "Area covered" is a floor-projected footprint, not mesh surface
  area, which would score a corridor above a large room. **Turbo/Gray now applies equally to Point
  cloud, SLAM, and Detailed**; the latter two re-map their presentation mesh only, never the
  reconstruction or sidecar.

**Bugs found and fixed here:** **BUG-047** (`id="btn-restart"` named two buttons — playback Restart
was dead *and* "Restart Server" fired a transport restart first) and **BUG-050** (recording
`elapsed_s` was `time.time() - time.monotonic()`; a 90 s take reported 1.78e9 s, and it survived
because the tests asserted the value was a *float*).

**Open, non-blocking:** the narrow-viewport overlap probes (1280×800 / 1100×560 / 820×700) were not
run — `ui_screenshot`'s width/height did not resize the viewport, so only 1600×1000 is measured
(0 overlaps with browser + preview + Playback open). `MOUNT_ROTATION` and the merged mag-cal view
both want an owner-in-hand confirmation. On-demand **Detailed** builds did not complete during
verification (7+ min on a 759-frame capture) — pre-existing, worth its own look.

### Phase 4 — Integrate X-NUCLEO-IKS4A1  ← **✅ Complete** *(swapped with Ethernet 2026-07-09, owner decision — sensors next)*

> **Status 2026-07-10:** verified on hardware — the full stack (ToF depth + SFLP orientation +
> environmental) streams together at **27.85 fps, 0 CRC failures, 0 seq gaps** (no measurable fps cost
> vs the ToF-alone 27.76-28.6 band).
>
> - **Bus** (per the HUB1 design below, plus one fix the plan missed): PartID-keyed multi-device ENTDAA
>   (ToF `0x0102`→`0x52`, LSM6DSV16X `0x0070`→`0x50`). Stacked, the IKS4A1's NXS0108 auto-direction
>   translator can't pass 12.5 MHz I3C push-pull, so `rs_assign_dynamic_addresses()` slows the PP clock
>   **for ENTDAA only** (ranging stays full-speed) — ToF enumeration went from intermittent to 100% stable (105/105 passes).
>   Second independent fix: sensor-hub env sensors needed J4/J5 = 5-6 **only** and the LPS22DF barometer
>   at `0x5D` (SA0=1 on this board). Full history: `docs/iks4a1-stacking.md` → "RESOLVED (2026-07-10)".
> - **Streams** (protocol v1 rev 2026-07-09, additive): **IMU_QUAT (9)** — SFLP game-rotation quaternion,
>   **4×float32, not the fp16 the research doc predicted** (the bullet below is kept for the record);
>   **ENV (10)** — pressure/mag/temp via the LSM6DSV16X's I2C sensor hub. Both emit **one sample per ToF
>   frame** (~28 Hz wire cadence; SFLP itself runs at 480 Hz on-chip) — a deliberate simplification of
>   the "independent IMU frames at native rate" bullet below; revisit in Phase 6 only if SLAM measurably
>   wants denser orientation samples.
> - **Host:** Sensors panel group (orientation gizmo, tilt-compensated compass, pressure/temp
>   sparklines); 9-axis magnetometer yaw-drift fusion (`docs/yaw-fusion.md`, PR #2) with
>   ellipsoid-fit mag-calibration CLI. Suite: **240 passed**.
>
> **Open follow-ups (not blockers):**
> - **[RESOLVED 2026-07-10] Visualizer camera model + world-space accumulation**: replaced the 3D axes gizmo with a 3D camera model entity, transformed point cloud and mesh data into the fixed world frame using the IMU orientation. Absolute gravity tilt (down direction of gravity from accelerometer) is preserved by only zeroing yaw during baseline resets, and persistent accumulation is controllable via a `self.persistence` configuration flag (defaulting to False). Added Clear UI/key controls.
> - **[RESOLVED 2026-07-10] On-rig mag calibration + `AXIS_CONVENTION` verification**:
>   calibration generated `mag_cal.json` (residual std/mean < 0.02, field_ut ~49.87 uT), `AXIS_CONVENTION` verified and set to `np.diag([1.0, -1.0, -1.0])` (representing `[x, -y, -z]`), and visual yaw-as-roll mapping resolved in `gizmo_pose`.
> - **SHT40 humidity (and the remaining IKS4A1 sensors) are not streamed** — ENV carries
>   pressure/mag/temp only. Add a field only when a consumer exists (protocol-change checklist applies).
> - **Metrics HUD** (draft PR #1) — presentation-layer only, no wire change; review and merge.

IMU (LSM6DSV16X hardware SFLP quaternions) / mag / baro drivers; fuse readings into the payload with
hardware timestamps. New streams = new `stream_id`s + a version bump per the protocol rule.
*(Older docs/reports may still call this "Phase 5" and Ethernet "Phase 4" — the swap reversed the
numbers; content is unchanged.)* The USB CDC link carries the added IMU/env traffic easily (~KB/s on
top of 445 KB/s raw), so nothing here waits on Ethernet.

- **Bus topology — resolved: HUB1-only native-I3C** (`docs/iks4a1-stacking.md` → "Resolved — HUB1
  native-I3C"; plan `docs/superpowers/plans/2026-07-09-iks4a1-hub1-multidevice-i3c.md`). The naive
  shared-I3C1 approach (IKS4A1's legacy-I2C sensors alongside the ToF) failed at the configured
  12.5 MHz push-pull speed once stacked (ENTDAA at ~1.85 MHz was fine; PP wasn't). The fix: jumper the
  IKS4A1 to **HUB1 only** (J4/J5 → `HUB1_SDx`/`HUB1_SCx`), so only the **LSM6DSV16X** — a genuine MIPI
  I3C v1.1 target (DS13510 §5.2) — shares I3C1 with the ToF, both running native I3C at the full
  12.5 MHz PP speed. A fork-owned `rs_assign_dynamic_addresses()` in
  `firmware/scanner-stream/Src/vl53l9_app.c` enumerates both ENTDAA responders and assigns each a
  distinct address keyed on **PID.PartID** (MIPIID is degenerate between the two devices): ToF
  (PartID `0x0102`) → `0x52`, LSM6DSV16X (PartID `0x0070`, WHO_AM_I `0x70`) → `0x50`, clear of every
  IKS4A1 static address. Verified on hardware with both boards stacked: native CDC port reappears
  (previously the boot hung), **0 CRC failures, 0 seq gaps, 28.24 fps interval / 28.13 fps wall-clock**
  over a 15 s capture (422 RAW + 7 CALIB frames). **Trade-off:** HUB1-only routing disconnects the
  environmental sensors (LPS22DF baro, LIS2MDL mag, STTS22H temp, SHT40 humidity) from the shared bus —
  reading them needs the LSM6DSV16X's own I2C sensor-hub (mode 2). *(Since implemented and working —
  ENV stream 10; see the status block above.)*
- SFLP quaternion wire format: ~~IEEE binary16 (fp16)~~ — **superseded**: shipped as **4×float32**
  (16 B, negligible on the wire at one-per-ToF-frame; skips an fp16 decode path entirely). Encoding +
  golden vector in `docs/protocol.md` stream 9.
- IMU sample rate (~100+ Hz) ≠ ToF frame rate — **superseded for now**: shipped one quat/ENV sample per
  ToF frame (~28 Hz on the wire) rather than independent native-rate frames; revisit in Phase 6 if SLAM
  wants denser samples (see the status block above).
- Edge-AI (MLC/ISPU) belongs in-sensor at this tier, not on the M33.

### Phase 5 — Transport cutover to Ethernet  ← **✅ Complete**

> **Status 2026-07-28 — the tether is genuinely gone.** Until now every "Ethernet" run still had the
> ST-LINK cable plugged in, which hid a hard dependency: the NUCLEO-H563ZI has **no HSE crystal**, so
> the system clock was the ST-LINK MCU's 8 MHz MCO. Unplug CN1 and the firmware wedged before
> `ETH_Init()` — board powered, PHY link LED on, network totally silent (**BUG-023**). PLL1 now runs
> off HSI unconditionally (same 250 MHz SYSCLK; ~1% RC accuracy on timestamps, measured immaterial at
> 91.5 vs 91.4 fps decoded-frame rate). Owner-verified streaming over Ethernet with the ST-LINK
> physically unplugged and the board powered from USB_USER (JP2 at 9-10). Two host-side launch bugs
> fell out of the same session: a *missing* CDC port no longer aborts an Ethernet-only launch
> (**BUG-024**) and `UdpSource` no longer adopts the host itself as the device by latching onto its
> own looped-back broadcast wake (**BUG-025**). New boot-progress LEDs (LD1 green = clocks up, LD2
> yellow blinking = acquisition loop alive, LD3 red = wedged) make a headless board diagnosable
> without a debugger — the PHY's own link/activity LEDs say nothing about whether firmware is running.
>
> **Status 2026-07-28 addition — Ethernet hot-plug recovery:** Fixed a firmware hard-fault (an lwIP double-add assertion on `mdns_resp_add_netif`) that wedged the board when the Ethernet cable was unplugged and replugged. The host Python `UdpSource` was also upgraded to actively re-query mDNS when the stream drops, rather than blindly broadcasting `255.255.255.255` which fails to route on some Linux/Docker setups. The system now seamlessly recovers if the Ethernet tether is physically disconnected and reconnected.
>
> **Status 2026-07-14:** verified end-to-end on hardware. The device streams flawlessly over both USB CDC and Ethernet UDP.
>
> **Ethernet Implementation:**
> - Integrated STM32 HAL Ethernet drivers (`ethernetif.c` + `lan8742.c`) and lwIP (v2.1.3) manually without STM32CubeMX pollution.
> - Developed a tiny custom `dhcpserver.c` that provides a self-assigned IP `172.31.253.1` for the device and assigns `172.31.253.2` to the host directly connected via cable.
> - UDP transmission implemented in `ethernet_transport.c` mapping headers, payload, and tail into lwIP `pbuf` chains and sending to port 5000 via UDP broadcast.
> - Fixed a hardware descriptor exhaustion issue that dropped the final UDP fragment (increased `ETH_TX_DESC_CNT` from 4 to 8).
> - Fixed an initialization hang where the sensor blocked indefinitely waiting for a USB CDC connection instead of accepting an Ethernet client.
> - Fallback logic in `vl53l9_app.c`: if the Ethernet link is up and DHCP leased an IP, `ETH_SendFrame_Gather` handles sending. Otherwise, seamlessly falls back to USB CDC streaming.
>
> **Host Implementation:**
> - Added `UdpSource` in `sources.py` listening on port 5000 and fragment reassembly.
> - Falls back automatically to `SerialSource` if UDP receives no data in a 0.5s window.
> - The live soak capture proved a stable ~28.5 fps across CDC fallback. Ethernet provides the plumbing for Phase 6's tighter timestamping/network scale when needed.
> - Metrics HUD (`metrics_hud.py`) updated to show a 11.0 MB/s limit ETH capacity bar, per-stream jitter tracking, and network frame gap/drop counts.
> - **Note:** Firmware doesn't support listening for UDP commands yet (device configuration currently occurs via USB CDC), but telemetry fully works over Ethernet.

Enable the ETH MAC + lwIP (RMII pins already muxed; LAN8742 PHY on-board), move the frame protocol onto
UDP, add hardware PTP (IEEE 1588) timestamping. Post-swap rationale: with the send off the raw-only
critical path (P2.5), Ethernet's value is ~60-80 Hz-class rates, PTP sync (which matters MORE once IMU
streams exist — good ordering), and the zero-config link; none of it blocks the sensor work.

- Protocol payload is unchanged by design — this phase is transport plumbing + a UDP source class in the
  host app (dgram boundaries replace the byte-stream resync logic).
- Fragmentation (updated for Phase 2 reality): the wire payload is now the 14,842 B RAW frame — chunk
  into ≤1400-byte datagrams with a fragment sub-header (IP fragmentation is fragile; don't rely on it).
- lwIP memory tuning (PBUF pools) is the main firmware risk (eased since Phase 2 — no transform on the
  MCU anymore, so SRAM is mostly free).
- **Zero-config direct link (owner requirement, 2026-07-08):** plugging the board straight into a PC must
  work with NO PC-side configuration — the device handles cabling/addressing/discovery. Design:
  - Cabling: LAN8742 supports auto-MDIX → any cable works (verify enabled in PHY init).
  - Addressing: device first listens as a DHCP *client* for ~3 s; if a real DHCP server answers, join
    that network (covers the plugged-into-a-LAN case — never run a rogue DHCP server on someone's LAN).
    If silent, assume direct link: self-assign and start a minimal single-lease DHCP *server* on an
    unusual private subnet (e.g. 172.31.253.0/30, dodging home/Wi-Fi collisions) so the PC — which
    defaults to DHCP — gets an address instantly with no APIPA wait.
  - Discovery: mDNS (lwIP's mdns app) advertising `roomscanner.local` + a service record; the host app
    resolves it (fallback: the fixed /30 device address). `SerialSource`-style auto-find for the network.
  - PTP master on the PC, as before.

### Phase 5.5 (interlude) — Transport hardening & compression  ← **partly done, two decisions open** (2026-07-31)

Landed this pass (all on-rig verified): **BUG-040** — the web UI's Drops/Gaps rows were structurally
pinned at 0, so the primary UI could not see transport loss at all. **BUG-041** — the firmware burst
all 11 fragments of a depth frame into an 8-deep TX descriptor ring and *abandoned the frame
mid-burst* on any failure; now a slot FIFO metered from `ETH_Process()`, strictly in order, retrying
rather than abandoning. **BUG-042** — host reassembly required `frag_idx == expected`, so a merely
*reordered* datagram destroyed a whole 14.8 KB frame exactly like a lost one, and counted nothing;
now indexed slots plus counters that separate reorder / loss / duplicate / invalid, reported on the
`metrics` message. Plus a table-driven `rs_crc32` (**measured 2.21×**, 125.2 → 56.7 µs/frame on x86).

**Open item 1 — the pacer's benefit is still unmeasured, and cannot be manufactured.** The RavPower
bridge is now in path (ping RTT 5.9–8.0 ms vs sub-ms wired, stream jitter 0.6 → ~2.0 ms) and the link
measures **zero loss**: 131 seq gaps over ~567,000 frames (0.023%) across a 5 h uptime, then 0 gaps in
60 s, then 0 gaps / 0 incomplete / 0 reordered in 45 s after a restart. Good for the rig, null result
for the change. Re-measure only if loss actually appears — the counters will then say *which* fault it
is, which is the thing that was impossible before.

**Open item 2 — compression, go/no-go.** Investigated 2026-07-31 with real codecs (lz4, zstandard,
heatshrink2 installed and measured; no proxies).

- **In transit — recommended.** Payload is 16-bit LE (proven from byte-lane entropy: 6.25 bits on even
  offsets vs 2.99 on odd). LZ4 on raw bytes is nearly worthless (66%); a **2-byte de-interleave first**
  takes it to 49.8% on one capture, **54.3% pooled over 1,400 frames from four captures** — quote the
  pooled figure. That is 11 fragments → 6–7, i.e. a **~1.6× reduction in frame-loss probability, not
  the 11× the amplification figure suggests** (loss scales with fragment count, so the gain is bounded
  by the ratio). Link 466 → 265 KB/s. CPU is ≈free because compressing *before* the CRC shrinks what
  the CRC covers; net −0.7 … +0.6 ms/frame (**estimated**, nothing built) against a 33 ms period that
  already spends ~20 ms spinning in `platform_wait_for_event`. **Reject the inter-frame delta**: 4.6 pp
  for the same worst-case fragment count, and it propagates a single lost datagram into multi-frame
  corruption — the exact failure it is meant to fix. Ship as a **new `stream_id` 14** (additive:
  `docs/protocol.md` requires decoders to skip unknown stream ids, so no version bump), CRC *after*
  compression, with an uncompressed fallback so the worst case stays bounded. Note this turns a stream
  fixed-length since Phase 2 into a variable-length one — `native.py`'s `len(raw) == 14842` assert and
  the pacer's slot sizing both inherit that. Full `protocol-change` checklist applies.
- **At rest — not worth it.** The number that settles it: the whole corpus is **1.10 GB**. Best
  realistic inline scheme is ~2.3× on real scans (~620 MB saved) and pays for it by making `Recorder`
  protocol-aware (it is deliberately protocol-blind, fed arbitrary chunks), breaking the documented
  *readable mid-recording* invariant, and adding an offset-mapping layer to `FileSource`,
  `build_capture_index`, `seek`, `_survey`, `analyze_capture` and `slam/cli`. Corrects a natural
  intuition: CALIB *is* byte-identical across all 49 copies, but that is **0.225%** of a file — the real
  redundancy is *positional* inside RAW_3DMD (2,367 of 14,842 byte positions never change across 3,163
  frames = 15% of the file), which is why long-window matching buys 3% and dictionary priming 0.2%.
  Roughly half the file is irreducible sensor noise. If disk pressure ever arrives: an out-of-band
  `capture_archive` MCP tool, not inline compression. Measured aside: **zstd-9 strictly dominates
  zlib-6** here (ratio 1.868 vs 1.808, 2× faster compress, 4.5× faster decompress) — use it if inline
  is ever mandated.
- **They interact.** `Recorder` tees verbatim wire bytes, so compressing on the MCU makes captures
  compressed *for free* — no container format, no seek work. That is an argument for doing transport
  and leaving captures alone, at the cost of an LZ4 decompress at `decoder.py` plus
  `web.build_capture_index` and the `native.py` size assert.

### Phase 6 — Real-time SLAM (PC)  ← **in progress**

SFLP quaternion as rotation prior → 3-DoF constrained **point-to-plane ICP, frame-to-model** against the
TSDF raycast (Open3D tensor pipeline: `t.pipelines.registration` + VoxelBlockGrid), IR as intensity
channel, barometer as soft 1-DoF Z constraint.

> **Status (2026-07-28):** the core pipeline is **built and running**. Shipped so far: the
> `roomscan.slam` subpackage (SFLP-prior + point-to-plane frame-to-model ICP + TSDF `Mapper`,
> `roomscan-slam` CLI; offline validation record: `docs/phase6-slam-validation.md` — its CPU timings
> predate the GPU); off-thread live-view rendering (`MeshPrep` + pose/mesh transport split, block
> below); the panel SLAM mode (since deprecated with the panel) and its replacement — **Web Phase 4's
> `SlamRunner` on local CUDA:0 (RTX 2000 Ada, ~7 ms/frame) is the primary live SLAM surface**; a
> display-only stationarity hold that kills stationary ICP-translation jitter (map accuracy
> byte-identical); and CUDA at-scale validation (GPU ~2.1× per-step vs CPU with a flat degradation
> curve; ~~3 of 4 latent CUDA bugs fixed — the 4th is sub-phase 6.G~~ **all 4 now fixed — the 4th
> closed 2026-07-29 as sub-phase 6.G / BUG-032**). **Open:** sub-phase **6.D** (drift correction /
> loop-closure evaluation — the owner's next target; ~~its "measure first" gate needs an
> owner-recorded closed-loop walk, which no capture contains~~ **the gate is SATISFIED as of
> 2026-07-30**: the owner recorded two room circuits, `captures/coffeeRoomCircuitNoMnt.bin` closes at
> ~~**0.150 m over 32.5 m (0.46%)**~~ **0.74 ± 0.19 m over 23.9 m (~3%)** with 0 lost frames — the
> old figure was a lucky single run over a barometer-inflated path, corrected by BUG-037 — and
> `captures/coffeeRoomCircuitMnt.bin` is the failure case — see the 6.D block below) and ~~the on-rig
> flat-field capture (Phase 2.5 follow-up, **DC-D**) gating reflectance-quality work~~ **DC-D landed
> and PASSED 2026-07-31 (residual 7.3%, all 2268 zones inside [0.5, 1.6]) — the flat-field correction
> is now unblocked and merely needs enabling**. Newly **proposed**: sub-phase **6.H** — an audible
> coverage cue (buzzer clicking on new TSDF blocks), owner idea 2026-07-31, block at the end of this phase.
>
> **⚠️ 6.D is now blocked on the TRANSPORT, not on SLAM (2026-07-31).** The multi-room captures
> (DC-B ×3) arrived and, unexpectedly, the loop-closure question cannot be scored on them: 2.29–9.35%
> of frames were lost in multi-second whole-group outages (**BUG-049**), and a paired ensemble over two
> takes of the *same route at the same path length* attributes **4.569 m [95% CI 3.950–5.168]** of
> closure difference to that loss alone. The good news underneath it: the cleanest take closes at
> **3.6% over 75.3 m**, versus ~3.1% single-room — so drift scales with path length and does **not**
> compound across rooms, which weakens rather than strengthens the case for loop closure. Fix the
> transport, re-record DC-B, then evaluate.

> **Orientation accuracy for handheld use (2026-07-29)** — a full pass on the orientation path,
> triggered by BUG-027's leftover "beat the fp16 floor" item and then **re-prioritised by the owner's
> note that this is a handheld device** ("maximum accuracy even over short timeframes"). Ranked by
> contribution during a 100 °/s pan, the error budget turned out to be the inverse of where the work
> started:
>
> | source | contribution | status |
> |---|---|---|
> | magnetometer calibration, direction-dependent | up to **~90°** heading error | **fixed 2026-07-30** — owner re-fit, BUG-030 |
> | LSM tick uncalibrated (2.98% scale) | ~2.7° on a 90° pan | **fixed** — stream 12 `IMU_CAL` |
> | ToF↔IMU frame-stamp skew | 0.19° → 0.107° → **0.002°** RMS | **fixed 2026-07-30** — stream 13 `IMU_SYNC`, BUG-031 |
> | stream-9 quat is a batch MEAN, valid +7.8 ms after the frame | **~0.30°** at 38.5 °/s | **measured & on the wire, not yet corrected** — `quat_mid_ticks`, BUG-031; needs **DC-F** |
> | fp16 SFLP quantization | 0.018–0.027°/frame | transport shipped (stream 11); fusion built, **not wired in** — the A/B needs **DC-F** |
>
> **Shipped:** **stream 11 `RS_STREAM_IMU_RAW`** — 480 Hz verbatim FIFO pass-through (GY 0x01 / XL 0x02 /
> TIMESTAMP 0x04 / SFLP-gbias 0x16 / SFLP-gravity 0x17, 8-byte records, tag byte rebuilt as
> `TAG_SENSOR<<3 | TAG_CNT<<1`, count in `width`); **stream 12 `IMU_CAL`** carrying `INTERNAL_FREQ_FINE`
> on the 64-frame CALIB cadence (host applies `t = 1/(46080·(1+0.0013·freq_fine))`); a **TIM2
> microsecond clock** replacing `HAL_GetTick()*1000`, with the ToF stamp moved to the sensor's
> FRAME_READY edge; **sensor-hub averaging** (mag/baro/temp were keeping 1 of ~2 samples per drain —
> same defect class as BUG-027, milder). On-target: **freq_fine = −20**, clock ratio **29790 → 3345 ppm**
> (residual is the MCU's HSI, not the LSM), stream 11 at **100.01% delivery / 0 gaps / exactly 480.0 Hz**,
> streams 7/9/10/11 at 30.3 fps (interval convention), 0 CRC, 0 drops, 0 gaps.
>
> **Host:** `roomscan.imufusion` — complementary filter (gyro propagation on LSM timestamps with gbias
> subtracted, gravity tilt correction, stream-9 yaw anchor), **gated OFF by default** with an explicit
> SLAM non-regression test. Synthetic gain 6.2× on tilt in the under-dithered regime.
> *(2026-07-30, **BUG-039**: its yaw loop measured heading about **body Z** — ZYX yaw — on a body
> frame whose X is Up, so at this device's attitudes it was nulling a quantity 4° from gimbal lock:
> 1.689° mean / 2.217° p95 of real heading error, insensitive to loop gain. Replaced by a world-Z
> swing-twist term, `sensors.graft_yaw_error_deg` — stationary 0.017° / 0.053°, and bit-identical on
> the two zero-pitch captures of a seven-capture ensemble, because the misreading scales as
> tilt × tan(pitch). **Still gated off**: this makes the SFLP-vs-`imufusion` comparison meaningful,
> not decided — no capture in the repo carries orientation ground truth.)*
> *(2026-07-31, **BUG-051**: the same defect, third instance — and this one was in the **shipped live**
> path, not a gated-off filter. `sensors.absolute_heading` stripped yaw with the ZYX `quat_yaw_deg`,
> giving an **18.4° systematic heading error** at the normal handheld pose (26.57° reported for a true
> 45°, exact only on the cardinal four, which is why it survived). `YawFusion`'s 15° gimbal gate — cited
> at the time as the defence `imufusion` had failed to inherit — turned out to be the bug rather than
> the defence: |ZYX pitch| ≈ 90° **is** the upright grip, so it froze the correction permanently in
> ordinary use. Both now use `sensors.yaw_twist_deg`; the gate is deleted. **The rule from BUG-039
> generalises: no Euler yaw as a quantity anywhere in a live path.**)*
> UI gained raw
> orientation readouts, per-signal p95/mean jitter, **four orientation decomposition modes**
> (zyx / zxy / boresight / world = gravity+mag) with renamable labels and a near-singularity warning,
> a **zero-yaw** control (SFLP yaw has an arbitrary origin — no magnetic input), and a **magnetometer
> calibration modal** with sphere-coverage visualisation.
>
> **Sensors-card declutter (owner: "cluttered and hard to read", 2026-07-29)** — those readouts landed
> as four flat blocks in a 232 px rail: ~25 rows, most wrapping onto two lines, and *duplicated*
> (the selected mode's roll/pitch/yaw ARE the raw ZYX ones in the default mode; heading appeared
> three times; World mode printed its gravity+mag caveat twice). At ~1600 px the card also ran past
> the dock band, so the jitter table was unreachable and `layout.js` auto-collapsed the whole card on
> a narrow window. Now three always-visible tiers (gizmo+compass · the selected orientation readout ·
> Fusion state with its two buttons · Environment) plus a collapsed `#sensor-diag` `<details>` holding
> the mode note, yaw offset, full-precision raw ZYX + quat, and jitter as a p95/mean **grid**.
> Nothing was removed but the duplicate world note. **Presentation only** — no `/ws` message, field,
> or precision changed, so `docs/web-protocol.md` is unaffected; 959 tests unchanged. Gotchas worth
> keeping: Chrome renders `<details>` content in an anonymous `::details-content` box, so a
> flex-shrunk `<details>` does **not** pass its height to a child (the scroll box must be the
> `<details>` itself); and driving the card with `web_ui_shot.py` needs `#sensor-diag.open = true`
> first — see `docs/web-ui-testing.md`. BUG-033.
>
> **3D calibration feedback ("Shell & Steering", 2026-07-29, `cf3b243`)** — the modal's hero is now a
> body-fixed 3D scene: a device model inside a translucent shell of the same 92 Fibonacci cells
> (covered = solid discs, fill = |B| deviation, radius = sample count; missing = dashed hollow rings),
> a 3 s comet trail, **B** and **g** arrows with a live **B∠g dip arc**, and a world-fixed "Steering"
> widget showing a ghost device at the target attitude. New binary **tag 5 `MAGPOSE`** (68 B at 30 Hz,
> only to clients with the modal open, ~2 kB/s). Guidance is now the exact rotation
> `axis = unit(t × d)`, `angle = acos(t·d)` — that axis is a *body* axis, so it draws directly on the
> model, which also **removed the old northern-hemisphere dip assumption** from the hint text.
> The dip arc is a second, scale-immune error channel: for a correct calibration B∠g is a constant of
> the location, so a wobbling arc reveals errors the |B| magnitude metrics structurally cannot.
> Degrades to the 2D Lambert discs on no-WebGL / context-loss / `?magcal2d=1`. 914 tests.
> Design: `docs/superpowers/specs/2026-07-29-magcal-3d-feedback-design.md`.
>
> **Amended 2026-07-30 (owner) — the hero camera is ~~body-fixed at a 3/4 offset~~ now
> FIRST-PERSON.** *"During mag cal, we should render the view from the first person perspective of the
> camera (gravity down always, similar to the fpv world view). The sphere should be translucent for
> points that are 'behind' the camera."* The **shell is still body-fixed**; what changed is the camera
> — parked behind the device on the boresight (body −Z, standoff 4.3, fov 40°) with `camera.up`
> tracking −g, the same rule `web.boresight_view_frame` applies to the live FPV cloud, here applied to
> a camera instead of to points. Screen-down is room-down always and the camera's only motion is a
> gravity roll, so the "a hole is where it was" property survives. Translucency **flipped side**: it
> now keys on `dir·boresight < 0` (the cells *behind the camera*) rather than ~~distance from the
> eye~~ — from this viewpoint the rear cap covers the whole silhouette, so the conventional
> far-is-faint cue would hide exactly the hemisphere you are aiming into. Implemented as a **material**
> split (four `InstancedMesh`es, since per-instance alpha does not exist), which also retires the old
> brightness-mix depth cue; `Front`/`Back` labels dropped (they project to screen centre). Presentation
> only — no `/ws` change, no protocol change, 1012 tests. §4.1 of the design spec is amended in place.
> **Deferred:** driving pose from stream 11 / `ImuFusion` (design Phase 3 — note `ImuFusion.update()`
> currently emits one quat per ToF frame, not 480/s, and should be a session-private instance never
> attached to `SensorState`); `#ef4444` colour re-stepping (Phase 2).
> **Unanswered by the owner:** free tumble with steering (implemented) vs a prescribed six-pose recipe,
> plus four more in §12 of the design. ~~**No capture contains a real tumble yet** — the tilt sweep fills
> only 2 of 92 cells (it is one-dimensional); covered-shell screenshots used a synthetic fixture.~~
>
> **Magnetometer recalibration — ✅ done (2026-07-30, BUG-030 closed).** The owner ran the tumble
> through this modal (hand-held, off the tripod, mount plate attached) and hit **Save & apply**,
> exercising the hot-reload path with a real fit for the first time. Validated against an independent
> 118 s room sweep (`captures/roomSweepFull20260730.bin`): **attitude-locked error 0.29 µT (0.56%)**,
> tilt ramp **1.042×** (was 2.721×), |B| flat across every tilt bin (was a 40 → 110 µT monotonic ramp),
> and `YawFusion` **`gated:anomaly` 58.6% → 0%, `active` 6.2% → 64.8%** — i.e. the eCompass was silently
> off for most of a scan and now runs. New scoring tool `host/tools/mag_check.py` +
> `capture_magcheck` MCP tool, built on a new `magsweep.attitude_locked_error`.
>
> Two things this taught, both now encoded in that tool. **Raw |B| spread is not calibration error on a
> moving capture** — the room's own field walked ±6% across the sweep (49.9 → 54.4 µT) while the spread
> *within* any 10 s window stayed ~0.4 µT, so `field_consistency` scores the good fit "bad"; detrend
> first (BUG-034). And **the detrended number alone is not a verdict** — it is a lower bound, because an
> attitude family held longer than the detrend window gets absorbed into the trend. A synthetic capture
> with a 59 µT hard-iron error scores "good" on it while ramping 3.5× across the detrend-free tilt table,
> so `mag_check` takes the worse of the two. **Still unproven: heading *direction*** (magnitude flatness
> cannot see DT0103's rotation ambiguity; ~2.5° bound from the near-spherical soft iron) — needs a
> braced, fixed-compass-heading tilt sweep: **DC-E**. A recorded tumble fixture is **DC-G**.
>
> **Measurement method matters here** — three plausible readings were wrong before the right one
> emerged; see the `orientation-noise-floor` memory for the five traps (notably: use **p95**, never the
> median, on this signal; and normalise quaternions for angles but NOT for tie counting).
>
> **Resume point:** `docs/superpowers/plans/2026-07-29-orientation-resume.md`.

> **Live-view rendering (2026-07-14)** — the "rendering-first for live view" step (live view ≥30 fps,
> ideally 120+, flat as the map grows; the fps goal is architecture-bound, not compute-bound). Shipped
> per `docs/superpowers/plans/2026-07-13-live-view-fps.md` (subagent-driven, 12 tasks + 2 review fixes,
> `feature/phase6-slam`): **(A)** an off-GUI-thread `slam/meshprep.py` (`MeshPrep`) does all O(map-size)
> mesh work (shade / decimate / wall-split / floor-grid → plain-data `MeshPacket`); the GUI tick only
> uploads a ready packet at `mesh_upload_hz` (default 3.0) with an **adaptive, latched** decimation
> controller (decimate to `live_vertex_budget`=150k once an upload exceeds `fps_budget_ms`=8.0, then
> stay decimated — no oscillation). Decimation is **display-only**; the saved map is always full-res.
> **(B)** the remote service now streams a tiny **pose message every frame** + a **mesh message only
> when new** (no full-trajectory resend); the client accumulates the trajectory from pose deltas. A
> viewport render-fps counter + HUD "VIEW" row measure the goal. The live trajectory ribbon is now
> **hidden by default** (a "Trajectory trail" checkbox) and throttled when shown. Both backends keep the
> `latest() -> (mesh, trajectory, FrameStep)` contract. **Status: code-complete + reviewed, 506 host
> tests green; the live ≥30/120-fps-flat numbers are UNVERIFIED-BY-RUNTIME** — the interactive GUI
> replay needs a physical display + a map-growing capture (measure on-rig, both backends).
>
> **Wire-format change + container-rebuild (protocol lockstep):** Component B changed the remote
> service→client wire format to **tagged `pose`/`mesh` messages** (was one untagged combined message
> per frame). A GPU container image built before this change starves the new client of meshes — the
> untagged legacy message has no `"type"` key, so the client never enters the mesh branch and the live
> view goes **blank** (pose/trajectory still work). Two mitigations shipped: **(1)** rebuild the container
> (`tools/slam-container/build.ps1` + `start.ps1`) so it runs the new service — required to get the
> split's bandwidth win; **(2)** the client is now **backward-compatible** — it recovers an inline mesh
> from a legacy untagged service and warns once (commit c500b0d), so a stale container no longer blanks
> the view. On-rig blank-surface bug (2026-07-14) traced to exactly this skew and fixed.
>
> **Panel UI redesign (2026-07-14)** — the `roomscan-panel` GUI was restructured from a sidebar-driven,
> multi-mode window into a **two-mode, first-person-by-default, HUD-driven** instrument, per
> `docs/superpowers/specs/2026-07-13-panel-ui-redesign-design.md` +
> `docs/superpowers/plans/2026-07-14-panel-ui-redesign.md` (subagent-driven, 13 tasks + 4 review fixes,
> `feature/phase6-slam`, commits `d654f93..8e24f6b`). What shipped: two view modes **Real-Time / SLAM**
> (SLAM absorbs the former Showcase record→process→reveal flow — no separate Showcase concept in the UI);
> a **First-person/Orbit** camera toggle defaulting to first-person in both modes; the always-visible
> sidebar retired in favor of a **menubar + one settings dialog** (`settings_dialog.py`); a **floating
> in-scene HUD** (mode switch, view toggle, action cluster, IR control, status chip) custom-drawn in the
> instrument language — new pure, unit-tested modules `instrument.py` (drawing primitives shared with
> `cards.py`), `hud.py` (renders + `HudLayout` hit-test), `ir_overlay.py` (first-person IR billboard
> quad); a camera-gizmo-flicker fix (gizmo gated on orbit only); and `mode`/`camera`/`ir_overlay`/
> `ir_opacity` config persistence. The HUD mode-switch + view-toggle are the **sole** mode/camera
> authority (the old SLAM/Showcase/Follow checkboxes were removed). **Status: code-complete + reviewed,
> 561 host tests green.** The **mouse-fallthrough question is RESOLVED on-rig (2026-07-14):** the floating
> `ImageWidget`s DID consume clicks (the SceneWidget's `set_on_mouse` never saw them) — fixed not with the
> planned invisible-button layer but by giving each HUD widget its own `set_on_mouse`
> (`_on_hud_widget_mouse`) that reuses `HudLayout.hit_test` unchanged (BUG-011). The per-frame `srgbColor`
> Filament console spam was fixed alongside it (BUG-012, `logfilter.py`). **Remaining GUI-runtime behavior
> is still UNVERIFIED-BY-RUNTIME** (Filament needs a display) — a supervised on-rig run should still
> eyeball: the smoke pass (`host/tools/panel_ui_smoke.py`), mode/camera switching + first-person cameras,
> IR billboard texture render/UV orientation + opacity, settings-dialog re-open widget lifetime, and
> dialog scroll reachability (currently a plain `Vert` — may need `ScrollableVert`).

#### Sub-phase 6.D — Drift correction with LiDAR: ICP-yaw feedback + loop-closure evaluation  ← **next (owner target, 2026-07-28)**

The SFLP orientation prior is 6-axis: roll/pitch are gravity-referenced and drift-free, but **yaw
drifts** — bounded today only by the slow magnetometer fusion (`docs/yaw-fusion.md`, a gentle drift
bound that freezes on magnetic anomalies, not a hard reference). Translation already comes from ICP
alone. "Use the LiDAR to correct IMU drift" is **not the same thing as loop closure** — three distinct
mechanisms, in increasing scope:

1. **Frame-to-model ICP (already running).** Every frame registers against the TSDF raycast, so the
   pose the map is built with is already LiDAR-corrected — the IMU is only the prior/initial guess.
   Local drift is continuously suppressed; the residual failure mode is slow *map warp* accumulated
   over a long scan.
2. **ICP→IMU feedback (cheap; the likely first deliverable).** Feed the ICP-refined yaw back into the
   orientation filter — complementing or replacing the mag graft while ICP tracking is good. Indoor
   point-cloud yaw beats magnetic yaw (rebar/wiring distortion — the yaw-fusion spec says so itself),
   and `YawFusion`'s gated-graft machinery already exists to receive it. Also stabilizes the prior
   handed to the *next* frame's ICP after fast motion.
3. **Loop closure (global; evaluate before building).** Detect a revisited place, add a pose-graph
   constraint, redistribute the accumulated error over the whole trajectory, then re-integrate the
   map. Frame-to-model already gives a *soft, implicit* version — re-entering a mapped area snaps the
   pose back onto the existing map — but it can never fix warp already integrated into the TSDF; only
   a pose graph + reintegration pass can. Caveats for this sensor: 55°×42° FoV / 2,268 pts per frame
   means classic LiDAR place-recognition descriptors (Scan Context et al.) don't apply — revisit
   detection would be pose-proximity-gated ICP verification, not appearance-based — and TSDF
   reintegration is expensive (needs a submap or keyframe-replay design).

Scope: **measure first** — record a closed-loop capture (walk a loop, return to a marked start pose)
and quantify end-to-end drift (`start_end_gap_m` in the `roomscan-slam` metrics; the Task-9 capture
showed 1.1–1.4 m over ~70 m of path, pre-GPU). Then ship (2), re-measure, and decide from the residual
whether (3) earns its complexity — frame-to-model may already be enough inside a single room; loop
closure pays off on multi-room trajectories.

**The "measure first" gate is satisfied (2026-07-30).** The owner walked two circuits of the same
room, each ~60 s, both byte-clean with stream 9. Measured at 1 cm voxels on CUDA:0, after BUG-036
and BUG-037. **These numbers are ensemble means ± sd over 10 numerically innocuous perturbations,
not single runs** — a single run cannot be trusted below ~0.3 m of closure, because a deliberate
3 mm height nudge moves the loop closure by 0.37 m and the height error by 146 mm (BUG-037). Every
figure in the first version of this table was a single run, and the ~35% of path that the barometer
invented deflated the drift percentages on top of that:

| capture | lost | horizontal closure | path | drift | height error |
|---|---|---|---|---|---|
| `coffeeRoomCircuitNoMnt.bin` | 0 | **0.74 ± 0.19 m** | 23.9 m | **~3%** | 125 ± 76 mm |
| `coffeeRoomCircuitMnt.bin` | 0 (was 423) | 0.91 ± 0.31 m | 20.9 m | ~4.4% | 102 ± 113 mm |

~~**Frame-to-model alone already closes a single room to 0.46%**~~ **Frame-to-model closes a single
room to ~3%** (~0.7 m over a ~24 m circuit) — that is the number (3) has to beat. It is ~5× worse
than the 0.46% this section originally claimed, so the "loop closure does not obviously earn its
complexity indoors" conclusion is **weaker than it looked**, though 0.7 m of absolute closure from a
54×42 imager with no loop closure is still respectable. Note this supersedes the 1.1–1.4 m / ~70 m
Task-9 figure above as the current baseline.

Three findings that change what 6.D should do next, in priority order:

1. **Robustness, not drift, was the real failure.** The mounted run did not drift to 2 m — it *died*
   at frame 1466 and dead-reckoned a frozen pose for the last 22% of the capture, silently reporting
   a plausible 2.05 m "drift". Fixed as **BUG-036** (escalating ICP retry radius + `tracking_stats`
   reporting `died`). Still architecturally open: **there is no relocalization**, so a bad *second*
   (as opposed to a bad frame) is still terminal.
2. ~~**The barometer is a bigger error source than yaw drift right now**~~ — **BUG-037, fixed
   2026-07-30.** The barometer's per-frame signal is **267 mm RMS of white noise**, and the old
   fixed-gain blend pushed ~5% of it into the pose every frame: ~12 mm of vertical step per *frame*,
   i.e. ~35% of reported path was motion that never happened, and a blend hands the barometer DC
   authority 1.0 in exactly the band where it is ~20× the worse instrument. Replaced by a
   low-passed, bounded-authority complementary correction (`baro_tau_frames` = 900,
   `baro_authority` = 0.05 derived from the two drift rates). Reported path drops 34/29/37% across
   three captures and the whole-run barometric contribution is now ~10 mm. **Honest caveat:** on
   1-minute scans the corrected constraint is indistinguishable from switching the barometer off —
   it is kept because its parameters are now measurable, not because it earns its keep today.
   The second finding there is methodological and applies to everything in this section: **single
   SLAM runs are chaotic** at the 0.3 m level, so score ensembles.
3. **Yaw drift is not the bottleneck at this timescale.** The datasheet's SFLP spec (Table 1) is
   5.9°/5 min heading drift at high dynamics — ~1.2° over a 62 s circuit, ≈8 cm over the 3.7 m max
   excursion. Deliverable (2) is still worth shipping, but it cannot be what limits these runs.

**✅ CLOSED 2026-07-31 — BUG-036's fix has now been exercised in the field, and it held.** `DebugCapA.bin`
(DC-A: 89 s brisk sweep, 24 whips above 100 °/s, 0.19% transport loss) scored over a 5-run ensemble:
**ICP escalations 7–47 per run** (so the retry genuinely fired), **0/5 runs died**, longest frozen run
**28 frames < 30**, closure 0.92 ± 0.55 m over 39.2 m of path. All three clauses of DC-A's gate pass.

`DebugCapC.bin` (DC-C) then probed what the retry does *not* cover: deliberate 2 s tracking kills
produced freezes of **82–120 frames (2.7–4.0 s)** and 376–467 lost frames per run — yet **every run
recovered** (0 died, 0 trailing). It recovered because DC-C's protocol says to *return to
already-mapped geometry*, which is precisely the job relocalization would do automatically. So the
relocalization gap is real but is **operator-maskable**, and the case for building it rests on how
often a scan cannot be walked back — see BUG-049, where a 2.4 s transport hole cost 21.2 s because the
operator had no reason to know they needed to retrace.

The original open item, retained for history:

**~~⬜ Open — BUG-036's fix has never actually been exercised on a live handheld scan (2026-07-31).~~**
The owner hit the classic failure that day: a SLAM run "started out great", degraded, then gave up.
Post-mortem on the saved trajectory (`results/web_20260731-070730.tum`) showed translation frozen at
**idx 333 / t = 11.89 s** while rotation kept tracking off the SFLP prior — the signature of
`predict_pose` holding `t_prev` on a lost frame. **391 of 725 poses (54%) were dead**, and the run
still reported a plausible-looking 0.764 m closure. The trigger was two fast frames (39 mm then
52 mm ≈ 1.5 m/s against a median 16.9 mm) overrunning the fixed 0.05 m ICP correspondence radius.

That is exactly BUG-036, **fixed the day before** — the run failed only because the `roomscan-web`
process had been up since Jul 29 and was executing pre-fix code (`block_count = 40000`, plain
`register`, no `lost_flags`). Restarting it picked up BUG-035/036/037. So the fix is live but
**unvalidated in the field**: the escalating retry survives a bad *frame*, not a bad *second*, and
there is still **no relocalization**. Next step is a fresh handheld scan with deliberately brisk
motion, checking `tracking_stats` / `lost_flags` rather than eyeballing the mesh — a dead run looks
fine until the trajectory is read. Cheap first pass: replay an existing capture through the fixed
pipeline before asking the owner to walk a room. That scan is **DC-A** in the data-collection queue;
**DC-C** (deliberate tracking-kill events) is the fixture for the relocalization gap it leaves.

*Process note worth keeping:* a long-lived server silently pinning old code is its own failure mode.
Both a stale-process check and the "● Restart Server" top-bar button (2026-07-31) exist because of
this.

**Ground truth without instrumentation, opportunistically.** Both captures start and end parked on
the ceiling at identical elevation (80.1°/80.2°, 89.2°/89.2°) and identical range (1420/1420 mm,
1453/1452 mm) ⇒ same height ⇒ true height error ~0. That is what exposed BUG-037 and what
corroborates the clean run's 0.46%. ⚠ **Do not turn this into a required protocol** — the owner's
objection (2026-07-30) is that walking to the table to park the device puts the operator in the
sensor's FOV. Score bookends when a capture happens to have them; never demand one. (Re-verified
independently 2026-07-30 while closing BUG-037: elevation matches to 0.17°/0.04°, range to
0.1 mm/0.6 mm, device stationary at both ends ⇒ true height change **<1 mm**.
`captures/roomSweepFull20260730.bin` has **no** bookend — it ends mid-sweep — so its height and
closure numbers have no ground truth and must not be quoted as drift.)

**Live/View Detailed-SLAM follow-ups (recorded 2026-07-31).** The Live/View playback foundation is
landed: server-authoritative `source`/`display` state, TIM2-based replay timing, stream-9 capability
gating, capture-keyed Detailed sidecars, progressive `MESH` updates, and the pure paired-gate helper.
Detailed now reports preview-mesh extraction separately from frame processing, shows asynchronous
saved-sidecar loading immediately, pauses invalid replay during Detailed, and displays live
GPU/CPU/RAM/VRAM utilization beneath its progress bar. This fixes the visible-feedback path; it does not
replace the end-to-end verification below.
The following work is deliberately still open; do not describe the fallback preset as a validated
loop-closure implementation.

1. **End-to-end browser/server verification.** Run the websocket integration and headless-browser
   flows with a permission profile that permits local port binding. Exercise Live↔View source swaps,
   recording only in Live, legacy no-stream-9 capture messaging, timestamp seek/`mm:ss` labels,
   Detailed build progress, cached-current sidecar load, stale-sidecar badge, and manual regenerate.
2. **Calibrate Detailed cost and iteration choice on CUDA:0.** Populate the clearly labelled
   `[slam.detailed]` per-frame and global-optimization estimate constants from real runs. Evaluate
   max ICP iterations **6, 8, 10, and 12** on both `coffeeRoomCircuitNoMnt.bin` and
   `coffeeRoomCircuitMnt.bin`, each with the matched ten innocuous perturbations. Retain the
   measured six-iteration setting unless a higher count passes the same tracking/closure guard.
3. **Implement and evaluate the offline pose-graph pass** (needs **DC-B** — the paired gate below has
   only the two single-room circuits today, and loop closure is supposed to earn its keep on
   multi-room trajectories). Build keyframes from offline tracking,
   find non-adjacent pose-proximity revisits, verify candidate edges with strict ICP, globally
   optimize, and re-integrate every raw frame against the optimized/interpolated trajectory before
   exporting the Detailed artifacts. Relocalization remains explicitly out of scope.
4. **Enable loop closure only through the paired gate.** Accept it only when *both* circuits have a
   positive paired 95% confidence interval for reduced horizontal closure, with no run that dies and
   no increased tracking loss. On failure, preserve the same Detailed UX and sidecar contract with
   `loop_closure.enabled=false`, and record the measurements/reason in each manifest and validation
   report. Never auto-regenerate a stale sidecar.

**Review pass on the landed foundation (2026-07-31).** Three defects that had already reached `main`
in `452b275`, all fixed, all now pinned by tests that were verified by reintroducing the defect
(`BUG-043`/`044`/`045`):

- **Save was deleted, not narrowed (BUG-043).** "Only Detailed writes a persistent sidecar" was meant
  to apply to *replay* SLAM; it was applied to Live too, leaving a greyed button and a no-op click
  handler. **Live SLAM keeps its one-shot export** (owner decision, 2026-07-31): its frames are never
  stored unless Record was running, so dropping the map discards the only copy and there is nothing
  to re-run as Detailed. Replay SLAM stays preview-only and now refuses with a reason.
- **Capability context was dropped from eight `state` echoes (BUG-044).** `slam_available` and
  `detailed` default permissive, so changing the colour or point size re-enabled the SLAM/Detailed
  segments on a stream-9-less capture and cleared the stale badge — the client drives disabled state
  purely from this echo.
- **The View library scanned on the event loop (BUG-045).** `list_captures`' new header walk measured
  **501 ms cold over 25 files / 1.06 GB** (0.4 ms warm), stalling the broadcaster on every tab
  connect and after every recording stop.

Also: the two top-bar switches were each stretching to the full 1175 px bar and stacking into
banners (`#mode-switch-slot { flex: 1 }` was written for one control); they now size to their labels.

Two things the review did **not** settle, both feeding item 2 above. The preset tracks *and* maps at
`voxel_size = 0.005`, so 5 mm reaches the trajectory — and 5 mm tracking is the one setting this repo
has measured going **backwards** (`docs/phase6-slam-validation.md`: gap 1.095 m at 10 mm vs 2.052 m
at 5 mm, though that is CPU-era, pre-frustum-raycast, on a different capture). Decoupling the
tracking voxel from the map voxel — track at 10 mm, reconstruct at 5 mm — would make the finding
irrelevant by construction rather than requiring it to be re-litigated; worth measuring before the
preset is called validated. Separately, `DetailedSlamPreset.fingerprint()` now **excludes**
`per_frame_ms`/`global_opt_ms`/`benchmark_note`: they describe how long a build takes, not what it
produces, and hashing them would have marked every existing sidecar stale the moment item 2 lands.

#### Sub-phase 6.G — SLAM GPU-memory hardening (long-scan OOM)  ← **✅ Complete (2026-07-29)**

The GPU SLAM path OOMed on a long scan: over a 68 m walk, CUDA memory crept to **~11.7 GB** and hit a
`ParallelFor` allocation failure. It was **not** the map itself — the raycast is already frustum-bounded
(`slam/mapper.py`) and the ~40k-block VoxelBlockGrid is only **~410 MB** — it was Open3D's **CUDA caching
allocator** holding temporaries that were never reused (see the `cuda-at-scale-validation` finding #4; the
first three CUDA bugs were fixed in `8258f2d`/`d229a58`, this fourth was deferred to this sub-project).
It capped how long an unattended GPU scan could run and was the last open item from the CUDA at-scale
validation. Tracked as **BUG-032**.

> **Superseded by measurement (2026-07-29):** this sub-phase originally attributed the creep to the
> caching allocator *plus* ~~"per-frame temporaries never released"~~, and scoped the fix as a release
> "**every N frames** or when a high-water mark is crossed". Both were wrong: the per-frame path is
> byte-flat, the *throttled extraction* is the leak, and the correct cadence is **per extraction**. The
> table below is the evidence.

**What the measurement actually found (and it inverted the stated hypothesis).** A new rig,
`host/tools/slam_gpu_memory.py`, logs NVML device bytes + active block count + hashmap capacity +
per-step wall time for **every** frame, so "memory tracks the map" and "memory tracks the work done"
can be told apart directly instead of inferred from a high-water mark. On the RTX 2000 Ada (8 GiB):

| run | frames | device memory above baseline | tail growth |
|---|---|---|---|
| per-frame path only (integrate + raycast + ICP, no extraction) | 4000 (80 m) | **byte-identical** — never moved off 937361408 B while the map grew 900 → 17k blocks | 0 |
| + extraction at the live cadence (`SlamWorker._MESH_EVERY = 5`) | 1500 (30 m) | 523 → **5483 MiB** | **5.13 MiB/frame** |
| + extraction, with the fix | 4000 (80 m) | 523 → peak **651 MiB**, ends at 523 | **0.005 MiB/frame** |

So the per-frame path — the thing the roadmap named as the suspect — **does not leak at all**. The
culprit is the *throttled* `mesh()`/`point_cloud()` extraction, specifically `TsdfMap._extract_vbg()`'s
whole-grid `self._vbg.cpu()` copy: its temporaries scale with the active-block count, so each extraction
asks the caching allocator for a slightly **larger** block than the last and the previous one is cached,
never reused. That is also why the fix is throttled on **extractions, not frames** — a frame-cadence
release would fire mostly on frames that allocated nothing.

**Fix (shipped):** `TsdfMap.release_cache_every` (`slam/tsdf.py`) calls `o3d.core.cuda.release_cache()`
after every Nth extraction — **default 1**, `0` disables, no-op on a CPU grid. Plumbed through
`Mapper(release_cache_every=…)`, `[slam] release_cache_every` in `roomscan.toml`, the `roomscan-slam`
CLI, and `web.SlamRunner` (so the live web SLAM surface gets it). The remote backend forwards it in its
existing mapper-kwargs JSON, so the container path is covered too.

**Verified:** no per-frame cost — step latency is unchanged (p50 6.1 ms, p90 7.0→7.1, p99 8.6→8.8) and
wall time matched (62.3 s off vs 62.7 s on over the same 1500 frames). The fixed 4000-frame / 80 m run
is **longer than the 68 m walk that OOM'd** and holds a flat ceiling; the unfixed run would have needed
~21 GB to reach the same point.

**Regression guard:** `tools/slam-container/cuda_smoke.py` gained `run_memory_ceiling()` — a 1200-frame
run with live-cadence extraction asserting a peak ceiling (1500 MiB), a flat tail growth
(≤0.5 MiB/frame), and that releases actually happened (catches the knob being disabled or the hook
becoming unwired). Thresholds sit in the wide gap between fixed (~0.005–0.04) and unfixed (5.13), so it
should not flap.

**Validated on real data (2026-07-30)** — the owner's first full room sweep, `captures/roomSweepFull20260730.bin`
(14,407 frames, 0 CRC failures, 3,525 depth frames), replayed through the rig. The effect is **larger**
than the synthetic estimate, because the leak scales with block count and a real sweep grows the map
faster than the generated walk:

| | unfixed (`release_cache_every=0`) | fixed (default) |
|---|---|---|
| growth | **10.34 MiB/frame** (2× synthetic) | **−0.03 MiB/frame** |
| device memory | 7305 of 8188 MiB by frame **800** | peak 811 MiB, ends at 523, over **3525** frames |
| outcome | ~100 frames from OOM; would need ~36 GB to finish | flat throughout |

Step latency on real data stayed healthy: p50 8.7 ms, p90 11.6, p99 14.8 (and 7.9 / 9.8 / 11.9 on the
final shipped-defaults run — see BUG-035; a bigger hashmap is slightly *faster*, not slower).

**…and removing the memory ceiling exposed the next wall — BUG-035.** With the leak fixed, the same scan
ran far enough to run up against its `VoxelBlockGrid` capacity: blocks froze at 38,937 of 40,000 (97.3%) on
frame 2879, and tracking collapsed 30 frames later (0 lost before, **560 after**; median ICP fitness
0.887 → 0.127; 18% of the scan ruined), with no log line anywhere. The sweep genuinely needs **42,917**
blocks — 7% over the old default. Fixed by raising `DEFAULT_BLOCK_COUNT` to 160,000, plumbing
`block_count` through `Mapper`/`[slam]`/CLI/`SlamRunner`/the rig, and warning once at 90% of the
*configured* capacity. **Mechanism correction:** the grid is not incapable of growing — a CUDA grid
rehashes cleanly 40,000 → 80,000 at 99.2% load. The failing run froze just *below* that trigger, at
97.3%; the precise cause is unproven (see BUGS.md), but the effect and the mitigation are measured.
Re-run on the shipped defaults: **11 lost frames of 3525**, fitness 1.00 at the end, 42,917/160,000
blocks (27% of capacity), memory flat at 0.013 MiB/frame, and step latency p50 **7.9** ms / p90 9.8 /
p99 11.9 — *better* than the 40,000 run, so the larger pre-allocation costs nothing per frame. (The
90%-capacity check is one hashmap size read, measured at **5.8 µs**, i.e. 0.02 s across the whole scan,
and it early-outs permanently once it has fired.) A **CPU grid** (`[slam] device =
"CPU:0"`, system RAM instead of VRAM) completed the same scan with **0 lost of 3525** at 46,037 blocks —
the grid is device-homogeneous, so that is the route to maps larger than VRAM holds.

Two pieces of shared scaffolding came out of this, both used by the rig *and* the guard:
`roomscan.slam.gpumem` (a ctypes NVML probe — Open3D exposes no "bytes allocated" API, and this avoids
adding pynvml) and `roomscan.slam.synthscene` (a deterministic analytic room + camera walk, runnable to
any length — needed because **no recorded capture contains a long walk**: everything in `captures/` is
stationary or a braced tilt sweep. Poses are not injected; `Mapper.step` still does its own raycast +
ICP + integrate, so the measured path is the production one).

*(Environment updated 2026-07-28: originally scoped for the retired Windows box's WSL GPU container —
native Windows Open3D CUDA was a dead end. The current headless Linux host runs SLAM **in-process on
local CUDA:0** (RTX 2000 Ada passthrough), so measure and fix there; `tools/slam-container/` survives
only as the optional `[slam] backend=remote` path.)* CPU SLAM is unaffected (it already meets the
~28 fps sensor ceiling). Belongs to the "GPU hardening for offline" leg of the owner's "both,
sequenced" directive — the live-view rendering leg already shipped (above).

**Read `docs/coordinate-frames.md` first** — every pose/prior/constraint here lives in one of the four
documented frames; the world frame, the body→world sandwich (`T_WORLD_TO_CV @ R @ T_CV_TO_BODY`), and the
baro-Z-is-Open3D-−Y mapping are all specified there.

- **Registration correction (2026-07-10):** the previously-specced "Open3D Tensor G-ICP" **does not
  exist** — verified against installed 0.19.0: `t.pipelines.registration` offers only point-to-point,
  point-to-plane, colored, and Doppler ICP; Generalized ICP lives only in the legacy CPU pipeline.
  Point-to-plane is the primary choice anyway (indoor scenes are plane-dominated; per-point covariances
  add little at 54×42 resolution). If GICP proves necessary, use
  [`small_gicp`](https://github.com/koide3/small_gicp) (koide3, v1.0.1 2026-06, pip-installable,
  Windows CI, multithreaded) — not Open3D's legacy GICP.
- **Track frame-to-model, not frame-to-frame:** register each frame against a point cloud raycast from
  the VoxelBlockGrid at the predicted pose (KinectFusion-style). This suppresses most odometry drift and
  matters more than the ICP flavor.
- **SLAM-stack survey (2026-07-10, owner question):** modern LiDAR stacks evaluated and rejected as the
  engine — the sensor is a 54×42 depth *imager* (~63 k pts/s, 55°×42° FoV, global exposure), i.e. an
  RGB-D/KinectFusion-class problem, not scanning-LiDAR odometry:
  - FAST-LIO2 / Point-LIO / CT-ICP: need raw high-rate IMU + per-point timestamps and wide-FoV
    long-range scans; degenerate on a 55° cone in room-scale scenes; ROS/Linux-centric. **Rejected.**
  - SHINE-Mapping: offline mapping only, superseded by PIN-SLAM (same lab). **Rejected.**
  - KISS-ICP (v1.3.0 2026-04, pip, sensor-agnostic): odometry-only, no prior/constraint hooks —
    **kept as an offline benchmark**: run it on deprojected recorded captures to sanity-check our
    odometry numbers.
  - PIN-SLAM (TRO'24, active, RGB-D-capable): research-grade, GPU-hungry, thin input at 2,268 pts/frame —
    **parked as an optional offline experiment** on recorded captures; not the real-time engine.
  - Open3D health: release cadence is slow (0.19.0 = 2025-01) but commits are steady through 2026-07;
    our usage is primitive-level (ICP + VoxelBlockGrid), so the cadence is low-risk.
- Baro is a *soft* constraint — indoor pressure transients (HVAC, door openings) are several Pa
  (~12 Pa/m); never treat as ground truth.
- Accel-derived translation is **not** an input (double-integration drift); translation comes from ICP.
- CPU-first: registration on 2,268-pt frames is sub-ms on CPU; the whole pipeline should hold 28 Hz
  without CUDA. Note the **Windows pip wheel is CPU-only** (`o3d.core.cuda.is_available() == False`);
  CUDA means a source build (`-DBUILD_CUDA_MODULE=ON`, MSVC) or WSL2 (Linux CUDA wheels; fine for
  recorded-capture work, needs usbipd for live device). Only do this if profiling shows VoxelBlockGrid
  integrate/raycast blowing the ~35 ms frame budget — the RTX 4080's real job is Phase 7 (3DGS
  training). Validate real-time budget with recorded Phase 1/2 datasets before hardware-in-the-loop.
  *(Superseded 2026-07-16: the runtime host is now the headless Linux box with an RTX 2000 Ada passed
  through — local CUDA:0 works in-process (~7 ms/frame), no source build or WSL needed; the
  Windows-wheel/RTX 4080 notes above describe the retired dev box. The capture-first validation rule
  was followed and stands.)*
- **Real-time RGB camera (owner question 2026-07-08, architecture decided):** live high-fidelity image
  mapping uses a webcam **plugged directly into the PC**, physically mounted on the handheld rig (the
  scanner is tethered anyway — the camera's USB run rides the same tether as the Ethernet cable).
  Routing a webcam through the board's freed-up USER USB port does NOT work for this: the H563's
  `USB_DRD_FS` can act as host, but it is **Full-Speed (12 Mbps)** — a UVC webcam at FS caps out around
  QVGA/low-fps MJPEG, the opposite of high fidelity; 1080p+ needs USB High-Speed (480 Mbps), which this
  MCU doesn't have. PC-attached also skips a host-side UVC stack on the MCU and lands frames directly in
  SLAM's clock domain (PTP-united with device timestamps). Needs: rigid mount + hand-eye/extrinsic
  calibration to the ToF (same calibration Phase 7 already requires for the phone camera — do it once,
  share it).

#### Sub-phase 6.H — Audible coverage feedback ("geiger counter" buzzer)  ← **proposed (owner, 2026-07-31)**

> **Owner's framing:** *"add a buzzer to the board that clicks when a new voxel has been received. If we
> keep sweeping over the same area and aren't getting any new information then the clicks will slow/stop,
> and a fresh area will be many rapid clicks."*

The problem it solves is real and currently unsolved: **the operator cannot see the map while scanning.**
The rig is handheld and untethered (battery + FileHub Wi-Fi bridge, Phase 5 / `hse-is-stlink-mco`), so
during a sweep the only coverage feedback lives on a screen the operator is not looking at. Every
map-completeness defect so far was found *after* the fact — BUG-035 silently dropped 18% of a sweep,
BUG-049's transport outages lost 2.3–9.4% of the multi-room takes. Audio is the right modality here
precisely because it needs no eyes and no hands.

**The signal does not exist on the board.** The MCU ships raw `3DMD`; nothing on it knows what is new —
novelty is a property of the *host's* TSDF. So this is a **host→device** feature that happens to end in a
transducer, not a firmware feature.

**Novelty signal — candidates, in preference order.**

1. **New-TSDF-block delta (recommended).** `TsdfMap.block_usage()` (`slam/tsdf.py:196`) returns the
   hashmap's live `size()`; `Mapper._sample_block_usage()` already polls it every
   `_BLOCK_USAGE_INTERVAL_S = 0.25 s`. The first derivative of that count *is* "map gained" — it is
   exactly the quantity the owner described, and it is already on the wire for the Web Phase 6 block
   gauge. Two constraints inherited, not negotiable: the read is a **CUDA device sync** (`mapper.py:42`),
   so it must **not** be moved to per-frame to get a finer click rate; and a block is 16³ voxels, so the
   granularity is decimetre-scale chunks, not points — coarse, but "did I gain map" is a coarse question.
2. **Newly-integrated voxel count** — finer and closer to the literal ask, but Open3D exposes no cheap
   primitive for it; it would cost a per-frame whole-grid comparison, which is what 6.G/BUG-032 spent a
   sub-phase removing. Reject unless (1) proves too chunky.
3. **ICP fitness / unmatched fraction** — free (already computed in `slam/odometry.py`), but it conflates
   "new area" with "**tracking is failing**", which would make the rig sing loudest exactly when the scan
   is dying. Reject as the primary signal; keep as a **mute condition** — clicks should stop, not
   accelerate, while `Mapper.lost_flags` is set.

**Delivery — rate-driven, not per-event.**

- ❌ *One COMMAND per click.* Up to ~30 datagrams/s host→device, on a link that has already demonstrated
  multi-second whole-group outages (BUG-049). Network jitter would smear the rhythm, which is the entire
  signal. Reject.
- ✅ *Send a rate.* Host sends a **click rate** at the existing metrics cadence (~4 Hz); firmware runs a
  local timer emitting clicks at that rate until a fresh update arrives. ~4 datagrams/s, jitter-immune
  (the rhythm is generated locally off TIM2), and it **fails safe** — a watchdog decays the rate to
  silence if no update lands within ~1 s, so a dropped link goes quiet rather than clicking forever.
  Consider Poisson rather than uniform spacing: an actual geiger counter's irregularity is what makes
  rate legible by ear.

**Protocol.** Additive `RS_CMD_SET_BUZZ` = **cmd 8** (`rs_protocol.h` registry currently ends at 7 /
`SET_STANDBY`); `param` u32 carries rate in centi-Hz plus a mode/mute field; ACK echoes the applied rate.
Run the **`protocol-change` skill** — `docs/protocol.md` command-registry row + rev entry, `rs_protocol.h`,
the host command enum, golden vectors, all in lockstep.

**Hardware — the one genuinely open question.** A piezo needs a free PWM-capable pin, and the
H563 + IKS4A1 + 53L9A1 stack consumes much of the Zio header. **Check the netlist via the
`stack-electrical` skill before committing to a pin** (`hardware-diagnosis-discipline`) — this is also the
one part of the loop Claude cannot do: soldering/mounting the transducer is an owner action. A self-driving
buzzer needs only a GPIO level; a bare magnetic transducer needs a gated ~2–4 kHz TIM PWM burst (~5–10 ms
per click). Current draw is ~10–30 mA peak — negligible against the untethered USB_USER supply, but keep it
off the sensor rails.

**De-risk first, in the browser.** Build the whole novelty→rate path host-side and drive **Web Audio in the
web UI** before any hardware or protocol change. Zero cost, and it answers the question that actually
decides the feature: *does the block delta feel right?* If it doesn't, the buzzer inherits the wrongness.

**Known failure mode, and a bonus.** Under pose drift the map allocates **new** blocks for an
already-scanned wall — so a fully-covered room would keep clicking, which is the exact opposite of the
intended meaning, and couples this feature to **6.D**. Inverted, that is a free diagnostic: sustained
clicking while the operator is *stationary* means the pose is drifting, audible in real time. Worth
surfacing deliberately rather than treating as a bug.

**Scope note.** Live SLAM only. In Live point-cloud display there is no TSDF and therefore no novelty
signal — the buzzer must be **silent and say why**, never synthesise a rate from frame arrival (that would
click at a constant 30 Hz and mean nothing).

**Latency budget:** 0.25 s block sampling + ~4 Hz update + link RTT. Adequate for a coverage cue; do not
extend this path to anything needing tighter timing.

### Phase 7 — Offline post-processing

COLMAP with ToF pose priors (hand-eye calibrated to the phone camera) → depth-regularized 3D Gaussian
Splatting seeded from the ToF cloud.

- Depends on recorded, timestamped datasets from Phase 3's recorder — design the recording format so
  offline tooling replays exactly what SLAM saw.
