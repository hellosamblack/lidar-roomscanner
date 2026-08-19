# Roadmap — 53L9A1 3D Room Mapping

Product goal: a **tethered handheld 3D room scanner**. The STM32H563 streams timestamped sensor
frames to a PC running real-time SLAM (Open3D tensor ICP + TSDF); an offline pass fuses 4K phone
video into a ToF-seeded 3D Gaussian Splat. Full design + critical review:
[`references/roadmapResearch.md`](./references/roadmapResearch.md).

Active development happens in this `roomscanner/` workspace. The existing STM32 firmware is **read-only
reference** in the vendored-in-repo `firmware/vendor/53L9A1/` package; firmware paths below (`Src/…`) are relative to
`firmware/vendor/53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/` (aka `<APP>`).
Engineering conventions live in [`docs/engineering-practices.md`](./docs/engineering-practices.md).

**How this file is organized.** This is the *current-state* doc: standing decisions, the
reference-firmware bug ledger, cross-cutting risks, and the plans/specs register. Forward-looking
work and open defects moved to **GitHub Issues** (2026-08-10) — see "Work tracking" below. The
completed-phase narratives and measured outcomes moved to
[`docs/roadmap-history.md`](./docs/roadmap-history.md) (they keep their `Phase N` names).

## Overriding architecture decisions

- **Transport: native USB CDC OR Ethernet UDP (Phase 5).**
  *(Revises the 2026-07-10 "Ethernet is shelved" decision.)* The device now streams flawlessly over either USB CDC or Ethernet (UDP unicast). If Ethernet is plugged in, the device acts as a DHCP client (or falls back to a self-assigned IP server) and streams via UDP to the host when a packet is received. This removes the USB cable length limit and prepares the plumbing for Phase 6's hardware time-sync (PTP) requirements. USB CDC is still supported and automatically falls back if Ethernet is not connected. *(2026-07-21: the host→device COMMAND channel now works over Ethernet too — previously CDC-only, the ETH transport discarded inbound datagrams; see the Phase 3 command-channel addendum in [`docs/roadmap-history.md`](docs/roadmap-history.md).)*
- **Sensors: X-NUCLEO-IKS4A1** adds IMU (LSM6DSV16X, hardware SFLP orientation), magnetometer (yaw-drift
  correction), barometer (Z-drift constraint), temp/humidity (thermal comp). **Integrated as of Phase 4
  (2026-07-10)** — LSM6DSV16X as a native I3C target sharing I3C1 with the ToF (HUB1-only routing,
  multi-device ENTDAA), SFLP orientation on stream 9, sensor-hub env (baro/mag/temp) on stream 10;
  stacking recipe + resolution history in `docs/iks4a1-stacking.md`. *(The original shared-bus
  legacy-I2C plan failed at speed once stacked — see the Phase 4 status block in [`docs/roadmap-history.md`](docs/roadmap-history.md).)*
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
- **Host transform builds from STSW-IMG053 (vl53l9-transform-c 1.5.0), with `VL53L9_TRANSFORM_LIGHT=0`
  pinned (2026-08-10).** `firmware/vendor/stsw-img053/` (the STM32N6570-DK package) carries 1.5.0 where
  the 53L9A1 reference carries 1.3.1; `host/transform/CMakeLists.txt` selects between them with
  `RS_TRANSFORM_PKG` and defaults to the newer. The upgrade is **bit-identical** to the 1.3.1 output we
  shipped (1800 frames, three captures, both ranging modes, all four planes) — but *only* because of the
  LIGHT pin. `vl53l9_transform.h` self-defines `VL53L9_TRANSFORM_LIGHT (1)` when the caller leaves it
  undefined, so every build of the shim before this date was a LIGHT build without saying so; under 1.5.0
  that flips `bypass-tnr-algo` and `bypass-flying-pixel-filter` to default-true, which would have silently
  disabled the temporal denoiser (worth 2.9× depth temporal noise and 0.65 → 0.83 m of SLAM closure).
  1.5.0 also adds binning 6/8/12/24 support and a reworked sharpener with optional glare recovery, both
  unused so far. Detail + the bisect: `docs/transform-streams.md` → "Library upgrade 1.3.1 → 1.5.0".
  Re-run the gate with `host/tests/compare_transform_versions.py` on the next vendor drop.
  *(The firmware fork still compiles 1.3.1 and does not define LIGHT — harmless, since LIGHT is
  string-only in 1.3.1 and `CONF_TRANSFORM_ONBOARD = 0`.)*
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
   see the Phase 3 status block in [`docs/roadmap-history.md`](docs/roadmap-history.md).
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
- **Frame-to-model chaos reads as a discrete bug, and two open issues are mis-scoped because of it**
  (2026-08-14, #81 + #180). A rotation-prior change that is *legitimately more accurate but
  different* is enough to flip the TSDF/raycast recursion into another basin, so the symptom looks
  like a defect in whatever module produced the prior. Both issues were investigated concurrently by
  independent workers over disjoint files and both cleared their own modules: #81's graft-yaw swing
  is invariant through ~600 frames of `imuTranslationError.bin` and only bifurcates between frames
  ~600–900 with **zero** ICP escalations either side, while `odometry.register()` provably never
  sees the graft (`Mapper.step` always passes `init_pose=np.eye(4)`, `mapper.py`); #180's trip-rate
  elevation survives every interpolation-level hypothesis and instead correlates with
  `slam_ensemble`'s `start_frame` perturbation. **Before attributing a drift delta to a module,
  bisect by frame count and check whether the divergence is a bifurcation rather than a bias** — and
  see the standing rule that ensembles, not single runs, are the unit of evidence here.

## Plans and specifications register (audited 2026-08-11)

This is the complete inventory of `docs/superpowers/plans/` and `docs/superpowers/specs/`.
Active work stays at the directory root; historical records are retained under `completed/`, and
superseded records under `deprecated/`. The archive status applies to the document's stated scope,
not necessarily to every later phase of the product.

| Status | Documents | Roadmap tracking |
| --- | --- | --- |
| **Active** | [Pi 3 bridge node design](docs/superpowers/specs/2026-08-17-pi3-bridge-node-design.md)<br>[Pi 3 bridge node plan](docs/superpowers/plans/build-a-plan-to-soft-truffle.md)<br>[LiDAR thin-client render design](docs/superpowers/specs/2026-08-17-thin-client-render-design.md)<br>[LiDAR thin-client render plan](docs/superpowers/plans/2026-08-17-lidar-thin-client-crowpanel.md)<br>[orientation / eCompass resume](docs/superpowers/plans/2026-07-29-orientation-resume.md)<br>[Rerun observability sidecar](docs/superpowers/specs/2026-08-02-rerun-observability-sidecar-design.md)<br>[SLAM compute & transport follow-ups](docs/superpowers/plans/2026-08-02-slam-compute-and-transport-followups.md)<br>[matched CUDA ICP / raycast study](docs/superpowers/plans/2026-08-02-cuda-icp-study.md)<br>[travel-mode Wi-Fi AP](docs/superpowers/plans/2026-08-03-travel-mode.md)<br>[splat quality improvements](docs/superpowers/specs/2026-08-07-splat-quality-improvements-design.md)<br>[splat resume handoff](docs/superpowers/plans/2026-08-08-splat-resume-handoff.md)<br>[framework exploration research report](references/software/frameworkExploration/researchResults.md) | The **Pi 3 bridge node** (#191) replaces the RavPower FileHub as the rig's wireless uplink -- routed + NAT, a rootless SD-image builder (`pi-bridge/`), `bridge_*` MCP administration, and a local pcap tee that makes frames lost over the air recoverable (the FileHub is the lead suspect for the 2.3-9.4% silent loss in #60). Implemented and unit-tested on the dev host; the spec's §11 acceptance gates all need a real Pi, so #191 stays open with `needs/operator` and the runbook is [`docs/pi-bridge-runbook.md`](docs/pi-bridge-runbook.md). The **LiDAR thin-client render** work (#194) serves pre-rendered 480×480 RGB565 frames over a new `/ws-thin` endpoint to a GPU-less CrowPanel ESP32-P4 client (companion spec in the `CrowPanelProp` repo — the protocol contract is duplicated verbatim in both, and in [`docs/thin-client.md`](docs/thin-client.md)). **Server side implemented and validated on replay 2026-08-17**: the Step 0 spike passed decisively (headless EGL works on llvmpipe, 17 ms/render, event-loop `tick_share` unchanged at the 2-client cap), so Open3D `OffscreenRenderer` was chosen and the numpy software-projector fallback was **not** needed. The spike also found two Filament constraints that shape the design — one renderer per process, and every call on its creating thread, both of which *abort* rather than raise — hence the singleton owning a dedicated render thread. Interop with a real CrowPanel is the companion repo's acceptance, not this one's. **`/ws-thin` v2 shipped 2026-08-19 (#202, superseding #197's 12-byte tag-2):** seq'd 16-byte `THIN_FRAME_JPEG` header, baseline 4:2:0 JPEG, credit-based flow control (`thin_ready` grants, drop-never-queue — the CrowPanel's 1-bit-SDIO link is ~20-40× too small for raw frames, and uncontrolled pushing only fills TCP buffers until the session flaps), and per-client `tx_fps`/`tx_bytes_per_s`/`dropped` telemetry; spec is `CrowPanelProp/docs/superpowers/specs/2026-08-18-lidar-thin-frame-bandwidth-protocol.md`, client follow-on lives there. Phase 6 orientation/DC-E (#142), the Rerun sidecar (#136), the BUG-061 compute/transport follow-ups, and Phase 7 splat quality (spec: A shipped, B–E specced). The **travel-mode plan is shipped** (`travel-ap.sh` + `travel-ap-secrets.example.yaml`) and stays listed as the operational contract for the AP gating policy. The **splat resume handoff** is a crash-recovery record; its source plan file lived outside the repo on the authoring machine and is not recoverable — the handoff itself is the surviving record. The ICP study is **complete** (items 4 + 5 landed 2026-08-02) and stays listed here because its "closed questions" — no GPU-resident solve, never `6dof` — are the reason not to re-open them. **"Never `6dof`" independently corroborated 2026-08-06**: an `icp_mode="adaptive"` LiDAR-primary experiment (accept the 6dof rotation only when strongly confident + rotationally observable, else fall back to the IMU-locked translation solve) accepts LiDAR on ~98% of frames and drifts **17 m vs translation's 0.63 m** on `imuTranslationError.bin` — the 54×42 ToF's frame-to-model rotation is noisier than the SFLP IMU. Kept as a documented opt-in (default stays `translation`); see `slam/odometry.py`. The **framework exploration report** evaluates the viability of migrating Open3D to a potential `roomscan-native` C++ engine (`small_gicp` + `GTSAM` + `nvblox` + `SuGaR`). **Note:** We are not married to these specific libraries; they are experimental migration candidates to benchmark against empirical performance and quality gates before any cutover. |
| **Completed plans** | [Phase 1 binary protocol + visualizer](docs/superpowers/plans/completed/2026-07-07-phase1-binary-protocol-visualizer.md)<br>[Phase 2 raw streaming + PC transform](docs/superpowers/plans/completed/2026-07-08-phase2-raw-streaming-pc-transform.md)<br>[Phase 2.5 color/FoV/overlap](docs/superpowers/plans/completed/2026-07-08-phase2.5-color-fov-overlap.md)<br>[Phase 3 runtime configuration](docs/superpowers/plans/completed/2026-07-08-phase3-runtime-config-robustness.md)<br>[IKS4A1 HUB1 I3C](docs/superpowers/plans/completed/2026-07-09-iks4a1-hub1-multidevice-i3c.md)<br>[LSM orientation/env panel](docs/superpowers/plans/completed/2026-07-09-lsm6dsv16x-orientation-env-panel.md)<br>[Phase 3.5 GUI panel](docs/superpowers/plans/completed/2026-07-09-phase3.5-gui-panel.md)<br>[surface interpolation design](docs/superpowers/plans/completed/2026-07-09-surface-interpolation-design.md)<br>[surface interpolation implementation](docs/superpowers/plans/completed/2026-07-09-surface-interpolation-implementation.md)<br>[magnetometer yaw correction](docs/superpowers/plans/completed/2026-07-10-lsm6dsv16x-mag-yaw-correction.md)<br>[Phase 6 core SLAM](docs/superpowers/plans/completed/2026-07-10-phase6-slam.md)<br>[live-view FPS](docs/superpowers/plans/completed/2026-07-13-live-view-fps.md)<br>[panel UI redesign](docs/superpowers/plans/completed/2026-07-14-panel-ui-redesign.md)<br>[high-frame-rate plan](docs/superpowers/plans/completed/2026-07-31-high-framerate-and-manual-ranging-modes.md)<br>[Live/View + Detailed plan](docs/superpowers/plans/completed/2026-07-31-web-live-view-detailed.md)<br>[high-frame-rate implementation handoff](docs/superpowers/plans/completed/2026-08-03-high-framerate-implementation-handoff.md)<br>[#155 pose-buffer session ledger](docs/superpowers/plans/2026-08-12-155-pose-buffer-session.md) | Phases 1–4 and completed Phase 6 sub-work; desktop-panel work is historical because the web UI is primary. High-frame-rate/manual ranging: all 12 tasks landed and validated 2026-08-04 (its one outstanding gate, Task 6 wired-Ethernet, is tracked as an issue, not a reason to keep the plan active). The #155 ledger is the decision log + full three-arm ensemble evidence for the timestamped quat interpolation (closed 2026-08-12; follow-ups #180/#181) — read it before re-litigating the quat-phase question. |
| **Completed specifications** | [LSM orientation/env](docs/superpowers/specs/completed/2026-07-09-lsm6dsv16x-orientation-env-panel-design.md)<br>[magnetometer yaw correction](docs/superpowers/specs/completed/2026-07-10-lsm6dsv16x-mag-yaw-correction-design.md)<br>[Phase 6 core SLAM](docs/superpowers/specs/completed/2026-07-10-phase6-slam-design.md)<br>[viewer metrics HUD](docs/superpowers/specs/completed/2026-07-10-viewer-metrics-hud-design.md)<br>[live-view FPS](docs/superpowers/specs/completed/2026-07-13-live-view-fps-design.md)<br>[panel UI redesign](docs/superpowers/specs/completed/2026-07-13-panel-ui-redesign-design.md)<br>[vendored external dependencies](docs/superpowers/specs/completed/2026-07-15-vendor-external-deps-design.md)<br>[Web Phase 1 core instrument](docs/superpowers/specs/completed/2026-07-15-web-phase1-core-instrument-design.md)<br>[Web Phase 2 sensors](docs/superpowers/specs/completed/2026-07-16-web-phase2-sensors-design.md)<br>[Web Phase 3 recording/playback](docs/superpowers/specs/completed/2026-07-16-web-phase3-recording-playback-design.md)<br>[Web Phase 4 SLAM](docs/superpowers/specs/completed/2026-07-16-web-phase4-slam-design.md)<br>[3D mag-cal feedback](docs/superpowers/specs/completed/2026-07-29-magcal-3d-feedback-design.md)<br>[high-frame-rate specification](docs/superpowers/specs/completed/2026-07-31-high-framerate-and-manual-ranging-modes.md)<br>[Live/View + Detailed specification](docs/superpowers/specs/completed/2026-07-31-web-live-view-detailed-design.md) | Completed Phase 4 / Web / Phase 6 foundations and their measured outcomes are recorded in [`docs/roadmap-history.md`](docs/roadmap-history.md). |
| **Deprecated** | [GPU container service plan](docs/superpowers/plans/deprecated/2026-07-13-slam-gpu-container-service.md)<br>[GPU container service design](docs/superpowers/specs/deprecated/2026-07-13-slam-gpu-container-service-design.md) | Superseded by in-process local CUDA:0; the optional remote backend remains documented only as legacy support. |

## Work tracking

Forward-looking work and open defects moved to **GitHub Issues** (2026-08-10):
`gh issue list --repo hellosamblack/lidar-roomscanner`. Labels: `bug` (was `BUGS.md`), `work-item`
(was the Work-item register: `SLAM-`/`WEB-`/`SENS-`/`XPORT-`/`FW-`/`OFFLINE-`/`TOOL-`), and
`data-collection` (was the Data-collection queue, `DC-*`), each further tagged `area/<subsystem>`
and, where the legacy status needed more than open/closed, a `status/<nuance>` label
(`by-design`, `anomaly`, `vendor`, `mitigated`, `investigated`, `fix-unverified`, `blocked`,
`partial`). All 98 bug write-ups and the register/DC-queue entries were migrated verbatim as issue
bodies. Titles carry **no** ID prefix: the old `BUG-042:`/`SLAM-4:`/`DC-E:` prefixes were stripped
(2026-08-11, `migrate_issues.py strip-prefixes`) because they collided with GitHub's own `#NNN` —
the type is now denoted by the label alone. Each body keeps a `Legacy ID:` line so GitHub's search
still finds the old ID, and old-ID → issue mapping stays authoritative in
[`docs/issue-migration-map.md`](docs/issue-migration-map.md).

File a new item: `gh issue create --label bug|work-item|data-collection --label area/<area>` with a
plain `<verb>: <what>` title (no ID prefix — the label is the type, `#NNN` is the only ID).
Close one: `gh issue close <n> --reason completed` (or `"not planned"` for a by-design/anomaly/
investigated call — add the matching `status/*` label).

**Nothing closes on unverified work (2026-08-12, #173).** Before any close, ask *what would prove
this is fixed, and did I actually run it, today, against real data?* If not, the issue stays open
with `needs/operator` plus one subtype — `needs/capture`, `-network`, `-hardware`, `-eyes`,
`-decision` — paired with `status/fix-unverified` (code landed, verification pending) or
`status/blocked`. The `operator-request` skill owns that judgement and writes the owner's runbook as
an issue comment: plain language, strictly alternating `[Claude]`/`[You]` steps composed from its
step library, with a machine-readable footer naming the artifact and the gate. `operator_queue()`
lists what is outstanding, and `host/tests/test_operator_skill.py` pins the routing from
`status-sync`, `session-end`, `session-start` and `issue-fleet` — a gate nobody consults is worth
nothing, which is what `status/fix-unverified` had been until then. **One exception to "the label is
a promise the instructions exist" (2026-08-13, #175):** a `priority/later` hold may carry
`needs/operator` *before* its runbook is written, deliberately, so it stays findable while queued
behind the now/next tiers. `operator_queue()` reports that as `parked` and keeps it out of
`problems`; nothing else is excused, since a `now`/`next` hold, a malformed footer or an unreadable
comment thread each mean someone is waiting or a runbook exists and is broken.

**Working several issues at once (2026-08-12, #170).** The `issue-fleet` skill runs a fleet of
subagent workers across open issues, one git worktree each, under an owner-declared usage ceiling;
`fleet_plan()` picks a batch whose file footprints do not collide and `fleet_budget()` gates it.
This carries the one **exception to "subagents don't commit"**: a fleet worker commits `Refs #NNN`
on its own branch inside its own worktree, and the orchestrator still owns review, rebase,
`merge --ff-only`, every doc delta, every `gh` call and the single `Closes` commit. The rule and its
carve-out are stated in `docs/engineering-practices.md` → Branch discipline and in the `status-sync`
skill.

**A fleet run rotates itself (2026-08-13, #183, closed 2026-08-14).** An orchestrator's cost is
accumulated context, so `fleet_budget()` returns `rotate` at 300K and the skill hands off to a fresh
session (#182). The restart used to be manual, which meant the expensive path stayed the default.
`host/tools/fleet_run.py` now owns the chain: it spawns each successor from `.fleet/<run-id>.md`, and
stops on a permission denial, a weighted-token ceiling, `--max-sessions`, or no progress. Deliberately
a CLI, not an MCP tool — it has to outlive the session it rotates. **A real supervised run landed real
work unattended 2026-08-14** (`fleet-20260814-1716`: claimed and worked two issues, merged a fix with
a regression test, closed one out with a well-evidenced negative result, ran its own `session-end`) —
after six attempts and four live diagnostic probes surfaced and fixed a chain of environment-splitting
bugs the mechanism itself never had: root-vs-`sam` `$HOME` and file-ownership splits, a user-level
`rtk` hook rewriting Bash commands invisibly, a real `plan_fleet_live()` signature bug two sessions
had misdiagnosed as process staleness, an invalid directory-only allowlist rule, and — the largest
single cost — the `issue-fleet` skill's own instructions teaching a `cd X && Y` Bash pattern that is
denied outright unattended (fine interactively, where a denial is just a permission prompt). Full
trace in `docs/fleet-ledger.md` and #183's comment history.

## Completed phases

Full narratives, with measured outcomes, are in [`docs/roadmap-history.md`](docs/roadmap-history.md)
under their original names. The completed `roomscan-mcp` agent-tooling write-up moved there too.

| Phase | Status |
| --- | --- |
| Phase 0 — On-device transform + ASCII depth map | ✅ Complete |
| Phase 1 — Real-time 3D visualizer (binary protocol, USB CDC) | ✅ Complete |
| Phase 2 (+2.5) — Raw streaming + PC-side transform; color / FoV / overlap | ✅ Complete |
| Phase 3 (+3.5 / 3.6) — UI & runtime config; GUI panel; web-UI migration | ✅ Complete |
| Web Phases 1–7 — Three.js web app supplants `panel.py` (now the primary UI) | ✅ Complete |
| Phase 4 — X-NUCLEO-IKS4A1 integrated (IMU / env) | ✅ Complete |
| Phase 5 — Ethernet / UDP transport cutover (untethered) | ✅ Complete |
| Phase 5.5 — Transport hardening | ⏳ Partly done → [#130](https://github.com/hellosamblack/lidar-roomscanner/issues/130) |
| Sub-phase 6.G — SLAM GPU-memory hardening | ✅ Complete |
| Phase 6 — Real-time SLAM (PC) | ⏳ In progress → [#110](https://github.com/hellosamblack/lidar-roomscanner/issues/110) [#111](https://github.com/hellosamblack/lidar-roomscanner/issues/111) [#112](https://github.com/hellosamblack/lidar-roomscanner/issues/112) |
| Phase 7 — Offline post-processing | 🔜 Future → [#132](https://github.com/hellosamblack/lidar-roomscanner/issues/132), reshaped by the RTAB-Map decision; ingest contract landed 2026-08-13 ([#158](https://github.com/hellosamblack/lidar-roomscanner/issues/158) ✅ — `roomscan.splat.rtabmap` reader + `inspect-rtabmap` CLI, fixture-validated; real-export check rides [#161](https://github.com/hellosamblack/lidar-roomscanner/issues/161), SfM seeding is [#159](https://github.com/hellosamblack/lidar-roomscanner/issues/159)) |
