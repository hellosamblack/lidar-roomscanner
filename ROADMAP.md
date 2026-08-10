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
reference-firmware bug ledger, cross-cutting risks, the plans/specs register, the data-collection
queue, and the forward-looking **Work-item register** (type-prefixed IDs grouped by subsystem —
`SLAM-`, `SENS-`, `XPORT-`, `FW-`, `OFFLINE-`, `TOOL-`, `DC-`). The completed-phase narratives and
measured outcomes moved to [`docs/roadmap-history.md`](./docs/roadmap-history.md) (they keep their
`Phase N` names). Open **defects** are tracked in [`BUGS.md`](./BUGS.md).

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

## Plans and specifications register (audited 2026-08-02)

This is the complete inventory of `docs/superpowers/plans/` and `docs/superpowers/specs/`.
Active work stays at the directory root; historical records are retained under `completed/`, and
superseded records under `deprecated/`. The archive status applies to the document's stated scope,
not necessarily to every later phase of the product.

| Status | Documents | Roadmap tracking |
| --- | --- | --- |
| **Active** | [orientation / eCompass resume](docs/superpowers/plans/2026-07-29-orientation-resume.md)<br>[high-frame-rate plan](docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md)<br>[Live/View + Detailed plan](docs/superpowers/plans/2026-07-31-web-live-view-detailed.md)<br>[high-frame-rate specification](docs/superpowers/specs/2026-07-31-high-framerate-and-manual-ranging-modes.md)<br>[Live/View + Detailed specification](docs/superpowers/specs/2026-07-31-web-live-view-detailed-design.md)<br>[Rerun observability sidecar](docs/superpowers/specs/2026-08-02-rerun-observability-sidecar-design.md)<br>[SLAM compute & transport follow-ups](docs/superpowers/plans/2026-08-02-slam-compute-and-transport-followups.md)<br>[matched CUDA ICP / raycast study](docs/superpowers/plans/2026-08-02-cuda-icp-study.md)<br>[framework exploration research report](references/software/framerworkExploration/researchResults.md) | Phase 3 high-rate/manual follow-up; Phase 6 orientation/DC-E, Detailed-SLAM follow-ups, sub-phase 6.J, and the BUG-061 compute/transport follow-ups. The ICP study is **complete** (items 4 + 5 landed 2026-08-02) and stays listed here because its "closed questions" — no GPU-resident solve, never `6dof` — are the reason not to re-open them. **"Never `6dof`" independently corroborated 2026-08-06**: an `icp_mode="adaptive"` LiDAR-primary experiment (accept the 6dof rotation only when strongly confident + rotationally observable, else fall back to the IMU-locked translation solve) accepts LiDAR on ~98% of frames and drifts **17 m vs translation's 0.63 m** on `imuTranslationError.bin` — the 54×42 ToF's frame-to-model rotation is noisier than the SFLP IMU. Kept as a documented opt-in (default stays `translation`); see `slam/odometry.py`. The **framework exploration report** evaluates the viability of migrating Open3D to a potential `roomscan-native` C++ engine (`small_gicp` + `GTSAM` + `nvblox` + `SuGaR`). **Note:** We are not married to these specific libraries; they are experimental migration candidates to benchmark against empirical performance and quality gates before any cutover. |
| **Completed plans** | [Phase 1 binary protocol + visualizer](docs/superpowers/plans/completed/2026-07-07-phase1-binary-protocol-visualizer.md)<br>[Phase 2 raw streaming + PC transform](docs/superpowers/plans/completed/2026-07-08-phase2-raw-streaming-pc-transform.md)<br>[Phase 2.5 color/FoV/overlap](docs/superpowers/plans/completed/2026-07-08-phase2.5-color-fov-overlap.md)<br>[Phase 3 runtime configuration](docs/superpowers/plans/completed/2026-07-08-phase3-runtime-config-robustness.md)<br>[IKS4A1 HUB1 I3C](docs/superpowers/plans/completed/2026-07-09-iks4a1-hub1-multidevice-i3c.md)<br>[LSM orientation/env panel](docs/superpowers/plans/completed/2026-07-09-lsm6dsv16x-orientation-env-panel.md)<br>[Phase 3.5 GUI panel](docs/superpowers/plans/completed/2026-07-09-phase3.5-gui-panel.md)<br>[surface interpolation design](docs/superpowers/plans/completed/2026-07-09-surface-interpolation-design.md)<br>[surface interpolation implementation](docs/superpowers/plans/completed/2026-07-09-surface-interpolation-implementation.md)<br>[magnetometer yaw correction](docs/superpowers/plans/completed/2026-07-10-lsm6dsv16x-mag-yaw-correction.md)<br>[Phase 6 core SLAM](docs/superpowers/plans/completed/2026-07-10-phase6-slam.md)<br>[live-view FPS](docs/superpowers/plans/completed/2026-07-13-live-view-fps.md)<br>[panel UI redesign](docs/superpowers/plans/completed/2026-07-14-panel-ui-redesign.md) | Phases 1–4 and completed Phase 6 sub-work; desktop-panel work is historical because the web UI is primary. |
| **Completed specifications** | [LSM orientation/env](docs/superpowers/specs/completed/2026-07-09-lsm6dsv16x-orientation-env-panel-design.md)<br>[magnetometer yaw correction](docs/superpowers/specs/completed/2026-07-10-lsm6dsv16x-mag-yaw-correction-design.md)<br>[Phase 6 core SLAM](docs/superpowers/specs/completed/2026-07-10-phase6-slam-design.md)<br>[viewer metrics HUD](docs/superpowers/specs/completed/2026-07-10-viewer-metrics-hud-design.md)<br>[live-view FPS](docs/superpowers/specs/completed/2026-07-13-live-view-fps-design.md)<br>[panel UI redesign](docs/superpowers/specs/completed/2026-07-13-panel-ui-redesign-design.md)<br>[vendored external dependencies](docs/superpowers/specs/completed/2026-07-15-vendor-external-deps-design.md)<br>[Web Phase 1 core instrument](docs/superpowers/specs/completed/2026-07-15-web-phase1-core-instrument-design.md)<br>[Web Phase 2 sensors](docs/superpowers/specs/completed/2026-07-16-web-phase2-sensors-design.md)<br>[Web Phase 3 recording/playback](docs/superpowers/specs/completed/2026-07-16-web-phase3-recording-playback-design.md)<br>[Web Phase 4 SLAM](docs/superpowers/specs/completed/2026-07-16-web-phase4-slam-design.md)<br>[3D mag-cal feedback](docs/superpowers/specs/completed/2026-07-29-magcal-3d-feedback-design.md) | Completed Phase 4 / Web / Phase 6 foundations and their measured outcomes are recorded in the corresponding phase sections below. |
| **Deprecated** | [GPU container service plan](docs/superpowers/plans/deprecated/2026-07-13-slam-gpu-container-service.md)<br>[GPU container service design](docs/superpowers/specs/deprecated/2026-07-13-slam-gpu-container-service-design.md) | Superseded by in-process local CUDA:0; the optional remote backend remains documented only as legacy support. |

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
| **DC-B** | **Multi-room closed loop ×2** — 2–3 rooms or a corridor circuit, 3–5 min each, two takes of the *same* route | Phase 6.D items 3–4, the loop-closure go/no-go. The baseline is single-room only (~3% over 24 m) and the paired 95%-CI gate has only the two coffee-room circuits to score. Loop closure is supposed to earn its keep on **multi-room** trajectories — nothing in `captures/` tests that | Start parked on a marked pose, walk the route, return to the **same** marked pose. Revisit at least one area mid-route (that is what a pose-proximity edge needs). Two takes so the gate is paired | 0 lost frames, stream 9, byte-clean. ~135 MB per 5 min | ❌ **collected ×3, gate FAILS on "0 lost frames"** (`DebugCapB1/B2/B3`, 2026-07-31) — 2.29/4.28/9.35% transport loss, **BUG-049**. B1 usable as a provisional baseline (**2.71 ± 0.58 m over 75.3 m = 3.6%**); B3 is 9.6% on the *same route*. **Re-record after BUG-049** — see [roadmap-history: DC-B](docs/roadmap-history.md) |
| **DC-C** | **Tracking-loss stress scan** — one room, ~90 s | Relocalization (Phase 6.D, explicitly open: the retry survives a bad *frame*, not a bad *second*). No capture contains a real tracking-kill event | Mid-scan, kill tracking deliberately 2–3×: palm over the sensor ~2 s, or point at a blank surface <20 cm away — then **return to already-mapped geometry** and continue | Byte-clean; the kill events identifiable on replay. This is a fixture, not a good scan — it should look bad | ✅ **PASS as a fixture** (`DebugCapC.bin`, 2026-07-31) — 0 CRC, 376–467 lost/run, freezes of 82–120 frames (2.7–4.0 s), 70–133 escalations. **Every run recovered** (0 died) because the protocol's "return to already-mapped geometry" is what relocalization would otherwise be for |
| **DC-D** | **Flat-field pan** — 20 s over a uniform matte surface | Phase 2.5 follow-up: reflectance carries ~18% per-zone FPN and the correction is **built and shipped-disabled** waiting only on this (`docs/flatfield-calibration.md`) | Blank painted wall / foam board / grey card at ~0.5–1 m, roughly perpendicular, **slowly panning the whole time**. A static capture is invalid — it bakes scene texture into the "correction" | ≥100 panned frames; `build_flatfield` residual in the low tens of percent, gains comfortably inside [0.5, 1.6]. Gains near the [0.33, 3.0] clip bounds ⇒ recapture | ✅ **PASS** (`DebugCapD.bin`, 2026-07-31) — residual **7.3%**, gains 0.754–1.224 (mean 1.000, sd 7.1%), **all 2268 zones** inside [0.5, 1.6], none near the clip bounds. ~490 panned frames at 14.9 °/s. ⚠️ **"Ready to enable" downgraded 2026-08-04**: DC-D is a *single-mode* pan. The stationary ceiling study (`calibration/flatfield/2026-08-04/`) found the FPN is **mode-family-specific** — Precision 6.6% vs Ambient 3.7%, map correlation only 0.34, cross-application leaves ~6% — so one global `flatfield_path` mis-corrects whichever family it wasn't built from. Enabling now requires **mode-aware map selection** and a pan **per mode family** (→ DC-D2), not just setting the path |
| **DC-D2** | **Per-mode flat-field pan set + held-out distance** — ~20 s each | Supersedes DC-D as the enable gate. The 2026-08-04 study proved (a) Precision and Ambient need different maps, and (b) the stationary holdout only proves *temporal stability*, not *scene-independence* — a static map can bake in the ceiling's own texture/illumination field | Slow pan over a uniform matte wall for **each mode family** (at minimum one Precision-family and one Ambient-family config), then **one held-out pan at a different distance** (e.g. build at ~0.6 m, validate at ~1.4 m). Panning mandatory; a static capture is invalid | Per family: `build_flatfield` residual in the low tens of percent, gains inside [0.5, 1.6]. **Held-out distance:** applying the built map to the other-distance pan still reduces FPN (scene-independence, the control DC-D/2026-08-04 could not provide). Precision map on Ambient (or vice versa) must be visibly worse than the matched map | ⚠️ **6 pans collected + analyzed 2026-08-04 (`calibration/flatfield/2026-08-04/ffpan_20260804_report.md`), gate PARTLY met.** ✅ **5/6 clean valid candidates** (0 CRC, slow multi-height pans, gains [0.78,1.20], within-family cross-residual ~2%). ❌ `ambientRegular8or16FFpan` **contaminated** (raw FPN 11.4%, illumination-dominated gain, cross-apply 8–11%, corr 0.40–0.63) → **recapture**. **Findings:** mode-family split **confirmed but milder** (~4% cross-family vs ~2% within, not the static study's ~6%/0.34); a **single-height map does NOT transfer across distance** (near-map on far frames worse than raw — bakes in single-standoff scene) so a **full multi-height pan is mandatory**; honest cross-scene floor **~2%**, not the ~1.3% self-map. **Scene-independence CLOSED 2026-08-04** (`crossroom_20260804_report.md`): two pans in a larger second room (`{precision,ambient}Regular8msFFpanLarge.bin`) — a room-A map flattens the unseen room-B data symmetrically: **Ambient 5.8%→2.4% (corr 0.945), Precision 6.2%→3.0% (corr 0.920)**, both wall-rejected (operator flagged wall at the turnarounds; 17–25% of frames rejected by a >25 mm per-frame plane-RMS test, 57–79% of rejects in the first/last 10%). Honest cross-scene floor **~2.4/3.0%** (removes ~50–60% of FPN), not the ~78% the self-scored study implied; mode split reproduces a 3rd time (matched 2.4/3.0% vs cross-family 3.7/4.2%). Exposure of the two `8or16` files unresolved (best guess 8 ms, immaterial). **Mode-aware selection SHIPPED 2026-08-04** (`FlatFieldSet` + `[viewer] flatfield_precision_path`/`flatfield_ambient_path`, map picked by the device ranging mode via `_commit_ranging_ack`, web Device-card **Calibrated/Uncalibrated** toggle — host-shared server state, config in a `[calibration]` table, not a per-client pref). **To turn on:** add a `[calibration]` table to `roomscan.toml` with `flatfield_precision_path`/`flatfield_ambient_path` pointing at the room-A `ffpan_20260804_*` maps and restart the server. |
| **DC-E** | **Braced fixed-heading tilt sweep** — ~2 min | `docs/superpowers/plans/2026-07-29-orientation-resume.md` §4.6: BUG-030's closure proved the calibration's **magnitude**, not its **direction**. An ellipsoid fit is ambiguous up to a rotation (DT0103) — every sample on a perfect sphere while the field vector is systematically rotated. Bounded at ~2.5° by the near-spherical soft iron; measured at nothing | Hand-held, off the tripod. Pick **one fixed compass bearing** and keep pointing at it. Sweep tilt level → 45° → vertical, **holding each ~15 s**. Two cycles | `mag_check` tilt table flat (expected) **and** `absolute_heading` agreeing across every hold. Disagreement ⇒ implement DT0103's accelerometer-assisted fit | ⚠️ **collected to spec, gate FAILS on the tilt-ramp clause** (`DebugCapE.bin`, 2026-07-31) — 7 holds of 15–18 s at tilt 3.9/46.1/90.3/50.3/4.1/47.8/90.9°, exactly two cycles. Ramp **1.72×** (GOOD < 1.10). ⚠️ The heading clause is **partly artifact** — see BUG-048; re-score it after the singularity fix before sizing DT0103. ~~**The singularity fix landed 2026-07-31 (BUG-051, `yaw_twist_deg`) — DC-E's heading clause is now re-scorable and that re-score is the next step here.**~~ **Do NOT re-score with that build (BUG-058, 2026-08-01): `yaw_twist_deg` made `absolute_heading` track ROLL at slope −0.978, and DC-E is a tilt sweep held at one bearing — exactly the geometry that heading was wrong for. The re-score is still the next step here, but it needs the BUG-058 build (`boresight_bearing_deg` − `magnetic_north_bearing_deg`), and note that build returns `None` for the vertical holds, where no bearing exists.** Note the artifact was larger than BUG-048 estimated: an 18.4° systematic bias at the operating pose, not only noise near lock |
| **DC-F** | **Controlled pan set** — 3 takes × ~60 s | The two claims currently inferred from stationary data plus arithmetic: applying the measured **+7.76 ms quat phase lead** (on the wire since BUG-031, nothing consumes it — now the largest motion-error term), and the `imufusion` A/B (built, gated off, no capture carries orientation ground truth). Also resume-doc §4.5 | Brace against a repeatable start (a corner, taped marks), pan to a repeatable end, hold. One take each at roughly **slow ~20 °/s / medium ~50 °/s / fast ~100 °/s**. 10 s stationary at both ends of every take | Endpoints repeatable enough that A→B is the same rotation across takes; 0 CRC; stream 11 present. The stationary bookends give the noise floor for free | ⚠️ **gate PASSES, purpose NOT unblocked** (`DebugCapF.bin`, 2026-07-31) — collected above spec: **4** pans at 19/25/36/89 °/s with 5 bookend holds of ~11 s. Pans agree to **1.42°** of nominal 90°. But the bookends' own noise floor (0.3–1.7°) **exceeds** the predicted phase-lead effect (0.15–0.69°), so the rate-vs-error test is underpowered, not falsifying. Phase offset re-measured here as **+5.13 ms** (sign confirmed *lead*), vs the +7.76 ms on record |
| **DC-G** | **Recorded magnetometer tumble** — 30–45 s | Magcal regression fixtures. The tumble that closed BUG-030 went straight through the modal, so **no capture contains one** — covered-shell tests still use a synthetic fixture (`tilt_sweep_20260729.bin` fills 2 of 92 cells, `web_20260729_061440.bin` fills 6) | Open the calibration modal, hit Record, free-tumble to good coverage, stop | ≥60 of 92 shell cells covered. Low priority — test data, not a decision | ⬜ open |
| **DC-H** | **USB CDC connect transient** — 5 × 15 s | **BUG-005**: fix implemented 2026-07-30, **the code path has never executed**. `CAFE:4001` does not enumerate on the headless host (USB_USER is powered from the battery bridge) and `/dev/ttyACM*` return `root:root` mode 0 after every replug | Needs the board's USB_USER cable into a machine that can open the port (udev rule or run as root). Fresh connect, `host/tools/capture.py --seconds 15`, five times | `capture_analyze` reports **0** CRC failures in the connect region (today: exactly 1) and the first frame after connect is CALIB | ⬜ open |
| **DC-I** | **Phase 7 seed set** | COLMAP pose priors + depth-regularized 3DGS | **Do not collect yet** — needs a rigid phone/webcam mount and a hand-eye extrinsic calibration, neither of which is designed. Listed so it is not a surprise when Phase 6 closes. *(2026-08-07: **OFFLINE-4** covers the phone-software half — an ARCore capture app giving posed 4K video + depth — and is deliberately **not** blocked on this row, because the video-only splat path needs no ToF fusion. The mount, the hand-eye extrinsic and the ARCore↔TIM2 clock alignment stay here.)* | — | ⬜ blocked on design |
| **DC-J** | **Specular / mirror behaviour** — 42 s, room scan then dwell on a large mirror | Nothing in the repo analysed specular surfaces; "mirror" elsewhere means the **UI view mode**. Unplanned — the owner recorded it to see what would happen | Normal room scan, then point at a large mirror and dwell | *(added retroactively)* Does the map gain phantom geometry; does tracking survive | ✅ **characterized, no defect** (`DebugCapMirror.bin`, 2026-07-31) — see [roadmap-history: DC-J](docs/roadmap-history.md) |

The full write-ups for **DC-B** (multi-room result + why 6.D is blocked) and **DC-J** (mirror
behaviour) moved to [`docs/roadmap-history.md`](docs/roadmap-history.md) when the roadmap was split.

## Work-item register

Forward-looking work, grouped by subsystem, with **type-prefixed IDs** (per-type counter; next free
ID within the type; never reused) using the same `Area` vocabulary as [`BUGS.md`](BUGS.md). Completed
phases live in [`docs/roadmap-history.md`](docs/roadmap-history.md) and keep their original `Phase N`
names; open **defects** live in [`BUGS.md`](BUGS.md). Legacy sub-phase labels are kept in parentheses
so existing references (`6.D`, `6.I`, …) still resolve.

> **Note on framework exploration items (`SLAM-4`..`7`, `OFFLINE-2`..`3`):** These proposed work items
> represent **experimental candidates and evaluative benchmarks**, not locked-in decisions. We are not
> married to any specific library (`small_gicp`, `GTSAM`, `nvblox`, `SuGaR`, etc.); each item will be
> experimentally prototyped and benchmarked against strict empirical performance, stability, and quality
> gates before any permanent production cutover is made.

### SLAM (`SLAM-`)

- **SLAM-1 — Drift correction / loop-closure evaluation** *(was sub-phase 6.D; owner target)*.
  ICP→IMU yaw feedback, then a go/no-go on pose-graph loop closure vs the ~3% frame-to-model baseline.
  **Blocked:** the multi-room gate (DC-B) fails on transport loss (**BUG-049**) — fix that and
  re-record before scoring loop closure. Detail: `docs/roadmap-history.md` → "Sub-phase 6.D".
- **SLAM-2 — Lift the mesh-extraction ceiling** *(was sub-phase 6.I; proposed)*. Chunked
  extract-and-stitch or a non-Open3D mesher, to get past **BUG-053** (marching cubes crashes above
  ~260k blocks) so Detailed can run finer than 10 mm voxels on room-sized captures. Detail: "Sub-phase 6.I".
- **SLAM-3 — Relocalization** *(open, architectural)*. The ICP retry (BUG-036) survives a bad *frame*,
  not a bad *second*; there is no recovery from a real tracking-kill (see fixture DC-C). *(2026-08-06:
  also the ceiling on BUG-067 — the accel ZUPT fixed the tripod's net drift/instability but a pure
  pan still over-reports total *path* 13.1 → 11.5 m because the coherence veto won't hold through
  partly-coherent fabricated drift; only relocalization or a full zero-velocity re-anchor removes the
  rest. See bugs/BUG-067.md.)*
- **SLAM-4 — Alternative ICP registration engine (`small_gicp`)** *(proposed)*.
  Replace Open3D ICP registration with `small_gicp` (C++/CUDA GICP with custom Pybind11 wrapper). Features 3-DoF translation constraints locking rotation to the LSM6DSV16X SFLP IMU, 8-bit IR intensity cost functions, and `Linear iVox` scan-to-model odometry, achieving <2 ms execution per frame at 46 Hz. See [`references/software/framerworkExploration/researchResults.md`](references/software/framerworkExploration/researchResults.md#L36-L44).
- **SLAM-5 — Alternative GPU/sparse TSDF meshing engine (`nvblox`)** *(proposed)*.
  Replace Open3D `VoxelBlockGrid` with standalone C++ NVIDIA `nvblox` (CUDA TSDF/ESDF + Marching Cubes). Bypasses Open3D's ~260k block crash ceiling (**BUG-053**), uses dynamic voxel hashing with spatial VRAM pruning, releases the Python GIL during mesh extraction, and exposes zero-copy DLPack / `__cuda_array_interface__` pointers to `roomscan-web`. See [`references/software/framerworkExploration/researchResults.md`](references/software/framerworkExploration/researchResults.md#L60-L67).
- **SLAM-6 — Factor-graph multi-sensor backend (`GTSAM`)** *(proposed)*.
  Integrate `GTSAM` (`iSAM2` factor graph running in a background C++ thread at 46 Hz). Fuses 3-DoF relative ICP steps, IMU pre-integration (`PreintegratedImuMeasurements`), magnetometer heading, barometer altitude, and VGICP submap loop-closure edges into a mathematically rigorous SLAM backend. See [`references/software/framerworkExploration/researchResults.md`](references/software/framerworkExploration/researchResults.md#L50-L55).
- **SLAM-7 — Native C++ Core Engine (`roomscan-native`)** *(proposed)*.
  Encapsulate UDP/CDC sensor stream reading, `small_gicp` odometry, `GTSAM` iSAM2 factor graph, and `nvblox` CUDA calls inside a single C++ library/daemon (`roomscan-native`). Pybind11 acts purely as a read-only interface exposing optimized trajectory and DLPack mesh pointers to FastAPI / Three.js. See [`references/software/framerworkExploration/researchResults.md`](references/software/framerworkExploration/researchResults.md#L130-L137).
- **SLAM-8 — Distributed GPU compute backend (LAN multi-GPU)** *(proposed)*. Offload heavy SLAM/splat workloads to a faster GPU on the LAN (available: i9-13900H + RTX 4080 Mobile over gigabit ethernet). Candidates: **(a) Detailed SLAM** — send frame batches to remote worker running ICP integrate/raycast/extract on CUDA, stream results back for replay display; **(b) Live ephemeral SLAM** — exploratory scans or low-latency feedback without the local 7 ms/frame budget; **(c) 3D Gaussian Splat training** — multi-GPU parallel (local + remote) or full offload to amortize 15–30 min trainer across both GPUs. **Infrastructure needs:** gRPC or `aiozmq` transport for pose/mesh/loss backhaul; local-first fallback for displays when remote unavailable; worker health/queue depth instrumentation so the UI shows whether offload is active. **Network available:** Gigabit ethernet (~125 MB/s theoretical, ~80–100 MB/s sustained) makes (c) feasible; mesh backhaul is trivial, splat loss throughput is the real question. **Gating questions before prototyping:** (1) Is inter-machine latency (network roundtrip) acceptable for live display at 30 Hz? (2) Does moving frame-by-frame integration off the reader thread (RPC wait) starve local point-cloud rendering? (3) Is splat loss throughput under the gigabit budget? Start with a feasibility spike: measure E2E latency of a single frame (capture → integrate → extract → transmit) and decide (a) vs (b) precedence.

### Web UI (`WEB-`)

- **WEB-1 — Simplify the Record controls.** Move the Record section to the top-right
  controls instead of giving it a separate sidebar section.
- **WEB-2 — Allow completed captures to be discarded.** Add an explicit discard action
  to the completed-capture workflow, with confirmation and removal from the capture list.
- **WEB-3 — Show FileHub battery in the top bar.** Add FileHub battery state beside
  communication status, including unavailable/unknown handling.
- **WEB-4 — Make View mode capture-focused.** When switching to View, minimize every
  panel except the Captures pane so replay can be selected and controlled without
  unrelated live controls. This complements **BUG-090**.
- **WEB-5 — WASD free-camera navigation.** Add keyboard controls for 3D navigation:
  **W/A/S/D** for forward/left/backward/right, **Spacebar** for vertical ascent,
  **Ctrl/C** for vertical descent. Decouple camera from auto-follow when any key is
  pressed; restore auto-follow on release of all navigation keys.
- **WEB-6 — Floating playback panel at 3D view bottom.** Move playback controls
  (seek bar, play/pause, speed) from the sidebar to a new floating panel docked at
  the bottom of the 3D viewport, styled like a video player. Complements the planned
  Record-on-top-bar (WEB-1) and Discard-completed (WEB-2) features.
- **WEB-7 — Floorplan view with dimensions and measure tool.** Add a floorplan display
  mode showing a top-down orthographic projection of the reconstructed space with
  estimated dimensions, grid overlay, and an interactive measure tool (click two points
  to measure distance). Useful for room planning and area estimation.
- **WEB-8 — Compact side-rail layout with progress-bar metrics.** Reorganize Sensors
  and Streams cards to reduce vertical footprint: (a) **Sensors** uses a 2×2 grid layout
  showing roll/tilt/heading/fusion corrections in one row instead of flat rows; (b) each
  metric visualized as a horizontal progress bar (like bandwidth gauge) with **jitter**
  and **worst-case** statistics displayed inline; (c) add a **Reset Stats** button to
  zero the worst-case counters. Reduces scroll burden on the dock band while improving
  signal visibility. Related: **BUG-033** (Sensors card outgrew the dock; this is the
  deliberate reorg answer).

### Sensors / IMU (`SENS-`)

- **SENS-1 — Apply the +7.76 ms quat-phase lead.** On the wire since BUG-031 (`quat_mid_ticks`).
  *(2026-08-06: now IMPLEMENTED as an opt-in SLAM lever — `[slam] apply_quat_phase`, rolls the
  batch-midpoint quat back to the frame instant with the gyro; offline/Detailed only, stream 13 not
  in the live reader. Measured on `imuTranslationError` it made the tripod WORSE and bistable
  (0.211 ± 0.353 vs 0.121 ± 0.069), so it is **default off** — the offset is real but not the
  dominant fabrication term there. Still wants a before/after on a moving stream-13 capture (**DC-F**)
  before the UI orientation adopts it. See bugs/BUG-067.md resolution.)*
- **SENS-2 — Enable the `imufusion` complementary filter.** Built, gated off; blocked on orientation
  ground truth (**DC-F**).
- **SENS-3 — Validate heading *direction*.** |B| flatness proved magnitude, not direction (DT0103
  ambiguity); needs the braced fixed-heading tilt sweep (**DC-E**, re-scored on the BUG-058 build) and
  possibly an accelerometer-assisted fit.
- **SENS-4 — Stream SHT40 humidity.** Firmware work, gated on a host consumer existing.

### Transport (`XPORT-`)

- **XPORT-1 — Compression + pacer decisions** *(was Phase 5.5, two decisions open)*. `zstd`/`lz4`
  go/no-go on the firmware side and the TX-pacer measurement. Detail: `docs/roadmap-history.md` → "Phase 5.5".
- **XPORT-2 — Multi-second whole-group frame loss.** Tracked as **BUG-049** (host/transport); blocks
  DC-B / SLAM-1. Kept here as a pointer, not a duplicate entry.

### Firmware / UX (`FW-`)

- **FW-1 — Audible coverage feedback ("geiger-counter" buzzer)** *(was sub-phase 6.H; proposed)*.
  A host→device signal that clicks on TSDF novelty so a handheld operator hears coverage without a
  screen. Detail: `docs/roadmap-history.md` → "Sub-phase 6.H".

### Offline (`OFFLINE-`)

- **OFFLINE-1 — COLMAP pose priors + depth-regularized 3DGS** *(was Phase 7)*. Blocked on **DC-I**
  (rigid phone/webcam mount + hand-eye extrinsic — undesigned). Detail: `docs/roadmap-history.md` → "Phase 7".
- **OFFLINE-2 — Surface-Aligned Gaussian Splatting (`SuGaR`)** *(proposed)*.
  Ingest GTSAM trajectory (`transforms.json`) and dToF seed cloud into `SuGaR` to optimize surface-aligned 3D Gaussians from 4K phone video without COLMAP SfM. Extract watertight, closed Poisson CAD surface meshes (`.ply`/`.obj`) via `extract_mesh.py --project_mesh_on_surface_points`, applying hand-eye extrinsic calibration $T_{\text{world\_camera}} = T_{\text{world\_lidar}} \times T_{\text{lidar\_camera}}$. See [`references/software/framerworkExploration/researchResults.md`](references/software/framerworkExploration/researchResults.md#L78-L85).
- **OFFLINE-3 — Modular depth-regularized 3DGS framework (`Nerfstudio` / `Splatfacto`)** *(proposed)*.
  Benchmark `Nerfstudio`'s `depth-nerfstudio` / `splatfacto` using dToF metric depth priors for photorealistic novel view synthesis and rendering. See [`references/software/framerworkExploration/researchResults.md`](references/software/framerworkExploration/researchResults.md#L86-L90).
- **OFFLINE-4 — Android capture app (Pixel 10 Pro XL) — high-res video + ARCore pose/depth** *(proposed, owner 2026-08-07)*.
  A phone-side companion app on the [ARCore Android SDK](https://github.com/google-ar/arcore-android-sdk)
  that records, in one take, **high-resolution video + per-frame camera pose + intrinsics + ARCore
  depth/confidence**. The point is to hand OFFLINE-1/2/3 a *posed* image set instead of a bag of
  frames, so the 3DGS pass no longer depends on COLMAP SfM succeeding on the featureless painted
  walls this scanner is aimed at — the exact failure mode already measured in the standalone splat
  pipeline (Sam Office registered **206 of 287 frames, 72%**; see `host/src/roomscan/splat/`).

  **What it produces.** Two candidate outputs, to be decided by prototype, not up front:
  1. **ARCore MP4 via the Recording & Playback API** — one container holding the CPU-image video,
     the depth-map track, and **custom data tracks** (`Frame.recordTrackData` / `getUpdatedTrackData`)
     carrying per-frame pose + timestamps. Self-describing and replayable on-device, but needs a
     host-side demuxer.
  2. **Plain video + a sidecar** — poses from `Camera.getDisplayOrientedPose()`, intrinsics from
     `Camera.getImageIntrinsics()`, depth PNGs. Directly ingestible: it is already the shape
     `roomscan.splat`'s trainer and Nerfstudio's `transforms.json` want.

  **Depth is a regularizer here, not metric truth — verify before relying on it.** ARCore depth is
  depth-from-motion + ML unless the device carries a hardware depth sensor; `Frame.acquireDepthImage16Bits`
  / `acquireRawDepthImage16Bits` + `acquireRawDepthConfidenceImage` return DEPTH16 millimetres at a
  small resolution (~160×120 raw), best between ~0.5–5 m, and explicitly imprecise on featureless
  white walls. **Whether the Pixel 10 Pro XL has a hardware ToF at all is unverified** — check
  `CameraConfig.getDepthSensorUsage()` on the actual handset rather than assuming; Pixel phones have
  historically shipped without one. The rig's VL53L9CX stays the metric source.

  **The real risk is 4K *concurrent with* an AR session, and it is a measurement.** ARCore owns the
  camera configuration; simultaneous high-res capture goes through `SharedCamera` (Camera2 interop,
  and the app must not issue its own `setRepeatingRequest`). Google documents only that high-end
  phones support ~2 YUV CPU streams + 1 GPU stream at up to 1080p. **First task is therefore to
  enumerate `CameraConfigFilter` on the handset and report the achievable (video resolution, fps,
  depth on/off) combinations** — if 4K and live depth turn out to be mutually exclusive, that is a
  finding to record, not a reason to fake either one.

  **Sequencing — this is not blocked on DC-I.** The video-only path improves the shipped
  `roomscan.splat` pipeline immediately (poses seed or replace SfM). ToF *fusion* still needs the
  rigid mount + hand-eye extrinsic, and clock alignment between ARCore's `CLOCK_MONOTONIC`
  nanoseconds and the rig's TIM2 µs frame clock — all of which stay under **DC-I**.

  **Gates before it counts as done:** (a) a capture from a real room where ARCore-posed
  reconstruction registers materially more frames than COLMAP-only on the *same* footage;
  (b) the camera-config enumeration above written down; (c) an MCP/host ingest path, per CLAUDE.md's
  "new capability lands as an MCP tool" rule.

  **Alternative avenue — [8th Wall](https://github.com/8thwall/8thwall)** *(noted, unexplored,
  2026-08-07)*. A WebAR SLAM engine (Niantic-owned), now self-hostable and open source, that runs in
  the phone browser — no native app install, and cross-platform (iOS + Android) where ARCore is
  Android-only. Worth a look if the native `SharedCamera` risk above (task-first: enumerate
  `CameraConfigFilter`) turns out to block 4K+depth concurrency, or if an iOS capture path becomes
  wanted later. Tradeoffs to weigh before committing: pose/depth come through its own JS API rather
  than `Frame.acquireRawDepthImage16Bits`, and a browser capture pipeline has to solve the same
  high-res-video-plus-pose recording problem ARCore's `Recording & Playback API` already solves
  natively — unproven whether WebAR exposes an equivalent. Not prototyped; ARCore stays the primary
  path above until/unless a concrete blocker there makes this worth spiking.

### Tooling / observability (`TOOL-`)

- **TOOL-1 — Rerun multimodal observability sidecar** *(was sub-phase 6.J; proposed)*. A bounded,
  disabled-by-default developer diagnostic; raw `RSCN` `.bin` stays authoritative. Detail:
  `docs/roadmap-history.md` → "Sub-phase 6.J".

### Data-collection (`DC-`)

Owner-collected captures that unblock the items above — see the **Data-collection queue** table above.
Still open: **DC-G** (magcal tumble fixture), **DC-H** (USB CDC connect transient, BUG-005), **DC-I**
(Phase 7 seed set, blocked on design), and a re-record of **DC-B** after BUG-049.

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
| Phase 5.5 — Transport hardening | ⏳ Partly done → XPORT-1 |
| Sub-phase 6.G — SLAM GPU-memory hardening | ✅ Complete |
| Phase 6 — Real-time SLAM (PC) | ⏳ In progress → SLAM-1…3 |
| Phase 7 — Offline post-processing | 🔜 Future → OFFLINE-1 |
