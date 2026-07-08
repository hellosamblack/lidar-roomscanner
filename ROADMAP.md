# Roadmap — 53L9A1 3D Room Mapping

Product goal: a **tethered handheld 3D room scanner**. The STM32H563 streams timestamped sensor
frames to a PC running real-time SLAM (Open3D Tensor G-ICP + TSDF); an offline pass fuses 4K phone
video into a ToF-seeded 3D Gaussian Splat. Full design + critical review:
[`references/roadmapResearch.md`](./references/roadmapResearch.md).

Active development happens in this `roomscanner/` workspace. The existing STM32 firmware is **read-only
reference** in the sibling `53L9A1/` package; firmware paths below (`Src/…`) are relative to
`../53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/` (aka `<APP>`).
Engineering conventions live in [`docs/engineering-practices.md`](./docs/engineering-practices.md).

## Overriding architecture decisions

- **Transport: Ethernet is the production link, not USB.** The board has a 10/100 MAC (RMII pins already
  `AF11_ETH` in `Src/main.c`; MAC + lwIP not yet enabled). Target = lwIP/UDP with hardware PTP
  (IEEE 1588) timestamping. Native USB CDC (`USB_DRD_FS`) is a bring-up/fallback link only. This voids the
  USB-bandwidth and timestamp-drift bottlenecks in `references/roadmapResearch.md`.
- **Sensors: X-NUCLEO-IKS4A1** adds IMU (LSM6DSV16X, hardware SFLP orientation), magnetometer (yaw-drift
  correction), barometer (Z-drift constraint), temp/humidity (thermal comp). Not yet in code. Bus-sharing
  is resolved — IKS4A1 rides the ToF's I3C1 bus as legacy-I2C targets (shared PB8/PB9), no separate
  peripheral; stacking recipe + bench checklist in `docs/iks4a1-stacking.md`.
- **Sequencing rule (owner):** mature the visualizer + UI/config on the **ToF sensor alone** before adding
  the IKS4A1 board.
- **Protocol rule:** design the frame protocol transport-agnostic from day one —
  `magic + version + seq + timestamp + payload + CRC32`, multi-stream, little-endian — so the Ethernet
  cutover (Phase 4) is plumbing, not a redesign. Spec lives in `docs/protocol.md`; any wire change bumps
  the version and follows the `protocol-change` skill checklist.
- **Firmware fork rule:** our firmware lives in `roomscanner/firmware/` as a copy of `<APP>` that
  references the `53L9A1/` package in place for shared Drivers/Middlewares/Utilities. `<APP>` itself is
  never edited. Our copy is hand-maintained (we accept divorcing from CubeMX regeneration; keep the
  `USER CODE` guards anyway so a future regen remains possible).
- **Post-processing runs on the PC (owner decision, 2026-07-08).** The `vl53l9-transform-c` pipeline is
  the throughput wall on the M33 (~37-40 ms/frame ≈ 25 fps ceiling at full 54×42 — a hard requirement;
  see `docs/h563-optimization-notes.md`: the M33 has no vector FPU, CORDIC/FMAC don't fit this workload,
  and fidelity-neutral micro-optimizations buy only ~5-10%). The MCU becomes a thin bridge: raw `3DMD`
  frames (14,842 B at full res, per `docs/vl53l9cx-datasheet-notes.md` p.20) + the calibration blob once
  at startup stream to the PC, which runs the same portable-C transform bit-exact at desktop speed. Raw
  at 30 Hz ≈ 445 KB/s fits USB CDC today; ~100 Hz ≈ 1.5 MB/s is the Ethernet (Phase 4) trigger — note
  I3C readout at 12.5 MHz makes 100 Hz raw marginal on this board (realistic I3C ceiling ~60-80 Hz,
  estimate; the sensor's CSI-2 output is its true 100 Hz path but the H5 has no CSI-2 receiver).
- **Deferred on-device optimizations** (recorded in case the on-MCU transform path is ever revived):
  `powf(x, const)` → multiplies in `ratenorm.c`/`sharpener.c` shadowed copies (verified `powf` survives
  in the ELF; est. 0.3-2 ms/frame), `-flto` (est. low single-digit %), SRAM bank placement for
  DMA-vs-CPU contention (speculative), acquisition/processing overlap via autonomous trigger mode +
  GPDMA2-driven async TX (est. → ~20-25 fps on-device). Full analysis: `docs/h563-optimization-notes.md`.

## Reference-firmware bugs — do not inherit

Found during review of `<APP>/Src/vl53l9_app.c`; fix these in our fork, leave the reference untouched:

1. **`vl53l9_trigger_frame` return value never checked** (`vl53l9_app.c:203-206`): the call's result is
   discarded and the stale `ret` from `vl53l9_start` is tested — trigger failures pass silently.
2. **`handle_error()` spins forever** (`vl53l9_app.c:317-322`): fine for a demo, wrong for a scanner. Our
   firmware must emit an error/event frame to the host and attempt sensor re-init before giving up.
3. **`print_frame` divide-by-zero on flat scenes** (`vl53l9_app.c:296`): `(max - min)` is the divisor; a
   uniform depth field makes it 0. Also `min - average` underflows `uint32_t` when `average > min`
   (`vl53l9_app.c:288`). Moot once ASCII printing is replaced, but don't copy the pattern.
4. **`allocate_memory(uint16_t size)`** caps buffers at 64 KB — silent truncation risk if a future
   profile/stream needs more. Widen to `size_t` in our fork.
5. **Blocking `printf` throttles the loop**: all output shares the 115200-baud VCOM. Any streaming path
   must be measured for TX-time vs frame-time and must drop frames rather than stall acquisition.
6. **Resource frees commented out** (`vl53l9_app.c:263-269`): acceptable in a never-exiting loop, but our
   app gains stop/reconfigure paths in Phase 3 — the teardown sequence must actually work by then.

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
>   and the viewer gained `--color {depth,reflectance,confidence}` (default `depth`, no behavior change)
>   with a one-time stderr fallback notice if the requested plane is absent from the stream. Verified live
>   on hardware (Task 5, this doc's Phase 2.5 note below): IR-shaded cloud renders, no fallback warning,
>   no traceback.
> - ✅ **Trigger-early overlap** — shipped Phase 2.5 Task 4: the raw-only loop now triggers frame N+1
>   before sending frame N over CDC, hiding the ~15 ms send inside the sensor's ranging window. Measured
>   **27.76 fps** (up from 24.6), 2 ms settle (down from 5 ms; the one bounded experiment the task allowed),
>   0 crc, 0 gaps, re-confirmed by this task's 60 s live soak (below). **Strategic implication**: the CDC
>   send is no longer on the critical path at all (fully hidden inside ranging) — so Phase 4's Ethernet
>   cutover will **not** by itself raise raw-only fps further; the sensor-serial chain (settle + ranging +
>   I3C DMA readout) is now the ceiling. Ethernet's value going forward is what the Phase 4 section already
>   says: 100 Hz-class rates, hardware PTP timestamping, and zero-config direct-link — not a fps lift for
>   this loop.
> - ✅ **ZAPC Deprojector validation** — done Phase 2.5 Task 3: ZAPC's z is bit-identical to ZF32 depth
>   (hard-asserted, 0.0 mm diff); best-fit FoV 54.65°H/42.50°V agrees with the datasheet defaults within
>   0.35°/0.50°; worst-case linear-model displacement is corner-concentrated (127 mm / 6.36% of z at
>   row 0, col 53, vs. 12-20 mm center-region) and doesn't improve with a global FoV tweak, so the linear
>   defaults stand and an **optional per-zone tan-table path** was added to `Deprojector` (constructor arg,
>   linear stays default) for future consumers needing corner accuracy. Full numbers, conventions, and the
>   decision writeup: `docs/deprojector-validation.md`. **Vendor-bug note**: ZAPC's 4th (confidence)
>   channel is structurally ~1.0 on every zone including no-return sentinels — not usable as a validity
>   gate. Root cause (uninitialized `conf_scaling` divisor, never assigned anywhere in the `53L9A1/`
>   tree — the channel is structurally constant, no capture can change it; the sentinel zones'
>   1e-6-digit micro-variation is actually a packed filter-status code, not a confidence score) is
>   documented in `docs/deprojector-validation.md`'s confidence-channel section. Depth-sentinel gating
>   remains the correct exclusion mechanism, not the ZAPC confidence field.
> - **Open — connect-time CRC/DROPPED transient** (first observed Phase 2 Task 7): still unexplained;
>   reproduced again in this task's 60 s soak (1 CRC failure + 1 dropped flag, both at connection time,
>   first-occurrence-transient class — no recurrence within any run so far). Tracked alongside the
>   ~1-in-5 boot hang (candidate common root: sensor bring-up timing); to be investigated with the
>   EVENT-frame/recovery work.
> - **Open — CALIB-on-DTR-connect**: CALIB retransmit cadence means a host attaching mid-cycle discards
>   up to 63 RAW frames (~2.3 s blind start at 27.76 fps); improvement: firmware sends CALIB immediately
>   on DTR-connect (cheap — the connect wait already exists). Not yet implemented. Live evidence of the
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
  **Vendor bug**: ZAPC's per-zone confidence channel is structurally ~1.0 everywhere (uninitialized
  `conf_scaling` divisor in the library, never assigned) and does not discriminate valid/invalid zones —
  don't gate on it; use the depth sentinel instead (root cause + measurements in
  `docs/deprojector-validation.md`'s confidence-channel section; see also the Phase 2.5 deferred-list
  entry above).
- Bandwidth: only the raw stream crosses the wire (14,842 B/frame — 1.63× the old depth payload,
  regardless of how many output streams the PC computes). 30 Hz ≈ 445 KB/s fits CDC FS; beyond ~60 Hz
  wants Phase 4's Ethernet (and I3C readout itself tops out ~60-80 Hz, estimate — see the architecture
  decision above).

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
planned. Left open: the connect-time transient and CALIB-on-DTR-connect (both above), carried forward
unchanged from Phase 2.

### Phase 3 — UI & runtime configuration

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

### Phase 4 — Transport cutover to Ethernet

Enable the ETH MAC + lwIP (RMII pins already muxed; LAN8742 PHY on-board), move the frame protocol onto
UDP, add hardware PTP (IEEE 1588) timestamping.

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

### Phase 5 — Integrate X-NUCLEO-IKS4A1

IMU (LSM6DSV16X hardware SFLP quaternions) / mag / baro drivers; fuse readings into the payload with
hardware timestamps. New streams = new `stream_id`s + a version bump per the protocol rule.

- **Bus topology — resolved** (`docs/iks4a1-stacking.md`): the IKS4A1 shares the ToF's **I3C1** bus as
  legacy-I2C targets (`I3C1.BusUsage=MixedUsage` already set in the `.ioc`), not a separate I2C peripheral.
  No static-address collision (ToF `0x29` vs IKS4A1 `0x1E`/`0x38`/`0x5C-5D`/`0x6A-6B`); keep IKS4A1 INT
  lines off the ToF control pins PB1/PB5/PB6/PB7, and match both boards' bus I/O rail to 3.3 V. The driver
  must assign the ToF's I3C dynamic address clear of the IKS4A1 statics and declare them as legacy-I2C
  targets.
- SFLP quaternion wire format: **IEEE binary16 (fp16), not fixed-point int16** — the research doc mislabels
  this; document the encoding in `docs/protocol.md` and test the fp16 decode path with a golden vector.
- IMU sample rate (~100+ Hz) ≠ ToF frame rate: IMU frames are independent small frames with their own
  timestamps, not fields bolted onto depth frames — SLAM interpolates host-side.
- Edge-AI (MLC/ISPU) belongs in-sensor at this tier, not on the M33.

### Phase 6 — Real-time SLAM (PC)

SFLP quaternion as rotation prior → 3-DoF constrained Open3D Tensor G-ICP → scalable TSDF
(VoxelBlockGrid), IR as intensity channel, barometer as soft 1-DoF Z constraint.

- Baro is a *soft* constraint — indoor pressure transients (HVAC, door openings) are several Pa
  (~12 Pa/m); never treat as ground truth.
- Accel-derived translation is **not** an input (double-integration drift); translation comes from ICP.
- CPU-first: Open3D tensor pipeline runs on CPU; CUDA optional. Validate real-time budget with recorded
  Phase 1/2 datasets before hardware-in-the-loop.
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

### Phase 7 — Offline post-processing

COLMAP with ToF pose priors (hand-eye calibrated to the phone camera) → depth-regularized 3D Gaussian
Splatting seeded from the ToF cloud.

- Depends on recorded, timestamped datasets from Phase 3's recorder — design the recording format so
  offline tooling replays exactly what SLAM saw.
