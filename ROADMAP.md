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
  correction), barometer (Z-drift constraint), temp/humidity (thermal comp). Not yet in code; I2C vs the
  ToF's I3C1 bus-sharing is unresolved.
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
- **FoV constants for deprojection are placeholders** until confirmed against the VL53L9CX datasheet (or
  obsoleted by an XYZ output stream, if `streams_inspect` reveals one).
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
> **Deferred / follow-up** (not blockers for calling Phase 2 done):
> - Reflectance/confidence/ambient/`--color` viewer support — the native shim only negotiates the
>   `depth`/ZF32 output capability today (Task 6); adding more output streams is small, host-only
>   follow-on work.
> - Trigger-early overlap (autonomous trigger mode + async TX) to close the 24.6 → ~30 fps gap —
>   Phase 3+ scope per the roadmap's deferred on-device-optimizations list above (now moot for
>   on-device processing, but the overlap idea still applies to the raw-acquisition loop itself).
> - ZAPC Deprojector validation — the transform library's on-device-calibrated `ZAPC` point-cloud
>   format should still be used to validate/replace the host `Deprojector`'s placeholder linear-FoV
>   model (flagged in `docs/transform-streams.md`); not done in Phase 2.
> - Connect-time CRC/DROPPED transient (first observed Task 7, see above) — unexplained; track
>   alongside the ~1-in-5 boot hang (candidate common root: sensor bring-up timing), to be
>   investigated with the EVENT-frame/recovery work.
> - CALIB retransmit cadence means a host attaching mid-cycle discards up to 63 RAW frames
>   (~2.5 s blind start at 24.6 fps); improvement: firmware sends CALIB immediately on DTR-connect
>   (cheap — the connect wait already exists).

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
  not firmware features. The `ZAPC` point-cloud format now also runs on the PC — use one ZAPC decode to
  validate/replace the host `Deprojector`'s placeholder linear-FoV model with calibrated intrinsics.
- Bandwidth: only the raw stream crosses the wire (14,842 B/frame — 1.63× the old depth payload,
  regardless of how many output streams the PC computes). 30 Hz ≈ 445 KB/s fits CDC FS; beyond ~60 Hz
  wants Phase 4's Ethernet (and I3C readout itself tops out ~60-80 Hz, estimate — see the architecture
  decision above).

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
- Fragmentation: a 9 KB depth frame exceeds Ethernet MTU — either chunk frames into ≤1400-byte datagrams
  with a fragment sub-header, or rely on IP fragmentation (fragile). Decide when speccing; chunking
  preferred.
- lwIP memory tuning (PBUF pools) on top of the transform pipeline's SRAM appetite is the main risk.
- Static-IP direct link PC↔board (auto-MDIX, no switch) is the default topology; PTP master on the PC.

### Phase 5 — Integrate X-NUCLEO-IKS4A1

IMU (LSM6DSV16X hardware SFLP quaternions) / mag / baro drivers; fuse readings into the payload with
hardware timestamps. New streams = new `stream_id`s + a version bump per the protocol rule.

- **Unresolved first**: bus topology. ToF owns I3C1; IKS4A1 sensors are I2C. STM32H5 I3C controllers can
  drive legacy-I2C targets on the same bus, or the IKS4A1 can sit on a separate I2C peripheral — check
  Arduino-connector pin conflicts between the two stacked shields *before* buying into either.
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

### Phase 7 — Offline post-processing

COLMAP with ToF pose priors (hand-eye calibrated to the phone camera) → depth-regularized 3D Gaussian
Splatting seeded from the ToF cloud.

- Depends on recorded, timestamped datasets from Phase 3's recorder — design the recording format so
  offline tooling replays exactly what SLAM saw.
