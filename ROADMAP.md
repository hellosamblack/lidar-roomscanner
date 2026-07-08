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

### Phase 2 — IR + additional sensor streams

Extend the protocol and PC UI to carry and toggle IR reflectance, confidence, ambient, etc.; colorize the
point cloud by IR intensity.

- **Gate resolved** (Task 7 capture, `docs/transform-streams.md`): the transform library exposes
  `depth`, `ambient`, `amplitude`, `confidence`, `reflectance`, `status` output streams; wire stream IDs
  0-6 are allocated in `docs/protocol.md`'s stream registry.
- **New option — on-device point cloud:** the depth stream's `ZAPC` format emits 4×float32
  [x, y, z, confidence] per zone (16 B/zone, 4× the ZF32 payload) deprojected on-device with
  factory-calibrated intrinsics. Phase 2 should evaluate ZAPC vs PC-side deprojection — and at minimum
  use one ZAPC capture to validate the host `Deprojector`'s placeholder linear-FoV model.
- Protocol is multi-stream from v1 (`stream_id` in the header), so this phase is: configure extra output
  capabilities on the transform (each needs its own output buffer — mind SRAM; 640 KB total, raw double
  buffer + N output planes), interleave frames per stream, and add per-stream toggles + colormap in the
  viewer.
- Watch: bandwidth multiplies per enabled stream — CDC FS (~1 MB/s) fits depth+IR+confidence at moderate
  rates; full rate on all streams may already want the Phase 4 transport.

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
