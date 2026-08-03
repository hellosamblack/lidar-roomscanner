# Bug tracker

Known bugs and open issues in **our** code (host `roomscan` package + `firmware/scanner-stream`).
Bugs in the read-only ST reference package are catalogued separately in `ROADMAP.md` →
"Reference-firmware bugs — do not inherit"; vendor-library defects we can only work around are
tracked here with status `vendor`.

Conventions: IDs are `BUG-NNN` and never reused. Statuses: `open`, `fixed` (keep the entry, note
the commit/PR), `vendor` (defect is upstream, we mitigate), `anomaly` (observed but not
reproducible/root-caused), `by-design` (reported as a bug, concluded intentional). New entries get
the next free ID, a date, and a file reference where the problem lives.

| ID      | Status  | Area          | Title |
|---------|---------|---------------|-------|
| BUG-001 | fixed   | host/viewer   | Spatial surface mode floods console with Open3D "invalid tetra" warnings |
| BUG-002 | fixed   | host/viewer   | Spatial surface mode pins many CPU cores; GPU sits idle |
| BUG-003 | fixed   | host/viewer   | View color defaulted to depth instead of reflectance |
| BUG-004 | fixed   | host/sensors  | Yaw fusion needs on-rig mag calibration + axis-convention check |
| BUG-005 | fix unverified | firmware/host | Connect-time transient: one CRC failure + RAW-frame skip on DTR connect — DTR-callback fix implemented, but the CDC path cannot be exercised on this host |
| BUG-006 | anomaly | firmware      | One 100 s post-flash boot-recovery hang (seen once, never reproduced) |
| BUG-007 | fixed   | transform lib | ZAPC confidence plane is structurally ~1.0 everywhere |
| BUG-008 | fixed   | host/viewer   | Minimizing the roomscanner panel triggers Filament Camera preconditions warning |
| BUG-009 | fixed   | host/panel    | SLAM/Showcase trajectory LineSet with a single point hard-crashes Filament (segfault) |
| BUG-010 | by-design | host/panel  | A Recorder capture started well into a session lacks CALIB and can't be post-processed |
| BUG-011 | fixed   | host/panel    | Floating HUD toggles unclickable — control `ImageWidget`s swallow clicks before the SceneWidget's `set_on_mouse` |
| BUG-012 | fixed   | host/panel    | Per-frame `srgbColor` Filament console spam from `defaultUnlitTransparency` material |
| BUG-013 | fixed   | host/panel    | SLAM-mode Record never stops/processes — action cluster armed the classic SLAM view, not the Showcase pipeline |
| BUG-014 | fixed   | host/panel    | First-person IR overlay renders edge-on (white/black) or not at all — first-person camera clobbered + texture not bound as albedo |
| BUG-015 | fixed   | host/panel    | Overlays → Sensors toggle showed nothing — sensor widgets lived only in the settings dialog, no floating overlay |
| BUG-016 | fixed   | host/panel    | First-person IR overlay: upside-down texture, hidden on a fresh launch, and oversized vs. the real point cloud |
| BUG-017 | fixed   | host/panel    | Panel launch always fails "port in use" — its own ST-Link log-tail thread races the CDC-missing serial fallback for the same COM port |
| BUG-018 | fixed   | host/panel    | Launch failures (missing/busy scanner port) never appeared in app.log — printed to console only |
| BUG-019 | fixed   | host/sources  | Ethernet preference was fragile: `.local` resolution always failed on Windows, and the "retry" loop only ever sent one wake packet |
| BUG-020 | fixed   | host/native   | Native transform loader was Windows-only (`.dll` name + `build/Release/` path) — blank viewer on the Linux headless host; reader fault never surfaced to the page |
| BUG-021 | fixed   | host/web      | Web viewer loaded Three.js from a CDN (unpkg) — headless/remote browser couldn't fetch it, `app.js` threw on the bare `three` import, page stuck at "Offline" |
| BUG-022 | fixed   | host/web      | Headless-host browser has no WebGL — `new THREE.WebGLRenderer()` threw "Error creating WebGL context", viewer stuck "Offline"; auto-open now passes Chrome `--enable-unsafe-swiftshader` |
| BUG-023 | fixed   | firmware      | System clock sourced from the ST-LINK's MCO — board is dead (silent network, PHY link LED on) whenever the ST-LINK cable is unplugged |
| BUG-024 | fixed   | host/sources  | A *missing* CDC serial port aborted the launch of an Ethernet-only deployment |
| BUG-025 | fixed   | host/sources  | `UdpSource` retargeted onto its own looped-back broadcast wake, adopting the host itself as the device |
| BUG-026 | fixed   | host/web      | Web UI never gravity-aligned the view — boot the board upside down and both IR and point cloud render upside down |
| BUG-027 | fixed   | firmware      | SFLP quaternion decimated 480 Hz → 30 Hz by keeping one sample of ~16, aliasing the whole noise band into the output |
| BUG-028 | fixed   | firmware      | Ethernet hot-plug replug wedges board (LD2 freezes) — lwIP double-add assertion on `mdns_resp_add_netif` |
| BUG-029 | fixed   | host/sources  | `UdpSource` stream recovery fails due to `255.255.255.255` fallback keepalives not routing on some Linux network configs |
| BUG-030 | fixed   | host/sensors  | Magnetometer calibration is direction-dependent: |B| ranges 47→85 µT with tilt — root-caused to a ~59 µT wrong hard-iron offset (+ tripod, BUG-034); owner re-fit 2026-07-30, validated on an independent room sweep |
| BUG-031 | fixed   | firmware      | ToF frame timestamp and IMU FIFO drain skewed ~0.9 ms — the drain sits 24.3 ms past the frame-ready edge and breathes with load; fixed by latching the LSM clock AT the edge (stream 13), 1070 → 18 µs RMS |
| BUG-032 | fixed   | host/slam     | GPU SLAM OOMs on a long scan — Open3D's CUDA cache grows ~5.1 MiB/frame from the throttled mesh extraction (NOT the per-frame path, which is byte-flat) |
| BUG-033 | fixed   | host/web      | Sensors card outgrew the dock band (~1600 px of flat, half-duplicated rows) — jitter table unreachable, whole card auto-collapsed on a narrow window |
| BUG-034 | by-design | environment   | The tripod adds 15–27 µT of magnetic field — heading is unreliable while the device is mounted on it, regardless of calibration |
| BUG-035 | fixed   | host/slam     | TSDF block capacity (40k) was *below* a real room sweep's demand (42.9k); running at ~97% of it stalled map growth and collapsed tracking for the last 18% of the scan (mechanism unproven — the grid *does* rehash) |
| BUG-036 | fixed   | host/slam     | A single fixed ICP correspondence radius (0.05) made one bad frame terminal — 423 frames (22% of a room circuit) silently dead-reckoned; fixed by retrying only on failure at 0.10 |
| BUG-037 | fixed   | host/slam     | `baro_weight = 0.05` fed the barometer's ~267 mm RMS of per-frame white noise straight into the pose — ~35% of reported path was invented vertical motion (flattering every %-of-path drift figure) and a drifting baro owned the height outright; replaced by a low-passed, bounded-authority complementary correction |
| BUG-038 | fixed   | host/web      | The live point cloud is frustum-culled against a stale, zero-radius `boundingSphere` — latent in World view, fatal in FPV (filed as a duplicate "BUG-033"; renumbered 2026-07-30) |
| BUG-039 | fixed   | host/sensors  | `imufusion._correct_yaw` measured heading error about **body Z** on a body frame whose X is Up — 3.76° from gimbal lock, giving 1.69° mean heading error that loop gain could not fix; replaced by a world-Z swing-twist term (2.218° → 0.053° p95, bit-identical at level attitudes). Filter still gated off |
| BUG-040 | fixed   | host/web      | The web UI's Drops/Gaps HUD rows were structurally pinned at 0 — `web.py` read `MetricsSnapshot.drops/gaps` (dataclass default 0) and nothing ever merged the reader's `Stats`; only the deprecated `panel.py` did. Since a lost UDP fragment makes the host discard the whole frame, a seq gap is the ONLY evidence of transport loss, so the primary UI could not see packet loss at all |
| BUG-041 | fixed   | firmware/eth  | `ETH_SendFrame_Gather` burst all 11 fragments of a depth frame back-to-back into an 8-deep TX descriptor ring, and **abandoned the frame mid-burst** on any `udp_sendto`/`pbuf_alloc` failure — the already-sent fragments then being guaranteed waste. Replaced by a slot-FIFO pacer that meters fragments from `ETH_Process()` and retries rather than abandoning |
| BUG-042 | fixed   | host/sources  | UDP reassembly required `frag_idx == expected` and appended, so a merely **reordered** datagram — which UDP explicitly permits — discarded the whole 14.8 KB frame exactly like a lost one, and counted nothing. Now reassembled into indexed slots, with counters separating reorder / loss / duplicate / invalid |
| BUG-046 | open    | host/web      | Mag-cal coverage runs *backwards* on Fit — "92/92 cells" drops to 91/92, because fitting the candidate replaces the calibration the coverage cells are binned with, moving every sample's direction ~3° |
| BUG-048 | fixed (by BUG-051) | host/sensors  | `absolute_heading`/`quat_yaw_deg` disintegrate near ZYX gimbal lock — a **braced, stationary** device reports frame-to-frame yaw jumps up to 180°, and 22.7% of a tilt-sweep capture sits within 1° of lock. Corrupted the DC-E magnetometer-direction analysis into a false 20–30° "calibration error" |
| BUG-049 | open    | host/transport | Multi-second **whole-group** frame loss on the multi-room captures — 2.29% / 4.28% / 9.35% of RAW frames lost while byte-clean and 0 CRC, in outages up to 215 frames (7.1 s). Cost DC-B take 2 a 628-frame (21.2 s) tracking collapse. Recurring 63-frame quantum in all three takes; not RF range (the bridge rides on the scanner, never roamed, signal > 80%) |
| BUG-047 | fixed   | host/web      | `id="btn-restart"` named **two** buttons — the top bar's "Restart Server" and the playback "Restart". `getElementById` takes the first, so playback Restart was dead and its transport handler landed on Restart Server, which therefore fired a transport restart *and* `POST /api/restart` |
| BUG-050 | fixed   | host/web      | Recording `elapsed_s` was `time.time() - time.monotonic()` -- two clocks with no shared origin -- so a 90-second take reported 1784067285.5 s. Every caller passed the wall clock; the start stamp was monotonic |
| BUG-051 | fixed (amended by BUG-058) | host/sensors  | The yaw-fusion gimbal gate fired **permanently in the normal upright grip** and its message named the wrong axis: ZYX pitch is the elevation of body **X = Up**, so upright *is* ZYX gimbal lock (measured 2.7° from it against a 15° threshold) while the boresight sat 2.1° off horizontal. Same root cause gave `absolute_heading` an **18.4° systematic error** at the operating pose (26.57° reported for a true 45°, exact only on the cardinal four), and put World's Roll readout on the ±180 wrap at 178.30°. Closes BUG-048 |
| BUG-052 | fixed   | host/slam     | Detailed build froze the whole web UI: `TsdfMap._extract_vbg()` copied the **entire preallocated** VoxelBlockGrid to the host on every extraction (`block_count`-sized, not map-sized), 1.11 s with the GIL held, once per 25 frames — starving roomscan-web's asyncio loop for **78–84% of wall clock**. Extracting in place on the device is 0.04 s |
| BUG-053 | vendor  | host/slam     | Open3D 0.19's marching cubes **crashes the process** above ~260k active blocks — host segfaults, CUDA raises "illegal memory access" then `terminate()`s. Bisected: fine at 258,161 blocks, fatal at 273,521, on **both** devices. Driven by the absolute block count, not load factor or free memory (same count dies at 68.4% load in a 400k grid), so raising `block_count` does not help. Mitigated by refusing to extract above 250,000 |
| BUG-054 | fixed   | host/slam     | The Detailed preset's `voxel_size = 0.005` could never build a room-sized capture — DebugCapB1 crosses BUG-053's ceiling at frame 2625 of 4808. Its own `benchmark_note` ("benchmark me on CUDA:0 with a full-room capture") had never been run; 5 mm was only exercised on captures small enough to fit. Default is now 0.01 |
| BUG-055 | fixed   | host/web      | The SLAM/Detailed map's scanner pose marker was drawn **4.44x oversize** — a 62 cm slab in a metres-scale room. `slam.js` passed no `dims` to `createDeviceMesh`, taking `DEVICE_DIMS`, which is documented as magcal3d's unitless *shell* scale, not metres |
| BUG-056 | fixed   | host/web      | A finished Detailed build's "Completed in" kept counting up — `DetailedRunner.status()` derives `elapsed_s` from `time.monotonic() - _started_at` with nothing stopping it at `done`. A ~3.5-minute build read **81:01** an hour later, and `elapsed_s` was observed climbing 4910.5 → 4914.5 in four seconds |
| BUG-057 | fixed   | host/web      | The HUD's **Gaps** counter booked a source swap as lost frames — `Stats._last_seq` survives live→replay→live, but sequence numbers are per-source, so the numbering difference lands in `seq_gaps`. Observed **1,529,274 gaps** in a session with 0 drops streaming cleanly at 30 fps. Discredits the exact counter BUG-049's transport-loss work reads |
| BUG-058 | fixed   | host/sensors  | Heading swung 154° while the device was rolled at a fixed bearing — `absolute_heading` stripped yaw with `yaw_twist_deg`, and world-Z twist absorbs roll about a horizontal boresight degree for degree. `YawFusion` compounded it: `heading - yaw` differenced a device bearing against a world-Z twist, so the fused quat came out **mirrored**, at −bearing |
| BUG-059 | fixed   | host/sensors  | **Every heading ever displayed was 180° out** — aimed north, the instrument read south. The calibrated field vector is delivered ANTI-PARALLEL to Earth's: it sat 70–72° *above* the horizon on every capture, where the northern-hemisphere field is that far below it. `AXIS_CONVENTION` carried the mounting rotation but not the field-direction sign. Predates BUG-058 and was hidden by it |
| BUG-060 | fixed   | host/web      | Live SLAM ran at 5 fps and updated the view once a second on an idle GPU — `uvicorn.run()` defaults `ws_per_message_deflate=True` and `_broadcast_bytes` awaits `send_bytes` **per client**, so each 3.2 MB MESH was zlib-deflated once per connected tab on the event loop. Worse than the UI symptom: `SlamRunner.submit()` ran only from that broadcaster, so **5 frames in 6 never reached the TSDF**, silently |
| BUG-062 | fixed   | host/slam     | **Thirteen `[slam]` keys were silently ignored by Live SLAM** — `SlamRunner._construct` hand-picked five mapper kwargs, so `icp_mode`, `voxel_size`, `max_iter`, `max_dist`, the quality gates and every `stationary_*` knob changed the CLI and Detailed paths but never reached the live mapper. Editing `roomscan.toml` looked like it worked and did nothing |
| BUG-061 | fixed   | host/web      | Ephemeral SLAM lagged **up to 15 s** behind the operator, in playback and far worse live. `/ws` has no write backpressure, so the 12 MB/s MESH governor was open-loop and the whole-map re-sends piled into an unbounded transport buffer — **37 MB in flight** on the owner's connection. The 30 Hz `slam` pose JSON was queued *behind* those megabytes on the same ordered stream, so the camera lagged with the geometry |
| BUG-063 | open    | host/tools    | The GIL-starvation metric under-reports precisely when starvation is total — summed tick lateness over wall time falls toward zero as blocking approaches 100%, because the watchdog thread stops being scheduled at all (1 tick landed of ~2186 due read as "10.3% starved"). `slam_icp_bench.py` fixed with `tick_share`; `slam_stall_profile.py` still to do |
| BUG-064 | fixed   | host/web      | **"SLAM rendered nothing" — it rendered perfectly and two UI cards covered 97% of it.** In View, `#browser-card` + `#preview-card` sit over the middle of the viewport, which is exactly where the map is drawn; only the 14 px gutter between them showed through. Live never had it (both cards hide when `source != "view"`) |
| BUG-065 | fixed   | host/web      | `slam.js`'s padded vertex buffers produced a **fractional `BufferAttribute.count`** (36799.666) and therefore **NaN bounding boxes/spheres** on every SLAM mesh — `Math.ceil(needLen * 1.5)` need not be a multiple of `itemSize`, and Three then reads one slot past the end. Invisible only while `frustumCulled = false` holds everywhere |
| BUG-066 | fixed   | host/web      | `load_capture` hard-coded `ui.mode = "realtime"`, so loading a capture while the display was `slam` broadcast the contradiction `mode: "realtime", display: "slam"` |
| BUG-067 | open    | host/slam     | **A stationary tripod scan reports 18–20 m of travel and 0.6–1.7 m of net displacement, with zero tracking-lost frames.** `icp_mode="translation"` freezes rotation at the IMU prior, so prior error has no rotational degree of freedom to land in and is absorbed *entirely* as translation — and frame-to-model then integrates the bad pose, making it permanent. Shifting the prior one frame (33 ms) moves drift 0.24 → 1.92 m |
| BUG-068 | fixed   | host/slam     | The point-to-plane degeneracy guard `_COND_CEILING = 1e8` **can never fire** — worst conditioning ever observed is 203.5, five orders of magnitude below it — so ill-conditioned translation solves are accepted silently at fitness 0.88. The estimate slid 1.2 m in 3 s (43 cm/s) through a wall while reporting a healthy fit |
| BUG-069 | open    | host/slam     | `StationarityGate` is structurally unable to fire on a tripod pan (`rot_ceiling_deg=0.3` vs the ~1 °/frame of an actual pan), and is **display-only** by construction, so even when it does fire it cannot stop the map from absorbing invented motion |
| BUG-070 | anomaly | host/slam     | Reported drift is not invariant to a **physically null relabelling of compass heading**: a constant `graft_yaw` offset swings a stationary capture's reported displacement from 0.81 ± 0.53 m (0°) to 2.89 ± 0.14 m (90°), repeatably. This is why Live SLAM (mag-grafted prior) reads ~4× worse than Detailed (raw SFLP prior) on the same bytes |

---

## BUG-001 — Spatial surface mode floods console with Open3D "invalid tetra" warnings

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/viewer
- **Where:** `host/src/roomscan/surface.py` (`alpha_shape_mesh`), called from
  `panel.py` `_rebuild_spatial_mesh`

Enabling surface interpolation with adjacency mode **spatial** spams the console with many
`[Open3D WARNING] [CreateFromPointCloudAlphaShape] invalid tetra in TetraMesh` lines, repeated on
every rebuild (throttled to 4 Hz, so continuously while the mode is on).

**Likely cause:** `create_from_point_cloud_alpha_shape` starts with a Qhull Delaunay
tetrahedralization of the cloud. Our deprojected zone grid is locally near-coplanar (flat wall
patches sampled on a regular 54×42 lattice), which yields many degenerate / near-zero-volume
tetrahedra; Open3D warns once per bad tetra instead of once per call.

**Fix:** Wrapped the Open3D `create_from_point_cloud_alpha_shape` call in
`o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error)` to silence the warning
spams. The mesh that comes back is still completely usable as the degenerate tetras are simply skipped.

## BUG-002 — Spatial surface mode pins many CPU cores; GPU sits idle

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/viewer
- **Where:** `host/src/roomscan/surface.py` (`grid_triangles_3d`), `panel.py` `_render_surface`

With spatial surface mode on, many CPU cores are pinned while the GPU stays nearly idle. Owner
question: can this be offloaded to the GPU?

**Analysis:** the cost is Open3D's `create_from_point_cloud_alpha_shape` — Qhull Delaunay +
tetra filtering, CPU-only with internal OpenMP/TBB parallelism (hence *many* cores, 4×/s). Open3D
has **no GPU implementation of alpha shape** (its tensor/CUDA API doesn't cover it), so this is
not a switch we can flip; a direct GPU port would be a custom-CUDA project. The Python-side
per-vertex KDTree back-matching loop in `alpha_shape_mesh` adds single-core cost on top.

**Realistic options, roughly by effort:**
1. Lower the rebuild rate for spatial mode only (e.g. 1-2 Hz instead of the shared 4 Hz throttle)
   and/or voxel-downsample the cloud before the alpha shape — the 2268-zone cloud is small, so most
   of the tetra work is degenerate-geometry churn (BUG-001), not useful triangles.
2. Vectorize the covered-point back-matching (single batched KDTree query instead of a Python loop).
3. Replace the alpha-shape backend for this use case: the cloud is an organized grid, so "spatial"
   adjacency can be computed as grid adjacency with a 3D-distance (not depth-gap) threshold —
   O(N) vectorized numpy like `grid_triangles`, no Qhull, no warnings, near-zero CPU.
4. True GPU surface reconstruction (TSDF/surfel raycast) — belongs to Phase 6 SLAM work, where a
   TSDF volume exists anyway; not worth building just for the panel preview.

**Fix:** Implemented Option 3. Since the cloud is structured as an organized grid, "spatial" adjacency is computed using grid-adjacency triangulation with a 3D Euclidean distance threshold (`grid_triangles_3d` in `surface.py`). This runs in a fully-vectorized O(N) NumPy pass every frame with near-zero CPU footprint, completely resolving CPU pinning and avoiding Qhull failures.

## BUG-003 — View color defaulted to depth instead of reflectance

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/viewer
- **Where:** `host/src/roomscan/config.py` (`ViewerConfig.color`)

The built-in view-color default was `depth`; owner wants `reflectance`. Fixed by changing
`ViewerConfig.color` to `"reflectance"` (priority chain CLI flag > `roomscan.toml` > built-in is
unchanged). Both viewers already fall back to depth coloring with a one-time warning when the
reflectance plane is absent (no transform DLL / plane not in stream), so the new default is safe
in every configuration.

## BUG-004 — Yaw fusion needs on-rig mag calibration + axis-convention check

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/sensors
- **Where:** `host/src/roomscan/sensors.py` (`AXIS_CONVENTION`), procedure in `docs/yaw-fusion.md`

**Fix:** 
1. Fixed a math bug in `fit_ellipsoid` that caused it to reject large hard-iron offsets (when the hard-iron offset is larger than the Earth's field magnitude). Allowing the scalar scale factor `d` to be negative resolved the degeneracy check, enabling successful calibration on the physical rig.
2. Ran a figure-eight magnetometer calibration to produce `mag_cal.json` (yielding a clean fit with $\text{field\_ut} \approx 49.87\,\mu\text{T}$).
3. Evaluated all 24 possible axis-swap and sign-permutation matrices. The optimal matrix with the lowest standard deviation under tilt and a correct $\text{slope} \approx +1.0$ tracking the IMU Yaw was mathematically identified as `[x, -y, -z]`. Set `AXIS_CONVENTION = np.diag([1.0, -1.0, -1.0])` in `sensors.py` and updated all test cases to adapt.
4. Resolved a visual coordinate mapping issue in `gizmo_pose` where yaw (Z-rotation in SFLP's gravity-aligned frame) was showing up as roll in the visualizer (due to Open3D's world up being Y instead of Z). Transforming the IMU rotation matrix by the coordinate alignment matrix (`R_align @ R @ R_align.T`) correctly maps SFLP Z-rotation to visualizer Y-rotation (yaw).

## BUG-005 — Connect-time transient: one CRC failure + RAW-frame skip on DTR connect

- **Status:** **fix implemented 2026-07-30, UNVERIFIED on hardware** (see "Why unverified") ·
  **Recorded:** Phase 3 · **Area:** firmware + host
- **Where:** `firmware/scanner-stream/Src/vl53l9_app.c` (`tud_cdc_line_state_cb`, `rs_cdc_send`);
  forensics in `docs/connect-transient-forensics.md`

On host connect (DTR rising) the first frame boundary lands mid-stream: exactly one CRC failure
and a stale RAW skip, then clean streaming. Shipped mitigation: manual `SEND_CALIB` (`C` key /
`roomscan-ctl calib`).

**Correction to this entry (2026-07-30):** it used to say "root-caused to stale TX FIFO residue
(not a DTR race)". `docs/connect-transient-forensics.md`, which is the primary evidence, says the
opposite in as many words — *"**Not** stale TX FIFO residue — the truncated bytes are a genuine
frame-1 payload prefix"*. The forensic root cause is `rs_cdc_send()`'s own 100 ms abort firing
once, because the host's reader has not started draining by the time the device begins frame 1;
the bytes on the wire are the intact **front** of a real frame, cut off mid-send. The summary here
had drifted from the document it cites.

### Implemented 2026-07-30 (the deferred `tud_cdc_line_state_cb` fix)

`tud_cdc_line_state_cb` now watches for the DTR **edge**; on a rise it clears the CDC TX FIFO and
sets one volatile flag. `rs_cdc_send()` checks that flag after each `tud_task()` and abandons the
frame in flight (the new host can only ever see its tail); the acquisition loop consumes the flag
at its existing per-frame safe point and zeroes `rs_calib_countdown`, so the next frame group leads
with CALIB instead of the host waiting out up to 63 frames.

The synchronisation that deferred this turned out to be smaller than it looked, and the reason is
checkable rather than assumed: TinyUSB's class callbacks **do not run in interrupt context** in
this build. `USB_DRD_FS_IRQHandler` calls `tud_int_handler`, which only enqueues a `dcd_event_t`
(`vendor/tinyusb/src/device/usbd.c`, `osal_queue_send`); `tud_task()` dequeues and dispatches. So
the callback runs on the acquisition loop's own thread — but **reentrantly**, since `rs_cdc_send()`
calls `tud_task()` between chunks. That makes it a reentrancy problem, not a concurrency one: one
volatile flag set in the callback and consumed in the loop is sufficient, and no state the loop
owns (`raw_mem_index`, the CALIB countdown, the in-flight byte cursor) is touched from the callback.

**It cannot affect the Ethernet path, by construction.** The callback only fires for a USB host,
and the abort only shortens the CDC copy: `rs_send_generic_cdc()` hands the whole frame to
`ETH_SendFrame_Gather()` *before* the CDC loop starts, so an Ethernet host cannot see a truncated
frame regardless of what a USB host does. On-rig after this change, over Ethernet: 30.3 fps,
0 CRC failures, 0 drops, 0 gaps.

### Why unverified

**The native CDC device does not exist on this host.** `lsusb` sees only the ST-LINK
(`0483:374e`) — the board's USB_USER port is powered from the battery bridge, not connected to
this machine, so `CAFE:4001` never enumerates and there is no DTR to raise. Even if it did,
`/dev/ttyACM*` come back `root:root` mode 0 after every replug on this box and `dialout` does not
help (see the `firmware-build-on-linux` memory). So no claim is made that the transient is gone:
the code path has never executed. What *is* verified is that shipping it did not disturb the
Ethernet path, which is what the rig actually runs on.

To verify, on a machine with the board's USB_USER cable attached: capture with
`host/tools/capture.py --seconds 15`, and check `capture_analyze` reports **0** CRC failures in
the connect region (today: exactly 1) and that the first frame after connect is CALIB. Tracked as
**DC-H** in `ROADMAP.md` → "Data-collection queue".

## BUG-006 — One 100 s post-flash boot-recovery hang

- **Status:** anomaly (low confidence, not root-caused) · **Recorded:** Phase 3 Task 5 · **Area:** firmware

Observed exactly once after a flash; did not reproduce in 9 subsequent identical-scenario runs.
Tracked so a second sighting upgrades it to a real defect with two data points. If it recurs:
capture SWD register state before power-cycling (see `firmware-loop` skill).

## BUG-007 — ZAPC confidence plane is structurally ~1.0 everywhere

- **Status:** **fixed** 2026-07-10 (this branch) · **Recorded:** Phase 2.5 · **Area:** vl53l9-transform-c
- **Where:** `53L9A1/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/radial_to_perp.c` (`vl53l9_algo_radial_to_perp_init_default_params`), analysis in `docs/deprojector-validation.md` (confidence-channel section)

The transform library's ZAPC 4th (confidence) channel read ~1.0 for every zone because the `conf_scaling` divisor parameter in `radial_to_perp_params_t` was never initialized. Since the params struct was zero-initialized, this resulted in division by zero (+inf), which then got clamped to 1.0.

**Fix:** Initialized `params->conf_scaling = 1.0f;` inside `vl53l9_algo_radial_to_perp_init_default_params` so the confidence values are properly scaled relative to their threshold. Rebuilt the host-side transform library and verified using the ZAPC validation script that the confidence channel values now vary dynamically.

## BUG-008 — Minimizing the roomscanner panel triggers Filament Camera preconditions warning

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/viewer
- **Where:** `host/src/roomscan/panel.py` (`_on_layout`, `_reset_camera`, `_apply_camera`)

When the roomscanner panel is minimized, the console shows:
`in void __cdecl filament::FCamera::setProjection(enum filament::Camera::Projection,double,double,double,double,double,double) noexcept:89 reason: Camera preconditions not met. Using default projection`

**Likely cause:** When the window is minimized, its content rectangle width and height drop to 0. The side panel layout calculations result in a zero or negative width and height for the `scene_widget.frame` (specifically `r.width - panel_w` becomes negative when `r.width` is 0). Passing zero/negative width or height to the Filament camera projection settings violates internal preconditions.

## BUG-009 — SLAM/Showcase trajectory LineSet with a single point hard-crashes Filament (segfault)

- **Status:** **fixed** 2026-07-11 (this branch) · **Reported:** 2026-07-11 (Task 12, Showcase mode)
  · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` `_render_slam_frame`'s trajectory upload block (Task 10,
  the classic SLAM view -- `_show_showcase_trajectory` in this same file, added by Task 12, sidesteps
  it, see that method's docstring)

Reproduced live, deterministically, replaying `captures/phase6_motion_ref.bin` through a real
`ControlPanel` (`gui.Application.instance.run_one_tick()`), on the very first successful
`SlamWorker`/`Mapper.step()` result: the trajectory at that point has exactly 1 pose. The existing
code builds an Open3D `LineSet` with 1 point and (since `len(pts) >= 2` gates setting `.lines`) 0
line segments, then uploads it via `scene.add_geometry(...)`. This crashes with:
```
in class filament::VertexBuffer *__cdecl filament::VertexBuffer::Builder::build(class filament::Engine &):111
reason: vertexCount cannot be 0
[Open3D WARNING] Resource [VertexBuffer, 0, hash: ...] not found.
[Open3D WARNING] Resource [IndexBuffer, 0, hash: ...] not found.
```
...followed by a hard process segfault a few ticks later (not always the very next tick -- timing-
dependent). Confirmed via a minimal repro script that toggles the classic SLAM checkbox alone (no
Showcase code involved) and ticks the panel: same crash, same tick offset. Not exercised previously
because nothing had driven the panel through `run_one_tick()` fast enough, immediately after
enabling the SLAM view with no "warm-up" frames rendered first, to reach the first 1-point
trajectory publish before the *next* mesh/trajectory render call replaced it with a ≥2-point one.

**Likely cause:** Filament's `VertexBuffer`/`IndexBuffer` builders reject (well, crash on) a
0-vertex-index (or otherwise degenerate) buffer being the very first `unlitLine`-shaded geometry
added to the scene under certain engine states, rather than raising a catchable Python exception.

**Fix (applied 2026-07-11, this branch):** guarded `_render_slam_frame`'s trajectory block the same
way `_show_showcase_trajectory` already did: skip the upload while `len(trajectory) < 2` instead of
uploading a point-only `LineSet`. The classic SLAM view (`chk_slam`) no longer hits this.

## BUG-010 — A Recorder capture started well into a session lacks CALIB and can't be post-processed

- **Status:** **by-design**, mitigated for live mode 2026-07-11 (Task 12) · **Reported:** 2026-07-11
  (Task 12, Showcase mode) · **Area:** host/panel

The scanner device streams its `CALIB` control frame once, near the very start of a session.
`roomscan.slam.cli._load_frames` (and therefore `PostProcessWorker.from_capture`) needs that CALIB
frame in the capture to run `TransformStage` and produce any depth frames at all -- without it,
`_load_frames` returns `frames=[], width=None, height=None`. The panel's `Recorder` (Record/Stop
button) just dumps raw bytes from whenever `Record` was pressed onward; if the user enables
Showcase mode and presses Record well after the device/replay session already started (the normal
case), the CALIB frame has already gone by and never lands in the new `.bin`.

**Mitigation (Task 12):** `panel.py`'s `_enter_showcase_recording` now dispatches
`CommandCode.SEND_CALIB` (the same command the Device group's "CALIB" button sends) every time
Showcase's Record is pressed, so a live device re-streams CALIB into the just-opened recording.
`CommandDispatcher.dispatch()` already no-ops harmlessly ("not available in replay") when there's
no live device, so **replay-mode Showcase recordings starting after tick 0 of the replay file are
still unprocessable** -- confirmed live: `PostProcessWorker` degrades gracefully (see
`showcase.py`'s `_publish_construction_failure`: a terminal `done=True`, 0-frames/0-verts publish,
not a hang or crash) rather than blocking PROCESSING forever, but the resulting "scan" is empty.
Recording from the very start of a replay file works fine. Marked `by-design` rather than `open`
because live mode (the feature's primary use case) is fixed; a full replay-mode fix would need
`_load_frames` to tolerate a missing CALIB (e.g. reuse the panel's already-warm `TransformStage`
instead of a fresh one) -- out of scope here.

## BUG-011 — Floating HUD toggles unclickable (mouse passthrough)

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` — HUD widget creation in `_build_overlay`, `_on_mouse`

The two-mode/HUD redesign (Phase 6 panel UX) draws each floating control (mode switch, view toggle,
action cluster, IR control, status chip) as a `gui.ImageWidget` added to the window and positioned
over the `SceneWidget`. Click routing was done through `scene_widget.set_on_mouse(self._on_mouse)` →
`HudLayout.hit_test`. But Open3D dispatches a mouse event to the **topmost child widget** whose frame
contains the cursor: over a control that is its `ImageWidget`, which has no handler and does not
forward, so the SceneWidget's `set_on_mouse` never fired and `hit_test` never ran. Every HUD toggle
was dead (camera orbit still worked everywhere the controls didn't cover). This was the exact failure
the Task-9 note in `_on_mouse` anticipated ("if the ImageWidget itself consumes clicks...").

**Fix:** `gui.ImageWidget` has its own `set_on_mouse`, so each HUD widget now binds
`w.set_on_mouse(self._on_hud_widget_mouse)` — the widget that's actually on top handles its own
clicks. The new handler reuses the existing `HudLayout.hit_test` / `_dispatch_hud_hit` unchanged
(event coords are window-absolute, so segments and the IR opacity slider work as-is) and consumes
every event over a control so it never leaks to camera nav. The now-dead HUD-intercept block was
removed from `_on_mouse` — that also fixed a latent bug where a click in a *hidden* control's screen
region still dispatched to it (the SceneWidget handler used the full layout regardless of visibility).
Regression tests in `host/tests/test_panel_modes.py` (`test_on_hud_widget_mouse_*`). The on-screen
click still wants an owner eyeball (Filament can't render headless), but the mechanism is API-sound.

## BUG-012 — Per-frame `srgbColor` Filament console spam

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner) · **Area:** host/panel
- **Where:** Open3D 0.19 library bug; worked around in `host/src/roomscan/logfilter.py` (wired in
  `panel.py` `run()`)

The console floods, at the sensor frame rate, with:
```
in ... filament::UniformInterfaceBlock::getUniformOffset(...):NNN
reason: uniform named "srgbColor" not found
```
Root cause (verified against the shipped resources): of Open3D 0.19's `.filamat` shaders **only**
`defaultUnlit.filamat` declares the `srgbColor` uniform; `defaultUnlitTransparency.filamat` does not —
yet Open3D's shared `FilamentScene::UpdateDefaultUnlit` binds `srgbColor` unconditionally, so Filament
warns on every material bind of a translucent geometry. The first-person IR billboard
(`_update_ir_overlay`) does `remove_geometry`+`add_geometry` with that transparency material **every
frame** in first-person mode (the default), so one warning prints per frame. It is cosmetic — rendering
is unaffected. Filament writes it at the C runtime level (fd 2), so `contextlib.redirect_stderr` and
Open3D's verbosity control can't touch it.

**Fix:** `logfilter.install_filament_stderr_filter()` interposes an OS pipe on fd 2 and a daemon reader
thread that drops exactly the two warning lines (matched on the `srgbColor` / `getUniformOffset`
substrings — specific enough that no genuine error collides) and re-emits everything else verbatim.
Verified end-to-end: a UCRT-level write (the same runtime Filament links) of the warning is dropped
while a sentinel survives (`host/tests/test_logfilter.py`). Opt out with `ROOMSCAN_KEEP_FILAMENT_LOGS=1`.

**Fix:** Added checks in `_on_layout` to return early if the window width or height is `<= 0`, or if the resulting `scene_w` is `<= 0`. Constrained `panel_w` to be at least `0` so it doesn't become negative. Additionally, guarded camera operations in `_reset_camera` and `_apply_camera` to skip execution if `scene_widget.frame` width or height are `<= 0` (preventing setup of degenerate projection matrices).

## BUG-013 — SLAM-mode Record never stops/processes (action cluster orphaned)

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` `_set_mode` + `__init__` mode application

The panel keeps two mutually-exclusive machines: `slam_enabled` (the classic always-on live SLAM
view, `_render_slam_frame`) and `showcase_enabled` (the record→process→reveal state machine over
`ShowcasePhase`, `_render_showcase_frame`). The two-mode redesign spec is explicit that SLAM mode IS
the record→process→reveal flow: *"SLAM: map building = the former SLAM view AND Showcase flow, merged.
Record → process → reveal is the Showcase pipeline under the hood."* But `_set_mode(VIEW_SLAM)` (and
the `__init__` default-mode application) called `_on_slam_toggle(True)` — arming the classic view, not
the showcase machine. So `showcase_phase` stayed `None`, `_hud_action_labels` was pinned at the IDLE
`[REC, LOAD, CLR]` set forever, and `_on_record` (which only bridges into
`_enter_showcase_recording`/`_enter_showcase_processing` `if self.showcase_enabled`) just wrote a raw
`.bin` with no phase transition. Clicking REC therefore never became STOP and never kicked off
processing — the action cluster was orphaned.

**Fix:** `_set_mode(VIEW_SLAM)` and the `__init__` mode application now call `_on_showcase_toggle(True)`
so SLAM mode drives the showcase machine (its RECORDING phase runs the same live SLAM preview = the
"former SLAM view"). Leaving SLAM tears down showcase (and the now-unused classic view only if it was
somehow on). Regression: `test_panel_modes.py::test_set_mode_slam_arms_showcase_not_classic_slam` /
`test_set_mode_real_time_disables_showcase`. On-screen record→process→reveal flow still wants an owner
eyeball (Filament can't render headless).

## BUG-014 — First-person IR overlay renders edge-on (white/black) or not at all

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` `_apply_real_time_first_person`, `_apply_camera_mode`,
  `_update_ir_overlay`

Two independent faults on the first-person IR billboard (`ir_overlay.camera_locked_quad` +
`_update_ir_overlay`), which is a camera-locked quad built to face a +Z first-person camera:

1. **Camera clobber (the "edge-on, white one side / black the other" in Real-Time).** Entering
   Real-Time first-person, `_apply_real_time_first_person` set the fixed `look_at` camera but left
   `_camera_set = False`. The very next cloud frame's `_show_geometries` sees `not _camera_set` and
   calls `_reset_camera` → `setup_camera(bounds)`, replacing the first-person view with a bounds-framed
   orbit camera. The +Z-facing billboard is then seen from the side (edge-on); its two triangles show
   front (textured/white) vs back (unlit/black). **Fix:** `_apply_real_time_first_person` now sets
   `_camera_set = True` to pin the view; `_apply_camera_mode` resets it to `False` when Real-Time
   switches to ORBIT so the cloud reframes. (SLAM first-person was never clobbered — it rides
   `_apply_follow_camera` every frame — but it *was* dead because of BUG-013, so IR "didn't show in
   SLAM" until that fix routed SLAM through the showcase RECORDING path that updates the billboard.)
2. **Texture not bound.** The mesh carried `.textures` + `triangle_uvs`, but the `MaterialRecord` had
   no `albedo_img`, so the Filament unlit shader fell back to the plain white `base_color`. **Fix:**
   `_update_ir_overlay` now sets `self.ir_overlay_material.albedo_img` to the IR image (the reliable
   Filament albedo slot).

Regression: `test_panel_modes.py::test_real_time_first_person_pins_camera_set` /
`test_real_time_first_person_noop_without_viewport`; quad geometry stays covered by
`test_ir_overlay.py`. On-screen render (texture + orientation) still wants an owner eyeball.

**Follow-up (2026-07-14, owner on-rig round 2):** the `_camera_set` pin above then made the IR overlay
render *nothing at all* — pinning stopped `_reset_camera` from ever running, so the camera **projection
was never set** and the near cloud/billboard fell outside a stale/degenerate frustum. Together with the
owner's other first-person feedback this became a first-person overhaul (confirmed design via a two-part
question — first-person = look out through the sensor at the cloud fixed in front + IR overlay; cloud
sensor-fixed in first-person, gravity-aligned in orbit):
- **Projection:** `_apply_real_time_first_person` now sets an explicit perspective projection
  (`camera.set_projection(60, aspect, 0.05, 50, Vertical)`) before `look_at`, and is also re-applied from
  `_on_layout` (so a session opening straight into Real-Time first-person isn't left projection-less) and
  self-heals in the render path if `_camera_set` is cleared.
- **True first-person (not a camera model + orbiting image):** in Real-Time first-person the cloud is
  kept in the raw **sensor frame** (no IMU rotation, so it stays dead ahead as you aim), the IMU "camera
  model" gizmo is removed (`_remove_camera_gizmo` — it lingered from orbit), and mouse nav is swallowed so
  a stray drag can't arcball out of the fixed view. Orbit keeps the gravity-aligned cloud + gizmo.
- **Camera never decimated (#2):** the follow camera's flat `_FOLLOW_SMOOTH=0.12` EMA lagged real motion
  ~0.3 s ("feels like the system didn't notice you moved"). `_follow_alpha` makes the weight
  velocity-adaptive — sub-`_FOLLOW_SNAP_M` (3 cm/frame) jitter still smooths, genuine motion tracks 1:1.
- **Orbit auto-zoom (#4):** entering ORBIT in either mode clears `_camera_set`, so `_reset_camera`
  (Real-Time) / `_slam_camera_frame` (SLAM) refits the view to all content on the next frame.

Regression: `test_panel_modes.py` (`test_real_time_first_person_aims_view_without_pinning`,
`test_follow_alpha_*`, `test_apply_camera_mode_orbit_clears_camera_set`, `test_remove_camera_gizmo_*`,
`test_on_mouse_swallows_nav_in_real_time_first_person`). All camera/render behavior still needs an owner
on-rig eyeball (Filament can't render headless).

**Follow-up (2026-07-14, owner on-rig round 3):** "first-person doesn't work right away — I have to go to
orbit and back." Root cause: the projection-pin approach above (`_camera_set=True` in
`_apply_real_time_first_person`) *blocked* `_reset_camera`, so at startup/mode-switch the projection was
never established and first-person rendered wrong until an orbit round-trip ran `_reset_camera` to prime
it. **Fix:** stop pinning and stop setting the projection in `_apply_real_time_first_person` — it now only
aims the `look_at` view, and is re-applied **every Real-Time first-person frame** (after `_show_geometries`
lets `_reset_camera` own the projection from the live cloud bounds), plus from `_on_layout`. This mirrors
SLAM exactly (`_slam_camera_frame`'s `setup_camera` once + `_apply_follow_camera`'s per-frame `look_at`),
so first-person is correct from the first frame with no orbit round-trip. (SLAM first-person already worked
this way — it activates on the RECORDING follow.)

**Follow-up (2026-07-14, owner on-rig round 4):** with first-person now rendering the cloud correctly, the
IR billboard still didn't appear with the opacity slider at full. Cause: `_set_ir_opacity` only set the
opacity — the draw gate is `fp and ir_overlay_enabled`, and nothing flipped `ir_overlay_enabled`, so the
slider did nothing until the (non-obvious) "IR" label was also clicked. **Fix:** the slider now doubles as
the on/off control (`_set_ir_opacity` enables the overlay for opacity > 0.02, hides it at ~0; toggling on
at 0 opacity bumps it to 1.0). Verified the `defaultUnlitTransparency` shader *does* carry an `albedo`
texture sampler, so the `albedo_img` binding renders the IR image (not the round-1 white base_color).
Added a one-time `_update_ir_overlay` log when enabled but the stream has no reflectance (depth-only).

## BUG-015 — Overlays → Sensors toggle showed nothing (no floating overlay)

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` (`_build_overlay`, `_on_layout`, `_update_sensors`,
  `_toggle_sensors_menu`), `host/src/roomscan/sensors_widgets.py` (`render_sensors_overlay`)

The redesign's **Overlays → Sensors** menu item toggled `sensors_panel` and logged, but nothing appeared:
the compass + pressure/temp widgets were only ever built into the **settings dialog**'s "Sensors" group
(not a menu target), so the menu had no floating overlay to show — unlike **Overlays → Metrics**, which
drives the top-left `metrics_hud` ImageWidget. Toggling Sensors therefore read as an empty overlay.

**Fix:** added a floating **Sensors overlay** mirroring the metrics HUD — a new pure
`sensors_widgets.render_sensors_overlay(heading, pressure_hist, temp_hist)` composites the compass dial +
heading readout and the pressure/temp sparklines into one panel image, drawn into a top-right
`gui.ImageWidget` (`_build_overlay`/`_on_layout`), refreshed on the ≤4 Hz UI tick (`_update_sensors`), and
shown/hidden by `_toggle_sensors_menu`. The settings-dialog Sensors group is retained for the Reset
Baseline control (its display-widget updates are now `hasattr`-guarded, closing a latent crash when Sensors
was toggled on after being built-disabled). Also (owner request) the app now **defaults to Real-Time
first-person** (`ViewerConfig.mode` "slam" → "real_time"). Tests: `render_sensors_overlay` shape/no-data/
heading-change; config default. On-screen placement still wants an owner eyeball.

## BUG-016 — First-person IR overlay: upside-down, hidden on launch, oversized

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/ir_overlay.py` (`camera_locked_quad`), `host/src/roomscan/config.py`
  (`ViewerConfig.ir_overlay`), `host/src/roomscan/panel.py` (`_update_ir_overlay`)

The on-rig eyeball that BUG-014/ROADMAP flagged as still outstanding ("IR billboard texture
render/UV orientation + opacity") surfaced three independent faults:

1. **Upside-down texture.** `camera_locked_quad`'s UVs mapped the top-left vertex to `v=0`, but
   Open3D/Filament samples textures bottom-left-origin (OpenGL convention) while the reflectance
   image (`reflectance_to_rgb`/`o3d.geometry.Image`) is row-major top-down like every other array
   in this codebase — rendering the billboard upside down. **Fix:** flip `v` (TL/TR/BR/BL now map
   to `v=1,1,0,0`).
2. **Hidden on a fresh launch despite the opacity slider sitting at 50%.** `ViewerConfig.ir_overlay`
   defaulted to `False` while `ir_opacity` defaults to `0.5` — inconsistent with the "opacity > 0.02
   implies enabled" invariant `_set_ir_opacity`/`_toggle_ir_overlay` already enforce on every runtime
   interaction (round 4 of BUG-014). A fresh install (no saved config yet) started in that
   self-contradictory state. **Fix:** default `ir_overlay` to `True`.
3. **Oversized relative to the real point cloud** (owner screenshots: "look at the size of the
   person in the foreground, compared to that same person in the overlay" — the same content
   filled far more of the billboard's real-terms footprint than its rectangle implied). Root cause:
   `_update_ir_overlay` built the quad from the *viewing* eye that every caller passes — Real-Time's
   fixed `[0,0,-_FOLLOW_BACK_OFF_M]`, SLAM/showcase's `follow_camera_target(pose)` result — which
   sits `_FOLLOW_BACK_OFF_M` (0.3 m) *behind* the true sensor origin ("a hair of context", per
   `follow_camera_target`'s own docstring). But `camera_locked_quad` sizes the quad as a true
   sensor-FOV footprint anchored at the sensor's own origin (matches `capture_square_corners`'s apex
   convention, verified by `test_quad_corners_match_capture_square_convention`) — building it from a
   point 0.3 m further back than that origin inflates the quad by `dist/(dist-back_off)` ≈ 43% at
   `dist=1.0`, dwarfing the real IR content drawn inside it. Verified numerically (no on-rig render
   needed — Filament can't render headless on this box): projecting the Deprojector's true FOV-corner
   rays and the billboard's corners through the same look-at + perspective transform showed the
   un-fixed quad ~23% oversized in NDC space at a representative 1.5 m scan depth; reconstructing the
   apex (`eye + _FOLLOW_BACK_OFF_M * forward`, the inverse of `follow_camera_target`'s
   `eye = sensor_pos - back_off*forward`) closed the gap to ~6%, fully explained by the
   Deprojector's zone-center (vs. edge) convention and the quad's fixed 1.0 m placement vs. the
   test depth — not a further bug. **Fix:** `_update_ir_overlay` now reconstructs the true apex
   before calling `camera_locked_quad`, fixing all three call sites (Real-Time, SLAM follow,
   showcase RECORDING) at once. Regression: `test_ir_overlay_sized_from_true_sensor_apex_not_the_offset_eye`.

Tests: 576 host tests green. On-screen render still wants an owner eyeball to confirm the fixes read
correctly (this pass was code+numerics only — Filament can't render headless on this box).

## BUG-017 — Panel launch always fails "port in use", every retry, no external process

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` (`_stlink_logger_thread`, `_open_source`),
  `host/src/roomscan/sources.py` (`SerialSource.find_port`)

`roomscan-panel` failed to launch with `error: the scanner port is in use: ... PermissionError`,
offering to close the one roomscan process it found holding the port — which was **the process
asking the question**. Killing it and relaunching reproduced identically every time.

Root cause: two features raced for the same COM port, entirely within a single process, whichever
port happened to be a Nucleo's ST-Link VID (`0x0483`) device. **(1)** Today's ST-Link log-tail
addition, `_stlink_logger_thread`, starts as a daemon thread at the very top of `run()` — before the
scanner port is even opened — and opens the first ST-Link-VID port it finds to tail firmware
`printf` output. **(2)** `SerialSource.find_port()` had a "milestone 1a" fallback (from before native
USB CDC existed): if the CAFE:4001 CDC device isn't enumerated, fall back to the first ST-Link-VID
port and treat it as the scanner data port — but in the current architecture that port only ever
carries plain-text debug printfs, never protocol frames, so this fallback couldn't actually have
worked even had it won the race. `_open_source`'s scanner-open attempt (the fallback branch) only
runs after `get_best_source`'s ~5 s UDP probe, so the logger thread — with zero delay — reliably won
the race for that port every time, and `_open_source` saw its own process's PermissionError.

**Fix (two parts):**
1. **Removed the vestigial fallback.** `SerialSource.find_port()` now only matches the CDC device
   (`CAFE:4001`); it no longer treats an ST-Link VCOM port as a candidate scanner port, so it no
   longer competes with `_stlink_logger_thread` for it. (Flashing over SWD via
   `STM32_Programmer_CLI` is on a separate USB interface from the VCOM UART bridge — holding the
   VCOM open for logging never blocks a flash.)
2. **A busy serial port is no longer a launch blocker.** Ethernet (Phase 5) is the production
   transport now, so `_open_source` warns and falls back to a listening `UdpSource` instead of
   aborting the app when the serial fallback is busy (still offers the interactive close-the-holder
   prompt first, when useful). Regression: `test_open_source_busy_port_warns_and_falls_back_to_udp`
   (`host/tests/test_panel.py`).

Tests: 576 host tests green.

## BUG-018 — Launch failures never appeared in app.log

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` (`_report`, `_open_source`)

Surfaced immediately after BUG-017: a missing-scanner launch failure (`error: scanner not found: no
scanner serial port found among [...]`) printed correctly to the console, but "None of these were
logged in .../app.log". Two independent sources of console-only output turned out to be involved —
the `[run]`/`[tip]`/`[hint]` lines are `echo`ed by `view-panel.bat` itself, entirely outside Python,
so they can never reach a Python-managed log file — but the actual diagnostic (`_open_source`'s
error/warning messages) *is* Python's own, and simply used `print(..., file=sys.stderr)` directly
instead of going through the `logging` module, bypassing the `RotatingFileHandler` that
`_setup_app_logger` (already called before `_open_source` in `run()`) attaches to the root logger.

**Fix:** added `_report(msg, level="error"|"warning")`, which prints to stderr (unchanged
console behavior) *and* logs at the matching level; `_open_source`'s four message sites now go
through it. Verified end-to-end (not just via `caplog`): a forced missing-port failure now leaves the
exact console text in `logs/app.log`. Regression: `test_open_source_messages_are_logged_not_just_printed`
(`host/tests/test_panel.py`).

Tests: 577 host tests green.

## BUG-019 — Ethernet preference was fragile (two independent bugs)

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner: "we had comms over ethernet
  working prior to this... it's supposed to prefer ethernet") · **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` (`UdpSource._resolve_target`, `get_best_source`)

`get_best_source` is supposed to prefer Ethernet (Phase 5's production transport), probing UDP for
5 s before falling back to serial. Two independent bugs made that probe unreliable:

1. **`.local` resolution always fails on Windows.** `_resolve_target` tried
   `socket.gethostbyname("roomscanner.local")` first — but Windows has no native mDNS resolver
   without Bonjour installed, so this always raises `gaierror` (confirmed on-box:
   `gethostbyname('roomscanner.local')` → `gaierror(11001, 'getaddrinfo failed')`), meaning the
   *every-time* path was the broadcast fallback, never the (more reliable, unicast) mDNS-resolved
   address — despite `zeroconf` already being an installed dependency and `tools/query_mdns.py`
   already proving the correct call (`Zeroconf().get_service_info("_roomscan._udp.local.",
   "roomscanner._roomscan._udp.local.")`, matching the lwIP mdns advertisement from ROADMAP Phase 5)
   works. That correct call was simply never wired into `UdpSource`.
2. **The "retry" loop only ever sent one wake packet.** `get_best_source` set the *socket's own*
   timeout to the full 5 s probe window *before* entering the retry loop — so the very first
   `udp.read()` call itself blocked for up to the whole window internally (returning early only on
   data), leaving the outer `while time.time() - t0 < 5.0` no real second iteration to resend on.
   The board doesn't know the host's address up front (needs the "wake" datagram to learn where to
   reply), and UDP has no delivery guarantee — one dropped packet silently killed Ethernet
   preference for the entire launch, no retry, every time.

**Fix:**
1. `_resolve_target` now queries mDNS properly via zeroconf's `get_service_info` (injectable
   `zeroconf_factory` for tests) and only falls back to subnet broadcast if that finds nothing or
   errors.
2. `get_best_source` now uses a short per-read socket timeout (0.2 s) with a real wall-clock polling
   loop, resending the wake packet every `resend_s` (default 0.5 s) — both now parameters
   (`probe_s`, `resend_s`) so tests don't need to wait out a real 5 s window.

`UdpSource`/`get_best_source` had zero prior test coverage. Added: mDNS-success /
mDNS-not-found-falls-back-to-broadcast / zeroconf-error-falls-back-to-broadcast
(`test_resolve_target_*`), and a loopback-free regression proving the retry loop actually resends
and returns promptly on data instead of blocking out the full probe window
(`test_get_best_source_resends_wake_packet_and_returns_promptly_on_data`), all in
`host/tests/test_sources.py`.

**Still open:** at the time of this fix, neither `socket.gethostbyname` nor a live
`tools/query_mdns.py` mDNS query found the device on this machine — i.e. this fix makes Ethernet
discovery *robust*, but doesn't by itself prove the device is currently reachable over Ethernet from
this PC. Worth an on-rig check: is the Ethernet cable actually connected right now (direct link,
self-assigned `172.31.253.1`/`.2` per ROADMAP Phase 5, or a real DHCP-served LAN)? The firmware
itself falls back to USB CDC if it doesn't have a leased Ethernet IP at boot, so "not found over
Ethernet" can also legitimately mean the device decided to stream over CDC instead.

Tests: 581 host tests green.

---

## BUG-020 — Native transform loader was Windows-only → blank viewer on the Linux headless host

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner, post-migration: "migrated this
  to a headless host... launching view-web.sh loads the page, but I see no data") · **Area:** host/native
- **Where:** `host/src/roomscan/native.py` (`_DLL_NAME`, `_candidate_paths`, `_BUILD_HINT`);
  observability side in `host/src/roomscan/web.py` + `host/src/roomscan/static/app.js`

The project was developed on Windows; this was its first run on Linux (a headless Proxmox/LXC host
streaming from the board over Ethernet). The board streams RAW `3DMD` frames and the PC runs the
`vl53l9-transform-c` pipeline through a compiled native library (`host/transform/`, Phase 2). The
loader that finds and `ctypes.CDLL()`s that library was hardcoded to Windows in two ways:

1. **Library name.** `_DLL_NAME = "roomscan_transform.dll"` — but Linux CMake (single-config
   Ninja/Make) emits `libroomscan_transform.so`, and macOS `libroomscan_transform.dylib`.
2. **Search path.** `_candidate_paths()` only looked in `build/Release/` — the MSVC multi-config
   layout. Single-config generators emit straight into `build/`.

Net effect on Linux: `_find_dll()` returned `None`, `Transform.__init__` raised
`RuntimeError(_BUILD_HINT)` on the first RAW frame, and — because `viewer._reader` deliberately
swallows exceptions into `fault` so one bad frame can't kill the process — the web viewer sat
**silently blank**. `get_best_source` had already succeeded (UDP probe reads raw bytes, needs no
native lib), so the socket was connected and the page said "Live", masking the real failure. The
build hint it *would* have shown was itself Windows-only (`-G "Visual Studio 18 2026"`).

Prerequisite (done by owner, same session): the `53L9A1` reference package — previously an untracked
sibling dir (`../53L9A1/`) that never migrated — was vendored in-repo at `firmware/vendor/53L9A1/`,
and `host/transform/CMakeLists.txt`'s `PKG_ROOT` repointed there. That unblocked the build; this bug
is the loader half.

**Fix:**
1. `_DLL_NAME` is now chosen per `sys.platform`: `.dll` (win32) / `.dylib` (darwin) / `.so` (else).
2. `_candidate_paths()` searches both `build/Release/<name>` (MSVC) and `build/<name>`
   (single-config), in addition to the `ROOMSCAN_TRANSFORM_DLL` env override and the alongside-package
   location.
3. `_BUILD_HINT` is platform-aware (Linux/macOS get `-DCMAKE_BUILD_TYPE=Release` + `cmake --build`,
   and the correct library name).
4. **Observability** (so the next silent-blank is a one-line answer, cf. BUG-018): `web.py` logs the
   selected transport at startup (`[source] Ethernet/UDP -> ip:port` / `Serial CDC` / `Replay`,
   flushed), a watchdog thread prints `[FATAL] reader thread stopped: ...` to stderr the instant
   `fault` is set, and the WebSocket handler sends a JSON `{"type":"error"}` text frame to the page on
   fault. `app.js` now distinguishes text frames (status/error → shown in the connection indicator)
   from binary point frames instead of blindly feeding every message into `Float32Array`.

Verified end-to-end against the live board on the Linux host: loader resolves
`host/transform/build/libroomscan_transform.so`, UDP source connects to `172.17.2.58`, pipeline
produces valid depth `(42×54)`, 100% valid, 564–12000 mm, at ~30.5 fps.

**Note:** the full uvicorn server can't be exercised from inside the agent's sandbox (the TCP bind is
killed, exit 144), so the end-to-end verification drives the identical source→pump→`TransformStage`
data path directly rather than through `roomscan.web`. The server code paths themselves were
import-checked only. A running instance started *before* the loader fix keeps the old module in
memory (Python caches imports) — it must be **restarted** to pick up the fix.

**Follow-on robustness (same session):** `UdpSource` now re-sends a wake
datagram every `keepalive_s` (default 1.0 s) from `read()`, not just during the
startup probe. The board unicasts frames to whichever host last woke it and
clears that target only on reboot, so a board reset / link flap / a second
client claiming the stream previously silenced the viewer permanently (it never
re-woke). Symptom seen live: a launched instance held UDP 5000 and picked
Ethernet, then processed zero frames (flat CPU) while the board streamed
elsewhere. Keepalive makes the app re-claim the target and self-heal. Verified:
streaming intact with keepalive on (110 slots/5 s); 580 host tests pass.

---

## BUG-021 — Web viewer loaded Three.js from a CDN → "Offline", blank on headless host

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner, post-migration: "It says
  offline") · **Area:** host/web
- **Where:** `host/src/roomscan/static/index.html` (import map), `host/src/roomscan/static/app.js`

The Three.js viewer front-end resolved its `import * as THREE from 'three'` and
`three/addons/controls/OrbitControls.js` via an import map pointing at
`https://unpkg.com/three@0.160.0/...`, and pulled Inter/JetBrains-Mono from Google Fonts. On the
Windows dev box (with internet in the same browser) this worked. On the migrated headless host the
rendering browser couldn't fetch the CDN module — a remote browser without a route to unpkg, a proxy,
or a stale tab from a load that failed — so the bare `three` specifier failed to resolve, `app.js`
threw at module-eval, `connect()` never ran, and no WebSocket was ever opened. Because
`#conn-text`'s initial markup is literally `Offline`, the page just sat there: server serving live
frames (verified: a WebSocket client and a fresh Chrome both pulled ~2160-pt frames), browser showing
"Offline", no error the user could see. Note the host shell *could* `curl` unpkg fine — CDN
reachability from the shell says nothing about the browser actually rendering the page.

**Fix:** vendored `three.module.js` + `OrbitControls.js` into
`host/src/roomscan/static/vendor/three/` and repointed the import map at `/static/vendor/three/...`,
so the viewer is fully self-contained (same rationale as the Artifacts "no external hosts" rule and
the firmware's vendored tinyusb/lwip). The Google-Fonts `<link>` is left as-is: it degrades to system
fonts if unreachable and never blocks scripts. Verified end-to-end with headless Chrome against the
live server: status **Live**, 2247 points, point cloud rendering, zero console errors.

**Note:** this was the third distinct Windows→Linux migration gap in one session, after BUG-020
(native loader) and the BUG-020 keepalive follow-on. All three shared a shape: something implicit on
the dev box (a built DLL, a CDN, a still-awake board) that the fresh host didn't have.

---

## BUG-022 — Headless-host browser has no WebGL → viewer stuck "Offline"

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner, via the BUG-021 in-browser
  diag trace: `Uncaught Error: Error creating WebGL context @ three.module.js`) · **Area:** host/web
- **Where:** `host/src/roomscan/web.py` (`_open_browser`)

After BUG-021 vendored three.js, the diag trace showed the real wall: three.js loaded (`THREE r160`),
`app.js` ran, then `new THREE.WebGLRenderer()` (module top-level) threw **"Error creating WebGL
context"** — so execution aborted *before* `connect()`, no WebSocket ever opened, page frozen at its
initial "Offline". The host is a headless Proxmox/LXC box with no GPU; the VNC X server (`:1`) has
software OpenGL (llvmpipe, GL 4.5, confirmed via `glxinfo`), but **Chrome 150 refuses software WebGL
by default** (SwiftShader-for-WebGL is gated behind a flag since ~Chrome 137). Measured on-box:

| Chrome launch | WebGL |
|---|---|
| no flags (what the auto-open used) | **NO-WEBGL** |
| `--enable-unsafe-swiftshader` | OK (SwiftShader) |
| `--use-gl=angle --use-angle=swiftshader` | OK (SwiftShader) |
| `--use-gl=angle --use-angle=gl --ignore-gpu-blocklist` | OK (Mesa llvmpipe GL 4.5) |

`webbrowser.open` (the viewer's auto-open) passes no flags — exactly the failing row.

**Fix:** `_open_browser()` launches google-chrome/chromium with `--enable-unsafe-swiftshader` on
Linux (only *permits* the software fallback; a real GPU still uses hardware, so it's unconditional-
safe), falling back to `webbrowser.open`. `ROOMSCAN_NO_BROWSER=1` skips auto-open for remote viewing.
Verified: with the flag, Chrome on `:1` reports `WEBGL-OK`; the viewer renders Live (headless-Chrome
screenshot: status Live, ~2100 pts, point cloud). This was the 4th distinct headless-migration gap
(after BUG-020 loader, BUG-020 keepalive, BUG-021 CDN) — all "implicit on the dev box, absent on the
fresh host".

**The in-browser diagnostic panel (added this session) is what cracked it** — it turned an opaque
"Offline" into the exact throwing line. Keep it.

---

## BUG-023 — System clock sourced from the ST-LINK's MCO

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner: "I am now powering the device
  over USBUSER and have removed the STLINK connection… this is preventing me from launching the
  application (which does not depend on stlink, just ethernet)") · **Area:** firmware
- **Where:** `firmware/scanner-stream/Src/main.c` (`SystemClock_Config`),
  `53L9A1_PostprocessSingle.ioc` (`RCC.PLLSourceVirtual`/`PLLM`/`PLLN`/`PLLFRACN`)

The NUCLEO-H563ZI has **no HSE crystal fitted**. `OSC_IN`/`PH0` is driven by `STLK_MCO`, the 8 MHz MCO
output of the on-board ST-LINK MCU (MB1404 schematic sheet 9: `T_MCO` → R77 → buffer U10 → SB49/SB50 →
`PH0-OSC_IN`; X3's footprint is unpopulated) — which is why the generated config asked for
`RCC_HSE_BYPASS_DIGITAL` rather than a crystal. That buffer runs off `3V3_STLK`, derived from
`VBUS_STLK`, i.e. **from the ST-LINK USB cable on CN1**.

So with the ST-LINK unplugged and the board powered from USB_USER (JP2 moved to 9-10 per the
schematic's "In SINK mode, the jumper JP2 must be set to 'USB USER'"), the system clock simply had no
source. The firmware wedged before `ETH_Init()` and the board sat there **powered, PHY link LED on,
activity LED flashing, and the network completely silent** — no DHCP, no ARP entry, no mDNS, nothing on
UDP :5000. Those PHY LEDs come from the LAN8742's own power-on autonegotiation and the switch's
broadcast traffic, so they indicate nothing about whether firmware is running. Diagnosis: board absent
from a full `172.17.2.0/24` ARP sweep and from a raw mDNS query pinned to the board's NIC.

**Attempted and rejected:** detecting the `HAL_RCC_OscConfig` failure and falling back to HSI. It
rescued a *forced* HSE failure (`RCC_HSE_ON` with no crystal, ST-LINK attached) on the bench but **did
not rescue the real unplugged case**, so HSE's absence does not present as a clean `HSERDY` timeout.
Best theory: the undriven `OSC_IN` pin floats and self-oscillates on noise in digital-bypass mode, so
`HSERDY` sets and PLL1 then locks onto garbage instead of timing out. Do not try to detect this.

**Fix:** PLL1 sources from **HSI unconditionally**; HSE is never enabled. `PLLM 4 / PLLN 31 /
FRACN 2048` puts HSI/4 = 16 MHz in the same `VCIRANGE_3` input band the 8 MHz HSE used, giving the same
500 MHz VCO and the same 250 MHz SYSCLK — every downstream clock is bit-identical. USB runs off HSI48
and Ethernet off the LAN8742's own 25 MHz crystal, so neither transport is affected. The only cost is
HSI's ~1% RC accuracy on frame timestamps, measured as immaterial: **91.5 fps decoded-frame rate vs the
91.4 fps HSE baseline**, inside run-to-run noise. Owner-verified streaming with the ST-LINK physically
unplugged.

**Also added, because this failure was invisible:** boot-progress LEDs (`BootLedsInit` /
`rs_boot_heartbeat` in `main.c`) — all dark = never reached `main()`; LD3 red = wedged in init or
`Error_Handler()`/`handle_error()`; LD1 green = clocks + peripherals up; LD2 yellow blinking = the
acquisition loop is turning over. Note the heartbeat must live inside `vl53l9_app()`'s own `while(1)`:
that function never returns, so a heartbeat in `main()`'s outer loop fires exactly once at boot and
then sits dark while the board streams normally (this cost a diagnostic round-trip).

---

## BUG-024 — A missing CDC serial port aborted an Ethernet-only launch

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner, same session as BUG-023) ·
  **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` (`get_best_source`)

`get_best_source` probes UDP first and falls back to `SerialSource`, whose `find_port()` raises
`RuntimeError: no scanner serial port found among [...]` when no CAFE:4001 device is enumerated. On a
headless Ethernet rig — ST-LINK unplugged, USB_USER carrying power only — there is legitimately no
scanner serial port at all, so `roomscan-web` died with a serial traceback for a deployment that only
ever wanted Ethernet. It also lost a launch whenever the board happened to be booting (DHCP) during the
5 s probe window.

**Fix:** a *missing* port is no longer a launch blocker — Ethernet has been the production transport
since Phase 5. `get_best_source` hands back the UDP source instead, whose keepalive keeps re-waking the
board so the stream starts by itself once the device appears, and prints a clear one-line reason to
stderr. A *busy* port still raises: that means a real scanner port exists and something else holds it,
which `panel._open_source` offers to resolve interactively. Classification reuses
`portguard.classify_open_error`.

---

## BUG-025 — `UdpSource` retargeted onto its own looped-back broadcast wake

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (found while verifying BUG-024: the
  launch banner read `[source] Ethernet/UDP · 172.17.2.54` — the host's *own* address) ·
  **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` (`UdpSource.read`)

When mDNS resolves nothing, `_resolve_target` falls back to broadcasting the wake datagram. Linux loops
that 1-byte broadcast straight back into the same socket, and `read()` set `self.target_ip = addr[0]`
from **any** sender before checking the length — so the source adopted *the host itself* as the device
on its very first read, permanently. Every subsequent wake and keepalive then went to us instead of the
board, which is never told where to stream. Self-poisoning, and it would have broken Ethernet discovery
even with a perfectly healthy board (it is only masked when mDNS succeeds and supplies a unicast IP).

**Fix:** only a datagram long enough to be a real fragment (≥ 6 B, the fragment sub-header) may
retarget; short datagrams return `b""` without touching `target_ip`.

## BUG-026 — Web UI never gravity-aligned the view; "Reset Heading" cannot fix tilt

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner: "powering it up with the board
  upside down shows an upside down view in both IR and point cloud, and Reset Heading doesn't fix it")
  · **Area:** host/web
- **Where:** `host/src/roomscan/web.py` (`_broadcaster`), `host/src/roomscan/static/ir.js`

Gravity alignment existed **only in the deprecated desktop panel** and was never ported to the web app
when it became the primary UI (Web Phases 1–5). `panel.py:3020` rolled the IR pane with
`ir_gravity_rot`, and `panel.py:1332-1337` gravity-aligned the orbit-mode cloud with
`T_WORLD_TO_CV @ R @ T_CV_TO_BODY` — but `web.py` shipped raw deprojected CV-frame points and an
unrotated IR array. `ir_gravity_rot` was imported nowhere outside `panel.py` and its tests. The
orientation matrix *was* already on the wire (`build_sensor_message`'s `rot`), but its only consumer
was the 2D gizmo canvas; `scene.js` never read it.

"Reset Heading" is a red herring by construction, not a second bug: it sends `reset_fusion` →
`YawFusion.reset()`, which clears only the accumulated **yaw** delta. The fusion is built on
`graft_yaw`, whose contract is "roll/pitch are preserved" — tilt comes from the accelerometer and is
deliberately never touched (`docs/coordinate-frames.md`). No yaw reset can un-flip a view.

**Fix:** the broadcaster pre-multiplies POINT_CLOUD/SURFACE positions by `display_rotation(quat)` and
rolls IR_IMAGE via `ir_gravity_rot` (owner chose continuous full alignment, desktop-panel parity).
`ir.js` now sets `canvas.style.aspectRatio` from the message, because a 90°/270° roll makes the pane
portrait (42×54) and the CSS had a hardcoded `aspect-ratio: 4/3` that would have squashed it.
Verified live: the broadcast cloud's ray grid is destroyed (2094×2016 distinct directions) and `rot⁻¹`
recovers exactly the 54×42 sensor grid. Protocol note in `docs/web-protocol.md`.

**Follow-up (2026-07-29, owner: "the IR preview window needs to rotate too").** The fix above gave the
cloud *continuous* alignment but the IR pane only the panel's **quarter-turn snap**, so the two disagreed
by up to 45°: at ~40° of roll the snap is **zero turns** and the pane sat still while the cloud tilted the
full 40°. Half-ported parity, not a new root cause. The rotation is now split — the server keeps the snap
(pixel-exact, and it is what makes the dimensions swap) and publishes the ≤45° remainder as
**`ir_roll_deg`** on the `sensor` message; `ir.js` finishes it with a CSS transform, so the 54×42 image is
never resampled. `ir_roll_deg` is CCW-positive (`np.rot90`'s sense, which is CCW on screen too), so CSS —
which turns clockwise — negates it; the sign was established by a marker test, not by reasoning about
conventions. It is computed from the **smoothed** display quat, the same one the snap uses, or the two
disagree at a 45° boundary and the pane jumps. The image now spins inside a fixed **square** frame
(`#ir-frame`) scaled so the rotated bounding box always fits, so the card never changes shape and no field
of view is cropped (empty corners at intermediate angles; worst at 45°, where it exactly inscribes).
Verified by DOM readback at 14.6°/31.6°/44.6° residual (all `fits:true`; at 44.6° the bbox is exactly
278×278 in a 278 px frame) and live at 169° roll, where snap 180° + residual −11.0° sums to the 169° the
cloud uses. **Caught in review:** `build_sensor_message` already had a *local* `display_quat` (the
yaw-offset orientation view), so the new parameter of that name was silently clobbered and the residual
was being taken from the raw fused quat — renamed `ir_display_quat`.

**Follow-up 2 (2026-07-29, owner: "it's also rotating the contents — if a target is upright in one
orientation it should remain upright at all orientations").** The roll was applied in the **wrong
direction**, a sign inversion inherited from `panel.py`. `atan2(gx, gy)` measures *where gravity sits*;
the correction is its **negative**. Applying `+angle` turned the content the wrong way, so rather than
holding still it counter-rotated at **2x** the board's rate — worse than doing nothing.

Why the first round missed it: both verifications were **sign-blind**. The upside-down check used 180°,
where a sign flip is a no-op (−180 ≡ +180), and the 90° check only asserted the width/height swap, which
happens either way. Fixed by one negation in `ir_gravity_angle_deg` (so `ir_gravity_rot`,
`ir_gravity_residual_deg` and the CSS transform all follow).

The trap that caused it: `T_WORLD_TO_CV @ R @ T_CV_TO_BODY` rotates points in the **CV frame, where Y
points down**, so a positive rotation there is *clockwise* on screen — while `np.rot90` is
*counter-clockwise*. The intended angle was right all along and matched the cloud exactly; only the
application flipped.

**Proven two independent ways, neither of which is a restatement of the formula.** (1) Exact geometry: the
IR image and the cloud come off the same 54x42 grid, so wherever the *verified* aligned cloud puts the
image's +u axis on screen is where the rotated image must put it — that came out as exactly the negative of
what the code applied, at every angle tested. This is now the regression test
`test_ir_gravity_angle_matches_the_point_cloud_rotation`, plus
`test_applied_rotation_stabilises_rather_than_doubling` which asserts the 2x failure mode directly.
(2) Real data: on `captures/web_20260729_174331.bin` (a physical boresight roll over a 179° span, so the
scene genuinely co-rotates), the dominant edge orientation of the raw reflectance plus the applied rotation
has a spread of **15-18°** under the fix versus **24-48°** inverted and **46-60°** uncorrected — i.e. the old
convention was *worse than applying no correction at all*, the signature of double rotation. The ~15° floor
is the structure-tensor metric's own resolution on a 54x42 image, not residual error (it does not tighten as
edge quality rises). Owner confirmed visually on the same recording. Two tests that had *encoded* the
inversion (`test_sensors.py::test_ir_gravity_rot_roll_90_cw`/`_ccw`, asserting 1 and 3) were corrected to 3
and 1 with the reasoning written down.

## BUG-027 — SFLP quaternion aliased by unfiltered 480 Hz → 30 Hz decimation

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner, after BUG-026: "the point cloud
  is very slightly jittery now, especially noticeable at the edges") · **Area:** firmware
- **Where:** `firmware/scanner-stream/Src/rs_lsm.c` (`rs_lsm_read_latest`)

Gravity-aligning the cloud (BUG-026) multiplied the orientation signal by the scene's lever arm, which
exposed a latent firmware defect: SFLP runs at 480 Hz, the host consumes one sample per ToF frame
(~30 Hz), and `rs_lsm_read_latest` drained the FIFO keeping only the **last** quaternion of each
~16-sample batch. Point-sampling a 480 Hz signal at 30 Hz folds the entire 0–240 Hz noise band into
0–15 Hz — textbook aliasing, with no anti-alias filter anywhere in the chain. Measured on a stationary
rig: 0.14° mean / 0.25° p95 of change per frame, net 0.14° over 15 s against 22.9° summed absolute
(ratio 0.006 — essentially all noise), i.e. ~7 mm mean / 13 mm p95 of edge shimmer at 3 m.

Two adjacent defaults were wrong for this rig and fixed in the same pass: the gyro's **LPF1 was
bypassed** (POR default), leaving the chain 187 Hz wide at ODR 480 Hz (AN5763 Table 20, the ODR = 480 Hz
block — the 342 Hz row is 960 Hz); and **LIS2MDL `CFG_REG_B` sat at its 0x00 POR default**, i.e. the
4.5 mG RMS / ODR÷2 corner of AN5069 Table 9 with both the low-pass and offset cancellation off.

**Fix:** `RS_LSM_SFLP_AVERAGE` averages the batch (correct decimation, √N on white noise; sign-aligned
before accumulating since q and −q are the same rotation); `RS_LSM_GY_LPF1_*` enables gyro LPF1 at
28.4 Hz; `CFG_REG_B = OFF_CANC | LPF` moves the mag to 3.0 mG RMS / 25 Hz. **Measured, firmware alone,
host smoothing bypassed:** 0.0329 → 0.0118 deg/frame (1.72 → 0.62 mm of edge motion at 3 m), **2.8×**,
with streams 7/9/10 still at 30.3 Hz, 0 drops, 0 gaps.

**Remaining floor is the LSM's FIFO encoding, not the sensor.** The SFLP game-rotation vector is
batched as three **IEEE-754 half-precision** components with w reconstructed
(`sflp_word_to_quat`); the fp16 ulp near 0.7 is ~4.9e-4, and a perturbation δ in a quaternion's vector
part is ≈2δ of angle → ~0.056° per step. Dithered over 16 samples that predicts 0.014 deg/frame against
0.0118 measured, and the residual's directional coherence is **0.14** — *below* the ~0.32 of white noise
at window 10, i.e. anti-correlated, the signature of quantization dither rather than sensor noise or
tripod vibration. AN5763 §6.5: SFLP data is readable **from the FIFO only**, so there is no
higher-precision register path. ~~Because fp16 is floating point the floor should **vary with
orientation** (finer near identity) — an untested prediction. Beating it means leaving the SFLP FIFO
format (batch raw XL/GY and fuse host-side); **open, not attempted.**~~ Analysis + method in
`docs/iks4a1-stacking.md` → "Orientation-noise pass"; measure with `host/tools/orientation_probe.py`.

**Superseded 2026-07-29 — both open items above were closed:**

*Orientation dependence: **CONFIRMED**.* Two-point test (owner rotated the rig ~90°). Old pose fp16
step RMS 0.03956°, k_eff 4.85, model 0.01797° vs **measured 0.01782° (ratio 0.99)**; new pose step
0.04846°, k_eff 4.84, model 0.02202° vs measured 0.02670° (ratio 1.21). Step ratio new/old 1.225,
measured ratio 1.498 — right direction, ~21% under-predicted at the coarser pose.

*The floor is **dither-limited**, not step-limited — a quieter board measures WORSE.* Averaging 16
samples only buys √16 if input noise keeps the quantizer toggling. Measured k_eff ≈ **5 of 16**
(tie fractions 14–28%, holds up to 18 consecutive frames), corroborated independently by the 1.85×
excess of measured over naive model under √k scaling → k_eff ≈ 4.7. This explained an apparent
0.0118 → 0.0183 "regression" after a power cycle that was **not** a regression: last session
back-solves to k_eff ≈ 16 (well dithered). Nothing had broken.

*Escaping fp16: **shipped as stream 11** (`RS_STREAM_IMU_RAW`) — 480 Hz raw FIFO pass-through of
GY/XL/timestamp/SFLP-gravity/SFLP-gbias, all 16-bit fixed point. Verified on-target: 100.01% of
samples delivered, 0 gaps, median tick delta exactly 96 ticks = 480.0 Hz. Host complementary filter
`roomscan.imufusion` built and **gated OFF by default** (SLAM non-regression guarded by test);
synthetic gain 6.2× on tilt in the under-dithered regime. **Not yet wired into the live display —
that is the resume point.**

*Also note (2026-07-29): the fp16 floor turned out **not** to be what the owner was actually seeing.
The visible noise is the eCompass — see BUG-030.*

## BUG-028 — Ethernet hot-plug replug wedges board (lwIP double-add assertion)

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner) · **Area:** firmware
- **Where:** `firmware/scanner-stream/Src/ethernet_transport.c`

When the Ethernet cable is unplugged and plugged back in, the firmware wedged. The PHY link LED turned on, but LD2 stopped blinking, indicating an infinite loop. The root cause was that `ETH_Process` called `mdns_resp_add_netif` a second time on the same network interface after the DHCP lease was re-acquired. The ST port's `LWIP_PLATFORM_ASSERT` trapped the "Double add" assertion into an infinite `while(1)` loop without triggering a hard fault handler (so LD3 stayed off).

**Fix:** Introduced a static flag `mdns_added` to ensure `mdns_resp_add_netif` and `mdns_resp_add_service` are only called once. Subsequent IP updates now correctly rely only on `mdns_resp_netif_settings_changed`.

## BUG-029 — UdpSource stream recovery fails due to broadcast routing

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner) · **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` (`UdpSource._maybe_keepalive`)

Even after the firmware crash (BUG-028) was fixed, the `UdpSource` stream would not recover after a replug until the python server was restarted. The source was correctly detecting a stream timeout (>2s) but was falling back to sending its keepalive wake datagram to `255.255.255.255`. On many Linux/Docker host setups, raw broadcasts to `255.255.255.255` fail to route out of the physical Ethernet interface and instead go to a virtual bridge, meaning the board never received the keepalives.

**Fix:** Updated `_maybe_keepalive` to actively re-query the board's IP via mDNS (`_resolve_target`) every keepalive interval when the stream is dead. This learns the new (or same) IP and resumes unicast wake packets, gracefully restoring the stream.

## BUG-030 — Magnetometer calibration is direction-dependent: |B| ranges 47→85 µT with tilt

- **Status:** **fixed 2026-07-30** (owner re-fit, validated on independent data — see "RESOLUTION"
  at the end of this entry) · **Reported:** 2026-07-29 (owner: "most of the noise I see in the UI is
  in the eCompass") · **Area:** host/sensors (calibration data, not code)
- **Where:** `mag_cal.json` at the **repo root** — the path a `roomscan-web` started from the repo
  root resolves `ViewerConfig.mag_cal_path` to; consumed via `host/src/roomscan/magcal.py` →
  `MagCalibration.apply()`. (Was `host/mag_cal.json`, fitted 2026-07-15, `field_ut = 49.87`;
  **that file is deleted** — see "The two-file trap" below.)

A correctly calibrated magnetometer reports a **constant** field magnitude at every orientation —
that is the defining property. Ours does not. Measured on a deliberate braced tilt sweep
(2026-07-29, `captures/web_20260729_061440.bin`, 8 stationary holds, current calibration applied):

| tilt from vertical | 0.3° | 0.2° | 30.5° | 60.6° | 80.0° | 90.6° | 30.6° | 2.8° |
|---|---|---|---|---|---|---|---|---|
| heading | 147.0° | 144.4° | 157.9° | 149.7° | **79.6°** | **239.6°** | 158.1° | 146.2° |
| **\|B\| µT** | 50.5 | 47.4 | 58.7 | 76.5 | 81.1 | 85.1 | 62.8 | 50.7 |

Accurate at ceiling-facing (≈ the fitted 49.87 µT) and degrading monotonically to ~1.7× toward
horizontal — the signature of an **incomplete calibration tumble**: good coverage in one attitude
family, poor everywhere else. Consequence is not noise but **systematic heading error up to ~90°**,
and it occurs precisely in the horizontal wall-scanning attitude the device is actually used in.
(An earlier near-vertical tripod pose read ~107–109 µT, i.e. a 2.15× anomaly — consistent story.)
That deviation also exceeds `YawFusion.anomaly_frac` (0.3 → ±15 µT), so yaw fusion is silently
**gated off** at those poses while the displayed `heading` — which ignores the gates — keeps showing
the biased value.

**The compass noise is magnetometer-dominated, not orientation-dominated.** Holding the quaternion
fixed and varying only the magnetometer reproduces essentially all of the heading jitter at every
attitude (mag-only 1.297° of 1.299° total at 0°; 0.780° of 0.779° at 30°; 1.185° of 1.187° at 60°).
Tilt-error propagation contributes 0.004–0.05° below 60°, rising to 0.182° at 80° and 0.944° at
90.6° (the DT0058 gimbal blow-up — real, but an order of magnitude below the calibration error).
Orientation-estimate jitter itself is excellent throughout: p95 0.006–0.079°/frame.

**Application-note review 2026-07-29** (`docs/imu-mag-appnote-review-2026-07-29.md`) narrowed this:
- **Ruled out:** magnetometer byte-tearing (missing BDU was real and is fixed in `46b81b3`, but measured
  on 27339 stationary samples the max jump is 23 LSB with ZERO above 64 LSB — no tears); **installation
  error** (DT0103 — de-rotation preserves magnitude, so it cannot change |B|); and `AXIS_CONVENTION`
  (an orthogonal sign matrix cannot change magnitude).
- **Arithmetically excluded:** a simple hard-iron residual. Reaching the observed 85 µT max from a
  ~50 µT field needs |δ| ≈ 35 µT, which would drive the minimum to ~15 µT; measured minimum is 47.4 µT.
- **Still live**, both consistent with the 85.1/47.4 = **1.80** max/min ratio: a soft-iron/diagonal-gain
  error the fit mis-estimates (follows the device anywhere), or a **world-fixed interferer — most
  likely the tripod**, since tilting on it *translates* the sensor through an arc past ferrous mass.
  The latter also explains the ~66 µT mean vs the 49.87 µT fit if the original tumble was hand-held.
- **Therefore calibrate HAND-HELD in open space, away from the tripod.** Calibrating while mounted
  would bake a position-dependent error into the fit that misbehaves in handheld use — the actual use
  case. Discriminating test (~2 min): a hand-held level/45°/vertical check; flat |B| implicates the
  tripod, a persistent 1.8:1 swing implicates the soft-iron model.
- **Also found (DT0103, separate from |B|):** a general ellipsoid fit is **ambiguous up to a rotation** —
  it can yield a perfectly constant |B| while systematically rotating the field vector, i.e. right
  magnitude, wrong heading. 3D fitting cannot detect this; DT0103's accelerometer-assisted method pins
  the magnetometer frame to the body frame. Worth adopting for heading accuracy independently of BUG-030.

## ROOT CAUSE (2026-07-29) — two faults stacked; fixing one does not fix the other

An owner hand-held tumble in open space (`captures/web_20260729_175941.bin`, 718 samples, ~24 s)
produced a candidate fit that is **validated on independent data**:

| capture (under the candidate fit) | \|B\| mean | std | ratio |
|---|---|---|---|
| tumble 17:59 — hand-held, *fit source* | 49.85 µT | 0.65% | 1.05 |
| tumble 17:43 — hand-held, **independent** | **50.06 µT** | **1.23%** | 1.07 |
| tilt sweep — on tripod, moving | 77.18 µT | 11.25% | 1.83 |
| stationary 900 s — on tripod, fixed 86° | 64.64 µT | **0.46%** | 1.04 |

**Fault 1 — the saved hard-iron offset was wrong by ~59 µT.** Saved `[44.4, −27.6, −41.7]` vs the new
`[25.4, −5.5, +9.2]`, mostly in z. Both soft-iron matrices are near-identity (candidate eigenvalues
0.966/0.991/1.045, axis-gain ratio 1.08), so **soft-iron was never the problem**. Under the saved
calibration the tumble spans 14.7 → 107.6 µT, ratio **7.3**.

**Fault 2 — the tripod is a genuine magnetic interferer, ~15–27 µT (a third to a half of Earth's
field).** See BUG-034; heading is unreliable while the device is mounted, *regardless of calibration*.
The proof is row 4 above: held stationary on the tripod at a fixed attitude, |B| is exquisitely
consistent (0.46%) but at the **wrong magnitude** (64.6 vs 49.9) — an added field is steady when
nothing moves. Row 3, where the sensor swings through an arc on the tripod, becomes 11.25% because the
added field varies with **position**, not orientation.

**Two earlier conclusions in this entry were WRONG and are corrected here** (kept, not deleted, per
repo convention):
- ~~"a simple hard-iron residual is arithmetically excluded"~~ — the arithmetic was right (|δ| ≈ 35 µT
  implies a minimum near 15 µT) but it was applied to a **one-dimensional** tilt sweep that never
  visited the cancelling orientation. The full tumble reaches **14.7 µT**, exactly as predicted.
  Hard iron was the answer all along.
- ~~the tilt sweep is valid independent validation data~~ — it is tripod-contaminated, so testing a fit
  against it measures the tripod. See the `validating-against-suspect-data` memory.

**Remedy (owner, via the UI — in progress):** re-fit and **Save & apply** in the calibration modal,
which also exercises the hot-reload path not yet tested with a real fit. Coverage on the 17:59 tumble
was 54/92 cells (59%), just under the tool's 60% marginal bar, so more tumbling raises confidence —
but the field metrics are good *and* generalise to unseen data, which is the stronger evidence.
Calibrate **hand-held in open space, away from the tripod**: calibrating while mounted would bake a
position-dependent error into a fit used hand-held. ~~`host/mag_cal.json` is still the 2026-07-15 file
until that is done.~~ Sources: DT0058, DT0059, DT0103, AN5069 §5/§8.

## RESOLUTION (2026-07-30) — owner re-fit, validated against a capture the fit never saw

The owner ran a full tumble through the calibration modal **hand-held, off the tripod, with the
metal tripod mount plate still attached** (body-fixed, so its hard iron is calibratable — and it is
now baked into the fit; **removing the plate invalidates the calibration**). Saved via **Save &
apply**, which exercised the hot-reload path for the first time with a real fit. Then recorded an
independent 118 s room sweep, `captures/roomSweepFull20260730.bin` (3574 mag samples at 30.3 Hz,
streams 7/8/9/10/11/12, walking the room).

New fit: hard iron `[24.8, −5.8, +8.6]` (|offset| 26.9 µT), `field_ut` 48.09, soft-iron axis-gain
ratio **1.091**. That offset sits **58.3 µT** from the superseded one — matching the ~59 µT the root
cause above predicted.

Scored with the new `host/tools/mag_check.py` (`capture_magcheck` MCP tool):

| metric on the sweep | new fit | superseded 2026-07-15 fit |
|---|---|---|
| **attitude-locked error** (the verdict) | **0.29 µT = 0.56%** → good | 3.44 µT = 3.81% → marginal |
| **tilt ramp**, \|B\| max/min across 10 tilt bins | **1.042×** → good | 2.721× → bad |
| \|B\| by tilt, 0° (ceiling) → 90° (horizontal) → 180° | 51.5 → 51.5 → 52.7 (flat) | 40.3 → 92.1 → 109.5 (ramp) |
| `YawFusion` `gated:anomaly` | **0%** | 58.6% |
| `YawFusion` `active` | **64.8%** | 6.2% |

The gating row is the practical consequence: fusion was silently off for most of a scan and is now
on (the remainder is `gated:motion` 30% / `gated:gimbal` 5%, both by design). Live on the desk
afterwards: `has_mag_cal=true`, `fusion="Active"`, |B| 50.24 µT over a 49.76–50.87 range.

**The raw spread is NOT the calibration.** `field_consistency` scores this good fit "bad" on the
sweep (std 4.58%, bias +6.91%) — that metric assumes every sample came from one place, which is true
of a tumble and false of a 118 s walk. Detrending |B| with a 5 s rolling median splits the 2.35 µT
total std into **2.00 µT of slow spatial drift** (the building's ferrous mass moving the ambient
field under the operator — |B| walks 49.9 → 53.6 → 48.3 → 54.4 across the room while the *within-bin*
std is often 0.4 µT) and only 0.29 µT locked to body attitude. See `magsweep.attitude_locked_error`.

**Still unproven: heading direction.** |B| flatness proves magnitude, not direction — an ellipsoid
fit is ambiguous up to a rotation (DT0103, noted in the app-note review above), which would give the
right magnitude at a systematically rotated heading. The near-spherical soft iron bounds that at
~2.5°, but the sweep cannot measure it: there is no ground truth in it, and a mag-vs-SFLP-yaw proxy
is invalid because `quat_yaw_deg` is ZYX yaw about body **Z (Forward)** while the SFLP body frame has
**X = Up** — that is not heading. The test that would settle it is a braced, fixed-compass-heading
tilt sweep through level → 45° → vertical, hand-held.

### The two-file trap (found while validating this)

`ViewerConfig.mag_cal_path` is **relative**, so it resolves against the server's cwd — the repo root
in practice. The tracked calibration lived at `host/mag_cal.json`, which the running server therefore
never loaded: it had **no calibration at all** from boot until the 07:22 save created the root file.
Anything run from `host/` (`tools/mag_calibrate.py`, the deprecated panel) would meanwhile have
silently applied the superseded fit.

Fixed by keeping **one** tracked file, at the root, and deleting `host/mag_cal.json`. A missing file
fails loudly and safely (`has_mag_cal=false`, `fusion="gated:no-cal"` in the UI); a stale one is
silently wrong, which is what happened here.

## BUG-031 — ToF frame timestamp and IMU FIFO drain are skewed ~0.9 ms

- **Status:** **fixed** 2026-07-30 (stream 13 IMU_SYNC) · **Reported:** 2026-07-29 (analysis) ·
  **Area:** firmware
- **Where:** `firmware/scanner-stream/Src/vl53l9_app.c` (frame stamp) vs `Src/rs_lsm.c` (FIFO drain)

For a **handheld** scanner, timing error between a depth frame and the IMU sample used to orient it
dominates the orientation-noise floor: at 100 °/s, 1 ms of skew is 0.1° of misalignment — ~10× the
stationary quantization floor. Measured against the LSM's own FIFO timestamps: **1.9 ms RMS /
3.4 ms p95 / 6.2 ms max**. Two causes — `rs_time_us()` was `HAL_GetTick() * 1000` (1 ms granular),
and the stamp was taken at *send* time, so variable processing/transmit latency folded in.

**Mitigated 2026-07-29** by a TIM2-based microsecond clock and moving the stamp to the sensor's
FRAME_READY edge (end of integration). **Measured after: 1072 µs RMS** — better, but far from the
predicted "tens of µs". Note the measurement itself has a ~600 µs floor (it compares against the
*last* IMU sample of each batch, whose phase varies by up to one 2.083 ms sample period), implying
~890 µs of genuine residual.

**Costed 2026-07-30 → recommendation: keep SFLP, do not take ODR-triggered mode.** Full note:
`docs/odr-triggered-sync-costing-2026-07-30.md`. (a) The gating precondition is untested — the
host-side alternative had a broken heading loop (**BUG-039**; *fixed 2026-07-30, and the
recommendation is unchanged* — the precondition went from unmeasurable to merely unmeasured, since
no capture in the repo carries orientation ground truth). (b) INT2 *is* routed and free
(CN9.5 → PE14 = TIM1_CH4), so availability does not settle it — but AN5763 Table 15's **40 ms
minimum T_ref** vs our very stable **33.00 ms** frame period forbids 1:1, and **480 Hz is absent
from the ODR-triggered ODRsel set**. (c) Losing SFLP also removes gravity *and* gbias from stream 11:
raw accel exits the 5% gate **23.1%** of room-sweep frames (SFLP gravity: 0/3000), and gbias is
0.18–0.20 °/s with no host replacement (`KI_BIAS_HZ = 0.0`). (d) Stream 13 `IMU_SYNC` (below) makes
the skew a *measured* hardware quantity with SFLP left on, which removes the last thing
ODR-triggered mode was going to buy.

⚠ **One premise of that costing note is now known wrong.** Its §2.3 assumed the frame-ready stamp sits
at the FIFO batch *end*, and concluded the quaternion was ~15.4 ms **stale**. Stream 13 measured the
actual geometry: the drain sits **+24.3 ms past** the edge, so the quat batch midpoint is **+7.76 ms
AFTER** it — the orientation **leads** the depth frame by ~0.30° at 38.5 °/s. Correcting it in the
"stale" direction would roughly *double* the error. The note's structural conclusions (a)–(c) are
unaffected; its sign is not.

**Remaining root cause (hypothesis, since confirmed and fixed):** the IMU FIFO is drained *later* in the loop
than the ToF frame-ready stamp, so the offset between them still breathes with processing load. The
principled fix is to capture the LSM timestamp **at the frame-ready moment** rather than inferring it
from whichever FIFO words happen to be present at drain time. Verification requires **real motion** —
both defects are invisible on a stationary rig.

**A hardware option, with a hard trade-off (found 2026-07-30 while reading DT0155).** ST's
**ODR-triggered mode** phase-locks the LSM6DSV16X's data generation to an external reference on the
**INT2** pin — the device aligns both frequency and phase to that signal's edges (DT0155,
*"Synchronizing multiple sensors using ODR-triggered mode in MEMS devices"*). Driving INT2 from the
ToF frame clock would make this skew a *hardware* property instead of a software race, which is
strictly stronger than any drain-ordering fix.

⚠ **But it is mutually exclusive with SFLP.** AN5763, verbatim: *"ODR-triggered mode is not compatible
with the pedometer, relative tilt, **SFLP**, DRDY mask, or activity/inactivity functionality (only
motion/stationary can be used)."* It also forbids some ODR settings (CTRL1 `ODR_XL` 0001/0010/1100,
CTRL2 `ODR_G` 0010/1100) and is incompatible with Qvar/EIS. So it is a choice, not an addition:

- **Keep SFLP** (stream 9, the on-chip game rotation vector that `Mapper` uses as its rotation prior)
  and live with a software-timed skew; or
- **Take ODR-triggered sync** and fuse orientation on the host from raw XL/GY — which is exactly what
  stream 11 (480 Hz raw IMU FIFO) and the currently gated-off `roomscan.imufusion` already exist for.

Worth costing before more software mitigation: option 2 removes the skew at the source but puts the
whole orientation path on the host, so it needs `imufusion` to first prove it beats SFLP.

**Costed and declined 2026-07-30** (`docs/odr-triggered-sync-costing-2026-07-30.md`): keep SFLP.
`imufusion` does not currently beat it, AN5763 Table 15's 40 ms minimum `T_ref` does not fit a
33 ms frame period, and 480 Hz is not in ODR-triggered mode's ODR set. The fix below removes the
last thing it would have bought.

### Fixed 2026-07-30 — measure the correspondence instead of inferring it (stream 13)

**The hypothesis was tested before anything was built, and it held.** Every 64th frame also carries
the 2332-byte CALIB blob, which is sent *before* the FIFO drain — a load experiment the firmware
was already running for free. Over 5331 static frames those frames' pairing sat **655 µs later**
than a plain frame's (Welch t = **−5.8**; reproduced at −688 µs / t = −6.1 and −783 µs / t = −4.9
on two later captures). The gap between the frame stamp and the drain therefore moves with
processing load, exactly as predicted.

**The fix:** the firmware now reads the LSM's `TIMESTAMP` register (0x40–0x43) **at the FRAME_READY
edge itself**, in the one window where the shared I3C bus is idle — immediately after the event ack,
before the ToF's DMA readout is kicked — and ships it as **stream 13 (IMU_SYNC)** with the delay
from the edge and the read's own duration, so the residual uncertainty is *reported* rather than
assumed. Costs one 4-byte register read per frame (**26.4 µs, std 0.5**) and no frame rate
(30.26 → 30.29 fps, 0 CRC failures, 0 gaps over 15692 frames).

**Before / after**, same rig, same static scene, same estimator family, windowed 2 s clock fits
(`host/tools/skew_check.py`, MCP `capture_skew`):

| | RMS | p95 | max | frame-to-frame |
|---|---|---|---|---|
| **before** — inferred from the last FIFO word (5332 frames, 176 s) | 1069.9 µs | 1260.6 µs | 5745.9 µs | 1628.7 µs |
| **after** — measured at the edge (3119 frames, 103 s) | **18.3 µs** | **35.8 µs** | **82.3 µs** | **11.0 µs** |

58× on RMS. Two honesty notes on those numbers:

- The old estimator is **unchanged** by the fix (1069.9 → 1050.1 µs on the after-capture) and always
  will be: it measures the drain's phase and lag, which are still there — we simply stopped
  depending on them. The two rows answer "how well can we place a ToF frame on the IMU clock",
  which is the question, not "did one number move".
- The 18.3 µs is **window-dependent** (18 / 38 / 150 µs at 2 / 5 / 20 s) because what survives is
  the two oscillators drifting against each other, lag-1 autocorrelation **0.992** — not per-frame
  skew. The window-free number is the residual's first difference: **11.0 µs**. The old estimator
  is window-*in*dependent (1070 / 1073 / 1085 µs) because its error is white, and it can never
  beat 601 µs anyway (one 2.083 ms sample period of FIFO phase, uniform).

**What the measurement then exposed — and it is bigger than the bug it closes.** With the edge
finally on the LSM's clock, the drain turns out to sit **+24.3 ms** past it (std 848 µs — which is
the "breathing" the host-side residual had inferred at ~900 µs, from the other side). That kills a
premise everyone had been reasoning from, including the ODR costing note above, which assumed the
FRAME_READY stamp sat at the *batch end*. It is 23.1 ms from it. Consequences, measured over 3119
frames:

- Stream 9's quaternion is a **batch mean** (`RS_LSM_SFLP_AVERAGE`, shipped for BUG-027's 2.8×
  noise cut), so it carries the batch's **midpoint** orientation. That midpoint sits **+7.76 ms
  AFTER** the frame-ready edge (std 0.68) — the batch straddles the edge.
- So the orientation attached to a depth frame **leads** it by ~7.8 ms (~0.30° at 38.5 °/s), ~9×
  the skew this bug was about. It is **not** ~15 ms stale, and a correction that propagates the
  quaternion *forward* — the natural reading of "stale" — would roughly double the error.
- Stream 13 therefore also carries `quat_mid_ticks` + `quat_n`, and
  `roomscan.protocol.ImuSync.quat_offset_us()` returns the signed offset, so nobody has to
  re-derive the sign from a capture. Firmware's value agrees with an independent host-side
  computation from stream 11's timestamps to **0.02 ms** (+7.74 vs +7.76).

**Still open, and deliberately not done here:** nothing yet *applies* that correction. The host
still pairs stream 9 with a frame by `seq` and uses it as-is, so the ~7.8 ms lead is measured,
documented and unremoved. Correcting it means propagating the averaged quaternion backward with
stream 11's gyro words — a change to the orientation the SLAM prior and the whole UI read, which
wants its own before/after on a moving capture (`docs/superpowers/plans/2026-07-29-orientation-resume.md`
§4.5). Filed as the next step rather than smuggled in behind a timing fix.

---

## BUG-032 — GPU SLAM OOMs on a long scan (Open3D CUDA cache grows per extraction)

- **Status:** **fixed** 2026-07-29 · **Reported:** 2026-07-16 (CUDA at-scale validation, finding #4) ·
  **Area:** host/slam
- **Where:** `host/src/roomscan/slam/tsdf.py` (`_extract_vbg` / `_release_cache_if_due`)
- **Sub-phase:** ROADMAP 6.G

Over a 68 m walk the GPU SLAM path crept to **~11.7 GB** of CUDA memory and died on a `ParallelFor`
allocation failure, capping how long an unattended GPU scan could run.

**The stated hypothesis was wrong in an important way.** It had been attributed to "the caching
allocator + **per-frame** temporaries never released". Measured with the new rig
(`host/tools/slam_gpu_memory.py`, which logs NVML device bytes *and* active block count per frame so
map-growth and work-growth can be separated):

- **the per-frame path does not leak at all.** 4000 frames / 80 m of integrate + raycast + ICP with no
  extraction: device memory stayed **byte-identical** at 937361408 B while the map grew 900 → 17k blocks.
- **the throttled extraction does.** Add `mesh()` at the live cadence (`SlamWorker._MESH_EVERY = 5`) and
  memory climbs 523 → **5483 MiB over 1500 frames (5.13 MiB/frame)** with the block count nearly flat.
  On this 8 GiB card that OOMs a few hundred frames later.

Mechanism: `_extract_vbg()` does a whole-grid `self._vbg.cpu()` copy (itself the fix for CUDA bug #3 —
marching cubes OOMs on-GPU). Its temporaries scale with the active-block count, so each extraction asks
Open3D's caching allocator for a slightly **larger** block than the last, and the previous one is cached
but never reused again.

**Fix:** `TsdfMap.release_cache_every` — `o3d.core.cuda.release_cache()` after every Nth *extraction*
(default 1; 0 disables; no-op on a CPU grid). Extractions, not frames, is the right cadence: a
frame-counter release would fire mostly on frames that allocated nothing. Exposed as
`[slam] release_cache_every` and plumbed through `Mapper`, the `roomscan-slam` CLI, and
`web.SlamRunner`; the remote backend forwards it in its existing mapper-kwargs JSON.

**After:** 4000 frames / 80 m — **longer than the 68 m walk that OOM'd** — peak **651 MiB**, ending at
523 MiB, tail growth **0.005 MiB/frame**. No per-frame cost: p50 6.1 ms unchanged, p90 7.0 → 7.1,
p99 8.6 → 8.8, wall 62.3 s → 62.7 s over an identical 1500-frame run. The unfixed run would have needed
~21 GB to reach the same frame count.

**Guard:** `tools/slam-container/cuda_smoke.py::run_memory_ceiling` — 1200 frames at the live extraction
cadence, asserting peak ≤ 1500 MiB, tail growth ≤ 0.5 MiB/frame, and that releases actually happened
(so the knob being disabled, or the hook being unwired, fails loudly). Unit coverage for the cadence and
wiring is in `host/tests/test_slam_tsdf.py` (monkeypatched, so it runs on a CPU box).

---

## BUG-033 — Sensors card outgrew the dock band; readouts unreachable and half-duplicated

- **Status:** **fixed** 2026-07-29 · **Reported:** 2026-07-29 (owner: "clean up our sensors panel,
  it's cluttered and hard to read") · **Area:** host/web
- **Where:** `host/src/roomscan/static/index.html` (`#sensors-card` markup + the `.sensor-*` CSS),
  `host/src/roomscan/static/sensors.js`

The card had accreted four flat blocks — Sensors, Orientation View, Raw Orientation, Jitter — plus two
button pairs, ~25 rows in a 232 px rail. Two consequences, one cosmetic and one functional:

- **Functional:** at ~1600 px the card was roughly twice the dock band (883 px at a 1000 px viewport).
  `.dock > * { max-height: 100% }` capped it and the overflow was simply *clipped* with no scroll, so
  the Jitter table — the whole point of the 2026-07-28 noise work — could not be read at any window
  size; and `layout.js`'s degradation ladder ends in `autoCollapse('sensors-card')`, so a narrow
  window hid the card entirely rather than a part of it.
- **Cosmetic but load-bearing for readability:** the numbers were *duplicated*. "Orientation View" and
  "Raw Orientation" both printed Roll/Pitch/Yaw, and in the default `zyx` mode those are the same
  quantity to the digit; heading appeared three times (compass caption, raw heading, and the
  mislabelled "Heading" row that actually showed *fusion state*); World mode printed its gravity+mag
  caveat twice, because `#orient-world-note` said what `ORIENT_MODE_DESC.world` already said. On top
  of that the shared `.hud-row` 16 px gap plus long strings ("p95 0.026° · mean 0.006°",
  "Orientation (always trustworthy)", "No mag calibration") wrapped nearly every row onto two lines,
  and the `<select>` clipped its own longest option mid-word.

**Fix — three always-visible tiers plus one disclosure.** Visible: gizmo + compass → full-width mode
`<select>` → the three (renamable, now borderless-until-hover) orientation values → conditional ⚠
warnings → `Fusion` state, colour-coded by `fusion_key` (green active / amber gated / muted off), with
`Reset Heading` + `Calibrate Mag` beside it → Environment. Inside a collapsed `#sensor-diag`
`<details>`: the mode's singularity note, yaw offset + `Zero Yaw`/`Clear`, full-precision raw ZYX +
quat, and Jitter as a `label · p95 · mean` grid under one unit heading. **Precision was not reduced
anywhere** and the only element removed is the duplicate world note (with its now-dead
`worldNote` binding).

**Two gotchas worth keeping:**

1. **Chrome does not pass a flex-shrunk `<details>`'s height to its content.** The obvious
   construction — `<details>` as a flex column, `.sensor-details__body` as the `flex: 1 1 auto;
   min-height: 0; overflow-y: auto` scroll box — measured `det h=376` with `body h=426`: the body kept
   its natural height and spilled past the card, because Chrome renders details content in an
   anonymous `::details-content` box, so the body is not a direct flex item. The scroll box has to be
   the **`<details>` itself** (`min-height: 0; overflow-y: auto`), with the summary `position: sticky`
   and an *opaque* background so scrolled rows don't read through the glass card.
2. **The scroll container needs its own `pointer-events: auto`.** `.hud-card * { pointer-events: none }`
   sets it explicitly on every descendant, so it defeats inheritance — a deliberate, scoped exception
   to the left dock's "never intercept OrbitControls" rule, justified because the drawer already held
   pointer-active buttons and the wheel would otherwise dolly the camera instead of scrolling.

No `/ws` message, field, or precision changed — `docs/web-protocol.md` is unaffected. Verified in
headless Chrome against `captures/tilt_sweep_20260729.bin` at 1600×1000 and 1280×720: card within the
band in both states (`card=865` / `603`, zero overflow past the dock), drawer scrolls to the full
jitter table, and mode switching, World-mode gating (`Zero Yaw` disabled, ⚠ invalid), zero-yaw/clear
and label rename (propagating to the jitter row labels) all still round-trip through the server.
959 tests, 0 console errors.

## BUG-034 — The tripod adds 15–27 µT: heading is unreliable while mounted

- **Status:** **by-design** (environmental, not our code — recorded so it survives BUG-030's closure) ·
  **Found:** 2026-07-29 while root-causing BUG-030 · **Area:** environment / operating procedure
- **Where:** no code. A property of the physical rig.

Separated from BUG-030 deliberately: BUG-030 is a calibration-data fault that an owner re-fit closes,
whereas this is a permanent constraint that must outlive it.

Measured with a magnetometer calibration that is *known good* (validated on two independent hand-held
tumbles at 49.85 µT / 0.65% and 50.06 µT / 1.23%):

| condition | \|B\| mean | std | interpretation |
|---|---|---|---|
| hand-held, open space | ~50 µT | 0.65–1.23% | Earth's field, correct |
| tripod, **stationary** at a fixed attitude | 64.64 µT | **0.46%** | added field — steady, wrong magnitude |
| tripod, tilting through an arc | 77.18 µT | 11.25% | added field varies with **position** |

The stationary row is the diagnostic tell: **exquisite consistency at the wrong value is the signature
of an added constant**, not of model error or noise. Tilting on a tripod *translates* the sensor through
an arc past ferrous mass (head, centre column, screws), so the field at the sensor changes with position
rather than orientation — which is why the tilt sweep looked like a calibration failure.

**Consequences:**
- **Never calibrate the magnetometer while mounted** — it bakes a position-dependent error into a fit
  that is then used hand-held (the actual use case).
- **Do not trust heading, `absolute_heading`, or `YawFusion` output from tripod-mounted captures**, and
  do not use such captures to validate a calibration. Existing tripod captures
  (`captures/web_20260729_061440.bin`, `captures/stationary_stream11_20260728_190311.bin`) are affected.
- Tripod captures remain perfectly good for ToF, orientation-noise (gravity/gyro) and SLAM work — this
  is a magnetometer-only constraint.

**The tripod is only the loudest instance (2026-07-30).** The *room* does the same thing at a smaller
amplitude: over BUG-030's validation sweep, |B| under a known-good calibration walked 49.9 → 53.6 →
48.3 → 54.4 µT as the operator crossed the room, while the spread *within* any 10 s window stayed
around 0.4 µT. That is ~±6% of position-dependent ambient field with no tripod involved, and it is
irreducible — no calibration can subtract a field that depends on where you are standing. Practical
consequences: judge a calibration by `magsweep.attitude_locked_error` (which detrends this out) rather
than by raw |B| spread, and expect `YawFusion.anomaly_frac` (0.3) to be doing real work indoors.
- Untested: whether a different mount, or more separation between the sensor and the tripod head, brings
  |B| back to ~50 µT. That would confirm the mechanism and might restore mounted heading.

---

## BUG-038 — The live point cloud is frustum-culled against a stale, zero-radius bounding sphere

*(Filed 2026-07-30 as a second "BUG-033" — an ID collision with the sensors-card entry above.
Renumbered to BUG-038 on 2026-07-30 per this tracker's "IDs are never reused" convention. Commits
and docs written before the renumber may still refer to this as BUG-033.)*

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 (surfaced by the FPV view mode) ·
  **Area:** host/web frontend
- **Where:** `host/src/roomscan/static/scene.js` (`points` / `uncovPoints` / `surfaceMesh`)

Three.js computes `BufferGeometry.boundingSphere` **lazily, once**, and never invalidates it. The live
cloud rewrites its position attribute every frame, so the sphere that gets cached is the one from the
*first* render — before any `POINT_CLOUD` arrived, when the 300k-vertex buffer was still all zeros.
That is a sphere of **centre (0,0,0), radius 0**, and it never updates again.

World mode survived this purely by luck: its default camera sits at (0.5, 0, -1.5) looking at (0,0,1),
so the origin falls inside the frustum, the degenerate point-sphere "intersects", and the object is
drawn. It was a live tripwire the whole time — orbiting until the origin left the frustum would have
blanked the cloud.

**The FPV/Mirror view modes step straight on it.** They park the camera *at* the origin, which puts the
cached centre behind the near plane (`distanceToPoint = -0.02 < -radius = 0`), so the cull test fails
and the entire cloud disappears — a black viewport with no error, while the data was arriving and
correct all along. Confirmed by probing the first vertex's NDC in the render loop: `(0.53, -0.68, 0.95)`,
comfortably inside the frustum, with `frustumCulled=true` and `bs=(0,0,0)/r0`.

**Fix:** `frustumCulled = false` on the three live-updating objects (`points`, `uncovPoints`,
`surfaceMesh`). Recomputing the sphere per frame is the alternative and is far more expensive — a
300k-vertex pass that ignores `drawRange` — for objects that are always in front of the viewer.

*Diagnostic lesson:* the symptom (black viewport) looked like a camera-placement bug, and the frame
algebra was re-derived twice before instrumenting. The decisive measurement was three numbers printed
from inside the render loop — projected NDC, `boundingSphere`, `frustumCulled` — which separated "the
geometry is in the wrong place" from "the geometry is fine and something refused to draw it".

---

## BUG-035 — TSDF block capacity sat below a real room sweep; the map silently stopped

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 · **Area:** host/slam
- **Where:** `host/src/roomscan/slam/tsdf.py` (`DEFAULT_BLOCK_COUNT`, `_check_saturation`)
- **Found by:** replaying the owner's first full room sweep
  (`captures/roomSweepFull20260730.bin`) through the sub-phase 6.G memory rig.

`TsdfMap` hard-coded a `block_count` of 40,000 and plumbed it nowhere — no `Mapper` kwarg, no `[slam]`
key — so it could not be raised without editing source. That 40,000 turned out to sit *below* what one
real room scan needs, and running right at it broke the scan.

**Measured on the owner's sweep** (3525 depth frames, 1 cm voxels, CUDA:0):

| `block_count` | peak blocks | tracking lost | fitness at frame 3500 |
|---|---|---|---|
| 40,000 (old default) | 38,937 — **saturated** at 97.3% | **560 / 3525** | 0.10 |
| 120,000 | **42,917** (true demand) | **11 / 3525** | 1.00 |

The scan needs 42,917 blocks — **7% more than the old default**. It overshot by a hair, and the failure
was not graceful:

- blocks stop growing at frame **2879** (38,937, i.e. 97.3% of capacity) and never move again;
- tracking-lost begins at frame **2909** — **30 frames later**. Because SLAM is frame-to-model, once
  the map stops gaining geometry, ICP has nothing fresh to register the next frame against;
- median ICP fitness **0.887 → 0.127**; **0** lost frames before saturation, **560** after;
- **646 frames — 18% of the scan** — produce a ruined trajectory and no new map;
- Open3D also emits `stdgpu::vector::size : Size out of bounds: -2 not in [0, 18275]. Clamping to 0`,
  i.e. internal state going bad rather than a clean "full" signal.

Nothing logged. The scan simply stopped mapping, and it was only visible here because the 6.G rig
records the active block count next to the memory every frame.

**⚠ The mechanism is NOT "the grid cannot grow" — that was wrong and is corrected here.** An earlier
version of this entry said the `VoxelBlockGrid` pre-allocates and never grows. It does grow: driving a
CUDA grid across the boundary shows a clean rehash **40,000 → 80,000 at 99.2% load**, continuing to
52,027 blocks with no errors, identically on CPU and CUDA. The real 40,000 run froze at **97.3%** —
just *below* that rehash trigger.

Best current hypothesis, **unproven**: insertion failures in the ~97–99% load band beneath the rehash
threshold, which the `stdgpu` underflow message is consistent with. The competing explanation — that
ICP degraded first and the frozen block count is a symptom rather than a cause — is not excluded by
frame ordering alone, though it does not explain why 3× the capacity fixes it. What *is* established
is the effect and the mitigation: 560 lost frames at 40,000 vs 11 at both 120,000 and 160,000, on the
same capture, measured twice.

**Fix:** `DEFAULT_BLOCK_COUNT = 160000` (~3.7× this scan, ~2.3 GiB of pre-allocated device memory at a
measured ~14.2 KiB/block — 1707 MiB at 120,000), plumbed through `Mapper(block_count=…)`,
`[slam] block_count`, the `roomscan-slam` CLI, `web.SlamRunner`, and `slam_gpu_memory.py --block-count`.
Plus `TsdfMap._check_saturation()`, which warns **once** at 90% of capacity naming the config key — the
defect was never really "40,000 is too small" (any fixed number can be), it was that crossing the limit
was invisible.

**Verified on the shipped defaults**, same capture: **11 lost frames of 3525**, fitness 1.00 at the end,
42,917/160,000 blocks (27% of capacity), memory flat at 0.013 MiB/frame. Step latency p50 **7.9** ms /
p90 9.8 / p99 11.9 — *better* than the 40,000 run (8.7 / 11.6 / 14.8), so the larger pre-allocation is
free per-frame. The saturation check is one hashmap size read, measured at **5.8 µs/call** (0.02 s
across the whole scan) and early-outs permanently once fired. (An earlier run of this same
configuration showed p99 22.1 ms; that was a concurrent CPU job on the box, not the change — re-measured
clean.)

**Note it was 6.G that exposed this.** Before the CUDA-cache fix (BUG-032) this scan OOM'd around frame
900, long before capacity mattered. Removing the memory ceiling moved the wall to the next constraint.

**Bigger scans:** the grid is device-homogeneous — Open3D offers no managed/unified memory, so blocks
live wherever the grid does; you cannot hold them in host RAM while integrating on the GPU. A CUDA grid
is capped by VRAM (~460k blocks in the ~6.5 GiB usable on the 8 GiB card, ~10× this scan); a CPU grid
(`[slam] device = "CPU:0"`) is bounded only by system RAM. **Measured on this same sweep:** the CPU grid
completed it with **0 lost frames of 3525** at 46,037 blocks and zero VRAM. (It needs ~7% more blocks
than CUDA — 46,037 vs 42,917 — from float-rounding differences in the transform, not a defect.) The
per-step cost of that is the ~2.1× CPU/GPU ratio from the CUDA at-scale validation (CPU 18.94 ms vs GPU
8.85 ms median), which still fits the ~28 fps sensor ceiling; this run did not time steps separately.
Extraction already crosses the device line — `_extract_vbg()` copies to CPU because CUDA marching cubes
OOMs at scale.

---

## BUG-036 — One bad frame is terminal: a single fixed ICP correspondence radius

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 · **Area:** host/slam
- **Where:** `host/src/roomscan/slam/odometry.py` (`register_escalating`),
  `slam/mapper.py` (`icp_retry_dist`, `icp_escalations`, `lost_flags`),
  `slam/metrics.py` (`tracking_stats`), `slam/config.py` (`icp_retry_dist`)
- **Found by:** checking loop closure on the owner's two room circuits
  (`captures/coffeeRoomCircuitMnt.bin`, `captures/coffeeRoomCircuitNoMnt.bin`).

`max_dist` was one fixed 0.05 m serving two jobs it cannot both do. A frame whose frame-to-model
residual exceeds it finds **zero** correspondences; `predict_pose` then freezes translation at
`t_prev` and nothing relocalizes, so the frozen pose drifts further from truth and every later
raycast fails too. One frame kills the rest of the scan.

**Measured on `coffeeRoomCircuitMnt.bin`** (1889 depth frames, 62.4 s, 1 cm voxels, CUDA:0):

- tracking died at frame **1466** (t+48.4 s) in **one unbroken 423-frame run to the end** — 22% of
  the capture;
- ICP fitness was **0.919 in the 4 s before it died**. There is no degradation to watch for; it
  falls off a cliff. Fitness reads 0.086 for ~130 frames (ICP running, failing the gate), then
  **exactly 0.000** to the end (the raycast finds no map at all);
- depth was healthy throughout — post-gate valid count never below 1821 of 2268 — so this is not an
  empty-frame problem;
- it still **reported a plausible 2.05 m "drift"**. That number is not a measurement; it is where
  the estimate stood when it died, held for the last 22% of the run. Nothing logged.

**The fix — escalate only on failure.** Keep 0.05 as the first attempt; retry a *failed* frame once
at `icp_retry_dist` (0.10). A wider radius is not free, so it must not apply to healthy frames:

| capture | config | lost | start-end gap | path | escalations |
|---|---|---|---|---|---|
| Mnt | 0.05 fixed (old) | **423** | 2.047 m (7.48%) | 27.37 m | — |
| Mnt | 0.05 → 0.10 retry | **0** | 1.521 m (4.80%) | 31.68 m | **1** |
| NoMnt | 0.05 fixed (old) | 0 | 0.150 m (0.46%) | 32.50 m | — |
| NoMnt | 0.05 → 0.10 retry | 0 | **0.150 m** (bit-identical) | 32.50 m | **0** |
| NoMnt | 0.10 fixed | 0 | 0.953 m (2.63%) | 36.25 m | — |

**One retry, in one frame out of 1889, recovered the whole second half of the scan**, and the clean
run is bit-identical (`0.1501406473934001` / `32.50240193119565` before and after) because it never
escalates. Cost: +0.4 ms p50, nowhere near the ~35 ms budget. A third rung at 0.20 never fired, so
only one retry is implemented. `icp_retry_dist = 0` restores the old single-attempt path exactly.

Rejected with data, on the same two captures: a **fixed** 0.10 (rescues Mnt but degrades NoMnt
0.150 → 0.953 m), **6-DoF** (518 lost, first at frame 535 — much worse), and a looser `min_fitness`
(identical to the radius change alone: the gate is not what fails, correspondences are).

**Also fixed here: the silence.** `Mapper.lost_flags` now records the per-frame flag and
`metrics.tracking_stats` reports `trailing_lost` / `longest_lost_run` / `died`. A count alone hides
this failure — 423 lost of 1889 reads like "78% tracked fine" when in fact the run ended and the
tail is fabricated. `roomscan-slam` now prints `<-- THE RUN DIED` when a run ends in a sustained
lost streak, and the `--json` report carries the same under `tracking`.

**Not fixed, and not claimed:** Mnt still closes at 4.80% versus NoMnt's 0.46%. The retry makes that
run *complete*, not *accurate*. The remaining error is partly BUG-037 (barometer). Its horizontal
residual is unexplained — the mount is **not** in the FOV (no body-fixed occluder: zero pixels with
median < 700 mm, IQR < 60 mm, seen > 90% in either capture) and the motion profile does not
discriminate (NoMnt had *more* close-and-fast exposure — 4.5% vs 3.5% of frames, longest run 37
frames at 337 mm / 75 °/s vs 28 frames at 383 mm / 64 °/s — and never lost tracking). No mechanism
is asserted for it.

**Still open architecturally:** the retry makes a single bad *frame* survivable; it does not make a
bad *second* survivable. There is still no relocalization — a lost pose can never re-find the map.

---

## BUG-037 — Height is slaved to a barometer that wanders metres per minute

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 · **Area:** host/slam
- **Where:** `host/src/roomscan/slam/mapper.py` (`_apply_baro_z`), `slam/config.py` (`baro_weight`)
- **Found by:** scoring the owner's two room circuits against a ceiling-bookend ground truth
  (see "How this was measured" below).

`_apply_baro_z` blends the pose's height toward the barometric height every frame at
`baro_weight = 0.05` — a ~20-frame (0.66 s) time constant. Height is therefore effectively *slaved*
to the barometer. Over these two ~1-minute captures the sensor wandered **2.8 m and 2.6 m of
apparent altitude** (33.7 Pa and 31.2 Pa), with **43 cm and 12 cm of net drift** start-to-end.

**Measured** (ladder fix applied in both rows, so this isolates the barometer):

| capture | `baro_weight` | height error | horizontal gap | reported path |
|---|---|---|---|---|
| Mnt | 0.05 (default) | **−581 mm** | 1.406 m | 31.68 m |
| Mnt | 0 (off) | **−21 mm** | 0.941 m | 20.53 m |
| NoMnt | 0.05 (default) | −7 mm | **0.150 m** | 32.50 m |
| NoMnt | 0 (off) | −22 mm | 0.462 m | 21.73 m |

Two separate problems:

1. **Height error.** True height error is ~0 for both runs (ground truth below). The mounted run is
   off by **581 mm** with the barometer on and **21 mm** with it off — a 28× improvement. The run
   with 43 cm of barometric drift is exactly the one with the large height error.
2. **Invented path length.** Turning the barometer off cuts reported path by ~35% in *both* runs
   (31.68 → 20.53 m, 32.50 → 21.73 m). The operator walked a level circuit; roughly **10 m of
   "path" is vertical motion the barometer invented**. This inflates every `%-of-path` drift figure
   in the SLAM reports, i.e. it has been flattering our numbers.

**It is not a simple flag flip.** NoMnt's *horizontal* closure gets worse with the barometer off
(0.150 → 0.462 m), and `baro_weight = 0.01` was worse than both ends on the mounted run (−450 mm),
so the response is not monotonic. This needs a proper sweep against ground truth, not a default
change. Do not "fix" it by setting `baro_weight = 0` on this evidence.

**How this was measured — ceiling bookends.** The owner's captures start and end with the device
parked facing the ceiling (elevation 80.1°/80.2° and 89.2°/89.2°) at an identical measured range
(1420/1420 mm and 1453/1452 mm). Same elevation + same range to a flat ceiling ⇒ **same height**, so
the true height error over the loop is ~0 with no external instrumentation. That independently
corroborates the clean run (SLAM says 7 mm) and condemns the mounted one.

⚠ **This is an opportunistic check, not a protocol.** Do not build tooling that *requires* a parked
bookend: the owner's objection (2026-07-30) is that reaching the table puts the operator in the
sensor's FOV. Score it when a capture happens to have stationary bookends; never demand one.

---

### Resolution (2026-07-30)

The table above reproduces exactly on CUDA:0 (all twelve numbers), and the bookends re-verify
independently: elevation matches to 0.17°/0.04° and range to 0.1 mm/0.6 mm with the device
stationary at both ends, so the true height change over each loop is **<1 mm**. What the table does
*not* say is that most of its columns are chaos — see "the measurement was the second defect" below.

#### What the defect actually was

Not "over-trusting a drifting barometer" — that was the visible half. Decomposing the pressure trace
(NoMnt: total σ 4.32 Pa) splits it into **σ 3.12 Pa of frame-to-frame white noise** and only ~3.0 Pa
of slow wander. In apparent altitude that is:

| band | magnitude | vs ICP |
|---|---|---|
| white noise, per frame | **267 mm RMS** (380 mm frame-to-frame, max 1.9 m) | — |
| slow wander, per minute | ~0.4–0.5 m (0.43 m net on Mnt) | ICP's own vertical drift ~20–100 mm/min |

Three distinct faults followed from feeding that raw into a fixed-gain blend:

1. **Unfiltered white noise went straight into the pose.** At a 0.66 s blend the pose absorbs ~5% of
   a 267 mm-RMS signal every frame ⇒ **~12 mm of vertical step per frame**, which is precisely what
   was measured: 11.5 / 11.8 / 15.1 mm per frame across the three captures. That is the whole of the
   "invented path" — it scales with *frame count*, not with distance walked.
2. **A blend gives the barometer DC authority 1.0.** Below its corner frequency the pose *becomes*
   the barometer, in exactly the band where the barometer is ~20× the worse instrument. There was no
   notion that the two estimates have different uncertainties.
3. **The datum was one sample.** `_ref_pa` froze on the first pressure reading — a single draw from
   that 267 mm distribution, baked in as a constant altitude offset for the whole run at gain 1.

#### The measurement was the second defect

Most of the original table cannot support the weight it was given. Re-running the *identical*
configuration under numerically innocuous perturbations (CPU vs CUDA float ordering, starting one
frame later, `max_dist` 0.05 → 0.0501) moves the answers by more than the effect being measured. A
controlled test settles it: a deliberate **3 mm** one-shot height nudge, barometer otherwise off,
moves the final height error by **146 mm** and the loop closure by **0.37 m**.

So: **single-run SLAM comparisons below ~0.3 m of closure or ~0.2 m of height are realization noise.**
The specific claim that blocked the obvious fix — "NoMnt's horizontal closure gets worse with the
barometer off, 0.150 → 0.462 m" — is not real. Across a 10-member ensemble NoMnt's closure is
0.68 ± 0.36 m with the old blend and 0.61 ± 0.25 m with it off; 0.150 was simply the luckiest member.
Everything below is an ensemble mean ± sd over 10 perturbations (effectively 9 — `weight_threshold`
+1e-4 turned out to be a no-op and duplicates the base run).

#### The fix

`baro_weight` is retired. The constraint is now a **low-passed, bounded-authority complementary
correction** whose whole lifetime contribution is exactly

```
baro_correction = baro_authority · LPF_tau(h_baro − h_icp)
```

with both parameters *measured quantities* rather than a magic gain:

- **`baro_tau_frames = 900`** (~30 s at 30 fps) — the noise filter. 267 mm RMS through a 1/900 EMA
  leaves ~6 mm. Answers fault (1).
- **`baro_authority = 0.05`** — the least-squares share of two drifting estimates,
  q_icp²/(q_icp² + q_baro²) ≈ 0.09²/(0.09² + 0.45²) ≈ 0.04, rounded up. Answers fault (2). *That this
  equals the old `baro_weight`'s numeral is a coincidence of arithmetic, not of meaning* — the old
  gain's DC authority was 1.0.
- The datum is the mean of the first 90 frames (`_BARO_REF_FRAMES`), and the filter opens at 0 rather
  than at the first innovation. Answers fault (3).
- New `Mapper.baro_correction_m` reports how much height came from the barometer; `roomscan-slam`
  prints it and puts it in `--json`, and `slam_rerender` surfaces it.

Plumbed the same way `block_count` was: `Mapper` / `[slam]` / CLI (`--baro-authority`, added so the
default can be re-measured) / `slam/service.py` / `web.SlamRunner` / `slam_gpu_memory.py`. A config
still carrying `baro_weight` loads fine — unknown keys are ignored — and is *not* reinterpreted.

#### Before / after (ensemble means ± sd, n = 10)

`roomSweepFull20260730.bin` is the independent check: nothing was tuned on it, and it has **no
bookend** (it ends mid-sweep, elevation 26.8° → 8.7°, still moving) so its height column is a
*reported height change*, not an error, and its closure is not a loop closure. It is in the table for
the path column only.

| capture | config | height error | horizontal closure | reported path | of which vertical |
|---|---|---|---|---|---|
| Mnt | old `baro_weight = 0.05` | 458 ± 232 mm | 1.286 ± 0.249 m | 31.52 m | 21.72 m (69%) |
| Mnt | **new default** | **102 ± 113 mm** | **0.912 ± 0.310 m** | **20.88 m** | **6.75 m (32%)** |
| Mnt | authority 0 (off) | 67 ± 57 mm | 0.912 ± 0.192 m | 20.77 m | 6.70 m |
| NoMnt | old `baro_weight = 0.05` | 125 ± 138 mm | 0.683 ± 0.364 m | 33.74 m | 23.27 m (69%) |
| NoMnt | **new default** | **125 ± 76 mm** | **0.738 ± 0.187 m** | **23.89 m** | **7.41 m (31%)** |
| NoMnt | authority 0 (off) | 204 ± 280 mm | 0.608 ± 0.252 m | 22.23 m | 7.29 m |
| Sweep † | old `baro_weight = 0.05` | (418 ± 202 mm) | (0.812 m) | 72.20 m | 53.25 m (74%) |
| Sweep † | **new default** | (963 ± 503 mm) | (2.550 m) | **45.15 m** | **19.31 m (43%)** |
| Sweep † | authority 0 (off) | (520 ± 170 mm) | (2.918 m) | 44.40 m | 19.38 m |

† not a closed loop and no bookend — parenthesised columns have no ground truth.

**What improved, robustly:** the invented path is gone. Reported path drops **34% / 29% / 37%** and
the vertical share drops from ~70% to ~32%, on all three captures including the one it was not tuned
on. The whole-run barometric correction is now **9 / 10 / 27 mm** instead of hundreds.

**Height error:** the old blend's 458 mm on the mounted run (the one whose barometer drifted 43 cm)
is gone — 102 mm, a 4.5× improvement, and the *spread* halves. Note the original entry's headline
"−581 mm vs −21 mm, 28×" overstated it: those were single members of ensembles whose means are
458 mm and 67 mm.

**Honest negative:** on this evidence the fixed constraint is **not distinguishable from switching
the barometer off**. It is better on NoMnt (125 vs 204 mm) and worse on Mnt (102 vs 67 mm), both
inside the chaos band, and its total contribution is ~10 mm — arithmetically far too small to explain
either difference. With *this* barometer, on a 1-minute scan, the correct authority is small enough
to be worth nothing. It is kept in this shape because the parameters are now measurable: a quieter
barometer (the firmware writes LPS22DF `CTRL_REG1 = 0x20`, which by that register map is minimum
averaging — untested hypothesis for the 3.1 Pa, which is measured) or a scan long enough for ICP's
drift to overtake the barometer's would make it earn its place by moving a number, not an argument.

**Consequence for every `%-of-path` figure.** They were all divided by an inflated denominator. The
headline "frame-to-model closes a room to **0.46 %**" was a lucky single run (0.150 m) over an
inflated path (32.50 m). Honestly: **0.74 ± 0.19 m over 23.9 m ≈ 3 %** on NoMnt and
**0.91 ± 0.31 m over 20.9 m ≈ 4.4 %** on Mnt. Still decent absolute closure for a 54×42 imager with
no loop closure, but ~5× the number 6.D's "does loop closure earn its complexity" argument was
resting on. `ROADMAP.md` and `CLAUDE.md` are corrected.

**Not fixed / still open:** the barometer's slow wander itself (environmental + sensor, not a
software defect); the LPS22DF averaging configuration in `rs_lsm.c`, which is the only lever that
would make the constraint useful — deliberately *not* touched here, the rig was live and streaming;
and ICP's own vertical jitter, which is what the remaining ~19 m of vertical path on the sweep is.

---

## BUG-039 — Host IMU fusion measures heading error about the wrong axis (near gimbal lock)

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 (found while costing ODR-triggered
  sync, BUG-031) · **Area:** host/sensors
- **Where:** `host/src/roomscan/imufusion.py` (`_correct_yaw`),
  `host/src/roomscan/sensors.py` (`graft_yaw_error_deg`, new). The filter remains **gated off** —
  see the warning at the end of this entry.
- **Found by:** `docs/odr-triggered-sync-costing-2026-07-30.md` §1

`_correct_yaw` measured its heading error with `quat_yaw_deg` — ZYX yaw, i.e. rotation about **body
Z** — but this device's SFLP body frame has **X = Up**. On the stationary capture the ZYX pitch is
86.2°, which is **3.76° from gimbal lock**, so the quantity the loop was nulling is not heading.

Measured heading error vs SFLP: **1.689° mean / 2.217° p95**. Shortening `tau_yaw` to 0.3 s barely
moved it (1.703°) — the signature of a *wrong measurement*, not a mistuned gain.

**The fix.** New `sensors.graft_yaw_error_deg(target, quat)` — the world-Z **swing-twist** of the
residual `target ⊗ quat*`, i.e. `2·atan2(rel_z, rel_w)`. That is the exact inverse of `graft_yaw`
(the loop's own actuator), so the loop now nulls precisely the quantity it can correct, and it is the
*optimal* pure-heading correction rather than merely a different one. It has no singularity at any
attitude, so unlike `YawFusion` — which defends against the same ZYX degeneracy with a
`gimbal_margin_deg = 15°` gate that would have gated this whole capture out — it needs no gate.
`_correct_yaw` is a one-line change on top.

**Measured, before → after, world-Z heading error vs SFLP, same bytes both ways** (`tau_yaw` 1.0 s;
the whole ensemble, not one capture — single-run metrics in this repo have proven chaotic):

| capture | ZYX pitch p95 | before mean/p95 | after mean/p95 |
|---|---|---|---|
| `stationary_stream11_20260728_190311` (3000 fr) | 86.2° | 1.689 / 2.218 | **0.017 / 0.053** |
| `stream11_verify` (304 fr) | 80.8° | 0.014 / 0.037 | **0.006 / 0.016** |
| `coffeeRoomCircuitNoMnt` (1988 fr) | 85.0° | 0.994 / 2.826 | **0.573 / 1.550** |
| `coffeeRoomCircuitMnt` (1928 fr) | 74.6° | 0.775 / 2.120 | **0.549 / 1.538** |
| `roomSweepFull20260730` (3000 fr) | 77.3° | 0.573 / 1.430 | **0.509 / 1.266** |
| `tilt_sweep_20260729` (3000 fr) | −0.1° | 0.011 / 0.026 | 0.011 / 0.026 |
| `web_20260730_181252` (2780 fr) | −0.2° | 0.003 / 0.015 | 0.003 / 0.015 |

The two **zero-pitch captures are bit-identical** before and after. That is the control, and it is
the point: the misreading scales as `tilt × tan(pitch)`, so it is 0.04° at level and ~7° at 86°.
This is a frame error at the attitudes this device actually flies, not a general retune.

**Tests that would have caught it** (`test_imu_fusion.py::test_yaw_loop_measures_heading_about_
world_z_not_body_z`, plus four in `test_sensors.py`). Verified by re-running the suite with the old
term monkeypatched back in globally: the axis test fails at 0.138° against a `< 0.01°` bound, and
nothing else in 1079 tests notices. The fixture holds the device still 4° from gimbal lock and lets
the reference's *static* fp16 tilt quantisation be the only error; the expected legacy failure is
**derived from the fixture** (it converges to exactly the ZYX-yaw misreading of that quantisation),
not hard-coded. A companion test records the trap that let this ship: on a residual that is *pure
heading*, both conventions agree exactly even at gimbal lock — so the obvious test, a clean 30°
heading offset, passes either way and proves nothing about the axis.

**Why it matters beyond the filter itself.** `imufusion` beating SFLP is the stated precondition for
trading SFLP away for hardware ODR-triggered sync (BUG-031). While this defect stands, that
precondition is untested, not merely unmet — the comparison that would decide it is measuring the
wrong thing.

⚠ **Do not treat "fixed this ⇒ imufusion beats SFLP" as established.** The only shipped evidence for
the filter is synthetic (`test_imu_fusion.py` is all-synthetic by its own docstring), the within-capture
win is measured at **one attitude, stationary**, and under motion the accel-referenced metric
**saturates** (SFLP 0.879°, imufusion 0.864°, SFLP-gravity 0.868° — all within 2%). **No capture in
the repo can adjudicate the two under motion**, and there is no orientation ground truth anywhere in
the repo. Fixing the axis makes the comparison *meaningful*; it does not make it *decided*.

**The gate is therefore deliberately unchanged** (2026-07-30): `SensorState(imu_fusion=None)` is
still the default and nothing in the repo constructs an `ImuFusion` outside its own module and tests
— verified by grep, and by the unchanged `test_slam_non_regression_*` guards. Note also what the
numbers above are and are not: they are *agreement with SFLP*, not accuracy. The table says the
filter's heading now follows its own anchor instead of fighting it; it says nothing about whether
either estimator points the right way. **Heading direction remains unvalidated repo-wide** (resume
doc §4.6) and nothing here touches it.

---

## BUG-040 — Web UI cannot see transport loss (Drops/Gaps always 0)

- **Status:** **fixed** 2026-07-31 · **Reported:** 2026-07-31 (owner, while diagnosing a SLAM run) · **Area:** host/web
- **Where:** `host/src/roomscan/web.py` `_broadcaster` / `build_metrics_message`

`MetricsRegistry` measures rates and bandwidth; it has no notion of frame *sequencing*, so
`MetricsSnapshot.drops`/`.gaps` keep their dataclass default of `0`. The desktop panel filled them
in (`panel.py:2978`, `replace(snap, drops=..., gaps=...)` off the reader's `Stats`), but the web
broadcaster never did — it passed the raw snapshot straight to `build_metrics_message`. The reader
*was* maintaining the counters correctly the whole time (`reader.py:121` calls `stats.update`);
only the hand-off was missing.

Why it matters more than a cosmetic zero: the Ethernet transport splits a 14,878-byte depth frame
into 11 UDP datagrams, and the host's reassembly (`sources.py`) requires every fragment of a `seq`
**in order** — one lost or reordered datagram silently discards the whole frame. The discarded
frame never reaches the decoder, so the loss surfaces *only* as a header sequence gap. With that
row pinned at 0, `roomscan-web` — the primary, supported UI — had no way to show packet loss.

**Fix:** merge the reader's `Stats` into the snapshot in the broadcaster, guarded with `getattr`
so a partially-built `app.state` (tests, replay harnesses) still broadcasts.

**Test:** `test_metrics_broadcast_reports_reader_drops_and_gaps` drives a real uvicorn server over
a websocket and asserts the counters arrive; removing the `replace(...)` fails it with `0 != 7`.

---

## BUG-041 — UDP fragments burst into an 8-deep TX ring, and a failed send abandoned the frame

- **Status:** **fixed** 2026-07-31 · **Reported:** 2026-07-31 (owner: "is UDP the right transport?") · **Area:** firmware/eth
- **Where:** `firmware/scanner-stream/Src/ethernet_transport.c` `ETH_SendFrame_Gather`

Two defects in one loop:

1. **Burst.** A depth frame is `RS_HEADER_SIZE + 14842 + CRC` = 14,878 B → 11 datagrams, pushed
   back-to-back with no pacing. `ETH_TX_DESC_CNT` is **8** (`stm32h5xx_hal_conf.h:186`), and at
   250 MHz the CPU enqueues a 1400 B memcpy far faster than the DMA drains a descriptor (~112 µs
   each at 100 Mbit), so the ring can be overrun by the frame's own fragments. Through a Wi-Fi
   bridge the same burst shape is what overflows a cheap AP's queue.
2. **Mid-frame abandon.** On `pbuf_alloc` returning NULL or `udp_sendto` returning non-`ERR_OK`,
   the old code `return false`d immediately — leaving fragments 0..k-1 already on the wire. Those
   are guaranteed waste: the host needs every fragment of a `seq`, so a partial frame is a lost
   frame *plus* wasted bandwidth.

Loss here is amplified ~11× (one datagram kills a 14.8 KB frame) and is not benign for SLAM: a
dropped depth frame doubles the inter-frame motion ICP has to solve, which is the same failure
mode as BUG-036 (a 52 mm step against a 0.05 m correspondence radius killed 54% of a run).

**Fix:** frames are copied into a fixed slot FIFO (8 × 15,104 B) and their fragments metered out
from `ETH_Process()`. Two invariants: frames drain **strictly in order, one at a time** (the host
resets its reassembly buffer the instant a different `seq` arrives, so interleaving would destroy
both frames), and a refused send **retries** rather than abandoning. Back-pressure on a full FIFO
drains synchronously rather than dropping, because the caller's raw double-buffer is about to be
reused. The per-call fragment budget is adaptive — sized from the outstanding count so the queue
clears inside `ETH_TX_WINDOW_MS` (25 ms) regardless of how coarsely `ETH_Process` is called
(`platform_wait_for_event` busy-waits in 5 ms slices, so a fixed inter-fragment gap would
under-drain and the backlog would grow without bound).

Cost: `bss` 54,256 → 175,200 B (184 KB static of 640 KB; 456 KB still free), `text` +424 B.

**Verified on-rig (2026-07-31):** flashed and soaked 90 s — 15,909 frames, **0 CRC failures, 0
bytes skipped, no anomalies**, device rate 30.303 Hz (identical to the pre-flash baseline),
0 drops / 0 gaps.

**Not yet demonstrated:** the loss *reduction*. The rig was on a direct cable for this test (the
RavPower FileHub was powered on but not bridged), so there was no loss to remove — the soak proves
no regression, not benefit. Re-measure with the Wi-Fi bridge in path, now that BUG-040 makes the
Gaps row readable.

---

## BUG-042 — A reordered UDP datagram destroyed a whole frame, silently

- **Status:** **fixed** 2026-07-31 · **Reported:** 2026-07-31 (found while costing transport compression) · **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` `UdpSource.read`

A depth frame is 11 UDP datagrams. Reassembly accepted a fragment only when
`frag_idx == self._expected_frag` and then *appended* it, so position was implied by arrival order.
UDP guarantees neither ordering nor delivery — a reordered pair therefore discarded the entire frame
just as surely as a lost datagram would, and nothing counted it. The frame never reached the decoder,
so the only downstream trace was a header sequence gap, which BUG-040 had left unwired in the web UI.
Net effect: the two most likely transport faults were both invisible, and one of them was
self-inflicted.

**Fix:** reassemble into indexed slots — fragment *k* always lands at index *k*, and the frame
completes when the last hole fills, in any arrival order. Joining slots `0..n-1` reconstructs the
frame exactly because the sender chunks at a fixed 1400 B with only the tail short, so the index
alone determines position. Added `frames_incomplete` / `frags_lost` (real loss),
`frags_reordered` (recovered, previously fatal), `frags_duplicate` and `frags_invalid`, which
separate causes that used to be indistinguishable. A fragment whose `total_frags` disagrees with the
rest of its seq is rejected rather than resizing the frame mid-flight.

Safe because the firmware's paced TX (BUG-041) drains frames strictly in order, one at a time, so
fragments of two seqs never interleave — a single in-flight frame buffer is sufficient.

**Tests:** `test_reassembly_*` in `host/tests/test_sources.py`. Reintroducing the strict-order check
fails `accepts_out_of_order_fragments` and `counts_a_genuinely_lost_fragment`.

**Related, same commit — CRC32 made table-driven.** `rs_crc32` (`firmware/scanner-stream/Src/rs_protocol.c`)
was table-free bit-serial: 8 shift/xor iterations per byte over header+payload for every frame
(~119,000 inner iterations at 14,874 B). Replaced with a 16-entry nibble table (64 B). Measured
**2.21× faster** on x86 -O2 (125.2 → 56.7 µs/frame) — note this is *below* the ~5× an earlier
estimate assumed, so the on-target saving is likely ~1.2–1.8 ms/frame, not 2.2–3.2. Bit-exactness
verified natively over 3,857 cases (all single bytes, all lengths 0–600, 3,000 full-frame trials
using the firmware's incremental `rs_crc32(rs_crc32(0,hdr,32), payload, …)` seeding, plus the
standard `CRC32("123456789") == 0xCBF43926` vector), then confirmed on-target: 13,434 frames
streamed with **0 CRC failures** — a single divergent bit would have failed every frame.

The loop is sensor-rate-limited (30.3 Hz) and spends ~20 ms of each 33 ms period in
`platform_wait_for_event`'s spin, so this buys **headroom, not throughput** — there is no fps change
to observe. It is worth having because it is the enabling margin for on-MCU compression, where
compressing before the CRC also shrinks what the CRC must cover.

## BUG-043 — Live SLAM lost its Save, so an unrecorded live scan became unrecoverable

- **Status:** **fixed** 2026-07-31 · **Reported:** 2026-07-31 (found reviewing the Live/View consolidation) · **Area:** host/web
- **Where:** `host/src/roomscan/web.py` `_handle_inbound` `"save"`, `static/slam.js` `updateSaveEnabled`, `mcp_server/tools_rig.py` `rig_save`

The Live/View consolidation made Detailed SLAM the owner of persistent reconstructions and, in doing
so, deleted `save` outright: the handler became a single bus line, `updateSaveEnabled` hard-coded
`b.disabled = true`, the click listener was replaced with `() => {}`, and `rig_save` returned a flat
error. The intended rule was only that **replay** SLAM is preview-only — Detailed is the
capture-keyed sidecar there — but the rule was applied to Live as well.

That case is exactly the one that cannot be redone. A live SLAM map is built from frames that are
never stored unless Record happened to be running, so dropping the map discards the only copy;
"re-run it as Detailed" has nothing to re-run. Nothing failed loudly either — the button simply sat
greyed with its original "Export the reconstructed mesh as a PLY file" tooltip.

**Fix:** `save` is gated on `source == "live" and display == "slam"`; replay SLAM refuses with a
reason naming Detailed, and Detailed writes its own artifacts. The button re-enables only for Live
SLAM with a non-empty map and retitles itself in the other two cases, so a disabled button explains
which one it is. `rig_save` works again and reports the offending `source`/`display` on timeout.
Pinned by `test_save_is_live_slam_only`, verified by reintroducing the defect.

## BUG-044 — Any unrelated control re-advertised SLAM on a capture that cannot do SLAM

- **Status:** **fixed** 2026-07-31 · **Reported:** 2026-07-31 (same review) · **Area:** host/web
- **Where:** `host/src/roomscan/web.py` `_state_message` / `_broadcast_state`

`_state_message(ui, controller, detailed)` derives two fields from the loaded capture:
`slam_available` (false when a legacy capture carries no stream 9, so there is no rotation prior) and
`detailed` (the sidecar present/stale status). Both default to the permissive value when the extra
arguments are omitted — and eight echo sites still called the bare `_state_message(ui)`.

Because the frontend drives *all* disabled state from this echo and never optimistically (the
one-way-flow invariant), changing the colour, point size, IR colormap, orientation mode or yaw offset
re-enabled the SLAM and Detailed segments on a capture that cannot run SLAM, and cleared the stale
badge. The next click then bounced off a server-side refusal, so the UI contradicted itself.

**Fix:** all eight sites go through `_broadcast_state(state)`, which always supplies the controller
and Detailed runner; its docstring says why the context is not optional. Pinned by
`test_state_echo_keeps_capability_context_on_an_unrelated_control`, verified by reintroducing the
defect on the `set_color` path.

## BUG-045 — The capture library scanned 1 GB of headers on the event loop

- **Status:** **fixed** 2026-07-31 · **Reported:** 2026-07-31 (same review) · **Area:** host/web
- **Where:** `host/src/roomscan/web.py` `list_captures` / `_broadcast_captures`

`list_captures` grew a per-capture header walk (frame count, real duration, stream-9 capability) to
feed the View library. It is cached on `(path, size, mtime)`, but every call site ran it **on the
event loop**: the websocket connect handler and the three `captures` broadcasts.

Measured cold over the current library — 25 files, 1.06 GB — at **501 ms** (0.4 ms warm). That is
half a second of stalled broadcaster, dropping ~15 frames for every already-connected tab whenever a
new tab connects. The worst case is the one that matters most: a just-stopped recording is
guaranteed cache-cold, and it is the largest file in the directory.

**Fix:** a `_broadcast_captures(state, ctrl)` helper dispatches the scan via `asyncio.to_thread`, as
the "off-loop for blocking work" invariant in `docs/web-protocol.md` already required; the
connect-time `captures` and `saved` sends do the same.

## BUG-046 — Mag-cal coverage counts down on Fit: 92/92 complete becomes 91/92

- **Status:** **open** · **Reported:** 2026-07-31 (owner, using the Calibrate Mag modal) · **Area:** host/web
- **Where:** `host/src/roomscan/magsweep.py` `MagSweepSession.binning_calibration` / `sync_occupied` /
  `build_report`; surfaced by `static/magcal.js` + `magcal3d.js`

**Symptom (owner).** Tumble until the modal reads **92/92 cells complete**, press **Fit** (the
`Stop & Fit` button, `magcal` action `stop`), and the coverage immediately reads **91/92** — the
sweep un-completes itself at exactly the moment it was finished. The shell view correspondingly
un-fills a cell.

**Mechanism.** Coverage is not measured on the raw samples; it is measured on their *calibrated*
directions — `build_report` does `dirs = calibrated_directions(x, session.binning_calibration(current))`
→ `assign_cells(dirs)` → `coverage_stats`. And `binning_calibration` prefers **the candidate if one
exists**, else the saved calibration, else a provisional hard-iron estimate. Pressing Fit creates the
candidate, so the frame the cells are binned in changes *underneath the tally*: every sample's
direction moves, some samples cross a cell boundary, and any cell whose occupancy rested on those
samples empties. `sync_occupied(cov["counts"])` then overwrites the incrementally-marked
`session.occupied` set with the recomputed truth (its docstring already anticipates this — "e.g.
after the binning calibration changed and every direction moved"), so the UI number drops.

**Reproduced numerically** (not yet on the rig): synthetic 600-sample tumbles binned first with a
saved calibration that is stale by a few µT — the situation that makes you recalibrate at all — then
re-binned with the `fit_ellipsoid` candidate. Mean direction shift on refit **3.35°** against ~24°-wide
cells; over 30 seeds, **3 runs went 92/92 → 91/92**, and others moved 90→92 or 91→92 (100–125 of 600
samples changed cell each time). With an already-good saved calibration the shift is ~0.01° and
occupancy never moves, which matches this only being visible after a real re-fit.

So the count is *arguably* correct in both frames — it is the comparison across a frame change that
is meaningless, and the UI presents it as a monotonic progress bar ("complete"). Two consequences
beyond the confusion: chasing the missing cell re-fits again on the next Stop, which can move the
boundary again (the last cell is not reliably reachable), and a user who reads 91/92 as "keep
tumbling" may reject a candidate that was in fact fitted from a fully-covered cloud.

**Not yet decided (fix options).**
1. Bin coverage in a **frozen** frame for the life of the sweep — e.g. always the calibration in
   force when `start()` was pressed — so the progress number never changes meaning mid-sweep, and
   only `reset()` re-frames it. Quality metrics keep using the candidate as they do now.
2. Keep the re-binning but make the regression legible rather than silent: report both counts
   (`92/92 under the saved calibration, 91/92 under the candidate`) and drop the word "complete".
3. Treat coverage as a high-water mark once reached (rejected on sight — it would hide a genuine gap
   that only the corrected frame can see, which is the whole reason the coverage map exists).

Option 1 looks right because the cell map is a *collection-guidance* instrument, not a quality
metric; but it needs an on-rig pass, since which cell empties depends on the real cloud's boundary
samples. Whatever lands should be pinned by a test that re-bins a fixed cloud under two calibrations
and asserts the reported progress does not go backwards — and proved by reintroducing the swap.

## BUG-047 — One id, two buttons: "Restart Server" also fired a playback restart

**Status:** fixed 2026-07-31 · **Area:** host/web · **Found by:** a structural check added while
auditing tooltips, not by either feature misbehaving.

`host/src/roomscan/static/index.html` used `id="btn-restart"` twice: the top bar's **Restart Server**
(an `/api/restart` POST, owned by `admin.js`) and the Capture card's playback **Restart** (a
`transport/restart` message, owned by `capture.js`). `document.getElementById` returns the **first**
match in document order, and the top bar is declared first.

Two consequences, neither of which points at the real cause:

1. The playback **Restart** button was inert. `capture.js` bound its handler to an element on the
   other side of the screen, so clicking Restart during replay did nothing at all.
2. **Restart Server** carried *both* handlers. Pressing it sent `{"type":"transport","action":"restart"}`
   over `/ws` and then restarted the server process. In live mode the transport message is a no-op so
   the damage was invisible; during a replay it seeked to frame 0 immediately before the process died,
   which reads as "the restart lost my playback position".

**Fix.** The playback button is now `id="btn-transport-restart"` (`index.html`, `capture.js`).

**Regression test.** `host/tests/test_static_ui.py::test_no_duplicate_element_ids` — a whole-file
check that no `id=` appears twice, rather than a test of either button. Neither symptom above would
have led anyone to write a test for *this*, and the next duplicate id will not resemble this one
either. Proved by reintroducing the duplicate and confirming the test fails.

**Lesson.** Two modules can each look up "their" element by id and one of them silently gets the other
module's button. The failure surfaces as a cross-feature side effect — a maintenance action firing an
unrelated playback command — so it is worth an invariant over the whole document rather than a test
per control.

---

## BUG-048 — Heading disintegrates near ZYX gimbal lock, and it faked a calibration fault

**Status:** fixed 2026-07-31 by **BUG-051**, which is the same defect met from the live rig instead of
from a capture — see there for the shipped fix, and for the correction to this entry's "Scope"
paragraph below (it understated the impact: the live path was the most affected, not the least).
· **Area:** host/sensors · **Found:** 2026-07-31, scoring DC-E (`captures/DebugCapE.bin`)

`roomscan.sensors.absolute_heading` (sensors.py:452) strips yaw with
`graft_yaw(quat, -quat_yaw_deg(quat))`, and `quat_yaw_deg` is a **ZYX Tait-Bryan yaw**, which
gimbal-locks as `|quat_pitch_deg|` → 90°. Near lock, yaw and roll are degenerate — only their sum is
determined — so both blow up.

**Measured on a capture that is braced and stationary for most of its length**, frame-to-frame yaw
change against proximity to lock:

| \|ZYX pitch\| | frames | median \|Δyaw\| | p95 | max |
|---|---|---|---|---|
| 0–60° | 3187 | 0.000° | 0.36° | 2.70° |
| 60–80° | 100 | 0.382° | 1.78° | 2.98° |
| 80–85° | 74 | 0.840° | 3.31° | 9.39° |
| 85–88° | 35 | 2.458° | **44.41°** | **175.31°** |
| 88–89.5° | 611 | 0.756° | 6.84° | **179.60°** |
| 89.5–90.1° | 452 | 2.362° | 8.35° | **150.65°** |

A stationary device reporting 180° yaw flips is the decomposition failing, not the sensor.
**1010 of 4459 frames (22.7%)** of this capture sit within 1° of lock.

**What it cost.** The first DC-E analysis concluded the magnetometer calibration had a direction error
of 44–59° and that ST DT0103's accelerometer-assisted fit was required. Two of the seven holds
(tilt ≈ 90°, measured ZYX pitch **89.6°** and **89.1°**) contributed 20–30° of that spread and a
within-hold circular std of **20.1°** on a *braced stationary* hold. Re-deriving heading with a
singularity-free estimator — twist about world Z, `2·atan2(z, w)`, no Euler decomposition, whose own
degenerate point is nowhere near this data (`min(w²+z²) = 0.42`) — collapses those two holds to
**156.8° and 156.0° with std 0.7° and 1.0°**, in agreement with the mid-tilt holds. That part of the
finding was an artifact of this bug.

**What survives, and is a separate question.** Holds 1 and 5 (tilt ≈ 4°, non-singular, near-identical
attitude) still disagree by **44.3°** after the fix, and the detrend-free `mag_check` tilt ramp is
**1.72×** (GOOD < 1.10, MARGINAL < 1.25) — computed from |B| against tilt with no yaw math at all, so
it is immune to this bug. There is a real magnetometer direction defect; its size is simply not yet
known, because the tool used to size it was broken.

**Scope — this is NOT a significant live-operation defect.** `absolute_heading` does feed live paths
(`YawFusion.update` at sensors.py:615, and the UI compass at web.py:1226, whose anomaly gate is on |B|
magnitude and cannot reject a geometrically-garbage heading). But normal scans do not visit the
singular attitude: fraction of frames within 1° of lock is **0.0–0.2%** across `DebugCapA`,
`DebugCapB1`, `DebugCapC`, `DebugCapF`, `DebugCapMirror`, `coffeeRoomCircuitNoMnt` and
`roomSweepFull20260730` (median |pitch| 51–78°). Only DC-E, which deliberately sweeps tilt to both
extremes, is heavily exposed. The urgent consequence is to **analysis**, not to live scanning.

**Fix direction.** Replace the Euler yaw strip in `absolute_heading` with the swing-twist term about
world Z. Note this is the **same defect class as BUG-039**, which replaced a body-Z yaw term with a
world-Z swing-twist in `imufusion._correct_yaw` — a second instance in a different code path, so a
sweep for other `quat_yaw_deg` consumers is warranted rather than a point fix.

**Regression test to write.** A braced synthetic sweep through `|pitch| = 90°` must show bounded
frame-to-frame heading change. Prove it by reintroducing the ZYX strip.

---

## BUG-049 — Multi-second whole-group frame loss on the multi-room captures

**Status:** open · **Area:** host/transport · **Found:** 2026-07-31, scoring DC-B

The three DC-B takes are **byte-perfect** — 0 CRC failures, 0 bytes skipped, `clean: true` — while
missing large numbers of frames the device demonstrably sent:

| take | duration | RAW loss | whole-group | fragment-only | worst outage |
|---|---|---|---|---|---|
| `DebugCapB1` | 163 s | **2.29%** (113/4933) | 72 | 46 | 63 frames = 2.1 s @ 37.3 s |
| `DebugCapB2` | 165 s | **4.28%** (214/5004) | 139 | 86 | 73 = 2.4 s @ 5.8 s; 63 @ 136.3 s |
| `DebugCapB3` | 181 s | **9.35%** (512/5474) | 456 | 69 | **215 = 7.1 s @ 66.2 s**; 153 = 5.1 s @ 162.6 s |
| *`coffeeRoomCircuitNoMnt`* | 66 s | 0.15% | **0** | 4 | 1 frame |

**The device emitted them.** Sequence numbers advance continuously at a steady **30.29 fps** (seq span
÷ elapsed `t_us`) straight across the gaps, so these were lost between board and file. There are **zero
EVENT frames** in any take — the firmware never noticed.

**Two distinct mechanisms**, separated by `capture_analyze`'s new continuity census:
1. **Whole-group outages** — the seq is absent from streams 9/10/11/13 *and* 7, i.e. the 20-byte
   IMU_QUAT datagram vanished alongside the ~15 KB RAW one. Not bandwidth, not fragmentation.
   CALIB spacing degrades to 64/128/256 where it should be uniformly 64.
2. **RAW-only fragment loss** — 46/86/69 frames where only the ~11-fragment RAW datagram was lost.
   This is the BUG-041/BUG-042 neighbourhood and is present at low rate on *every* capture.

**Cost to SLAM, measured.** `DebugCapB2`'s 628-frame (21.2 s) tracking collapse begins at **t = 8.26 s**;
its transport outage ran **5.81 → 8.19 s**. Tracking died within ~70 ms of the hole ending and stayed
dead 21.2 s. This is the open "no relocalization" hole, quantified: SLAM is frame-to-model, so after a
2.4 s gap the pose prior is stale and ICP cannot re-register. **A bad 2.4 seconds cost 21 seconds.**
Timing matters more than length — `DebugCapB1`'s comparable 63-frame outage at t = 37.3 s cost it
**zero** lost frames, because by then there was a mature model to re-register against.

**What it is NOT.** Not RF range: the Wi-Fi bridge is permanently mounted on the scanner with a patch
cable, so its link endpoints never separate; the owner confirms it never roamed and signal stayed
above 80% (owner, 2026-07-31). Not block-grid saturation: re-running B2 at `block_count=500000`
gives byte-identical results with `blocks_used` unchanged at 154469, so 160000 was never limiting
(knob confirmed applied via the report's `block_count` field).

**What discriminates the remaining candidates.** The loss correlates with **walking**, not duration —
`DebugCapE` is 147 s, nearly as long as B1, recorded the same session, and has 3 whole-group losses.
A **63-frame (2.08 s) outage recurs in all three takes**, which is a quantum, not the continuous
distribution RF fading produces. Leading candidates: (a) Ethernet patch-cable link bounce — the one
hop that physically flexes, and link-down plus auto-negotiation is ~1–3 s, matching the quantum;
(b) background roaming scans on the bridge, which take the radio off-channel to *evaluate* candidates
without ever losing signal or roaming; (c) a stall in the firmware's slot-FIFO TX pacer (BUG-041),
which would block all streams together.

**Next step (owner action).** Record ~3 minutes stationary, flexing the patch cable at the connectors
partway through. If the outages follow the cable it is (a); if a still, untouched 3-minute capture
still shows them it is (b) or (c). Do it with the Drops/Gaps HUD visible — BUG-040 fixed those rows,
and BUG-042's counters separate reorder from loss, but none of that is stored in the capture file, so
it has to be read live.

**Instrument gap this exposed.** `capture_analyze` called all three takes `clean: true`, because it
only ever checked CRC and resync. It now reports a `continuity` block; `clean` and
`continuity.complete` are deliberately separate properties.

## BUG-050 — Recording elapsed time was the Unix epoch

**Status:** fixed 2026-07-31 · **Area:** host/web · **Found by:** an agent adding the Live-SLAM
auto-record feature, which read `session.recording.elapsed_s` back to check its own work.

`SessionController.session_message(position, now)` computed
`rec_elapsed = now - self._record_started`. All three call sites pass `time.time()`; but
`start_record` stamps `self._record_started = time.monotonic()`. The two clocks share no origin, so
the subtraction returns roughly the Unix epoch: a live 90-second take reported
`elapsed_s: 1784067285.5`.

**Why it survived.** `test_build_session_message_shape` and
`test_controller_session_message_live_vs_replay` both assert the *shape* — the key exists and is a
float — and a wrong-by-a-billion float is still a float. Nothing asserted the magnitude, and nothing
compared it against a known elapsed interval. The web UI rendered it through `fmtTime()`, which
divides into minutes and seconds without complaint, so it displayed as an absurd but well-formatted
duration in a status line nobody was reading closely.

**Fix.** `session_message` now reads `time.monotonic()` itself for the recording clock. The `now`
parameter is kept for the callers' convenience but is deliberately unused for this, with a comment
saying why — passing a wall-clock `now` is the trap, and the next caller will pass one too.

**Regression test.** `test_recording_elapsed_is_measured_on_the_clock_that_started_it` starts a real
recording, advances `time.monotonic` by a known 90 s via monkeypatch while leaving the wall clock
alone, and asserts the reported elapsed is ~90. Proved by reintroducing the subtraction and
confirming the failure.

**Lesson.** A shape test cannot see a units or origin error. Where two clocks exist in one file,
assert an interval against a *known* elapsed time, not the presence of a number.

---

## BUG-051 — The yaw-fusion gimbal gate fired permanently in the normal grip, on the wrong axis

**Status:** fixed 2026-07-31 · **Area:** host/sensors · **Found by:** the owner, holding the
instrument in its ordinary upright grip and reading *"Aimed within 15° of straight up/down, where yaw
is undefined — tilt back toward horizontal"* while the boresight was 2.1° off horizontal. Their
objection was exactly right: "yaw is just heading, why should it be undefined? If anything our roll is
close to being undefined."

One root cause, three symptoms: **yaw math computed in the ZYX Tait-Bryan frame on a device whose body
X is Up.** ZYX's gimbal lock sits where body X goes vertical, so *"held upright"* **is** ZYX gimbal
lock, structurally.

**1 — the gate could never clear.** `YawFusion.update` froze the correction whenever
`|quat_pitch_deg| > 90 - gimbal_margin_deg` (15°). Measured live at the owner's grip: body X at
**−87.3°** elevation, **2.7°** from lock against a 15° threshold. The message was also wrong twice
over — it fired on body X while its text described the **boresight**, which was 2.1° from horizontal,
and World's own decomposition reported `singularity_margin_deg` **87.88**, `near_singularity: false`,
at the same instant. The wording had been written against World's semantics; the gate was computed in
ZYX.

**2 — the heading itself was ill-conditioned, and biased.** `absolute_heading` stripped yaw with
`graft_yaw(quat, -quat_yaw_deg(quat))` — identically `tilt_compensated_heading(quat) − ZYX_yaw`, a
well-conditioned quantity minus an ill-conditioned one (verified: both give 284.6548° on the live
quat). Against a synthetic known bearing at the operating pose the shipped formula reports **26.57°
for a true 45°** — an **18.4° systematic error**, exact at 0/90/180/270°, which is why it survived
review and every cardinal-angle eyeball check. Noise amplification at the same pose, per 0.1° of quat
tilt:

| 0.1° of tilt about | shipped `absolute_heading` | world-Z twist strip |
|---|---|---|
| body Y (Right) | **1.673°** | 0.304° |
| body Z (boresight) | **1.551°** | 0.040° |
| body X (Up) | 0.104° | 0.007° |

**3 — World's Roll slot referenced the wrong end of the device.** `triad_roll_deg` defaults to rolling
body **+X**, which on this instrument is the **bottom**. The normal grip read **178.30°**, so a few
degrees of real roll swung the readout across the branch cut (+178 → −178) and read like a fault.

**Fix.** New `sensors.yaw_twist_deg` — the swing–twist term about world Z, `2·atan2(z, w)` on the
world-Z-relative quat, with no singularity at any attitude the device can reach. `absolute_heading`
strips with it, and `YawFusion.update`'s yaw term uses **the same function**: the two must share one
convention or `_delta` chases a moving difference of unlike quantities. `yaw_twist_deg` is
additionally *exactly additive* under `graft_yaw`, so a converged `_delta` lands the fused quat's
heading precisely on the mag heading rather than near it. The gimbal gate and `yaw_gimbal_margin_deg`
are deleted from `sensors.py`, `config.py`, `panel.py`, `web.py` and `static/sensors.js`; a stale key
left in an existing `roomscan.toml` is harmless (the loader ignores unknown keys), so there is no
migration. World roll now passes `up_ref_body=_DEVICE_TOP_BODY` (body −X) and reads **−1.70°** in the
grip; `triad_roll_deg` itself is untouched — it is a primitive and its default stays body +X.

**Relationship to BUG-048 — and a correction to it.** Same defect, same function, found by analysis on
`DebugCapE` instead of by holding the thing; its stated fix direction (replace the Euler yaw strip
with the world-Z swing–twist) is what shipped here, so it closes with this change. But its **scope
paragraph was wrong**: it concluded "NOT a significant live-operation defect" from the fraction of
frames within 1° of lock (0.0–0.2% on normal captures). Proximity in *frames* is the wrong measure —
the error is a smooth 18° bias well before lock, and the live rig in the owner's ordinary grip sits at
ZYX pitch 87.3°. The live path was the *most* affected, not the least.

**Regression tests.** `test_normal_upright_grip_is_not_gated`,
`test_absolute_heading_recovers_true_bearing_at_the_operating_pose` (off-axis bearings are
load-bearing — the ZYX strip is exact on the cardinal four),
`test_yaw_twist_is_the_negated_graft_yaw_error_from_identity`,
`test_yaw_twist_is_exactly_additive_under_graft_yaw`, and
`test_orientation_view_world_roll_is_near_zero_in_the_normal_grip`. All proved by reintroducing each
defect and confirming the failure (`bearing 30.0: heading off by 60.00 deg`; `World roll 178.30 deg`).
Note every pre-existing `YawFusion` test used `LEVEL` (identity) — the one attitude the bug could not
reach, which is how a permanently-tripping gate passed a suite for weeks.

**Lesson.** Third instance of the class after **BUG-039** (body-Z yaw term) and **BUG-048**: an Euler
decomposition used as a *quantity* rather than a display. Any `quat_yaw_deg` consumer in a live path
is suspect. And a test suite that exercises only the identity attitude cannot see a bug whose whole
domain is the pose the device is actually held in.

**⚠ Amended by BUG-058 (2026-08-01): this fix was itself wrong.** `yaw_twist_deg` is not a bearing
either — it absorbs roll about a horizontal boresight one-for-one, so heading tracked ROLL, and the
fusion's `heading − yaw` came out mirrored. The replacement above (`yaw_twist_deg`) is superseded by
`boresight_bearing_deg` / `magnetic_north_bearing_deg`; `test_absolute_heading_recovers_true_bearing_at_the_operating_pose`
survives but no longer fits a convention offset. Read BUG-058 before touching any of this.

## BUG-052 — Detailed build starved the web server's event loop for 80% of wall clock

**Status:** fixed 2026-08-01 · **Area:** host/slam (`slam/tsdf.py`) · **Found by:** owner — "the
progress bar only made it to 325/4808 before the view and bar locked up".

`TsdfMap._extract_vbg()` returned `self._vbg.cpu()` on every extraction: a device→host copy of the
**whole preallocated block buffer**. Its cost is set by `block_count`, not by how much map exists, so
it did not shrink for a small map — measured flat at 1.1 s from the very first extraction (17k
verts) onward, at the Detailed preset's 320,000 blocks (3.05 GiB of buffer).

Open3D holds the GIL for the duration. A background thread does not help: those 1.1 s are 1.1 s in
which **no** Python thread runs, including the asyncio loop serving `/ws` and HTTP. At one
extraction per 25 frames the loop got roughly 0.3 s of every 1.5 s.

**Measured**, captures/DebugCapB1.bin, watchdog thread standing in for the event loop:

| | `.cpu()` whole-grid (before) | extract in place (after) |
|---|---|---|
| worst single stall | 1.11 s | 0.08 s |
| event loop starved | 78–84% of wall | 11.7% |
| full 4808-frame build | ~6 min of extraction alone | 55.6 s total |
| host RSS | 4.24 GB | 1.03 GB |
| peak device memory | — | bounded, 2109 → 2461 MiB |

Splitting the old path: `.cpu()` 1.11 s, then marching cubes on the host only 0.20 s. The copy was
~85% of it, and it was pure overhead — the workaround cost far more than the failure it avoided.

**Fix.** `_extract_vbg()` extracts on the compute device when `_cuda_extract_fits()` sees NVML
headroom (measured 8.0–9.5 KiB/block including the mesh; the check requires 20 KiB, ~2x), and only
the extracted geometry is downloaded — 0.10 s for a 4.09M-vertex mesh vs 1.11 s for the grid that
produced it. `_extract()` keeps the return value host-resident, so no caller (MeshPrep,
`DetailedRunner._commit`, the CLI writers) had to change. The whole-grid host copy remains the
fallback for a map too large, a box without NVML, and — latched permanently by
`_host_extract_reason` — a card that OOMs anyway. That latch is not paranoia: Open3D's own
"Unable to allocate assistance mesh structure for Marching Cubes" surfaced at 219,932 blocks during
this work, and it is exactly the error the original unconditional workaround was written for.

**Lesson.** A mitigation's cost has to be measured against the failure it prevents. This one traded a
rare OOM for a guaranteed 1.1 s process-wide freeze on *every* extraction, and the freeze was
invisible in every unit test because nothing measured whether other threads could run.

---

## BUG-053 — Open3D's marching cubes crashes the process above ~260k active blocks

**Status:** vendor 2026-08-01 · **Area:** host/slam (Open3D 0.19) · **Found by:** investigating
BUG-052 — the Detailed build kept dying even after the lockup was fixed.

Extracting a mesh from a VoxelBlockGrid with too many active blocks does not fail, it **kills the
process**: the host path segfaults inside `extract_triangle_mesh`, and the CUDA path raises
`CUDA runtime error: an illegal memory access was encountered` whose unwind then aborts in a
destructor. Neither is catchable from Python.

Bisected on captures/DebugCapB1.bin (voxel 0.005), one extraction per run:

| blocks | load | host | CUDA |
|---|---|---|---|
| 240,061 | 75.0% | OK (4.10M verts) | — |
| 258,161 | 80.7% | OK (4.39M verts) | — |
| 273,521 | 85.5% | **segfault** | **illegal memory access → terminate** |
| 285,411 | 89.2% | **segfault** | **illegal memory access → terminate** |
| 298,173 | 93.2% | **segfault** | — |

**The trigger is the absolute block count, not the load fraction and not free memory.** Re-running
the 273,521-block case in a 400,000-block grid — 68.4% load, 3.7 GiB free — dies identically on both
devices. So **raising `block_count` does not help**; only producing fewer blocks does, i.e. a
coarser `voxel_size`. This is the natural wrong conclusion to draw, and BUG-035 primes it.

**Mitigation.** `_MAX_SAFE_EXTRACT_BLOCKS = 250,000` (below the last measured-good 258,161).
`TsdfMap.mesh()`/`point_cloud()` raise `TsdfCapacityError` above it rather than making the call.
`PostProcessWorker` stops the build and publishes `phase="failed"` with the remedy in the message;
`DetailedRunner._commit` refuses to write a sidecar for a stopped build, so a partial room is never
marked current. `SlamWorker` instead holds its last good mesh and warns once — a live scan is
unrepeatable, so freezing the view beats killing the server mid-sweep.

The failure publish deliberately **reuses the last published mesh** instead of extracting a fresh
one: a map past the wall cannot be extracted at all, and asking for one there swaps this crash for
an identical one (verified — the guard fires at frame 3038 and `mapper.mesh()` immediately after
segfaults).

A separate, rarer failure is guarded alongside it: at 100% capacity Open3D's rehash allocates a new
buffer of twice the capacity beside the live one, which OOMs an 8 GiB card at 320,000 blocks and
also aborts rather than raising. `_check_rehash_headroom` raises `TsdfCapacityError` first.

**Open opportunity (owner, 2026-08-01):** this is a limit of *this* library, not of the problem.
Chunked extraction — segment the grid spatially, extract each piece under the ceiling, stitch —
or a different meshing package would lift it and let 5 mm voxels work on room-sized captures. See
ROADMAP Phase 6.

---

## BUG-054 — The Detailed preset's 5 mm voxel could never build a room-sized capture

**Status:** fixed 2026-08-01 · **Area:** host/slam (`slam/config.py`) · **Found by:** the BUG-053
bisection.

`DetailedSlamPreset.voxel_size` shipped as 0.005. At 5 mm a room-sized capture generates far more
blocks than BUG-053's ceiling allows: captures/DebugCapB1.bin (4808 frames) crosses 250,000 blocks
at frame 2625 and can never produce a mesh, at any `block_count`. The preset's own
`benchmark_note` — *"benchmark me on CUDA:0 with a full-room capture"* — had never been run, and
5 mm had only ever been exercised on captures small enough to fit (DebugCapC, 2397 frames, builds
fine at 5 mm and its sidecar exists).

**Fix.** Default is now `voxel_size = 0.01`. Measured on the same capture: all 4808 frames,
139,785 blocks (56% of the ceiling), 55.6 s, 4.1M vertices. This changes `preset.fingerprint()`, so
sidecars built at 5 mm are correctly reported stale and want regenerating.

**Lesson.** A default carrying a note that says it was never measured is an untested default. This
one had been shipping as the headline quality setting of an entire display mode.

---

## BUG-055 — The map's scanner marker was drawn 4.44x oversize

**Status:** fixed 2026-08-01 · **Area:** host/web (`static/slam.js`, `static/devicemodel.js`) ·
**Found by:** owner — "the scanner object is far larger than it is in real life".

`devicemodel.js` exports `DEVICE_DIMS = {x: 0.62, y: 0.338, z: 0.282}`, and its own comment says
these are magcal3d's **shell units** — the block scaled to keep the coverage shell's existing 0.62
framing constant, a scene with no metric ground truth in it. `slam.js` called
`createDeviceMesh(THREE, {...})` with no `dims`, took that default, and dropped it into the SLAM /
Detailed scene, which is in **metres**: a 62 cm slab of scanner in a room with 2 m doorways, 4.438x
its real 5.5" x 3" x 2.5".

Nothing catches this by eye where the constant is defined — inside a unitless shell, being 4.44x
too big means nothing.

**Fix.** New `DEVICE_DIMS_M` carries the physical truth (`5.5 * 0.0254` etc.); `slam.js` passes it.
The aperture had the same latent problem — `APERTURE_R = 0.048` absolute is 7.7% of the shell block
but 34% of the metric one, so the real-size device would have rendered as mostly lens — and is now
`APERTURE_R_FRAC` of `dims.x`, with the two face offsets likewise fractions of `dims.z`.

**Verified** in the live page: the `scanner-model` object in the scene measures 5.50 x 3.00 x 2.53
inches (the 2.53 is the aperture disc standing proud of the face, as designed).

**Regression tests.** `test_the_map_marker_is_drawn_at_the_devices_real_metric_size` pins the
`dims: DEVICE_DIMS_M` argument; `test_device_dims_m_are_the_real_block_in_metres` also asserts the
two dimension sets stay *proportional*, so a tweak to one cannot silently reshape the other.

---
## BUG-056 — A finished Detailed build's "Completed in" time never stopped counting

**Status:** fixed 2026-08-01 · **Area:** host/web (`web.py`, `DetailedRunner.status`) ·
**Found by:** verifying the BUG-052 fix — the completed build's toast read "Completed in 81:01" for
a build that had demonstrably taken about three and a half minutes.

`status()` computes `elapsed_s = max(0.0, time.monotonic() - started_at)` on every poll, and
`_started_at` is never frozen when the worker finishes. `done=True` is reported at ~30 Hz forever
after, each time with a larger elapsed. Measured live on the rig: `elapsed_s` went 4910.5 → 4914.5
across four seconds of polling, on a build whose sidecar timestamp put it 4:35 after server start.

The number is not cosmetic — it is the only report of how long a reconstruction takes, and the
preset's `per_frame_ms` calibration (still `0.0`, "benchmark me") is supposed to be derived from it.
A monotonically inflating duration would have poisoned that calibration too.

**Fix.** `_elapsed_at_done` is stamped the first time `status()` sees `progress.done` and reused for
every later poll. It is cleared only by `start()`, `begin_load_cached()` and `close()` — never by
`status()` itself, so a completed build's time cannot drift.

**Regression test.** `test_detailed_runner_freezes_elapsed_time_when_the_build_finishes` reads the
elapsed at completion, advances `time.monotonic` by an hour, asserts it is unchanged, then starts a
fresh build and asserts it does *not* inherit the previous one's time. Proved by reintroducing the
unfrozen subtraction and confirming the failure.

**Lesson.** Third clock/duration defect in this file after BUG-050 and the recording elapsed. A
duration derived from `now - start` needs an explicit answer to "when does it stop?", and "when the
thing finishes" has to be written down, not assumed.
## BUG-057 — Going Live after viewing a capture booked 1.5M phantom frame gaps

**Status:** fixed 2026-08-01 · **Area:** host/web (`viewer.py` `Stats`, `web.py` `SessionController`) ·
**Found by:** restoring the owner's live view at the end of the BUG-052 session — the HUD came back
reading 1,529,274 gaps against 0 drops at a clean 30 fps.

`SessionController` holds one long-lived `Stats`, and `Stats._last_seq` is never cleared when the
source changes. Sequence numbers are **per-source**, so the first live frame after a replay differs
from the last replayed frame by however far apart the two numberings happen to be — and
`update()` adds that entire difference to `seq_gaps` as lost frames.

**Why it matters more than a cosmetic counter.** `seq_gaps` is one of the numbers BUG-049's
whole-group transport-loss investigation reads. A counter that silently absorbs a six-figure
artifact the moment an operator previews a capture and goes back to live is worse than no counter:
it is the same trap as BUG-040 (`drops`/`gaps` structurally pinned at 0) seen from the other end —
**a number nobody has confirmed is reset is not evidence.**

**Fix.** `Stats.new_stream()` clears `_last_seq` while deliberately keeping `frames` /
`seq_gaps` / `dropped_flags` — those are session-level health totals. `switch_to_replay` and
`switch_to_live` both call it, beside the existing `sensor_state.clear_imu_raw()` which exists for
exactly the same per-source reason.

**Regression test.** `test_stats_new_stream_forgets_the_sequence_without_losing_the_totals` counts a
real gap, swaps to a distant numbering, and asserts the swap adds nothing while the earlier total
and the running frame count survive — then that real gaps are still counted afterwards. Proved by
reintroducing the defect.

---
## BUG-058 — Heading tracked ROLL, and the yaw fusion came out mirrored

**Status:** fixed 2026-08-01 · **Area:** host/sensors (`sensors.py` `absolute_heading`, `YawFusion`) ·
**Found by:** owner — `captures/NorthFacingRoll.bin`, "the device was always facing north" while the
reported heading swung wildly.

Rolling the instrument in the hand at a fixed bearing moved the reported heading **154°** over 153°
of roll (circular range; the raw min-max crosses the ±180 wrap), at slope **−0.978** against roll and **+0.005** against the device's actual bearing. It is
not a calibration problem: `capture_magcheck` scores that capture `attitude_locked 0.18%`, and the
field's own world-frame azimuth is flat across it (sd 2.2°).

`absolute_heading` stripped the quat's yaw with `yaw_twist_deg` — the swing–twist twist about world
Z — then read the de-tilted field. But **world-Z twist is a property of the whole rotation, not the
azimuth of any axis**: with the boresight horizontal, a roll about it is a rotation about a
*horizontal* world axis, and the twist absorbs it one-for-one. So the function reduced to
`constant − roll`. It read exactly right at roll 0 and 180 — the two grips anyone checks.

**`YawFusion` had a worse form of the same defect, and it was live.** Its correction was
`heading − yaw`, a device bearing minus a world-Z twist. Those are not the same quantity and the
device term does not cancel, so every turn was counted twice: fed a synthetic device at a known
bearing with a known yaw drift, the fused quaternion's boresight came out at **−bearing**, mirrored,
at every roll and pitch tried. Since `SensorState.fused_quat()` is the primary orientation — gizmo,
World readout, cloud rotation, SLAM rotation prior — that mirror was in the shipped path from the
day BUG-051 first let the filter run (2026-07-31; before that the gimbal gate had frozen it
permanently, which is why this never showed up earlier).

**Fix.** Stop extracting a scalar yaw from an attitude at all.
* `magnetic_north_bearing_deg(quat, mag)` — the compass bearing of magnetic north *in the quat's own
  frame*, i.e. how far that frame's +X datum has drifted off north. Reads no body axis, so it has no
  attitude singularity, and it is exactly the correction: `_delta` converges on it directly.
* `absolute_heading` — the boresight's bearing minus north's, both read in the same frame. The
  drifting datum cancels identically rather than being stripped, and the answer is roll-invariant
  because neither term touches roll. `None` within 10° of vertical (`boresight_bearing_deg`), where a
  bearing does not exist; `web.orientation_view` reports the reason and the client already drew a dash.
* `YawFusion` gains `gated:no-field` (horizontal field ≈ 0) and drops its heading/yaw terms entirely.

**Verified.** On the owner's capture the heading-minus-bearing residual falls from sd 42.7° / span
154.6° to sd 2.2° / span 11.0°, and it holds on three more real captures spanning full 360° yaw
(`roomSweepFull20260730` sd 6.1°, `coffeeRoomCircuitNoMnt` sd 6.0°) — the mag-derived bearing now
tracks the quat's own bearing 1:1, which is what "same physical quantity" has to look like.
`tilt_sweep_20260729` correctly returns no heading at all: it is aimed 89.7° up for its whole length.

Then end-to-end on the running server, replaying that capture and reading the World card's own
`/ws` `sensor` messages: over the full 152.8° of roll the reported Heading moves 19.3°, slope
**+0.022** deg per deg of roll (was −0.978), residual sd 4.11°. The `gated:no-field` path is
unreachable on this capture and correctly never fires.

**Regression tests.** `test_absolute_heading_is_unmoved_by_roll_about_the_boresight` sweeps roll
(and pitch) at four bearings; `test_fusion_lands_the_fused_boresight_on_the_true_magnetic_bearing`
drives the filter from ground truth across bearing × roll × pitch × drift and asserts the *fused
boresight*, which is what catches the mirror; `test_absolute_heading_is_none_where_no_bearing_exists`
pins the pole. `test_no_new_yaw_twist_consumers` is a new AST guard, empty by design. All proved by
reintroducing the defect.

**Instrument.** `host/tools/heading_check.py` / MCP `capture_heading` — regresses `absolute_heading`
on the quat's own boresight bearing *and* its roll together, which must come out 1 and 0. It scores
**−0.984 [−1.036, −0.944]** for the old heading on `NorthFacingRoll.bin` and **+0.016 [−0.036,
+0.056]** for the new one, so it is proved against a known-bad input and not only against a working
one. Per-axis verdicts sit on a block-bootstrap interval: the residual is yaw drift and room field
wandering over seconds, and treating that as white noise called a real circuit's roll axis `bad` at
0.181 when the honest answer is that 36° of roll and 5.8° of drift cannot resolve it.

**Lesson.** The BUG-051 fix replaced one wrong yaw scalar with another and its regression test swept
the axis that had just burned it — bearing — while holding **roll** fixed; the fix's own test could
not see the bug the fix introduced. Fourth instance of the class (BUG-039/048/051/058), so the rule is
now stated without an escape hatch: **no scalar "yaw" off an attitude is the bearing of an axis.**
Ask for the bearing of the axis you actually mean. Second lesson: every assertion in the yaw-fusion
suite was on the *magnitude* of a correction, and a 180°-mirrored filter passed all of them — sign
conventions need a test that names a physical direction, not a number.

---
## BUG-059 — The magnetometer vector was anti-parallel: north read as south

**Status:** fixed 2026-08-02 · **Area:** host/sensors (`sensors.py` `AXIS_CONVENTION`) ·
**Found by:** owner, immediately after BUG-058 made the heading mean something — "when facing
north, the heading shows south".

A clean +180°, and the cause needs no compass to find. Earth's field points north **and down** —
about 70° below horizontal at mid-northern latitudes. Rotating the calibrated, axis-corrected
vector into the quat's world frame put it **70.0° ABOVE** the horizon on `NorthFacingRoll.bin` and
**72.4°** above on `roomSweepFull20260730.bin`: anti-parallel, right magnitude, exactly reversed.
An inverted field puts magnetic north 180° from where it is, so `absolute_heading` read 186.9° on a
capture the owner had aimed north.

The frame that test assumes was verified rather than trusted: rotating the **accelerometer's**
gravity into the same world frame gives (0.000, 0.000, −1.000) over that capture, so world Z is up
as documented and the dip sign means what it says.

**Which half was wrong.** Enumerating all 48 signed permutations against three constraints
separates them. The field is world-fixed, so magnetic north's bearing must be *constant* over a
360° sweep — that needs no ground truth, and only the existing axis assignment and its negation
hold it (sd 6.1°; every other permutation scatters north 75–79°). Of those two, only one puts the
field below the horizon. So the **mounting rotation was always right** and only the overall
**sign** was wrong. `AXIS_CONVENTION` is now written as its two factors,
`MAG_FIELD_SIGN * MAG_MOUNT_ROTATION`, because they rest on different evidence.

The product has **det −1**, which is correct and is not a defect to tidy away: a sign convention
composed with a rotation is not a rotation. Same class as `gravity_body_from_imu_raw` negating the
accelerometer's sensed reaction.

**Age.** This predates BUG-058 and is independent of it. The old `absolute_heading` also derived
north from this vector, so it carried the same 180° — it was simply invisible underneath a number
that swung 154° with the operator's wrist. Every heading displayed and every `YawFusion` datum since
BUG-004 set this matrix on 2026-07-10 was reversed; before that the matrix was identity, which was
wrong differently.

**Verified.** Inclination −70.0° → **+70.0°**; heading on the owner's north-facing capture
**186.9° → 6.9°**, the residual being magnetic declination plus how precisely "north" is held by
hand. `mag_cal.json` is untouched and unaffected — the ellipsoid fit is applied *before* this
matrix, so no recalibration is needed.

**Regression tests.** `test_axis_convention_puts_the_field_below_the_horizon` and
`test_heading_reads_north_on_the_owners_north_facing_capture` run on 16 real (quat, raw mag)
samples embedded from that capture, with the calibration inlined so a re-fit cannot move them;
`test_the_mounting_half_of_the_axis_convention_is_a_proper_rotation` pins the decomposition and
says in its docstring why det −1 is deliberate. All three proved by reintroducing the sign.

**Lesson.** BUG-004 recorded `AXIS_CONVENTION` as "verified against all 24 permutations" — but the
criterion available then was |B|, which is invariant under **every** signed permutation, including
the ones that invert the field. **A check that cannot distinguish the candidates is not a
verification, however many candidates it is run against.** The discriminating check was cheap and
sitting there the whole time: the field has a *direction* relative to gravity, and gravity is
measured. Note also that `capture_heading`, written hours earlier for BUG-058, scores this capture
identically before and after — it regresses slopes, so a constant offset is invisible to it. Its
docstring already said it cannot see absolute direction; this is that limitation biting on the
very next bug.


## BUG-060 — Live SLAM ran at 5 fps and updated the view once a second, on an idle GPU

**Status:** fixed 2026-08-02 · **Area:** host/web (`web.py` broadcaster + `SlamRunner`,
`reader.py`, `slam/meshprep.py`, `static/slam.js`) · **Found by:** owner — "in SLAM mode the GPU
and CPU are hardly working at all, yet the viewport updates extremely jerkily (1 Hz). The display
is 144 Hz and the data stream is 30 Hz. nvidia-smi shows 10 W and 0% gpu-util."

The idle GPU was the clue, not the puzzle: the pipeline was only being *asked* for ~5 frames of
work a second. Measured on the live rig — MESH 1.03/s, `slam` messages 5.05/s, mapper
`frames_integrated` growing 5.00 fps against a 30.3 Hz stream.

**It was the event loop, and a plain HTTP probe proved it.** Hammering
`GET /static/index.html` alongside a `/ws` probe showed HTTP stalling by the *same* ~780 ms inside
every websocket gap (p50 4.4 ms, p90 766 ms). That rules out the client and rules out websocket
send-backpressure: the whole loop was frozen, ~630 ms out of every second.

**Cause: `permessage-deflate`, once per client, on the loop.** `uvicorn.run()` defaults
`ws_per_message_deflate=True`, and `_broadcast_bytes` awaits `send_bytes` **per client** — so each
3.2 MB MESH payload was zlib-deflated once per connected tab, synchronously. The A/B on the live
server is clean: **+130 ms of whole-process freeze per additional deflate client** (0→1 +128 ms,
1→2 +133 ms), **+0 ms** for clients negotiating `compression=None`. Six clients were attached (the
owner's browser plus three stale headless-Chrome connections and two MCP clients) ≈ 630 ms. What
the cost bought: float32 geometry deflates to **80%**.

**The consequence nobody would have seen.** `SlamRunner.submit()` was called only from that frozen
broadcaster, so the mapper was fed 5 fps and **5 frames in 6 never reached the TSDF** — silently.
That is a reconstruction-quality defect, not a UI one, and it is why the viewport sat at 1 Hz:
`mesh_every = 5` on a 5 fps worker is one mesh per second.

**Fixes.** (1) `ws_per_message_deflate=False`. (2) A new `on_frame` tap in `reader.py` feeds SLAM
from the **reader thread**, so reconstruction rate no longer depends on display health; the
broadcaster only polls. `SlamRunner` therefore builds its Open3D/CUDA pipeline **asynchronously**
(inline construction on the reader thread would overflow the UDP socket) and drops frames until it
lands, with a `_generation` counter so a source swap mid-build cannot install an orphan.
(3) `MeshPrep` gained an optional `packer`, moving the O(map) serialisation off the loop.
(4) `slam.js` evaluates the Turbo/Gray colormap in the **vertex shader** instead of recoloring
every vertex in JS on every mesh, and uploads the received typed-array views instead of copying
them.

**Verified on the live rig**, 20 s in Live SLAM: mapper 5.00 → **30.40 fps**, MESH 1.03 → **6.11/s**,
`slam` 5.05 → **30.05/s**, HTTP p90 766 → **4.8 ms** / max 816 → **16.7 ms**, stalls over 300 ms
22 → **0**. The per-client scaling is gone entirely (0/+1/+2 clients: no stalls at all).

**The wrong fix, measured and reverted.** `MeshPrep`'s adaptive decimation arms only via
`note_upload_ms()`, which **only `panel.py` calls** — so `live_vertex_budget = 150000` is dead code
on the web path and the live mesh streams full-res forever (611,290 verts / **30.97 MB** by frame
1500 of `roomSweepFull20260730.bin`). Forcing decimation on looks obviously right and is 14× worse:
`prepare_packet` went 178 → **2440 ms p50** (max 383 → **8377 ms**) and GIL starvation 11.9% →
**94.3% of wall**, because `simplify_quadric_decimation` is C++ that never releases the GIL.
Reverted. The payload is bounded by **cadence** instead — a byte-rate governor in `SlamRunner.poll`
spaces publishes by `len(last_payload) / live_mesh_bytes_per_s` (new `[slam]` key, default
12 MB/s), which is free: a small map is unaffected, a 31 MB map updates once per ~2.6 s, and a map
that big changes slowly anyway.

**Still open.** The governor bounds the wire *rate*, not the *peak* — the map is still re-sent
whole and grows without bound. Delta / dirty-region mesh transport is the real answer and is worth
its own sub-phase; it is the same incremental-extraction idea ROADMAP 6.I already wants.

**New instrument.** `host/tools/slam_stall_profile.py` / MCP `slam_stall_profile` — per-stage
timings *and* the GIL-starvation watchdog ROADMAP 6.I asked for. It is the tool that separated the
two decimation results above, and the reason to read starvation rather than wall time.

**Lessons.** (a) **A native call's wall time is not its blocking cost** — `prepare_packet` at
178 ms costs 11.9% of wall, at 2440 ms it costs 94.3%; only the watchdog distinguishes numpy
(releases the GIL) from Open3D C++ (does not). (b) **Cost a mitigation against the failure it
prevents** — second instance after BUG-052, and this time the mitigation was mine. (c) **Test at
scale**: 1200 frames of a near-static capture showed *zero* stalls on code that freezes 1261 ms on
a real room sweep. (d) **Connection count is a performance variable here** — five stale
headless/MCP connections were 5/6 of the freeze.

---

## BUG-061 — Ephemeral SLAM lagged 15 s behind the operator: 37 MB of unbounded websocket backlog

**Status:** fixed 2026-08-02 · **Area:** host/web (`web.py` mesh flow, `static/ws.js` ·
`slam.js` · `scene.js` · `index.html`, `slam/worker.py`, `slam/gpumem.py`,
`mcp_server/session.py`) · **Found by:** owner — "Ephemeral SLAM is still extremely laggy, even in
playback mode. When in live mode, it's terrible. After we start moving the camera around, the
camera will take up to 15 seconds to reflect its new orientation and show new points."

This is BUG-060's sequel, and it survived that fix because BUG-060 removed the event-loop *freeze*,
not the *queue*. The byte-rate governor it added bounds the wire **rate**; nothing bounded the
backlog.

**Proved before a line changed.** `ss -tnp | grep :8000` on the owner's browser connection:
**32,861,676 bytes unread client-side + 4,132,544 bytes in the server's send queue ≈ 37 MB in
flight**, while the MCP client's connection on the same server sat at 0/0. 37 MB draining at the
governor's assumed 12 MB/s *is* the ~15 s the owner saw. The queue held the answer, and one
command read it.

**Cause: `/ws` has no backpressure, and pose rode behind the mesh.** uvicorn's
`WebSocketsSansIOProtocol` never blocks `send_bytes` — the `writable` event is never cleared — so
the per-client asyncio transport buffer is unbounded and the MESH governor is **open-loop**: it
meters whole-map re-sends against an *assumed* 12 MB/s that has no relationship to the real link
(the rig is behind a Wi-Fi bridge). Every excess byte accumulated. The 30 Hz `slam` pose JSON was
then enqueued **after** those megabytes on the same ordered TCP stream, so orientation was
head-of-line blocked behind the entire backlog — which is why the *pose* lagged exactly as badly
as the geometry, and why the lag grew without bound as the map grew.

The client compounded it: a `dispose()` + brand-new `BufferGeometry` per MESH, and no
`frustumCulled = false`, so every packet also cost an O(N) `computeBoundingSphere` on the render
thread.

**Fixes.**
1. **MESH moved off `/ws` entirely, onto a new credit-gated `/ws-mesh`** — one mesh in flight per
   client, latest-wins, credit released by an inbound `{"type":"mesh_ack","seq":N}`. Newness is
   decided by **bytes identity, not `mesh_seq`** (the sequence resets on `_reset_slam`). A client
   that never acks (legacy) still gets a bounded 1 mesh / 5 s; an unanswered credit clears after
   60 s. *Accepted deviation:* the ack timeout clears credit but does **not** re-push identical
   bytes — the next new mesh flows, which is what live SLAM always produces.
2. **Pose goes out first.** Both broadcaster tick sites send the `slam` JSON before touching mesh
   bytes, and `SlamWorker.run_once` now publishes the pose **immediately after `Mapper.step`** —
   before the throttled `mesh()` extraction — republishing only when extraction actually
   succeeded. Pose age no longer depends on extraction cadence at all.
3. **Client**: `frustumCulled = false` on all six SLAM objects and the scanner (the BUG-033
   stale-bounding-sphere trap again), geometry **reused** via `ensureAttrCapacity` /
   `ensureIndexCapacity` with 1.5× headroom instead of rebuilt per packet, and the ack fired from
   `requestAnimationFrame` *after* the render — so credit returns only when a frame really landed.
4. `metrics.ws` flow counters, `ts` + `device` on the `slam` message, and an NVML utilization row.
   It is labelled `gpu_source = "nvml-device"` because device-wide utilization **cannot** prove
   that *SLAM* used the GPU.
5. The MCP `RigSession` opens `/ws-mesh` best-effort and auto-acks, so an agent client can never
   become the stalled client that starves the owner's browser.

**Verified** on a restarted server replaying `web_20260802_124501.bin` at 1× in SLAM display:

| | before | after |
|---|---|---|
| backlog, owner's browser connection | 32.8 MB unread + 4.1 MB send-Q | **0 / 0** (peak 7,888 B) |
| `slam` pose age, client that keeps up | unbounded, ~15 s | p50 **1.1 ms**, p95 **4.8 ms**, max 63.4 ms (n=575) |
| SLAM compute device in the HUD | not shown | **CUDA:0** |

Credit is demonstrably cycling (32 acks across mesh seq 113→162), and FPV/Mirror now hide the
scanner model with Follow disabled.

**The residual, stated plainly.** On the *headless llvmpipe* client pose age is p50 2.61 s /
p95 2.82 s — it does **not** meet the ≤0.15 s target. That client renders at **2 fps**, and the
ack is deliberately tied to `rAF`, so its pose age is now bounded by its own render rate instead
of by transport. That is the intended shape of the fix — a slow consumer starves *itself* and
never accumulates a server-side queue — but it means the ≤0.15 s figure is only demonstrated on a
client that can keep up. **The owner's real browser was never instrumented**, only its socket
queues, which were clean.

**Still open.** Unchanged from BUG-060: the map is re-sent **whole** and grows without bound.
`/ws-mesh` bounds the *backlog*, not the *payload*. Delta / dirty-region extraction and quantized
mesh transport remain ROADMAP 6.I.

**Lessons.** (a) **"No backpressure" is a property of the stack, not something to assume away** —
a governor metered against an *assumed* drain rate is open-loop, and the socket queue is where the
truth was. (b) **Ordering matters on an ordered transport**: putting a 30 Hz control message behind
a multi-megabyte payload on one stream turns a bandwidth problem into a latency problem, and the
symptom (stale *camera*) points nowhere near the cause (mesh size). (c) A credit/ack loop makes a
slow consumer starve itself rather than the server — but then **the contract must be measured on a
client that can keep up**, or you are measuring the renderer, not the transport.

---

## BUG-062 — thirteen `[slam]` keys were honoured by the CLI and ignored by Live SLAM

**Status:** fixed 2026-08-02 · **Area:** host/slam (`slam/config.py`, `web.py`
`SlamRunner._construct`) · **Found by:** the owner's concurrency/GPU review notes (2026-08-02),
which flagged `icp_mode`, `max_iter`, `max_dist` and the quality gates as "not forwarded"; verifying
the claim found **ten more**.

`Mapper.__init__` takes eighteen knobs that `SlamConfig` also carries.
`DetailedSlamPreset.mapper_kwargs()` forwards nearly all of them and `slam/cli.py` builds its
`Mapper` directly — but **`web.SlamRunner._construct` hand-picked five**
(`release_cache_every`, `block_count`, `icp_retry_dist`, `baro_authority`, `baro_tau_frames`).
The other thirteen were dropped: `icp_mode`, `voxel_size`, `max_dist`, `max_iter`, `min_fitness`,
`max_rmse`, `min_confidence`, `weight_threshold`, and all five `stationary_*` keys.

**Why this is worse than a missing feature.** The failure is silent and it inverts trust in the
config: a user edits `roomscan.toml`, the CLI and Detailed reconstructions change, Live SLAM does
not, and nothing anywhere says so. It also **invalidates the study that was about to be run** — the
review's own item #4 is a matched CPU/CUDA comparison of `icp_mode` `translation` vs `6dof` on the
live path, and that path ignored `icp_mode`. The study would have measured `translation` twice and
reported it as a result.

**Fix.** `SlamConfig.mapper_kwargs()` is now the single source for the live mapper's knobs. The web
path splats it and overrides only the two genuinely live-specific values — the measured sensor FOV
and the resolved compute device — applied *after* the splat so a `[slam]` key cannot shadow either.

**Behaviour-neutral at stock config, and pinned.** `test_mapper_kwargs_defaults_match_mapper_signature`
asserts every default in the dict equals the corresponding `Mapper.__init__` default, so a user who
never set a value gets bit-identical behaviour; forwarding a field whose defaults had drifted would
otherwise have silently changed Live SLAM for everyone. Two further guards: every key must be a real
`Mapper` parameter (a typo would now fail at the moment SLAM arms, on the reader thread, in front of
the owner), and every field `SlamConfig` and `Mapper` share must appear — which is the drift that
caused this.

**Proved by reintroducing the defect.** Restoring the hand-picked call makes
`test_live_slam_forwards_every_configured_mapper_knob` fail and name all thirteen keys.

**`device` is deliberately still not read from `[slam]`.** `SlamConfig.device` defaults to
`"CPU:0"` and the live path has always resolved `preferred_device()` instead. Making the field
authoritative here would move every stock-config user's Live SLAM onto the CPU, so the helper keeps
`preferred_device()` and that field's existing note stands. Worth revisiting as its own decision —
not as a side effect of a plumbing fix.

**Lesson.** A second construction site for the same object is where configuration quietly stops
being configuration. The guard that matters is not "did I remember to forward this field" but
"is there exactly one place that knows what the field list is".

## BUG-063 — the GIL-starvation metric under-reports precisely when starvation is total

**Status:** open (measured, one instrument fixed, one not) · **Area:** host/tools
(`slam_stall_profile.py`; `slam_icp_bench.py` already corrected) · **Found by:** the item 4 matched
CUDA ICP study, 2026-08-02, when a variant that visibly froze the process reported a *better*
starvation figure than stages that ran fine.

**The defect.** Both watchdog-based instruments compute starvation as **summed tick lateness over
wall time**. That construction is self-defeating: if the measured code holds the GIL continuously,
the watchdog thread barely gets scheduled, so there are almost no lateness samples to sum, and the
metric falls toward zero exactly as blocking approaches total.

**Measured.** Over a 10.93 s `gpu_translation` stage the watchdog landed **1 tick where ~2186 were
due**, and that single tick was 2998.9 ms late. The stage reported **"10.3% starvation"** — better
than stages that were genuinely healthy. Its true behaviour: whole-process freezes of 598.7 /
1626.8 / 2998.9 / 3981.6 ms, while **no single `register()` call exceeded 9 ms**.

**The fix, where applied.** `slam_icp_bench.py` reports **`tick_share` = ticks landed / ticks due**,
which degrades in the right direction:

| Variant | `tick_share` | ticks / due | worst stall | (legacy) starved % |
|---|---|---|---|---|
| `translation` (shipped) | 0.953 | 1470 / 1542 | 2.0 ms | 4.7% |
| `6dof` | 0.989 | 1334 / 1349 | 0.2 ms | 1.1% |
| `gpu_translation` | **0.058** | 127 / 2209 | **3981.6 ms** | 66.7% |
| `translation_cpu_nns` | 0.951 | 1553 / 1632 | 4.2 ms | 4.9% |

**Still to do.** `host/tools/slam_stall_profile.py` computes starvation the same way and has the
same blind spot. **Its published numbers are not in doubt** — BUG-060's 11.9% vs 94.3% comparison
had ample ticks in both regimes, so the sum was well-sampled — but it should gain `tick_share`
before it is next used to clear a change, because the one situation it cannot currently see is the
one it exists to catch.

**Why this matters beyond the metric.** A low starvation reading is used as evidence that a change
is safe for `roomscan-web`, whose asyncio loop, reader thread and broadcaster all need the GIL. An
instrument that reads *healthy* under total blocking would have waved through exactly the
BUG-060/BUG-061 failure mode. Generalises the repo's standing lesson one level down: a native call's
wall time is not its blocking cost — and here even the starvation *percentage* was not its blocking
cost. Pair any "it didn't starve" claim with the number of samples the claim rests on.

---

## BUG-064 — "SLAM rendered nothing": it rendered perfectly, and two cards covered 97% of it

**Status:** fixed · **Area:** host/web (`static/browser.js`) · **Found by:** the owner, 2026-08-02,
reporting a live SLAM session that "rendered nothing at all", then the same of an offline replay.

**The symptom was true and the inference from it was wrong.** Nothing appeared, so every candidate
considered was upstream: the mapper never armed, the worker failed to construct, the mesh never got
packed, the credit transport stalled. All four were wrong. Measured on the live server while it was
displaying the capture:

| stage | evidence | verdict |
|---|---|---|
| mapper | `slam` msg: 468 frames integrated, `mesh_seq` 76, 2656 blocks, `CUDA:0` | healthy |
| transport | `/ws` 360 `slam` msgs in 12 s; `/ws-mesh` delivered 3,410,108 B | healthy |
| payload | re-parsed with `slam.js`'s layout: 8 sections, **slack 0 bytes** | healthy |
| client | `slam.js: first mesh: 24533 non-wall verts`, `group.visible = true` | healthy |

The map was on screen the whole time. `#browser-card` (Captures) and `#preview-card` sit in the
centre column, which is where the map is drawn: projecting the mesh's bounding box gave a screen
rect of 447x482 px, of which **97% was covered** by those two cards — 112,788 px by the browser and
95,918 px by the preview — leaving the 14 px gutter between them, which is the thin coloured strip
visible in the owner's screenshots and reads as noise. **Live mode never showed this** because both
cards carry `classList.toggle('hidden', source !== 'view')`.

**The fix.** Entering a map display (`slam`/`detailed`) in View collapses both cards. Collapse, not
`.hidden`: the squircle rail is a permanent map of every panel, so one click brings them back and
you can still pick another capture without leaving the map. It fires **only on the transition into**
a map display, never on each `state` echo — `state` is re-broadcast on every unrelated setting
change, and re-collapsing on each one would fight the user every time they reopened a card (the
same trap that killed the oscillate orbit's return leg). After: **coverage 0.97 → 0.02**.

**What this cost, and the lesson.** Roughly an hour of instrumenting a pipeline that was working,
because "nothing rendered" was read as "nothing was produced". A renderer has a stage the other
diagnostics never reach: **something drew over it**. When every upstream stage measures healthy,
stop bisecting upstream and ask what is in front of the camera — projecting the geometry's bounding
box to screen coordinates and intersecting it with the DOM answers that in one call. Generalises
BUG-033 and BUG-061 Part D, both of which were also "the geometry is fine, the *view* is not".

---

## BUG-065 — padded vertex buffers gave a fractional `count`, so every SLAM bound was NaN

**Status:** fixed · **Area:** host/web (`static/slam.js`) · **Found by:** inspecting the live scene
graph while chasing BUG-064, 2026-08-02.

**The defect.** `ensureAttrCapacity` allocates with headroom so consecutive growing packets don't
re-trigger a GPU realloc (BUG-061 Part A):

```js
const capacity = Math.ceil(Math.max(needLen, 1) * GROWTH_HEADROOM);   // 1.5
```

`needLen` is a **flat element count** (`3 * vertexCount`), so this rounds to a whole *element*, not
a whole *item*. For most inputs the result is not a multiple of `itemSize`: `3 * 24533 * 1.5 =
110398.5 → 110399`. `BufferAttribute.count` is `array.length / itemSize`, which is then
**36799.666…**, and `computeBoundingBox`/`computeBoundingSphere` walk `i < count`, read
`array[110399]` — one past the end — get `undefined`, and return **NaN**.

**Measured live**, non-wall mesh: `count: 36799.666666666664`, `boundingBox.min.z: NaN`,
`boundingBox.max.z: NaN`. Unioning it into the map's bounds poisoned those too, so "where is the map
on screen" could not be answered until the padding was accounted for.

**Why nothing was visibly broken.** The *draw* is bounded by the index plus `setDrawRange`, neither
of which consults `count`, and all six SLAM objects carry `frustumCulled = false` (BUG-033,
BUG-061 Part D) so nothing asked for the bounds. It is a landmine, not a live failure: re-enable
culling on any one of them, add a raycast/pick, or write a frame-the-map camera helper, and it
inherits the NaN — and a NaN bounding sphere culls the object **always**, which presents as exactly
the blank viewport of BUG-033.

**The fix.** Round the padded capacity up to a whole number of items, and set `count` to the **live**
element count after each `array.set` so bounds describe the data rather than the allocation (the
zeroed tail would otherwise fold the world origin into every bound). Bounds are invalidated on each
rewrite. Verified on the rig: all 14 SLAM geometries report integer `count`, array lengths divisible
by 3, and finite boxes and spheres.

---

## BUG-066 — `load_capture` broadcast `mode: "realtime"` alongside `display: "slam"`

**Status:** fixed · **Area:** host/web (`web.py`) · **Found by:** reading `rig_status` output during
BUG-064, 2026-08-02.

`load_capture` set `ui.mode = "realtime"` unconditionally while leaving `ui.display` alone, so
loading a capture while the display was `slam` put a self-contradicting `state` on the wire. Every
other site that moves either field derives the alias (`ui.mode = "realtime" if ui.display ==
"point_cloud" else "slam"`); this one did not.

Nothing was visibly broken: `slam.js` reads `display` and treats `mode` only as a fallback for an old
server. But `mode` is the documented compatibility alias, and a stale alias is trusted by whoever
reads it next — which is the whole shape of BUG-044 (eight `state` echoes dropping capability
context) one field over. Fixed by deriving it like everywhere else; verified by driving a real
`load_capture` over `/ws` while the display was `slam` and asserting the echo agrees.

---

## BUG-067 — a stationary tripod scan reports 18–20 m of travel, silently

**Status:** open (architectural — needs a design decision, not a patch) · **Area:** host/slam
(`slam/odometry.py` `register`, `slam/mapper.py` `step`) · **Found by:** the owner's
`captures/imuTranslationError.bin`, 2026-08-02: "in ephemeral slam mode it seems to go crazy and
start moving in different directions; in detailed slam mode it also shows the scanner moving
through a wall."

The capture is a **tripod** scan — 4 holds and 3 pans, tilt 29.6°→124.4°, up to 156 °/s — so the
sensor's true translation is the tripod head's lever arm, centimetres. It is also clean: 0 CRC
failures, 3 lost frames of 3044 (0.1%), so BUG-049 is not in play.

Ensembles of 10 runs each (standard innocuous perturbations), truth ≈ 0:

| prior | max excursion | net | path | tracking-lost |
|---|---|---|---|---|
| raw SFLP (Detailed, `_load_frames`) | 0.62 ± 0.37 m | 0.40 ± 0.43 m | 18.0 m | **0 / 3019** |
| mag-fused (Live, `sensor.fused_quat()`) | 1.69 ± 0.52 m | 1.56 ± 0.57 m | 20.0 m | **0 / 3019** |

**The failure is silent and confident.** Mean ICP fitness is 0.88 in every run and not one frame is
reported lost. Nothing in the HUD, the metrics, or `Mapper.lost_flags` says the trajectory is
fabricated — which is the same shape as BUG-035 and BUG-049: the reconstruction is wrong in a way
the instrument cannot see.

**Mechanism.** `register(mode="translation")` holds rotation at `init_pose`'s rotation — the SFLP
prior — and solves only 3-DoF translation. There is therefore **no degree of freedom in which a
rotation-prior error can be expressed as a rotation**; the solver's only way to reduce the
point-to-plane residual is to move the sensor. Two measurements:

* Per-frame step scales monotonically with angular rate — 3.3 mm/frame below 0.1 °/frame, 7.0 at
  0.3–1.0, 11.3 at 2–4, 17.9 above 4 (corr 0.40). Drift is concentrated in the pans; the four
  holds contribute ~2 cm each while one 26 s pan contributes 0.39 m.
* Shifting the prior by **one frame (33 ms)** moves net drift from 0.24 m to **1.92 m**.

Because SLAM is frame-to-model, each fabricated increment is integrated into the TSDF and becomes
the reference the next frame is matched against, so the error is latched rather than averaged out.

**Ruled out.** Transport loss (0.1%); the barometer (`baro_authority=0` gives 0.53 vs 0.49 m);
live frame-drops — decimating to 10 fps and 5 fps makes it *better*, 0.38 and 0.31 m, not worse, so
`SlamWorker`'s latest-wins slot is not implicated; and empty FOV (97–100% of zones valid
throughout, mean range 1.25–3.3 m).

**Not fixable by tuning.** The two candidate directions are a genuine zero-velocity constraint (see
BUG-069, whose gate is both unable to fire here and display-only) and giving the solver a rotational
degree of freedom with the IMU as a *soft* prior rather than a hard constraint — but `6dof` is
already disqualified on accuracy by the 2026-08-02 CUDA ICP study (8.0 ± 3.6 m closure vs 0.67), so
this is a soft-prior design question, not a mode switch. BUG-068 bounds the worst single event but
does not address this.

**Related, and worth costing first:** BUG-031 measured the SFLP quaternion as leading the depth
frame by **+7.76 ms**, put `quat_mid_ticks`/`quat_n` on the wire, and applied nothing, noting the
correction "wants its own before/after on a moving capture". The one-frame-shift number above is
that sensitivity, and it is large.

**But a sub-frame phase sweep does NOT localize the prior's phase — do not read it as validating
that correction.** Slerping the prior by α frames (n=5 per point, max excursion):

```
  alpha    ms        excursion               net
  -0.50  -16.6   0.502 +/- 0.046    0.230 +/- 0.031
  -0.25   -8.3   0.496 +/- 0.017    0.234 +/- 0.022
  +0.00   +0.0   0.752 +/- 0.489    0.546 +/- 0.569
  +0.25   +8.3   0.523 +/- 0.044    0.339 +/- 0.009
  +0.50  +16.6   0.519 +/- 0.028    0.360 +/- 0.089
  +0.75  +25.0   0.565 +/- 0.011    0.314 +/- 0.008
```

α = −0.25 (−8.3 ms) is the minimum and sits temptingly close to BUG-031's +7.76 ms — but **every**
non-zero α is better than α = 0, in *both* directions, and all of them collapse the variance
(sd 0.489 → 0.01–0.05). A genuine phase error would give a one-sided minimum, not a notch at zero.
What actually distinguishes α = 0 is that it is the only point using a *raw* sample: any α ≠ 0
interpolates two neighbours, which low-passes the prior. So this measures the benefit of
**smoothing the rotation prior**, not its phase, and BUG-031's offset remains untested. Worth
pursuing on its own terms — a filtered prior is cheap and collapsed the instability here — but it
needs validation on real-motion captures before it goes anywhere near the shipped path.

---

## BUG-068 — the point-to-plane degeneracy guard can never fire

**Status:** fixed 2026-08-03 · **Area:** host/slam (`slam/odometry.py`) · **Found by:** instrumenting
the 3×3 normal equations while chasing BUG-067, 2026-08-02.

`_translation_icp` guards the translation solve with

```python
_COND_CEILING = 1e8
cond = np.linalg.cond(a)
if not np.isfinite(cond) or cond > _COND_CEILING:
    return t, fitness, rmse, True     # singular -> caller treats as tracking-lost
```

Its comment is right about the physics — a planar target makes `A = Σ nᵢnᵢᵀ` rank-deficient and
in-plane translation genuinely unrecoverable — but **1e8 is not a threshold any real frame can
reach**. Over 3018 consecutive frames of a room scan the worst conditioning observed is **203.5**
(median 7.8, p99 39.1). The guard fires on **0** frames; its margin is ~4.9e5× too loose. It is,
in practice, dead code.

**What it lets through.** At t ≈ 85–95 s of `captures/imuTranslationError.bin` — end of the third
pan, aimed at a close near-planar surface (mean range 1.25→1.70 m, depth sd 0.44 m) — conditioning
degrades to median 10.6 / p95 91 / max 203 against 2–9 elsewhere, and the estimate **slides 1.2 m
in 3 s (43 cm/s)** through a wall, preferentially along the weakest-observability axis (per-frame
|cos| 0.685 vs 0.549 baseline). Mean fitness across that window is 0.867 and no frame is reported
lost. Conditioning predicts the damage across the whole capture: mean step is 5.0 mm at cond < 5,
8.5 at 10–20, 14.0 at 20–50 and **31.0 at 50–200**.

**Why raising the gate's sensitivity is the wrong fix.** Rejection is terminal here: a rejected
frame is tracking-lost, `predict_pose` freezes translation at `t_prev`, and nothing relocalizes —
that is precisely BUG-036, where one rejected frame cost 423 frames (22%) of a circuit. Tightening
`_COND_CEILING` to ~50 would trade a slide for a dead run.

**Fix — cap the effective condition number instead of rejecting.** The eigen-decomposition of the
(symmetric PSD) normal equations is floored at `λ_max / cond_cap` before the solve, which shrinks
the correction along directions the geometry cannot observe while leaving observable directions
**exactly** untouched. Frames whose conditioning is already under the cap take the original
`np.linalg.solve` path and are bit-identical, so this is a no-op everywhere except the tail it
targets. The genuinely-singular path (non-finite, or a zero largest eigenvalue) still reports
`singular=True`.

**Choosing the cap — measured on three captures, not on the failing one.** Matched ensembles
(n=5, standard perturbations), max excursion; the tripod capture's truth is ~0, the other two are
real travel:

| `icp_cond_cap` | imuTranslationError | coffeeRoomCircuitNoMnt | roomSweepFull |
|---|---|---|---|
| 0 (pre-fix) | 0.752 ± 0.489 | 3.440 ± 0.072 | 3.398 ± 0.207 |
| 40 | 0.675 ± 0.327 | 3.389 ± 0.024 | 3.498 ± 0.110 |
| **20 (shipped)** | **0.459 ± 0.074** | **3.406 ± 0.066** | **3.373 ± 0.158** |
| 10 | 0.454 ± 0.017 | 3.369 ± 0.008 | **2.025 ± 0.371** |

The figure that matters is the tripod capture's **standard deviation**, not its mean: at cap 0 the
run is bistable (BUG-070) and capping collapses the spread 0.489 → 0.074, i.e. it removes the slide
rather than shifting an average. Real travel survives — both real-motion captures stay inside their
own ensemble spread — while their *path length* falls (28.9 → 21.6 m on the circuit) without max
excursion moving, which is jitter leaving rather than signal. **10 is deliberately not shipped**
despite scoring best on the tripod: it moves `roomSweepFull`'s reported displacement by 40%, i.e. it
starts eating real motion, and it damps ~40% of frames where 20 damps ~10% (cond p90 = 19.7).

Tracking-lost count is 0 at every cap on all three captures, and ICP escalations are 0 at every cap,
so the cap neither kills frames nor suppresses BUG-036's rescue path on real data.

**The cost, stated plainly.** From a single frame, genuine motion along a weakly observed axis and
an ICP slide are *the same measurement*, so the cap suppresses both. On the test suite's
cond-207 curved-plane fixture it recovers ~60% of a real in-plane shift (the normal-direction
component stays exact at every cap). That trade is taken deliberately because it is **asymmetric**:
an under-recovery is self-correcting, since frame-to-model re-aligns against the map absolutely on
the next frame, whereas an over-recovery is integrated into the TSDF and is permanent.
`test_weakly_observable_translation_is_damped_by_the_cap` pins that cost with numbers so retuning
`_COND_CAP` cannot hide it.

**An unexpected benefit.** The cap also prevents the *divergence* that used to present as a lost
frame: an in-plane displacement of 0.70 m made the unbounded solve overshoot to −0.76 and then find
zero correspondences, reporting tracking-lost. Bounded, the same frame registers. This changed
`test_escalating_rescues_a_frame_the_tight_radius_loses`, whose fixture had been exercising
escalation *via* that divergence — it now displaces along the plane normal, so its failure is a
genuine point-to-plane residual that no cap can mask.

**Not a fix for BUG-067 or BUG-070.** The tripod capture still reports 0.459 m of travel it never
made, and re-running BUG-070's heading sweep with the cap leaves the heading sensitivity intact
(0° 0.469 ± 0.079, 90° 2.107 ± 0.540). This bounds one failure mode; it does not make the
translation estimate trustworthy.

---

## BUG-069 — the stationarity gate cannot fire on a tripod, and could not help if it did

**Status:** open · **Area:** host/slam (`slam/motion.py`, `slam/mapper.py` `step`) · **Found by:**
BUG-067's tripod capture, 2026-08-02.

`StationarityGate` exists to stop the ICP translation estimate random-walking while the sensor sits
still — the owner's original "device is stationary → model should be too". It has two properties
that together make it useless for the case it is named after.

**1. It is structurally unable to fire during a pan.** The gate requires mean per-frame rotation
≤ `rot_ceiling_deg = 0.3`, on the documented reasoning that "during a real scan the user is almost
always rotating the sensor to aim at the scene, so any appreciable rotation means *actively
scanning, not still*". A tripod pan is the counterexample that assumption excludes by construction:
rotation with **zero** translation. The capture's three pans run at 25–31 °/s ≈ 0.8–1.0 °/frame,
3× over the ceiling, so the gate is off for exactly the 43% of the capture that produces
essentially all of the drift.

**2. It is display-only, so it could not protect the map anyway.** `Mapper.step` applies a True
verdict to `report_pose` alone; `self._t_prev` and the TSDF integration always use the raw ICP
pose. That is deliberate and documented ("a false hold can never corrupt the reconstruction"), and
it is the right call for a gate that can misfire — but it means the *reconstruction* has no
zero-velocity constraint at all, only the preview does.

Fixing this is not a threshold change. It needs a discriminator that separates "rotating in place"
from "rotating while walking" — the coherence test was meant to be it, and `rot_ceiling_deg` was
added precisely because coherence alone misfired on a scan's curved path. Doing it properly means
earning the right to let a hold reach the map, which is a bigger change than relaxing a constant.

---

## BUG-070 — reported drift changes by 2 m under a physically null change of compass heading

**Status:** anomaly (reproducible, measured, not root-caused) · **Area:** host/slam · **Found by:**
isolating why Live SLAM and Detailed disagree on BUG-067's capture, 2026-08-02.

Live SLAM feeds `sensor.fused_quat()` (SFLP with a magnetometer heading graft, +56.8° on this
capture); Detailed feeds the raw SFLP quat via `_load_frames`. Live reads ~4× worse. The obvious
explanation — that the mag correction *wanders*, which it does, ±2–3° over the run — **is wrong**:
a **frozen constant** graft is just as bad (1.81 ± 0.77 m vs the live 1.69 ± 0.52 m).

`graft_yaw` is a pure heading change; verified numerically that boresight tilt and the CV-world
up-component are preserved to machine precision under grafts of 45/90/180°, and the
`T_WORLD_TO_CV @ R @ T_CV_TO_BODY` sandwich carries it correctly. So a constant graft relabels
compass directions and changes no physics. Yet, sweeping it (n=4 each, max excursion):

```
  0° 0.81 ± 0.53    15° 1.28 ± 0.89    30° 0.81 ± 0.32    45° 1.42 ± 0.61
 57° 1.59 ± 0.70    60° 1.37 ± 0.62    75° 2.21 ± 0.62    90° 2.89 ± 0.14   180° 2.84 ± 0.17
```

**What it is not.** Not the voxel lattice: a 90° rotation about world up maps the cubic lattice onto
itself, so 0/90/180 would agree, and they do not. Not the barometer: `baro_authority=0` gives 2.77
at 90° vs 3.02 with it on. Not a tracking failure: 0 lost frames, mean fitness 0.882–0.885 at both
0° and 90°.

**What it looks like.** The 0° and 90° runs track *identically* (distance-from-origin within 0.1 m)
until t ≈ 88 s, then 90° slips 0.83 → 2.98 m across the BUG-068 window while 0° does not. So heading
is not a systematic cause — it biases which side of a **bistable** marginal event the run lands on.
That a null relabelling can decide a 2 m outcome is the honest measure of how unstable this
translation estimate is (BUG-067), and it means **any single-run comparison across captures with
different headings is confounded** — a sharper version of the "score ensembles, not single runs"
rule from BUG-037.

**BUG-068 did NOT fix it** — the prediction first written here was wrong, and is corrected rather
than deleted. Re-running the sweep with the shipped conditioning cap (n=4, max excursion):

```
   heading      cap 0 (pre-fix)     cap 20 (shipped)
       0°      0.814 +/- 0.529      0.469 +/- 0.079
      45°      1.421 +/- 0.605      1.454 +/- 0.706
      57°      1.591 +/- 0.699      0.926 +/- 0.417
      90°      2.887 +/- 0.141      2.107 +/- 0.540
     180°      2.844 +/- 0.170      2.409 +/- 0.605
```

The cap stabilises the 0° case (spread 0.529 → 0.079) and does nothing for the heading dependence
itself: 45° is unchanged and 90° is still 4.5× the 0° result. So this is a **second, independent**
defect, not a downstream symptom of BUG-068. Still not root-caused; recorded so a future session
does not have to rediscover it, and so nobody assumes the conditioning fix covered it.

---

## BUG-071 — `resolve_command()` never wired up `"standby"`, so `rig_command`/generic `cmd` can't send it

**Status:** open, not fixed (found incidentally, out of scope for the pass that found it) ·
**Area:** host/web · **Found by:** trying to exercise `SET_STANDBY` via `rig_command(name="standby",
...)` while verifying the 2026-08-03 auto-idle work, 2026-08-03.

`resolve_command()` (`web.py`, backs the generic inbound `{"type": "cmd", "name": ...}` message) maps
`ping`/`calib`/`reinit`/`usecase`/`period`/`exposure` to their `CommandCode`s and returns `None` —
"unknown/invalid cmd request" — for anything else. `"standby"` was never added, even though the wire
protocol fully supports it (`CommandCode.SET_STANDBY`, `RS_CMD_SET_STANDBY`) and the CLI tool
(`roomscan-ctl standby <0|1|2>`, `host/src/roomscan/control.py`) already parses it correctly
(`test_control_cli_parses_standby`). The web app's own `SET_STANDBY` traffic goes entirely through
the dedicated `set_idle` message → `_dispatch_standby()`, which never routes through
`resolve_command()`, so this gap was invisible until something tried to send `standby` as a generic
`cmd` — which is exactly what the `rig_command` MCP tool's own docstring claims is supported
(`"name is one of ping, calib, reinit, usecase, period, exposure, standby"`).

**Not fixed**: found while validating an unrelated feature (auto-idle activity accounting), and
`rig_set`/`set_idle`/the auto-idle machinery are all unaffected — this only blocks a *manual* generic
`cmd` standby request over `/ws`, e.g. via `rig_command`. Fix is a one-line addition to
`resolve_command()` (`if name == "standby": return CommandCode.SET_STANDBY, int(param), f"standby
{int(param)}"`), but wasn't made without a clearer picture of why the app deliberately built a
separate `set_idle` path instead of just using generic `cmd` here in the first place — worth checking
that history before adding the case, in case there was a reason (e.g. wanting persisted
enabled/level state that a bare `cmd` round-trip doesn't carry).

## BUG-072 — standby-shadow desync: a failed `vl53l9_stop()` recovery left the loop parked forever

**Status:** fixed (2026-08-03, Task 5 of the high-framerate plan) · **Area:** firmware/scanner-stream ·
**Found by:** the "one-off first-command-after-flash timeout" flagged in Task 4's hardware pass, which
reproduced and traced during Task 5's autonomous-sync work.

If a profile-apply's `vl53l9_stop()` failed because the sensor was already standby-parked (e.g. the web
server's own auto-idle had fired), the recovery path reinitialized the sensor to genuinely STREAMING but
never resynced `rs_standby_level` — the main loop then stayed in its idle branch forever: RAW/CALIB/
stream-13 stopped while IMU/env kept flowing at ~18 Hz and ACKs kept reporting OK, so every health
signal looked alive. Fixed at both call sites (profile-apply stop-failure path and `RS_CMD_REINIT`).
Sibling fix in the same pass: under autonomous sync, `vl53l9_stop()` returning success only means the
stop command was *accepted* — the FSM can still read STREAMING for a few ms, failing the next
STANDBY-gated call on both the candidate write *and* its restore, so no ACK was ever sent (6/6 repro).
Fixed with `rs_wait_standby()`, a bounded poll of `vl53l9_get_status().fsm` chained onto every stop.
After both fixes: 12/12 stress switches + 4/4 live 90↔30 switches clean.

## BUG-073 — rapid reconfiguration near the rate ceiling can drop the ACK entirely (no ACK, no BUSY)

**Status:** open · **Area:** firmware/scanner-stream · **Found by:** the 2026-08-03 frame-rate-ceiling
investigation, sweeping 90→92→95→97→99→100 Hz manual params back-to-back with ~1 s settle.

Three consecutive `SET_MANUAL_PARAMS` requests (92/95/97 Hz) got neither an ACK nor a BUSY; the next
(99 Hz) ACKed after 4.7 s with bimodal interval stats. A controlled retest of the same values from a
settled baseline (2 s settle) succeeded every time, so the trigger is reconfig-before-settle, not the
values. Points at `rs_apply_pending_config`'s documented `handle_error(); return true;` no-ACK path
firing when a reconfig lands before the previous one has settled. A host retry recovers, but the
contract says every accepted command produces exactly one ACK/BUSY. Recheck after the ceiling
amendment (the shipped preset moves out of the 90–100 Hz band, but manual requests can still go there).

## BUG-074 — SET_STANDBY wake path likely missing the BUG-072 standby-shadow repair

**Status:** open, unconfirmed · **Area:** firmware/scanner-stream · **Found by:** same investigation —
the ToF stream was found wedged at 0 Hz at session start (IMU/env still free-running ~18.2 Hz, PING
fine) and `SET_STANDBY(wake=0)` timed out 3× (up to 20 s) while `REINIT` fixed it immediately.

BUG-072 repaired the `rs_standby_level` shadow on the profile-apply stop-failure and REINIT paths, but
the SET_STANDBY wake path appears not to have the same repair — matching the observed "wake times out,
REINIT recovers" signature. Check and fix alongside Task 7's work in `vl53l9_app.c` (same file), with a
wake-after-desync hardware test.
