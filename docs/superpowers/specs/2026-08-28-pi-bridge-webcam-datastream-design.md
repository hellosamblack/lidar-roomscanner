# Pi-bridge webcam datastream, truthful recording, and posed ingest

**Status:** Specced (three gated issues open; no implementation)
**Date:** 2026-08-28
**Issues:** #226 (characterize + live preview), #227 (capture bundles), #228
(calibrated posed/ToF ingest)
**Related:** #191 (Pi bridge), #200 (Pi reliability), #217 (bounded host queues),
#132/#146/#159 (offline posed-capture program and its gates)

This replaces the reviewed external draft
`pi_bridge_webcam_datastream_plan.md`. The useful decision from that draft survives:
bulk camera pixels use a dedicated HTTP MJPEG path, never the ordered `/ws` stream. The
claims that did not survive review -- high-precision synchronization from arrival times,
hard-coded nominal intrinsics, direct JPEG input to `imageio_ffmpeg.write_frames`, and
"paired files" as a substitute for posed/ToF ingest -- are explicitly excluded below.

## 1. Decision summary

The work is three deliverables with hard gates between them:

| Stage | Issue | Delivers | Must not claim |
| --- | --- | --- | --- |
| A | #226 | Measured C270/Pi limits, bounded Pi daemon, one host ingest, live browser monitor | synchronized recording, calibrated camera, or splat fusion |
| B | #227 | Atomic scanner+camera bundles, original JPEG preservation, named clock domains, variable-cadence preview MP4 | camera/ToF alignment or metric RGB-D |
| C | #228 | Measured intrinsics/distortion, hand-eye extrinsic, validated clock alignment, canonical `PosedCapture`, metric ToF depth | success unless it beats the simpler RTAB-Map/ARCore + ML-depth route |

Stage B is blocked until Stage A's coexistence benchmark passes. Stage C is blocked on
Stage B, the rigid-mount/seed-set work in #146, and the RTAB-Map posed-input measurement in
#159. If #159 and posed+ML-depth already reach the offline program's quality gate, Stage C
is killed rather than built.

## 2. Observed baseline, not an assumed one

A read-only probe on 2026-08-28 found:

- Raspberry Pi 3 Model B Rev 1.2, connected over `wlan0` at 72.2 Mbps and −46 dBm;
- Logitech C270, USB VID:PID `046d:0825`, currently enumerated as `/dev/video0` and
  `/dev/video1`;
- capture index 0 currently negotiated as MJPEG 1280×720 at 30 fps;
- `exposure_dynamic_framerate=1`, automatic exposure and automatic white balance;
- the C270 and the SMSC9512/9514 Ethernet adapter share the Pi's USB2 tree;
- Pi temperature 60.1 °C, no throttling, NTP synchronized at the time of the probe.

This proves that 720p30 MJPEG is a real advertised/current mode on this unit. It does not
prove frame size, sustained cadence in low light, CPU cost, end-to-end latency, shared-bus
headroom, or scanner non-interference. Those are Stage A measurements. `/dev/video0` is an
observation, not an identity: selection uses USB identity + capture capability, preferring
the stable `video-index0` path for the physical port.

## 3. Product scope

### Goals

- Show the rig-mounted camera live in `roomscan-web` without adding an unbounded queue or
  touching the primary `/ws` payload order.
- Preserve exactly what the camera produced, together with enough timing evidence to say
  what is known and what remains uncertain.
- Make record, auto-record, rename, delete, restart, and crash recovery operate on a whole
  capture bundle.
- Provide a later, separately gated path from a calibrated bundle to the existing
  `PosedCapture` contract and the offline 3DGS benchmark arms.

### Non-goals

- No camera bytes on `/ws`, `/ws-mesh`, or `/ws-thin`.
- No H.264 encode on the Pi.
- No claim that host receive time is camera exposure time.
- No nominal product FOV presented as calibration.
- No direct camera feed into real-time SLAM in Stages A or B.
- No replacement of the owner-ordered RTAB-Map/ARCore posed-capture evaluation.
- No browser operating-system Picture-in-Picture promise for an `<img>` element. Stage A
  provides an in-app expanded/floating view; native PiP requires a later `<video>`-based
  product decision.

## 4. Cross-cutting invariants

1. **Latest wins for presentation.** A slow viewer skips frames; it never causes a queue
   of historical JPEGs.
2. **Recording never backpressures the scanner.** The camera writer has a bounded byte
   queue and reports every overflow. Scanner `.bin` recording continues if the camera
   disappears or its writer fails.
3. **One capture per layer.** One V4L2 loop on the Pi, one camera ingest loop on the host,
   any number of latest-only consumers. Browser count must not multiply Pi camera opens.
4. **Requested is not actual.** Resolution, cadence, controls, timestamp flags, service
   state, queue drops, and artifact state come from readback/observation.
5. **Clock domains stay typed and named.** Pi monotonic, host monotonic, wall time, and
   STM32 TIM2 are never subtracted without an explicit fitted transform. Add domain
   wrappers alongside `HostMono`, `WallTime`, and `McuMicros`; do not pass bare floats
   across the new APIs.
6. **The far side decides.** Stream health is measured in browser pixels and simultaneous
   scanner arrivals, not inferred from a successful setter, HTTP response, or server send
   counter.
7. **A bundle commits last.** A final manifest is the commit marker. Missing or partial
   artifacts remain visible as partial/recoverable, never silently read as complete.
8. **No new root network daemon.** The Pi service is unprivileged, device-scoped, and
   reachable only on the intended bridge-facing network surface.

## 5. Stage A — characterize and serve a bounded live feed (#226)

### 5.1 Characterization gate

Run the real C270 with the scanner simultaneously streaming. Capture one shared time window
for camera-off baseline, preview-on, and preview+snapshot load. For each arm record:

- negotiated V4L2 format, interval, buffer timestamp flags/domain, and camera controls;
- achieved frame cadence and frame-size median/p95/p99/max;
- USB and wlan0 bytes/s, Pi process/system CPU, RSSI/bitrate, temperature, throttling;
- scanner device/host FPS, fragment loss/reorder/incomplete counts, `/ws` backlog, and
  broadcaster `tick_share`;
- camera host-arrival age and a far-side browser-pixel latency probe.

The far-side latency stimulus is a host-timed black/white fiducial shown on a monitor in the
camera view. A browser probe samples the displayed `<img>` through a same-origin canvas and
detects the transition. A server frame counter or successful MJPEG write is not a substitute.

Acceptance: no new scanner frame loss, no monotonic queue/latency growth over five minutes,
no material scanner/pose cadence regression in the same window, target camera pixel-latency
p95 ≤150 ms and hard gate ≤250 ms on the home WLAN, and no Pi throttling. If 720p30 fails,
the implementation records the result and selects the highest real mode that passes; it does
not silently fall back while continuing to advertise 720p30.

### 5.2 Pi capture daemon

`roomscan-cam` owns one V4L2 MMAP capture loop. Device resolution is:

1. configured stable by-path override, if present;
2. USB VID:PID `046d:0825` + Video Capture/Streaming capability + `video-index0`;
3. offline with a diagnostic listing the candidates -- never an arbitrary first `/dev/video*`.

Each dequeued buffer becomes one immutable latest-frame record:

```text
CameraFrame {
  generation            # increments on reopen/reconnect
  seq                   # monotonic within a generation
  jpeg                   # exact DQBUF bytesused bytes
  pi_capture_mono_ns     # V4L2 buffer timestamp when MONOTONIC is asserted
  timestamp_valid
  timestamp_flags
  width, height, fourcc
}
```

If the driver does not assert a usable timestamp domain, `timestamp_valid=false`; dequeue
time is recorded separately for diagnostics and is never renamed capture time. Frame-size
and JPEG SOI/EOI limits are checked before publication. Requeue happens promptly; network
clients never hold V4L2 buffers.

A condition-variable latest slot fans out frames. Each HTTP client retains only its last
sent sequence and at most one immutable JPEG reference. A blocked socket therefore stalls
only that client thread; after it drains, it sends the newest frame and skips intermediates.
There is no per-client frame deque.

Endpoints:

- `GET`/`HEAD /stream.mjpg`
- `GET`/`HEAD /frame.jpg`
- `GET`/`HEAD /status.json`
- `GET /clock.json`

Multipart framing is exactly:

```http
Content-Type: multipart/x-mixed-replace; boundary=frame

--frame\r\n
Content-Type: image/jpeg\r\n
Content-Length: N\r\n
X-Roomscan-Camera-Generation: G\r\n
X-Roomscan-Camera-Seq: S\r\n
X-Roomscan-Capture-Mono-Ns: T\r\n
X-Roomscan-Timestamp-Domain: pi_monotonic\r\n
\r\n
<N JPEG bytes>\r\n
```

`/clock.json` returns Pi receive/send monotonic and realtime nanoseconds for a host
four-timestamp exchange. `/status.json` reports actual device identity/mode/controls,
capture and sent rates, generation, malformed/dropped buffers, connected clients, restart
count, and last error. It distinguishes `device_absent`, `opening`, `streaming`, and
`faulted`.

### 5.3 Pi service and bridge integration

The service runs as a dedicated unprivileged user with `video` device access, no-new-
privileges, a read-only filesystem except its runtime directory, a bounded open-file/thread
budget, and systemd device/network hardening. It has a five-in-five-minutes start limit plus
`Restart=on-failure`; the daemon itself handles ordinary unplug/replug without exiting.

The installer treats the unit as optional when no C270 is present, verifies that a started
unit remains active, and reports actual state. The reconcile loop restarts it only when the
stable camera identity is present. Bridge status and `bridge_status()` add camera fields and
warnings without turning a deliberately absent optional camera into a bridge failure.

Port 5001 is accepted on `wlan0`, not the scanner-facing `eth0`; deployment documents that
the roomscan LAN is the trust boundary, matching the currently unauthenticated web UI. The
service is not published to the internet. Host discovery reuses the existing mDNS-first
bridge resolver and never persists the DHCP address observed during development.

### 5.4 Host ingest and proxy

`roomscan.camera` contains the reusable implementation. `CameraClient` is started and
stopped by FastAPI lifespan, not opportunistically by the first route or session message.
It maintains one persistent Pi connection, parses multipart input incrementally across
arbitrary TCP chunk boundaries, validates content lengths/limits, and reconnects with
bounded exponential backoff. Teardown closes the socket, cancels/joins its worker, and
leaves no process-global liveness flag behind.

The client publishes into one host latest-frame slot. HTTP routes are:

- `/api/camera/stream.mjpg` -- latest-only proxy;
- `/api/camera/frame.jpg` -- one current frame plus generation/sequence headers;
- `/api/camera/status` -- host ingest + Pi-reported status.

Each proxy generator awaits completion of the previous ASGI send before selecting the next
latest frame. Uvicorn transport-buffer depth is sampled in the live probe; a slow client
must not make it grow without bound. Camera parsing and status polling do not share the
scanner broadcaster's event-loop tick.

A low-rate `camera` WebSocket message carries state, not pixels:

```text
camera {
  state, bridge_addr, generation, seq,
  width, height, fourcc, capture_fps, host_fps,
  capture_timestamp_valid, age_ms,
  source_drops, ingest_drops, clients,
  recording {state, writer_drops, last_error}
}
```

The cadence is human-rate (≤4 Hz). Null means unknown; zero means measured zero.

### 5.5 Browser monitor

The new card follows the existing dock/card contract and design language. It uses one
`<img>` pointed at the same-origin proxy. Controls are Freeze, Snapshot, and in-app Expand.
Freeze keeps the last decoded bitmap without pretending the camera stopped. Snapshot uses
the host's latest exact JPEG. Expand floats within the app; it is not native video PiP.

The card displays actual negotiated resolution/FPS and distinct Offline, Connecting, Live,
Stale, and Recording/Partial states. A reconnect changes a cache-busted stream URL only on a
camera generation/state transition, not on every telemetry tick. The card's hidden/collapsed
state must not create an extra Pi or host ingest connection.

### 5.6 Agent surface

Pure functions under the camera implementation return structured status and snapshot
results. Thin MCP wrappers expose `camera_status` and `camera_snapshot`; the snapshot path
is resolved under an allowed output root and reports the path actually written. A CLI is
added only if the owner needs a direct human workflow. MCP registry and docs move together.

## 6. Stage B — atomic bundles with truthful timing (#227)

### 6.1 Authoritative artifacts

The camera's original JPEGs, not a second-generation H.264 transcode, are the source of
truth. One completed take uses a common stem:

```text
captures/<stem>.bin                 raw scanner protocol frames
captures/<stem>.cam.mjpg            concatenated original JPEG packets
captures/<stem>.cam.frames.jsonl    one crash-tolerant frame index record per JPEG
captures/<stem>.cam.clock.jsonl     raw Pi↔host clock exchanges
captures/<stem>.scanner.clock.jsonl scanner TIM2↔host-arrival observations
captures/<stem>.mp4                 optional derived browser preview
captures/<stem>.capture.json        versioned bundle manifest; commit marker, written last
```

Active members use `.part` suffixes. On normal stop, writers close and fsync, members are
renamed, hashes/sizes are read back, and the manifest is atomically written last. On restart,
an inspector can recover complete JPEG records by offset/length/CRC and reports remaining
files as a partial bundle. It never fabricates a final manifest.

The JSONL frame record contains at least:

```json
{
  "schema": 1,
  "frame_idx": 42,
  "camera_generation": 3,
  "camera_seq": 918,
  "jpeg_offset": 1234567,
  "jpeg_bytes": 48219,
  "jpeg_crc32": 123456789,
  "pi_capture_mono_ns": 123,
  "capture_clock": "pi_monotonic",
  "timestamp_valid": true,
  "host_receive_mono_ns": 456,
  "receive_clock": "host_monotonic"
}
```

Camera sequence is the pair `(generation, seq)`; reconnects never create duplicate identity.
The manifest records actual camera mode and controls for every generation, artifact state,
frame/duration/drop counts, clock-fit summaries and uncertainty, member identities/hashes,
and whether the optional MP4 preview succeeded.

### 6.2 Clock evidence, not a synchronization claim

The host periodically performs four-timestamp exchanges against `/clock.json` and records
the raw samples. A robust affine fit maps Pi monotonic to host monotonic using low-RTT
samples and reports slope, offset, residuals, RTT distribution, validity interval, and an
uncertainty bound. Every accepted scanner ToF group also records its `McuMicros` FRAME_READY
stamp and host monotonic receive instant in the scanner clock sidecar.

Stage B may say "Pi clock mapped to host with X uncertainty." It may not say "camera aligned
to ToF." Wi-Fi/TCP/USB pipeline delays and the semantic point represented by the C270's V4L2
timestamp remain Stage C validation inputs.

### 6.3 Recording behavior

Starting a take creates one bundle coordinator. Scanner recording remains primary. The
camera writer has a bounded byte queue; because it only appends source JPEGs and JSONL, it
should outrun the measured stream. If it does not, it drops the incoming camera frame,
increments `writer_drops`, and leaves the sequence gap visible. It never blocks the scanner
reader or FastAPI event loop.

States are explicit:

- `scanner_only` -- camera absent at start;
- `recording` -- both writers healthy;
- `camera_partial` -- scanner healthy, camera missing/failed/gapped;
- `finalizing` -- writers closed, preview/manifest work running off-loop;
- `complete` -- manifest committed with all required members valid;
- `failed` -- no valid scanner capture or unrecoverable bundle error.

Camera failure does not close the `.bin`. Reconnection creates a new camera generation in
the same partial bundle. Stop and shutdown have bounded waits; an uncooperative child process
is terminated and reported, not waited on forever.

The optional `<stem>.mp4` is generated after stop from the authoritative JPEG/index pair.
Frame durations come from mapped capture deltas, not `fps=30`; H.264/yuv420p/CRF 18 is a
preview policy, not the photogrammetry source. `ffprobe` must read back PTS/duration and the
finalizer must compare them with the index. Preview failure leaves a usable committed raw
bundle with `preview.state="failed"`.

### 6.4 Whole-bundle lifecycle

`SessionController` delegates all bundle mutations to one capture-bundle implementation:

- manual and Live-SLAM auto-record use the same start/stop path;
- post-stop rename renames every member and patches the manifest;
- capture-browser rename/delete operate on the bundle, including partials and previews;
- collision checks consider every destination member before moving any;
- delete reports each removed/refused member and actual bytes freed;
- active bundles cannot be renamed/deleted;
- the source-video list shows only a valid preview or an explicitly supported raw bundle,
  never an orphan `.mp4` guessed by matching stems.

Finalization and ffmpeg/inspection waits run off the event loop. The session message reports
bundle state and camera partiality so a red recording badge cannot imply healthy dual capture
when only the scanner is being written.

## 7. Stage C — calibrated posed/ToF ingest (#228)

### 7.1 Pre-gates and kill criterion

Run #159's RTAB-Map posed-only comparison first, preserving the owner decision in the current
offline 3DGS program. Continue only if a rigid C270 path has measured product value after that
comparison. Kill this stage if posed RTAB-Map/ARCore frames plus the implemented ML-depth prior
already reach the existing Scaniverse-class gate on two rooms.

### 7.2 Camera calibration

Calibrate the physical unit and every accepted resolution with a checkerboard/ChArUco capture.
Store serial/device identity, image size, K, distortion model/coefficients, calibration-date
controls, reprojection distribution, and held-out validation results. The product FOV may seed
an optimizer only. For reference, 58.5° at width 1280 implies `fx≈1142 px`, not 1350 px;
neither value is accepted without measurement.

The posed trainer consumes undistorted pinhole images and adjusted K, or a path that explicitly
supports the chosen distortion model. A silent distorted-image/pinhole-K pairing is forbidden.
Record/lock exposure and white balance when the measured capture protocol shows that doing so
improves feature/photometry stability; actual controls remain in the bundle either way.

### 7.3 Hand-eye extrinsic and frames

Define `lidar_T_camera` as camera-optical coordinates transformed into ToF-CV coordinates.
Then:

```text
world_T_camera = world_T_lidar @ lidar_T_camera
camera_T_world = inverse(world_T_camera)   # PosedFrame.pose_camera_from_world
```

Both camera and ToF optical frames use x-right, y-down, z-forward before the mount transform.
The calibration must pass a held-out target/capture test that detects inverse direction, axis
swap, sign flip, and a plausible-looking but wrong translation. Norm preservation is not an
acceptance test.

### 7.4 Temporal alignment

Compose the recorded Pi→host and TIM2→host models only with their uncertainties intact. Refine
and independently validate relative offset using an invariant-to-extrinsic motion observable,
such as angular-speed magnitude from camera optical flow versus the rig IMU/trajectory (the
existing RTAB-Map comparison uses this shape). Account for exposure duration/rolling shutter and
determine whether the V4L2 timestamp denotes exposure start, midpoint, end, or only buffer delivery.

The acceptance motion requires p95 temporal residual ≤5 ms, or measured projected spatial error
below one ToF zone at the recorded angular/translation speed. Otherwise the metric-fusion arm is
killed or explicitly downgraded; it is never called high precision because a file contains many
decimal places.

### 7.5 Canonical ingest

For each valid camera exposure:

1. map its capture time into TIM2 with uncertainty;
2. interpolate `world_T_lidar` using linear translation + quaternion SLERP;
3. reject frames spanning a SLAM discontinuity/tracking-loss interval;
4. apply the hand-eye transform;
5. undistort RGB and adjust K;
6. project confidence-filtered ToF samples into the RGB image with a z-buffer and an explicit
   sparse-depth mask;
7. emit `PosedFrame(image_path, K, camera_T_world, depth, confidence, timestamp, domain)`.

The output is a `PosedCapture`, not a special-case sidecar reader hidden inside the trainer.
The output manifest fingerprints every source member plus calibration, extrinsic, clock model,
trajectory, projection policy, and reconstruction preset.

The build UI and MCP surface offer four arms on the same accepted frames:

1. COLMAP-only;
2. posed-only;
3. posed + ML depth;
4. posed + metric ToF depth.

Report frame use/registration, fragmentation, opacity/scale statistics, and `splat_compare`
geometry error. The metric arm ships only if it materially beats the simplest surviving arm on
two rooms. A source-video-list badge is not end-to-end integration.

## 8. Failure table

| Failure | Required behavior |
| --- | --- |
| Camera absent at boot | bridge/scanner remain healthy; camera reports `device_absent` |
| Camera unplugged live | current MJPEG response ends/reconnects; host state becomes Offline/Stale; no queue survives |
| Camera reconnects | generation increments; live view resumes; active bundle records a visible gap |
| Slow Pi HTTP client | only that client blocks; it resumes at newest frame |
| Slow host proxy client | ASGI send applies bounded transport flow control; next selected frame is newest |
| Camera writer full | drop and count camera frame; scanner writer continues |
| Disk full | close writers, report actual partial artifacts; never commit a complete manifest |
| MP4 finalizer fails | raw camera capture remains authoritative; preview marked failed |
| Server shuts down | bounded stop/join/child termination; partial bundle recoverable |
| Pi clock unsynchronized to wall time | monotonic recording continues; wall time marked unavailable/uncertain |
| Timestamp flag absent | capture timestamp is null/invalid; arrival time remains separately named |
| Calibration missing/wrong mode | Stage C ingest refuses metric/posed claim; raw bundle remains usable |
| Tracking lost at camera time | omit/flag posed frame; never interpolate through the discontinuity |

## 9. Verification matrix

### Automated

- Multipart parser: boundaries split at every byte position, malformed/missing lengths, oversized
  frame, reconnect, sequence reset, and cancellation.
- Latest slots: stalled subscribers, no per-client queue, generation changes, bounded memory.
- Lifespan: repeated app start/stop, socket failure, worker self-cleanup, no leaked task/thread/process.
- Bundle: normal finalize, camera absent/partial, queue overflow, rename collision, delete, crash
  recovery, disk full, malformed JPEG, and manifest-last semantics.
- Timing: typed-domain rejection, affine clock fixtures with drift/jitter/outliers, uncertainty
  propagation, variable cadence, and MP4 PTS readback.
- Calibration/ingest: held-out calibration fixture, transform inverse/sign negative cases, time-
  offset negative cases, tracking discontinuity, occlusion/z-buffer projection, `PosedFrame`
  validation, and manifest fingerprints.
- MCP registry/docstring coverage and web-protocol/static UI tests.

### Live Stage A

- Real Pi/C270 mode/control/timestamp characterization.
- Five-minute camera-off / preview-on simultaneous scanner comparison over identical windows.
- One normal and one deliberately stalled browser client.
- Browser-pixel latency stimulus and screenshot/state verification.
- Camera daemon kill/restart plus one physical unplug/replug close gate.

### Live Stage B

- Five-minute dual capture with actual frame/PTS inspection.
- Camera unplug/replug mid-take; scanner capture remains valid and camera is partial.
- Server stop during a take followed by recovery inspection.
- Rename and delete through the real web UI, verifying no orphan members.

### Live Stage C

- Held-out intrinsic and hand-eye validation.
- Independent motion sequence for clock-offset/drift validation.
- Two-room four-arm splat benchmark against the current manifest and `splat_compare` gates.

Any physical action, owner visual judgement, or capture that this session cannot perform routes
through `operator-request`; code completion alone does not close an issue.

## 10. Expected implementation surface

Stage A is expected to touch:

- `pi-bridge/payload/usr/local/sbin/roomscan-cam`
- `pi-bridge/payload/etc/systemd/system/roomscan-cam.service`
- Pi installer/status/reconcile/firewall and their tests
- `host/src/roomscan/camera.py`
- server lifespan/routes/message builder, web camera component/card/layout
- camera MCP tools, registry tests, and protocol/architecture/runbook/MCP docs

Stage B is expected to add a capture-bundle module and update `SessionController`, capture-browser
rename/delete/listing, auto-record, session state, and lifecycle tests. It must coordinate with any
active session touching `host/src/roomscan/server/session.py` before editing.

Stage C is expected to add calibration/timing/rig-bundle ingest under `roomscan.splat`, reuse the
canonical coordinate-frame and `PosedCapture` contracts, expose one pure ingest implementation via a
thin MCP wrapper, and extend the current offline benchmark rather than creating a parallel trainer.

## 11. Documentation and landing

Each stage updates its governing issue and only the documents its implementation changes. At minimum:

- Stage A: `docs/web-protocol.md`, `docs/system-architecture.md`, `docs/pi-bridge-runbook.md`,
  `docs/mcp-server.md`;
- Stage B: web protocol, capture-browser/recording documentation, and this bundle schema;
- Stage C: `docs/coordinate-frames.md`, offline 3DGS program status, MCP docs, and calibration
  procedure/artifacts.

`status-sync` moves issue/doc status with code. `operator-request` decides close-or-hold before every
closing commit. This design remains **Specced** until Stage A's real characterization gate passes;
writing the daemon first and measuring later is explicitly out of order.
