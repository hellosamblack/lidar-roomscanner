# Rerun observability sidecar

## Status

**Proposed.** This is an additive host-only diagnostic capability. It does not
replace the production visualizer or mapper. Implementation needs a separate,
reviewed plan after the API/version probe in §10 succeeds.

## 1. Goal

Make a scanner session inspectable as a time-aligned, multimodal Rerun
recording: transformed ToF products, IMU/environment data, host fusion state,
SLAM poses and health, and rate-limited map snapshots. The purpose is to find
*when* a capture or reconstruction went wrong and compare the raw measurement,
intermediate products, and decisions at that instant.

Rerun is an observability sidecar. The production path remains:

```text
STM32 -> RSCN frames -> decoder -> TransformStage -> web + Open3D SLAM
                         |                         |
                         +-> canonical .bin        +-> Rerun sidecar
```

The RSCN `.bin` is the authoritative acquisition record. An `.rrd` is a
derived, expendable diagnostic artifact; it never becomes an input to firmware,
the decoder, transform, mapping, or capture playback.

## 2. Scope and non-goals

**In scope:** a disabled-by-default, version-pinned `rerun-sdk`; bounded live
logging; offline capture-derived export; a stable entity/timeline schema;
provenance manifests; and a reproducible performance/usefulness gate.

**Out of scope:** replacing Open3D tensor ICP/TSDF, `SlamRunner`, `MeshPrep`,
`.ply` export, `roomscan-web`, device controls, MESH credit transport, or MCP
tools. This adds no firmware, wire-protocol, or second-source-reader change.
It also does not log `RAW_3DMD` or `CALIB` into Rerun: the canonical capture
already preserves those exact bytes and their replay contract.

## 3. Required invariants

1. **The reader remains sole source owner.** The sidecar receives immutable,
   already-decoded snapshots from existing fan-out hooks. It never opens USB,
   UDP, a capture file, or a second `StreamDecoder`.
2. **Fail open.** A missing SDK, logger exception, viewer/server disconnect,
   full disk, or queue overload disables only the sidecar and emits a local
   event/metric. It must not stop recording, transformation, SLAM, web
   broadcast, or command handling.
3. **Off the hot path.** The reader copies bounded NumPy snapshots into a
   bounded queue. Serialization, file writes, compression, and connection work
   run in one dedicated worker. A full queue drops the sidecar sample and counts
   it; it never delays or drops a scanner frame.
4. **Explicit time.** Every dynamic log carries `device_time` from
   `FrameHeader.t_us` and `frame_seq`. Wall time is metadata only; it must not
   be used for scanner duration or combined with the monotonic device clock.
5. **One coordinate convention.** Dynamic poses use the already-verified
   `coordinate-frames.md` conversion. No new Open3D, CV, Three.js, or Rerun
   convention is invented.
6. **Bounded live operation.** A live session has duration, byte, queue, and
   mesh-cadence limits. Exceeding one stops only sidecar logging with a recorded
   reason. Offline export has no live queue but still has bounded mesh history.

## 4. Modes and artifacts

### Live diagnostic session

`roomscan-web` creates a `RerunSidecar` only when an explicit developer-facing
option enables it. It writes `results/rerun/live_<UTC>.rrd` plus
`live_<UTC>.rerun.json`. The live profile is latest-wins and bounded: ToF
frames are sampled at configured cadence; sensor/scalar state is attached to
the nearest preceding ToF time; mesh snapshots are rate- and byte-limited.

Starting/stopping it changes no capture state. An operator still uses the
existing Record control for a canonical raw session. If both run, the Rerun
manifest names the raw capture after it stops.

### Offline capture export

`roomscan-rerun-export CAPTURE.bin` replays through the existing
`FileSource -> StreamDecoder -> TransformStage` path and writes
`results/rerun/<capture-stem>.rrd` plus a neighboring manifest. It emits every
replayable transformed ToF sample, but does not claim to contain raw firmware
bytes. SLAM is opt-in; default export shows the recorded sensor/transform path
without rebuilding a map.

Every manifest records source path, size, mtime, protocol version(s), transform
mode, Rerun SDK/viewer version, host revision, schema version, device-time
range, sample/drop counts, bytes written, and any terminal fault. A source
identity mismatch marks the artifact stale; it is never silently reused.

## 5. Entity contract (schema v1)

Static entities are logged once; dynamic entities use `device_time` and
`frame_seq`.

| Entity path | Data | Rule |
|---|---|---|
| `/world` | declared coordinate system and axes | static |
| `/world/scanner` | body-to-sensor transform and physical bounds | static |
| `/world/scanner/pose` | raw SFLP, yaw-fused, and display poses, clearly labelled | orientation cadence; never conflate them |
| `/world/scanner/tof/{depth,reflectance,confidence,ambient}` | transformed images and correction/invalid-value metadata | selected ToF frames |
| `/world/scanner/tof/points` | valid deprojected points, visual color, point count | same selected frame; no duplicate pre-rotated cloud |
| `/world/scanner/imu/raw` | aggregate raw-IMU diagnostics needed to explain fusion | configured cadence |
| `/world/scanner/environment` | pressure, temperature, magnetic field, heading | available sensor samples |
| `/host/stream` | FPS, CRC, gaps, flags, transport, transform latency | scalar time series |
| `/host/slam` | tracking state, fitness, RMSE, step time, block usage, pose/trajectory | SLAM steps |
| `/world/map/mesh` | latest display-ready map snapshot and counts | limited latest state, not history |
| `/events` | firmware/host events, sidecar drops/faults/start/stop reasons | on event |

Implementation maps these values to the installed SDK's spatial, image,
point-cloud, scalar, text/event, and mesh archetypes. The exact Python call
shapes are intentionally not frozen: first probe the pinned SDK and write a
minimal fixture before production code. The paths, units, timestamps, and
provenance above are stable.

Depth is perpendicular Z in **mm**, matching transform output. Point positions,
transforms, poses, and meshes are **m**. Tests must use a signed non-cardinal
case such as 30°; 90°/180° checks are sign-blind.

## 6. Proposed ownership and modules

| Module | Responsibility |
|---|---|
| `roomscan/rerun_sidecar.py` | SDK-isolated worker, bounded queue, lifecycle, manifest, counters, fault containment |
| `roomscan/rerun_schema.py` | Pure conversion into schema-v1 records; no SDK import |
| `roomscan/rerun_export.py` | Offline replay orchestration and CLI |
| `mcp_server/tools_data.py` | Thin status/export wrapper only if agent-facing; reports actual artifact/counts/outcome |

`rerun-sdk` imports lazily inside the worker. Importing `roomscan`, protocol
tools, or MCP registry must not import Rerun, start a viewer, bind a port, or
allocate GPU state. Default configuration is `enabled = false`.

The live hook is after `TransformStage` and sensor-state update, next to the
existing latest-wins consumers. SLAM facts come from existing `FrameStep` and
already prepared mesh packets. The sidecar must not call `Mapper.mesh()`, run
Open3D extraction, or deproject a second time merely to improve its log.

## 7. Backpressure and measurements

Live limits are configuration keys until measured on a representative 30 Hz
room capture: `max_duration_s`, `max_bytes`, `tof_hz`, `mesh_hz`,
`mesh_vertex_budget`, and `queue_capacity`. The worker reports accepted,
sampled, and dropped records by kind; bytes; worker lag; peak queue depth;
artifact path; and terminal reason (`stopped`, `duration_budget`,
`byte_budget`, or `fault`) through existing metrics/logging.

`sidecar_drop_count > 0` explicitly marks the `.rrd` lossy. This is not a
scanner transport drop. Meshes use existing display-ready CPU packets, are
latest-only, vertex-bounded, and skipped under queue pressure. Full-resolution
Open3D mesh and `.ply` save are untouched.

## 8. Deployment and compatibility

- Pin SDK and viewer compatibility together in the lockfile and record both
  versions in every manifest; upgrade them as one tested change.
- Local `.rrd` is default. Network viewing is opt-in and loopback by default;
  roomscan itself opens no new public listener.
- The viewer is a developer tool containing unredacted sensor data.
- Report installed wheel size, runtime dependencies, and spawned browser/server
  process rather than treating host overhead as free.

## 9. Tests

Unit tests use a fake writer, so CI does not require Rerun or a viewer.

1. Schema tests cover paths, units, dtypes/shapes, invalid depths, sequence,
   and `t_us -> device_time` conversion.
2. Coordinate tests compare a known 30° pose/point with the verified roomscan
   conversion path, including sign.
3. Queue tests prove overload drops only sidecar items and reader callbacks do
   not wait for the worker.
4. Fault tests inject write/disk/start failures and prove truthful reporting
   while capture, web, and mapping remain live.
5. Export tests replay a small golden RSCN fixture and prove deterministic
   counts/times and stale-manifest detection.
6. An optional installed-SDK integration test inspects an emitted `.rrd` and
   asserts required paths/timelines exist.

## 10. Acceptance gates and decision

Do not enable by default merely because a cloud renders. Record all gates in
the implementation task report:

1. **SDK probe:** pinned SDK writes, and matching viewer opens, a minimal `.rrd`
   with depth, points, transform, and scalar on the headless host. Document
   footprint and process/network behavior.
2. **Offline fidelity:** a known capture export has expected selected-frame
   count, strictly monotonic device times, matching sequences/units, and a
   manifest tied to that source.
3. **Live non-interference:** compare representative sidecar-off/on 30 Hz runs.
   The on run has no additional decoder CRC/sequence failures, recorder
   regression, or sidecar-attributable tracking loss. Report source/transform/
   mapper FPS, web responsiveness, CPU/RAM, queue drops, and bytes for both;
   do not compare different operator motion.
4. **Bounded failure:** deliberately stop the writer or exhaust budget. The
   sidecar records the true reason while raw recording, web viewing, and mapping
   continue.
5. **Usefulness:** use the recording to answer a real diagnostic question that
   current tools made slow, such as the timing among a transform anomaly,
   orientation correction, and ICP tracking loss. An attractive rendering alone
   is insufficient.

Retain the sidecar only if all gates pass and diagnostic value outweighs
measured CPU/RAM/disk and maintenance cost. Otherwise remove the optional
dependency and retain the findings; RSCN plus roomscan-web/Open3D remain the
production infrastructure.

## 11. Documentation affected by implementation

Implementation updates `host/README.md`, `docs/system-architecture.md`, host
dependency/lock files, MCP documentation if a tool is added, and this status
with actual limits. It must not modify `docs/protocol.md` unless a separate
future wire change independently requires it.
