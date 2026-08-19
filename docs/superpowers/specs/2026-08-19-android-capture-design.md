# Android capture app — posed high-res video from the Pixel 10 Pro XL

**Status:** Specced (priority/later)
**Date:** 2026-08-19
**Issues:** #135 (Android capture app — high-res video + ARCore pose/depth)
**Companion spec:** offline 3DGS program (`2026-08-19-offline-3dgs-program-design.md` —
this app is that program's posed-capture source, plan B)

## Goal

One take from the phone yields **video + per-frame pose + intrinsics + ARCore
depth/confidence**, so the 3DGS pass never depends on COLMAP SfM surviving featureless
walls (measured: 206/287 frames, 72%, Sam Office). The issue body (2026-08-07) is
already design-grade; this spec fixes the decision points, adds what has been learned
since, and attaches the gates.

## Corrections and constraints learned since the issue was filed

- **The Pixel 10 Pro XL has a ToF sensor** (`docs/rtabmap-pixel10-capture.md:83`) — the
  issue's "historically no ToF, unverified" is superseded, but the check stands: read
  `CameraConfig.getDepthSensorUsage()` on the handset and record the answer. Either way
  ARCore depth remains a **regularizer, not metric truth**; the rig's VL53L9CX stays the
  metric source.
- **RTAB-Map's own app cannot be the high-res path:** it hard-selects the lowest CPU
  camera config (driver 1) and its shared-camera driver carries an in-code `//FIXME` on
  exactly the unconfigured-surface problem this app must solve (`docs/rtabmap-study.md`
  §5). So #135 is not duplicating RTAB-Map — it exists *because* RTAB-Map caps
  resolution.
- **Plan A before plan B** (owner decision, 2026-08-11): first measure whether RTAB-Map's
  own exported low-res posed frames already beat COLMAP-on-4K (`#159`'s gate). This app
  is plan B — it is justified by the resolution ceiling plan A measures. Build order
  follows: the enumeration spike can start anytime, but full app investment waits for
  plan A's number.

## Decisions

1. **First deliverable is a measurement, not an app:** a minimal probe APK (or
   instrumented sample) that enumerates `CameraConfigFilter` on the actual handset and
   reports every achievable (video resolution, fps, depth on/off) combination through
   `SharedCamera`. If 4K and live depth are mutually exclusive, that is a recorded
   finding that reshapes the app — not something to design around blindly. This needs
   the owner's phone: `needs/operator`, `needs/hardware`.
2. **Output format: plain video + sidecar (option 2), decided now.** Poses from
   `Camera.getDisplayOrientedPose()`, intrinsics from `getImageIntrinsics()`, depth +
   confidence PNGs. It is directly the shape `roomscan.splat` and `transforms.json`
   want, and it keeps the host ingest trivial (the ARCore-MP4 custom-track route needs a
   host demuxer nobody else will ever use). The Recording & Playback API remains a
   fallback if `SharedCamera` recording proves unstable — reversible because the data
   content is identical.
3. **ToF fusion stays out of scope** — clock alignment (ARCore `CLOCK_MONOTONIC` ns vs.
   TIM2 µs) and the rigid mount/hand-eye extrinsic remain under DC-I (#146). This app
   must be useful with zero rig coupling.
4. **8th Wall stays parked** as the issue states: spiked only if the enumeration shows
   `SharedCamera` cannot reach ≥1080p60 with poses, and even then it must demonstrate an
   equivalent of the pose+video recording path before earning effort.
5. **Ingest lands as an MCP tool** per the standing rule (pure function + thin wrapper;
   CLI front end only if a human runs it) — extending the shipped #158 RTAB-Map ingest
   rather than a parallel path.

## Benchmark plan

- **Gate (a), unchanged from the issue and now with a harness:** on one real room, the
  ARCore-posed set must register/use **materially more frames than COLMAP-only on the
  same footage** — run through `splat_sfm_probe` so the comparison is per-config numbers
  (registration ratio, sub-model fragmentation), with the 206/287 (72%) COLMAP baseline
  as the stake in the ground. "Materially" is pre-registered as ≥90% frames used, or
  ≥+15 points over the COLMAP arm on the same video.
- **Gate (b):** the camera-config enumeration table, written into the issue and this
  spec's records.
- **Gate (c):** the MCP ingest path exists and a build runs end-to-end from a phone take
  with no manual file surgery.
- **Pose-quality check (cheap, catches silent garbage):** ArUco marker or bookend-style
  start/end-pose check on one take, so "poses recorded" ≠ "poses correct" (the
  invariant-check lesson: pick a check that can actually see wrong answers).

## Kill criteria

- **Killed by plan A succeeding:** if RTAB-Map's exported posed set already beats
  COLMAP-on-4K *and* its resolution ceiling does not show up in the manifest stats /
  `splat_compare` on a real room, the app adds resolution nobody measured a need for —
  #135 closes citing #159's numbers, and the shipped RTAB-Map export path is the
  permanent capture route.
- **Killed by the enumeration:** if `SharedCamera` tops out below 1080p30-with-poses on
  this handset *and* the 8th Wall spike cannot demonstrate pose+video recording, the
  phone-capture route degrades to RTAB-Map exports only — same closure as above, with
  the enumeration table as the evidence.
- **Descoped, not killed,** if gate (a) passes but ARCore depth proves useless on the
  featureless walls (its documented weakness): the app then ships poses+video only, and
  depth regularization stays with Depth-Anything-V2 (already implemented).
