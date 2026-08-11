# Roomscan-web Live / View + Detailed SLAM

## Status

Implemented in phases described by `docs/superpowers/plans/completed/2026-07-31-web-live-view-detailed.md`.

## Product model

`roomscan-web` has one source choice and one display choice. **Live** means the
wire device and permits recording. **View** means a selected immutable capture.
The shared display choice is `point_cloud` or `slam`; `detailed` is an additional
View-only choice. Point cloud and SLAM are live previews; only Detailed writes a
**capture-keyed** sidecar.

*Amended 2026-07-31 after review (BUG-043).* "Only Detailed persists" holds for
**replay** SLAM, not for Live. Live SLAM keeps its existing one-shot `save` →
`results/web_<ts>.ply`/`.tum`, because a live scan is unrepeatable: unless Record
was running its frames are never stored, so dropping the map discards the only
copy and there is nothing to re-run as Detailed. That export is a free-standing
result, not keyed to a capture, so it never interacts with sidecar staleness.
Replay SLAM refuses `save` with a reason naming Detailed.

The server remains authoritative. `state` carries `source` (`live|view`) and
`display` (`point_cloud|slam|detailed`); `session` remains the transport snapshot
and adds timestamp-derived playback time. A client cannot enable SLAM for a
capture without stream 9. That case stays point-cloud playable and explains why
the controls are disabled.

## Detailed artifacts

Detailed processing uses a named `DetailedSlamPreset`: all depth frames are
processed, the retry radius is always attempted after the tight solve, and its
effective settings are fingerprinted. A completed build writes only:

```
results/<capture-stem>.ply
results/<capture-stem>.tum
results/<capture-stem>.slam.json
```

The manifest is written last and records the capture stat identity, resolved
preset/fingerprint, timing estimate calibration, tracking metrics, and loop
closure decision/results. A changed fingerprint or capture stat makes an
otherwise readable mesh **stale**; it is shown immediately but never regenerated
without an explicit request.

`PostProcessWorker` is the offline runner. It publishes progressive meshes into
the established MESH transport, so the Detailed progress dialog and visible room
build share the same source of truth. A job survives selection changes and is
stopped only on server shutdown.

## Loop-closure gate

The offline validation runner compares frame-to-model Detailed processing with a
pose-graph/reintegration pass over ten matched harmless perturbations for each
coffee-room circuit. Loop closure is acceptable only when both circuits have a
positive paired 95% confidence interval for improved horizontal closure, without
a died run or extra tracking loss. Until that result exists, the Detailed preset
is deliberately offline-only and its manifest says so. Relocalization is not
part of this feature.

## Protocol and timeline

The existing MESH binary layout is unchanged. The `/ws` JSON contract adds
`set_source`, `set_display`, Detailed status/start/regenerate messages, and
capture capability/sidecar metadata; `docs/web-protocol.md` is the normative
wire reference.

Playback uses valid monotonic DATA `FrameHeader.t_us` values from the TIM2 clock
for duration, display, and seeking. A legacy capture with absent/invalid header
times falls back to nominal 30 FPS indexed time.
