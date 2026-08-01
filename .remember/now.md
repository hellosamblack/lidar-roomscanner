# Current State
- A reviewed implementation plan now exists at
  `docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md`; it starts with a
  hardware-backed contract/baseline gate before the protocol v2, autonomous capture, transport, SLAM,
  Web UI, and MCP work.
- Fixed firmware freeze when Ethernet cable is replugged (lwIP `mdns_resp_add_netif` double-add assertion).
- Fixed host `UdpSource` stream recovery by adding an active mDNS re-query when stream data stops (instead of broadcast).
- Live/View playback work is committed on `main` as `f15e4e3`: timestamp-aware capture metadata,
  stream-9 SLAM gating, a Detailed-SLAM sidecar runner, Preview as a View display, one-way `/ws`
  source/display controls, build confirmation/estimate, 1× playback default, and MCP controls. The
  Detailed preset is intentionally offline-only until the two-circuit paired loop-closure gate is
  measured on CUDA:0.
- Web UI follow-up landed on `main` as `5c27ae7`: World / FPV / Mirror now applies across Point cloud,
  Preview, SLAM, and Detailed. SLAM's green pose sphere is replaced by the shared scanner model;
  Detailed progress carries its latest pose for its FPV/Mirror camera.
- **BUG-051 landed on `main` as `049a975`** — the orientation stack's yaw math is out of the ZYX frame.
  ZYX gimbal lock sits at the *normal upright grip* on this device (body X = Up), which had three live
  consequences: `YawFusion`'s 15° gimbal gate never cleared, so the mag correction had effectively
  never run; `absolute_heading` carried an **18.4° systematic error** at the operating pose (exact only
  at 0/90/180/270°); and World's Roll readout sat on the ±180 wrap. All three fixed via the new
  `sensors.yaw_twist_deg` (world-Z swing–twist); the gate and `yaw_gimbal_margin_deg` are deleted.
  Closes BUG-048 and corrects its "not a significant live-operation defect" scope note. Verified live
  at ZYX pitch 87.72°: fusion Active 80/80, World roll 178.31° → −0.93°.
- Detailed build feedback landed on `main` as `16024ed`: explicit preview-mesh extraction and
  asynchronous saved-mesh loading, replay paused in Detailed, and live GPU/CPU/RAM/VRAM bars under
  the viewport progress bar. The live Ethernet/UDP viewer was restored after the wrap-up.

# Next steps
- **Re-score DC-E's heading clause** (`captures/DebugCapE.bin`) now that BUG-051's singularity fix has
  landed — ROADMAP's DC-E row was explicitly waiting on it before sizing DT0103's accelerometer-assisted
  magnetometer fit. The tilt-ramp clause (1.72×) is immune to the bug and still fails.
- Start Task 1 of the high-frame-rate plan: reconcile estimate anchors, prove exposure granularity and
  schedulable blanking, add the pure profile model/probe, and record the current 30 Hz hardware baseline.
- Monitor stability of Ethernet stream.
- Run browser/server integration with elevated local-socket permission, benchmark Detailed timing plus the
  6/8/10/12 ICP variants on CUDA:0, and only then implement/enable the pose-graph pass if both circuits pass.
