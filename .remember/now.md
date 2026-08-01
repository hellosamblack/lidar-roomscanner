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

# Next steps
- Start Task 1 of the high-frame-rate plan: reconcile estimate anchors, prove exposure granularity and
  schedulable blanking, add the pure profile model/probe, and record the current 30 Hz hardware baseline.
- Monitor stability of Ethernet stream.
- Run browser/server integration with elevated local-socket permission, benchmark Detailed timing plus the
  6/8/10/12 ICP variants on CUDA:0, and only then implement/enable the pose-graph pass if both circuits pass.
