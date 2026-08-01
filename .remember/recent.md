# Recent Milestones
- Phase 5 (Transport cutover to Ethernet) completed.
- Implemented robust Ethernet hot-plug recovery, ensuring that cable replugs gracefully restart DHCP and mDNS without firmware hard-faults.
- Host logic updated to actively resolve mDNS rather than failing back to broadcasts that don't route.
- Landed the Live/View playback foundation: timestamp-aware capture metadata, server-authoritative display state,
  Detailed-SLAM sidecars, and MCP controls. Pose-graph loop closure remains disabled pending the two-circuit CUDA gate.
- Landed View playback controls (`f15e4e3`): Preview display mode, 1× default playback, Detailed Build
  confirmation/estimate, and Turbo/Gray parity across Point cloud, SLAM, and Detailed.
