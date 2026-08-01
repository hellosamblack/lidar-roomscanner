# Recent Milestones
- Phase 5 (Transport cutover to Ethernet) completed.
- Implemented robust Ethernet hot-plug recovery, ensuring that cable replugs gracefully restart DHCP and mDNS without firmware hard-faults.
- Host logic updated to actively resolve mDNS rather than failing back to broadcasts that don't route.
- Landed the Live/View playback foundation: timestamp-aware capture metadata, server-authoritative display state,
  Detailed-SLAM sidecars, and MCP controls. Pose-graph loop closure remains disabled pending the two-circuit CUDA gate.
- Landed View playback controls (`f15e4e3`): Preview display mode, 1× default playback, Detailed Build
  confirmation/estimate, and Turbo/Gray parity across Point cloud, SLAM, and Detailed.
- Landed Detailed build feedback (`16024ed`): mesh-extraction and cached-load phases are visible,
  replay is paused in Detailed, and the build overlay carries live GPU/CPU/RAM/VRAM bars.
- Landed the shared camera view (`5c27ae7`): World/FPV/Mirror across Point cloud, Preview, SLAM and
  Detailed, with the shared scanner model replacing SLAM's pose sphere.
- Landed **BUG-051** (`049a975`): yaw math moved off the ZYX decomposition onto a world-Z swing–twist.
  ZYX gimbal lock *is* the normal upright grip on this device, so the yaw-fusion gimbal gate had frozen
  the magnetometer correction permanently and `absolute_heading` carried an 18.4° systematic error at
  the operating pose. Gate deleted, BUG-048 closed, verified live on the rig.
- Proposed sub-phase **6.H** (audible coverage cue / buzzer, owner idea) in ROADMAP Phase 6.
