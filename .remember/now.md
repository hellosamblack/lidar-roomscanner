# Current State
- Fixed firmware freeze when Ethernet cable is replugged (lwIP `mdns_resp_add_netif` double-add assertion).
- Fixed host `UdpSource` stream recovery by adding an active mDNS re-query when stream data stops (instead of broadcast).
- Live/View playback work is in the shared checkout: timestamp-aware capture metadata, stream-9 SLAM gating,
  a Detailed-SLAM sidecar runner, new `/ws` source/display controls, and MCP controls. The Detailed preset is
  intentionally offline-only until the two-circuit paired loop-closure gate is measured on CUDA:0.

# Next steps
- Monitor stability of Ethernet stream.
- Run browser/server integration outside the sandbox (local sockets are denied here), benchmark Detailed timing
  plus the 8/10/12 ICP variants on CUDA:0, and only then implement/enable the pose-graph pass if both circuits pass.
