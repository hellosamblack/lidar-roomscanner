# Web Live / View + Detailed SLAM implementation plan

1. Add timestamp-aware capture indexing, capability scanning, Detailed preset,
   sidecar manifest helpers, and unit coverage.
2. Convert the server's public state to Live/View + Point cloud/SLAM/Detailed,
   preserving the existing reader swap and record guards; update MCP controls.
3. Wire the browser's source/display controls, timestamp timeline, capability
   notice, Detail confirmation/progress/stale UI, and existing MESH renderer.
4. Run the offline preset through `PostProcessWorker`, persist atomically, then
   load current or stale sidecars without mutating `.bin` data.
5. Add the reusable loop-closure evaluator and MCP entry point. Keep the runtime
   preset offline-only until both circuit ensembles satisfy the documented gate.
6. Verify unit/integration/browser paths, benchmark the real GPU constants, and
   reconcile ROADMAP, protocol/MCP docs, and shared status at landing.
