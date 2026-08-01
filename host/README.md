# <img src="https://api.iconify.design/material-symbols/computer.svg?color=white#gh-dark-mode-only" width="32" height="32" align="absmiddle"><img src="https://api.iconify.design/material-symbols/computer.svg#gh-light-mode-only" width="32" height="32" align="absmiddle"> Host PC Software (`roomscan`)

This directory contains the Python package `roomscan`, which serves as the brain on the host PC. Because the M33 MCU is bottlenecked by the transform pipeline, the host takes over heavy processing, acting as the primary visualization and SLAM engine.

## <img src="https://api.iconify.design/material-symbols/build.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/build.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Core Responsibilities

1.  **Transport & Decoding:** Connects to the scanner via USB CDC or Ethernet UDP. Decodes the binary frame protocol, resyncs on corruption, and handles sequences.
2.  **ToF Transform:** Runs the `vl53l9-transform-c` pipeline natively on the PC (compiled as a C-extension) to convert raw I3C data into depth, reflectance, and confidence maps.
3.  **Real-time SLAM (`roomscan.slam`):** Point-to-plane ICP frame-to-model tracking against a TSDF VoxelBlockGrid using Open3D's tensor API on the GPU. Capable of generating meshes at ~7ms/frame.
4.  **Web Visualizer (`roomscan-web`):** A FastAPI server providing a WebSocket (`/ws`) stream to a Three.js-based frontend. It replaces the old desktop panel, offering real-time point clouds, SLAM trajectories, sensor gizmos, and headless remote rendering.

## <img src="https://api.iconify.design/material-symbols/account-tree.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/account-tree.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Frame Flow

USB CDC, UDP, and capture replay all provide raw byte chunks to the same
incremental decoder. The shared reader loop routes control responses and
telemetry, tees raw bytes to recordings, and sends ToF frames through
`TransformStage`. That stage builds the native transform from a `CALIB` frame,
turns `RAW_3DMD` into depth/image arrays, and places only the newest completed
frame in a render slot. The web broadcaster and SLAM worker consume that slot
without competing for the device stream.

This means a capture is a protocol-level recording rather than a special export:
replay follows the same decoder, transform, deprojection, visualizer, and SLAM
path as live hardware. The complete cross-boundary explanation is in
[**`docs/system-architecture.md`**](../docs/system-architecture.md).

## <img src="https://api.iconify.design/material-symbols/folder-open.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/folder-open.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Structure

*   **`src/roomscan/`** - Core library source code.
*   **`tests/`** - Pytest suite for the host package (run from the `host/` directory).
*   **`tools/`** - Assorted Python utilities, recording playback scripts, diagnostics, and the MCP Server entrypoint (`roomscan-mcp`).

For details on the web UI WebSocket communication, see [**`docs/web-protocol.md`**](../docs/web-protocol.md).
