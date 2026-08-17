# <img src="https://api.iconify.design/material-symbols/menu-book.svg?color=white#gh-dark-mode-only" width="32" height="32" align="absmiddle"><img src="https://api.iconify.design/material-symbols/menu-book.svg#gh-light-mode-only" width="32" height="32" align="absmiddle"> Documentation

Welcome to the documentation hub. This directory contains engineering conventions, protocol specifications, calibration and validation records, and historical context for the Roomscanner project.

Agent guidance lives one level up in [`AGENTS.md`](../AGENTS.md) (`CLAUDE.md` is a symlink to it); project skills are in [`.agents/skills/`](../.agents/skills). Forward-looking work and open defects are **GitHub Issues**, not files — `gh issue list --label bug|work-item|data-collection`, further tagged `area/*`, `status/*`, and `priority/now|next|later`.

## <img src="https://api.iconify.design/material-symbols/description.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/description.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Key Specifications & Guidelines

*   [**`system-architecture.md`**](system-architecture.md) - How a measurement moves through the STM32 acquisition loop, transport, host transform, web viewer, and SLAM pipeline.
*   [**`engineering-practices.md`**](engineering-practices.md) - The constitution for contributing. Defines repo rules, commit structures, C/Python coding standards, and firmware loop practices.
*   [**`protocol.md`**](protocol.md) - The binary wire protocol specification defining the transport layer between the MCU and Host PC.
*   [**`web-protocol.md`**](web-protocol.md) - The WebSocket JSON/Binary protocol for the `roomscan-web` frontend.
*   [**`mcp-server.md`**](mcp-server.md) - API and structural documentation for the `roomscan-mcp` agentic tool server.
*   [**`coordinate-frames.md`**](coordinate-frames.md) - **Binding**: the sensor/body/world frame definitions and the rotation conventions every orientation claim is expressed in.

## <img src="https://api.iconify.design/material-symbols/electrical-services.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/electrical-services.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Hardware & Setup Guides

*   [**`headless-host-setup.md`**](headless-host-setup.md) - Guide for bringing up a GPU-less Linux host to run the web server.
*   [**`pi-bridge-runbook.md`**](pi-bridge-runbook.md) - The Raspberry Pi 3 bridge node that replaces the FileHub as the rig's wireless uplink: build the SD image, flash, first boot, day-2 administration over MCP, recovering lost frames from the pcap tee, and the failure playbook.
*   [**`iks4a1-stacking.md`**](iks4a1-stacking.md) - The hardware stacking recipe, jumper configurations, and bus-conflict resolution history for the sensor array.
*   [**`web-ui-testing.md`**](web-ui-testing.md) - How to drive and test the headless web UI.
*   [**`h563-optimization-notes.md`**](h563-optimization-notes.md) - Why the transform pipeline runs on the PC: measured M33 throughput limits and the deferred on-device optimizations.
*   [**`vl53l9cx-datasheet-notes.md`**](vl53l9cx-datasheet-notes.md) - Working notes on the ToF sensor's datasheet (raw frame sizes, binning, DSS).
*   [**`vl53l9cx-fov-notes.md`**](vl53l9cx-fov-notes.md) - Field-of-view geometry and the deprojection consequences.

## <img src="https://api.iconify.design/material-symbols/tune.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/tune.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Calibration, Sensors & Validation

*   [**`flatfield-calibration.md`**](flatfield-calibration.md) - Mode-aware reflectance flat-field correction: the FPN measurement, map generation, and cross-room validation.
*   [**`yaw-fusion.md`**](yaw-fusion.md) - Host-side magnetometer/IMU heading fusion, and the heading-frame traps it has hit.
*   [**`transform-streams.md`**](transform-streams.md) - The `vl53l9-transform-c` pipeline's streams and algorithm toggles, including the 1.3.1 → 1.5.0 upgrade gate.
*   [**`deprojector-validation.md`**](deprojector-validation.md) - How depth-to-point-cloud deprojection was validated.
*   [**`phase6-slam-validation.md`**](phase6-slam-validation.md) - The offline SLAM validation record (accuracy gates and measured outcomes).
*   [**`imu-mag-appnote-review-2026-07-29.md`**](imu-mag-appnote-review-2026-07-29.md) - Review of ST's IMU/magnetometer application notes against our implementation.
*   [**`odr-triggered-sync-costing-2026-07-30.md`**](odr-triggered-sync-costing-2026-07-30.md) - Costing study for ODR-triggered ToF/IMU synchronization (note the amended §2.3 premise).
*   [**`connect-transient-forensics.md`**](connect-transient-forensics.md) - Forensics on the USB CDC connect-time CRC transient.

## <img src="https://api.iconify.design/material-symbols/hub.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/hub.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> RTAB-Map Study (Phase 6/7 reference design)

*   [**`rtabmap-study.md`**](rtabmap-study.md) - What RTAB-Map does differently (per-node clouds + pose graph vs our monolithic TSDF); indexes the issues it generated.
*   [**`rtabmap-pixel10-capture.md`**](rtabmap-pixel10-capture.md) - Exact RTAB-Map capture settings for Phase 7 on the Pixel 10 Pro XL.

## <img src="https://api.iconify.design/material-symbols/folder-open.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/folder-open.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> History & Archives

*   [**`roadmap-history.md`**](roadmap-history.md) - Completed-phase narratives and measured outcomes; the historical record behind [`ROADMAP.md`](../ROADMAP.md).
*   [**`issue-migration-map.md`**](issue-migration-map.md) - Legacy `BUG-NNN` / `SLAM-N` / `DC-<letter>` → GitHub issue mapping (2026-08-10 migration). Generated from `issue-migration-map.json`, which is the machine-readable source of truth.
*   **`superpowers/`** - Design documents (`specs/`) and phase execution checklists (`plans/`). Active work sits at each directory root; historical records under `completed/`, superseded ones under `deprecated/`. The register in [`ROADMAP.md`](../ROADMAP.md) is the inventory.
