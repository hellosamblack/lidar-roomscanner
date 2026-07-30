# <img src="https://api.iconify.design/material-symbols/menu-book.svg?color=white#gh-dark-mode-only" width="32" height="32" align="absmiddle"><img src="https://api.iconify.design/material-symbols/menu-book.svg#gh-light-mode-only" width="32" height="32" align="absmiddle"> Documentation

Welcome to the documentation hub. This directory contains engineering conventions, protocol specifications, and historical context for the Roomscanner project.

## <img src="https://api.iconify.design/material-symbols/description.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/description.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Key Specifications & Guidelines

*   [**`engineering-practices.md`**](engineering-practices.md) - The constitution for contributing. Defines repo rules, commit structures, C/Python coding standards, and firmware loop practices.
*   [**`protocol.md`**](protocol.md) - The binary wire protocol specification defining the transport layer between the MCU and Host PC.
*   [**`web-protocol.md`**](web-protocol.md) - The WebSocket JSON/Binary protocol for the `roomscan-web` frontend.
*   [**`mcp-server.md`**](mcp-server.md) - API and structural documentation for the `roomscan-mcp` agentic tool server.

## <img src="https://api.iconify.design/material-symbols/electrical-services.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/electrical-services.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Hardware & Setup Guides

*   [**`headless-host-setup.md`**](headless-host-setup.md) - Guide for bringing up a GPU-less Linux host to run the web server.
*   [**`iks4a1-stacking.md`**](iks4a1-stacking.md) - The hardware stacking recipe, jumper configurations, and bus-conflict resolution history for the sensor array.
*   [**`web-ui-testing.md`**](web-ui-testing.md) - How to drive and test the headless web UI.

## <img src="https://api.iconify.design/material-symbols/folder-open.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/folder-open.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Implementation Archives

*   **`superpowers/`** - Contains raw design documents (`specs/`) and phase execution checklists (`plans/`).
