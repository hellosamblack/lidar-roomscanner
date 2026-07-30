# <img src="https://api.iconify.design/material-symbols/memory.svg?color=white#gh-dark-mode-only" width="32" height="32" align="absmiddle"><img src="https://api.iconify.design/material-symbols/memory.svg#gh-light-mode-only" width="32" height="32" align="absmiddle"> Firmware Layer

This directory contains the embedded firmware components for the Roomscanner project.

## <img src="https://api.iconify.design/material-symbols/my-location.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/my-location.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Target Hardware

*   **Core:** NUCLEO-H563ZI (STM32H563ZI Cortex-M33)
*   **ToF Expansion:** X-NUCLEO-53L9A1 (VL53L9CX ToF 3D LiDAR)
*   **IMU/Env Expansion:** X-NUCLEO-IKS4A1 (LSM6DSV16X, LIS2MDL, LPS22DF, SHT40)

## <img src="https://api.iconify.design/material-symbols/folder-open.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/folder-open.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Directory Structure

*   [**`scanner-stream/`**](scanner-stream/)
    *   Our active firmware fork. Handles reading ToF data over I3C + DMA, reading IMU streams, packaging into binary frames, and streaming over USB CDC or Ethernet UDP.
*   [**`vendor/`**](vendor/) - Vendored dependencies:
    *   `tinyusb/` - USB CDC stack (FS).
    *   `lwip/` - TCP/IP stack for Ethernet UDP streaming.
    *   `53L9A1/` - ST's reference package. We treat this as a **read-only reference** rather than editing it in place.

## <img src="https://api.iconify.design/material-symbols/settings.svg?color=white#gh-dark-mode-only" width="28" height="28" align="absmiddle"><img src="https://api.iconify.design/material-symbols/settings.svg#gh-light-mode-only" width="28" height="28" align="absmiddle"> Architecture Highlights
The firmware acquires frames using a double-buffered DMA setup via I3C, processing them with ST's `vl53l9-transform-c` pipeline (if done on-device) and streaming raw or processed buffers (along with synchronized IMU quaternions) to the host PC. 

It handles multiple streams (Depth/Raw ToF, IMU SFLP Quat, Env Sensors) multiplexed into a single binary transport protocol.

For build instructions (`cmake`, `ninja`), debugging setup, and detailed architecture, refer to the root [**`CLAUDE.md`**](../CLAUDE.md) file.
