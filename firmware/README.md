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
The active firmware owns the sensor-time-critical work: it starts manual ToF
exposures, stamps the frame-ready edge, and DMA-reads into alternating raw
buffers. While one buffer is being filled, it can package the completed buffer,
drain the optional IMU FIFO, and start the next exposure. Commands and recovery
run only at a safe point after readout acknowledgement, so they cannot race an
in-flight trigger.

The normal build streams raw `3DMD` ToF data plus periodic calibration to the
PC, where the transform runs. It also emits available quaternion,
environmental, raw-IMU, and clock-synchronization streams. A shared versioned
binary protocol carries DATA, COMMAND, ACK, and EVENT frames over USB CDC and
Ethernet UDP; the host may receive either transport or replay the exact bytes
from a capture file.

For the end-to-end sequence, see [**`docs/system-architecture.md`**](../docs/system-architecture.md).
For wire details, see [**`docs/protocol.md`**](../docs/protocol.md). Build and
firmware-loop guidance remains in [**`CLAUDE.md`**](../CLAUDE.md).
