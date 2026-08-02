# RAVPower RP-WD007 FileHub Technical Guide & Reference

This document provides a comprehensive overview of the **RAVPower FileHub RP-WD007**, its underlying hardware/software architecture, CLI interaction techniques over Telnet, and actionable instructions for expanding its functionality (such as operating as an Independent Access Point or querying battery status).

---

## Table of Contents
1. [Device Architecture Overview](#device-architecture-overview)
2. [Key Telnet Commands & Utilities](#key-telnet-commands--utilities)
3. [Mode 1: Transparent Ethernet-to-Wi-Fi Bridge (Current Setup)](#mode-1-transparent-ethernet-to-wi-fi-bridge-current-setup)
4. [Mode 2: Independent Wireless Access Point (Standalone AP Mode)](#mode-2-independent-wireless-access-point-standalone-ap-mode)
5. [Reading Battery & Power Status](#reading-battery--power-status)
6. [Summary of Automation & Scripting](#summary-of-automation--scripting)

---

## 1. Device Architecture Overview

The RAVPower RP-WD007 is built on a MediaTek/Ralink System-on-Chip (SoC) architecture (typically **MT7628**).

### Operating System & Storage
* **Kernel & Userland:** Embedded Linux kernel (2.6.36 generation) using a lightweight BusyBox system environment.
* **Root Filesystem (`/`):** Compressed read-only **SquashFS** filesystem. Any modifications made to system paths like `/etc` or `/sbin` exist in RAM (tmpfs) and **do not survive a reboot**.
* **Storage Mounts:** External SD cards and USB storage devices are auto-mounted under `/data/UsbDisk1/Volume1`, `/data/UsbDisk2/Volume1`, etc.

### Core Daemons & Middleware
* `ioos`: The primary proprietary RAVPower middleware daemon. It manages system events, app APIs, file sharing services (Samba/DLNA), and hardware events (such as Ethernet cable insertion).
* `led_control`: Manages LED status indicators (Power, Wi-Fi, Internet, SD).
* `control`: Secondary daemon monitoring system state and hardware events.
* `fileserv` / `vshttpd`: Serves the web-based management UI and mobile app APIs.

### Network Interface Mapping
* `br0`: The main internal LAN bridge interface (default IP `10.10.10.254`).
* `eth2`: The physical RJ-45 Ethernet port.
* `ra0`: 2.4 GHz Wi-Fi Access Point interface.
* `rai0`: 5 GHz Wi-Fi Access Point interface.
* `apcli0`: Wi-Fi Client interface (used when connecting to an upstream Wi-Fi network/hotspot).

---

## 2. Key Telnet Commands & Utilities

Telnet access is available over port 23.
* **Authentication:** User `root`, Password `20080826` (or `admin` / blank depending on firmware build).
* Once authenticated, Telnet drops into a root shell (`#`). Below are essential commands for inspecting and manipulating the hardware state:

### Network Diagnostics & Bridging
```bash
# Display all network interfaces and assigned IPs
ifconfig -a

# Display active bridge interfaces and attached ports
brctl show

# Add an interface to the LAN bridge (e.g. bridging Wi-Fi client to Ethernet)
brctl addif br0 apcli0

# Remove an interface from the bridge
brctl delif br0 eth2

# Show wireless interface details
iwconfig
```

### Process Control
```bash
# List all running processes (using BusyBox implementation)
busybox ps

# Find Process ID of a specific binary
pidof udhcpd

# Kill daemons or services
killall udhcpd
killall ioos
```

### NVRAM Configuration
MediaTek/Ralink SDKs store device settings in NVRAM.
```bash
# Get NVRAM variable value
nvram_get 2860 OperationMode
nvram_get 2860 lan_ipaddr

# Set NVRAM variable (Note: ioos may overwrite some values on boot)
nvram_set 2860 OperationMode 0
```

*Operation Modes (`OperationMode`):*
* `0`: **Bridge / AP Mode** (`eth2` is LAN, wireless bridged to LAN)
* `1`: **Gateway / Router Mode** (`eth2` is WAN with NAT enabled)
* `2`: **Ethernet Converter Mode**
* `3`: **AP Client / WISP Mode** (`eth2` is LAN, `apcli0` is WAN with NAT enabled)

---

## 3. Mode 1: Transparent Ethernet-to-Wi-Fi Bridge (Current Setup)

In this mode, the RP-WD007 acts as a transparent Layer-2 bridge between its physical Ethernet port (`eth2`) and an upstream Wi-Fi network (`apcli0`).

### The Challenge
When an Ethernet cable is plugged in, the `ioos` daemon automatically executes `/sbin/EnterRouterMode.sh`, which tears down `apcli0` and forces the device into Gateway mode.

### The Solution (Live Workaround)
1. **Blindfold `ioos`:** Create an empty shell script `/tmp/dummy.sh` and bind-mount it over `/sbin/EnterRouterMode.sh` and `/sbin/net_auto_switch`.
2. **Bridge `apcli0`:** Add `apcli0` to `br0` via `brctl addif br0 apcli0`.
3. **Disable Local DHCP:** Kill `udhcpd` so connected Ethernet devices receive DHCP leases directly from the upstream Wi-Fi network (`172.17.2.x`).

---

## 4. Mode 2: Independent Wireless Access Point (Standalone AP Mode)

To operate the RP-WD007 as a standalone, isolated Wi-Fi Access Point (creating its own `10.10.10.x` network with local DHCP server, independent of any upstream WAN):

### CLI Setup Commands for Independent AP Mode
Run the following script over Telnet to enable local AP mode:

```bash
#!/bin/sh

# 1. Blindfold auto-router daemons to prevent unwanted mode switching
echo "#!/bin/sh" > /tmp/dummy.sh
echo "exit 0" >> /tmp/dummy.sh
chmod +x /tmp/dummy.sh
mount --bind /tmp/dummy.sh /sbin/EnterRouterMode.sh
mount --bind /tmp/dummy.sh /sbin/net_auto_switch

# 2. Remove Wi-Fi client from bridge if connected
brctl delif br0 apcli0 2>/dev/null
ifconfig apcli0 down 2>/dev/null

# 3. Ensure physical ethernet is part of the local LAN bridge
brctl addif br0 eth2 2>/dev/null

# 4. Ensure LAN bridge has the static gateway IP
ifconfig br0 10.10.10.254 netmask 255.255.255.0 up

# 5. Start the local DHCP server (udhcpd)
killall udhcpd 2>/dev/null
/usr/sbin/udhcpd /etc/udhcpd.conf &

echo "Independent Access Point Mode Enabled!"
```

### Explanation of Independent AP Mode Logic
* **DHCP Server (`udhcpd`):** Started using `/etc/udhcpd.conf`, which assigns IP addresses in the `10.10.10.100` – `10.10.10.200` range to wireless and ethernet clients.
* **Bridge Topology (`br0`):** Combines `ra0` (2.4G), `rai0` (5G), and `eth2` (Ethernet) under the local IP `10.10.10.254`.

---

## 5. Reading Battery & Power Status

The RP-WD007 SoC reads power and battery state via vendor kernel procfs entries.

### Primary Verified Method: `/proc/vs_battery_quantity` (Recommended)
Tested and verified live on the RP-WD007 hardware:
```bash
cat /proc/vs_battery_quantity
# Outputs integer percentage, e.g. 100
```
* **Kernel Node:** `/proc/vs_battery_quantity`
* **Format:** Plain text integer percentage (0–100).
* **Daemon Usage:** `/usr/sbin/ioos` opens and reads this procfs file directly to monitor device power state.
* **Scraping:** Can be queried over Telnet on demand (e.g. `cat /proc/vs_battery_quantity`).

### Other Related System / Proc Nodes
* `/proc/vs_poweroff_key_status` (returns power key status integer, e.g. `255`)
* `/proc/vs_gpio_power_off`

### Hardware / Web Probing Summary (Firmware Build Notes)
* **`pioctl`:** `/usr/sbin/pioctl` exists on this device but only controls status/Wi-Fi/internet/WPS LEDs, fans, SATA, and USB power toggles. It does not support a `battery` command in this build.
* **`sysfs`:** `/sys/class/power_supply/` is not present in this kernel (2.6.36).
* **Web UI / `fileserv`:** The built-in lighttpd web server (`fileserv`) acts as a static WebDAV and UI file server. Battery status is managed internally by the `ioos` daemon via `/proc/vs_battery_quantity`.

---

## 6. Summary of Automation & Scripting

Because the filesystem is read-only SquashFS, automations (such as `filehub-bridgemode.sh`) should be executed externally over Telnet using `expect` upon boot or on demand.

### Script Summary Table
| Mode / Action | Key Interfaces | Active Services | Command / Script |
| :--- | :--- | :--- | :--- |
| **Transparent Bridge** | `eth2` + `apcli0` in `br0` | `udhcpd` killed, `EnterRouterMode` blindfolded | `filehub-bridgemode.sh` |
| **Independent AP** | `eth2` + `ra0` + `rai0` in `br0` | `udhcpd` running on `10.10.10.254` | Custom AP Telnet script |
| **Battery Check** | `/proc/vs_battery_quantity` | Procfs / `ioos` | `cat /proc/vs_battery_quantity` over Telnet |
