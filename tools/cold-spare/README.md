# Cold spares

Scripts kept around for a piece of hardware that is no longer in the primary
path, in case it is ever pressed back into service. Nothing here is wired
into `roomscan-web`, the MCP server, or any test -- these are cold, not a
maintained integration.

## `filehub-bridgemode.sh` / `filehub-batteryCheck.sh`

Retired 2026-08-20 (issue #192). The RavPower FileHub was the scanner's
original Wi-Fi uplink; the Raspberry Pi bridge (issue #191,
`host/tools/pi_bridge.py`, `pi-bridge/`) replaced it because nothing about
the FileHub was observable when it dropped frames (2.3-9.4% loss, silent).

`filehub-bridgemode.sh` re-applied transparent-bridge mode on the FileHub over
telnet (`expect`-driven); it was step 3 of a 4-step physical recovery (unplug
Ethernet -> power-cycle the FileHub -> run this -> replug) and used to be a
top-bar "Bridge Mode" button in the web UI. `filehub-batteryCheck.sh` read the
FileHub's battery level the same way. Both hardcode `ROUTER_IP=172.17.2.57`
and a plaintext telnet password -- treat that password as stale/rotate it
before ever reusing these against real hardware again.

If the FileHub comes back into the loop, these are a starting point, not a
guarantee: they predate the Pi bridge's status/health surfacing and have no
equivalent to `bridge_status`'s structured JSON or warnings list.
