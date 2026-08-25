"""Zeroconf / mDNS service advertisement for the roomscan web server (#194)."""
from __future__ import annotations

import logging
import socket
from contextlib import suppress

from zeroconf import ServiceInfo, Zeroconf

log = logging.getLogger("roomscan.web")

MDNS_SERVICE_TYPE = "_roomscan._tcp.local."
MDNS_INSTANCE = "roomscan"


def _lan_address() -> str:
    """Best-guess LAN address to advertise. The server binds 0.0.0.0, so mDNS
    has to name one concrete address; the outbound-route trick picks the
    interface that actually reaches the network without sending a packet."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except Exception:  # noqa: BLE001 - no route: fall back to loopback
        return "127.0.0.1"
    finally:
        sock.close()


def register_mdns(port: int, *, zeroconf_factory=Zeroconf,
                  address: str | None = None):
    """Advertise this web server as `_roomscan._tcp.local.` (#194).

    First time this server has ever advertised itself -- the existing
    `zeroconf` use in `sources.py` only *browses*, to find the MCU. A thin
    client can now find the host without a hardcoded IP.

    Returns `(zeroconf, service_info)` or None. Never raises: mDNS is a
    convenience, and a host without multicast must still serve the UI. The
    factory is injected the same way `UdpSource` does it, so a test can
    exercise this without touching the network.
    """
    try:
        addr = address or _lan_address()
        info = ServiceInfo(
            MDNS_SERVICE_TYPE,
            f"{MDNS_INSTANCE}.{MDNS_SERVICE_TYPE}",
            addresses=[socket.inet_aton(addr)],
            port=int(port),
            properties={"path": "/static/index.html", "thin": "/ws-thin"},
            server=f"{MDNS_INSTANCE}.local.",
        )
        zc = zeroconf_factory()
        zc.register_service(info)
        log.info("mDNS: advertised %s at %s:%d", MDNS_SERVICE_TYPE, addr, port)
        return zc, info
    except Exception as exc:  # noqa: BLE001
        log.warning("mDNS advertisement unavailable: %s: %s",
                    type(exc).__name__, exc)
        return None


def unregister_mdns(registration) -> None:
    """Withdraw the advertisement. Tolerates None and a half-dead stack."""
    if not registration:
        return
    zc, info = registration
    try:
        zc.unregister_service(info)
    except Exception as exc:  # noqa: BLE001
        log.warning("mDNS unregister failed: %s", exc)
    finally:
        with suppress(Exception):
            zc.close()



__all__ = [k for k in globals() if not k.startswith("__")]
