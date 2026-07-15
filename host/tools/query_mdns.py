import socket
from zeroconf import Zeroconf, ServiceBrowser

zeroconf = Zeroconf()
try:
    print("Checking for _roomscan._udp.local...")
    info = zeroconf.get_service_info("_roomscan._udp.local.", "roomscanner._roomscan._udp.local.")
    if info:
        print(f"Found roomscanner via mDNS!")
        print(f"IPs: {[socket.inet_ntoa(a) for a in info.addresses_by_version(4)]}")
    else:
        print("Not found via zeroconf.")
finally:
    zeroconf.close()
