from zeroconf import Zeroconf, ServiceBrowser, ServiceListener
import time

class MyListener(ServiceListener):
    def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        pass
    def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        pass
    def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        info = zc.get_service_info(type_, name)
        if info:
            print(f"Service {name} added, addresses: {[socket.inet_ntoa(a) for a in info.addresses_by_version(4)]}")

if __name__ == "__main__":
    import socket
    zeroconf = Zeroconf()
    listener = MyListener()
    browser = ServiceBrowser(zeroconf, "_roomscan._udp.local.", listener)
    print("Searching for _roomscan._udp.local...")
    try:
        time.sleep(5)
    finally:
        zeroconf.close()
