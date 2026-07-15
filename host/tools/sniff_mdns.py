import socket
import struct

def main():
    MCAST_GRP = '224.0.0.251'
    MCAST_PORT = 5353

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', MCAST_PORT))

    mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(1.0)

    print(f"Listening for mDNS on {MCAST_GRP}:{MCAST_PORT}")
    
    count = 0
    try:
        while count < 10:
            try:
                data, addr = sock.recvfrom(1024)
                # Ignore my own pings if possible, or print
                print(f"[{addr}] -> {data[:32]}")
                count += 1
            except socket.timeout:
                pass
    except KeyboardInterrupt:
        pass
    print("Done sniffing")

if __name__ == '__main__':
    main()
