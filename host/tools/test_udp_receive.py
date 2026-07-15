import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 5000))
s.settimeout(0.5)

# send wakeup
s.sendto(b'\x00', ('172.17.2.58', 5000))
print("Sent wakeup to 172.17.2.58:5000")

start = time.time()
while time.time() - start < 10:
    try:
        data, addr = s.recvfrom(2048)
        print(f"Received {len(data)} bytes from {addr}")
    except socket.timeout:
        pass
    except Exception as e:
        print(e)
