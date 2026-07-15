import socket, time
time.sleep(2)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b'\x00', ('172.17.2.58', 5000))
print("sent to 172.17.2.58:5000")
