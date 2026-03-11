import socket

PORT = 8000   # change to 7000 if needed

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("192.168.0.60", PORT))

print(f"Listening on UDP {PORT} ...")
x= 0
while x<200:
    data, addr = sock.recvfrom(4096)
    print("Received", len(data), "bytes from", addr)
    print(data)
    x += 1