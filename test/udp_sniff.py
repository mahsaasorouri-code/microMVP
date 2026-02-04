import socket
import time

PORT = 9001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", PORT))

print(f"Listening UDP on :{PORT} ... (Ctrl+C to stop)")

last = 0.0
cnt = 0

while True:
    data, addr = sock.recvfrom(4096)
    cnt += 1
    now = time.time()

    # limit prints to ~20 Hz to avoid flooding
    if now - last > 0.05:
        last = now
        n = len(data)
        print(f"\n#{cnt} from {addr} len={n}")

        if n == 33:
            level = data[0]
            payload = data[1:]
            print("level:", level)
            print("payload[0:16]:", list(payload[:16]))
            print("payload[16:32]:", list(payload[16:32]))
        else:
            print("data:", list(data[:min(n, 40)]), ("..." if n > 40 else ""))
