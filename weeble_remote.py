"""
weeble_remote.py  --  talk to the drone over its WiFi.

1. Power the drone. On the laptop, join the WiFi network "weeble" (password is in src/main.cpp).
2. Run:   py weeble_remote.py
3. Type commands exactly like in the serial monitor (status, cal, unlock, arm, hop 5, ...) and press Enter.
   Type  k  and Enter to KILL the motors.  Ctrl+C to quit.

While this runs it sends a small "hb" (heartbeat) message twice a second. If the laptop disappears
(closed, crashed, out of range), the heartbeats stop and the drone applies its link-lost rule.
"""
import socket, sys, threading, time

HOST, PORT = "192.168.4.1", 23

try:
    sock = socket.create_connection((HOST, PORT), timeout=5)
except OSError as e:
    sys.exit(f"Could not connect to {HOST}:{PORT} ({e}). Is the laptop on the 'weeble' WiFi network?")
sock.settimeout(None)
send_lock = threading.Lock()
alive = True


def send(text):
    with send_lock:                       # heartbeat thread and typing never interleave mid-line
        sock.sendall((text + "\n").encode())


def reader():                             # print everything the drone says
    global alive
    while alive:
        try:
            data = sock.recv(1024)
        except OSError:
            break
        if not data:
            break
        sys.stdout.write(data.decode(errors="replace"))
        sys.stdout.flush()
    alive = False
    print("\n[connection to drone closed -- press Enter to exit]")


def heartbeat():
    while alive:
        try:
            send("hb")
        except OSError:
            break
        time.sleep(0.5)


threading.Thread(target=reader, daemon=True).start()
threading.Thread(target=heartbeat, daemon=True).start()
print(f"connected to {HOST}:{PORT}. Type commands. 'k' + Enter = KILL. Ctrl+C to quit.")
try:
    while alive:
        line = input().strip()
        if line and alive:
            send(line)
except (KeyboardInterrupt, EOFError):
    pass
alive = False
sock.close()
