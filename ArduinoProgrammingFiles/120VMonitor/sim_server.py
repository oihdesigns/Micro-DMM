"""
sim_server.py  --  serve the simulated 120VMonitor (monitor_sim.py) over TCP,
for the Android app's "Network…" connect.

    py -3 sim_server.py [port]        (default 8120)

From the Android emulator the PC is 10.0.2.2, so connect the app to
10.0.2.2:8120. Each connection gets a fresh simulated board.
"""

import socket
import sys
import threading

import monitor_sim


def serve(conn, addr):
    sim = monitor_sim.SimSerial()
    alive = threading.Event()
    alive.set()

    def pump():                                   # board -> app
        try:
            while alive.is_set():
                chunk = sim.read(4096)
                if chunk:
                    conn.sendall(chunk)
        except OSError:
            alive.clear()

    threading.Thread(target=pump, daemon=True).start()
    buf = b""
    try:
        while alive.is_set():                     # app -> board, one command line at a time
            data = conn.recv(4096)
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                sim.write(line + b"\n")
    except OSError:
        pass
    finally:
        alive.clear()
        conn.close()
        print(f"{addr[0]}:{addr[1]} disconnected")


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8120
    srv = socket.create_server(("0.0.0.0", port))
    print(f"simulated 120VMonitor on port {port} (emulator: 10.0.2.2:{port})")
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"{addr[0]}:{addr[1]} connected")
        threading.Thread(target=serve, args=(conn, addr), daemon=True).start()


if __name__ == "__main__":
    main()
