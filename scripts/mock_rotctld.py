#!/usr/bin/env python3
import argparse
import socketserver


class RotctldHandler(socketserver.StreamRequestHandler):
    def handle(self):
        state = self.server.state
        while True:
            line = self.rfile.readline()
            if not line:
                break

            cmd = line.decode("ascii", "ignore").strip()
            if not cmd:
                continue

            if cmd == "q":
                break

            if cmd.startswith("\\dump_state"):
                dump = [
                    "1",
                    "202",
                    "Hamlib Mock rotctld",
                    "min_az: 0",
                    "max_az: 360",
                    "min_el: 0",
                    "max_el: 180",
                    "",
                ]
                self.wfile.write("\n".join(dump).encode("ascii"))
                self.wfile.flush()
                continue

            if cmd == "p":
                reply = "%0.1f\n%0.1f\nRPRT 0\n" % (
                    state["az"],
                    state["el"],
                )
                self.wfile.write(reply.encode("ascii"))
                self.wfile.flush()
                continue

            if cmd.startswith("P "):
                parts = cmd.split()
                if len(parts) >= 3:
                    try:
                        state["az"] = float(parts[1])
                        state["el"] = float(parts[2])
                    except ValueError:
                        pass
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            if cmd == "S":
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            self.wfile.write(b"RPRT 0\n")
            self.wfile.flush()


class RotctldServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4533)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    server = RotctldServer((args.host, args.port), RotctldHandler)
    server.state = {"az": 0.0, "el": 0.0}

    try:
        if args.once:
            server.handle_request()
        else:
            server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
