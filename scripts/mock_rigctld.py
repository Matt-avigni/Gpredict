#!/usr/bin/env python3
import argparse
import socketserver


class RigctldHandler(socketserver.StreamRequestHandler):
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
                    "3081",
                    "Hamlib Mock rigctld",
                    "has_get_vfo: 1",
                    "has_set_vfo: 1",
                    "has_set_vfo_opt: 1",
                    "vfo list: VFOA VFOB Main Sub currVFO",
                    "",
                ]
                self.wfile.write("\n".join(dump).encode("ascii"))
                self.wfile.flush()
                continue

            if cmd.startswith("\\dump_caps"):
                self.wfile.write(b"Mock caps\nRPRT 0\n")
                self.wfile.flush()
                continue

            if cmd == "\\reset_set_freq_count":
                state["set_freq_count"] = 0
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            if cmd == "\\get_set_freq_count":
                self.wfile.write(("%d\nRPRT 0\n" % state["set_freq_count"]).encode("ascii"))
                self.wfile.flush()
                continue

            if cmd == "f" or cmd.startswith("f "):
                self.wfile.write(("%d\nRPRT 0\n" % state["freq"]).encode("ascii"))
                self.wfile.flush()
                continue

            if cmd.startswith("F"):
                parts = cmd.split()
                if len(parts) >= 2:
                    freq_str = parts[-1]
                    try:
                        state["freq"] = int(float(freq_str))
                    except ValueError:
                        pass
                state["set_freq_count"] += 1
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            if cmd == "v":
                self.wfile.write((state["vfo"] + "\nRPRT 0\n").encode("ascii"))
                self.wfile.flush()
                continue

            if cmd.startswith("V "):
                state["vfo"] = cmd.split(None, 1)[1]
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            if cmd.startswith("\\set_vfo_opt"):
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            self.wfile.write(b"RPRT 0\n")
            self.wfile.flush()


class RigctldServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4532)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    server = RigctldServer((args.host, args.port), RigctldHandler)
    server.state = {"freq": 145800000, "vfo": "VFOA", "set_freq_count": 0}

    try:
        if args.once:
            server.handle_request()
        else:
            server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
