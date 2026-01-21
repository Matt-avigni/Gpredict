#!/usr/bin/env python3
import argparse
import socketserver
import threading


class RigctldHandler(socketserver.StreamRequestHandler):
    def setup(self):
        super().setup()
        with self.server.conn_lock:
            self.server.state["conn_count"] += 1

    def handle(self):
        state = self.server.state
        while True:
            line = self.rfile.readline()
            if not line:
                break

            cmd_raw = line.decode("ascii", "ignore").rstrip("\n")
            cmd = cmd_raw.strip()
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
                    "has_set_vfo_opt: %d" % (1 if state["has_set_vfo_opt"] else 0),
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

            if cmd == "\\reset_conn_count":
                state["conn_count"] = 0
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            if cmd == "\\get_conn_count":
                self.wfile.write(
                    ("%d\nRPRT 0\n" % state["conn_count"]).encode("ascii")
                )
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

            if cmd == "\\reset_cmd_log":
                state["cmd_log"] = []
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            if cmd == "\\get_cmd_log":
                body = "\n".join(state["cmd_log"])
                if body:
                    body += "\n"
                self.wfile.write((body + "RPRT 0\n").encode("ascii"))
                self.wfile.flush()
                continue

            state["cmd_log"].append(cmd)

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
                if state["has_set_vfo_opt"]:
                    self.wfile.write(b"RPRT 0\n")
                else:
                    self.wfile.write(b"RPRT -11\n")
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
    parser.add_argument("--no-vfo-opt", action="store_true")
    args = parser.parse_args()

    server = RigctldServer((args.host, args.port), RigctldHandler)
    server.state = {
        "freq": 145800000,
        "vfo": "VFOA",
        "set_freq_count": 0,
        "conn_count": 0,
        "has_set_vfo_opt": not args.no_vfo_opt,
        "cmd_log": [],
    }
    server.conn_lock = threading.Lock()

    try:
        if args.once:
            server.handle_request()
        else:
            server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
