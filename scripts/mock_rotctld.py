#!/usr/bin/env python3
import argparse
import socketserver
import threading
import time


class RotctldHandler(socketserver.StreamRequestHandler):
    def setup(self):
        super().setup()
        self.command_count = 0
        with self.server.conn_lock:
            self.server.state["conn_count"] += 1

    def send_reply(self, text):
        encoded = text.encode("ascii")

        if not self.server.state.get("split_replies"):
            self.wfile.write(encoded)
            self.wfile.flush()
            return

        chunk_size = max(1, len(encoded) // 2)
        for idx in range(0, len(encoded), chunk_size):
            self.wfile.write(encoded[idx : idx + chunk_size])
            self.wfile.flush()
            if idx + chunk_size < len(encoded):
                time.sleep(0.01)

    def maybe_disconnect(self):
        disconnect_after = self.server.state.get("disconnect_after", 0)

        self.command_count += 1
        return disconnect_after > 0 and self.command_count >= disconnect_after

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
                    "done",
                    "",
                ]
                self.send_reply("\n".join(dump))
                if self.maybe_disconnect():
                    break
                continue

            if cmd == "\\reset_conn_count":
                state["conn_count"] = 0
                self.send_reply("RPRT 0\n")
                if self.maybe_disconnect():
                    break
                continue

            if cmd == "\\get_conn_count":
                self.send_reply("%d\nRPRT 0\n" % state["conn_count"])
                if self.maybe_disconnect():
                    break
                continue

            if cmd == "p":
                if state.get("fail_get_pos"):
                    self.send_reply("RPRT -6\n")
                    if self.maybe_disconnect():
                        break
                    continue
                reply = "%0.1f\n%0.1f\n" % (
                    state["az"],
                    state["el"],
                )
                self.send_reply(reply)
                if self.maybe_disconnect():
                    break
                continue

            if cmd.startswith("P "):
                if state.get("drop_set_pos"):
                    if self.maybe_disconnect():
                        break
                    continue
                if state.get("fail_set_pos"):
                    self.send_reply("RPRT -6\n")
                    if self.maybe_disconnect():
                        break
                    continue
                parts = cmd.split()
                if len(parts) >= 3:
                    try:
                        state["az"] = float(parts[1])
                        state["el"] = float(parts[2])
                    except ValueError:
                        pass
                self.send_reply("RPRT 0\n")
                if self.maybe_disconnect():
                    break
                continue

            if cmd == "S":
                self.send_reply("RPRT 0\n")
                if self.maybe_disconnect():
                    break
                continue

            self.send_reply("RPRT 0\n")
            if self.maybe_disconnect():
                break


class RotctldServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


class SingleClientRotctldServer(socketserver.TCPServer):
    allow_reuse_address = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4533)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--fail-get-pos", action="store_true")
    parser.add_argument("--fail-set-pos", action="store_true")
    parser.add_argument("--drop-set-pos", action="store_true")
    parser.add_argument("--split-replies", action="store_true")
    parser.add_argument("--disconnect-after", type=int, default=0)
    args = parser.parse_args()

    server_cls = SingleClientRotctldServer if args.once else RotctldServer
    server = server_cls((args.host, args.port), RotctldHandler)
    server.state = {
        "az": 0.0,
        "el": 0.0,
        "conn_count": 0,
        "fail_get_pos": args.fail_get_pos,
        "fail_set_pos": args.fail_set_pos,
        "drop_set_pos": args.drop_set_pos,
        "split_replies": args.split_replies,
        "disconnect_after": args.disconnect_after,
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
