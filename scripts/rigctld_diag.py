#!/usr/bin/env python3
import socket
import time


MAIN_VFO_TOKENS = ["VFOA", "Main", "MainA", "VFO_MAIN"]
SUB_VFO_TOKENS = ["VFOB", "Sub", "SubA", "VFO_SUB"]


class RigctldConnection:
    def __init__(self, host, port, timeout_ms):
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self.sock = None
        self._buffer = b""

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def connect(self):
        if self.sock is not None:
            return
        timeout = max(self.timeout_ms, 1) / 1000.0
        self.sock = socket.create_connection((self.host, self.port), timeout)
        self.sock.settimeout(timeout)
        self._buffer = b""

    def close(self):
        if self.sock is None:
            return
        try:
            self.sock.close()
        finally:
            self.sock = None
            self._buffer = b""

    def _deadline(self, timeout_ms):
        timeout = self.timeout_ms if timeout_ms is None else timeout_ms
        return time.monotonic() + (max(timeout, 1) / 1000.0)

    def _readline(self, timeout_ms=None):
        if self.sock is None:
            raise RuntimeError("socket not connected")

        deadline = self._deadline(timeout_ms)
        while True:
            if b"\n" in self._buffer:
                line, self._buffer = self._buffer.split(b"\n", 1)
                return line.decode("ascii", "ignore").rstrip("\r")

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError()

            self.sock.settimeout(remaining)
            chunk = self.sock.recv(4096)
            if not chunk:
                if self._buffer:
                    line = self._buffer
                    self._buffer = b""
                    return line.decode("ascii", "ignore").rstrip("\r")
                raise ConnectionError("rigctld closed the connection")
            self._buffer += chunk

    def _send(self, cmd):
        if self.sock is None:
            raise RuntimeError("socket not connected")
        payload = cmd.rstrip("\n") + "\n"
        self.sock.sendall(payload.encode("ascii"))

    def request_rprt(self, cmd, timeout_ms=None):
        started = time.monotonic()
        self._send(cmd)
        lines = []
        timed_out = False
        rprt = None

        while True:
            try:
                line = self._readline(timeout_ms)
            except TimeoutError:
                timed_out = True
                break

            if line.startswith("RPRT"):
                parts = line.split()
                if len(parts) >= 2:
                    try:
                        rprt = int(parts[1])
                    except ValueError:
                        rprt = None
                else:
                    rprt = None
                break
            lines.append(line)

        elapsed_ms = (time.monotonic() - started) * 1000.0
        return {
            "cmd": cmd,
            "lines": lines,
            "rprt": rprt,
            "timed_out": timed_out,
            "elapsed_ms": elapsed_ms,
            "ok": (not timed_out and rprt == 0),
        }

    def request_blank_terminated(self, cmd, timeout_ms=None):
        started = time.monotonic()
        self._send(cmd)
        lines = []
        timed_out = False

        while True:
            try:
                line = self._readline(timeout_ms)
            except TimeoutError:
                timed_out = (len(lines) == 0)
                break

            if line == "":
                break
            lines.append(line)

        elapsed_ms = (time.monotonic() - started) * 1000.0
        return {
            "cmd": cmd,
            "lines": lines,
            "timed_out": timed_out,
            "elapsed_ms": elapsed_ms,
            "ok": (not timed_out and len(lines) > 0),
        }


def parse_vfo_candidates(lines):
    for line in lines:
        if line.lower().startswith("vfo list:"):
            return line.split(":", 1)[1].strip().split()
    return []


def format_result(result):
    if result["timed_out"]:
        return "timeout"
    if result.get("rprt") is None:
        return "no-rprt"
    return "RPRT %d" % result["rprt"]


def probe_token_once(host, port, timeout_ms, token, freq_hz=None):
    with RigctldConnection(host, port, timeout_ms) as conn:
        select_result = conn.request_rprt("V %s" % token, timeout_ms)
        freq_result = None
        if select_result["ok"] and freq_hz is not None:
            freq_result = conn.request_rprt("F %d" % freq_hz, timeout_ms)
        return select_result, freq_result


def probe_plain_set_once(host, port, timeout_ms, freq_hz):
    with RigctldConnection(host, port, timeout_ms) as conn:
        return conn.request_rprt("F %d" % freq_hz, timeout_ms)
