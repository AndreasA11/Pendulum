"""A small client for the course rosbridge protocol, for your Python tests.

    from rosbridge_client import Rosbridge

    with Rosbridge() as ros:                      # connects to 127.0.0.1:9095
        reply = ros.call_service("/heap_sort", {"numbers": [3, 1, 2]})
        assert reply["result"] is True, reply
        assert reply["values"]["sorted"] == [1, 2, 3], reply

Messages are plain dicts, one JSON object per line, exactly as in
ROSBRIDGE_PROTOCOL.md. Every wait has a timeout and raises TimeoutError
instead of hanging. Standard library only. This file is course-supplied;
you may change it.
"""

from __future__ import annotations

import itertools
import json
import os
from pathlib import Path
import socket
import subprocess
import time
from typing import Any, Callable

_ids = itertools.count(1)


class Rosbridge:
    def __init__(self, host: str = "127.0.0.1", port: int = 9095, timeout: float = 5.0, links: int = 2) -> None:
        self.host = host
        self.port = port
        self.proc: subprocess.Popen | None = None
        self.sock: socket.socket | None = None

        # Check if runtime is already listening
        try:
            self.sock = socket.create_connection((host, port), timeout=0.2)
        except OSError:
            self._start_runtime(links)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    self.sock = socket.create_connection((host, port), timeout=0.3)
                    break
                except OSError:
                    time.sleep(0.05)
            if self.sock is None:
                if self.proc:
                    try:
                        self.proc.kill()
                    except Exception:
                        pass
                raise ConnectionError(f"Could not connect to {host}:{port}")

        self.buffer = b""
        # Messages read while waiting for something else, kept for later waits.
        self.pending: list[dict] = []

    def _start_runtime(self, links: int) -> None:
        project_root = Path(__file__).resolve().parent.parent
        binary = project_root / "build" / "arm_sim"
        if not binary.is_file():
            subprocess.run(["make", "build"], cwd=project_root, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        env = os.environ.copy()
        env["ARM_SIM_LINKS"] = str(links)
        env["ARM_SIM_PORT"] = str(self.port)
        self.proc = subprocess.Popen(
            [str(binary)],
            cwd=project_root,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def __enter__(self) -> "Rosbridge":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        if self.proc:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=1.0)
            except Exception:
                try:
                    self.proc.kill()
                    self.proc.wait(timeout=1.0)
                except Exception:
                    pass
            self.proc = None

    def send(self, message: dict) -> None:
        assert self.sock is not None, "Connection is closed"
        self.sock.sendall(json.dumps(message).encode("utf-8") + b"\n")

    def _read(self, deadline: float) -> dict:
        """The next message from the gateway, waiting at most until deadline."""
        assert self.sock is not None, "Connection is closed"
        while b"\n" not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("no message from the gateway in time")
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                raise TimeoutError("no message from the gateway in time") from None
            if not chunk:
                raise ConnectionError("the gateway closed the connection")
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line)

    def wait_for(self, match: Callable[[dict], bool], timeout: float = 5.0) -> dict:
        """The first message (already received or arriving within timeout) for which match is true."""
        for index, message in enumerate(self.pending):
            if match(message):
                return self.pending.pop(index)
        deadline = time.monotonic() + timeout
        while True:
            message = self._read(deadline)
            if match(message):
                return message
            self.pending.append(message)
            del self.pending[:-1000]  # busy topics: keep only the newest

    def call_service(self, service: str, args: Any, timeout: float = 10.0) -> dict:
        """Call a service and return its whole service_response message."""
        call_id = f"call-{next(_ids)}"
        self.send({"op": "call_service", "service": service, "id": call_id, "args": args})
        return self.wait_for(
            lambda m: m.get("op") == "service_response" and m.get("id") == call_id, timeout)

    def advertise(self, topic: str, msg_type: str) -> None:
        self.send({"op": "advertise", "topic": topic, "type": msg_type})

    def publish(self, topic: str, msg: Any) -> None:
        self.send({"op": "publish", "topic": topic, "msg": msg})

    def subscribe(self, topic: str, msg_type: str, timeout: float = 5.0) -> None:
        """Subscribe and wait for the gateway's status saying it is registered."""
        sub_id = f"sub-{next(_ids)}"
        self.send({"op": "subscribe", "topic": topic, "type": msg_type, "id": sub_id})
        self.wait_for(lambda m: m.get("op") == "status" and m.get("id") == sub_id, timeout)

    def drain_pending(self, topic: str | None = None) -> None:
        if topic is None:
            self.pending.clear()
        else:
            self.pending = [m for m in self.pending if not (m.get("op") == "publish" and m.get("topic") == topic)]

    def latest_message(self, topic: str, timeout: float = 5.0) -> Any:
        self.drain_pending(topic)
        return self.next_message(topic, timeout)

    def next_message(self, topic: str, timeout: float = 5.0,
                     match: Callable[[Any], bool] = lambda msg: True) -> Any:
        """The msg of the next publication on a subscribed topic for which match is true."""
        return self.wait_for(
            lambda m: m.get("op") == "publish" and m.get("topic") == topic and match(m.get("msg")),
            timeout)["msg"]
