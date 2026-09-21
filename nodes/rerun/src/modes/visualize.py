"""`visualize` mode — open a rerun viewer on a running `record` server.

The viewer is the process rerun-sdk installs next to this interpreter, started
in its own session with the record node's gRPC url. A viewer does not retry a
failed connection, so the node first waits until the server accepts
connections, then launches. The viewer shows the live stream and the blueprint
the recorder sent, and is closed again when this node stops. Nothing is
logged from here: the recorder is the only writer, so the viewer can run on
this machine or on any other that can reach the recorder's port.

The viewer window is the operator's UI, so its state is reported like any
producer's: `ready` once it is spawned, `finished` once the operator closes it
(noticed on `tick`). What that means for the program is the manager's call.

Inputs:  tick (dora/timer, to notice a closed window), program_state.
Outputs: node_state.
"""

from __future__ import annotations

import logging
import os
import signal
import socket
import subprocess
import sys
import time
from collections.abc import Callable
from pathlib import Path
from urllib.parse import urlparse

import pyarrow as pa
from dora import Node

from config import RerunConfig, VisualizeModeConfig

logger = logging.getLogger("rerun")


def viewer_binary() -> Path:
    """The `rerun` viewer CLI that rerun-sdk installs next to this interpreter."""
    path = Path(sys.executable).parent / "rerun"
    if not path.is_file():
        raise RuntimeError(f"rerun viewer binary not found at {path}")
    return path


def wait_for_server(url: str, timeout: float) -> None:
    """Block until the gRPC server behind `rerun+http://host:port/proxy` accepts TCP connections."""
    parsed = urlparse(url.removeprefix("rerun+"))
    deadline = time.monotonic() + timeout
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            if s.connect_ex((parsed.hostname, parsed.port)) == 0:
                return
        if time.monotonic() > deadline:
            raise RuntimeError(f"no rerun server at {url} after {timeout}s (is the record node running?)")
        time.sleep(0.2)


def start_viewer(opts: VisualizeModeConfig) -> subprocess.Popen:
    """Launch the viewer connected to the server url, in its own session so
    dora's stop neither waits on nor signals the window; its lifetime is ours
    (`stop_viewer`). `--connect` matters: a viewer always hosts a gRPC server of
    its own, and only with `--connect` does it pick a free port for it instead
    of the default 9876, which is the recorder's. The viewer's own output stays
    on this node's stderr so a failure to open or connect shows in the dora log."""
    wait_for_server(opts.server_url, opts.server_timeout)
    return subprocess.Popen(
        [str(viewer_binary()), "--connect", opts.server_url, "--memory-limit", opts.memory_limit],
        start_new_session=True,
    )


def stop_viewer(proc: subprocess.Popen) -> None:
    """End the viewer. The venv's `rerun` entry point is a launcher that starts
    the real viewer as a child, so signal its whole process group."""
    pgid = os.getpgid(proc.pid)
    os.killpg(pgid, signal.SIGTERM)
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        os.killpg(pgid, signal.SIGKILL)
        proc.wait(timeout=2)


class VisualizeMode:
    def __init__(self, cfg: RerunConfig):
        """Wait for the record server and open the viewer before joining the dataflow."""
        self.opts = cfg.mode
        self.node: Node | None = None
        self.state: str | None = None
        self._handlers: dict[str, Callable[[dict], bool]] = {
            "tick": self._on_tick,
            "program_state": self._on_program_state,
        }
        self._viewer = start_viewer(self.opts)
        logger.info("visualize: viewer on %s", self.opts.server_url)

    def start(self, node: Node) -> None:
        self.node = node
        self._set_state("ready")

    def handle(self, event: dict) -> bool:
        """Dispatch one INPUT event. True means: stop the node."""
        return self._handlers[event["id"]](event)  # KeyError on an unwired input id is a wiring bug

    def close(self) -> None:
        if self._viewer.poll() is None:  # still open: close it
            stop_viewer(self._viewer)

    def _on_tick(self, event: dict) -> bool:
        if self._viewer.poll() is not None:  # the operator closed the window
            self._set_state("finished")
        return False

    def _on_program_state(self, event: dict) -> bool:
        return event["value"][0].as_py() == "disconnect"

    def _set_state(self, state: str) -> None:
        """Edge-triggered node_state: emitted only when the state changes."""
        if state != self.state:
            self.state = state
            self.node.send_output("node_state", pa.array([state]))
