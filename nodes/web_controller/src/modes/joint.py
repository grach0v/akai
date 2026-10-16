"""`joint` mode — the default: drive every robot part in joint space from a web page.

The node answers every `observation` (a tuple, `akai_tuple`): its joint states (the elements
that are named vectors, `names` in their metadata) update the page, one card per element with a
slider per joint bounded by that joint's limits in the robot's URDF (`urdf`), and it sends every
part's target together, as one named vector `joint_target` carrying the observation's `step`, so
each target is the answer to one observation. Another part in the observation is another card.

Three parties share one `Board` under its lock:

* the dora loop (main thread) writes the measured joint values and the program
  state into it, and on every observation publishes the targets of all parts, but only
  while the program is `running`;
* the page's WebSocket (the web server's thread) is pushed a snapshot at
  `push_hz` and writes targets from the sliders and -/+ buttons into it;
* nothing else: targets are a latest-value slot per part, not a queue.

The loop is closed on the measured state:

* a part's target is seeded from its first measured state, so the robot does
  not jump when the page first publishes;
* targets are clamped to the joint limits;
* on every program-state change the targets are reset to the measured state, so
  a stale target from an earlier stage never drags the robot back.

The page's Disconnect button reports `disconnect` on `node_state`, which the
manager answers with `program_state == disconnect`.

Inputs:  observation (a tuple), program_state.
Outputs: joint_target (every part's, one named vector, with the observation's `step`), node_state.
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import pyarrow as pa
from akai_tuple import elements
from dora import Node
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

from config import JointModeConfig, WebControllerConfig
from server import WebServer

logger = logging.getLogger("web_controller")


@dataclass
class PartState:
    names: list[str]
    lower: list[float]
    upper: list[float]
    measured: list[float]
    target: list[float]


def urdf_limits(urdf: Path) -> dict[str, tuple[float, float]]:
    """(lower, upper) of every URDF joint that has both, by joint name."""
    limits = {}
    for joint in ET.parse(urdf).getroot().iter("joint"):
        limit = joint.find("limit")
        if limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
            limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return limits


class Board:
    """The state the dora loop and the page share. Every method takes the lock."""

    def __init__(self, opts: JointModeConfig):
        self._opts = opts
        self._limits = urdf_limits(opts.urdf)
        self._lock = threading.Lock()
        self._parts: dict[str, PartState] = {}
        self._program_state: str | None = None
        self._disconnect_requested = False

    # -- from the dora loop --------------------------------------------------

    def measured(self, part: str, names: list[str], values: list[float]) -> None:
        """A part's measured joint values. The first one adds the part, seeded with target = measured.
        A joint without limits in the URDF has nothing to bound its slider by, and raises."""
        with self._lock:
            state = self._parts.get(part)
            if state is None:
                unbounded = [n for n in names if n not in self._limits]
                if unbounded:
                    raise ValueError(f"{part}: no joint limits in {self._opts.urdf} for {unbounded}")
                self._parts[part] = PartState(
                    names=list(names),
                    lower=[self._limits[n][0] for n in names],
                    upper=[self._limits[n][1] for n in names],
                    measured=values,
                    target=list(values),
                )
                logger.info("part %s: %s", part, self._parts[part].names)
            else:
                state.measured = values

    def program_state(self, value: str) -> None:
        """A new program state resets every target to the measured state."""
        with self._lock:
            if value != self._program_state:
                self._program_state = value
                for state in self._parts.values():
                    state.target = list(state.measured)

    def targets(self) -> tuple[list[str], list[float]] | None:
        """Every part's joint names and targets, in the order the parts appeared, while the program
        is running (and some part is known), else None."""
        with self._lock:
            if self._program_state != "running" or not self._parts:
                return None
            names = [n for s in self._parts.values() for n in s.names]
            return names, [v for s in self._parts.values() for v in s.target]

    @property
    def disconnect_requested(self) -> bool:
        with self._lock:
            return self._disconnect_requested

    # -- from the page -------------------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "program_state": self._program_state,
                "step": self._opts.step,
                "parts": {
                    part: {
                        "names": s.names,
                        "lower": s.lower,
                        "upper": s.upper,
                        "measured": s.measured,
                        "target": s.target,
                    }
                    for part, s in self._parts.items()
                },
            }

    def command(self, msg: dict) -> None:
        """One message from the page: set / nudge / sync / disconnect. A part the node does
        not know is skipped with a warning: a page left open across a restart may still show it."""
        with self._lock:
            kind = msg["type"]
            if kind == "disconnect":
                self._disconnect_requested = True
            elif kind == "sync":
                for state in self._parts.values():
                    state.target = list(state.measured)
            elif kind not in ("set", "nudge"):
                logger.warning("page sent an unknown message type %r", kind)
            elif msg["part"] not in self._parts:
                logger.warning("page sent a target for unknown part %r", msg["part"])
            else:
                state, i = self._parts[msg["part"]], int(msg["index"])
                value = (
                    float(msg["value"]) if kind == "set" else state.target[i] + msg["dir"] * self._opts.step
                )
                state.target[i] = min(max(value, state.lower[i]), state.upper[i])


def build_app(board: Board, push_hz: float) -> FastAPI:
    app = FastAPI(title="web_controller")

    page_file = Path(__file__).resolve().parent.parent / "web" / "joint.html"

    @app.get("/")
    async def page() -> FileResponse:
        return FileResponse(page_file)

    @app.websocket("/ws")
    async def ws(socket: WebSocket) -> None:
        await socket.accept()

        async def push() -> None:
            while True:
                await socket.send_json(board.snapshot())
                await asyncio.sleep(1.0 / push_hz)

        pusher = asyncio.create_task(push())
        try:
            while True:
                board.command(await socket.receive_json())
        except WebSocketDisconnect:
            pass
        finally:
            pusher.cancel()

    return app


class JointMode:
    def __init__(self, cfg: WebControllerConfig):
        """Start the web server before joining the dataflow, so a taken port fails early."""
        self.opts = cfg.mode
        self.node: Node | None = None
        self.state: str | None = None
        self.board = Board(self.opts)
        self.server = WebServer(
            build_app(self.board, self.opts.push_hz), cfg.host, cfg.port, cfg.server_timeout
        )
        self._handlers: dict[str, Callable[[dict], bool]] = {
            "observation": self._on_observation,
            "program_state": self._on_program_state,
        }

    # -- node protocol -------------------------------------------------------

    def start(self, node: Node) -> None:
        self.node = node
        self._set_state("ready")

    def handle(self, event: dict) -> bool:
        """Dispatch one INPUT event. True means: stop the node."""
        return self._handlers[event["id"]](event)  # KeyError on an unwired input id is a wiring bug

    def close(self) -> None:
        self.server.stop()

    # -- inputs --------------------------------------------------------------

    def _on_observation(self, event: dict) -> bool:
        """Update the measured joints from the observation's named vectors, and answer it."""
        md = event["metadata"]
        for name, array, element_md in elements(event["value"], md):
            if "names" in element_md:
                self.board.measured(name, element_md["names"], array.to_pylist())
        if self.board.disconnect_requested:
            self._set_state("disconnect")
        targets = self.board.targets()
        if targets is not None:
            names, values = targets
            self.node.send_output(
                "joint_target",
                pa.array(values, type=pa.float64()),
                metadata={"names": names, "step": md["step"], "capture_time": time.time()},
            )
        return False

    def _on_program_state(self, event: dict) -> bool:
        value = event["value"][0].as_py()
        self.board.program_state(value)
        return value == "disconnect"

    def _set_state(self, state: str) -> None:
        """Edge-triggered node_state: emitted only when the state changes."""
        if state != self.state:
            self.state = state
            self.node.send_output("node_state", pa.array([state]))
