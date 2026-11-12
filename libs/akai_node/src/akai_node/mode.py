"""The node protocol: a node runs one mode, picked by its config's `mode.name`.

1. `Mode(cfg)` opens the mode's resources before the node joins the dataflow: dora starts the
   timers once every node has joined, so nothing ticks into a node still opening hardware.
2. `start(node)` keeps the node and reports `ready`.
3. `handle(event)` dispatches one INPUT event by its id to `handlers`; True stops the node.
   An input id with no handler is a wiring bug and raises.
4. `close()` releases the resources, however the loop ends.

Every mode stops on `program_state == disconnect`, and reports its own state on `node_state`,
edge-triggered.
"""

from __future__ import annotations

import logging
from collections.abc import Callable

import pyarrow as pa
from dora import Node
from pydantic_settings import BaseSettings


class Mode:
    def __init__(self) -> None:
        self.node: Node | None = None
        self.state: str | None = None
        self.handlers: dict[str, Callable[[dict], bool]] = {"program_state": self.on_program_state}

    def start(self, node: Node) -> None:
        self.node = node
        self.set_state("ready")

    def handle(self, event: dict) -> bool:
        return self.handlers[event["id"]](event)

    def close(self) -> None:
        pass

    def on_program_state(self, event: dict) -> bool:
        return event["value"][0].as_py() == "disconnect"

    def set_state(self, state: str) -> None:
        """Report `state` on `node_state`, only when it changes."""
        if state != self.state:
            self.state = state
            self.node.send_output("node_state", pa.array([state]))


def run(modes: dict[str, type], config: type[BaseSettings]) -> None:
    """Read the config (env, then CLI flags), build its mode, join the dataflow and run the loop."""
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = config(_cli_parse_args=True)
    mode = modes[cfg.mode.name](cfg)
    node = Node()
    mode.start(node)
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] == "INPUT" and mode.handle(event):
                break
    finally:
        mode.close()
