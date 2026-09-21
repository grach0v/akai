"""manager node — owns the program lifecycle as an explicit state machine.

Other nodes emit their logical `node_state` (edge-triggered tokens such as
`ready`, `finished`). The manager feeds each of those events to its program
state machine, which advances itself and broadcasts `program_state` on every
state entry. There is no heartbeat: dora detects node death by itself.

Every mode is a state machine with the node protocol start(node) / handle(event)
/ close(); `handle` returns True once the program has reached its final state.

Inputs:  one per watched node, wired to that node's `<name>_node_state`
         (or any other single-token utf8 stream, e.g. a controller's command).
Outputs: program_state   utf8[1], e.g. boot / running / disconnect
"""

from __future__ import annotations

import logging
import sys

from dora import Node

from config import load_config
from modes import MODES


def run(node: Node, mode) -> None:
    mode.start(node)
    try:
        for event in node:
            if event["type"] == "STOP":
                break
            if event["type"] == "INPUT" and mode.handle(event):
                break
    finally:
        mode.close()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = load_config()
    mode = MODES[cfg.mode.name](cfg)  # opens its resources first; KeyError on an unknown mode is intended
    node = Node()  # joining the dataflow: dora starts timers once every node has joined
    run(node, mode)
    return 0


if __name__ == "__main__":
    sys.exit(main())
