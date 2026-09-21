"""rerun node — dora skeleton.

Reads the config, builds the selected mode and runs the event loop. The mode
owns the behaviour and its resources (the recording and its gRPC server, or
the viewer process); this loop is mode-agnostic. Per-mode inputs and outputs
are documented in the mode files.
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
