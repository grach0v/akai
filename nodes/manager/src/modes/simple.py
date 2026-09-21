"""`simple` mode — the minimal program lifecycle.

States: `boot` (initial), `running`, `disconnect` (final).

- `boot` -> `running` once every producer has reported `ready`. Readiness is a
  latch: a producer counts once it has said `ready`, even if its state moved on.
- any state -> `disconnect` when any input reports `finished` (it has nothing
  more to produce, e.g. a camera's device stopped delivering) or `disconnect`
  (an operator request). Every node tears down on `disconnect`, so the dataflow
  finishes.

`program_state` is broadcast on every state entry, including the initial `boot`.
"""

from __future__ import annotations

import logging

import pyarrow as pa
from dora import Node
from statemachine import State, StateMachine

from config import ManagerConfig

logger = logging.getLogger("manager")


class SimpleProgram(StateMachine):
    boot = State(value="boot", initial=True)
    running = State(value="running")
    disconnect = State(value="disconnect", final=True)

    run = boot.to(running, cond="all_ready")
    end = boot.to(disconnect, cond="end_requested") | running.to(disconnect, cond="end_requested")

    def __init__(self, cfg: ManagerConfig):
        self.producers = set(cfg.mode.producers)
        self.node: Node | None = None
        self.tokens: dict[str, str] = {}  # input id -> latest token
        self.ready: set[str] = set()  # producers that have reported `ready`
        super().__init__()  # enters `boot`; broadcast once the node exists, in start()

    # -- node protocol -----------------------------------------------------------

    def start(self, node: Node) -> None:
        self.node = node
        self._broadcast()

    def handle(self, event: dict) -> bool:
        """One node_state event: record it, fire every enabled transition. True once final."""
        input_id, token = event["id"], event["value"][0].as_py()
        logger.info("%s -> %s", input_id, token)
        self.tokens[input_id] = token
        if token == "ready":
            self.ready.add(input_id)
        for transition in self.enabled_events():
            self.send(transition.id)
        return self.is_terminated

    def close(self) -> None:
        """Nothing to release."""

    # -- transition conditions -------------------------------------------------

    def all_ready(self) -> bool:
        return self.ready >= self.producers

    def end_requested(self) -> bool:
        return any(token in ("finished", "disconnect") for token in self.tokens.values())

    # -- actions ----------------------------------------------------------------

    def on_enter_state(self, state: State, **kwargs) -> None:
        if self.node is not None:  # the initial `boot` is entered before the node exists
            self._broadcast()

    def _broadcast(self) -> None:
        (state,) = self.configuration
        logger.info("program_state = %s", state.value)
        self.node.send_output("program_state", pa.array([state.value]))
