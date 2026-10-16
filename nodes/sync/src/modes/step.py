"""`step` mode: one tuple per observation step, of every input's message for that step.

Every input's messages carry `step` (an observation's index, or a message answering it, like a
controller's target). Once every input has sent a message with the same step, one tuple of them
goes out, with that `step`, and the `capture_time` of the oldest when they all have one. An
input that is a tuple itself (an observation) is joined in flat: its elements become the new
tuple's (`akai_tuple.build`), so a consumer gets one level, e.g. an observation's joint states
and images next to the target answering it.

A step some input never sent is dropped once a newer one completes, handled as `on_drop` says;
a message for a step older than the last tuple's is dropped the same way. At most `max_pending`
incomplete steps wait.

Inputs:  program_state, plus every id in `inputs`.
Outputs: <output> (the tuple), node_state.
"""

from __future__ import annotations

import logging
from collections.abc import Callable

import pyarrow as pa
from akai_tuple import build
from dora import Node

from bundler import report
from config import SyncConfig

logger = logging.getLogger("sync")


class StepMode:
    def __init__(self, cfg: SyncConfig):
        self.opts = cfg.mode
        self.node: Node | None = None
        self.state: str | None = None
        self._pending: dict[int, dict[str, dict]] = {}  # step -> input id -> its event
        self._last: int | None = None  # the last tuple's step
        self._handlers: dict[str, Callable[[dict], bool]] = {"program_state": self._on_program_state}
        for input_id in self.opts.inputs:
            self._handlers[input_id] = self._input_handler(input_id)

    # -- node protocol -------------------------------------------------------

    def start(self, node: Node) -> None:
        self.node = node
        self._set_state("ready")

    def handle(self, event: dict) -> bool:
        """Dispatch one INPUT event. True means: stop the node."""
        return self._handlers[event["id"]](event)  # KeyError on an unwired input id is a wiring bug

    def close(self) -> None:
        """Nothing to release."""

    # -- inputs --------------------------------------------------------------

    def _on_program_state(self, event: dict) -> bool:
        return event["value"][0].as_py() == "disconnect"

    def _input_handler(self, input_id: str) -> Callable[[dict], bool]:
        def on_input(event: dict) -> bool:
            if "step" not in event["metadata"]:
                raise ValueError(f"{input_id}: a message without `step`; step mode joins by it")
            step = event["metadata"]["step"]
            if self._last is not None and step <= self._last:
                report(
                    logger,
                    self.opts.on_drop,
                    f"{input_id}: step {step} came after step {self._last}'s tuple; dropped",
                )
                return False
            self._pending.setdefault(step, {})[input_id] = event
            if len(self._pending[step]) == len(self.opts.inputs):
                self._emit(step)
            elif len(self._pending) > self.opts.max_pending:
                oldest = min(self._pending)
                self._drop(oldest)
            return False

        return on_input

    def _emit(self, step: int) -> None:
        for older in sorted(s for s in self._pending if s < step):
            self._drop(older)
        pending = self._pending.pop(step)
        events = [pending[i] for i in self.opts.inputs]
        self._last = step
        values, metadata = build(self.opts.inputs, events)
        captured = [e["metadata"].get("capture_time") for e in events]
        if all(t is not None for t in captured):
            metadata["capture_time"] = min(captured)
        metadata["step"] = step
        self.node.send_output(self.opts.output, values, metadata=metadata)

    def _drop(self, step: int) -> None:
        missing = [i for i in self.opts.inputs if i not in self._pending.pop(step)]
        report(logger, self.opts.on_drop, f"step {step} dropped: no message from {missing}")

    def _set_state(self, state: str) -> None:
        """Edge-triggered node_state: emitted only when the state changes."""
        if state != self.state:
            self.state = state
            self.node.send_output("node_state", pa.array([state]))
