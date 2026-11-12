"""`step` mode: one tuple per observation step, of every input's message for that step.

Every input's messages carry `step` (an observation's index, or a message answering it, like a
controller's target). Once every input has sent a message with the same step, one tuple of them
goes out, with that `step`, and the `capture_time` of the oldest when they all have one. An
input that is a tuple itself (an observation) is joined in flat: its elements become the new
tuple's (`akai_tuple.build`), so a consumer gets one level, e.g. an observation's joint states
and images next to the target answering it. `elements`, when set, picks which of them go on.

A step some input never sent is dropped with a warning once a newer one completes; so is a
message for a step older than the last tuple's. At most 30 incomplete steps (a second of
observations) wait.

Inputs:  program_state, plus every id in `inputs`.
Outputs: <output> (the tuple), node_state.
"""

from __future__ import annotations

import logging
from collections.abc import Callable

from akai_node import Mode
from akai_tuple import build, select

from config import SyncConfig

logger = logging.getLogger("sync")


class StepMode(Mode):
    def __init__(self, cfg: SyncConfig):
        super().__init__()
        self.opts = cfg.mode
        self._pending: dict[int, dict[str, dict]] = {}  # step -> input id -> its event
        self._last: int | None = None  # the last tuple's step
        for input_id in self.opts.inputs:
            self.handlers[input_id] = self._input_handler(input_id)

    def _input_handler(self, input_id: str) -> Callable[[dict], bool]:
        def on_input(event: dict) -> bool:
            if "step" not in event["metadata"]:
                raise ValueError(f"{input_id}: a message without `step`; step mode joins by it")
            step = event["metadata"]["step"]
            if self._last is not None and step <= self._last:
                logger.warning("%s: step %d came after step %d's tuple; dropped", input_id, step, self._last)
                return False
            self._pending.setdefault(step, {})[input_id] = event
            if len(self._pending[step]) == len(self.opts.inputs):
                self._emit(step)
            elif len(self._pending) > 30:
                self._drop(min(self._pending))
            return False

        return on_input

    def _emit(self, step: int) -> None:
        for older in sorted(s for s in self._pending if s < step):
            self._drop(older)
        pending = self._pending.pop(step)
        events = [pending[i] for i in self.opts.inputs]
        self._last = step
        values, metadata = build(self.opts.inputs, events)
        if self.opts.elements is not None:
            values, metadata = select(values, metadata, self.opts.elements)
        captured = [e["metadata"].get("capture_time") for e in events]
        if all(t is not None for t in captured):
            metadata["capture_time"] = min(captured)
        metadata["step"] = step
        self.node.send_output(self.opts.output, values, metadata=metadata)

    def _drop(self, step: int) -> None:
        pending = self._pending.pop(step)
        missing = [i for i in self.opts.inputs if i not in pending]
        logger.warning("step %d dropped: no message from %s", step, missing)
