"""`tuple` mode — the default: one message holding a new message from every input.

The node assumes nothing about what the inputs carry. A tuple keeps values and metadata
apart, as every dora message does:

* value: a one-row Arrow struct with one field per input, named by its input id, in
  `inputs` order, holding that input's array (as a one-row list):
  `struct<cam_high: list<uint8>, arm_joint_state: list<double>>`;
* metadata: every key of every input's metadata, as `<input id>.<key>`
  (`cam_high.encoding`, `arm_joint_state.names`, `cam_high.timestamp`: dora's send time of
  that message), plus the tuple's own `capture_time`, the oldest of the inputs'
  `capture_time`s, when every input carries one.

so images, joint vectors and anything else travel together untouched, their metadata
keeping its types (dora's metadata is flat, so it cannot nest one dict per input).

When a tuple goes out, and what is done about repeats and skew, follows `bundler.py`.
Skew is measured by `skew_time`: the inputs' `capture_time`s, or dora's send timestamps.

Inputs:  program_state, plus every id in `inputs`.
Outputs: <output> (the tuple), node_state.
"""

from __future__ import annotations

import logging
from collections.abc import Callable

import pyarrow as pa
from dora import Node

from bundler import Bundler, SkewCheck
from config import SyncConfig

logger = logging.getLogger("sync")


def one_row(array: pa.Array) -> pa.ListArray:
    """`array` as the single row of a list array, the type of a tuple field."""
    return pa.ListArray.from_arrays(pa.array([0, len(array)], pa.int32()), array)


class TupleMode:
    def __init__(self, cfg: SyncConfig):
        self.opts = cfg.mode
        self.node: Node | None = None
        self.state: str | None = None
        self.bundler = Bundler(self.opts.inputs, self.opts.on_repeat, logger)
        self.skew = SkewCheck(self.opts.max_skew, self.opts.on_skew, self.opts.skew_time, logger)
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
            events = self.bundler.add(input_id, event)
            if events is not None:
                self._emit(events)
            return False

        return on_input

    def _emit(self, events: list[dict]) -> None:
        self.skew.check(events, self.opts.inputs)
        captured = [e["metadata"].get("capture_time") for e in events]
        stamped = all(t is not None for t in captured)
        values = pa.StructArray.from_arrays([one_row(e["value"]) for e in events], names=self.opts.inputs)
        metadata = {
            f"{input_id}.{key}": value
            for input_id, event in zip(self.opts.inputs, events)
            for key, value in event["metadata"].items()
        }
        if stamped:
            metadata["capture_time"] = min(captured)
        self.node.send_output(self.opts.output, values, metadata=metadata)

    def _set_state(self, state: str) -> None:
        """Edge-triggered node_state: emitted only when the state changes."""
        if state != self.state:
            self.state = state
            self.node.send_output("node_state", pa.array([state]))
