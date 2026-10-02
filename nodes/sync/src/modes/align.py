"""`align` mode: one tuple per reference time, of every input's sample nearest it in time.

A reference input (typically the observation clock, a dora timer shared with the producers)
gives the reference times; every sample's time is its `capture_time` (or, by `input_time`,
dora's send `timestamp`). Delivery delays then cost latency, never alignment. The rules:

1. Every input keeps its samples of the last `history` seconds.
2. A reference message adds its reference time (`reference_time`: dora's send `timestamp`, or its
   `capture_time`) to the pending ones.
3. A pending reference time t is settled once every input has a sample at or after t (so the
   nearest is known), or once `timeout` has passed since t (then the nearest so far). Pending
   ones settle in order.
4. A settled t becomes one tuple (see `tuples.py`): every input's sample nearest t, with the
   tuple's `capture_time` = t. A tuple with an element further than `max_skew` from t is
   handled as `on_skew` says (a lagging or stopped producer): one warning or error per such
   tuple, naming every element over it.
5. Until every input has sent at least once, reference times are dropped: there is nothing to
   pair them with.

Inputs:  program_state, the `reference` input, and every id in `inputs`.
Outputs: <output> (the tuple), node_state.
"""

from __future__ import annotations

import logging
import time
from collections import deque
from collections.abc import Callable

import pyarrow as pa
from dora import Node

import tuples
from bundler import report
from config import SyncConfig

logger = logging.getLogger("sync")


class AlignMode:
    def __init__(self, cfg: SyncConfig):
        self.opts = cfg.mode
        self.node: Node | None = None
        self.state: str | None = None
        # input id -> its (time, event) samples, oldest first
        self.history: dict[str, deque[tuple[float, dict]]] = {i: deque() for i in self.opts.inputs}
        self.pending: deque[float] = deque()  # reference times not yet settled, oldest first
        self._handlers: dict[str, Callable[[dict], bool]] = {
            "program_state": self._on_program_state,
            self.opts.reference: self._on_reference,
        }
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

    def _on_reference(self, event: dict) -> bool:
        self.pending.append(_time(event, self.opts.reference_time))
        self._settle()
        return False

    def _input_handler(self, input_id: str) -> Callable[[dict], bool]:
        def on_input(event: dict) -> bool:
            samples = self.history[input_id]
            newest = _time(event, self.opts.input_time)
            samples.append((newest, event))
            while samples[0][0] < newest - self.opts.history:
                samples.popleft()
            self._settle()
            return False

        return on_input

    # -- aligning ------------------------------------------------------------

    def _settle(self) -> None:
        """Turn every pending reference time that can be settled into a tuple, in order."""
        now = time.time()
        while self.pending:
            t = self.pending[0]
            if any(not samples for samples in self.history.values()):
                self.pending.popleft()  # rule 5: some input has not sent yet
                continue
            timed_out = now - t >= self.opts.timeout
            if not timed_out and any(s[-1][0] < t for s in self.history.values()):
                return  # rule 3: some input may still send a sample nearer t
            self.pending.popleft()
            self._emit(t, [min(self.history[i], key=lambda s: abs(s[0] - t)) for i in self.opts.inputs])

    def _emit(self, t: float, samples: list[tuple[float, dict]]) -> None:
        self._check_skew({i: sample_time - t for i, (sample_time, _) in zip(self.opts.inputs, samples)})
        values, metadata = tuples.build(self.opts.inputs, [event for _, event in samples])
        metadata["capture_time"] = t
        self.node.send_output(self.opts.output, values, metadata=metadata)

    def _check_skew(self, offsets: dict[str, float]) -> None:
        """Handle a tuple with elements over max_skew as `on_skew` says: one warning (or error) per
        such tuple, naming every element over it."""
        if self.opts.max_skew is None:
            return
        over = {i: o for i, o in offsets.items() if abs(o) > self.opts.max_skew}
        if over:
            elements = ", ".join(f"{i} {o * 1e3:+.0f} ms" for i, o in over.items())
            report(logger, self.opts.on_skew, f"tuple over max_skew, off its reference time: {elements}")

    def _set_state(self, state: str) -> None:
        """Edge-triggered node_state: emitted only when the state changes."""
        if state != self.state:
            self.state = state
            self.node.send_output("node_state", pa.array([state]))


def _time(event: dict, key: str) -> float:
    """An event's time in seconds: its `capture_time`, or dora's send `timestamp` (a datetime)."""
    if key not in event["metadata"]:
        raise ValueError(f"{event['id']}: no `{key}` in its metadata to align by")
    t = event["metadata"][key]
    return t.timestamp() if key == "timestamp" else float(t)
