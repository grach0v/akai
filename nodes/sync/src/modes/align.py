"""`align` mode: one tuple per reference time, of every input's sample nearest it in time.

A reference input (typically the observation clock, a dora timer shared with the producers)
gives the reference times. A sample's time is, by `input_time`, dora's send `timestamp` (every
message has one) or its `capture_time` (when it was taken): aligned by capture time, delivery
delays cost latency, never alignment. The rules:

1. Every input keeps its samples of the last second.
2. A reference message adds its reference time (`reference_time`: dora's send `timestamp`, or its
   `capture_time`) to the pending ones.
3. A pending reference time t is settled once every input has a sample at or after t (so the
   nearest is known), or 0.2 s after t (then the nearest so far). Pending ones settle in order.
4. A settled t becomes one tuple (`akai_tuple`), numbered by `step`: every input's sample nearest
   t, with the tuple's `capture_time` = t. A tuple with an element further than `max_skew` from t
   is handled as `on_skew` says (a lagging or stopped producer): one warning or error per such
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

from akai_node import Mode, message_time
from akai_tuple import build

from config import SyncConfig

logger = logging.getLogger("sync")


class AlignMode(Mode):
    def __init__(self, cfg: SyncConfig):
        super().__init__()
        self.opts = cfg.mode
        # input id -> its (time, event) samples, oldest first
        self.history: dict[str, deque[tuple[float, dict]]] = {i: deque() for i in self.opts.inputs}
        self.pending: deque[float] = deque()  # reference times not yet settled, oldest first
        self.step = 0  # the next tuple's index
        self.handlers[self.opts.reference] = self._on_reference
        for input_id in self.opts.inputs:
            self.handlers[input_id] = self._input_handler(input_id)
        if self.opts.reference in self.opts.inputs:  # also aligned: its sample first, then its time
            sample = self.handlers[self.opts.reference]

            def sample_and_reference(event: dict) -> bool:
                sample(event)
                return self._on_reference(event)

            self.handlers[self.opts.reference] = sample_and_reference

    def _on_reference(self, event: dict) -> bool:
        self.pending.append(message_time(event["metadata"], self.opts.reference_time, event["id"]))
        self._settle()
        return False

    def _input_handler(self, input_id: str) -> Callable[[dict], bool]:
        def on_input(event: dict) -> bool:
            samples = self.history[input_id]
            newest = message_time(event["metadata"], self.opts.input_time, input_id)
            samples.append((newest, event))
            while samples[0][0] < newest - 1.0:  # rule 1: a second is plenty to choose from
                samples.popleft()
            self._settle()
            return False

        return on_input

    def _settle(self) -> None:
        """Turn every pending reference time that can be settled into a tuple, in order."""
        now = time.time()
        while self.pending:
            t = self.pending[0]
            if any(not samples for samples in self.history.values()):
                self.pending.popleft()  # rule 5: some input has not sent yet
                continue
            # rule 3: wait for a sample nearer t, but not longer than a few ticks' worth
            timed_out = now - t >= 0.2
            if not timed_out and any(s[-1][0] < t for s in self.history.values()):
                return
            self.pending.popleft()
            self._emit(t, [min(self.history[i], key=lambda s: abs(s[0] - t)) for i in self.opts.inputs])

    def _emit(self, t: float, samples: list[tuple[float, dict]]) -> None:
        self._check_skew({i: sample_time - t for i, (sample_time, _) in zip(self.opts.inputs, samples)})
        values, metadata = build(self.opts.inputs, [event for _, event in samples])
        metadata["capture_time"] = t
        metadata["step"], self.step = self.step, self.step + 1
        self.node.send_output(self.opts.output, values, metadata=metadata)

    def _check_skew(self, offsets: dict[str, float]) -> None:
        if self.opts.max_skew is None:
            return
        over = {i: o for i, o in offsets.items() if abs(o) > self.opts.max_skew}
        if over:
            elements = ", ".join(f"{i} {o * 1e3:+.0f} ms" for i, o in over.items())
            text = f"tuple over max_skew, off its reference time: {elements}"
            if self.opts.on_skew == "error":
                raise RuntimeError(text)
            logger.warning(text)
