"""When a tuple goes out: the rules every sync mode follows (see the README):

1. When a message arrives, it replaces that input's previous one (the newest wins).
2. If every input has sent a new message since the last tuple, a tuple of them is
   emitted right away (no timer: it waits only for the slowest producer).
3. If some input has not, the node waits: no old message is resent, no partial tuple
   goes out, and nothing at all until every input has sent once.
4. If a producer stops sending, tuples stop too, on purpose: resending its last message
   would tell the consumer that a dead part is still alive and where it was.

Two conditions are reported as configured (`Action`): a *repeat*, an input sending again
before its tuple is complete (rule 1 drops its previous message), and *skew*, the messages
of a tuple further apart in time than `max_skew` (a producer lagging its peers).

It keeps the dora events as they arrived; a mode decides what goes out, this only decides when.
"""

from __future__ import annotations

import logging
from typing import Literal

# What to do about a condition: nothing, log a warning, or stop the node with an error.
Action = Literal["ignore", "warn", "error"]

# Which time skew is measured by: the producers' `capture_time`, or dora's send `timestamp`.
SkewTime = Literal["capture_time", "timestamp"]


def report(logger: logging.Logger, action: Action, text: str) -> None:
    if action == "warn":
        logger.warning(text)
    elif action == "error":
        raise RuntimeError(text)


class Bundler:
    def __init__(self, inputs: list[str], on_repeat: Action, logger: logging.Logger):
        self._inputs = inputs
        self._on_repeat, self._logger = on_repeat, logger
        self._latest: dict[str, dict] = {}  # input id -> its latest dora event
        self._fresh: set[str] = set()  # inputs updated since the last tuple

    def add(self, input_id: str, event: dict) -> list[dict] | None:
        """Keep `event` as the input's latest; the events of every input, in input order,
        when this completes a tuple, else None. `input_id` must be one of `inputs` (the modes
        only route those here), so a full `_fresh` means every input is fresh."""
        if input_id in self._fresh:
            report(
                self._logger,
                self._on_repeat,
                f"{input_id} sent again before the tuple was complete; its previous message is dropped",
            )
        self._latest[input_id] = event
        self._fresh.add(input_id)
        if len(self._fresh) != len(self._inputs):
            return None
        self._fresh.clear()
        return [self._latest[name] for name in self._inputs]


class SkewCheck:
    """Reports a lagging producer as configured: a warning once when the skew appears, not for
    every tuple while it lasts; an error at once. Times come from the `time_key` metadata of
    every message: `capture_time` (seconds) or dora's send `timestamp` (a datetime)."""

    def __init__(self, max_skew: float | None, on_skew: Action, time_key: SkewTime, logger: logging.Logger):
        self._max_skew, self._on_skew, self._time_key, self._logger = max_skew, on_skew, time_key, logger
        self._skewed = False

    def check(self, events: list[dict], inputs: list[str]) -> None:
        if self._max_skew is None:
            return
        times = [self._time(event, input_id) for event, input_id in zip(events, inputs)]
        skew = max(times) - min(times)
        skewed = skew > self._max_skew
        if skewed and not self._skewed:
            oldest = inputs[times.index(min(times))]
            report(
                self._logger,
                self._on_skew,
                f"tuple skew {skew * 1e3:.0f} ms by {self._time_key}, over max_skew (oldest: {oldest})",
            )
        self._skewed = skewed

    def _time(self, event: dict, input_id: str) -> float:
        if self._time_key not in event["metadata"]:
            raise ValueError(f"{input_id}: no `{self._time_key}` in its metadata to check skew by")
        time = event["metadata"][self._time_key]
        return time.timestamp() if self._time_key == "timestamp" else float(time)
