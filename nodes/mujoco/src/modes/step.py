"""`step` mode: one fixed period of simulated time per tick, prepared before the tick.

The first joint target after a send starts the next period on a background thread: physics
steps one `period`, then the cameras render. The tick sends what is ready, stamped with the
tick's capture_time. With no command, the tick steps on the last targets itself.

Inputs:  tick (every `period`), program_state, joint_target.
Outputs: as in `sim` mode.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable

import mujoco
from akai_node import Worker

from config import MujocoConfig
from modes.sim import SimMode

logger = logging.getLogger("mujoco")


class Stepper(Worker):
    """Advances the model by one period on request, and hands back what `capture` returns then.
    A period that is not a whole number of physics steps carries the rest into the next one
    (33 ms of 2 ms steps: 16 and 17 in turn), so the simulation keeps the tick's rate exactly."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        lock: threading.Lock,
        period: float,
        capture: Callable[[], tuple | None],
    ):
        super().__init__("mujoco-step")
        self._model, self._data, self._lock, self._period, self._capture = model, data, lock, period, capture
        self._owed = 0.0  # simulated seconds the advances so far fell short of
        self._todo: queue.SimpleQueue = queue.SimpleQueue()  # an advance to make
        self._done: queue.SimpleQueue = queue.SimpleQueue()  # what `capture` returned after it

    def advance(self) -> None:
        """Start the next advance; returns at once."""
        self._todo.put(None)

    def result(self) -> tuple | None:
        """Wait for the started advance: what `capture` returned, or None if this thread has died."""
        while True:
            try:
                return self._done.get(timeout=0.1)
            except queue.Empty:
                if self.ended:
                    return None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._todo.get(timeout=0.1)
            except queue.Empty:
                continue
            self._owed += self._period
            steps = int(self._owed / self._model.opt.timestep + 1e-9)
            self._owed -= steps * self._model.opt.timestep
            with self._lock:
                for _ in range(steps):
                    mujoco.mj_step(self._model, self._data)
            self._done.put(self._capture())


class StepMode(SimMode):
    def __init__(self, cfg: MujocoConfig):
        super().__init__(cfg)
        self._advancing = False  # an advance is started and not yet sent
        logger.info(
            "step: %.3f s per tick, of %.4f s physics steps", self.opts.period, self.model.opt.timestep
        )

    def _physics(self) -> Stepper:
        if self.opts.period < self.model.opt.timestep:
            raise ValueError(f"MODE__PERIOD {self.opts.period} s is under one physics step")
        return Stepper(self.model, self.data, self._lock, self.opts.period, self._render)

    def _render(self) -> tuple | None:
        """On the stepper thread: the capture of the advanced simulation, with its frames."""
        capture = self._capture()
        if self.cameras is None:
            return capture, {}
        return self.cameras.render(capture)

    def _advance(self) -> None:
        if not self._advancing:
            self._advancing = True
            self.physics.advance()

    # -- inputs --------------------------------------------------------------

    def _on_tick(self, event: dict) -> bool:
        if any(worker.ended for worker in self._workers):
            self.set_state("finished")  # a background thread died; its traceback is in the log
            return False
        self._advance()  # no command started it: advance on the last targets
        done = self.physics.result()  # ready by now when a command started it
        self._advancing = False
        if done is not None:
            (_, positions), frames = done
            self._publish((time.time(), positions), frames)
        return False

    def _on_joint_target(self, event: dict) -> bool:
        super()._on_joint_target(event)
        self._advance()  # the first target after a send starts the next period from it
        return False
