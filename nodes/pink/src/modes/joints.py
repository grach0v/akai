"""`joints` mode — the default: pass joint targets on to the robot, safely.

Every incoming `step` tuple (sync's `step` mode: one observation step's measured joint states and
the target answering it, element `target`) is answered at once with the `joint_target` sent (the
command): one pink IK step from the last command toward the target, over the time since the
previous one (capped at 0.1 s). The step is a posture task on the target, solved under

1. each joint's URDF velocity limit times `speed_scale`, so a streamed target is speed-limited,
2. the joint limits,
3. a collision barrier (with itself and whatever else the URDF holds, a cell's table and floor):
   the robot slows down near contact and keeps `min_distance`, sliding along rather than
   stopping. It acts on the `barrier_pairs` closest pairs.

The barrier is not a hard guarantee at one step per target, so a step that still collides, or
that the solver finds no solution for, is not taken: the last command is held and the reason
reported once.

The command starts from the measured joint states (the tuple's other named vectors), so the
first one does not jump, and carries the tuple's `step`.

Inputs:  step (a tuple), program_state.
Outputs: joint_target (a named vector of every actuated joint, with `capture_time` and the
         tuple's `step`), node_state.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable

import numpy as np
import pink
import pinocchio as pin
import pyarrow as pa
from akai_tuple import elements
from dora import Node
from pink.barriers import SelfCollisionBarrier
from pink.exceptions import NoSolutionFound
from pink.limits import ConfigurationLimit, VelocityLimit
from pink.tasks import PostureTask

from config import PinkConfig
from robot import Robot

logger = logging.getLogger("pink")


class JointsMode:
    def __init__(self, cfg: PinkConfig):
        """Load the robot model and set up the IK problem before joining the dataflow."""
        self.opts = cfg.mode
        self.node: Node | None = None
        self.state: str | None = None
        self.robot = Robot(cfg.urdf)
        model = self.robot.model
        self._index = {name: i for i, name in enumerate(self.robot.names)}
        self._posture = PostureTask(cost=1.0)
        self._limits = [
            ConfigurationLimit(model),
            VelocityLimit(model, self.robot.velocity * self.opts.speed_scale),
        ]
        self._barriers = [
            SelfCollisionBarrier(
                n_collision_pairs=min(self.opts.barrier_pairs, len(self.robot.geom.collisionPairs)),
                gain=self.opts.barrier_gain,
                d_min=self.opts.min_distance,
            )
        ]
        self._ik: pink.Configuration | None = None  # at the last command
        self._target: np.ndarray | None = None  # what is asked for
        self._held: str | None = None  # why the command is held, if it is
        self._last_target: float | None = None  # when the previous target arrived (monotonic)
        self._handlers: dict[str, Callable[[dict], bool]] = {
            "step": self._on_step,
            "program_state": self._on_program_state,
        }
        logger.info("joints: %s, %d collision pairs", self.robot.names, len(self.robot.geom.collisionPairs))

    # -- node protocol -------------------------------------------------------

    def start(self, node: Node) -> None:
        self.node = node
        self._set_state("ready")

    def handle(self, event: dict) -> bool:
        """Dispatch one INPUT event. True means: stop the node."""
        return self._handlers[event["id"]](event)  # KeyError on an unwired input id is a wiring bug

    def close(self) -> None:
        pass

    # -- inputs --------------------------------------------------------------

    def _on_step(self, event: dict) -> bool:
        """Update the joints the target names (the others keep their last target), and send the
        command one step toward it."""
        target, measured = None, {}
        for name, array, md in elements(event["value"], event["metadata"]):
            if name == self.opts.target:
                target = (md["names"], array.to_pylist())
            elif "names" in md:
                measured.update(zip(md["names"], array.to_pylist()))
        if target is None:
            raise ValueError(f"step: the tuple has no element {self.opts.target!r}, the target")
        if self._ik is None and not self._start(measured):
            return False
        names, values = target
        unknown = [n for n in names if n not in self._index]
        if unknown:
            raise ValueError(f"joint_target: the URDF has no actuated joint {unknown}")
        for name, value in zip(names, values):
            self._target[self._index[name]] = value
        now = time.monotonic()
        dt = None if self._last_target is None else min(now - self._last_target, 0.1)
        self._last_target = now
        if dt:
            self._step(dt)
        metadata = {"names": self.robot.names, "capture_time": time.time(), "step": event["metadata"]["step"]}
        self.node.send_output("joint_target", pa.array(self._ik.q, type=pa.float64()), metadata=metadata)
        return False

    def _on_program_state(self, event: dict) -> bool:
        return event["value"][0].as_py() == "disconnect"

    # -- helpers -------------------------------------------------------------

    def _start(self, measured: dict[str, float]) -> bool:
        """Start the command and the target at the measured joints, once every joint is measured."""
        missing = [n for n in self.robot.names if n not in measured]
        if missing:
            logger.warning("joints: ignoring a target, no measured state yet for %s", missing)
            return False
        q = np.clip([measured[n] for n in self.robot.names], self.robot.lower, self.robot.upper)
        self._ik = pink.Configuration(
            self.robot.model,
            self.robot.model.createData(),
            q,
            collision_model=self.robot.geom,
            collision_data=pin.GeometryData(self.robot.geom),
        )
        self._target = q.copy()
        return True

    def _step(self, dt: float) -> None:
        """Move the command one IK step toward the target, unless that step is unsafe."""
        self._posture.set_target(self._target)
        try:
            v = pink.solve_ik(
                self._ik, [self._posture], dt, solver="daqp", limits=self._limits, barriers=self._barriers
            )
        except NoSolutionFound:
            return self._hold("no IK solution")
        q = pin.integrate(self.robot.model, self._ik.q, v * dt)
        collisions = self.robot.collisions(q)
        if collisions:
            return self._hold(f"the next step collides: {collisions[0]}")
        if self._held is not None:
            logger.info("joints: moving again")
            self._held = None
        self._ik.update(q)

    def _hold(self, reason: str) -> None:
        if self._held != reason:
            logger.warning("joints: holding, %s", reason)
        self._held = reason

    def _set_state(self, state: str) -> None:
        """Edge-triggered node_state: emitted only when the state changes."""
        if state != self.state:
            self.state = state
            self.node.send_output("node_state", pa.array([state]))
