"""`ik` mode — the default: move the robot toward its targets, safely.

Every `step` tuple (the measured joints and the targets answering them, joined by sync) is
answered at once with a command one IK step toward the targets. A target is a joint target
(`names`) or a pose target (`frame`, `reference`: `[x, y, z, qx, qy, qz, qw]`). Joints no joint
target names are held loosely, so a pose moves them freely. The step respects the URDF velocity
limits times `speed_scale`, the joint limits, and a collision barrier; a step that still
collides, or has no solution, is not taken and the last command is held.

Inputs:  step (a tuple), program_state.
Outputs: joint_target (every actuated joint, with the tuple's `step`), node_state.
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
from pink.tasks import FrameTask, PostureTask

from config import PinkConfig
from robot import Robot

logger = logging.getLogger("pink")


class IkMode:
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
                # pink's default also pulls every step toward standing still, which outweighs a
                # pose task; the barrier's constraint is what keeps the robot safe
                safe_displacement_gain=0.0,
            )
        ]
        self._ik: pink.Configuration | None = None  # at the last command
        self._target: np.ndarray | None = None  # the joint targets
        self._driven = np.zeros(model.nq, dtype=bool)  # joints some joint target has named
        self._poses: dict[str, FrameTask] = {}  # frame -> its task, at the latest pose target
        self._held: str | None = None  # why the command is held, if it is
        self._last_target: float | None = None  # when the previous target arrived (monotonic)
        self._handlers: dict[str, Callable[[dict], bool]] = {
            "step": self._on_step,
            "program_state": self._on_program_state,
        }
        logger.info("ik: %s, %d collision pairs", self.robot.names, len(self.robot.geom.collisionPairs))

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
        """Take the step's targets, and send the command one step toward them."""
        targets, measured = [], {}
        for name, array, md in elements(event["value"], event["metadata"]):
            if name in self.opts.targets:
                targets.append((name, array.to_numpy(zero_copy_only=False), md))
            elif "names" in md:
                measured.update(zip(md["names"], array.to_pylist()))
        missing = [t for t in self.opts.targets if t not in [name for name, _, _ in targets]]
        if missing:
            raise ValueError(f"step: the tuple has no target element {missing}")
        if self._ik is None and not self._start(measured):
            return False
        for name, values, md in targets:
            if "frame" in md:
                self._set_pose(name, values, md)
            elif "names" in md:
                self._set_joints(name, values, md["names"])
            else:
                raise ValueError(f"{name}: a target needs `names` (joints) or `frame` (a pose)")
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
            logger.warning("ik: ignoring a target, no measured state yet for %s", missing)
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

    def _set_joints(self, element: str, values: np.ndarray, names: list[str]) -> None:
        unknown = [n for n in names if n not in self._index]
        if unknown:
            raise ValueError(f"{element}: the URDF has no actuated joint {unknown}")
        index = [self._index[n] for n in names]
        self._target[index] = values
        self._driven[index] = True

    def _set_pose(self, element: str, values: np.ndarray, md: dict) -> None:
        """The frame's task, at the pose given in its reference frame (a pose of the model's world)."""
        frame, reference = md["frame"], md["reference"]
        for f in (frame, reference):
            if not self.robot.model.existFrame(f):
                raise ValueError(f"{element}: the URDF has no frame {f!r}")
        if len(values) != 7:
            raise ValueError(f"{element}: a pose is [x, y, z, qx, qy, qz, qw], got {len(values)} values")
        task = self._poses.get(frame)
        if task is None:
            task = self._poses[frame] = FrameTask(
                frame, position_cost=self.opts.position_cost, orientation_cost=self.opts.orientation_cost
            )
            logger.info("ik: pose target for %s in %s", frame, reference)
        world_reference = self._ik.get_transform_frame_to_world(reference)
        task.set_target(world_reference * pin.XYZQUATToSE3(values))

    def _step(self, dt: float) -> None:
        """Move the command one IK step toward the targets, unless that step is unsafe."""
        self._posture.set_target(np.where(self._driven, self._target, self._ik.q))
        self._posture.cost = np.where(self._driven, 1.0, self.opts.posture_cost)
        tasks = [self._posture, *self._poses.values()]
        try:
            v = pink.solve_ik(
                self._ik, tasks, dt, solver="daqp", limits=self._limits, barriers=self._barriers
            )
        except NoSolutionFound:
            return self._hold("no IK solution")
        q = pin.integrate(self.robot.model, self._ik.q, v * dt)
        collisions = self.robot.collisions(q)
        if collisions:
            return self._hold(f"the next step collides: {collisions[0]}")
        if self._held is not None:
            logger.info("ik: moving again")
            self._held = None
        self._ik.update(q)

    def _hold(self, reason: str) -> None:
        if self._held != reason:
            logger.warning("ik: holding, %s", reason)
        self._held = reason

    def _set_state(self, state: str) -> None:
        """Edge-triggered node_state: emitted only when the state changes."""
        if state != self.state:
            self.state = state
            self.node.send_output("node_state", pa.array([state]))
