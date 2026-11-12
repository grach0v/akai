"""`pose` mode: drive a frame's pose (the TCP) from the page, the other joints in joint space.

Like `joint` mode, but the joints moving `frame` get a -/+ per pose axis instead of sliders:
a click moves the target pose by `linear_step` along the reference frame's x, y or z, or by
`step` radians about the frame's own x, y or z. Every observation is answered with
`pose_target` (the pose of `frame` in `reference`) and `joint_target` for the other joints, both
with the observation's `step`; pink decides how the robot gets there. The target starts at the
measured pose (forward kinematics of the measured joints) and goes back to it on Sync and on
every program-state change.

Inputs:  observation, program_state.
Outputs: pose_target, joint_target, node_state.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import pyarrow as pa
from akai_tuple import elements

from config import PoseModeConfig, WebControllerConfig
from modes.joint import Board, JointMode

logger = logging.getLogger("web_controller")


class Kinematics:
    """The pose of `frame` in `reference` from joint values, on the URDF (Pinocchio)."""

    def __init__(self, urdf: Path, frame: str, reference: str):
        self.model = pin.buildModelFromUrdf(str(urdf))
        self._data = self.model.createData()
        for f in (frame, reference):
            if not self.model.existFrame(f):
                raise ValueError(f"{urdf.name} has no frame {f!r}")
        self._frame, self._reference = self.model.getFrameId(frame), self.model.getFrameId(reference)

        def chain(f: int) -> list[int]:
            return list(self.model.supports[self.model.frames[f].parentJoint])[1:]  # without the universe

        base = set(chain(self._reference))
        self.joints = [self.model.names[j] for j in chain(self._frame) if j not in base]
        if any(self.model.joints[self.model.getJointId(n)].nq != 1 for n in self.joints):
            raise ValueError(f"{frame}: every joint moving it must have one coordinate")

    def pose(self, measured: dict[str, float]) -> pin.SE3 | None:
        """The frame's pose in the reference frame, or None while a joint moving it is unmeasured."""
        if any(n not in measured for n in self.joints):
            return None
        q = pin.neutral(self.model)
        for n in self.joints:
            q[self.model.joints[self.model.getJointId(n)].idx_q] = measured[n]
        pin.framesForwardKinematics(self.model, self._data, q)
        return self._data.oMf[self._reference].actInv(self._data.oMf[self._frame])


def readable(pose: pin.SE3) -> list[float]:
    """x, y, z, roll, pitch, yaw: for the page to show."""
    return [*pose.translation, *pin.rpy.matrixToRpy(pose.rotation)]


class PoseBoard(Board):
    """The joint board plus the pose: measured and target."""

    def __init__(self, urdf: Path, opts: PoseModeConfig, kinematics: Kinematics):
        super().__init__(urdf, opts.step, exclude=frozenset(kinematics.joints))
        self._opts = opts
        self._measured_pose: pin.SE3 | None = None
        self._target_pose: pin.SE3 | None = None

    # -- from the dora loop --------------------------------------------------

    def measured_pose(self, pose: pin.SE3) -> None:
        """The measured pose. The first one seeds the target."""
        with self._lock:
            self._measured_pose = pose
            if self._target_pose is None:
                self._target_pose = pose.copy()

    def pose_target(self) -> np.ndarray | None:
        """The target pose as [x, y, z, qx, qy, qz, qw] while the program is running, else None."""
        with self._lock:
            if self._program_state != "running" or self._target_pose is None:
                return None
            return pin.SE3ToXYZQUAT(self._target_pose)

    # -- from the page -------------------------------------------------------

    def snapshot(self) -> dict:
        snap = super().snapshot()
        with self._lock:
            if self._measured_pose is not None:
                snap["pose"] = {
                    "frame": self._opts.frame,
                    "reference": self._opts.reference,
                    "measured": readable(self._measured_pose),
                    "target": readable(self._target_pose),
                }
        return snap

    def command(self, msg: dict) -> None:
        """`{"type": "pose", "axis": 0-5, "dir": -1 or 1}`: one click on a pose axis (x, y, z, then
        about the frame's x, y, z), or a `joint` mode message."""
        if msg["type"] != "pose":
            return super().command(msg)
        axis, sign = int(msg["axis"]), float(np.sign(msg["dir"]))
        with self._lock:
            if self._target_pose is None:
                return  # no measured pose yet
            delta = np.zeros(3)
            delta[axis % 3] = sign
            target = self._target_pose
            if axis < 3:
                self._target_pose = pin.SE3(
                    target.rotation, target.translation + delta * self._opts.linear_step
                )
            else:
                self._target_pose = pin.SE3(
                    target.rotation @ pin.exp3(delta * self._opts.step), target.translation
                )

    def _reset(self) -> None:
        super()._reset()
        if self._measured_pose is not None:
            self._target_pose = self._measured_pose.copy()


class PoseMode(JointMode):
    def __init__(self, cfg: WebControllerConfig):
        self.kinematics = Kinematics(cfg.urdf, cfg.mode.frame, cfg.mode.reference)
        logger.info("pose: %s in %s, moved by %s", cfg.mode.frame, cfg.mode.reference, self.kinematics.joints)
        super().__init__(cfg)

    def _board(self, urdf: Path) -> PoseBoard:
        return PoseBoard(urdf, self.opts, self.kinematics)

    # -- inputs --------------------------------------------------------------

    def _on_observation(self, event: dict) -> bool:
        """Update the measured joints and pose from the observation, and answer it."""
        md = event["metadata"]
        measured = {}
        for name, array, element_md in elements(event["value"], md):
            if "names" in element_md:
                values = array.to_pylist()
                self.board.measured(name, element_md["names"], values)
                measured.update(zip(element_md["names"], values))
        pose = self.kinematics.pose(measured)
        if pose is not None:
            self.board.measured_pose(pose)
        if self.board.disconnect_requested:
            self.set_state("disconnect")
        answer = {"step": md["step"], "capture_time": time.time()}
        target = self.board.pose_target()
        if target is not None:
            self.node.send_output(
                "pose_target",
                pa.array(target, type=pa.float64()),
                metadata={"frame": self.opts.frame, "reference": self.opts.reference, **answer},
            )
        targets = self.board.targets()
        if targets is not None:
            names, values = targets
            self.node.send_output(
                "joint_target", pa.array(values, type=pa.float64()), metadata={"names": names, **answer}
            )
        return False
