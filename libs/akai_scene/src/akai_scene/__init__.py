"""The scene file: one YAML file describing a robot cell (format: assets/README.md). Its
`robots` and `props` say what the cell is made of, for the cell builder; the rest names what
the nodes need, and is a convenience: every node that reads one can be given the same
information through its own config instead.

    robots: {ur10e: {urdf: ..., mjcf: ..., pose: [0, 0, 0.75, 0, 0, 1, 0]}}
    props: {table: {urdf: ../../props/table/table.urdf, pose: [0.35, 0, 0, 0, 0, 0, 1]}}
    urdf: ur10e_tabletop.urdf    # the kinematics (built)
    calibration: calibration.yaml # the cameras
    mujoco:                       # the simulation of the cell
      model: ur10e_tabletop.xml   # (built)
      parts: {arm: [shoulder_pan_joint, ...], gripper: [finger_joint]}

Paths are relative to the scene file. Every section is optional; a node that needs one the
scene lacks fails. This package only reads and validates the file; using it (loading the
URDF, resolving the parts in the MJCF, ...) is each node's business.
"""

from .calibration import CameraCalibration, load_calibration
from .cell import CellProp, CellRobot
from .config import one_of
from .mujoco import MujocoScene
from .scene import Scene, load_scene

__all__ = [
    "CameraCalibration",
    "CellProp",
    "CellRobot",
    "MujocoScene",
    "Scene",
    "load_calibration",
    "load_scene",
    "one_of",
]
