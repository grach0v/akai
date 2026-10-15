"""The scene file: one YAML file naming what the nodes need to know about a robot cell (format:
assets/README.md). It is a convenience: every node that reads one can be given the same
information through its own config instead.

    urdf: ../urdf/ur10e_2f85.urdf             # the kinematics
    calibration: ../calibration/tabletop.yaml  # the cameras
    mujoco:                                    # the simulation of the cell
      model: ../schemas/ur10e_2f85.xml
      parts: {arm: {joints: [...]}, ...}

Paths are relative to the scene file. Every section is optional; a node that needs one the
scene lacks fails. This package only reads and validates the file; using it (loading the
URDF, resolving the parts in the MJCF, ...) is each node's business.
"""

from .calibration import CameraCalibration, load_calibration
from .config import one_of
from .mujoco import MujocoScene, PartSpec
from .scene import Scene, load_scene

__all__ = ["CameraCalibration", "MujocoScene", "PartSpec", "Scene", "load_calibration", "load_scene", "one_of"]
