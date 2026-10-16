"""The scene's `robots` and `props`: what the cell is made of, placed in its `world`.

Only the cell builder (assets/cells/build_cell.py) reads these; it composes them into the
cell's URDF and MJCF, which the nodes load.
"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import BaseModel, ConfigDict, Field

# [x, y, z, qx, qy, qz, qw]: a frame in the cell's `world` (floor level), metres and a unit quaternion.
Pose = Annotated[list[float], Field(min_length=7, max_length=7)]


class CellRobot(BaseModel):
    model_config = ConfigDict(extra="forbid")

    urdf: Path
    # The robot's MJCF: its actuators named after the URDF joints, and a site per URDF frame the
    # cell uses (its root link, every camera mount).
    mjcf: Path
    # Where the URDF's root link is.
    pose: Pose
    # The `home` keyframe: actuator (URDF joint) -> position. Unlisted joints start at zero.
    home: dict[str, float] = {}


class CellProp(BaseModel):
    model_config = ConfigDict(extra="forbid")

    urdf: Path  # one link, primitive shapes (assets/props/)
    pose: Pose  # where its link is
