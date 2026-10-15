"""The scene's `mujoco` section: the cell's MJCF and the controllable parts in it."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import BaseModel, ConfigDict, Field


class MujocoScene(BaseModel):
    """The cell's simulation: its MJCF, and the controllable parts in it."""

    model_config = ConfigDict(extra="forbid")

    model: Path
    # The controllable parts, by name: each the MJCF actuators that drive it. Every actuator is
    # named after the URDF joint it drives, its ctrl and length in that joint's units.
    parts: dict[str, Annotated[list[str], Field(min_length=1)]] = Field(min_length=1)
