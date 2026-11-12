"""Config for the pink node.

`MODE__NAME` picks the mode and `MODE__<FIELD>` sets its fields. Every field is also a CLI
flag (`--mode.speed_scale 0.5`), which wins over the env.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from akai_scene import load_scene, one_of
from pydantic import BaseModel, ConfigDict, Field, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class IkModeConfig(BaseModel):
    """`ik` mode: move the robot toward joint and pose targets, within its limits and collision-free."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["ik"] = "ik"
    # The `step` tuple's elements holding the targets, joint targets or poses (known by their
    # metadata); its other named vectors are the measured joints. A JSON list in the env.
    targets: list[str] = Field(["joint_target"], min_length=1)
    # Fraction of each joint's URDF velocity limit the command may move at.
    speed_scale: float = Field(0.8, gt=0, le=1)
    # The distance (m) the command keeps between the robot's geometries, and from the cell's.
    min_distance: float = Field(0.01, ge=0)


ModeConfig = IkModeConfig  # union of the mode configs, discriminated by `name`


class PinkConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # The robot's URDF (relative to the dataflow directory), or a scene file giving it.
    urdf: Path | None = None
    scene: Path | None = None
    # Which mode runs, and its settings.
    mode: ModeConfig = Field(default_factory=IkModeConfig, discriminator="name")

    @model_validator(mode="after")
    def _from_scene(self) -> PinkConfig:
        if self.scene is not None:
            self.urdf = one_of("URDF", self.urdf, self.scene, load_scene(self.scene).urdf)
        if self.urdf is None:
            raise ValueError("the robot model needs URDF, or a SCENE file with `urdf`")
        return self
