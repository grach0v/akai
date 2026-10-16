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


class JointsModeConfig(BaseModel):
    """`joints` mode: pass joint targets on to the robot, within its limits and collision-free."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["joints"] = "joints"
    # The `step` tuple's element holding the target; its other named vectors are the measured joints.
    target: str = "joint_target"
    # Fraction of each joint's URDF velocity limit the command may move at.
    speed_scale: float = Field(0.5, gt=0, le=1)
    # Collision barrier: the distance (m) kept between geometries, and how fast it may be closed
    # (per second, as a fraction of the remaining distance).
    min_distance: float = Field(0.01, ge=0)
    barrier_gain: float = Field(1.0, gt=0)
    # The barrier acts on this many closest pairs only (far ones do not matter, and each costs
    # time); the final collision check still covers every pair.
    barrier_pairs: int = Field(16, gt=0)


ModeConfig = JointsModeConfig  # union of the mode configs, discriminated by `name`


class PinkConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # The robot's URDF (relative to the dataflow directory), or a scene file giving it.
    urdf: Path | None = None
    scene: Path | None = None
    # Which mode runs, and its settings.
    mode: ModeConfig = Field(default_factory=JointsModeConfig, discriminator="name")

    @model_validator(mode="after")
    def _from_scene(self) -> PinkConfig:
        if self.scene is not None:
            self.urdf = one_of("URDF", self.urdf, self.scene, load_scene(self.scene).urdf)
        if self.urdf is None:
            raise ValueError("the robot model needs URDF, or a SCENE file with `urdf`")
        return self


def load_config() -> PinkConfig:
    return PinkConfig(_cli_parse_args=True)
