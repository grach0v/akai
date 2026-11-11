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
    # How much a pose target's errors weigh, per metre and per radian, against a joint target's
    # (1 per radian).
    position_cost: float = Field(1.0, gt=0)
    orientation_cost: float = Field(0.5, gt=0)
    # How strongly the joints no joint target names are held where they are, so a pose target
    # moves them without a fight.
    posture_cost: float = Field(1e-3, gt=0)
    # Fraction of each joint's URDF velocity limit the command may move at.
    speed_scale: float = Field(0.8, gt=0, le=1)
    # Collision barrier: the distance (m) kept between geometries, and how fast it may be closed
    # (per second, as a fraction of the remaining distance). Below 5, moves near the robot's own
    # links (2 cm apart at home) crawl; at 10 a step (at most 0.1 s) may close it all.
    min_distance: float = Field(0.01, ge=0)
    barrier_gain: float = Field(5.0, gt=0, le=10)
    # The barrier acts on this many closest pairs only (far ones do not matter, and each costs
    # time); the final collision check still covers every pair.
    barrier_pairs: int = Field(16, gt=0)


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


def load_config() -> PinkConfig:
    return PinkConfig(_cli_parse_args=True)
