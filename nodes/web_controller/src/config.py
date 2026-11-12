"""Config for the web_controller node.

`WebControllerConfig` holds the node-wide fields plus `mode`, a discriminated union
of one config class per mode: `MODE__NAME` selects the class and `MODE__<FIELD>`
sets its fields (or one JSON object in `MODE`). Every field is also a CLI flag
(`--mode.step 0.05`), which wins over the env.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from akai_scene import load_scene, one_of
from pydantic import BaseModel, ConfigDict, Field, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class JointModeConfig(BaseModel):
    """`joint` mode: drive every robot part in joint space from the page."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["joint"] = "joint"
    # One -/+ click, in the joint's unit (rad for revolute joints).
    step: float = Field(0.02, gt=0)


class PoseModeConfig(JointModeConfig):
    """`pose` mode: drive one frame's pose (the tool's) from the page, the other joints in joint space.
    Rotations click by `step` radians, like the joints."""

    name: Literal["pose"] = "pose"
    # The URDF frame driven by the pose, and the frame the pose is given in (the robot's base).
    frame: str = "tool0"
    reference: str = "base_link"
    # One -/+ click along a pose axis, in m.
    linear_step: float = Field(0.01, gt=0)


ModeConfig = JointModeConfig | PoseModeConfig  # discriminated by `name`


class WebControllerConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # Where the page is served. 127.0.0.1 is this machine only; 0.0.0.0 admits other machines.
    host: str = "127.0.0.1"
    port: int = 8000
    # The robot's URDF, relative to the dataflow directory: its joint limits bound the sliders,
    # its kinematics give the pose. Given here or by a scene file's `urdf` (format:
    # assets/README.md), not both.
    urdf: Path | None = None
    scene: Path | None = None
    # Which mode runs, and its settings.
    mode: ModeConfig = Field(discriminator="name")

    @model_validator(mode="after")
    def _from_scene(self) -> WebControllerConfig:
        if self.scene is not None:
            self.urdf = one_of("URDF", self.urdf, self.scene, load_scene(self.scene).urdf)
        if self.urdf is None:
            raise ValueError("the page needs the robot's URDF: URDF, or a SCENE file with `urdf`")
        return self
