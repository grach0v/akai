"""Config for the mujoco node.

`MujocoConfig` holds the node-wide fields plus `mode`, a discriminated union of one
config class per mode: `MODE__NAME` selects the class and `MODE__<FIELD>` sets
its fields (or one JSON object in `MODE`). Every field is also a CLI flag
(`--mode.jpeg_quality 80`), which wins over the env.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from akai_scene import load_scene, one_of
from pydantic import BaseModel, ConfigDict, Field, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class SimModeConfig(BaseModel):
    """`sim` mode: run the scene in real time, drive its parts, render its cameras."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["sim"] = "sim"
    # MJCF camera names to render and publish as `<camera>_image`. A JSON list in
    # the env: MODE__CAMERAS='["cam_overhead"]'. Empty: no renderer at all.
    cameras: list[str] = []
    # Also render depth (`<camera>_depth`) for the RGB-D cameras (`depth: true` in the calibration).
    depth: bool = False
    # Rendered image size; at most the model's offscreen buffer (<visual><global offwidth/offheight>).
    width: int = Field(640, gt=0)
    height: int = Field(480, gt=0)
    # MuJoCo render flags, by `mjtRndFlag` name without its `mjRND_` prefix (SHADOW, REFLECTION,
    # SKYBOX, FOG, HAZE, ...); unset ones keep MuJoCo's defaults. A JSON object in the env:
    # MODE__RENDER_FLAGS='{"SHADOW": false}'.
    render_flags: dict[str, bool] = {}
    # Wire format of `<camera>_image`: raw RGB bytes or JPEG.
    encoding: Literal["rgb8", "jpeg"] = "rgb8"
    jpeg_quality: int = Field(90, ge=1, le=100)
    # Simulated seconds per wall-clock second.
    realtime_factor: float = Field(1.0, gt=0)
    # When the physics falls further behind the wall clock than this (seconds of sim time),
    # it drops the backlog instead of racing to catch up.
    max_lag: float = Field(0.1, gt=0)


ModeConfig = SimModeConfig  # union of the mode configs, discriminated by `name`


class MujocoConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # The scene: the MJCF and its controllable parts (by name; part <p> goes out as
    # `<p>_joint_state`), given here or by a scene file's `mujoco` section, not both. Paths are
    # relative to the dataflow directory; PARTS is JSON, in the scene file's format
    # (assets/README.md): PARTS='{"arm": ["shoulder_pan_joint", ...], "gripper": ["finger_joint"]}'.
    scene: Path | None = None
    model: Path | None = None
    parts: dict[str, list[str]] | None = None
    # Camera calibration file (or the scene's): its RGB-D cameras (`depth: true`) also render depth.
    calibration: Path | None = None
    # Which mode runs, and its settings.
    mode: ModeConfig = Field(default_factory=SimModeConfig, discriminator="name")

    @model_validator(mode="after")
    def _from_scene(self) -> MujocoConfig:
        if self.scene is not None:
            scene = load_scene(self.scene)
            sim = scene.mujoco
            self.model = one_of("MODEL", self.model, self.scene, sim and sim.model)
            self.parts = one_of("PARTS", self.parts, self.scene, sim and sim.parts)
            self.calibration = one_of("CALIBRATION", self.calibration, self.scene, scene.calibration)
        if self.model is None or not self.parts:
            raise ValueError("the sim needs MODEL and PARTS, or a SCENE file with a `mujoco` section")
        return self


def load_config() -> MujocoConfig:
    return MujocoConfig(_cli_parse_args=True)
