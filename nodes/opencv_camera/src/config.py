"""Config for the opencv_camera node.

`MODE__NAME` picks the mode and `MODE__<FIELD>` sets its fields. Every field is also a CLI
flag (`--mode.fps 15`), which wins over the env.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class CameraModeConfig(BaseModel):
    """`camera` mode: stream one camera device."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["camera"] = "camera"
    # Device index (0) or, on Linux, a path (/dev/video0, /dev/v4l/by-id/...).
    source: int | str = Field(0, union_mode="left_to_right")
    # Requested resolution; the device may pick another, the metadata has the real one.
    width: int = Field(640, gt=0)
    height: int = Field(480, gt=0)
    fps: int = Field(30, gt=0)
    # Capture FourCC, e.g. "MJPG": many USB cameras need it for full FPS at high resolution.
    fourcc: str | None = Field(None, min_length=4, max_length=4)
    # Wire format of `<camera_name>_image`: raw RGB bytes or JPEG.
    encoding: Literal["rgb8", "jpeg"] = "rgb8"
    jpeg_quality: int = Field(90, ge=1, le=100)


ModeConfig = CameraModeConfig  # union of the mode configs, discriminated by `name`


class CameraConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # Prefixes the outputs, and is the `camera` in the image metadata.
    camera_name: str
    # Which mode runs, and its settings.
    mode: ModeConfig = Field(default_factory=CameraModeConfig, discriminator="name")


def load_config() -> CameraConfig:
    return CameraConfig(_cli_parse_args=True)
