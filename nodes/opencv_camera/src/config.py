"""Config for the opencv_camera node.

`CameraConfig` holds the node-wide fields plus `mode`, a discriminated union of one
config class per mode: `MODE__NAME` selects the class and `MODE__<FIELD>` sets
its fields (or one JSON object in `MODE`). Every field is also a CLI flag
(`--mode.fps 15`), which wins over the env; that is how
dora modules reach a node through `args:`.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class CameraModeConfig(BaseModel):
    """`camera` mode: stream one camera device."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["camera"] = "camera"
    # The camera device: an index (0) or, on Linux, a device path such as
    # /dev/video0 or the stable /dev/v4l/by-id/... form. Validated left to
    # right, so "0" from the env becomes the int 0 and a path stays a string.
    source: int | str = Field(0, union_mode="left_to_right")
    # Requested capture resolution. Devices may pick the closest supported mode;
    # the frame's real size travels in the message metadata.
    width: int = Field(640, gt=0)
    height: int = Field(480, gt=0)
    # Capture rate requested from the device.
    fps: int = Field(30, gt=0)
    # Optional capture FourCC, e.g. "MJPG" — many USB cameras only reach their
    # advertised FPS at high resolutions in a compressed mode.
    fourcc: str | None = Field(None, min_length=4, max_length=4)
    # Wire format of `<camera_name>_image`: raw RGB bytes or JPEG.
    encoding: Literal["rgb8", "jpeg"] = "rgb8"
    jpeg_quality: int = Field(90, ge=1, le=100)


ModeConfig = CameraModeConfig  # union of the mode configs, discriminated by `name`


class CameraConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # Stream name: prefixes every output (`<camera_name>_image`, `<camera_name>_node_state`).
    camera_name: str
    # Which mode runs, and its settings.
    mode: ModeConfig = Field(default_factory=CameraModeConfig, discriminator="name")


def load_config() -> CameraConfig:
    return CameraConfig(_cli_parse_args=True)
