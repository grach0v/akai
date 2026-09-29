"""Config for the rerun node.

`RerunConfig` holds the node-wide fields plus `mode`, a discriminated union of one
config class per mode: `MODE__NAME` selects the class and `MODE__<FIELD>` sets
its fields (or one JSON object in `MODE`). Every field is also a CLI flag
(`--mode.grpc_port 9876`), which wins over the env.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class RecordModeConfig(BaseModel):
    """`record` mode: write every stream to an .rrd file (cameras as H.264) and
    serve it live over gRPC (cameras as JPEG, for the lowest viewer latency)."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["record"] = "record"
    # Directory for the recording, relative to the dataflow directory. A session is one
    # recording, `<app_id>_<YYYYmmdd_HHMMSS>`, written as `<recording>.rrd` (the raw streams)
    # and, with a robot, `<recording>.fk.rrd` (its forward kinematics, a layer on the same
    # recording id); the robot model itself goes to `<urdf name>.model.rrd` (see robot_urdf).
    rrd_dir: Path = Path("out/rrd")
    # The gRPC server viewers connect to (rerun+http://<host>:<port>/proxy). 0.0.0.0 admits remote viewers.
    bind_ip: str = "0.0.0.0"
    grpc_port: int = 9876
    # How much the server buffers for a viewer that joins late; small keeps a live view current.
    server_memory_limit: str = "256MB"
    # H.264 encoding of camera streams for the file (libx264). The frame rate is
    # the encoder's time base and should match the cameras'; the preset trades
    # CPU for size; a keyframe every `keyframe_interval` frames (SPS/PPS repeated)
    # is where a viewer can start decoding after a seek.
    video_fps: int = Field(30, gt=0)
    h264_preset: Literal["ultrafast", "superfast", "veryfast", "faster", "fast", "medium"] = "veryfast"
    keyframe_interval: int = Field(60, gt=0)
    # JPEG quality for the live stream when a camera sends raw rgb8 (a camera
    # sending JPEG is passed through untouched).
    live_jpeg_quality: int = Field(80, ge=1, le=100)
    # What the inputs hold, by input id; an input listed nowhere is plotted as numbers.
    # JSON lists in the env: MODE__CAMERAS='["cam_high"]'.
    # Image inputs: recorded as video, shown in a grid in the viewer above one plot per
    # part. With neither cameras nor a robot: the viewer's automatic layout.
    cameras: list[str] = []
    # Depth-image inputs (uint16 millimetres).
    depth: list[str] = []
    # Joint-state inputs: plotted, and moving the 3D robot's joints (with robot_urdf).
    joint_states: list[str] = []
    # Optional robot URDF (relative to the dataflow directory), shown in 3D under `robot/`
    # and moved by every `joint_state` stream: their `names` are URDF joint names. Its
    # static model is written once to `<rrd_dir>/<urdf name>.model.rrd`, the dataset asset
    # every recording shares, and sent to the live viewer.
    robot_urdf: Path | None = None


class VisualizeModeConfig(BaseModel):
    """`visualize` mode: open a rerun viewer on a running record server."""

    model_config = ConfigDict(extra="forbid")

    name: Literal["visualize"] = "visualize"
    # The record node's gRPC server. Another machine: replace 127.0.0.1 with its address.
    server_url: str = "rerun+http://127.0.0.1:9876/proxy"
    # Seconds to wait for that server to accept connections before launching the
    # viewer (a viewer does not retry a failed connection).
    server_timeout: float = Field(15.0, gt=0)
    # Viewer memory budget; the viewer drops its oldest data past it.
    memory_limit: str = "2GB"


ModeConfig = RecordModeConfig | VisualizeModeConfig  # discriminated by `name`


class RerunConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # Rerun application id; also names the recording file.
    app_id: str = "akai"
    # Which mode runs, and its settings.
    mode: ModeConfig = Field(default_factory=RecordModeConfig, discriminator="name")


def load_config() -> RerunConfig:
    return RerunConfig(_cli_parse_args=True)
