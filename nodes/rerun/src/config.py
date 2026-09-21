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
    # Directory for the recording, `<app_id>_<YYYYmmdd_HHMMSS>.rrd`, relative to the dataflow directory.
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
    # Camera input ids to arrange in a grid above the time-series plot in the
    # viewer. Empty: automatic layout. A JSON list in the env: MODE__CAMERAS='["cam_high"]'.
    cameras: list[str] = []


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
