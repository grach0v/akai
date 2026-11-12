"""Config for the rerun node.

`RerunConfig` holds the node-wide fields plus `mode`, a discriminated union of one
config class per mode: `MODE__NAME` selects the class and `MODE__<FIELD>` sets
its fields (or one JSON object in `MODE`). Every field is also a CLI flag
(`--mode.grpc_port 9876`), which wins over the env.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from akai_scene import load_scene, one_of
from pydantic import BaseModel, ConfigDict, Field, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class RecordModeConfig(BaseModel):
    """`record` mode: write every stream to an .rrd file (cameras as H.264) and
    serve it live over gRPC (cameras as JPEG, for the lowest viewer latency)."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["record"] = "record"
    # Where recordings go, relative to the dataflow directory.
    rrd_dir: Path = Path("out/rrd")
    # Live gRPC server; 0.0.0.0 admits remote viewers.
    bind_ip: str = "0.0.0.0"
    grpc_port: int = 9876
    # libx264 preset of the file's video: faster presets use less CPU (a small robot computer),
    # slower ones make smaller files.
    h264_preset: Literal["ultrafast", "superfast", "veryfast", "faster", "fast", "medium"] = "veryfast"
    # JPEG quality of the live stream for rgb8 cameras (JPEG cameras pass through): lower it for
    # a remote viewer on a slow link.
    live_jpeg_quality: int = Field(80, ge=1, le=100)
    # Metadata key of the time messages are logged at (a datetime, or seconds since the epoch).
    # dora's send `timestamp` is on every message; `capture_time` is when the data was taken,
    # for producers that stamp it.
    input_time: str = "timestamp"
    # Joint-state streams, which also move the 3D robot: input ids, or `<input>/<field>` for tuple
    # elements. Images and depth are known by their `encoding`; other numeric streams are plotted.
    joint_states: list[str] = []
    # Viewer plots: name -> entities, e.g. {"elbow": ["arm_joint_state/elbow_joint"]}. Empty: one plot.
    plots: dict[str, list[str]] = {}
    # Robot shown in 3D, moved by joint_states (their `names` are URDF joints).
    urdf: Path | None = None
    # Camera calibration file; an image is placed in 3D by its `camera` metadata.
    calibration: Path | None = None
    # Scene file giving urdf and calibration instead (not both).
    scene: Path | None = None

    @model_validator(mode="after")
    def _from_scene(self) -> RecordModeConfig:
        if self.scene is not None:
            scene = load_scene(self.scene)
            self.urdf = one_of("MODE__URDF", self.urdf, self.scene, scene.urdf)
            self.calibration = one_of("MODE__CALIBRATION", self.calibration, self.scene, scene.calibration)
        if self.calibration is not None and self.urdf is None:  # a camera's `frame` is a URDF link
            raise ValueError("a calibration file needs the robot URDF (MODE__URDF or MODE__SCENE)")
        return self


class VisualizeModeConfig(BaseModel):
    """`visualize` mode: open a rerun viewer on a running record server."""

    model_config = ConfigDict(extra="forbid")

    name: Literal["visualize"] = "visualize"
    # The record node's gRPC server. Another machine: replace 127.0.0.1 with its address.
    server_url: str = "rerun+http://127.0.0.1:9876/proxy"


ModeConfig = RecordModeConfig | VisualizeModeConfig  # discriminated by `name`


class RerunConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # Rerun application id; also names the recording file.
    app_id: str = "akai"
    # Which mode runs, and its settings.
    mode: ModeConfig = Field(default_factory=RecordModeConfig, discriminator="name")
