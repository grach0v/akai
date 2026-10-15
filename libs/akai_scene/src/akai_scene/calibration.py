"""The calibration file: every camera of the cell, where it is and how it projects."""

from __future__ import annotations

from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field


class CameraCalibration(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # The URDF frame (link) the camera is mounted on.
    frame: str
    # [x, y, z, qx, qy, qz, qw]: the optical frame (x right, y down, z forward) in `frame`.
    extrinsics: list[float] = Field(min_length=7, max_length=7)
    # [fx, fy, cx, cy] in pixels, for an image of width x height.
    intrinsics: list[float] = Field(min_length=4, max_length=4)
    width: int = Field(gt=0)
    height: int = Field(gt=0)
    # The camera also gives a depth image aligned to its color image (same intrinsics and size).
    depth: bool = False


class Calibration(BaseModel):
    model_config = ConfigDict(extra="forbid")

    cameras: dict[str, CameraCalibration]


def load_calibration(path: Path) -> dict[str, CameraCalibration]:
    """Read and validate a calibration file: its cameras, by name."""
    return Calibration.model_validate(yaml.safe_load(path.read_text())).cameras
