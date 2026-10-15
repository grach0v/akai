"""The scene file itself, and reading it."""

from __future__ import annotations

from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict

from .mujoco import MujocoScene


class Scene(BaseModel):
    model_config = ConfigDict(extra="forbid")

    urdf: Path | None = None
    calibration: Path | None = None
    mujoco: MujocoScene | None = None


def load_scene(path: Path) -> Scene:
    """Read and validate a scene file, its paths made relative to the working directory. Unknown
    keys, lists of the wrong length and a file it names that does not exist are errors."""
    scene = Scene.model_validate(yaml.safe_load(path.read_text()))
    here = path.parent

    def resolve(file: Path) -> Path:
        resolved = here / file
        if not resolved.is_file():
            raise FileNotFoundError(f"{path}: no file at {resolved}")
        return resolved

    if scene.urdf is not None:
        scene.urdf = resolve(scene.urdf)
    if scene.calibration is not None:
        scene.calibration = resolve(scene.calibration)
    if scene.mujoco is not None:
        scene.mujoco.model = resolve(scene.mujoco.model)
    return scene
