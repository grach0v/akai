"""The scene file: one YAML file naming what the nodes need to know about a robot cell (format:
assets/README.md). It is a convenience: every node that reads one can be given the same
information through its own config instead.

    urdf: ../urdf/ur10e_2f85.urdf             # the kinematics
    calibration: ../calibration/tabletop.yaml  # the cameras
    mujoco:                                    # the simulation of the cell
      model: ../schemas/ur10e_2f85.xml
      parts: {arm: {joints: [...]}, ...}

Paths are relative to the scene file. Every section is optional; a node that needs one the
scene lacks fails. This package only reads and validates the file; using it (loading the
URDF, resolving the parts in the MJCF, ...) is each node's business.
"""

from __future__ import annotations

from pathlib import Path
from typing import TypeVar

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

T = TypeVar("T")


class PartSpec(BaseModel):
    """A controllable part: a set of MJCF joints driven together."""

    model_config = ConfigDict(extra="forbid")

    # MJCF joint names, in the order the part's vectors list them.
    joints: list[str] = Field(min_length=1)
    # MJCF actuators driving them, one per joint. Default: named after the joints.
    actuators: list[str] | None = None
    # URDF joint names, where they differ from the MJCF's. Default: the MJCF names.
    urdf_joints: list[str] | None = None
    # URDF value = MJCF value * scale, per joint, where the two models define a joint
    # differently. Default: 1.
    urdf_scale: list[float] | None = None

    @model_validator(mode="after")
    def _one_per_joint(self) -> PartSpec:
        for key in ("actuators", "urdf_joints", "urdf_scale"):
            value = getattr(self, key)
            if value is not None and len(value) != len(self.joints):
                raise ValueError(f"{key} has {len(value)} entries for {len(self.joints)} joints")
        return self

    @property
    def actuator_names(self) -> list[str]:
        return self.actuators or self.joints

    @property
    def urdf_names(self) -> list[str]:
        """The joints' URDF names: what the part's joint states and targets carry."""
        return self.urdf_joints or self.joints

    @property
    def scale(self) -> list[float]:
        """URDF value = MJCF value * scale, per joint."""
        return self.urdf_scale or [1.0] * len(self.joints)


class MujocoScene(BaseModel):
    """The cell's simulation: its MJCF, and the controllable parts in it."""

    model_config = ConfigDict(extra="forbid")

    model: Path
    # The controllable parts, by name.
    parts: dict[str, PartSpec] = Field(min_length=1)


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


def one_of(option: str, explicit: T | None, scene: Path, from_scene: T | None) -> T | None:
    """A config value given directly (by `option`) or by the scene file, never both. Either may be
    missing (None): whether the value is required is the node's business."""
    if explicit is not None and from_scene is not None:
        raise ValueError(f"{option} and the scene {scene} both give it: set one")
    return explicit if explicit is not None else from_scene
