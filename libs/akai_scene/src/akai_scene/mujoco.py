"""The scene's `mujoco` section: the cell's MJCF and the controllable parts in it."""

from __future__ import annotations

from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, model_validator


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
