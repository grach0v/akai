"""Helpers for a node config that can take its values from a scene file."""

from __future__ import annotations

from pathlib import Path
from typing import TypeVar

T = TypeVar("T")


def one_of(option: str, explicit: T | None, scene: Path, from_scene: T | None) -> T | None:
    """A config value given directly (by `option`) or by the scene file, never both. Either may be
    missing (None): whether the value is required is the node's business."""
    if explicit is not None and from_scene is not None:
        raise ValueError(f"{option} and the scene {scene} both give it: set one")
    return explicit if explicit is not None else from_scene
