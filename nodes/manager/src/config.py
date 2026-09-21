"""Config for the manager node.

`ManagerConfig` holds the node-wide fields plus `mode`, a discriminated union of one
config class per mode: `MODE__NAME` selects the class and `MODE__<FIELD>` sets
its fields (or one JSON object in `MODE`). Every field is also a CLI flag
(`--mode.producers ...`), which wins over the env.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class SimpleModeConfig(BaseModel):
    """`simple` mode: boot -> running -> disconnect."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["simple"] = "simple"
    # Manager input ids that must each report `ready` before the program leaves
    # `boot`. A JSON list in the env: MODE__PRODUCERS='["cam_high", "cam_wrist"]'.
    producers: list[str] = []


ModeConfig = SimpleModeConfig  # union of the mode configs, discriminated by `name`


class ManagerConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # Which program lifecycle runs, and its settings.
    mode: ModeConfig = Field(default_factory=SimpleModeConfig, discriminator="name")


def load_config() -> ManagerConfig:
    return ManagerConfig(_cli_parse_args=True)
