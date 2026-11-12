"""Config for the sync node.

`SyncConfig` holds `mode`, a discriminated union of one config class per mode:
`MODE__NAME` selects the class and `MODE__<FIELD>` sets its fields (or one JSON
object in `MODE`). Every field is also a CLI flag (`--mode.max_skew 0.05`), which
wins over the env.
"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import AfterValidator, BaseModel, ConfigDict, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


def _no_dot(inputs: list[str]) -> list[str]:
    """An input's metadata goes out as `<input id>.<key>`: a `.` in an id would be ambiguous."""
    dotted = [i for i in inputs if "." in i]
    if dotted:
        raise ValueError(f"input ids must not contain '.': {dotted}")
    return inputs


# The inputs to join; each becomes a field of the tuple, in this order. A JSON list in the env.
Inputs = Annotated[list[str], Field(min_length=1), AfterValidator(_no_dot)]


class AlignModeConfig(BaseModel):
    """`align` mode: one tuple per reference time, of every input's sample captured nearest it."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["align"] = "align"
    # The input whose messages give the reference times, one tuple each: a dora timer, or the
    # clocked producer's own stream (the robot's joint state), which may also be one of `inputs`.
    reference: str
    # Metadata key of a reference message's time; a dora timer only has `timestamp`.
    reference_time: str = "timestamp"
    inputs: Inputs
    # Metadata key of an input message's time. Late producers (a camera, a render) need a capture
    # time like `capture_time` to line up; dora's send `timestamp` is on every message.
    input_time: str = "timestamp"
    # Output id of the tuple.
    output: str
    # The most an element's time may be off its tuple's reference time, in seconds: more means a
    # producer lags or stopped. None: not checked.
    max_skew: float | None = Field(None, gt=0)
    # A tuple over max_skew: "warn" (one line per such tuple) or "error" (stop the node).
    on_skew: Literal["warn", "error"] = "warn"


class StepModeConfig(BaseModel):
    """`step` mode: one tuple per observation step, of every input's message for that step."""

    model_config = ConfigDict(extra="forbid")

    name: Literal["step"] = "step"
    # Every message needs `step`. An input that is a tuple is joined in flat.
    inputs: Inputs
    # The elements of the joined tuple to send on, in this order, so a consumer gets only what it
    # uses (the joint states, not an observation's images). None: every element. A JSON list in the env.
    elements: list[str] | None = Field(None, min_length=1)
    # Output id of the tuple.
    output: str


ModeConfig = AlignModeConfig | StepModeConfig  # discriminated by `name`


class SyncConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # Which mode runs, and its settings.
    mode: ModeConfig = Field(discriminator="name")
