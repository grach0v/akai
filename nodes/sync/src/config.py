"""Config for the sync node.

`SyncConfig` holds `mode`, a discriminated union of one config class per mode:
`MODE__NAME` selects the class and `MODE__<FIELD>` sets its fields (or one JSON
object in `MODE`). Every field is also a CLI flag (`--mode.max_skew 0.05`), which
wins over the env.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

from bundler import Action, SkewTime


class TupleModeConfig(BaseModel):
    """`tuple` mode: one message holding a new message from every input."""

    model_config = ConfigDict(extra="forbid")  # a variable meant for another mode is a mistake

    name: Literal["tuple"] = "tuple"
    # The input ids to bundle; each becomes a field of the tuple, in this order. A JSON
    # list in the env: MODE__INPUTS='["arm_joint_target", "gripper_joint_target"]'.
    inputs: list[str] = Field(min_length=1)
    # Output id of the tuple.
    output: str
    # What to do when an input sends again before its tuple is complete, so its previous
    # message is dropped (e.g. a fast producer waiting on a slow one): "ignore", "warn"
    # (for every repeated message) or "error" (stop the node).
    on_repeat: Action = "ignore"
    # The most the messages of one tuple may be apart in time, in seconds: more means a
    # producer lags its peers. None: not checked.
    max_skew: float | None = Field(None, gt=0)
    # What to do when a tuple is over max_skew: "ignore", "warn" (once when the skew
    # appears) or "error" (stop the node).
    on_skew: Action = "warn"
    # What skew is measured by: every input's `capture_time` (when its data was captured;
    # an input without one is an error), or dora's send `timestamp` (every message has one).
    skew_time: SkewTime = "capture_time"

    @field_validator("inputs")
    @classmethod
    def _no_dot(cls, inputs: list[str]) -> list[str]:
        """An input's metadata goes out as `<input id>.<key>`: a `.` in an id would be ambiguous."""
        dotted = [i for i in inputs if "." in i]
        if dotted:
            raise ValueError(f"input ids must not contain '.': {dotted}")
        return inputs


class AlignModeConfig(BaseModel):
    """`align` mode: one tuple per reference time, of every input's sample captured nearest it."""

    model_config = ConfigDict(extra="forbid")

    name: Literal["align"] = "align"
    # The input whose messages give the reference times, one tuple each: typically the
    # observation clock, a dora timer shared with the producers.
    reference: str
    # Which time of a reference message is the reference time: dora's send `timestamp` (a
    # timer's tick time; a timer carries nothing else) or its `capture_time`.
    reference_time: SkewTime = "timestamp"
    # The inputs to align; each becomes a field of the tuple, in this order. A JSON list in the env.
    inputs: list[str] = Field(min_length=1)
    # Which time of an input message it is aligned by: its `capture_time` (when it was captured;
    # required on every message) or dora's send `timestamp`.
    input_time: SkewTime = "timestamp"
    # Output id of the tuple.
    output: str
    # How long each input's samples are kept to choose from, in seconds.
    history: float = Field(1.0, gt=0)
    # How long a reference time waits for every input to send a sample captured at or after it
    # (which settles which sample is nearest), in seconds; after that, the nearest one so far.
    timeout: float = Field(0.2, gt=0)
    # The most an element's capture_time may be off its tuple's reference time, in seconds:
    # more means a producer lags or stopped. None: not checked.
    max_skew: float | None = Field(None, gt=0)
    # What to do with a tuple that has elements over max_skew: "ignore", "warn" (one line per such
    # tuple) or "error" (stop the node).
    on_skew: Action = "warn"

    @field_validator("inputs")
    @classmethod
    def _no_dot(cls, inputs: list[str]) -> list[str]:
        """An input's metadata goes out as `<input id>.<key>`: a `.` in an id would be ambiguous."""
        dotted = [i for i in inputs if "." in i]
        if dotted:
            raise ValueError(f"input ids must not contain '.': {dotted}")
        return inputs


ModeConfig = TupleModeConfig | AlignModeConfig  # discriminated by `name`


class SyncConfig(BaseSettings):
    model_config = SettingsConfigDict(env_nested_delimiter="__")

    # Which mode runs, and its settings.
    mode: ModeConfig = Field(discriminator="name")


def load_config() -> SyncConfig:
    return SyncConfig(_cli_parse_args=True)
