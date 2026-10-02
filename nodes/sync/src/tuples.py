"""The tuple every sync mode emits (see the README): values and metadata kept apart, as in every
dora message.

* value: a one-row Arrow struct, one field per input (named by its input id, in input order)
  holding that input's array as a one-row list;
* metadata: every key of every input's metadata as `<input id>.<key>`.

The caller adds the tuple's own `capture_time`.
"""

from __future__ import annotations

import pyarrow as pa


def one_row(array: pa.Array) -> pa.ListArray:
    """`array` as the single row of a list array, the type of a tuple field. Zero-copy: the
    list's one row spans the whole array."""
    return pa.ListArray.from_arrays(pa.array([0, len(array)], pa.int32()), array)


def build(inputs: list[str], events: list[dict]) -> tuple[pa.StructArray, dict]:
    """The tuple's (value, metadata) from one dora event per input, in input order."""
    values = pa.StructArray.from_arrays([one_row(e["value"]) for e in events], names=inputs)
    metadata = {
        f"{input_id}.{key}": value
        for input_id, event in zip(inputs, events)
        for key, value in event["metadata"].items()
    }
    return values, metadata
