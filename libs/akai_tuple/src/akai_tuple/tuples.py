"""A tuple: several streams' samples in one dora message, values and metadata kept apart.

* value: a one-row Arrow struct, one field per element (named after it) holding that element's
  array as a one-row list;
* metadata: every key of every element's metadata as `<element>.<key>`, plus the tuple's own keys
  (no `.`): its `capture_time` and its `step`, set by whoever builds it.

Tuples join flat: a tuple built from another one holds its elements, not the tuple.
"""

from __future__ import annotations

import pyarrow as pa


def one_row(array: pa.Array) -> pa.ListArray:
    """`array` as the single row of a list array, the type of a tuple field. Zero-copy: the list's
    one row spans the whole array."""
    return pa.ListArray.from_arrays(pa.array([0, len(array)], pa.int32()), array)


def build(names: list[str], events: list[dict]) -> tuple[pa.StructArray, dict]:
    """The tuple's (value, metadata) from one dora event per element, in order, named `names`.

    An event that is itself a tuple (a struct value) is joined in, not nested: its elements become
    this tuple's, under their own names and metadata, and its own keys (`capture_time`, `step`)
    go out as `<name>.<key>`. Element names must not repeat."""
    fields: list[str] = []
    arrays: list[pa.Array] = []
    metadata: dict = {}
    for name, event in zip(names, events):
        value, md = event["value"], event["metadata"]
        if pa.types.is_struct(value.type):
            inner = [field.name for field in value.type]
            fields += inner
            arrays += value.flatten()
            for key, item in md.items():
                own = not any(key.startswith(f"{field}.") for field in inner)
                metadata[f"{name}.{key}" if own else key] = item
        else:
            fields.append(name)
            arrays.append(one_row(value))
            metadata.update({f"{name}.{key}": item for key, item in md.items()})
    repeated = sorted({f for f in fields if fields.count(f) > 1})
    if repeated:
        raise ValueError(f"tuple elements repeat: {repeated}")
    return pa.StructArray.from_arrays(arrays, names=fields), metadata


def elements(value: pa.StructArray, metadata: dict) -> list[tuple[str, pa.Array, dict]]:
    """The tuple's elements, in order: (name, its array, its metadata without the `<name>.` prefix)."""
    out = []
    for field in value.type:
        prefix = f"{field.name}."
        element_md = {k.removeprefix(prefix): v for k, v in metadata.items() if k.startswith(prefix)}
        out.append((field.name, value.field(field.name)[0].values, element_md))
    return out
