# akai_tuple

The tuple format as a Python package: several streams' samples in one dora message, as the
`sync` node emits them. The format is in [`tuples.py`](src/akai_tuple/tuples.py).

| name | what |
| --- | --- |
| `build(names, events)` | a tuple's (value, metadata) from one dora event per element; an event that is a tuple is joined in flat, its elements becoming this tuple's |
| `elements(value, metadata)` | a tuple's elements: (name, array, metadata) each |
| `one_row(array)` | an array as one row of a list array (a tuple field), zero-copy |

A tuple's own metadata keys have no `.`: `capture_time`, and `step`, the observation index
that every node answering the tuple copies onto what it sends.
