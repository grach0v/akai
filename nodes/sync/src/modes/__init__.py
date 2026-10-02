"""Mode registry: mode name (config `MODE__NAME`) -> mode class.

Every mode implements the node protocol that main.py drives:

1. `__init__(cfg)`: open the mode's resources, before the node joins the dataflow.
2. `start(node)`: keep the node and report the first `node_state`.
3. `handle(event) -> bool`: handle one INPUT event; True stops the node.
4. `close()`: release the resources; called however the loop ends.

TODO: this node already has two modes: add an abstract base class `Mode`
(`abc.ABC`, these four methods abstract) that every mode subclasses, so the protocol
is explicit and a mode missing one fails when it is built, not mid-run.
"""

from modes.align import AlignMode
from modes.tuple import TupleMode

MODES = {
    "align": AlignMode,
    "tuple": TupleMode,
}
