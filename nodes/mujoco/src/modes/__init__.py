"""Mode registry: mode name (config `MODE__NAME`) -> mode class.

Every mode implements the node protocol that main.py drives:

1. `__init__(cfg)`: open the mode's resources, before the node joins the dataflow.
2. `start(node)`: keep the node and report the first `node_state`.
3. `handle(event) -> bool`: handle one INPUT event; True stops the node.
4. `close()`: release the resources; called however the loop ends.

`step` mode extends `sim` mode, so the two share the protocol.
"""

from modes.sim import SimMode
from modes.step import StepMode

MODES = {
    "sim": SimMode,
    "step": StepMode,
}
