"""Mode registry: mode name (config `MODE__NAME`) -> mode class.

Every mode implements the node protocol that main.py drives:

1. `__init__(cfg)`: open the mode's resources, before the node joins the dataflow.
2. `start(node)`: keep the node and report the first `node_state`.
3. `handle(event) -> bool`: handle one INPUT event; True stops the node.
4. `close()`: release the resources; called however the loop ends.
"""

from modes.joints import JointsMode

MODES = {
    "joints": JointsMode,
}
