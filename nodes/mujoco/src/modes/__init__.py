"""Mode registry: `MODE__NAME` -> mode class (the protocol: akai_node.Mode).

`step` mode extends `sim` mode, so the two share the protocol.
"""

from modes.sim import SimMode
from modes.step import StepMode

MODES = {
    "sim": SimMode,
    "step": StepMode,
}
