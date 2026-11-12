"""Mode registry: `MODE__NAME` -> mode class (the protocol: akai_node.Mode)."""

from modes.align import AlignMode
from modes.step import StepMode

MODES = {
    "align": AlignMode,
    "step": StepMode,
}
