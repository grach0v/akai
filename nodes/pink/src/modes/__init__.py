"""Mode registry: `MODE__NAME` -> mode class (the protocol: akai_node.Mode)."""

from modes.ik import IkMode

MODES = {
    "ik": IkMode,
}
