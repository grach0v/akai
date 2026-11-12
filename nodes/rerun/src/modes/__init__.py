"""Mode registry: `MODE__NAME` -> mode class (the protocol: akai_node.Mode)."""

from modes.record import RecordMode
from modes.visualize import VisualizeMode

MODES = {
    "record": RecordMode,
    "visualize": VisualizeMode,
}
