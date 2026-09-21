"""Mode registry: mode name (config `MODE__NAME`) -> mode class."""

from modes.record import RecordMode
from modes.visualize import VisualizeMode

MODES = {
    "record": RecordMode,
    "visualize": VisualizeMode,
}
