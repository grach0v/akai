"""Mode registry: mode name (config `MODE__NAME`) -> program state machine class.

Every program machine implements the node protocol start(node) / handle(event) /
close(); main.py drives any of them the same way.
"""

from modes.simple import SimpleProgram

MODES = {
    "simple": SimpleProgram,
}
