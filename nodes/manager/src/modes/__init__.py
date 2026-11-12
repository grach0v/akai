"""Mode registry: `MODE__NAME` -> program state machine. A machine follows the node protocol
(akai_node.mode: start, handle, close) without being an akai_node.Mode: it reports
`program_state`, not `node_state`."""

from modes.simple import SimpleProgram

MODES = {
    "simple": SimpleProgram,
}
