"""Mode registry: mode name (config `MODE__NAME`) -> program state machine class.

Every mode implements the node protocol that main.py drives:

1. `__init__(cfg)`: open the mode's resources, before the node joins the dataflow.
2. `start(node)`: keep the node and broadcast the first `program_state`.
3. `handle(event) -> bool`: handle one INPUT event; True stops the node.
4. `close()`: release the resources; called however the loop ends.

TODO: once the node has more than one mode, add an abstract base class `Mode`
(these four methods abstract) that every program machine subclasses, so the protocol
is explicit and a mode missing one fails when it is built, not mid-run. The machines
are python-statemachine `StateMachine`s, so check that its metaclass combines with
`abc.ABCMeta` (or declare the protocol as a `typing.Protocol`).
"""

from modes.simple import SimpleProgram

MODES = {
    "simple": SimpleProgram,
}
