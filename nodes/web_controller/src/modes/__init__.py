"""Mode registry: `MODE__NAME` -> mode class (the protocol: akai_node.Mode).

`pose` mode extends `joint` mode (its board, page and answers).
"""

from modes.joint import JointMode
from modes.pose import PoseMode

MODES = {
    "joint": JointMode,
    "pose": PoseMode,
}
