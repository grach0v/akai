"""Mode registry: `MODE__NAME` -> mode class (the protocol: akai_node.Mode)."""

from modes.camera import CameraMode

MODES = {
    "camera": CameraMode,
}
