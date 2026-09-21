"""Mode registry: mode name (config `MODE__NAME`) -> mode class."""

from modes.camera import CameraMode

MODES = {
    "camera": CameraMode,
}
