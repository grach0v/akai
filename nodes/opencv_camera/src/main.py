"""opencv_camera node: the shared runtime (akai_node) running this node's modes; see README.md."""

from akai_node import run

from config import CameraConfig
from modes import MODES

if __name__ == "__main__":
    run(MODES, CameraConfig)
