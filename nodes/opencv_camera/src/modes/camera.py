"""`camera` mode — the default: stream frames from one camera device.

* `CameraReader` (background thread) reads frames as the device delivers them and keeps the
  latest, encoded and stamped with its capture time.
* `CameraMode` (dora thread) sends that frame on every `tick`, once: an empty slot sends nothing.
  When the device stops, it reports `finished` and waits for `disconnect` or STOP.

Inputs:  tick (dora/timer), program_state (optional).
Outputs: <camera_name>_image, <camera_name>_node_state.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import cv2
import numpy as np
import pyarrow as pa
from dora import Node

from config import CameraConfig

logger = logging.getLogger("opencv_camera")


@dataclass(frozen=True)
class Frame:
    data: np.ndarray  # flat uint8: raw RGB bytes or a JPEG buffer
    metadata: dict  # camera, encoding, width, height, capture_time


class CameraReader:
    """Background capture thread keeping the latest encoded frame."""

    def __init__(self, cfg: CameraConfig):
        self._name = cfg.camera_name
        self._opts = cfg.mode
        self._cap = self._open()
        self._lock = threading.Lock()
        self._latest: Frame | None = None
        self._ended = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"{self._name}-capture", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._cap.release()

    def take(self) -> Frame | None:
        """The latest frame, or None if it was already taken. Each frame is handed out once."""
        with self._lock:
            frame, self._latest = self._latest, None
            return frame

    @property
    def ended(self) -> bool:
        return self._ended.is_set()

    def _open(self) -> cv2.VideoCapture:
        opts = self._opts
        cap = cv2.VideoCapture(opts.source)
        if not cap.isOpened():
            raise RuntimeError(f"{self._name}: failed to open camera {opts.source!r}")
        if opts.fourcc is not None:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*opts.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, opts.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, opts.height)
        cap.set(cv2.CAP_PROP_FPS, opts.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # no driver queue (V4L2 only)
        got = (
            cap.get(cv2.CAP_PROP_FRAME_WIDTH),
            cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
            cap.get(cv2.CAP_PROP_FPS),
        )
        logger.info(
            "%s: opened %r at %dx%d @ %.0f fps (requested %dx%d @ %d)",
            self._name,
            opts.source,
            *got,
            opts.width,
            opts.height,
            opts.fps,
        )
        return cap

    def _encode(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Flat uint8 array in the configured wire encoding.

        TODO(perf): `jpeg` re-encodes what OpenCV just decoded from the camera's MJPEG; on V4L2,
        CAP_PROP_CONVERT_RGB=0 would pass the camera's JPEG through.
        """
        if self._opts.encoding == "jpeg":
            ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self._opts.jpeg_quality])
            if not ok:
                raise RuntimeError(f"{self._name}: JPEG encode failed")
            return buf.ravel()
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).ravel()

    def _run(self) -> None:
        while not self._stop.is_set():
            ok, frame_bgr = self._cap.read()  # blocks until the next frame
            # TODO(capture time): when read() returned, tens of ms after exposure. Use the driver's
            # buffer timestamp (V4L2), or subtract a delay measured per camera.
            capture_time = time.time()
            if not ok:
                logger.warning("%s: device stopped delivering frames", self._name)
                self._ended.set()
                return
            height, width = frame_bgr.shape[:2]
            metadata = {
                "camera": self._name,
                "encoding": self._opts.encoding,
                "width": width,
                "height": height,
                "capture_time": capture_time,
            }
            frame = Frame(self._encode(frame_bgr), metadata)
            with self._lock:
                self._latest = frame


class CameraMode:
    def __init__(self, cfg: CameraConfig):
        """Open the camera before joining the dataflow, so no tick arrives while it opens."""
        self.name = cfg.camera_name
        self.node: Node | None = None
        self.reader = CameraReader(cfg)
        self.state: str | None = None
        self._handlers: dict[str, Callable[[dict], bool]] = {
            "tick": self._on_tick,
            "program_state": self._on_program_state,
        }

    def start(self, node: Node) -> None:
        self.node = node
        self.reader.start()
        self._set_state("ready")

    def handle(self, event: dict) -> bool:
        """Dispatch one INPUT event. True means: stop the node."""
        return self._handlers[event["id"]](event)  # KeyError on an unwired input id is a wiring bug

    def close(self) -> None:
        self.reader.stop()

    def _on_tick(self, event: dict) -> bool:
        frame = self.reader.take()
        if frame is not None:
            self.node.send_output(f"{self.name}_image", pa.array(frame.data), metadata=frame.metadata)
        if self.reader.ended:
            self._set_state("finished")
        return False

    def _on_program_state(self, event: dict) -> bool:
        return event["value"][0].as_py() == "disconnect"

    def _set_state(self, state: str) -> None:
        """Edge-triggered node_state: emitted only when the state changes."""
        if state != self.state:
            self.state = state
            self.node.send_output(f"{self.name}_node_state", pa.array([state]))
