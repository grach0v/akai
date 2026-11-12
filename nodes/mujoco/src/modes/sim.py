"""`sim` mode — the default: run a MuJoCo scene in real time, captured on every tick.

* `Physics` (background thread) steps the model paced to the wall clock.
* `Cameras` (background thread) renders every camera from a snapshot, on request.
* `SimMode` (the dora thread) writes joint targets to the actuators; on every tick it copies the
  simulation into the snapshot, waits for its render (~7 ms) and sends the joint states and
  frames of that instant, stamped with its capture_time.

Inputs:  tick (dora/timer), program_state, joint_target.
Outputs: <part>_joint_state, <camera>_image, <camera>_depth (with MODE__DEPTH), node_state.
"""

from __future__ import annotations

import io
import logging
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import pyarrow as pa
from akai_node import Mode, Worker
from akai_scene import load_calibration
from PIL import Image

from config import CameraConfig, MujocoConfig

logger = logging.getLogger("mujoco")


@dataclass(frozen=True)
class Part:
    """A part resolved in the model: its actuators, each named after the URDF joint it drives."""

    name: str
    names: list[str]  # the actuators' (= URDF joints') names: what joint states and targets carry
    actuators: np.ndarray  # their ids


def resolve(model_path: Path, parts: dict[str, list[str]]) -> tuple[mujoco.MjModel, list[Part]]:
    """The MJCF loaded, and its parts resolved in it. A name the model does not have raises."""
    model = mujoco.MjModel.from_xml_path(str(model_path))
    resolved = [
        Part(name, names, np.array([model.actuator(a).id for a in names])) for name, names in parts.items()
    ]
    return model, resolved


class Physics(Worker):
    """Steps the model in real time, under the lock it shares with the dora thread."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, lock: threading.Lock):
        super().__init__("mujoco-physics")
        self._model, self._data, self._lock = model, data, lock

    def _run(self) -> None:
        wall0, sim0 = time.monotonic(), self._data.time
        while not self._stop.is_set():
            ahead = (self._data.time - sim0) - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)
                continue
            if -ahead > 0.1:  # racing to catch up would run the robot fast: drop the backlog
                logger.warning("physics %.0f ms behind the wall clock; dropping the backlog", -ahead * 1e3)
                wall0, sim0 = time.monotonic(), self._data.time
            with self._lock:
                mujoco.mj_step(self._model, self._data)


def _render_flag(name: str) -> mujoco.mjtRndFlag:
    """The `mjtRndFlag` named `name` without its `mjRND_` prefix (`SHADOW`)."""
    flag = getattr(mujoco.mjtRndFlag, f"mjRND_{name}", None)
    if flag is None:
        raise ValueError(f"no MuJoCo render flag mjRND_{name}")
    return flag


def depth_mm(depth: np.ndarray) -> np.ndarray:
    """MuJoCo's depth (metres along the optical axis, float) as uint16 millimetres, raveled."""
    return (depth * 1000.0).round().clip(0, 65535).astype(np.uint16).ravel()


def resolution(model: mujoco.MjModel, camera: str) -> tuple[int, int]:
    """(width, height) of an MJCF camera: its `resolution`. An unknown camera raises."""
    width, height = (int(v) for v in model.cam_resolution[model.camera(camera).id])
    if (width, height) == (1, 1):  # MuJoCo's default: none given
        raise ValueError(f"camera {camera}: no `resolution` in the MJCF (the cell builder sets it)")
    return width, height


class Cameras(Worker):
    """Owns the offscreen renderers, one per image size: renders every camera from the snapshot,
    on request, at its MJCF resolution."""

    def __init__(self, model: mujoco.MjModel, snapshot: mujoco.MjData, opts: CameraConfig, depth: list[str]):
        super().__init__("mujoco-render")
        self.size = {cam: resolution(model, cam) for cam in opts.cameras}
        self.depth = depth  # the cameras that also render depth
        self._flags = {_render_flag(name): on for name, on in opts.render_flags.items()}
        self._model, self._snapshot, self._opts = model, snapshot, opts
        self._todo: queue.SimpleQueue = queue.SimpleQueue()  # the capture to render
        self._done: queue.SimpleQueue = queue.SimpleQueue()  # (capture, frames) rendered

    def render(self, capture) -> tuple | None:
        """Render every camera from the snapshot and wait for it: (capture, {camera: (image, depth
        or None)}), or None if the render thread has died."""
        self._todo.put(capture)
        while True:
            try:
                return self._done.get(timeout=0.1)
            except queue.Empty:
                if self.ended:
                    return None

    def _encode(self, rgb: np.ndarray) -> np.ndarray:
        if self._opts.encoding == "jpeg":
            buf = io.BytesIO()
            Image.fromarray(rgb).save(buf, format="JPEG", quality=self._opts.jpeg_quality)
            return np.frombuffer(buf.getvalue(), dtype=np.uint8)
        return rgb.ravel()

    def _run(self) -> None:
        # The GL context belongs to the thread that creates it, so the renderers are made here.
        renderers = {}
        for width, height in set(self.size.values()):
            renderers[width, height] = renderer = mujoco.Renderer(self._model, height=height, width=width)
            for flag, on in self._flags.items():
                renderer.scene.flags[flag] = on
        try:
            while not self._stop.is_set():
                try:
                    capture = self._todo.get(timeout=0.1)
                except queue.Empty:
                    continue
                frames = {}
                for cam in self._opts.cameras:
                    renderer = renderers[self.size[cam]]
                    renderer.update_scene(self._snapshot, camera=cam)
                    image, depth = self._encode(renderer.render()), None
                    if cam in self.depth:
                        renderer.enable_depth_rendering()
                        depth = depth_mm(renderer.render())
                        renderer.disable_depth_rendering()
                    frames[cam] = (image, depth)
                self._done.put((capture, frames))
        finally:
            for renderer in renderers.values():
                renderer.close()


class SimMode(Mode):
    def __init__(self, cfg: MujocoConfig):
        """Load the scene and put it in its home pose before joining the dataflow."""
        super().__init__()
        self.opts = opts = cfg.mode
        self.model, self.parts = resolve(cfg.model, cfg.parts)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        mujoco.mj_forward(self.model, self.data)
        self._lock = threading.Lock()
        self.physics = self._physics()
        self.snapshot = mujoco.MjData(self.model)  # what a tick captures into
        calibration = load_calibration(cfg.calibration) if cfg.calibration else {}
        depth = (
            [cam for cam in opts.cameras if cam in calibration and calibration[cam].depth]
            if opts.depth
            else []
        )
        if opts.depth and not depth:
            raise ValueError("MODE__DEPTH: none of MODE__CAMERAS is RGB-D (`depth: true`) in the calibration")
        self.cameras = Cameras(self.model, self.snapshot, opts, depth) if opts.cameras else None
        self._workers: list[Worker] = [self.physics] + ([self.cameras] if self.cameras else [])
        self.handlers["tick"] = self._on_tick
        self.handlers["joint_target"] = self._on_joint_target
        # URDF joint name -> the actuator driving it.
        self._actuator_of: dict[str, int] = {
            name: int(actuator) for part in self.parts for name, actuator in zip(part.names, part.actuators)
        }
        logger.info(
            "sim: %s, parts %s, cameras %s (depth %s), timestep %.4f s",
            cfg.model,
            {p.name: p.names for p in self.parts},
            opts.cameras,
            depth,
            self.model.opt.timestep,
        )

    def _physics(self) -> Worker:
        """The thread that moves the simulation: here in real time."""
        return Physics(self.model, self.data, self._lock)

    # -- node protocol -------------------------------------------------------

    def start(self, node) -> None:
        for worker in self._workers:
            worker.start()
        super().start(node)

    def close(self) -> None:
        for worker in self._workers:
            worker.stop()

    # -- inputs --------------------------------------------------------------

    def _on_tick(self, event: dict) -> bool:
        if any(worker.ended for worker in self._workers):
            self.set_state("finished")  # a background thread died; its traceback is in the log
            return False
        capture = self._capture()
        if self.cameras is None:
            self._publish(capture, {})
            return False
        done = self.cameras.render(capture)  # waits for it (~7 ms); physics keeps running
        if done is not None:
            self._publish(*done)
        return False

    def _capture(self) -> tuple[float, list[np.ndarray]]:
        """Copy the simulation into the snapshot, in one go under the lock: its capture_time and every
        part's joint positions: its actuators' lengths, in URDF units. The cameras render from the same
        snapshot (lengths and rendered poses both come from the step's kinematics)."""
        with self._lock:
            mujoco.mj_copyData(self.snapshot, self.model, self.data)
            capture_time = time.time()
        return capture_time, [self.snapshot.actuator_length[part.actuators].copy() for part in self.parts]

    def _publish(self, capture: tuple[float, list[np.ndarray]], frames: dict[str, tuple]) -> None:
        """Every part's joint positions and every camera's frame (and depth) of one capture, all
        stamped with its capture_time."""
        capture_time, positions = capture
        for part, position in zip(self.parts, positions):
            self.node.send_output(
                f"{part.name}_joint_state",
                pa.array(position, type=pa.float64()),
                metadata={"names": part.names, "capture_time": capture_time},
            )
        for cam, (image, depth) in frames.items():
            width, height = self.cameras.size[cam]
            md = {"camera": cam, "width": width, "height": height, "capture_time": capture_time}
            self.node.send_output(
                f"{cam}_image", pa.array(image), metadata={**md, "encoding": self.opts.encoding}
            )
            if depth is not None:
                self.node.send_output(f"{cam}_depth", pa.array(depth), metadata={**md, "encoding": "uint16"})

    def _on_joint_target(self, event: dict) -> bool:
        """A named vector of joint targets, by URDF joint name and in URDF units: each value goes to
        the actuator of that name, where physics applies it from its next step (MuJoCo clamps it to
        the actuator's control range). A name the scene has no actuator for is a wiring bug."""
        names, values = event["metadata"]["names"], event["value"].to_pylist()
        if len(names) != len(values):
            raise ValueError(f"joint_target: {len(values)} values but {len(names)} names")
        unknown = [n for n in names if n not in self._actuator_of]
        if unknown:
            raise ValueError(f"joint_target: the scene drives no joint named {unknown}")
        actuators = np.array([self._actuator_of[n] for n in names])
        with self._lock:
            self.data.ctrl[actuators] = values
        return False
