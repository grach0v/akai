"""`record` mode — write every incoming stream to an .rrd file and serve it live.

A session is one rerun recording (`<app_id>_<YYYYmmdd_HHMMSS>`, also its recording
id), sent to two sinks, because the file and a live viewer want different things:

* **file**: a `FileSink`. Camera streams become one H.264 `rr.VideoStream` per
  camera, encoded with libx264 as frames arrive (jpeg input is decoded first),
  so a session is a video and not a pile of images.
* **live**: a `GrpcServerSink` any viewer can connect to
  (`rerun --connect rerun+http://<host>:<port>/proxy`), on this machine or on
  another. Camera streams are sent as JPEG images, the camera's own bytes when
  it already produces JPEG. The viewer decodes an image instantly, whereas its
  H.264 decoder holds back 18 samples (600 ms at 30 fps) and restarts on every
  hiccup. The stream uses rerun's low-latency batcher.

The config says what each input holds; any input it does not list is plotted:

    listed in `cameras`       -> file: rr.VideoStream (H.264), live: rr.EncodedImage
    listed in `depth`         -> rr.DepthImage (uint16 millimetres)
    listed in `joint_states`  -> rr.Scalars, and the 3D robot's joints (with `robot_urdf`)
    anything else numeric     -> rr.Scalars, series named by `names`

A numeric stream of a robot part (metadata `part`) is logged under `<part>/`
(`arm/arm_joint_state`, `arm/arm_joint_target`), so a part's measured and
commanded joints share one plot; any other stream is logged under its input id.

With `robot_urdf` set, the robot is shown in 3D under `robot/` (see robot.py),
split the way the `rerun-data-model` / `rerun-urdf` skills split a robot dataset:

    <recording>.rrd          base: the raw streams, as above
    <recording>.fk.rrd       layer: forward kinematics of every joint state (same recording id)
    <urdf name>.model.rrd    asset: the static URDF model, written once, shared by all recordings

The live viewer gets all three. A calibrated camera (image metadata `frame`,
`extrinsics`, `intrinsics`) is placed in the robot's frames and its images drawn
inside its frustum in 3D; see `_camera_entity`.

Every message is logged at its `capture_time`, the repo-wide metadata key, so
streams line up by when they were captured, not when they arrived.

Inputs:  program_state, plus any stream wired in the dataflow (recorders keep
         the default queue, they must not drop frames).
Outputs: node_state (`ready`).
"""

from __future__ import annotations

import logging
import socket
from collections.abc import Callable
from datetime import datetime

import av
import numpy as np
import pyarrow as pa
import rerun as rr
import rerun.blueprint as rrb
from dora import Node
from rerun.chunk import OptimizationProfile

from config import RecordModeConfig, RerunConfig
from robot import RobotUrdf

logger = logging.getLogger("rerun")


def listening(port: int) -> bool:
    """True if something already accepts TCP connections on 127.0.0.1:port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.25)
        return s.connect_ex(("127.0.0.1", port)) == 0


class H264Encoder:
    """libx264 for one camera. Yields Annex B samples with SPS/PPS in-band at
    every keyframe, so a viewer can start decoding at any keyframe; B-frames
    are off so samples arrive in display order."""

    def __init__(self, width: int, height: int, opts: RecordModeConfig):
        self._codec = av.CodecContext.create("libx264", "w")
        self._codec.width, self._codec.height = width, height
        self._codec.pix_fmt = "yuv420p"
        self._codec.framerate = opts.video_fps
        self._codec.max_b_frames = 0
        self._codec.options = {
            "preset": opts.h264_preset,
            "tune": "zerolatency",
            "x264-params": f"keyint={opts.keyframe_interval}:min-keyint={opts.keyframe_interval}:scenecut=0:repeat-headers=1",
        }
        self._pts = 0

    def encode(self, frame: av.VideoFrame) -> list[rr.VideoStream]:
        # A frame decoded from JPEG arrives typed as an I-frame and libx264 would
        # honour that, making every sample a keyframe; the viewer then restarts
        # its decoder on each one and never shows a frame. Let x264 decide.
        frame.pict_type = av.video.frame.PictureType.NONE
        frame.pts = self._pts
        self._pts += 1
        return [self._sample(p) for p in self._codec.encode(frame)]

    def flush(self) -> list[rr.VideoStream]:
        return [self._sample(p) for p in self._codec.encode(None)]

    @staticmethod
    def _sample(packet: av.Packet) -> rr.VideoStream:
        return rr.VideoStream.from_fields(sample=bytes(packet), is_keyframe=bool(packet.is_keyframe))


def decode_image(data: np.ndarray, md: dict) -> av.VideoFrame:
    """A dora image message as a yuv420p frame ready for the encoder."""
    if md["encoding"] == "rgb8":
        rgb = data.reshape(int(md["height"]), int(md["width"]), 3)
        return av.VideoFrame.from_ndarray(rgb, format="rgb24").reformat(format="yuv420p")
    if md["encoding"] == "jpeg":
        (frame,) = av.CodecContext.create("mjpeg", "r").decode(av.Packet(data.tobytes()))
        return frame.reformat(format="yuv420p")
    raise ValueError(f"unsupported image encoding {md['encoding']!r}")


def live_image(data: np.ndarray, md: dict, jpeg_quality: int) -> rr.EncodedImage:
    """A dora image message as a JPEG for the live viewer: the camera's own bytes
    when it sends JPEG, otherwise compressed here."""
    if md["encoding"] == "jpeg":
        return rr.EncodedImage(contents=data.tobytes(), media_type="image/jpeg")
    if md["encoding"] == "rgb8":
        rgb = data.reshape(int(md["height"]), int(md["width"]), 3)
        return rr.Image(rgb, color_model="RGB").compress(jpeg_quality=jpeg_quality)
    raise ValueError(f"unsupported image encoding {md['encoding']!r}")


def build_blueprint(cameras: dict[str, str], robot: str | None, plots: list[str]) -> rrb.Blueprint:
    """Cameras in a grid and the 3D robot on top, one time-series plot per part (or
    stream) below, so every stream is visible at once (the automatic layout tends
    to surface just one). `cameras`: view name (the input id) -> the entity its images land on."""
    top = []
    if cameras:
        top.append(
            rrb.Grid(*[rrb.Spatial2DView(origin=entity, name=name) for name, entity in cameras.items()])
        )
    if robot is not None:
        top.append(rrb.Spatial3DView(origin=robot, name="robot"))
    rows, shares = [], []
    if top:
        rows.append(rrb.Horizontal(*top))
        shares.append(3)
    if plots:
        rows.append(rrb.Horizontal(*[rrb.TimeSeriesView(origin=p, name=p) for p in plots]))
        shares.append(1)
    return rrb.Blueprint(rrb.Vertical(*rows, row_shares=shares), collapse_panels=True)


class RecordMode:
    def __init__(self, cfg: RerunConfig):
        """Open the file and the server before joining the dataflow."""
        self.cfg = cfg
        self.opts = opts = cfg.mode
        self.node: Node | None = None
        self.encoders: dict[str, H264Encoder] = {}  # one per camera entity, created on its first frame
        self._handlers: dict[str, Callable[[dict], bool]] = {
            "program_state": self._on_program_state,
        }
        if listening(opts.grpc_port):  # rerun would only log the failed bind and record without serving
            raise RuntimeError(
                f"port {opts.grpc_port} is taken (a leftover rerun viewer or recorder?). "
                f"Close it or set MODE__GRPC_PORT to a free port."
            )
        opts.rrd_dir.mkdir(parents=True, exist_ok=True)
        self.recording_id = f"{cfg.app_id}_{datetime.now().astimezone():%Y%m%d_%H%M%S}"
        self.path = opts.rrd_dir / f"{self.recording_id}.rrd"
        self.file = rr.RecordingStream(cfg.app_id, recording_id=self.recording_id)
        self.file.set_sinks(rr.FileSink(path=str(self.path)))
        self.live = rr.RecordingStream(
            cfg.app_id, recording_id=self.recording_id, batcher_config=rr.ChunkBatcherConfig.LOW_LATENCY()
        )
        self.live.set_sinks(
            rr.GrpcServerSink(
                bind_ip=opts.bind_ip, port=opts.grpc_port, server_memory_limit=opts.server_memory_limit
            )
        )
        self.robot = RobotUrdf(opts.robot_urdf) if opts.robot_urdf else None
        self.fk: rr.RecordingStream | None = None
        if self.robot is not None:
            model_path = opts.rrd_dir / f"{opts.robot_urdf.stem}.model.rrd"
            self.robot.model().collect(optimize=OptimizationProfile.OBJECT_STORE).write_rrd(
                model_path, application_id=cfg.app_id, recording_id=f"{opts.robot_urdf.stem}_model"
            )
            self.live.send_chunks(self.robot.model())
            # A layer of this recording: its own properties would collide with the base's in a catalog.
            self.fk = rr.RecordingStream(cfg.app_id, recording_id=self.recording_id, send_properties=False)
            self.fk.set_sinks(rr.FileSink(path=str(opts.rrd_dir / f"{self.recording_id}.fk.rrd")))
            logger.info(
                "record: robot model %s, forward kinematics layer %s.fk.rrd", model_path, self.recording_id
            )
        # The layout: the cameras (by input id: where their images land, which moves into
        # the robot once a calibrated image arrives), the robot, and one plot per part or
        # stream in arrival order. The blueprint is re-sent whenever it changes.
        self._image_entity: dict[str, str] = {cam: cam for cam in opts.cameras}
        self._calibrated: set[str] = set()  # input ids whose camera calibration is logged
        self._plots: list[str] = []
        self._named: set[str] = set()  # series entities already seen (their names logged)
        self._send_blueprint()
        logger.info(
            "record: writing %s, serving rerun+http://%s:%d/proxy", self.path, opts.bind_ip, opts.grpc_port
        )

    # -- node protocol -------------------------------------------------------

    def start(self, node: Node) -> None:
        self.node = node
        node.send_output("node_state", pa.array(["ready"]))

    def handle(self, event: dict) -> bool:
        """Dispatch one INPUT event. True means: stop the node. A sink accepts any
        input id: everything that is not a control input is a stream to record."""
        handler = self._handlers.get(event["id"], self._on_stream)
        return handler(event)

    def close(self) -> None:
        for entity, encoder in self.encoders.items():
            for sample in encoder.flush():
                self.file.log(entity, sample)
        self.file.disconnect()  # flushes and closes the file
        if self.fk is not None:
            self.fk.disconnect()
        self.live.disconnect()  # stops the server
        logger.info("record: closed %s", self.path)

    # -- inputs --------------------------------------------------------------

    def _on_program_state(self, event: dict) -> bool:
        return event["value"][0].as_py() == "disconnect"

    def _on_stream(self, event: dict) -> bool:
        input_id, md = event["id"], event["metadata"]
        data = event["value"].to_numpy(zero_copy_only=False)
        for rec in (self.file, self.live):
            rec.set_time("capture_time", timestamp=md["capture_time"])
        if input_id in self.opts.cameras:
            self._log_image(self._camera_entity(input_id, md), data, md)
        elif input_id in self.opts.depth:
            depth = data.reshape(int(md["height"]), int(md["width"]))
            self._log_both(input_id, rr.DepthImage(depth, meter=1000.0))
        else:
            self._log_series(input_id, data, md)
            if input_id in self.opts.joint_states and self.robot is not None:
                self._send_transforms(md["names"], data.tolist(), md["capture_time"])
        return False

    # -- logging -------------------------------------------------------------

    def _log_both(self, entity: str, archetype) -> None:
        self.file.log(entity, archetype)
        self.live.log(entity, archetype)

    def _camera_entity(self, input_id: str, md: dict) -> str:
        """Where an image goes. A calibrated camera of a recording with a robot goes into the
        robot's frames, the way the `rerun-data-model` skill models a camera: on its first image,
        `robot/cameras/<camera>` gets the extrinsics (a static Transform3D from the URDF frame
        the camera is mounted on to `<camera>_optical_frame`) and the intrinsics (a Pinhole from
        there into `<camera>_image_plane`), and `robot/cameras/<camera>/image`, where its images
        go, sits in the image plane, so the viewer draws them in the frustum. Any other image
        goes to its input id."""
        if self.robot is None or "extrinsics" not in md:
            return input_id
        camera = md["camera"]
        entity = f"{self.robot.root}/cameras/{rr.escape_entity_path_part(camera)}"
        image_entity = f"{entity}/image"
        if input_id not in self._calibrated:
            self._calibrated.add(input_id)
            optical, plane = f"{camera}_optical_frame", f"{camera}_image_plane"
            x, y, z, qx, qy, qz, qw = md["extrinsics"]
            fx, fy, cx, cy = md["intrinsics"]
            extrinsics = rr.Transform3D(
                translation=[x, y, z],
                quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw]),
                parent_frame=md["frame"],
                child_frame=optical,
            )
            intrinsics = rr.Pinhole(
                resolution=[int(md["width"]), int(md["height"])],
                focal_length=[fx, fy],
                principal_point=[cx, cy],
                camera_xyz=rr.ViewCoordinates.RDF,  # the extrinsics' optical frame: x right, y down, z forward
                image_plane_distance=0.1,
                parent_frame=optical,
                child_frame=plane,
            )
            for rec in (self.file, self.live):
                rec.log(entity, extrinsics, intrinsics, static=True)
                rec.log(image_entity, rr.CoordinateFrame(plane), static=True)
            if input_id in self._image_entity:
                self._image_entity[input_id] = image_entity
                self._send_blueprint()
        return image_entity

    def _log_image(self, entity: str, data: np.ndarray, md: dict) -> None:
        self.live.log(entity, live_image(data, md, self.opts.live_jpeg_quality))
        frame = decode_image(data, md)
        encoder = self.encoders.get(entity)
        if encoder is None:
            encoder = self.encoders[entity] = H264Encoder(frame.width, frame.height, self.opts)
            self.file.log(entity, rr.VideoStream(codec=rr.VideoCodec.H264), static=True)
        for sample in encoder.encode(frame):
            self.file.log(entity, sample)

    def _send_transforms(self, names: list[str], values: list[float], capture_time: float) -> None:
        """This joint state's forward kinematics, as one chunk at its capture time, to the layer and live."""
        update = self.robot.transforms(names, values, capture_time)
        if update is None:
            return
        indexes, columns = update
        for rec in (self.fk, self.live):
            rec.send_columns(self.robot.transforms_entity, indexes=indexes, columns=columns)

    def _log_series(self, input_id: str, data: np.ndarray, md: dict) -> None:
        """A numeric vector as one series per component. A part's streams share its plot."""
        if "part" in md:
            entity, plot = f"{md['part']}/{input_id}", md["part"]
        else:
            entity = plot = input_id
        if entity not in self._named:
            self._named.add(entity)
            if "names" in md:
                for rec in (self.file, self.live):
                    rec.log(entity, rr.SeriesLines(names=list(md["names"])), static=True)
            if plot not in self._plots:
                self._plots.append(plot)
                self._send_blueprint()
        self._log_both(entity, rr.Scalars(data.astype(np.float64)))

    def _send_blueprint(self) -> None:
        """Only with something to arrange (cameras or a robot); otherwise the viewer's automatic layout."""
        if not self.opts.cameras and self.robot is None:
            return
        blueprint = build_blueprint(
            self._image_entity, f"/{self.robot.root}" if self.robot else None, self._plots
        )
        for rec in (self.file, self.live):
            rec.send_blueprint(blueprint)
