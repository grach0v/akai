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
  H.264 decoder (an ffmpeg process with one frame thread per core) lags ~0.4 s
  (README: "Why the live stream is JPEG"). The stream uses rerun's low-latency batcher.

A stream is an input, or one element of a `sync` tuple input: a tuple (an Arrow struct)
is unpacked into its elements, `<input>/<field>`, each with its own metadata (the tuple's
`<field>.<key>` keys). What a stream holds comes from its metadata:

    encoding rgb8 / jpeg      -> file: rr.VideoStream (H.264), live: rr.EncodedImage
    encoding uint16           -> rr.DepthImage (millimetres)
    numeric, in joint_states  -> rr.Scalars, and the 3D robot's joints (with `robot_urdf`)
    anything else numeric     -> rr.Scalars, series named by `names`

Every message is logged at its time by `input_time`, on a timeline of that name: dora's send
`timestamp` (every message has one) or its `capture_time`. A tuple's elements are logged at
their own times, plus the tuple's index on a timeline named after the input, so a recording of
observations is one row per observation (see `_record_tuple`). A stream is logged at the entity
named like it (`arm_joint_state`, `observation/cam_high`), and a stream missing a key its role
requires stops the node with an error naming it.

With `robot_urdf` set, the robot is shown in 3D under `robot/` (see robot.py),
split the way the `rerun-data-model` / `rerun-urdf` skills split a robot dataset:

    <recording>.rrd          base: the raw streams, as above
    <recording>.fk.rrd       layer: forward kinematics of every joint state (same recording id)
    <urdf name>.model.rrd    asset: the static URDF model, written once, shared by all recordings

The live viewer gets all three. An image whose `camera` is in the `calibration` file is placed
in the robot's frames and drawn inside its frustum in 3D; see `log_calibration`.

The viewer layout (blueprint) is the `blueprint` file's, or generated from the robot, the plots
and the cameras, and sent again when a new camera appears.

Inputs:  program_state, plus any stream wired in the dataflow (give them a deep queue:
         a recorder must not drop messages).
Outputs: node_state (`ready`).
"""

from __future__ import annotations

import logging
import runpy
import socket
from collections.abc import Callable
from datetime import datetime

import av
import numpy as np
import pyarrow as pa
import rerun as rr
import rerun.blueprint as rrb
from akai_scene import CameraCalibration, load_calibration
from dora import Node
from rerun.chunk import OptimizationProfile

from config import RecordModeConfig, RerunConfig
from robot import RobotUrdf

logger = logging.getLogger("rerun")


def listening(bind_ip: str, port: int) -> bool:
    """True if something already accepts TCP connections where a server bound to bind_ip:port
    would be reached: bind_ip itself, or 127.0.0.1 for 0.0.0.0. Connecting, not binding: a
    wildcard bind succeeds next to a server on 127.0.0.1:port, which then keeps local viewers."""
    host = "127.0.0.1" if bind_ip == "0.0.0.0" else bind_ip
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.25)
        return s.connect_ex((host, port)) == 0


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


def require(md: dict, stream: str, *keys: str) -> None:
    """Stop with an error naming the stream when its metadata lacks a key its role requires."""
    missing = [k for k in keys if k not in md]
    if missing:
        raise ValueError(f"{stream}: metadata has no {missing} (see the README's message conventions)")


def message_time(md: dict, key: str, stream: str) -> float:
    """The time under metadata `key`, in seconds since the epoch: a datetime (like dora's send
    `timestamp`) or a number of seconds (like `capture_time`)."""
    require(md, stream, key)
    t = md[key]
    return t.timestamp() if isinstance(t, datetime) else float(t)


def log_calibration(recordings: list[rr.RecordingStream], stream: str, cal: CameraCalibration) -> None:
    """Place a calibrated camera in the robot's frames, the way the `rerun-data-model` skill
    models a camera: `<stream>/calibration` holds a static Transform3D from the URDF frame the
    camera is mounted on to `<stream>_optical_frame` (the extrinsics) and a Pinhole from there
    into `<stream>_image_plane` (the intrinsics), and the stream's images sit in the image plane,
    so the viewer draws them in the frustum."""
    optical, plane = f"{stream}_optical_frame", f"{stream}_image_plane"
    x, y, z, qx, qy, qz, qw = cal.extrinsics
    fx, fy, cx, cy = cal.intrinsics
    extrinsics = rr.Transform3D(
        translation=[x, y, z],
        quaternion=rr.Quaternion(xyzw=[qx, qy, qz, qw]),
        parent_frame=cal.frame,
        child_frame=optical,
    )
    intrinsics = rr.Pinhole(
        resolution=[cal.width, cal.height],
        focal_length=[fx, fy],
        principal_point=[cx, cy],
        camera_xyz=rr.ViewCoordinates.RDF,  # the extrinsics' optical frame: x right, y down, z forward
        image_plane_distance=0.1,
        parent_frame=optical,
        child_frame=plane,
    )
    for rec in recordings:
        rec.log(f"{stream}/calibration", extrinsics, intrinsics, static=True)
        rec.log(stream, rr.CoordinateFrame(plane), static=True)


def build_blueprint(
    cameras: list[str], calibrated: list[str], robot: str | None, plots: dict[str, list[str]]
) -> rrb.Blueprint:
    """The cameras in a grid and the 3D robot (with the calibrated cameras in their frustums) on
    top, the plots below: every stream visible at once, where the automatic
    layout tends to surface just one."""
    top = []
    if cameras:
        top.append(rrb.Grid(*[rrb.Spatial2DView(origin=f"/{cam}", name=cam) for cam in cameras]))
    if robot is not None:
        contents = [f"{robot}/**", *(f"/{cam}/**" for cam in calibrated)]
        top.append(rrb.Spatial3DView(origin=robot, contents=contents, name="robot"))
    if plots:
        views = [
            rrb.TimeSeriesView(name=name, contents=[f"/{s}/**" for s in streams])
            for name, streams in plots.items()
        ]
    else:
        views = [rrb.TimeSeriesView(origin="/", name="plots")]
    rows, shares = [rrb.Horizontal(*top)] if top else [], [3] if top else []
    return rrb.Blueprint(
        rrb.Vertical(*rows, rrb.Horizontal(*views), row_shares=[*shares, 1]), collapse_panels=True
    )


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
        # rerun would only log a failed bind and record without serving
        if listening(opts.bind_ip, opts.grpc_port):
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
        self._calibration = load_calibration(opts.calibration) if opts.calibration else {}  # by camera
        self._images: list[str] = []  # image and depth streams, in the order they appeared
        self._calibrated_size: dict[str, tuple[int, int]] = {}  # stream -> the size it is calibrated for
        self._blueprint = runpy.run_path(str(opts.blueprint))["blueprint"] if opts.blueprint else None
        self._named: set[str] = set()  # series entities whose line label is logged
        self._tuples: dict[str, int] = {}  # tuple input -> index of its next tuple
        self._send_layout()
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
        value, md = event["value"], event["metadata"]
        if pa.types.is_struct(
            value.type
        ):  # a sync tuple: one field per element, its metadata `<field>.<key>`
            self._record_tuple(event["id"], value, md)
        else:
            timestamps = {self.opts.input_time: message_time(md, self.opts.input_time, event["id"])}
            self._set_times(timestamps, {})
            self._record(event["id"], value, md, timestamps, {})
        return False

    def _record_tuple(self, input_id: str, value: pa.StructArray, md: dict) -> None:
        """A tuple as one aligned sample: every element logged at its own time (its
        `<field>.<input_time>` key) and at the tuple's index on a timeline named after the input,
        so a recording of observations is one row per observation and no element's time is lost."""
        index = self._tuples.get(input_id, 0)
        self._tuples[input_id] = index + 1
        sequences = {input_id: index}
        for field in value.type:
            prefix = f"{field.name}."
            element_md = {k.removeprefix(prefix): v for k, v in md.items() if k.startswith(prefix)}
            stream = f"{input_id}/{field.name}"
            timestamps = {self.opts.input_time: message_time(element_md, self.opts.input_time, stream)}
            self._set_times(timestamps, sequences)
            self._record(stream, value.field(field.name)[0].values, element_md, timestamps, sequences)
        for rec in (self.file, self.live):  # set_time sticks: later messages must not carry it
            rec.disable_timeline(input_id)

    def _set_times(self, timestamps: dict[str, float], sequences: dict[str, int]) -> None:
        """Set these timelines on both recordings, for the next logs (`set_time` sticks)."""
        for rec in (self.file, self.live):
            for timeline, seconds in timestamps.items():
                rec.set_time(timeline, timestamp=seconds)
            for timeline, index in sequences.items():
                rec.set_time(timeline, sequence=index)

    def _record(
        self, stream: str, value: pa.Array, md: dict, timestamps: dict[str, float], sequences: dict[str, int]
    ) -> None:
        """One message of a stream, logged by the role the config gives it, at the times already
        set (`_set_times`); they are passed on for the transforms, which are sent as columns."""
        data = value.to_numpy(zero_copy_only=False)
        encoding = md.get("encoding")
        if encoding in ("rgb8", "jpeg", "uint16"):
            require(md, stream, "width", "height")
            self._place_image(stream, md, depth=encoding == "uint16")
            if encoding == "uint16":
                depth = data.reshape(int(md["height"]), int(md["width"]))
                self._log_both(stream, rr.DepthImage(depth, meter=1000.0))
            else:
                self._log_image(stream, data, md)
        elif encoding is not None:
            raise ValueError(f"{stream}: unsupported encoding {encoding!r}")
        else:
            if not (pa.types.is_integer(value.type) or pa.types.is_floating(value.type)):
                raise ValueError(f"{stream}: {value.type} values without an `encoding`")
            if stream in self.opts.joint_states:
                require(md, stream, "names")
            self._log_series(stream, data, md)
            if stream in self.opts.joint_states and self.robot is not None:
                self._send_transforms(md["names"], data.tolist(), timestamps, sequences)

    def _place_image(self, stream: str, md: dict, depth: bool) -> None:
        """On an image stream's first message: add it to the layout, and put it in 3D when its
        `camera` is calibrated (color only: depth stays 2D).
        Every message: check its size against the calibration."""
        if stream not in self._images:
            self._images.append(stream)
            cal = self._calibration.get(md.get("camera"))
            if cal is not None and not depth:
                log_calibration([self.file, self.live], stream, cal)
                self._calibrated_size[stream] = (cal.width, cal.height)
            if self._blueprint is None:
                self._send_layout()  # with this camera, in 3D when calibrated
            in_3d = "3D" if stream in self._calibrated_size else "2D"
            logger.info("record: %s (camera %s, %s)", stream, md.get("camera"), in_3d)
        size = self._calibrated_size.get(stream)
        if size is not None and size != (md["width"], md["height"]):
            raise ValueError(
                f"{stream}: {md['width']}x{md['height']} images, calibrated for {size[0]}x{size[1]}"
            )

    def _send_layout(self) -> None:
        """Send the viewer layout: the `blueprint` file's, or one generated from the robot, the
        plots and the cameras seen so far (sent again when a new camera appears)."""
        blueprint = self._blueprint
        if blueprint is None and (self._images or self.robot is not None or self.opts.plots):
            robot = f"/{self.robot.root}" if self.robot else None
            blueprint = build_blueprint(self._images, list(self._calibrated_size), robot, self.opts.plots)
        if blueprint is not None:  # else: the viewer's automatic layout
            for rec in (self.file, self.live):
                rec.send_blueprint(blueprint)

    # -- logging -------------------------------------------------------------

    def _log_both(self, entity: str, archetype) -> None:
        self.file.log(entity, archetype)
        self.live.log(entity, archetype)

    def _log_image(self, entity: str, data: np.ndarray, md: dict) -> None:
        self.live.log(entity, live_image(data, md, self.opts.live_jpeg_quality))
        frame = decode_image(data, md)
        encoder = self.encoders.get(entity)
        if encoder is None:
            encoder = self.encoders[entity] = H264Encoder(frame.width, frame.height, self.opts)
            self.file.log(entity, rr.VideoStream(codec=rr.VideoCodec.H264), static=True)
        for sample in encoder.encode(frame):
            self.file.log(entity, sample)

    def _send_transforms(
        self, names: list[str], values: list[float], timestamps: dict[str, float], sequences: dict[str, int]
    ) -> None:
        """This joint state's forward kinematics, as one chunk at its times, to the layer and live."""
        update = self.robot.transforms(names, values, timestamps, sequences)
        if update is None:
            return
        indexes, columns = update
        for rec in (self.fk, self.live):
            rec.send_columns(self.robot.transforms_entity, indexes=indexes, columns=columns)

    def _log_series(self, stream: str, data: np.ndarray, md: dict) -> None:
        """A numeric vector. With `names`, every component is its own entity, `<stream>/<name>`, whose
        line is labelled with the stream: plots can then put the same joint of different streams
        side by side (measured next to commanded). Without names, the vector is one entity."""
        if "names" not in md:
            self._log_both(stream, rr.Scalars(data.astype(np.float64)))
            return
        names = md["names"]
        if len(names) != len(data):
            raise ValueError(f"{stream}: {len(data)} values but {len(names)} names")
        for name, value in zip(names, data):
            entity = f"{stream}/{rr.escape_entity_path_part(name)}"
            if entity not in self._named:
                self._named.add(entity)
                for rec in (self.file, self.live):
                    rec.log(entity, rr.SeriesLines(names=[stream]), static=True)
            self._log_both(entity, rr.Scalars(float(value)))
