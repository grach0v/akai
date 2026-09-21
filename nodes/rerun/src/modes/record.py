"""`record` mode — write every incoming stream to an .rrd file and serve it live.

Two rerun recordings, because the file and a live viewer want different things:

* **file**: a `FileSink`. Camera streams become one H.264 `rr.VideoStream` per
  camera, encoded with libx264 as frames arrive (jpeg input is decoded first),
  so a session is a video and not a pile of images.
* **live**: a `GrpcServerSink` any viewer can connect to
  (`rerun --connect rerun+http://<host>:<port>/proxy`), on this machine or on
  another. Camera streams are sent as JPEG images, the camera's own bytes when
  it already produces JPEG. The viewer decodes an image instantly, whereas its
  H.264 decoder holds back 18 samples (600 ms at 30 fps) and restarts on every
  hiccup. The stream uses rerun's low-latency batcher.

Everything that is not an image goes to both recordings, dispatched by shape:

    id ends with `depth`         -> rr.DepthImage (uint16 millimetres)
    id ends with `pose`/`target` -> rr.Transform3D (xyz + xyzw quaternion)
    anything else numeric        -> rr.Scalars (one plot per input)

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

from config import RecordModeConfig, RerunConfig

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


def archetype(entity: str, data: np.ndarray, md: dict):
    """The rerun archetype for a non-image message, dispatched by its shape."""
    if entity.endswith("depth"):
        return rr.DepthImage(data.reshape(int(md["height"]), int(md["width"])), meter=1000.0)
    if entity.endswith(("pose", "target")):
        return rr.Transform3D(translation=data[:3], quaternion=rr.Quaternion(xyzw=data[3:7]))
    return rr.Scalars(data.tolist())


def build_blueprint(cameras: list[str]) -> rrb.Blueprint:
    """All cameras in a grid above one shared time-series plot, so every stream
    is visible at once (the automatic layout tends to surface just one)."""
    grid = rrb.Grid(*[rrb.Spatial2DView(origin=cam, name=cam) for cam in cameras])
    return rrb.Blueprint(
        rrb.Vertical(grid, rrb.TimeSeriesView(origin="/", name="state"), row_shares=[3, 1]),
        collapse_panels=True,
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
        if listening(opts.grpc_port):  # rerun would only log the failed bind and record without serving
            raise RuntimeError(
                f"port {opts.grpc_port} is taken (a leftover rerun viewer or recorder?). "
                f"Close it or set MODE__GRPC_PORT to a free port."
            )
        opts.rrd_dir.mkdir(parents=True, exist_ok=True)
        self.path = opts.rrd_dir / f"{cfg.app_id}_{datetime.now().astimezone():%Y%m%d_%H%M%S}.rrd"
        self.file = rr.RecordingStream(cfg.app_id)
        self.file.set_sinks(rr.FileSink(path=str(self.path)))
        self.live = rr.RecordingStream(cfg.app_id, batcher_config=rr.ChunkBatcherConfig.LOW_LATENCY())
        self.live.set_sinks(
            rr.GrpcServerSink(
                bind_ip=opts.bind_ip, port=opts.grpc_port, server_memory_limit=opts.server_memory_limit
            )
        )
        if opts.cameras:
            for rec in (self.file, self.live):
                rec.send_blueprint(build_blueprint(opts.cameras))
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
        self.live.disconnect()  # stops the server
        logger.info("record: closed %s", self.path)

    # -- inputs --------------------------------------------------------------

    def _on_program_state(self, event: dict) -> bool:
        return event["value"][0].as_py() == "disconnect"

    def _on_stream(self, event: dict) -> bool:
        entity, md = event["id"], event["metadata"]
        data = event["value"].to_numpy(zero_copy_only=False)
        for rec in (self.file, self.live):
            rec.set_time("capture_time", timestamp=md["capture_time"])
        if "encoding" in md:
            self.live.log(entity, live_image(data, md, self.opts.live_jpeg_quality))
            frame = decode_image(data, md)
            encoder = self.encoders.get(entity)
            if encoder is None:
                encoder = self.encoders[entity] = H264Encoder(frame.width, frame.height, self.opts)
                self.file.log(entity, rr.VideoStream(codec=rr.VideoCodec.H264), static=True)
            for sample in encoder.encode(frame):
                self.file.log(entity, sample)
        else:
            arch = archetype(entity, data, md)
            self.file.log(entity, arch)
            self.live.log(entity, arch)
        return False
