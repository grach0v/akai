# rerun

The [rerun](https://rerun.io) node (rerun-sdk 0.38.1), with two modes selected
by `MODE__NAME`:

- **`record`** writes every incoming stream to an `.rrd` file **and** serves
  it live over gRPC. In the file, camera images are one H.264 video stream per
  camera, not a pile of images. On the live server they are JPEG images, for
  the lowest possible viewer latency (see below).
- **`visualize`** opens a rerun viewer connected to a `record` server. It logs
  nothing itself, but reports the window's state to the manager.

The recorder is the hub and the viewer is a client. That is deliberate: in
remote teleoperation the recorder runs next to the robot and the viewer on the
operator's machine, connected over the network to the same url. The recorder
never depends on a viewer being present, and a viewer can join or leave at any
time.

The source directory is `src`, not `rerun`: a package named `rerun` would shadow
the SDK's `import rerun`.

## Layout

```
pyproject.toml         # uv project: dora-rs, rerun-sdk==0.38.1, av (H.264), pydantic-settings
src/config.py          # RerunConfig: node fields + `mode`, one config class per mode picked by MODE__NAME
src/main.py            # dora skeleton, identical in every node: config -> mode -> Node -> loop
src/modes/record.py    # `record`: file recording (H.264) + live server (JPEG)
src/modes/visualize.py # `visualize`: spawn the viewer on the server url, close it on stop
```

## `record` mode

Two rerun recordings, because a file and a live viewer want different things:

- **file**: a `FileSink` writing `<rrd_dir>/<app_id>_<YYYYmmdd_HHMMSS>.rrd`.
  Cameras are recorded as H.264 (`rr.VideoStream`, libx264, a keyframe with
  SPS/PPS every `keyframe_interval` frames so playback can start at any of them).
- **live**: a `GrpcServerSink` at `rerun+http://<bind_ip>:<grpc_port>/proxy`
  with rerun's low-latency batcher. Cameras are sent as JPEG: the camera's own
  bytes when it already sends JPEG (no CPU spent), otherwise compressed here.

Why not H.264 live as well: the native viewer decodes H.264 through an
external ffmpeg process and holds back 18 samples before it shows a frame,
600 ms at 30 fps, and it restarts that decoder on every hiccup, which shows
as a loading spinner. A JPEG is decoded the moment it arrives. Measured on
this stack, a frame is on the live server 6 ms (median) after its capture.

A taken port is refused at startup rather than recorded around.

Dispatch is by message shape, so any stream can be wired in without config.
Each input id becomes the rerun entity path:

| message | recorded as |
| --- | --- |
| `uint8[N]` + metadata `encoding=rgb8`/`jpeg`, `width`, `height` | file: `rr.VideoStream` (H.264); live: `rr.EncodedImage` (JPEG) |
| id ends with `depth`, `uint16[w*h]` | `rr.DepthImage` (millimetres) |
| id ends with `pose` or `target`, `float64[7]` `[x y z qx qy qz qw]` | `rr.Transform3D` |
| any other numeric vector | `rr.Scalars` (one plot per input) |

Every message is logged at its `capture_time`, so streams line up by when they
were captured. Encoders are flushed and the file closed on stop.

### Inputs and outputs

| id              | source                  | meaning |
| --------------- | ----------------------- | ------- |
| `program_state` | `manager/program_state` | the node stops on `disconnect` |
| `<any id>`      | any stream              | recorded; keep the default queue so no frame is dropped |

Output `node_state`: `ready` once the sinks are up.

### Config

| var | default | meaning |
| --- | --- | --- |
| `APP_ID` | `akai` | rerun application id; also names the file |
| `MODE__NAME` | `record` | |
| `MODE__RRD_DIR` | `out/rrd` | where the `.rrd` goes, relative to the dataflow directory |
| `MODE__BIND_IP` | `0.0.0.0` | server bind address; `0.0.0.0` admits viewers from other machines |
| `MODE__GRPC_PORT` | `9876` | server port |
| `MODE__SERVER_MEMORY_LIMIT` | `256MB` | how much the server buffers for a late-joining viewer |
| `MODE__VIDEO_FPS` | `30` | encoder time base; match the cameras |
| `MODE__H264_PRESET` | `veryfast` | libx264 preset: `ultrafast` … `medium`, CPU vs file size |
| `MODE__KEYFRAME_INTERVAL` | `60` | frames between H.264 keyframes in the file (2 s at 30 fps) |
| `MODE__LIVE_JPEG_QUALITY` | `80` | JPEG quality for the live stream when a camera sends rgb8 |
| `MODE__CAMERAS` | `[]` | JSON list of camera input ids laid out in a grid above one time-series plot in the viewer, e.g. `'["cam_high"]'` |

## `visualize` mode

Waits until the record server accepts connections (a viewer does not retry a
failed connection), then starts the viewer bundled with rerun-sdk on the server
url in its own process session, and closes it when the node stops. The
viewer's own log lines appear in the node's output. The blueprint comes from
the recorder.

The window is the operator's UI, so the node reports `node_state` like a
producer: `ready` once the viewer is spawned, `finished` once the operator
closes the window (noticed on `tick`). In the manager's `simple` mode that
ends the program; a robot program's mode decides for itself what a closed
viewer means.

| id              | source                    | meaning |
| --------------- | ------------------------- | ------- |
| `tick`          | `dora/timer/millis/500`   | polls whether the viewer window is still open |
| `program_state` | `manager/program_state`   | the node stops on `disconnect` |

Output `node_state`: `ready`, `finished`.

| var | default | meaning |
| --- | --- | --- |
| `MODE__NAME` | `visualize` | |
| `MODE__SERVER_URL` | `rerun+http://127.0.0.1:9876/proxy` | the record server; another machine: its address instead of 127.0.0.1 |
| `MODE__SERVER_TIMEOUT` | `15` | seconds to wait for that server before giving up |
| `MODE__MEMORY_LIMIT` | `2GB` | viewer memory budget |

Any rerun viewer works as well: `rerun --connect rerun+http://<robot>:9876/proxy`
(`--connect`, or the viewer's own server would take port 9876 on that machine).

## Dataflow snippet

```yaml
  - id: rerun_record
    build: uv sync --project ../../../nodes/rerun
    path: ../../../nodes/rerun/.venv/bin/python
    args: ../../../nodes/rerun/src/main.py
    inputs:
      program_state: manager/program_state
      cam_high: cam_high/cam_high_image
    outputs: [node_state]
    env:
      MODE__NAME: "record"
      MODE__CAMERAS: '["cam_high"]'

  - id: rerun_viewer
    build: uv sync --project ../../../nodes/rerun
    path: ../../../nodes/rerun/.venv/bin/python
    args: ../../../nodes/rerun/src/main.py
    inputs:
      tick: { source: dora/timer/millis/500, queue_size: 1, queue_policy: drop_oldest }
      program_state: manager/program_state
    outputs: [node_state]
    env:
      MODE__NAME: "visualize"
```
