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
src/robot.py           # a URDF shown in 3D, moved by joint states (record mode, rerun.urdf.UrdfTree)
src/modes/record.py    # `record`: file recording (H.264) + live server (JPEG)
src/modes/visualize.py # `visualize`: spawn the viewer on the server url, close it on stop
```

## `record` mode

A session is one rerun recording, `<app_id>_<YYYYmmdd_HHMMSS>` (also its
recording id), sent to two sinks, because a file and a live viewer want different
things:

- **file**: a `FileSink` writing `<rrd_dir>/<recording>.rrd`.
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

The config says what each input holds (the streams' metadata is in the
top-level README's message conventions); an input it does not list is plotted:

| input | recorded as | entity |
| --- | --- | --- |
| listed in `MODE__CAMERAS`: `uint8[N]` + `encoding=rgb8`/`jpeg`, `width`, `height` | file: `rr.VideoStream` (H.264); live: `rr.EncodedImage` (JPEG) | the input id, or the camera's frustum when calibrated (below) |
| listed in `MODE__DEPTH`: `uint16[w*h]` + `width`, `height` | `rr.DepthImage` (millimetres) | the input id |
| listed in `MODE__JOINT_STATES`: a named vector | `rr.Scalars`, and the 3D robot's joints (below) | as any numeric vector |
| any other numeric vector | `rr.Scalars`, series named by metadata `names` | `<part>/<input id>` with metadata `part`, else the input id |

So a part's measured and commanded joints share one plot (`<part>`).

**3D robot** (`MODE__ROBOT_URDF`): the robot's URDF, through rerun's own
`rerun.urdf.UrdfTree`, following the `rerun-data-model` and `rerun-urdf` skills
(`.claude/skills/`). A session with a robot writes three files, one logical recording
plus a shared asset:

| file | what | why separate |
| --- | --- | --- |
| `<recording>.rrd` | base: the raw streams (cameras, joint states and targets, ...) | the faithful record, nothing computed |
| `<recording>.fk.rrd` | layer: `Transform3D` rows from forward kinematics, on `/robot/transforms`, same recording id | derived data; recomputable from base + URDF |
| `<urdf name>.model.rrd` | asset: the static model (visual meshes, joint transforms at rest on `/robot/tf_static`), recording id `<urdf name>_model` | identical for every recording: written once, registered on a catalog dataset as an asset (`dataset.register_asset`) so the viewer loads it with every segment |

The live viewer gets all three. Every message on a `MODE__JOINT_STATES` input is turned into
transforms for the joints its `names` metadata lists, which must be the URDF's
joint names (a producer maps its own onto them: the sim through the scene
descriptor's `urdf_joints` / `urdf_scale`). A joint the URDF does not have is
skipped with one warning. Mimic joints, which a gripper reports only through its
driver, get `driver * multiplier + offset` as the URDF declares; the
transforms' parent and child frames are the URDF link names, the same the model
uses, so the chain connects.

A calibrated camera (image metadata `frame`, `extrinsics`, `intrinsics`) is placed
in the robot's frames, the way the `rerun-data-model` skill models a camera. On its
first image, `robot/cameras/<camera>` gets a static `Transform3D` from `frame` to
`<camera>_optical_frame` (extrinsics) and a `Pinhole` from there into
`<camera>_image_plane` (intrinsics), and its images go to
`robot/cameras/<camera>/image`, which sits in the image plane: the viewer draws
them inside the frustum, and a camera mounted on the arm (`tool0`) moves with
it. Other images stay under their input id, and every camera also gets a 2D view.

**Layout**: with cameras or a robot configured, the blueprint puts the
camera grid and the 3D view on top and one time-series plot per part (or other
numeric stream) below; it is re-sent as new streams appear. Otherwise the
viewer's automatic layout decides.

Every message is logged at its `capture_time`, which every stream must carry, so
streams line up by when they were captured. Encoders are flushed and the file
closed on stop.

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
| `MODE__RRD_DIR` | `out/rrd` | where the `.rrd` files go (recording, FK layer, robot model), relative to the dataflow directory |
| `MODE__BIND_IP` | `0.0.0.0` | server bind address; `0.0.0.0` admits viewers from other machines |
| `MODE__GRPC_PORT` | `9876` | server port |
| `MODE__SERVER_MEMORY_LIMIT` | `256MB` | how much the server buffers for a late-joining viewer |
| `MODE__VIDEO_FPS` | `30` | encoder time base; match the cameras |
| `MODE__H264_PRESET` | `veryfast` | libx264 preset: `ultrafast` … `medium`, CPU vs file size |
| `MODE__KEYFRAME_INTERVAL` | `60` | frames between H.264 keyframes in the file (2 s at 30 fps) |
| `MODE__LIVE_JPEG_QUALITY` | `80` | JPEG quality for the live stream when a camera sends rgb8 |
| `MODE__CAMERAS` | `[]` | JSON list of the image inputs: recorded as video, laid out in a grid in the viewer, e.g. `'["cam_high"]'` |
| `MODE__DEPTH` | `[]` | JSON list of the depth-image inputs |
| `MODE__JOINT_STATES` | `[]` | JSON list of the joint-state inputs, which also move the 3D robot |
| `MODE__ROBOT_URDF` | unset | robot URDF shown in 3D and moved by `joint_state` streams, relative to the dataflow directory |

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
