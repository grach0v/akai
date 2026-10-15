# rerun

The [rerun](https://rerun.io) node (rerun-sdk 0.39.0), with two modes selected
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
pyproject.toml         # uv project: dora-rs, rerun-sdk==0.39.0, av (H.264), pydantic-settings
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

A taken port is refused at startup rather than recorded around.

### Why the live stream is JPEG, not H.264

H.264 would let both sinks share one recording, but the live view would lag
about 0.4 s. rerun's viewer decodes H.264 through ffmpeg with one frame thread
per core, and ffmpeg holds back roughly one frame per thread. A JPEG shows up
as soon as it arrives.

Measured with rerun 0.39, 30 fps, on a 10-core Mac (median, capture to screen):

| live stream | local | over a good internet link | bandwidth per camera |
| --- | --- | --- | --- |
| JPEG (quality 75) | 40 ms | 60 ms | ~12 Mbit/s |
| H.264 | 408 ms | 423 ms | ~0.9 Mbit/s |

ffmpeg can get down to one frame of lag (`-threads 1` or slice threading), but
rerun hard-codes its decoder flags. If that changes, switch to H.264 everywhere.

A *stream* is an input, or one element of a `sync` tuple input: a tuple (an
Arrow struct) is unpacked into its elements, named `<input>/<field>`, each with
its own metadata (the tuple's `<field>.<key>` keys). Its metadata says what it
holds; other numeric streams are plotted. Every stream is logged at
the entity named like it (`arm_joint_state`, `observation/cam_high`):

| stream | must carry | recorded as |
| --- | --- | --- |
| `encoding` `rgb8` / `jpeg` | `uint8[N]` + `width`, `height` | file: `rr.VideoStream` (H.264); live: `rr.EncodedImage` (JPEG); in 3D when calibrated (below) |
| `encoding` `uint16` | `uint16[w*h]` + `width`, `height` | `rr.DepthImage` (millimetres) |
| listed in `MODE__JOINT_STATES` | a named vector (`names`) | `rr.Scalars`, and the 3D robot's joints (below) |
| anything else | a numeric array | `rr.Scalars`: with `names`, one entity per component, `<stream>/<name>`, its line labelled with the stream; without, one entity |

Every message is logged at its time by `MODE__INPUT_TIME`, on a timeline of
that name: dora's send `timestamp` by default, since every message has one, or
`capture_time` when the producers stamp it (then every message needs it).

A tuple is one aligned sample. Each element is logged at its own time, plus the
tuple's index on a timeline named after the input (`observation` = 0, 1, 2,
...), so a recording of observations is one row per observation and no
element's time is lost.

A stream missing a key its role
requires, or a non-numeric stream listed nowhere, stops the node with an error
naming it; a stream listed under two roles is a config error.

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
transforms, on the same timelines as its scalars, for the joints its `names` metadata lists, which must be the URDF's
joint names (a producer maps its own onto them: the sim through its parts'
`urdf_joints` / `urdf_scale`). A joint the URDF does not have is
skipped with one warning. Mimic joints, which a gripper reports only through its
driver, get `driver * multiplier + offset` as the URDF declares; the
transforms' parent and child frames are the URDF link names, the same the model
uses, so the chain connects.

**Calibrated cameras** (`MODE__CALIBRATION`): the cell's calibration file
gives every camera's `frame` (the URDF frame it is mounted on), `extrinsics`,
`intrinsics` and image size. An image whose `camera` metadata names one of them
is placed in the robot's frames on its first message, the way the
`rerun-data-model` skill models a camera. Depth images stay 2D. It needs `MODE__ROBOT_URDF`, since `frame` is one of its links.
`<stream>/calibration` gets a static `Transform3D` from `frame` to
`<stream>_optical_frame` (the extrinsics) and a `Pinhole` from there into
`<stream>_image_plane` (the intrinsics), and the stream's images sit in the
image plane: the viewer draws them inside the frustum, and a camera mounted on
the arm (`tool0`) moves with it. An image of another size than its calibration
is for stops the node. An image without a calibrated `camera` stays 2D.

**Layout**: the camera grid and the 3D view (the robot and every camera) on
top, and below either the plots of `MODE__PLOTS` (plot name -> its entities,
e.g. a joint's measured and commanded lines together) or one plot of every
numeric stream. Sent at start and again whenever a new camera appears.

To use your own layout, set `MODE__BLUEPRINT` to a Python file that defines
`blueprint`:

```python
import rerun.blueprint as rrb

blueprint = rrb.Blueprint(rrb.Spatial2DView(origin="/observation/cam_wrist_image"))
```

It is sent to every viewer and into the file, instead of the generated layout.
(A saved `.rbl` cannot be applied to a live viewer in rerun 0.39, so the
blueprint is code.)

Encoders are flushed and the file closed on stop.

### Inputs and outputs

| id              | source                  | meaning |
| --------------- | ----------------------- | ------- |
| `program_state` | `manager/program_state` | the node stops on `disconnect` |
| `<any id>`      | any stream              | recorded; give it a deep queue (`queue_size: 1000`), a recorder must not drop messages, not even while it starts |

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
| `MODE__INPUT_TIME` | `timestamp` | metadata key of the time messages are logged at: a datetime (dora's `timestamp`) or seconds since the epoch (`capture_time`, or your own) |
| `MODE__JOINT_STATES` | `[]` | JSON list of the joint-state streams, which also move the 3D robot |
| `MODE__PLOTS` | `{}` | JSON object, plot name -> the entities it shows: a whole stream, or one component `<stream>/<name>`, e.g. `'{"elbow": ["arm_joint_state/elbow_joint", "joint_target/elbow_joint"]}'` (measured next to commanded); empty: one plot of all |
| `MODE__ROBOT_URDF` | unset | robot URDF shown in 3D and moved by the `MODE__JOINT_STATES` streams, relative to the dataflow directory |
| `MODE__CALIBRATION` | unset | camera calibration file (YAML), relative to the dataflow directory, e.g. `assets/universal_robots/ur10e/calibration/tabletop.yaml` |
| `MODE__SCENE` | unset | a scene file (format: `assets/README.md`) giving `MODE__ROBOT_URDF` and `MODE__CALIBRATION` instead (its `urdf`, `calibration`); each comes one way, not both |
| `MODE__BLUEPRINT` | unset | Python file defining `blueprint`, sent instead of the generated layout |

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
      cam_high: { source: cam_high/cam_high_image, queue_size: 1000 }
    outputs: [node_state]
    env:
      MODE__NAME: "record"

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
