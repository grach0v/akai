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
src/main.py            # runs the modes (akai_node)
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
  SPS/PPS every 60 frames so playback can start at any of them).
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

A *stream* is an input, or one element of a `sync` tuple (`<input>/<field>`).
Its metadata says what it is:

| stream | recorded as |
| --- | --- |
| `encoding` `rgb8` / `jpeg` | file: H.264 video; live: JPEG; in 3D when calibrated |
| `encoding` `uint16` | depth image (millimetres) |
| listed in `MODE__JOINT_STATES` | plots, and the 3D robot's joints |
| `frame` (a pose, with `reference`) | axes in the 3D view |
| any other numeric array | plots, one line per `names` entry |

Messages are logged at `MODE__INPUT_TIME` (dora's `timestamp`, or
`capture_time`), and those with `step` also on the `step` timeline, so a
recording has one row per observation with what was sent for it. A stream
missing a required key stops the node.

**3D robot** (`MODE__URDF`, via `rerun.urdf.UrdfTree`). Joint states move
it by URDF joint name; mimic joints follow their driver. A session writes three
files:

| file | what |
| --- | --- |
| `<recording>.rrd` | the raw streams |
| `<recording>.fk.rrd` | the forward-kinematics transforms, a layer on the same recording |
| `<urdf name>.model.rrd` | the static model, written once and shared by every recording |

**Calibrated cameras** (`MODE__CALIBRATION`): an image whose `camera` is in the
calibration file is drawn in its frustum in the 3D view, moving with the arm
when mounted on it.

**Layout**: cameras and the 3D view on top, plots below (`MODE__PLOTS`, or one
plot of everything).

Encoders are flushed and the file closed on stop.

### Inputs and outputs

| id              | source                  | meaning |
| --------------- | ----------------------- | ------- |
| `program_state` | `manager/program_state` | the node stops on `disconnect` |
| `<any id>`      | any stream              | recorded; give it a deep queue (`queue_size: 1000`) |

Output `node_state`: `ready` once the sinks are up.

### Config

| var | default | meaning |
| --- | --- | --- |
| `APP_ID` | `akai` | rerun application id; also names the file |
| `MODE__NAME` | `record` | |
| `MODE__RRD_DIR` | `out/rrd` | where the `.rrd` files go (recording, FK layer, robot model), relative to the dataflow directory |
| `MODE__BIND_IP` | `0.0.0.0` | server bind address; `0.0.0.0` admits viewers from other machines |
| `MODE__GRPC_PORT` | `9876` | server port |
| `MODE__H264_PRESET` | `veryfast` | libx264 preset: `ultrafast` … `medium`, CPU vs file size |
| `MODE__LIVE_JPEG_QUALITY` | `80` | JPEG quality for the live stream when a camera sends rgb8; lower it for a remote viewer on a slow link |
| `MODE__INPUT_TIME` | `timestamp` | metadata key of the time messages are logged at (`timestamp` or `capture_time`) |
| `MODE__JOINT_STATES` | `[]` | JSON list of the joint-state streams, which also move the 3D robot |
| `MODE__PLOTS` | `{}` | JSON object, plot name -> its entities, e.g. `'{"elbow": ["arm_joint_state/elbow_joint", "joint_target/elbow_joint"]}'`; empty: one plot of all |
| `MODE__URDF` | unset | robot URDF shown in 3D and moved by the `MODE__JOINT_STATES` streams, relative to the dataflow directory |
| `MODE__CALIBRATION` | unset | camera calibration file (YAML), relative to the dataflow directory, e.g. `assets/cells/ur10e_tabletop/calibration.yaml` |
| `MODE__SCENE` | unset | a scene file giving `MODE__URDF` and `MODE__CALIBRATION` |

## `visualize` mode

Waits up to 15 s until the record server accepts connections (a viewer does not
retry a failed connection), then starts the viewer bundled with rerun-sdk on the server
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
