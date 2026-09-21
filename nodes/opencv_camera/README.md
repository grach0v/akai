# opencv_camera

Publishes frames from one camera device, opened through OpenCV, as
`<camera_name>_image`.

A background thread owns the camera: it reads frames as the device delivers
them, encodes each one and keeps only the latest. The dora (main) thread
publishes that latest frame on every `tick` from a dora timer, once per
captured frame: the slot is emptied when taken, so a tick faster than the
camera never duplicates a frame, it only shortens how long a frame waits
(10 ms tick: at most 10 ms). The camera is opened before the node joins the
dataflow, so no tick is queued while it opens.

## Layout

```
pyproject.toml     # uv project: dora-rs + opencv-python (+ pyarrow, pydantic-settings)
src/config.py      # CameraConfig: node fields + `mode`, one config class per mode picked by MODE__NAME
src/main.py        # dora skeleton, identical in every node: config -> mode -> Node -> loop
src/modes/camera.py  # `camera` mode: CameraReader thread keeps the latest frame, CameraMode publishes on tick
```

`src/main.py` is a plain script, run by this project's own interpreter (dora runs
a bare `.py` path with the *system* python, so a dataflow points at the venv
explicitly, see below).

## Inputs

| id              | source                   | meaning |
| --------------- | ------------------------ | ------- |
| `tick`          | `dora/timer/millis/<N>`  | publishes the latest frame, if new; N below the frame period (10 for a 30 fps camera) keeps the publish delay small. Wire it `queue_size: 1, queue_policy: drop_oldest` |
| `program_state` | `manager/program_state`  | optional; the node stops on `disconnect` |

Any other input id raises: an unexpected input on a producer is a wiring bug.

## Outputs

| id                         | payload              | metadata |
| -------------------------- | -------------------- | -------- |
| `<camera_name>_image`      | `uint8[N]`           | `encoding` (`rgb8` \| `jpeg`), `width`, `height`, `capture_time` |
| `<camera_name>_node_state` | `utf8[1]` state token | none |

- `rgb8`: the raveled H×W×3 RGB frame, `N = width * height * 3`.
- `jpeg`: JPEG bytes; `width`/`height` still describe the decoded frame.
- `capture_time` is wall-clock seconds (`time.time()`) at capture, for aligning
  streams downstream. Not `timestamp`: dora owns that key.
- `node_state` is edge-triggered: `ready` once after the capture opens, then
  `finished` when the device stops delivering frames. The node then waits for `program_state == disconnect` (the manager's response to
  `finished`) or dora's STOP before exiting.

`rgb8` at 30 FPS is heavy (27 MB/s at 640×480). Consumers that may fall behind
should wire the image input with `queue_size: 1, queue_policy: drop_oldest`;
recorders keep the default queue.

## Config

Set via the dataflow's `env:` block. `MODE__NAME` selects the mode and with it
the config class whose fields are set as `MODE__<FIELD>` (or all at once as one
JSON object in `MODE`). Any `MODE__*` variable requires `MODE__NAME`, and a
variable that belongs to another mode is rejected. Every field is also a CLI
flag (`--mode.fps 15`), which overrides the env, for dora modules whose `params:` reach a node only through
`args:`.

| var            | default  | meaning |
| -------------- | -------- | ------- |
| `CAMERA_NAME`  | required | stream name; prefixes both outputs |
| `MODE__NAME` | `camera` | which mode runs |

`camera` mode config:

| var                         | default | meaning |
| --------------------------- | ------- | ------- |
| `MODE__SOURCE`       | `0`     | device index, or on Linux a device path (`/dev/video0`, `/dev/v4l/by-id/...`) |
| `MODE__WIDTH`        | `640`   | requested capture width (the device may pick another) |
| `MODE__HEIGHT`       | `480`   | requested capture height |
| `MODE__FPS`          | `30`    | capture rate requested from the device |
| `MODE__FOURCC`       | unset   | optional capture FourCC, e.g. `MJPG`; many USB cameras need it for high FPS at high resolution |
| `MODE__ENCODING`     | `rgb8`  | `rgb8` or `jpeg` |
| `MODE__JPEG_QUALITY` | `90`    | 1–100, for `jpeg` |

The real negotiated size and FPS are logged at startup; the frame's real size is
always in the message metadata.

TODO(perf): `jpeg` re-encodes what the camera already sent as MJPEG. On Linux
(V4L2) the compressed frame could be passed through untouched.

## Dataflow snippet

```yaml
nodes:
  - id: cam_high
    build: uv sync --project ../../../nodes/opencv_camera
    path: ../../../nodes/opencv_camera/.venv/bin/python
    args: ../../../nodes/opencv_camera/src/main.py
    inputs:
      tick: { source: dora/timer/millis/10, queue_size: 1, queue_policy: drop_oldest }
      program_state: manager/program_state
    outputs: [cam_high_image, cam_high_node_state]
    env:
      CAMERA_NAME: "cam_high"
      MODE__NAME: "camera"
      MODE__SOURCE: "0"
      MODE__WIDTH: "640"
      MODE__HEIGHT: "480"
      MODE__FPS: "30"
      MODE__ENCODING: "jpeg"
      MODE__JPEG_QUALITY: "80"
```

## Development

```sh
cd nodes/opencv_camera
uv sync
uv run ruff check src && uv run ruff format --check src   # ruff from the repo root venv also works
```

On macOS the first camera access from a new interpreter triggers the system
camera-permission grant and that first open may fail; run again after granting.
