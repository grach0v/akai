# mujoco

Simulates a robot cell in [MuJoCo](https://mujoco.org) with the contract a real
robot node has: each *part* (arm, gripper) goes out as `<part>_joint_state` in
the URDF's joint names and units, `joint_target` drives it in the same names,
and the MJCF's cameras go out as `<camera>_image` (and `<camera>_depth`).

```
src/config.py      # MujocoConfig: the scene (MODEL + PARTS, or SCENE) + `mode`
src/main.py        # dora skeleton, identical in every node
src/modes/sim.py   # `sim`: real time
src/modes/step.py  # `step`: one fixed period per tick
```

The MJCF and its parts can come from a scene file (`SCENE`); the format is in
[`assets/README.md`](../../assets/README.md). The MJCF needs a `home` keyframe,
where the sim starts.

## `sim` mode

Real time, as a real robot. Physics runs on its own thread, paced to the wall
clock. A `joint_target` is written to the actuators as it arrives. On each tick
the node copies the simulation, waits for the cameras to render it (~7 ms for
two 640×480 JPEG cameras, ~15 ms with depth), and sends joint states and images
of that instant, stamped with `capture_time`.

## `step` mode

One fixed `period` of simulated time per tick. The first `joint_target` after a
send starts the next period on a background thread: physics steps one period,
then the cameras render. The tick sends what is ready at once. With no command
(nothing driving the robot yet), the tick steps on the last targets itself.

So the observation goes out right on the tick, but a command acts from the start
of its period, which no real robot does. Use `sim` to see a slow controller or
policy cost what it would on hardware.

### Inputs

| id | source | meaning |
| --- | --- | --- |
| `tick` | `dora/timer/millis/<N>` | the observation rate; `queue_size: 1, queue_policy: drop_oldest` |
| `program_state` | `manager/program_state` | the node stops on `disconnect` |
| `joint_target` | a controller | a named vector of any joints; `queue_size: 1, queue_policy: drop_oldest` |

### Outputs

| id | payload | metadata |
| --- | --- | --- |
| `<part>_joint_state` | `float64[n]` joint positions (URDF units) | `names` (URDF joint names), `capture_time` |
| `<camera>_image` | `uint8[N]` | `camera`, `encoding` (`rgb8` \| `jpeg`), `width`, `height`, `capture_time` |
| `<camera>_depth` | `uint16[w*h]` millimetres, RGB-D cameras with `MODE__DEPTH` | `camera`, `encoding` (`uint16`), `width`, `height`, `capture_time` |
| `node_state` | `utf8[1]` | `ready` once running, `finished` if a background thread died |

### Config

| var | default | meaning |
| --- | --- | --- |
| `MODEL` | unset | the MJCF, relative to the dataflow directory |
| `PARTS` | unset | JSON object, part name -> its joints |
| `SCENE` | unset | a scene file giving `MODEL`, `PARTS` and `CALIBRATION` |
| `CALIBRATION` | unset | camera calibration file (or the scene's): its RGB-D cameras also render depth |
| `MODE__NAME` | `sim` | `sim` (real time) or `step` (one period per tick) |
| `MODE__CAMERAS` | `[]` | JSON list of MJCF cameras to publish; empty: no renderer |
| `MODE__DEPTH` | `false` | also publish depth for the RGB-D cameras |
| `MODE__WIDTH` / `MODE__HEIGHT` | `640` / `480` | image size, at most the model's offscreen buffer |
| `MODE__RENDER_FLAGS` | `{}` | JSON object of MuJoCo render flags without `mjRND_`, e.g. `{"SHADOW": false}` |
| `MODE__ENCODING` | `rgb8` | `rgb8` or `jpeg` |
| `MODE__JPEG_QUALITY` | `90` | 1–100, for `jpeg` |
| `MODE__REALTIME_FACTOR` | `1.0` | `sim`: simulated seconds per wall-clock second |
| `MODE__MAX_LAG` | `0.1` | `sim`: seconds behind the wall clock before the backlog is dropped |
| `MODE__PERIOD` | `0.033` | `step`: simulated seconds per tick, the tick's interval |

Headless Linux needs an offscreen GL: set `MUJOCO_GL: "egl"` (or `osmesa`) in
the node's `env:`. macOS needs nothing.

## Dataflow snippet

```yaml
  - id: sim
    build: uv sync --project ../../../nodes/mujoco
    path: ../../../nodes/mujoco/.venv/bin/python
    args: ../../../nodes/mujoco/src/main.py
    inputs:
      tick: { source: dora/timer/millis/33, queue_size: 1, queue_policy: drop_oldest }
      program_state: manager/program_state
      joint_target: { source: web/joint_target, queue_size: 1, queue_policy: drop_oldest }
    outputs: [arm_joint_state, gripper_joint_state, cam_overhead_image, cam_wrist_image, node_state]
    env:
      SCENE: "../../../assets/cells/ur10e_tabletop/scene.yaml"
      MODE__CAMERAS: '["cam_overhead", "cam_wrist"]'
      MODE__ENCODING: "jpeg"
```
