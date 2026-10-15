# mujoco

Simulates a robot cell in [MuJoCo](https://mujoco.org) in real time, with the
contract a real robot node has, so the sim and the hardware are interchangeable
in a dataflow: every controllable *part* (an arm, a gripper, ...) is reported as
`<part>_joint_state` in the robot URDF's joint names and units, the sim is driven
by joint targets in the same names, and the MJCF's cameras are rendered and
published as `<camera>_image`, plus `<camera>_depth` for the RGB-D ones with `MODE__DEPTH`.

```
src/config.py      # MujocoConfig: the scene (MODEL + PARTS, or SCENE) + `mode`, picked by MODE__NAME
src/main.py        # dora skeleton, identical in every node: config -> mode -> Node -> loop
src/modes/sim.py   # `sim`: physics thread, render thread, capture on tick
```

## The scene

The sim needs the MJCF (`MODEL`) and its parts (`PARTS`): the MJCF actuators of
each part. Every actuator is named after the URDF joint it drives, in that
joint's units, so joint states (the actuators' lengths) and targets (their ctrl)
need no conversion. Both can come from a scene file's `mujoco` section instead (`SCENE`), not
both ways. The format, and how to make the files, are in
[`assets/README.md`](../../assets/README.md). Unknown keys and names the model
does not have fail at startup.

The MJCF needs a `home` keyframe: the sim starts there.

## `sim` mode

The sim captures on the tick, the way triggered cameras and a robot would:
everything a tick publishes shows one instant. Three threads:

1. **physics** steps the model paced to the wall clock (`realtime_factor`).
   When it falls more than `max_lag` behind, it drops the backlog instead of
   racing to catch up.
2. **render** (only with cameras) owns the offscreen renderer and renders every
   camera from a snapshot of the simulation, on request.
3. **dora** (the main thread) writes every `joint_target` straight to its
   joints' actuators, clipped to their control range, so physics applies it from
   its next step: a new target replaces the old one, nothing queues. It never
   waits for a render. On every `tick`:
   1. if the previous capture has been rendered, it sends it: every part's joint
      positions and every camera's frame, stamped with its `capture_time`;
   2. if the render still runs, this tick captures nothing (a warning);
      otherwise it copies the simulation into the snapshot (microseconds, under
      the lock physics steps under) and hands it to the render thread.

So a capture goes out one tick after it was taken: align it by `capture_time`
(a `sync` in `align` mode with `input_time: capture_time`), not by when it was
sent. Without cameras, a tick captures and sends at once. If a background thread
dies, its traceback is logged and the node reports `finished`.

### Inputs

| id | source | meaning |
| --- | --- | --- |
| `tick` | `dora/timer/millis/<N>` | capture: the observation rate (33 for 30 Hz); `queue_size: 1, queue_policy: drop_oldest` |
| `program_state` | `manager/program_state` | the node stops on `disconnect` |
| `joint_target` | a controller | a named vector (`float64[n]` + `names`, URDF names and units): each value drives the joint it names, so one message may cover one part or the whole robot; a name the sim has no actuator for raises. `queue_size: 1, queue_policy: drop_oldest` |

Any other input id raises: an unexpected input on a producer is a wiring bug.

### Outputs

| id | payload | metadata |
| --- | --- | --- |
| `<part>_joint_state` | `float64[n]` joint positions (URDF units) | `names` (URDF joint names), `capture_time` |
| `<camera>_image` | `uint8[N]` | `camera`, `encoding` (`rgb8` \| `jpeg`), `width`, `height`, `capture_time` |
| `<camera>_depth` | `uint16[w*h]` millimetres, with `MODE__DEPTH`, for RGB-D cameras (`depth: true` in the calibration) | `camera`, `encoding` (`uint16`), `width`, `height`, `capture_time` |
| `node_state` | `utf8[1]` | `ready` once running, `finished` if a background thread died |

### Config

| var | default | meaning |
| --- | --- | --- |
| `MODEL` | unset | the MJCF, relative to the dataflow directory |
| `PARTS` | unset | JSON object, part name -> its joints (format: `assets/README.md`), e.g. `'{"arm": ["shoulder_pan_joint", ...], "gripper": ["finger_joint"]}'` |
| `SCENE` | unset | a scene file giving `MODEL` and `PARTS` instead (its `mujoco` section); one of the two ways is required |
| `CALIBRATION` | unset | camera calibration file (or the scene's): its RGB-D cameras also render depth |
| `MODE__NAME` | `sim` | |
| `MODE__CAMERAS` | `[]` | JSON list of MJCF cameras to publish; empty: no renderer |
| `MODE__DEPTH` | `false` | also publish `<camera>_depth` for the RGB-D cameras (`depth: true` in the calibration) |
| `MODE__WIDTH` / `MODE__HEIGHT` | `640` / `480` | image size, at most the model's offscreen buffer |
| `MODE__RENDER_FLAGS` | `{}` | JSON object of MuJoCo render flags by `mjtRndFlag` name without `mjRND_` (`SHADOW`, `REFLECTION`, `SKYBOX`, `FOG`, `HAZE`, ...); unset ones keep MuJoCo's defaults |
| `MODE__ENCODING` | `rgb8` | `rgb8` or `jpeg` |
| `MODE__JPEG_QUALITY` | `90` | 1–100, for `jpeg` |
| `MODE__REALTIME_FACTOR` | `1.0` | simulated seconds per wall-clock second |
| `MODE__MAX_LAG` | `0.1` | seconds behind the wall clock before the backlog is dropped |

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
      SCENE: "../../../assets/universal_robots/ur10e/scenes/tabletop.yaml"
      MODE__CAMERAS: '["cam_overhead", "cam_wrist"]'
      MODE__ENCODING: "jpeg"
```
