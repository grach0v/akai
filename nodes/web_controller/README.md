# web_controller

Serves a web page to drive a robot. The page talks to the node over a
WebSocket; the node answers every observation with the page's target. The page
is one HTML file with plain JavaScript.

```
src/config.py          # WebControllerConfig: node fields + `mode`, picked by MODE__NAME
src/main.py            # dora skeleton, identical in every node
src/server.py          # uvicorn on a background thread
src/modes/joint.py     # `joint`: drive every part in joint space
src/modes/pose.py      # `pose`: drive the TCP pose, the rest in joint space
src/web/index.html     # the page
```

## `joint` mode

The node answers every `observation` tuple: each named vector in it is a part,
shown as a card with a slider per joint (bounded by the URDF's limits). All
parts' targets go out as one `joint_target` carrying the observation's `step`.

- Targets start at the measured state and reset to it on every program-state
  change, so nothing jumps.
- Targets go out only while the program is `running`.
- Disconnect on the page ends the program.

| input | source | meaning |
| --- | --- | --- |
| `observation` | `sync` (`align`) | a tuple of joint states; `queue_size: 1, queue_policy: drop_oldest` |
| `program_state` | `manager/program_state` | the node stops on `disconnect` |

| output | payload | metadata |
| --- | --- | --- |
| `joint_target` | `float64[n]`, every part's targets | `names`, `step`, `capture_time` |
| `node_state` | `utf8[1]` | `ready`, `disconnect` from the page |

| var | default | meaning |
| --- | --- | --- |
| `HOST` | `127.0.0.1` | bind address; `0.0.0.0` serves other machines too |
| `PORT` | `8000` | |
| `MODE__NAME` | required | `joint` or `pose` |
| `MODE__URDF` / `MODE__SCENE` | unset | the robot's URDF, or a scene file giving it |
| `MODE__STEP` | `0.02` | one −/+ click, in the joint's unit |
| `MODE__PUSH_HZ` | `15` | page refresh rate |

## `pose` mode

Like `joint`, but the arm (the joints moving `frame`) gets jog buttons instead
of sliders. While a button is held, every `tick` (the teleop clock) moves the
target pose along the base's axes or about the tool's. Each observation is
answered with a `pose_target` (the pose of `frame` in `reference`) and a
`joint_target` for the other joints; pink turns them into a command.

The target starts at the measured pose (forward kinematics of the measured
joints) and stays within `max_lead` of it, so jogging past reach does not wind
up.

| extra input / output | meaning |
| --- | --- |
| `tick` | the teleop clock, a dora timer; `queue_size: 1, queue_policy: drop_oldest` |
| `pose_target` | `float64[7]` `[x, y, z, qx, qy, qz, qw]`, with `frame`, `reference`, `step`, `capture_time` |

| var | default | meaning |
| --- | --- | --- |
| `MODE__FRAME` | `tool0` | the URDF frame the pose drives (`tcp` for the TCP) |
| `MODE__REFERENCE` | `base_link` | the URDF frame the pose is given in |
| `MODE__LINEAR_SPEED` / `MODE__ANGULAR_SPEED` | `0.25` / `1.0` | jog speeds, m/s and rad/s |
| `MODE__MAX_LEAD` / `MODE__MAX_LEAD_ANGLE` | `0.1` / `0.5` | how far the target may lead the measured pose, m and rad |

## Dataflow snippet

```yaml
  - id: web
    build: uv sync --project ../../../nodes/web_controller
    path: ../../../nodes/web_controller/.venv/bin/python
    args: ../../../nodes/web_controller/src/main.py
    inputs:
      program_state: manager/program_state
      observation: { source: observation/observation, queue_size: 1, queue_policy: drop_oldest }
    outputs: [joint_target, node_state]
    env:
      PORT: "8000"
      MODE__NAME: "joint"
      MODE__SCENE: "../../../assets/cells/ur10e_tabletop/scene.yaml"
```
