# web_controller

Serves a web page to drive a robot and follow the program. The page talks to the
node over a WebSocket; the node publishes what the page asks for into the
dataflow. No build step: the page is one HTML file with plain JavaScript.

## Layout

```
pyproject.toml         # uv project: dora-rs, fastapi, uvicorn[standard], pydantic-settings
src/config.py          # WebControllerConfig: node fields + `mode`, one config class per mode picked by MODE__NAME
src/main.py            # dora skeleton, identical in every node: config -> mode -> Node -> loop
src/server.py          # uvicorn on a background thread, shared by every mode
src/modes/joint.py     # `joint`: drive every part in joint space
src/web/joint.html     # the `joint` page
```

The web server runs on its own thread with its own asyncio loop, started before
the node joins the dataflow (a taken port fails right away). The dora loop and
the page share the mode's state under a lock; only the dora loop sends outputs.

## `joint` mode

Drive every robot part in joint space. The node answers every `observation`, a
tuple (`akai_tuple`): each of its named vectors (`names` in their metadata) is a
part's joint state, a card on the page labelled by the element's name, with a
slider per joint bounded by that joint's limits in the robot's URDF (`urdf`; a
joint without limits there raises). Every part's target goes out together, as one
named vector `joint_target` (all parts' joint names in `names`) carrying the
observation's `step`, so each target is the answer to one observation and a
recording puts it on that observation's row.

- A part's target is seeded from its first measured state, so nothing jumps.
- Targets are clamped to the joint limits.
- A target is published for every observation, but only while the program is `running`.
- Every program-state change resets the targets to the measured state, so a stale
  target never drags the robot back.
- The page's Disconnect button reports `disconnect` on `node_state`, which ends
  the program in the manager.

The page shows one card per part with a row per joint: measured value, a target
slider over the joint range, −/+ buttons (`step` per click), and the target.
"Sync to measured" sets every target to the measured position.

### Inputs

| id                   | source                  | meaning |
| -------------------- | ----------------------- | ------- |
| `observation`        | `sync` (`align`)        | a tuple: its named vectors are the measured joints; answered with a target; `queue_size: 1, queue_policy: drop_oldest` |
| `program_state`      | `manager/program_state` | shown on the page; targets go out only while `running`; the node stops on `disconnect` |

### Outputs

| id                    | payload      | metadata |
| --------------------- | ------------ | -------- |
| `joint_target` | `float64[Σn]`, every part's targets | `names` (every part's joint names, in the order the parts appeared), `step` (the observation's), `capture_time` |
| `node_state`          | `utf8[1]`    | `ready` once serving, `disconnect` from the page's button |

### WebSocket protocol (`/ws`)

The server pushes, at `push_hz`:
`{"program_state", "step", "parts": {part: {"names", "lower", "upper", "measured", "target"}}}`.
The page sends `{"type": "set", "part", "index", "value"}`,
`{"type": "nudge", "part", "index", "dir": ±1}`, `{"type": "sync"}` or
`{"type": "disconnect"}`.

### Config

| var | default | meaning |
| --- | --- | --- |
| `HOST` | `127.0.0.1` | bind address; `0.0.0.0` serves other machines too |
| `PORT` | `8000` | |
| `SERVER_TIMEOUT` | `5` | seconds for the server to come up |
| `MODE__NAME` | required | `joint` |
| `MODE__URDF` | unset | the robot's URDF, relative to the dataflow directory: its joint limits bound the sliders |
| `MODE__SCENE` | unset | a scene file giving `MODE__URDF` instead (its `urdf`); one of the two is required |
| `MODE__STEP` | `0.02` | one −/+ click, in the joint's unit |
| `MODE__PUSH_HZ` | `15` | page refresh rate |

Planned: a `cartesian` mode (end-effector targets, needs the pink node) and
an `episode` mode (task + start/finish for recording).

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
      MODE__URDF: "../../../assets/cells/ur10e_tabletop/ur10e_tabletop.urdf"
```
