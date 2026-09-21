# manager

Owns the **program lifecycle** of a dataflow. Every other node reports its
logical state as an edge-triggered `node_state` token; the manager runs a state
machine over those events and broadcasts `program_state` whenever the program
moves to a new state. Nodes gate their behaviour on `program_state` and tear
down on `disconnect`.

No heartbeat, no liveness polling: dora already detects node death.

## Layout

```
pyproject.toml     # uv project: dora-rs, python-statemachine, pydantic-settings
src/config.py      # ManagerConfig: node fields + `mode`, one config class per mode picked by MODE__NAME
src/main.py        # dora skeleton, identical in every node: config -> mode -> Node -> loop
src/modes/simple.py  # `simple` mode: boot -> running -> disconnect
```

## Modes

### `simple` (default)

States: `boot` (initial), `running`, `disconnect` (final).

- `boot` → `running` once every id in `PRODUCERS` has reported `ready`.
  Readiness is a latch per producer.
- Any state → `disconnect` when any input reports `finished` (it has nothing
  more to produce, e.g. a camera's device stopped delivering) or `disconnect`
  (an operator request).
- After `disconnect` the manager exits; with every other node tearing down on
  `disconnect`, the dataflow finishes.

## Inputs

| id          | source                        | meaning |
| ----------- | ----------------------------- | ------- |
| `<any id>`  | `<node>/<name>_node_state`    | one input per watched node; the payload is a single utf8 token |

Input ids listed in `PRODUCERS` must report `ready` before the program leaves `boot`.

## Outputs

| id              | payload   | meaning |
| --------------- | --------- | ------- |
| `program_state` | `utf8[1]` | `boot` / `running` / `disconnect`, sent on every state entry |

## Config

Set via the dataflow's `env:` block. `MODE__NAME` selects the mode and with it
the config class whose fields are set as `MODE__<FIELD>` (or all at once as one
JSON object in `MODE`). Any `MODE__*` variable requires `MODE__NAME`, and a
variable that belongs to another mode is rejected. Every field is also a CLI
flag (`--mode.producers '["a"]'`), which overrides the env.

| var    | default  | meaning |
| ------ | -------- | ------- |
| `MODE__NAME` | `simple` | which program lifecycle runs |

`simple` mode config:

| var                      | default | meaning |
| ------------------------ | ------- | ------- |
| `MODE__PRODUCERS` | `[]`    | JSON list of input ids that must report `ready` to leave `boot`, e.g. `'["cam_high"]'`; empty means leave `boot` on the first event |

## Dataflow snippet

```yaml
  - id: manager
    build: uv sync --project ../../../nodes/manager
    path: ../../../nodes/manager/.venv/bin/python
    args: ../../../nodes/manager/src/main.py
    inputs:
      cam_high: cam_high/cam_high_node_state
    outputs: [program_state]
    env:
      MODE__NAME: "simple"
      MODE__PRODUCERS: '["cam_high"]'
```
