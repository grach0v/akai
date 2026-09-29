# sync

Joins several streams into one message.
Dora has no built-in join, and a consumer that needs a whole-robot snapshot or
command (every part at once) would otherwise have to wire, queue and match one
input per part.

It assumes nothing about what the inputs carry: it bundles one message from
each input into a *tuple*, whatever those messages are (images, joint vectors,
anything), so a consumer that needs them together, such as a robot taking all
its parts' targets at once, or a policy or recorder taking an observation of
cameras and state, gets them in one message.

## Layout

```
pyproject.toml         # uv project: dora-rs, pyarrow, pydantic-settings; `examples` group: node-hub nodes
src/config.py          # SyncConfig: `mode`, one config class per mode picked by MODE__NAME
src/main.py            # dora skeleton, identical in every node: config -> mode -> Node -> loop
src/bundler.py         # when a tuple goes out, and the repeat and skew checks: the rules below
src/modes/tuple.py     # `tuple`: one message holding a new message from every input
examples/              # dataflows running sync with dora's timers and node-hub nodes only
```

## When a tuple goes out

The node keeps the latest message of each input and follows these rules:

1. When a message arrives, it replaces that input's previous message. Messages
   are not queued: the newest wins.
2. If every input has sent a new message since the last tuple, the node emits a
   tuple of them right away. It uses no timer, so a tuple waits only for the
   slowest producer.
3. If some input has not sent a new message yet, the node waits. It never
   resends an old message and never emits a partial tuple, so nothing goes out
   until every input has sent at least once.
4. If a producer stops sending, tuples stop too. This is on purpose: resending
   its last message would tell the consumer (a robot, the sim) that a dead part
   is still alive and where it was.

Two conditions are handled as configured, each with `ignore`, `warn` or
`error` (stop the node with an error):

1. **Repeat** (`on_repeat`, default `ignore`): an input sends again before its
   tuple is complete, so its previous message is dropped (rule 1). Normal when
   a fast producer waits on a slow one; a sign of trouble when the inputs
   should arrive in lockstep. A warning is logged for every repeated message.
2. **Skew** (`on_skew`, default `warn`, checked only with `max_skew` set): the
   messages of a tuple are further apart in time than `max_skew`, so one
   producer lags the others. Time is measured by `skew_time`: every input's
   `capture_time` (the default; an input without one is an error), or dora's
   send `timestamp`, which every message has. A warning is logged once when the skew appears,
   not for every tuple while it lasts.

## `tuple` mode

A tuple keeps values and metadata apart, as every dora message does:

1. **Value:** a one-row Arrow struct with one field per input, named by its input
   id, in `inputs` order, holding that input's array.
2. **Metadata:** every key of every input's metadata as `<input id>.<key>`,
   dora's send `timestamp` of each message included, plus the tuple's own
   `capture_time`: the oldest of the inputs' `capture_time`s, when every input
   carries one.

For example, a camera and a joint state:

| | content |
| --- | --- |
| value | `struct<cam_high: list<uint8>, arm_joint_state: list<double>>`, one row |
| metadata | `cam_high.encoding`, `cam_high.width`, `cam_high.height`, `cam_high.capture_time`, `cam_high.timestamp`, `arm_joint_state.names`, `arm_joint_state.capture_time`, `arm_joint_state.timestamp`, `capture_time` |

A consumer reads an input's array as `value.field("cam_high")[0].values` and
its metadata as `metadata["cam_high.encoding"]`. Nothing is converted or
dropped, and every metadata value keeps its type. Dora's metadata is flat (a
nested dict would arrive as a string), hence one key per input and key rather
than one dict per input; an input id may therefore not contain a `.`.

| id | payload | metadata |
| --- | --- | --- |
| input: each of `inputs` (no `.` in the id) | any message | any |
| input: `program_state` | `utf8[1]` | the node stops on `disconnect` |
| output: `<output>` | `struct<input id: list<its type>, ...>`, one row | `<input id>.<key>` for every input's metadata; `capture_time` (the oldest input's), when every input has one |
| output: `node_state` | `utf8[1]` | `ready` |

| var | default | meaning |
| --- | --- | --- |
| `MODE__NAME` | `tuple` | |
| `MODE__INPUTS` | required | JSON list of the input ids, in field order |
| `MODE__OUTPUT` | required | output id of the tuple |
| `MODE__ON_REPEAT` | `ignore` | `ignore`, `warn` or `error` when an input sends again before its tuple is complete |
| `MODE__MAX_SKEW` | unset | the most the messages of a tuple may be apart in time, in seconds; unset: not checked |
| `MODE__ON_SKEW` | `warn` | `ignore`, `warn` or `error` when a tuple is over `MODE__MAX_SKEW` |
| `MODE__SKEW_TIME` | `capture_time` | what skew is measured by: `capture_time` (required on every input) or `timestamp` (dora's send time) |

## Examples

Two dataflows in `examples/` run the node with nothing of ours but sync: dora's
built-in timers and nodes from the dora node hub (`pyarrow-sender`,
`pyarrow-assert`: the `examples` dependency group of this node's `pyproject.toml`,
installed into its environment by the examples' build step).

1. `tuple_test.yml` is a test. Two `pyarrow-sender`s send one fixed array each,
   sync bundles them, and `pyarrow-assert` checks the tuple equals the expected
   one: the run fails (exit 1) if it does not, and ends by itself otherwise.
2. `tuple_timers.yml` shows the rules at work. A 100 ms and a 330 ms timer feed
   sync, which emits one tuple per slow tick: the fast timer's extra ticks are
   logged as repeats, and the drifting rates put some tuples over `max_skew`,
   logged as skew. `pyarrow-assert` checks every tuple. Stop it with Ctrl-C.

```sh
uv run dora build nodes/sync/examples/tuple_test.yml
uv run dora run   nodes/sync/examples/tuple_test.yml
uv run dora build nodes/sync/examples/tuple_timers.yml
uv run dora run   nodes/sync/examples/tuple_timers.yml
```

## Dataflow snippets

The sim's joint targets from the web controller's per-part targets, as one
message (the sim takes a tuple of named vectors):

```yaml
  - id: robot_target
    build: uv sync --project ../../../nodes/sync
    path: ../../../nodes/sync/.venv/bin/python
    args: ../../../nodes/sync/src/main.py
    inputs:
      program_state: manager/program_state
      arm_joint_target: { source: web/arm_joint_target, queue_size: 1, queue_policy: drop_oldest }
      gripper_joint_target: { source: web/gripper_joint_target, queue_size: 1, queue_policy: drop_oldest }
    outputs: [joint_target, node_state]
    env:
      MODE__NAME: "tuple"
      MODE__INPUTS: '["arm_joint_target", "gripper_joint_target"]'
      MODE__OUTPUT: "joint_target"
      MODE__MAX_SKEW: "0.05"
```

An observation of cameras and robot state for a policy or a recorder:

```yaml
    inputs:
      cam_high: sim/cam_overhead_image
      cam_wrist: sim/cam_wrist_image
      arm_joint_state: sim/arm_joint_state
      gripper_joint_state: sim/gripper_joint_state
    outputs: [observation, node_state]
    env:
      MODE__NAME: "tuple"
      MODE__INPUTS: '["cam_high", "cam_wrist", "arm_joint_state", "gripper_joint_state"]'
      MODE__OUTPUT: "observation"
```
