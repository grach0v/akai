# sync

Joins several streams into one message, a *tuple* (dora has no built-in join).
Two modes decide which messages go together:

1. **`align`**: every input's sample nearest a reference time: observations.
2. **`step`**: the messages with the same `step`: an observation and its answer.

```
src/config.py          # SyncConfig: `mode`, picked by MODE__NAME
src/main.py            # runs the modes (akai_node)
src/modes/align.py     # `align` mode
src/modes/step.py      # `step` mode
examples/              # test dataflows: sync with dora's timers and node-hub nodes only
```

## The tuple output type

1. **Value:** a one-row Arrow struct, a field per input named by its id, e.g.
   `struct<cam_high: list<uint8>, arm_joint_state: list<double>>`. Read it with
   `akai_tuple.elements(value, metadata)` ([`libs/akai_tuple`](../../libs/akai_tuple)).
2. **Metadata:** every input's keys as `<input id>.<key>`, plus the tuple's own
   `capture_time` and `step` (its index, copied by every answer to it).
3. **Tuples join flat:** an input that is a tuple gives its elements, not one
   nested field. Element names must not repeat.

Inputs: every id in `inputs`, and `program_state` (the node stops on
`disconnect`). Outputs: `<output>` (the tuple) and `node_state` (`ready`).

Every `*_TIME` setting names a metadata key holding a time: dora's `timestamp`
or seconds since the epoch, like `capture_time`.

## `align` mode

1. Every input keeps its samples of the last second. A sample's time is dora's
   send `timestamp`, or its `capture_time` (`input_time`).
2. Every message on the `reference` input adds a reference time t (its
   `timestamp` or `capture_time`, by `reference_time`).
3. t is settled once every input has a sample at or after it, so the nearest is
   known, or 0.2 s after t (then the nearest so far).
   Reference times settle in order.
4. A settled t becomes a tuple of every input's sample nearest t, with
   `capture_time` = t. Skew is each element's distance from t; every tuple over
   `max_skew` is reported as `on_skew` says, naming its elements.
5. Until every input has sent once, reference times are dropped.

Aligned by `capture_time`, a late camera frame still lines up with the joint
states of its instant. Give the inputs deep queues (`queue_size: 100`): the node
chooses among samples.

| var | default | meaning |
| --- | --- | --- |
| `MODE__NAME` | required | `align` |
| `MODE__REFERENCE` | required | the input giving the reference times: a timer, or the robot's joint state; may be one of `MODE__INPUTS` |
| `MODE__REFERENCE_TIME` | `timestamp` | metadata key of a reference message's time (a timer only has `timestamp`) |
| `MODE__INPUTS` | required | JSON list of the input ids to align, in field order |
| `MODE__INPUT_TIME` | `timestamp` | metadata key of an input message's time (required on every message) |
| `MODE__OUTPUT` | required | output id of the tuple |
| `MODE__MAX_SKEW` | unset | seconds an element may be off its reference time; unset: not checked |
| `MODE__ON_SKEW` | `warn` | `warn` (per tuple) or `error` (stop the node) |

For example, an observation aligned to the observation clock the sim ticks on:

```yaml
    inputs:
      tick: dora/timer/millis/33
      cam_high: { source: sim/cam_overhead_image, queue_size: 100 }
      arm_joint_state: { source: sim/arm_joint_state, queue_size: 100 }
    outputs: [observation, node_state]
    env:
      MODE__NAME: "align"
      MODE__REFERENCE: "tick"
      MODE__INPUTS: '["cam_high", "arm_joint_state"]'
      MODE__INPUT_TIME: "capture_time"
      MODE__OUTPUT: "observation"
      MODE__MAX_SKEW: "0.02"
```

## `step` mode

1. Every message needs `step`.
2. Once every input has sent the same step, their tuple goes out at once.
3. Incomplete steps are dropped with a warning when a newer one completes; at
   most 30 wait.
4. `elements` picks which elements go on, so a consumer gets only what it uses.

| var | default | meaning |
| --- | --- | --- |
| `MODE__NAME` | required | `step` |
| `MODE__INPUTS` | required | JSON list of the input ids to join, in field order |
| `MODE__ELEMENTS` | every element | JSON list of the joined tuple's elements to send on, in order |
| `MODE__OUTPUT` | required | output id of the tuple |

For example, the joint states of an observation and the target answering it,
for pink:

```yaml
    inputs:
      observation: { source: observation/observation, queue_size: 10 }
      joint_target: { source: web/joint_target, queue_size: 10 }
    outputs: [step, node_state]
    env:
      MODE__NAME: "step"
      MODE__INPUTS: '["observation", "joint_target"]'
      MODE__ELEMENTS: '["arm_joint_state", "gripper_joint_state", "joint_target"]'
      MODE__OUTPUT: "step"
```

## Examples

`examples/align_timers.yml` tests `align` with dora's timers and the node-hub
`pyarrow-assert` only: a 100 ms reference and a 20 ms and a 30 ms input; every
tuple must hold each input's nearest tick (`max_skew` 15 ms, `on_skew: error`).
It runs until stopped, so it passes when it ends by itself:

```sh
uv run dora build nodes/sync/examples/align_timers.yml
uv run dora run   nodes/sync/examples/align_timers.yml --stop-after 5s
```
