# sync

Joins several streams into one message, a *tuple*, for a consumer that needs
them together: a policy or recorder taking an observation of cameras and robot
state, a robot taking all its parts' targets at once. Dora has no built-in join.
The node assumes nothing about what the inputs carry. Two modes decide which
messages go together:

1. **`tuple`**: the newest message of each input, as soon as every input has a
   new one. For commands, where only the latest counts.
2. **`align`**: per reference time (a tick of the observation clock), every
   input's sample nearest it in time. For observations, where the elements must
   show the same instant however late each one is delivered.

```
pyproject.toml         # uv project; the `examples` group adds the node-hub nodes the examples use
src/config.py          # SyncConfig: `mode`, one config class per mode picked by MODE__NAME
src/main.py            # dora skeleton, identical in every node: config -> mode -> Node -> loop
src/tuples.py          # the tuple both modes emit
src/bundler.py         # `tuple` mode's rules: when a tuple goes out, repeats, skew
src/modes/tuple.py     # `tuple` mode
src/modes/align.py     # `align` mode
examples/              # test dataflows: sync with dora's timers and node-hub nodes only
```

## The tuple output type

Both modes emit the same message, values and metadata apart as in every dora
message:

1. **Value:** a one-row Arrow struct with a field per input, named by its id, in
   `inputs` order, holding that input's array, e.g.
   `struct<cam_high: list<uint8>, arm_joint_state: list<double>>`. A consumer
   reads an input's array as `value.field("cam_high")[0].values`.
2. **Metadata:** every key of every input's metadata as `<input id>.<key>`
   (`cam_high.encoding`, `arm_joint_state.names`, `cam_high.timestamp`: dora's
   send time), plus the tuple's own `capture_time`. Nothing is converted or
   dropped. Dora's metadata is flat, hence one key per input and key, and an
   input id may not contain a `.`.

Inputs: every id in `inputs`, and `program_state` (the node stops on
`disconnect`). Outputs: `<output>` (the tuple) and `node_state` (`ready`).

Both modes check *skew*, how far apart in time a tuple's elements are, when
`max_skew` is set, and handle it as `on_skew` says: `ignore`, `warn` or `error`
(stop the node).

## `tuple` mode

1. A message replaces its input's previous one: the newest wins, nothing queues.
2. Once every input has sent a new message since the last tuple, a tuple of them
   goes out at once. There is no timer: a tuple waits only for the slowest input.
3. Until then nothing goes out: no old message is resent, no partial tuple sent.
   If a producer stops, tuples stop too, on purpose: resending its last message
   would tell the consumer that a dead part is still alive.
4. An input that sends again before its tuple is complete is a *repeat*, handled
   as `on_repeat` says: normal when a fast producer waits on a slow one, trouble
   when the inputs should arrive in lockstep.
5. The tuple's `capture_time` is the oldest of its elements', when they all have
   one. Skew is the spread of the elements' times (`skew_time`).

| var | default | meaning |
| --- | --- | --- |
| `MODE__NAME` | required | `tuple` |
| `MODE__INPUTS` | required | JSON list of the input ids, in field order |
| `MODE__OUTPUT` | required | output id of the tuple |
| `MODE__ON_REPEAT` | `ignore` | `ignore`, `warn` or `error` on a repeat |
| `MODE__MAX_SKEW` | unset | seconds; unset: not checked |
| `MODE__ON_SKEW` | `warn` | `ignore`, `warn` (once when skew appears) or `error` |
| `MODE__SKEW_TIME` | `capture_time` | the elements' time: `capture_time` (required on every input) or dora's send `timestamp` |

## `align` mode

1. Every input keeps its samples of the last `history` seconds. A sample's time
   is its `capture_time`, or dora's send `timestamp` (`input_time`).
2. Every message on the `reference` input adds a reference time t (its
   `timestamp` or `capture_time`, by `reference_time`).
3. t is settled once every input has a sample at or after it, so the nearest is
   known, or once `timeout` has passed since t (then the nearest so far).
   Reference times settle in order.
4. A settled t becomes a tuple of every input's sample nearest t, with
   `capture_time` = t. Skew is each element's distance from t; every tuple over
   `max_skew` is reported, naming its elements.
5. Until every input has sent once, reference times are dropped.

So a camera whose frames arrive 25 ms after they were taken still lines up with
the joint states captured at the same instant: delivery delay costs latency, not
alignment. Give the inputs deep queues (`queue_size: 100`): the node chooses
among samples rather than taking the newest.

| var | default | meaning |
| --- | --- | --- |
| `MODE__NAME` | required | `align` |
| `MODE__REFERENCE` | required | the input whose messages give the reference times |
| `MODE__REFERENCE_TIME` | `timestamp` | a reference message's time: dora's send `timestamp` (a timer's tick) or `capture_time` |
| `MODE__INPUTS` | required | JSON list of the input ids to align, in field order |
| `MODE__INPUT_TIME` | `capture_time` | an input message's time: `capture_time` (required on every message) or dora's send `timestamp` |
| `MODE__OUTPUT` | required | output id of the tuple |
| `MODE__HISTORY` | `1.0` | seconds of samples kept per input |
| `MODE__TIMEOUT` | `0.2` | seconds a reference time waits for every input |
| `MODE__MAX_SKEW` | unset | seconds an element may be off its reference time; unset: not checked |
| `MODE__ON_SKEW` | `warn` | `ignore`, `warn` (per tuple) or `error` |

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
      MODE__OUTPUT: "observation"
      MODE__MAX_SKEW: "0.02"
```

## Examples

The dataflows in `examples/` use nothing of ours but sync: dora's timers and the
node-hub nodes `pyarrow-sender` and `pyarrow-assert` (installed by their build
step). Each is a test: a run that ends with exit code 1 has failed.

1. `tuple_test.yml`: two senders send one fixed array each; `pyarrow-assert`
   checks the tuple. It ends by itself.
2. `tuple_timers.yml`: a 100 ms and a 330 ms timer; one tuple per slow tick, the
   fast timer's extra ticks logged as repeats, drifting rates logged as skew.
3. `align_timers.yml`: a 100 ms reference and a 20 ms and a 30 ms input; every
   tuple must hold each input's nearest tick (`max_skew` 20 ms, `on_skew: error`).

The timer examples run until stopped, so run them with `--stop-after`:

```sh
uv run dora build nodes/sync/examples/align_timers.yml
uv run dora run   nodes/sync/examples/tuple_test.yml
uv run dora run   nodes/sync/examples/tuple_timers.yml --stop-after 5s
uv run dora run   nodes/sync/examples/align_timers.yml --stop-after 5s
```
