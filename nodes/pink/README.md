# pink

Robot kinematics on the robot's URDF with [pink](https://github.com/stephane-caron/pink): every
mode is one pink IK step per input, on the [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
model and its collision library Coal. Today one mode:

- **`joints`** (default): a safety filter between a controller and the robot. Every incoming
  target (with the observation it answers, joined by `sync`'s `step` mode) is answered at once with a command one IK step toward it, over the time since the
  previous target: at most `speed_scale` × each joint's URDF velocity limit, within the joint
  limits, and kept `min_distance` away from the robot itself and the cell (table, floor) by a collision barrier,
  so near contact it slows down and slides along. A step that still collides, or has no solution,
  is not taken: the last command is held and the reason logged once. A controller streams its
  target (the web controller does, at 30 Hz); one sent once moves the robot one step.

```
src/config.py        # PinkConfig: URDF (or SCENE) + `mode`, picked by MODE__NAME
src/main.py          # dora skeleton, identical in every node: config -> mode -> Node -> loop
src/robot.py         # the URDF in Pinocchio: actuated joints, limits, collision pairs
src/modes/joints.py  # `joints`: the filter, a pink posture task
```

## The robot model

The URDF's actuated joints are the configuration; mimic joints (a gripper's linkage) follow their
driver, as the URDF declares. Self-collision is checked between every pair of collision
geometries except those on one joint or on neighbouring joints, which touch by design; a mimic
joint counts as the joint it follows, so a linkage never collides with itself. Give it a cell's
URDF (`assets/cells/`), and the table and floor in it, fixed to `world` like the robot's base, are
collision geometry too. Collision meshes are replaced by their convex hulls, which makes distances
~400× faster and smooth enough for the barrier; the UR meshes are nearly convex. The barrier acts
on the closest `MODE__BARRIER_PAIRS` pairs only, and the final check on all of them. One step
takes ~4 ms, mostly pink's barrier in Python.

## Inputs and outputs (`joints`)

| id | source | meaning |
| --- | --- | --- |
| `step` | `sync` (`step`) | one step's tuple: the target (element `MODE__TARGET`, a named vector of any of the URDF's actuated joints) and the observation answered by it (its named vectors are the measured joints, where the command starts); each one is answered with a command |
| `program_state` | manager | `disconnect` stops the node |
| output `joint_target` | | a named vector of every actuated joint, with `capture_time` and the tuple's `step` |
| output `node_state` | | `ready` |

## Config

| var | default | meaning |
| --- | --- | --- |
| `URDF` | unset | the robot's URDF, relative to the dataflow directory |
| `SCENE` | unset | a scene file giving `URDF` instead |
| `MODE__TARGET` | `joint_target` | the `step` tuple's element holding the target |
| `MODE__SPEED_SCALE` | `0.5` | fraction of each joint's URDF velocity limit |
| `MODE__MIN_DISTANCE` | `0.01` | distance (m) the barrier keeps between geometries |
| `MODE__BARRIER_GAIN` | `1.0` | how fast that distance may be closed (per second, fraction of what is left) |
| `MODE__BARRIER_PAIRS` | `16` | how many of the closest pairs the barrier acts on |
