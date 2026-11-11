# pink

IK on the robot's URDF with [pink](https://github.com/stephane-caron/pink)
(Pinocchio + Coal), one IK step per input.

**`ik`** (default) sits between a controller and the robot. Each `step` tuple
(the targets and the observation they answer, joined by `sync`) is answered at
once with a command one IK step toward the targets. A target is either a joint
target (`names`) or a pose target (`frame`, `reference`), told apart by their
metadata. The command:

1. starts from the measured joints,
2. moves at most `speed_scale` × each joint's URDF velocity limit, within the
   joint limits,
3. stays `min_distance` away from the robot itself and the cell (table, floor),
   slowing down near contact,
4. is not taken if it still collides or has no solution: the last command is
   held and the reason logged.

Joints no target names are held loosely, so a pose moves them freely. An
unreachable pose is approached as closely as the limits allow. A step takes
~4–6 ms.

```
src/config.py        # PinkConfig: URDF (or SCENE) + `mode`
src/main.py          # dora skeleton, identical in every node
src/robot.py         # the URDF in Pinocchio: actuated joints, limits, collision pairs
src/modes/ik.py      # `ik`
```

Collision meshes are replaced by their convex hulls (~400× faster distances).
Pairs on the same or neighbouring joints are not checked; they touch by design.

| id | meaning |
| --- | --- |
| input `step` | from `sync` (`step`): the targets (`MODE__TARGETS`) and the measured joints |
| input `program_state` | `disconnect` stops the node |
| output `joint_target` | every actuated joint, with `step` and `capture_time` |
| output `node_state` | `ready` |

| var | default | meaning |
| --- | --- | --- |
| `URDF` / `SCENE` | unset | the robot's URDF, or a scene file giving it |
| `MODE__TARGETS` | `["joint_target"]` | JSON list of the tuple's elements holding targets |
| `MODE__POSITION_COST` / `MODE__ORIENTATION_COST` | `1.0` / `0.5` | weight of a pose's errors, per m and per rad |
| `MODE__POSTURE_COST` | `0.001` | how firmly untargeted joints are held |
| `MODE__SPEED_SCALE` | `0.8` | fraction of the URDF velocity limits |
| `MODE__MIN_DISTANCE` | `0.01` | clearance kept, m |
| `MODE__BARRIER_GAIN` | `5.0` | how fast that clearance may close, per second (at most 10) |
| `MODE__BARRIER_PAIRS` | `16` | closest pairs the barrier acts on |
