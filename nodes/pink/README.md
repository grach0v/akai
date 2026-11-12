# pink

IK on the robot's URDF with [pink](https://github.com/stephane-caron/pink)
(Pinocchio + Coal), one IK step per input.

**`ik`** (default) sits between a controller and the robot. Each `step` tuple
(the targets and the observation they answer, joined by `sync`) is answered at
once with a command one IK step toward the targets. A target is either a joint
target (`names`) or a pose target (`frame`, `reference`), told apart by their
metadata. The command:

1. starts from the measured joints, and again on every `program_state` change;
   after that it builds on itself,
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
src/main.py          # runs the modes (akai_node)
src/robot.py         # the URDF in Pinocchio: actuated joints, limits, collision pairs
src/modes/ik.py      # `ik`
```

Collision meshes are replaced by their convex hulls (~400× faster distances).
Pairs on the same or neighbouring joints are not checked; they touch by design.

| id | meaning |
| --- | --- |
| input `step` | from `sync` (`step`): the targets (`MODE__TARGETS`) and the measured joints |
| input `program_state` | a change restarts the command at the measured joints; `disconnect` stops the node |
| output `joint_target` | every actuated joint, with `step` and `capture_time` |
| output `node_state` | `ready` |

| var | default | meaning |
| --- | --- | --- |
| `URDF` / `SCENE` | unset | the robot's URDF, or a scene file giving it |
| `MODE__TARGETS` | `["joint_target"]` | JSON list of the tuple's elements holding targets |
| `MODE__SPEED_SCALE` | `0.8` | fraction of the URDF velocity limits |
| `MODE__MIN_DISTANCE` | `0.01` | clearance kept, m |
