# `ur10e` — UR10e with a Robotiq 2F-85

The file formats and how each file is made: [`assets/README.md`](../../README.md).

One UR10e standing on a table, a 2F-85 on its flange, and two cameras:
`cam_overhead` (fixed, looking down at the table) and `cam_wrist` (on the
flange, looking past the fingers). Their calibration (`calibration/tabletop.yaml`)
is the input: the generator builds the MJCF cameras from it, and consumers
(the rerun 3D view) read it.

```
calibration/
  tabletop.yaml             # the cell's cameras: URDF frame, extrinsics, intrinsics, image size
meshes/                     # menagerie UR10e .obj + 2F-85 .stl, flat (the model's meshdir), in Git LFS
schemas/
  src/                      # the vendored menagerie models + their licenses (generator input)
  build_ur10e_2f85.py       # one-off generator: attach, rename, rescale, add table + home, cameras from calibration/
  ur10e_2f85.xml            # the generated model every node loads (committed)
urdf/
  build_urdf.py             # one-off generator: UR's ur10e xacro + Robotiq 2F-85 URDF, joined at tool0
  ur10e_2f85.urdf           # the URDF every kinematic consumer loads (committed): rerun 3D, later IK
  meshes/                   # its meshes, relative paths (UR's Collada visuals converted to GLB), Git LFS
scenes/
  tabletop.yaml             # scene file: the URDF, the calibration, the MJCF and its parts
```

Two descriptions of one robot: the MJCF (from MuJoCo Menagerie) is the
simulation, the URDF (from the vendors' own descriptions) is the kinematics.
Joint states and targets on the wire use the URDF's names and units; the scene
file's parts map the MJCF onto them (`urdf_joints`, `urdf_scale`). Checked
against each other: the arm joints have the same zeros and directions (the tool
orientation agrees for every joint vector), the Menagerie arm's link lengths
put its flange 1.6-1.7 cm from UR's, and the gripper fingertips agree within a
few mm across the stroke with `urdf_scale` 0.90625.

## Model conventions

- Arm actuators are named after their joints (`shoulder_pan_joint`, …).
- The gripper actuator `gripper` is a position servo in driver-joint radians:
  0 = open, 0.8 = closed. Gripper command, state and ctrl share that unit.
- Gravity compensation is on for every robot body.
- `home` keyframe: a ready pose with the gripper pointing down, 0.3 m above the
  table, open.

The meshes are Git LFS files (see the repo's `.gitattributes`): without
git-lfs a clone holds pointer files and loading the model fails with a mesh
parse error; `git lfs pull` fetches the real ones.

## Regenerating

```sh
uv run assets/universal_robots/ur10e/schemas/build_ur10e_2f85.py   # the MJCF
uv run assets/universal_robots/ur10e/urdf/build_urdf.py                                  # the URDF
```

## Provenance

`schemas/src/ur10e.xml`, `schemas/src/2f85.xml` and the meshes are from
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
(`universal_robots_ur10e`, `robotiq_2f85`), BSD-3: see `schemas/src/LICENSE.*`.
`urdf/` is built from Universal Robots'
[`ur_description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
(BSD-3, `urdf/LICENSE.ur_description`) and
[`robotiq_arg85_description`](https://github.com/a-price/robotiq_arg85_description)
(BSD, per its `package.xml`), fetched by
[`robot_descriptions`](https://github.com/robot-descriptions/robot_descriptions.py).
