# `ur10e` — UR10e with a Robotiq 2F-85

The file formats and how each file is made: [`assets/README.md`](../../README.md).

The robot alone: a UR10e with a 2F-85 on its flange. The tabletop cell it works
in, with its table and cameras, is [`assets/cells/ur10e_tabletop`](../../cells/ur10e_tabletop).

```
meshes/                     # menagerie UR10e .obj + 2F-85 .stl, flat (the model's meshdir), in Git LFS
schemas/
  src/                      # the vendored menagerie models + their licenses (generator input)
  build_ur10e_2f85.py       # one-off generator: attach, rename and gear the actuators, add the URDF frame sites
  ur10e_2f85.xml            # the generated robot MJCF (committed), which a cell places
urdf/
  build_urdf.py             # one-off generator: UR's ur10e xacro + Robotiq 2F-85 URDF, joined at tool0
  ur10e_2f85.urdf           # the robot URDF (committed), rooted at base_link, which a cell places
  meshes/                   # its meshes, relative paths (UR's Collada visuals converted to GLB), Git LFS
```

Two descriptions of one robot: the MJCF (from MuJoCo Menagerie) is the
simulation, the URDF (from the vendors' own descriptions) is the kinematics.
Joint states and targets on the wire use the URDF's names and units, and so do
the MJCF's actuators (the generator names and scales them). Checked against
each other: the arm joints have the same zeros and directions (the tool
orientation agrees for every joint vector), the arms agree within ~1 mm in
every pose, the MJCF has a `tool0` site at UR's `tool0` (the Menagerie flange
site sits 16.5 mm short of it; tool0 cameras mount on the site), and the gripper fingertips agree within a
few mm across the stroke with the gripper actuator geared 0.725 / 0.8 (the
URDF's `finger_joint` range over the Menagerie driver's).

## Model conventions

- Every actuator is named after the URDF joint it drives, in its units:
  `shoulder_pan_joint`, … and `finger_joint` (0 = open, 0.725 = closed), a
  position servo geared from the menagerie driver (0..0.8 rad).
- Gravity compensation is on for every robot body.
- Sites for the URDF frames: `base_link` on the `base` body (whose Rz(pi) is
  ROS's base_link convention) and `tool0`.
- No keyframe: a cell writes `home`.

The meshes are Git LFS files (see the repo's `.gitattributes`): without
git-lfs a clone holds pointer files and loading the model fails with a mesh
parse error; `git lfs pull` fetches the real ones.

## Regenerating

```sh
uv run assets/universal_robots/ur10e/schemas/build_ur10e_2f85.py   # the MJCF
uv run assets/universal_robots/ur10e/urdf/build_urdf.py             # the URDF
uv run assets/cells/build_cell.py --scene assets/cells/ur10e_tabletop/scene.yaml   # then its cells
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
