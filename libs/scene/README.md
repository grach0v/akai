# akai-scene

The scene file format as a Python package. A scene file names everything the
nodes need to know about a robot cell (its URDF, its camera calibration, its
MuJoCo model and parts), so a dataflow names the cell once instead of every file
in every node. The format, and how to write the files, are in
[`assets/README.md`](../../assets/README.md).

The scene file is a convenience, never a requirement: every value in it can be
given to a node through the node's own config instead. A node built on this
package therefore takes each value from exactly one of two places, its config or
the scene file, and refuses both.

The package only reads and validates the file. Using what it names (loading the
URDF, resolving parts in the MJCF, ...) is each node's business.

## What it provides

| name | what |
| --- | --- |
| `load_scene(path) -> Scene` | read and validate a scene file: unknown keys, lists of the wrong length and files that do not exist are errors; paths come back relative to the working directory |
| `Scene` | `urdf`, `calibration` (paths, or None) and `mujoco` (a `MujocoScene`, or None) |
| `MujocoScene` | `model` (the MJCF path) and `parts` (name -> `PartSpec`) |
| `PartSpec` | one part: `joints`, `actuators`, `urdf_joints`, `urdf_scale`, and the defaults applied: `actuator_names`, `urdf_names`, `scale` |
| `one_of(option, explicit, scene, from_scene)` | a value from the config or from the scene file: raises if both give it, None if neither does |

## Using it in a node

1. Depend on it by path, editable, so a change here reaches every node:

   ```toml
   # nodes/<node>/pyproject.toml
   dependencies = [..., "akai-scene"]

   [tool.uv.sources]
   akai-scene = { path = "../../libs/scene", editable = true }
   ```

2. Give the config an explicit field for every value the node needs, plus an
   optional `scene`, and fill the fields from the scene file in a validator. This
   is the `rerun` node's:

   ```python
   from akai_scene import load_scene, one_of

   class RecordModeConfig(BaseModel):
       robot_urdf: Path | None = None    # MODE__ROBOT_URDF
       calibration: Path | None = None   # MODE__CALIBRATION
       scene: Path | None = None         # MODE__SCENE: gives both instead

       @model_validator(mode="after")
       def _from_scene(self):
           if self.scene is not None:
               scene = load_scene(self.scene)
               self.robot_urdf = one_of("MODE__ROBOT_URDF", self.robot_urdf, self.scene, scene.urdf)
               self.calibration = one_of("MODE__CALIBRATION", self.calibration, self.scene, scene.calibration)
           return self
   ```

   If a value is required, check it after filling, with an error naming both
   ways, as the `web_controller` node does:

   ```python
   if self.urdf is None:
       raise ValueError("the sliders need the robot's URDF: MODE__URDF, or a MODE__SCENE file with `urdf`")
   ```

3. The rest of the node only reads its config fields (`opts.robot_urdf`, ...)
   and never knows whether they came from a scene file.

Parts work the same way: the `mujoco` node has `MODEL` and `PARTS` fields, and
`PARTS` uses the `PartSpec` type, so parts given in the config are validated
exactly like parts in a scene file:

```python
from akai_scene import PartSpec, load_scene, one_of

class MujocoConfig(BaseSettings):
    scene: Path | None = None                    # SCENE
    model: Path | None = None                    # MODEL
    parts: dict[str, PartSpec] | None = None     # PARTS (JSON)

    @model_validator(mode="after")
    def _from_scene(self):
        if self.scene is not None:
            sim = load_scene(self.scene).mujoco
            self.model = one_of("MODEL", self.model, self.scene, sim and sim.model)
            self.parts = one_of("PARTS", self.parts, self.scene, sim and sim.parts)
        if self.model is None or not self.parts:
            raise ValueError("the sim needs MODEL and PARTS, or a SCENE file with a `mujoco` section")
        return self
```

## Without a scene file

Any node runs without one: set its explicit fields in the dataflow. The same
cell as `assets/universal_robots/ur10e/scenes/tabletop.yaml`, by hand:

```yaml
  - id: sim
    env:
      MODEL: "../../../assets/universal_robots/ur10e/schemas/ur10e_2f85.xml"
      PARTS: >-
        {"arm": {"joints": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]},
         "gripper": {"joints": ["2f85/right_driver_joint"], "actuators": ["gripper"],
                     "urdf_joints": ["finger_joint"], "urdf_scale": [0.90625]}}

  - id: web
    env:
      MODE__URDF: "../../../assets/universal_robots/ur10e/urdf/ur10e_2f85.urdf"

  - id: rerun_record
    env:
      MODE__ROBOT_URDF: "../../../assets/universal_robots/ur10e/urdf/ur10e_2f85.urdf"
      MODE__CALIBRATION: "../../../assets/universal_robots/ur10e/calibration/tabletop.yaml"
```

A scene file and explicit fields can also be mixed, as long as each value comes
from one place: a scene file without `calibration`, say, plus `MODE__CALIBRATION`
in the dataflow. Paths in the config are relative to the dataflow directory;
paths in a scene file are relative to the scene file.
