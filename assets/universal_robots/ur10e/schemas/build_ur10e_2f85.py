# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "mujoco>=3.3",
#     "numpy",
#     "pydantic-settings>=2.6",
# ]
# ///
"""One-off generator for `ur10e_2f85.xml`: a UR10e with a Robotiq 2F-85, the robot alone.

Not a node and not imported by one. It composes the two vendored MuJoCo Menagerie
models (`src/ur10e.xml`, `src/2f85.xml`, BSD-3, see `src/LICENSE.*`) into one flat,
self-contained MJCF with MjSpec, and the committed `ur10e_2f85.xml` is the real
artifact. A cell places it in its world, with props, lights and cameras
(`assets/cells/build_cell.py`). Re-run it only when the robot changes:

    uv run assets/universal_robots/ur10e/schemas/build_ur10e_2f85.py

The gripper servo is `RobotConfig`; every field is also a CLI flag and a `ROBOT_` env var.

What it changes on top of the plain attach, so every node can treat the model uniformly:

* every actuator speaks the URDF: named after the URDF joint it drives, its ctrl and length in
  that joint's units. The arm's are renamed to their joints (`shoulder_pan` ->
  `shoulder_pan_joint`). The 2F-85 `fingers_actuator` (ctrl 0..255) becomes a position servo
  named `finger_joint`, geared from the driver angle (menagerie 0..0.8 rad) to the URDF's
  `finger_joint` (0..0.725 rad), with gains and force range scaled so the servo acts as before.
  Its transmission stays the menagerie's `split` tendon, which equals the driver angle.
* gravity compensation on every robot body, so the position servos hold their pose
  instead of sagging to a torque-balanced offset.
* a site per URDF frame a cell needs: `base_link` (the URDF root, which the cell places) on
  the `base` body, and `tool0` at UR's `tool0`, 16.5 mm beyond the menagerie flange site
  (a camera may be mounted there).
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from pydantic_settings import BaseSettings, SettingsConfigDict


class RobotConfig(BaseSettings):
    """The gripper servo and UR's tool frame."""

    model_config = SettingsConfigDict(env_prefix="ROBOT_")

    # Gripper position servo on the driver joint: its gains and range in driver radians (the
    # menagerie's; 0 = open, 0.8 = closed), and the URDF `finger_joint` angle per driver radian
    # (its range 0..0.725 over the same stroke).
    gripper_kp: float = 100.0
    gripper_kv: float = 10.0
    gripper_range: tuple[float, float] = (0.0, 0.8)
    gripper_gear: float = 0.725 / 0.8
    # UR's `tool0` lies this far beyond the menagerie flange site along the tool axis (measured:
    # the arms otherwise agree within ~1 mm in every pose). A `tool0` site is added there.
    tool0_offset: float = 0.0165


def build(cfg: RobotConfig, here: Path) -> tuple[str, mujoco.MjModel]:
    """The generated MJCF text (meshes relative to `here`) and its compiled model."""
    # The arm model is the base (attaching it with an empty prefix would leave an unnamed
    # default class); the gripper is attached to its flange.
    # Mesh files are flat in ../meshes (menagerie arm .obj + gripper .stl never collide).
    # An attached model keeps resolving meshes against its own meshdir, so build with the
    # absolute directory and make it relative to the written file at the end.
    meshes = here.parent / "meshes"
    spec = mujoco.MjSpec.from_file(str(here / "src" / "ur10e.xml"))
    grip = mujoco.MjSpec.from_file(str(here / "src" / "2f85.xml"))
    for part in (spec, grip):
        part.compiler.meshdir = str(meshes)
    spec.modelname = "ur10e_2f85"
    spec.option.cone = grip.option.cone  # the gripper wants these for stable grasps
    spec.option.impratio = grip.option.impratio
    spec.attach(grip, site=spec.site("attachment_site"), prefix="2f85/")
    for key in list(spec.keys):  # the menagerie's home poses; a cell writes its own
        spec.delete(key)

    # Gripper: position servo in URDF `finger_joint` radians, same tendon transmission. The gear g
    # makes length and ctrl g x the driver angle; dividing kp, kv by g^2 and the force range by
    # g keeps the servo's torque on the driver what it was.
    g = cfg.gripper_gear
    gripper = spec.actuator("2f85/fingers_actuator")
    gripper.name = "finger_joint"
    gripper.gear = [g, 0, 0, 0, 0, 0]
    gripper.ctrlrange = [r * g for r in cfg.gripper_range]
    gripper.forcerange = [f / g for f in gripper.forcerange]
    gripper.gainprm = [cfg.gripper_kp / g**2] + [0.0] * 9
    gripper.biasprm = [0.0, -cfg.gripper_kp / g**2, -cfg.gripper_kv / g**2] + [0.0] * 7
    # Every joint actuator (the arm's) named after its joint.
    for act in spec.actuators:
        if act.trntype == mujoco.mjtTrn.mjTRN_JOINT:
            act.name = act.target

    for body in spec.bodies:
        if body.name and body.name != "world":
            body.gravcomp = 1.0

    # The URDF's frames: its root `base_link` is the menagerie `base` body (whose Rz(pi) is ROS's
    # base_link convention), and UR's `tool0` the flange site moved out along its z.
    spec.body("base").add_site(name="base_link")
    flange = spec.site("attachment_site")
    quat = np.array(flange.quat) / np.linalg.norm(flange.quat)  # MjSpec keeps it unnormalized
    offset = np.zeros(3)
    mujoco.mju_rotVecQuat(offset, np.array([0.0, 0.0, cfg.tool0_offset]), quat)
    flange.parent.add_site(name="tool0", pos=list(np.array(flange.pos) + offset), quat=list(flange.quat))

    model = spec.compile()
    # to_xml writes bare mesh file names under the absolute meshdir: make it relative.
    xml = spec.to_xml().replace(f'meshdir="{meshes}/"', 'meshdir="../meshes"', 1)
    if str(meshes) in xml:
        raise RuntimeError("absolute mesh path left in the generated MJCF")
    return xml, model


def main() -> None:
    here = Path(__file__).resolve().parent
    xml, model = build(RobotConfig(_cli_parse_args=True), here)
    out = here / "ur10e_2f85.xml"
    out.write_text(xml)
    print(f"wrote {out.name}: nq={model.nq} nu={model.nu} nbody={model.nbody}")


if __name__ == "__main__":
    main()
