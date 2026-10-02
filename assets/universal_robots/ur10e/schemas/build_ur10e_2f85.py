# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "mujoco>=3.3",
#     "numpy",
#     "pydantic-settings>=2.6",
#     "pyyaml",
# ]
# ///
"""One-off generator for `ur10e_2f85.xml`: a UR10e with a Robotiq 2F-85 on a table.

Not a node and not imported by one. It composes the two vendored MuJoCo Menagerie
models (`src/ur10e.xml`, `src/2f85.xml`, BSD-3, see `src/LICENSE.*`) into one flat,
self-contained MJCF with MjSpec, and the committed `ur10e_2f85.xml` is the real
artifact. Re-run it only when the cell changes:

    uv run assets/universal_robots/ur10e/schemas/build_ur10e_2f85.py

The cell (table, home pose, gripper servo, cameras) is `CellConfig`; every field is also a
CLI flag (`--table_top_z 0.8`) and a `CELL_` env var, so a variant needs no code edit.

What it changes on top of the plain attach, so every node can treat the model uniformly:

* every joint actuator is renamed to its joint (`shoulder_pan` -> `shoulder_pan_joint`);
  a node finds an arm part's actuators by its joint names.
* the 2F-85 `fingers_actuator` (ctrl 0..255) is rescaled to a position servo in driver-joint
  radians (ctrl 0..0.8, 0 = open) and renamed `gripper`, so gripper command, state and ctrl
  share one unit. The transmission stays the menagerie's `split` tendon, which with the
  driver-joint equality equals the driver angle.
* gravity compensation on every robot body, so the position servos hold their pose
  instead of sagging to a torque-balanced offset.
* a floor, a table the arm stands on, lights, and the cell's cameras, built from its
  calibration file (`../calibration/tabletop.yaml`): each camera is mounted where its
  extrinsics say, on the MJCF element of its URDF frame (`urdf_frames`), with its exact
  pinhole intrinsics, so the simulated cameras match their calibration by construction.
* one `home` keyframe for the whole model (a ready pose over the table, gripper open), with
  ctrl equal to the joint positions so the servos start at rest.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import mujoco
import numpy as np
import yaml
from pydantic_settings import BaseSettings, SettingsConfigDict


class CellConfig(BaseSettings):
    """The cell around the arm, in the world frame (floor at z = 0), and the model's defaults."""

    model_config = SettingsConfigDict(env_prefix="CELL_")  # a bare HOME would be the shell's

    # The arm base stands on the table top; the table extends in front of it (+x).
    table_top_z: float = 0.75
    table_center: tuple[float, float] = (0.35, 0.0)
    table_half_size: tuple[float, float, float] = (0.75, 0.6, 0.02)
    # `home` keyframe of the arm joints: a ready pose, gripper down, 0.3 m above the table at x = 0.65.
    home: dict[str, float] = {
        "shoulder_pan_joint": 2.8706,
        "shoulder_lift_joint": -1.5915,
        "elbow_joint": 2.0216,
        "wrist_1_joint": -2.0009,
        "wrist_2_joint": -1.5708,
        "wrist_3_joint": -1.8418,
    }
    # Gripper position servo on the driver joint: its gains (the menagerie's, now in radians)
    # and range (0 = open, 0.8 rad = closed). `home` opens it.
    gripper_kp: float = 100.0
    gripper_kv: float = 10.0
    gripper_range: tuple[float, float] = (0.0, 0.8)
    # The cell's camera calibration, relative to the robot directory (the parent of schemas/).
    calibration: Path = Path("calibration/tabletop.yaml")
    # Where each URDF frame a camera may be mounted on is in the MJCF: a body, or a site.
    # Checked: the URDF root `world` is the MJCF arm base (the base body's Rz(pi) is ROS's
    # base_link convention), and `tool0` is the flange site, orientation for orientation.
    urdf_frames: dict[str, tuple[Literal["body", "site"], str]] = {
        "world": ("body", "base"),
        "tool0": ("site", "attachment_site"),
    }
    # Largest offscreen render the model allows (a renderer's width/height must fit).
    offscreen_size: tuple[int, int] = (1280, 960)


def add_camera(spec: mujoco.MjSpec, name: str, cal: dict, frames: dict[str, tuple[str, str]]) -> None:
    """Mount a calibrated camera: on the body of its URDF frame, at the pose its extrinsics give
    in that frame, with its pinhole intrinsics."""
    kind, element = frames[cal["frame"]]
    if kind == "body":
        body, frame_pos, frame_quat = spec.body(element), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    else:
        site = spec.site(element)
        body, frame_pos, frame_quat = site.parent, np.array(site.pos), np.array(site.quat)
    frame_quat = frame_quat / np.linalg.norm(frame_quat)
    x, y, z, qx, qy, qz, qw = cal["extrinsics"]
    # A MuJoCo camera looks along its -z with +y up: the optical frame turned half a turn about x.
    optical_quat, flip = np.zeros(4), np.array([0.0, 1.0, 0.0, 0.0])
    mujoco.mju_mulQuat(optical_quat, np.array([qw, qx, qy, qz]), flip)
    pos, quat = np.zeros(3), np.zeros(4)
    mujoco.mju_mulPose(pos, quat, frame_pos, frame_quat, np.array([x, y, z]), optical_quat)
    fx, fy, cx, cy = cal["intrinsics"]
    width, height = cal["width"], cal["height"]
    camera = body.add_camera(name=name, pos=pos.tolist(), quat=quat.tolist())
    camera.resolution = [width, height]
    camera.focal_pixel = [fx, fy]
    # MuJoCo's principal point is the offset of the sensor centre from the optical axis.
    camera.principal_pixel = [width / 2 - cx, height / 2 - cy]
    # A sensor size switches MuJoCo from fovy to these intrinsics; pixel values are scaled by
    # size / resolution, so any pixel pitch (here 3 um) gives the same camera.
    camera.sensor_size = [width * 3e-6, height * 3e-6]


def build(cfg: CellConfig, here: Path) -> tuple[str, mujoco.MjModel]:
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
    for key in list(spec.keys):  # the arm-only `home`; a whole-model one is written below
        spec.delete(key)
    spec.option.cone = grip.option.cone  # the gripper wants these for stable grasps
    spec.option.impratio = grip.option.impratio
    spec.visual.global_.offwidth, spec.visual.global_.offheight = cfg.offscreen_size
    spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
    spec.visual.headlight.ambient = [0.2, 0.2, 0.2]

    world = spec.worldbody
    world.add_light(name="sun", pos=[0, 0, 3], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], rgba=[0.3, 0.35, 0.4, 1])
    (cx, cy), (hx, hy, hz), top = cfg.table_center, cfg.table_half_size, cfg.table_top_z
    world.add_geom(
        name="table",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[cx, cy, top - hz],
        size=[hx, hy, hz],
        rgba=[0.65, 0.5, 0.35, 1],
    )
    leg_half_height = (top - 2 * hz) / 2
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        world.add_geom(
            name=f"table_leg{i}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[cx + sx * (hx - 0.05), cy + sy * (hy - 0.05), leg_half_height],
            size=[0.03, 0.03, leg_half_height],
            rgba=[0.25, 0.2, 0.15, 1],
        )
    spec.body("base").pos = [0, 0, top]
    spec.attach(grip, site=spec.site("attachment_site"), prefix="2f85/")

    # Gripper: position servo in driver-joint radians, same tendon transmission.
    gripper = spec.actuator("2f85/fingers_actuator")
    gripper.name = "gripper"
    gripper.ctrlrange = list(cfg.gripper_range)
    gripper.gainprm = [cfg.gripper_kp] + [0.0] * 9
    gripper.biasprm = [0.0, -cfg.gripper_kp, -cfg.gripper_kv] + [0.0] * 7
    # Every joint actuator (the arm's) named after its joint.
    for act in spec.actuators:
        if act.trntype == mujoco.mjtTrn.mjTRN_JOINT:
            act.name = act.target

    for body in spec.bodies:
        if body.name and body.name != "world":
            body.gravcomp = 1.0

    calibration = yaml.safe_load((here.parent / cfg.calibration).read_text())
    for name, cal in calibration["cameras"].items():
        add_camera(spec, name, cal, cfg.urdf_frames)

    # One whole-model home keyframe (the gripper's own key is dropped: its size is wrong),
    # with every joint servo's ctrl at its joint's home so the servos start at rest.
    model = spec.compile()
    for key in list(spec.keys):
        spec.delete(key)
    qpos = model.qpos0.copy()
    for joint, q in cfg.home.items():
        qpos[model.jnt_qposadr[model.joint(joint).id]] = q
    ctrl = np.zeros(model.nu)
    for a in range(model.nu):
        if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT:
            ctrl[a] = qpos[model.jnt_qposadr[model.actuator_trnid[a][0]]]
    ctrl[model.actuator("gripper").id] = cfg.gripper_range[0]
    spec.add_key(name="home", qpos=qpos.tolist(), ctrl=ctrl.tolist())

    model = spec.compile()
    # to_xml writes bare mesh file names under the absolute meshdir: make it relative.
    xml = spec.to_xml().replace(f'meshdir="{meshes}/"', 'meshdir="../meshes"', 1)
    if str(meshes) in xml:
        raise RuntimeError("absolute mesh path left in the generated MJCF")
    return xml, model


def main() -> None:
    here = Path(__file__).resolve().parent
    xml, model = build(CellConfig(_cli_parse_args=True), here)
    out = here / "ur10e_2f85.xml"
    out.write_text(xml)
    print(f"wrote {out.name}: nq={model.nq} nu={model.nu} nbody={model.nbody} ncam={model.ncam}")


if __name__ == "__main__":
    main()
