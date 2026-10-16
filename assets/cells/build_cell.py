# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "akai_scene",
#     "mujoco>=3.3",
#     "numpy",
#     "pydantic-settings>=2.6",
# ]
#
# [tool.uv.sources]
# akai_scene = { path = "../../libs/akai_scene", editable = true }
# ///
"""Cell builder: composes a scene file's `robots` and `props` into the cell's URDF and MJCF.

Not a node and not imported by one. Every cell has its own `world` frame, at floor level, and
places each robot and prop in it by its `pose`. The builder writes the two files the scene file
names, which the nodes load, and both are committed:

* the URDF (`urdf`): a `world` link, every robot's URDF and every prop's link, each fixed to
  `world` by a `<name>_mount` joint at its pose. Mesh paths are made relative to the new file.
* the MJCF (`mujoco.model`): the robot's MJCF with its bodies placed so its URDF root link (the
  site of that name) is at the robot's pose; every prop, which MuJoCo reads from its URDF;
  a light; the cameras of the calibration file, each mounted where its extrinsics say, on
  `world` or on the robot's site named after its URDF frame, with its exact pinhole
  intrinsics; and a `home` keyframe, with every servo's ctrl at its joint so it starts at rest.

Re-run it when the scene's robots, props or calibration change:

    uv run assets/cells/build_cell.py --scene assets/cells/ur10e_tabletop/scene.yaml

One robot per cell for now: a second one needs its names prefixed in both files.
"""

from __future__ import annotations

import math
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from akai_scene import CameraCalibration, Scene, load_calibration, load_scene
from pydantic_settings import BaseSettings, SettingsConfigDict


class CellBuildConfig(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="CELL_")

    scene: Path
    # Largest offscreen render the MJCF allows (a renderer's width/height must fit).
    offscreen_size: tuple[int, int] = (1280, 960)
    # The cameras' near clipping distance (m). MuJoCo's is a fraction of the model's extent,
    # which a floor makes large (8.5 cm here, cutting the wrist camera's fingers off).
    near_clip: float = 0.01


# -- URDF ------------------------------------------------------------------------------------


def origin(pose: list[float]) -> dict[str, str]:
    """A URDF `<origin>` for a pose [x, y, z, qx, qy, qz, qw]: roll, pitch, yaw about fixed axes."""
    x, y, z, qx, qy, qz, qw = pose
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return {"xyz": f"{x:g} {y:g} {z:g}", "rpy": f"{roll:.9g} {pitch:.9g} {yaw:.9g}"}


def root_link(urdf: ET.Element) -> str:
    children = {joint.find("child").get("link") for joint in urdf.iter("joint")}
    (root,) = [link.get("name") for link in urdf.findall("link") if link.get("name") not in children]
    return root


def relink_meshes(urdf: ET.Element, source_dir: Path, out_dir: Path) -> None:
    """Make relative mesh paths relative to `out_dir` instead of `source_dir`."""
    for mesh in urdf.iter("mesh"):
        filename = mesh.get("filename")
        if "://" not in filename and not os.path.isabs(filename):
            mesh.set("filename", os.path.relpath(source_dir / filename, out_dir))


def mount(cell: ET.Element, name: str, child: str, pose: list[float]) -> None:
    joint = ET.SubElement(cell, "joint", name=f"{name}_mount", type="fixed")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", origin(pose))


def build_urdf(scene: Scene, out: Path) -> str:
    cell = ET.Element("robot", name=out.stem)
    ET.SubElement(cell, "link", name="world")
    for name, robot in scene.robots.items():
        urdf = ET.parse(robot.urdf).getroot()
        relink_meshes(urdf, robot.urdf.parent, out.parent)
        cell.extend(list(urdf))
        mount(cell, name, root_link(urdf), robot.pose)
    for name, prop in scene.props.items():
        urdf = ET.parse(prop.urdf).getroot()
        (link,) = urdf.findall("link")
        link.set("name", name)
        for material in urdf.iter("material"):  # materials are global in a URDF: keep them apart
            material.set("name", f"{name}_{material.get('name')}")
        relink_meshes(urdf, prop.urdf.parent, out.parent)
        cell.extend(list(urdf))
        mount(cell, name, name, prop.pose)
    for tag in ("link", "joint"):
        names = [element.get("name") for element in cell.findall(tag)]
        if len(names) != len(set(names)):
            raise ValueError(
                f"{out.name}: {tag} names repeat: {sorted({n for n in names if names.count(n) > 1})}"
            )
    ET.indent(cell)
    return ET.tostring(cell, encoding="unicode") + "\n"


# -- MJCF ------------------------------------------------------------------------------------


def wxyz(pose: list[float]) -> tuple[np.ndarray, np.ndarray]:
    """A pose's position and MuJoCo (w, x, y, z) quaternion."""
    x, y, z, qx, qy, qz, qw = pose
    return np.array([x, y, z]), np.array([qw, qx, qy, qz])


def site_pose(model: mujoco.MjModel, name: str) -> tuple[np.ndarray, np.ndarray]:
    """A site's pose in the model's world, at qpos0."""
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, data.site(name).xmat)
    return data.site(name).xpos.copy(), quat


def add_camera(spec: mujoco.MjSpec, name: str, cal: CameraCalibration) -> None:
    """Mount a calibrated camera: on `world` or on the site named after its URDF frame, at the
    pose its extrinsics give in that frame, with its pinhole intrinsics."""
    if cal.frame == "world":
        body, frame_pos, frame_quat = spec.worldbody, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    else:
        site = spec.site(cal.frame)
        if site is None:
            raise ValueError(f"camera {name}: the robot's MJCF has no site {cal.frame!r} for its frame")
        body, frame_pos, frame_quat = site.parent, np.array(site.pos), np.array(site.quat)
    frame_quat = frame_quat / np.linalg.norm(frame_quat)  # MjSpec keeps it unnormalized
    position, rotation = wxyz(cal.extrinsics)
    # A MuJoCo camera looks along its -z with +y up: the optical frame turned half a turn about x.
    optical_quat, flip = np.zeros(4), np.array([0.0, 1.0, 0.0, 0.0])
    mujoco.mju_mulQuat(optical_quat, rotation, flip)
    pos, quat = np.zeros(3), np.zeros(4)
    mujoco.mju_mulPose(pos, quat, frame_pos, frame_quat, position, optical_quat)
    fx, fy, cx, cy = cal.intrinsics
    camera = body.add_camera(name=name, pos=pos.tolist(), quat=quat.tolist())
    camera.resolution = [cal.width, cal.height]
    camera.focal_pixel = [fx, fy]
    # MuJoCo's principal point is the offset of the sensor centre from the optical axis.
    camera.principal_pixel = [cal.width / 2 - cx, cal.height / 2 - cy]
    # A sensor size switches MuJoCo from fovy to these intrinsics; pixel values are scaled by
    # size / resolution, so any pixel pitch (here 3 um) gives the same camera.
    camera.sensor_size = [cal.width * 3e-6, cal.height * 3e-6]


def load_prop(urdf: Path) -> mujoco.MjSpec:
    """A prop's URDF in MuJoCo: its visuals drawn, its collision shapes colliding but hidden
    (MuJoCo would otherwise drop the visuals and colour the collision shapes by guesswork)."""
    text = urdf.read_text().replace("<link", '<mujoco><compiler discardvisual="false"/></mujoco><link', 1)
    prop = mujoco.MjSpec.from_string(text)
    for geom in prop.geoms:
        if geom.contype or geom.conaffinity:
            geom.group = 3  # not drawn by default
    return prop


def build_mjcf(scene: Scene, out: Path, cfg: CellBuildConfig) -> tuple[str, mujoco.MjModel]:
    if len(scene.robots) != 1:
        raise ValueError(f"{cfg.scene}: one robot per cell for now, not {len(scene.robots)}")
    ((name, robot),) = scene.robots.items()
    spec = mujoco.MjSpec.from_file(str(robot.mjcf))
    # Meshes resolve against the robot's own meshdir: build with it absolute, and make it relative
    # to the written file at the end.
    meshes = (robot.mjcf.parent / spec.compiler.meshdir).resolve()
    spec.compiler.meshdir = str(meshes)
    spec.modelname = out.stem

    # Place the robot: move its top-level bodies by F = pose * root^-1, so its URDF root link's
    # site lands at the pose.
    root = root_link(ET.parse(robot.urdf).getroot())
    root_pos, root_quat = site_pose(spec.compile(), root)
    inv_pos, inv_quat = np.zeros(3), np.zeros(4)
    mujoco.mju_negPose(inv_pos, inv_quat, root_pos, root_quat)
    f_pos, f_quat = np.zeros(3), np.zeros(4)
    mujoco.mju_mulPose(f_pos, f_quat, *wxyz(robot.pose), inv_pos, inv_quat)
    for body in spec.worldbody.bodies:
        pos, quat = np.zeros(3), np.zeros(4)
        body_quat = np.array(body.quat) / np.linalg.norm(body.quat)
        mujoco.mju_mulPose(pos, quat, f_pos, f_quat, np.array(body.pos), body_quat)
        body.pos, body.quat = pos.tolist(), quat.tolist()

    for prop_name, prop in scene.props.items():
        pos, quat = wxyz(prop.pose)
        frame = spec.worldbody.add_frame(pos=pos.tolist(), quat=quat.tolist())
        spec.attach(load_prop(prop.urdf), frame=frame, prefix=f"{prop_name}/")

    spec.worldbody.add_light(
        name="sun", pos=[0, 0, 3], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    )
    spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
    spec.visual.headlight.ambient = [0.2, 0.2, 0.2]
    spec.visual.global_.offwidth, spec.visual.global_.offheight = cfg.offscreen_size
    if scene.calibration is not None:
        for camera, cal in load_calibration(scene.calibration).items():
            add_camera(spec, camera, cal)

    # `home`: the listed joints (by the actuator driving each), every servo's ctrl at its length.
    model = spec.compile()
    data = mujoco.MjData(model)
    for actuator, value in robot.home.items():
        a = model.actuator(actuator)
        if a.trntype[0] != mujoco.mjtTrn.mjTRN_JOINT:
            raise ValueError(f"{name}.home: {actuator} does not drive a joint")
        data.qpos[model.jnt_qposadr[a.trnid[0]]] = value / a.gear[0]
    mujoco.mj_forward(model, data)
    spec.add_key(name="home", qpos=data.qpos.tolist(), ctrl=data.actuator_length.tolist())

    model = spec.compile()
    spec.visual.map.znear = cfg.near_clip / model.stat.extent
    model = spec.compile()
    xml = re.sub(
        f'meshdir="{re.escape(str(meshes))}/?"',
        f'meshdir="{os.path.relpath(meshes, out.parent)}"',
        spec.to_xml(),
        count=1,
    )
    if str(meshes) in xml:
        raise RuntimeError("absolute mesh path left in the generated MJCF")
    return xml, model


def main() -> None:
    cfg = CellBuildConfig(_cli_parse_args=True)
    scene = load_scene(cfg.scene, check_files=False)
    if scene.urdf is None or scene.mujoco is None:
        raise ValueError(f"{cfg.scene}: the builder writes `urdf` and `mujoco.model`: name both")
    scene.urdf.write_text(build_urdf(scene, scene.urdf))
    xml, model = build_mjcf(scene, scene.mujoco.model, cfg)
    scene.mujoco.model.write_text(xml)
    print(f"wrote {scene.urdf} and {scene.mujoco.model}: nq={model.nq} nu={model.nu} ncam={model.ncam}")


if __name__ == "__main__":
    main()
