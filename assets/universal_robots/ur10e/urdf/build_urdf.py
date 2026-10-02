# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "pycollada",
#     "pydantic-settings>=2.6",
#     "robot-descriptions",
#     "trimesh",
#     "xacrodoc",
# ]
# ///
"""One-off generator for `ur10e_2f85.urdf`: a UR10e with a Robotiq 2F-85 on its flange.

Not a node and not imported by one. It builds the URDF every URDF consumer loads (the
rerun node's 3D robot, later IK), from the vendors' own descriptions, fetched and cached
by `robot_descriptions`:

* the UR10e: Universal Robots' `ur_description` xacro (BSD-3), rendered with `ur_type:=ur10e`;
* the 2F-85: `robotiq_arg85_description` (BSD), a plain URDF whose `finger_joint` drives the
  rest of the linkage through `<mimic>` joints.

It joins them with one fixed joint from the arm's `tool0` to the gripper's
`robotiq_85_base_link`, and makes the result self-contained: every mesh is copied next to
it under `meshes/` with a relative path, and UR's Collada visuals are converted to GLB
(rerun's Asset3D reads glTF/GLB, OBJ and STL, not Collada). Re-run it only when the
robot changes:

    uv run assets/universal_robots/ur10e/urdf/build_urdf.py

Checked against the MuJoCo model (`schemas/ur10e_2f85.xml`) when this was written: with the
URDF's root `base_link` rotated 180 degrees about z onto the MJCF base, both give the same
tool orientation for every joint vector (the joint zeros and directions agree), and the
flange 1.6-1.7 cm apart (the Menagerie UR10e has slightly different link lengths than UR's
own description). The gripper mount below puts the URDF fingertips within 2 mm of the MJCF
pads, open and closed.
"""

from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import urlparse

import trimesh
from pydantic_settings import BaseSettings, SettingsConfigDict
from robot_descriptions import robotiq_2f85_description, ur10e_description
from xacrodoc import XacroDoc, packages


class UrdfBuildConfig(BaseSettings):
    """How the gripper is mounted on the arm's `tool0` flange frame."""

    model_config = SettingsConfigDict(env_prefix="URDF_")

    # The coupling between flange and gripper base, along the tool axis (m).
    gripper_mount_z: float = 0.008
    # Rotation about the tool axis (rad): the fingers open along the flange's x axis, the
    # left finger on the same side as in the MuJoCo model.
    gripper_mount_yaw: float = 3.141592653589793


def render_ur10e() -> ET.Element:
    packages.update_package_cache({"ur_description": ur10e_description.PACKAGE_PATH})
    doc = XacroDoc.from_file(ur10e_description.XACRO_PATH, subargs=dict(ur10e_description.XACRO_ARGS))
    return ET.fromstring(doc.to_urdf_string())


def mesh_source(uri: str, package_dirs: dict[str, Path]) -> Path:
    """The file a URDF mesh URI points at: `file://` or `package://<name>/...`."""
    parsed = urlparse(uri)
    if parsed.scheme == "file":
        return Path(parsed.path)
    if parsed.scheme == "package":
        return package_dirs[parsed.netloc] / parsed.path.lstrip("/")
    raise ValueError(f"unsupported mesh uri {uri!r}")


def localize_meshes(robot: ET.Element, name: str, package_dirs: dict[str, Path], out: Path) -> None:
    """Copy every mesh of `robot` to `out/meshes/<name>/<visual|collision>/` and point the URDF
    at it relatively; Collada becomes GLB, and file extensions are lower-cased."""
    for kind in ("visual", "collision"):
        for element in robot.iter(kind):
            for mesh in element.iter("mesh"):
                source = mesh_source(mesh.get("filename"), package_dirs)
                suffix = ".glb" if source.suffix.lower() == ".dae" else source.suffix.lower()
                relative = Path("meshes") / name / kind / f"{source.stem}{suffix}"
                target = out / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                if suffix == ".glb":
                    trimesh.load(source, force="scene").export(target)
                else:
                    shutil.copyfile(source, target)
                mesh.set("filename", relative.as_posix())


def main() -> None:
    cfg = UrdfBuildConfig(_cli_parse_args=True)
    out = Path(__file__).resolve().parent
    shutil.rmtree(out / "meshes", ignore_errors=True)

    arm = render_ur10e()
    gripper = ET.parse(robotiq_2f85_description.URDF_PATH).getroot()
    localize_meshes(arm, "ur10e", {}, out)
    localize_meshes(
        gripper,
        "robotiq_2f85",
        {"robotiq_arg85_description": Path(robotiq_2f85_description.PACKAGE_PATH)},
        out,
    )

    robot = ET.Element("robot", name="ur10e_2f85")
    robot.extend(list(arm))
    robot.extend(list(gripper))
    mount = ET.SubElement(robot, "joint", name="tool0-robotiq_85_base_link", type="fixed")
    ET.SubElement(mount, "parent", link="tool0")
    ET.SubElement(mount, "child", link="robotiq_85_base_link")
    ET.SubElement(mount, "origin", xyz=f"0 0 {cfg.gripper_mount_z}", rpy=f"0 0 {cfg.gripper_mount_yaw}")

    ET.indent(robot)
    (out / "ur10e_2f85.urdf").write_text(ET.tostring(robot, encoding="unicode") + "\n")
    shutil.copyfile(Path(ur10e_description.PACKAGE_PATH) / "LICENSE", out / "LICENSE.ur_description")
    meshes = sorted(p.relative_to(out) for p in (out / "meshes").rglob("*") if p.is_file())
    print(f"wrote ur10e_2f85.urdf with {len(meshes)} meshes")


if __name__ == "__main__":
    main()
