"""The 3D robot of the `record` mode: a URDF, moved by joint states (rerun.urdf.UrdfTree).

Follows the `rerun-urdf` skill:

* the static model (visual meshes, and every joint's transform at rest) is the URDF's own
  chunk stream, `UrdfTree.stream()`, minus the collision geometry. Being static and the same
  for every recording, it is written once as the dataset asset `<urdf name>.model.rrd` and
  sent to the live viewer; recordings do not repeat it;
* forward kinematics is `UrdfTree.compute_joint_transform_batches`: each joint state becomes
  one `Transform3D` row per joint it moves (parent and child link frames named by the URDF),
  all on `robot/transforms`, at the state's times (the timelines its scalars are logged on).
  The transforms are derived data,
  so the recorder keeps them in their own layer (`<recording>.fk.rrd`).

Joint states carry URDF joint names (metadata `names`), so the mapping from a producer's
joints to the URDF is the producer's (the sim's MJCF actuators already use URDF names
and units). Mimic joints, which a gripper reports only through its driver, get their
value here as the URDF says: `driver * multiplier + offset`.
"""

from __future__ import annotations

import logging
from pathlib import Path

import pyarrow as pa
import rerun as rr
from rerun.chunk import LazyChunkStream
from rerun.urdf import UrdfTree

logger = logging.getLogger("rerun")


class RobotUrdf:
    def __init__(self, path: Path, root: str = "robot"):
        self.path = path
        self.root = root
        self.transforms_entity = f"{root}/transforms"
        self.tree = UrdfTree.from_file_path(
            path, entity_path_prefix=root, static_transform_entity_path=f"{root}/tf_static"
        )
        movable = [j for j in self.tree.joints() if j.joint_type != "fixed"]
        self._drivers = {j.name for j in movable if j.mimic is None}
        self._mimics: dict[str, list[tuple[str, float, float]]] = {}  # driver -> (mimic, multiplier, offset)
        for j in movable:
            if j.mimic is not None:
                self._mimics.setdefault(j.mimic.joint, []).append(
                    (j.name, j.mimic.multiplier, j.mimic.offset)
                )
        self._unknown: set[str] = set()
        logger.info(
            "robot: %s, joints %s, mimic joints %d",
            path,
            sorted(self._drivers),
            sum(len(m) for m in self._mimics.values()),
        )

    def model(self) -> LazyChunkStream:
        """The static model: visual meshes and every joint transform at rest (a fresh stream per call)."""
        # A `**` only matches at the end of a content path, so name the collision subtree fully.
        return self.tree.stream(include_joint_transforms=True).drop(
            content=f"/{self.root}/{self.tree.name}/collision_geometries/**"
        )

    def transforms(
        self, names: list[str], values: list[float], timestamps: dict[str, float], sequences: dict[str, int]
    ) -> tuple[list[rr.TimeColumn], rr.ComponentColumnList] | None:
        """(indexes, columns) for `send_columns` on `transforms_entity`: the joints this state
        names, and the mimic joints they drive, at the joint state's times (timeline -> seconds
        since the epoch, and timeline -> index). None when it names none."""
        joint_names, joint_values = [], []
        for name, value in zip(names, values):
            if name not in self._drivers:
                if name not in self._unknown:
                    self._unknown.add(name)
                    logger.warning("robot: %s has no movable joint %r; it is not shown", self.path.name, name)
                continue
            joint_names.append(name)
            joint_values.append(value)
            for mimic, multiplier, offset in self._mimics.get(name, []):
                joint_names.append(mimic)
                joint_values.append(value * multiplier + offset)
        if not joint_names:
            return None
        (batch,) = self.tree.compute_joint_transform_batches(
            pa.array([joint_names], type=pa.list_(pa.utf8())),
            pa.array([joint_values], type=pa.list_(pa.float64())),
        ).to_pylist()
        columns = rr.Transform3D.columns(
            translation=[t["translation"] for t in batch],
            quaternion=[t["quaternion"] for t in batch],
            parent_frame=[t["parent_frame"] for t in batch],
            child_frame=[t["child_frame"] for t in batch],
        )
        rows = len(batch)
        indexes = [rr.TimeColumn(t, timestamp=[seconds] * rows) for t, seconds in timestamps.items()]
        indexes += [rr.TimeColumn(t, sequence=[index] * rows) for t, index in sequences.items()]
        return indexes, columns
