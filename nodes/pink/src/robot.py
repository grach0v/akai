"""The robot on its URDF, loaded by Pinocchio: joint limits and collision geometry (Coal).

* Its configuration is the actuated joints (`names`, Pinocchio's order). Mimic joints, like the
  gripper's linkage, follow the joint they mimic, as the URDF declares.
* Collision meshes are replaced by their convex hulls: distances get ~400x faster (12 ms to
  0.03 ms per configuration on the UR10e) and smooth enough for pink's collision barrier. The
  UR meshes are nearly convex, so the hulls agree with them to the millimetre.
* Self-collision is checked between every pair of collision geometries except those on one joint
  or on neighbouring joints, which touch by design. A mimic joint counts as the joint it follows,
  so a linkage never collides with itself.
* Whatever else the URDF holds is collision geometry too: a cell's URDF (assets/cells/) has its
  table and floor fixed to `world`, like the robot's base, so every link not neighbouring the
  base must stay clear of them.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import coal
import numpy as np
import pinocchio as pin


class Robot:
    def __init__(self, urdf: Path):
        self.model, self.geom = pin.buildModelsFromUrdf(
            str(urdf),
            package_dirs=[str(urdf.parent)],
            geometry_types=[pin.GeometryType.COLLISION],
            mimic=True,
        )
        model = self.model
        mimic_of = {
            j.get("name"): j.find("mimic").get("joint")
            for j in ET.parse(urdf).getroot().findall("joint")
            if j.find("mimic") is not None
        }
        actuated = [j for j in range(1, model.njoints) if model.names[j] not in mimic_of]
        self.names = [model.names[j] for j in sorted(actuated, key=lambda j: model.joints[j].idx_q)]
        if len(self.names) != model.nq:
            raise ValueError(f"{urdf.name}: every actuated joint must have one coordinate")
        self.lower, self.upper = model.lowerPositionLimit.copy(), model.upperPositionLimit.copy()
        self.velocity = model.velocityLimit.copy()

        for geometry in self.geom.geometryObjects:
            if isinstance(geometry.geometry, coal.BVHModelBase):
                geometry.geometry.buildConvexRepresentation(False)
                geometry.geometry = geometry.geometry.convex

        def group(geometry: int) -> int:
            joint = self.geom.geometryObjects[geometry].parentJoint
            name = model.names[joint]
            return model.getJointId(mimic_of[name]) if name in mimic_of else joint

        self.geom.addAllCollisionPairs()
        checked = []
        for pair in self.geom.collisionPairs:
            a, b = group(pair.first), group(pair.second)
            if a != b and model.parents[a] != b and model.parents[b] != a:
                checked.append(pair)
        self.geom.removeAllCollisionPairs()
        for pair in checked:
            self.geom.addCollisionPair(pair)

        self._data = model.createData()
        self._geom_data = pin.GeometryData(self.geom)

    def collisions(self, q: np.ndarray) -> list[tuple[str, str]]:
        """The pairs of geometries in collision at `q` (empty: collision-free)."""
        pin.computeCollisions(self.model, self._data, self.geom, self._geom_data, q, False)
        objects = self.geom.geometryObjects
        return [
            (objects[pair.first].name, objects[pair.second].name)
            for pair, result in zip(self.geom.collisionPairs, self._geom_data.collisionResults)
            if result.isCollision()
        ]
