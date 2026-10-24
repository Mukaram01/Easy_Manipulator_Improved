"""Primitive contact geometry for the physics simulator, derived from the tool's own collision meshes.

Fingertip pads modelled as a concave triangle mesh (Robotiq 2F-85: 2436 triangles,
sliver facets down to 7e-9 m^2) make DART+Bullet rebuild the pad contact manifold from
changing triangle pairs. A standalone replay of the recorded Stage-A transfer (same
part, grasp, squeeze and trajectory; no ROS, controller or MoveIt; LENLEG 2026-10-24)
showed pad force collapsing to ~0 N and in-grip rotation of 2.6-104 mrad at every
speed (0.25x-2x) and force (20/90 N) with the mesh, and 0.0013-0.0019 mrad with no
collapse when the same pad is the mesh's bounding box. A convex hull is still a
triangle mesh here and failed at 0.5x (30 mrad).

The bounding box is computed from the mesh's own vertices in the collision frame, so
every face lies on an extreme plane of the real geometry: it never reaches beyond the
tool in any axis and only fills concavities (conservative for collision checking),
and the flat pad face is preserved exactly. Binding fails closed unless the box
encloses every vertex. Only the simulator derivation uses it; the fake-hardware
description is unchanged.

Capability record block (``end_effector.simulator_collision``):

    simulator_collision:
      mode: mesh_bounding_box
      links: [gripper_finger1_finger_tip_link, gripper_finger2_finger_tip_link]
      evidence: "..."
"""
from __future__ import annotations

import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

MODES = ('mesh_bounding_box',)
ENCLOSURE_TOLERANCE_M = 1e-9


class SimulationCollisionError(ValueError):
    """The declared simulator collision primitives cannot be bound (fail closed)."""


def block_errors(end_effector):
    block = end_effector.get('simulator_collision')
    if block is None:
        return []
    if not isinstance(block, dict):
        return ["Field 'end_effector.simulator_collision' must be a mapping/object."]
    errors = []
    if block.get('mode') not in MODES:
        errors.append(f"Field 'simulator_collision.mode' must be one of {list(MODES)}.")
    links = block.get('links')
    if not isinstance(links, list) or not links or not all(isinstance(v, str) and v for v in links) or len(set(links)) != len(links):
        errors.append("Field 'simulator_collision.links' must be a non-empty list of distinct link names.")
    if not isinstance(block.get('evidence'), str) or not block['evidence'].strip():
        errors.append("Field 'simulator_collision.evidence' must name the measurement that justifies it.")
    return errors


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def _resolve(uri, package_share):
    if uri.startswith('package://'):
        package, relative = uri[10:].split('/', 1)
        return Path(package_share(package)) / relative
    return Path(uri.removeprefix('file://'))


def _binary_stl_vertices(path):
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise SimulationCollisionError(f'{path}: binary STL required')
    count = struct.unpack_from('<I', data, 80)[0]
    if not count or len(data) != 84 + 50 * count:
        raise SimulationCollisionError(f'{path}: binary STL required')
    vertices = np.array([struct.unpack_from('<12fH', data, 84 + 50 * i)[3:12] for i in range(count)], dtype=float).reshape(-1, 3)
    if not np.isfinite(vertices).all():
        raise SimulationCollisionError(f'{path}: nonfinite vertices')
    return vertices, count, hashlib.sha256(data).hexdigest()


def bind_mesh_bounding_boxes(xml, links, *, package_share=None, evidence=None):
    """Replace each listed link's single mesh collision with the mesh's bounding box.

    Returns the bound description and the binding record. The box is expressed in the
    collision frame (the URDF collision origin keeps its rotation; its translation
    moves to the box centre).
    """
    if package_share is None:
        from ament_index_python.packages import get_package_share_directory as package_share
    root = ET.fromstring(xml)
    by_name = {link.get('name'): link for link in root.findall('link')}
    records = []
    for name in links:
        link = by_name.get(name)
        collisions = link.findall('collision') if link is not None else []
        if len(collisions) != 1:
            raise SimulationCollisionError(f'{name}: exactly one collision element required, found {len(collisions)}')
        collision = collisions[0]
        geometry = collision.find('geometry'); mesh = geometry.find('mesh') if geometry is not None else None
        if mesh is None:
            raise SimulationCollisionError(f'{name}: collision is not a mesh')
        path = _resolve(mesh.get('filename', ''), package_share).resolve(strict=True)
        scale = np.array([float(v) for v in mesh.get('scale', '1 1 1').split()])
        if scale.shape != (3,) or not np.isfinite(scale).all() or (scale <= 0).any():
            raise SimulationCollisionError(f'{name}: invalid mesh scale')
        vertices, triangles, sha256 = _binary_stl_vertices(path)
        vertices = vertices * scale
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        size, centre = high - low, (high + low) / 2
        if (size <= 0).any():
            raise SimulationCollisionError(f'{name}: degenerate mesh extent')
        if ((vertices < low - ENCLOSURE_TOLERANCE_M) | (vertices > high + ENCLOSURE_TOLERANCE_M)).any():
            raise SimulationCollisionError(f'{name}: bounding box does not enclose the mesh')
        origin = collision.find('origin')
        xyz = np.array([float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()])
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        shifted = xyz + _rpy(*rpy) @ centre
        if origin is None:
            origin = ET.SubElement(collision, 'origin')
        origin.set('xyz', ' '.join(repr(float(v)) for v in shifted)); origin.set('rpy', ' '.join(repr(v) for v in rpy))
        geometry.remove(mesh)
        ET.SubElement(geometry, 'box', size=' '.join(repr(float(v)) for v in size))
        records.append(dict(link=name, mesh=str(path), mesh_sha256=sha256, triangles=triangles, scale=scale.tolist(),
                            box_size_m=size.tolist(), box_centre_m=centre.tolist(), origin_xyz_m=shifted.tolist(),
                            origin_rpy_rad=rpy))
    return ET.tostring(root, encoding='unicode'), dict(mode='mesh_bounding_box', links=records, evidence=evidence)


def bind_from_capability(xml, end_effector):
    """Apply a capability record's ``simulator_collision`` block; None when it declares none."""
    errors = block_errors(end_effector)
    if errors:
        raise SimulationCollisionError(' '.join(errors))
    block = end_effector.get('simulator_collision')
    if block is None:
        return xml, None
    return bind_mesh_bounding_boxes(xml, block['links'], evidence=block['evidence'])


def augment_spec(output, binding):
    """Record the binding in the prepared simulator spec (read by the receipt)."""
    spec_path = Path(output) / 'spec.json'
    spec = json.loads(spec_path.read_text())
    spec['simulator_collision_primitives'] = binding
    spec_path.write_text(json.dumps(spec, indent=2))
    return binding
