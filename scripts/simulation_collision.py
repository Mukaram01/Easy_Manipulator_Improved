"""Primitive contact geometry for the physics simulator, derived from the tool's own collision meshes.

Fingertip pads modelled as a concave triangle mesh (Robotiq 2F-85: 2436 triangles,
sliver facets down to 7e-9 m^2) make DART+Bullet rebuild the pad contact manifold from
changing triangle pairs. A standalone replay of the recorded Stage-A transfer (same
part, grasp, squeeze and trajectory; no ROS, controller or MoveIt; LENLEG 2026-10-24)
showed pad force collapsing to ~0 N and in-grip rotation of 2.6-104 mrad at every
speed (0.25x-2x) and force (20/90 N) with the mesh, and 0.0013-0.0019 mrad with no
collapse when the same pad is the mesh's bounding box. A convex hull is still a
triangle mesh here and failed at 0.5x (30 mrad).

One bounding box over the whole fingertip over-represents the pad: the real flat pad
face spans x 7.0-44.5 mm (814 mm^2) but the whole-tip box makes all 57 mm flush with it,
and grasp qualification rightly rejected it (rotated-part penetration 1.014 mm > 1 mm).
``mesh_slab_bounding_boxes`` therefore cuts the mesh into slabs along one axis: every
triangle goes wholly to the slab holding its centroid and each slab becomes the
bounding box of its own triangles. The union encloses every triangle by construction,
every face lies on an extreme plane of real geometry (never beyond the tool,
conservative for collision checking), and with a cut just before the pad the distal box
face is the pad face while the proximal box stays 1.8 mm behind it. The same replay held
0.0016-0.0022 mrad with no collapse. Binding fails closed unless the boxes enclose every
triangle. Only the simulator derivation uses it; the fake-hardware description is
unchanged.

Capability record block (``end_effector.simulator_collision``):

    simulator_collision:
      mode: mesh_slab_bounding_boxes      # or mesh_bounding_box (no cuts)
      axis: x                             # slab axis in the collision frame
      cuts_m: [0.0069]                    # strictly increasing cut planes
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

MODES = ('mesh_bounding_box', 'mesh_slab_bounding_boxes')
AXES = {'x': 0, 'y': 1, 'z': 2}
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
    if block.get('mode') == 'mesh_slab_bounding_boxes':
        cuts = block.get('cuts_m')
        if block.get('axis') not in AXES:
            errors.append("Field 'simulator_collision.axis' must be x, y or z.")
        if (not isinstance(cuts, list) or not cuts or
                not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in cuts) or
                any(b <= a for a, b in zip(cuts, cuts[1:]))):
            errors.append("Field 'simulator_collision.cuts_m' must be strictly increasing finite meters.")
    elif block.get('mode') == 'mesh_bounding_box' and ('cuts_m' in block or 'axis' in block):
        errors.append("mesh_bounding_box takes no axis or cuts_m.")
    return errors


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def _resolve(uri, package_share):
    if uri.startswith('package://'):
        package, relative = uri[10:].split('/', 1)
        return Path(package_share(package)) / relative
    return Path(uri.removeprefix('file://'))


def _binary_stl_triangles(path):
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise SimulationCollisionError(f'{path}: binary STL required')
    count = struct.unpack_from('<I', data, 80)[0]
    if not count or len(data) != 84 + 50 * count:
        raise SimulationCollisionError(f'{path}: binary STL required')
    triangles = np.array([struct.unpack_from('<12fH', data, 84 + 50 * i)[3:12] for i in range(count)], dtype=float).reshape(-1, 3, 3)
    if not np.isfinite(triangles).all():
        raise SimulationCollisionError(f'{path}: nonfinite vertices')
    return triangles, count, hashlib.sha256(data).hexdigest()


def slab_boxes(triangles, axis=None, cuts=()):
    """Bounding boxes of centroid-assigned slabs (one box when no cuts); each as (low, high)."""
    if axis is None or not cuts:
        groups = [triangles]
    else:
        centroid = triangles.mean(axis=1)[:, AXES[axis]]
        slab = np.searchsorted(np.asarray(cuts, dtype=float), centroid, side='right')
        groups = [triangles[slab == k] for k in range(len(cuts) + 1)]
        groups = [g for g in groups if len(g)]
    boxes = []
    for group in groups:
        points = group.reshape(-1, 3); boxes.append((points.min(axis=0), points.max(axis=0)))
    # Enclosure: every triangle must lie inside at least one box (by construction, its own slab's).
    for tri in triangles:
        if not any(((tri >= lo - ENCLOSURE_TOLERANCE_M) & (tri <= hi + ENCLOSURE_TOLERANCE_M)).all() for lo, hi in boxes):
            raise SimulationCollisionError('slab boxes do not enclose the mesh')
    return boxes


def bind_mesh_bounding_boxes(xml, links, *, package_share=None, evidence=None, axis=None, cuts=()):
    """Replace each listed link's single mesh collision with slab bounding boxes of that mesh.

    Returns the bound description and the binding record. Boxes are expressed in the
    collision frame (each collision origin keeps its rotation; its translation moves to
    the box centre). Without cuts, a single bounding box.
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
        triangles, count, sha256 = _binary_stl_triangles(path)
        triangles = triangles * scale
        try:
            boxes = slab_boxes(triangles, axis, cuts)
        except SimulationCollisionError as exc:
            raise SimulationCollisionError(f'{name}: {exc}') from exc
        if any(((high - low) <= 0).any() for low, high in boxes):
            raise SimulationCollisionError(f'{name}: degenerate mesh extent')
        origin = collision.find('origin')
        xyz = np.array([float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()])
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        position = list(link).index(collision)
        link.remove(collision)
        shapes = []
        for index, (low, high) in enumerate(boxes):
            size, centre = high - low, (high + low) / 2
            shifted = xyz + _rpy(*rpy) @ centre
            element = ET.Element('collision', {**collision.attrib, **({'name': f"{collision.get('name')}_{index}"} if collision.get('name') else {})})
            ET.SubElement(element, 'origin', xyz=' '.join(repr(float(v)) for v in shifted), rpy=' '.join(repr(v) for v in rpy))
            ET.SubElement(ET.SubElement(element, 'geometry'), 'box', size=' '.join(repr(float(v)) for v in size))
            link.insert(position + index, element)
            shapes.append(dict(box_size_m=size.tolist(), box_centre_m=centre.tolist(), origin_xyz_m=shifted.tolist()))
        records.append(dict(link=name, mesh=str(path), mesh_sha256=sha256, triangles=count, scale=scale.tolist(),
                            origin_rpy_rad=rpy, boxes=shapes))
    mode = 'mesh_slab_bounding_boxes' if axis is not None and cuts else 'mesh_bounding_box'
    return ET.tostring(root, encoding='unicode'), dict(mode=mode, axis=axis, cuts_m=list(cuts), links=records, evidence=evidence)


def bind_from_capability(xml, end_effector):
    """Apply a capability record's ``simulator_collision`` block; None when it declares none."""
    errors = block_errors(end_effector)
    if errors:
        raise SimulationCollisionError(' '.join(errors))
    block = end_effector.get('simulator_collision')
    if block is None:
        return xml, None
    return bind_mesh_bounding_boxes(xml, block['links'], evidence=block['evidence'],
                                    axis=block.get('axis'), cuts=tuple(block.get('cuts_m') or ()))


def augment_spec(output, binding):
    """Record the binding in the prepared simulator spec (read by the receipt)."""
    spec_path = Path(output) / 'spec.json'
    spec = json.loads(spec_path.read_text())
    spec['simulator_collision_primitives'] = binding
    spec_path.write_text(json.dumps(spec, indent=2))
    return binding
