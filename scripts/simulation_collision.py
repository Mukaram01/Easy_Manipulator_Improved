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
triangle.

Only the physics model Gazebo spawns (robot.sdf) changes. Planning keeps the exact mesh:
with two shapes per fingertip, the controller-interval certificate loses its
single-shape triangle-cover bound against the concave bin mesh and could no longer
certify the place descent (fingertips inside the bin), and grasp qualification is defined
on the planning geometry. The boxes are a superset of the mesh, so any extra physical
contact is still caught by the unpermitted-contact guard. The fake-hardware path is
unchanged.

``contact_surface_layer_m`` gives the pad boxes an authored contact surface layer (SDF
``<surface><contact><ode><min_depth>``). The physics overlay applies a pair's layer (the smaller of
its two surfaces') as an uncorrected depth: the rigid pinch otherwise sits on ~2 um of
penetration per pad, and a few um of in-grip drift during transfer separated a pad (LENLEG
2026-10-25, faithful replay in simulator_physics_repro/mechanism_replay.cpp). Pairs with a
surface that authors no layer, e.g. pad and bin wall, stay rigid.

Capability record block (``end_effector.simulator_collision``):

    simulator_collision:
      mode: mesh_slab_bounding_boxes      # or mesh_bounding_box (no cuts)
      axis: x                             # slab axis in the collision frame
      cuts_m: [0.0069]                    # strictly increasing cut planes
      links: [gripper_finger1_finger_tip_link, gripper_finger2_finger_tip_link]
      contact_surface_layer_m: 0.0001     # optional, (0, 0.001] m
      evidence: "..."
"""
from __future__ import annotations

import copy
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
MAX_SURFACE_LAYER_M = 1e-3       # the physics overlay rejects larger authored layers


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
    layer = block.get('contact_surface_layer_m')
    if layer is not None and (not isinstance(layer, (int, float)) or isinstance(layer, bool) or
                              not math.isfinite(layer) or not 0 < layer <= MAX_SURFACE_LAYER_M):
        errors.append(f"Field 'simulator_collision.contact_surface_layer_m' must be in (0, {MAX_SURFACE_LAYER_M}] meters.")
    return errors


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


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


def _child(parent, tag):
    found = parent.find(tag)
    return ET.SubElement(parent, tag) if found is None else found


def _pose(text):
    values = [float(v) for v in (text or '0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise SimulationCollisionError('collision pose must be six finite numbers')
    return np.array(values[:3]), values[3:]


def bind_native_model(output, end_effector):
    """Bind a capability record's ``simulator_collision`` block into the derived physics model.

    Only ``<output>/robot.sdf`` (the model Gazebo spawns) changes: each listed link's single
    mesh collision becomes its slab bounding boxes, keeping the collision's surface
    (friction/contact) parameters. The MoveIt/planning description (``robot.urdf`` and
    ``robot_description``) keeps the original mesh, so planning, the controller-interval
    certificate and grasp qualification are unchanged. ``spec.json`` gets the binding and the
    new ``native_description_sha256`` that the receipt check verifies. None when the record
    declares no block.
    """
    errors = block_errors(end_effector)
    if errors:
        raise SimulationCollisionError(' '.join(errors))
    block = end_effector.get('simulator_collision')
    if block is None:
        return None
    output = Path(output); model_path = output / 'robot.sdf'
    if not model_path.is_file():
        raise SimulationCollisionError('SIMULATOR_COLLISION_REQUIRES_NATIVE_MODEL: no derived robot.sdf to bind')
    original = model_path.read_bytes()
    root = ET.fromstring(original)
    links = {link.get('name'): link for link in root.iter('link')}
    axis, cuts = block.get('axis'), tuple(block.get('cuts_m') or ())
    layer = block.get('contact_surface_layer_m')
    records = []
    for name in block['links']:
        link = links.get(name)
        collisions = link.findall('collision') if link is not None else []
        if len(collisions) != 1:
            raise SimulationCollisionError(f'{name}: exactly one collision element required, found {len(collisions)}')
        collision = collisions[0]
        geometry = collision.find('geometry'); mesh = geometry.find('mesh') if geometry is not None else None
        if mesh is None or not mesh.findtext('uri'):
            raise SimulationCollisionError(f'{name}: collision is not a mesh')
        if collision.find('pose') is not None and collision.find('pose').get('relative_to'):
            raise SimulationCollisionError(f'{name}: relative_to collision poses are unsupported')
        path = Path(mesh.findtext('uri').removeprefix('file://')).resolve(strict=True)
        scale = np.array([float(v) for v in (mesh.findtext('scale') or '1 1 1').split()])
        if scale.shape != (3,) or not np.isfinite(scale).all() or (scale <= 0).any():
            raise SimulationCollisionError(f'{name}: invalid mesh scale')
        triangles, count, sha256 = _binary_stl_triangles(path)
        try:
            boxes = slab_boxes(triangles * scale, axis, cuts)
        except SimulationCollisionError as exc:
            raise SimulationCollisionError(f'{name}: {exc}') from exc
        if any(((high - low) <= 0).any() for low, high in boxes):
            raise SimulationCollisionError(f'{name}: degenerate mesh extent')
        xyz, rpy = _pose(collision.findtext('pose'))
        position = list(link).index(collision)
        link.remove(collision)
        shapes = []
        for index, (low, high) in enumerate(boxes):
            size, centre = high - low, (high + low) / 2
            shifted = xyz + _rpy(*rpy) @ centre
            element = copy.deepcopy(collision)
            element.set('name', f"{collision.get('name', name)}_box{index}")
            for old in element.findall('pose'):
                element.remove(old)
            pose = ET.Element('pose'); pose.text = ' '.join(repr(float(v)) for v in list(shifted) + list(rpy))
            element.insert(0, pose)
            box_geometry = element.find('geometry')
            for old in list(box_geometry):
                box_geometry.remove(old)
            ET.SubElement(ET.SubElement(box_geometry, 'box'), 'size').text = ' '.join(repr(float(v)) for v in size)
            if layer is not None:
                contact = _child(_child(_child(element, 'surface'), 'contact'), 'ode')
                for old in contact.findall('min_depth'):
                    contact.remove(old)
                ET.SubElement(contact, 'min_depth').text = repr(float(layer))
            link.insert(position + index, element)
            shapes.append(dict(box_size_m=size.tolist(), box_centre_m=centre.tolist(), pose=list(shifted) + list(rpy)))
        records.append(dict(link=name, mesh=str(path), mesh_sha256=sha256, triangles=count, scale=scale.tolist(), boxes=shapes))
    model_path.write_text(ET.tostring(root, encoding='unicode'))
    digest = lambda data: hashlib.sha256(data).hexdigest()
    binding = dict(mode=block['mode'], axis=axis, cuts_m=list(cuts), contact_surface_layer_m=layer, links=records, evidence=block['evidence'],
                   source_native_description_sha256=digest(original),
                   native_description_sha256=digest(model_path.read_bytes()))
    spec_path = output / 'spec.json'
    spec = json.loads(spec_path.read_text())
    if spec.get('native_description_sha256') != binding['source_native_description_sha256']:
        model_path.write_bytes(original)
        raise SimulationCollisionError('SIMULATOR_COLLISION_SOURCE_CHANGED: robot.sdf differs from the prepared spec')
    spec['native_description_sha256'] = binding['native_description_sha256']
    spec['simulator_collision_primitives'] = binding
    spec_path.write_text(json.dumps(spec, indent=2))
    return binding
