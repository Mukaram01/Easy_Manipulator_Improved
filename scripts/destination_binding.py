#!/usr/bin/env python3
"""Bind the physics destination (bin) floor to the planning destination object.

Applied by the simulator launch after ``simulator_backend.prepare`` (whose file is bound
by the reviewed shutdown-lifecycle evidence and therefore left unchanged). The planning
scene carries the authored bin mesh whose local z=0 is its floor top; the physics world
carries a static box floor. The binding records the one floor height both contact
checkers use and refuses a world whose physics floor does not sit at the authored
destination origin, so a placed part cannot rest below the planning floor
(stage_a0_sparse before 2026-10-20: 5 mm low).
"""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from simulator_backend import digest


def bind_destination_geometry(xml, manifest_path):
    import yaml
    root = ET.fromstring(xml)
    models = root.findall("world/model[@name='target_bin_physics']")
    if not models:
        return None
    if len(models) != 1 or manifest_path is None:
        raise ValueError('destination requires one bin model and a collision manifest')
    try:
        path = Path(manifest_path).resolve(); raw = path.read_bytes()
        manifest = yaml.safe_load(raw)
        if (not isinstance(manifest, dict) or not isinstance(manifest.get('objects'), list) or
                any(not isinstance(item, dict) for item in manifest['objects'])):
            raise ValueError('invalid destination manifest objects')
        bins = [item for item in manifest['objects'] if item.get('semantic_role') == 'target_bin']
        if len(bins) != 1:
            raise ValueError('ambiguous destination objects')
        item = bins[0]; pose = item['pose']
        if item['frame_id'] != 'world' or item['operation'] != 'ADD' or not isinstance(item['id'], str) or not item['id']:
            raise ValueError('unsupported destination identity/frame')
        xyz = [float(v) for v in pose['xyz']]
        quaternion = [float(v) for v in pose['quaternion_xyzw']]
        if (len(xyz) != 3 or len(quaternion) != 4 or any(abs(v) > 1e-9 for v in quaternion[:3]) or
                abs(abs(quaternion[3]) - 1.) > 1e-9):
            raise ValueError('destination must be axis-aligned in the world frame')
        model = models[0]; links = model.findall('link')
        if (model.findtext('static') != 'true' or len(links) != 1 or links[0].get('name') != 'bin_link' or
                model.find('pose') is not None):
            raise ValueError('unsupported destination model/link')
        link = links[0]; floors = link.findall("collision[@name='floor']")
        if len(floors) != 1 or link.find('pose') is not None:
            raise ValueError('ambiguous destination floor collision')
        floor = floors[0]; pose_element = floor.find('pose')
        if pose_element is not None and pose_element.attrib:
            raise ValueError('destination floor pose must be expressed in the world frame without attributes')
        fp = [float(v) for v in (floor.findtext('pose') or '0 0 0 0 0 0').split()]
        size = [float(v) for v in floor.findtext('geometry/box/size', '').split()]
        if len(fp) != 6 or any(abs(v) > 1e-12 for v in fp[3:]) or len(size) != 3 or min(size) <= 0:
            raise ValueError('destination floor must be an axis-aligned box')
        floor_z = fp[2] + size[2] / 2
        if math.dist(fp[:2], xyz[:2]) > 1e-6 or abs(floor_z - xyz[2]) > 1e-6:
            raise ValueError('physics destination floor top %.6f/(%.4f,%.4f) differs from authored destination origin %.6f/(%.4f,%.4f)'
                             % (floor_z, fp[0], fp[1], xyz[2], xyz[0], xyz[1]))
        return dict(manifest_path=str(path), manifest_sha256=digest(raw), object_id=item['id'],
                    model='target_bin_physics', link='bin_link', collision='floor', floor_z=floor_z,
                    floor_xyz=fp[:3], floor_size_m=size, pose_xyz=xyz)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ValueError('destination geometry binding rejected: ' + str(exc)) from exc


def augment_spec(output, manifest_path):
    """Record the destination binding in the prepared simulator spec (read by the receipt)."""
    output = Path(output)
    binding = bind_destination_geometry((output / 'world.sdf').read_text(), manifest_path)
    spec_path = output / 'spec.json'
    spec = json.loads(spec_path.read_text())
    spec['destination_geometry_binding'] = binding
    spec_path.write_text(json.dumps(spec, indent=2))
    return binding
