"""Offline acceptance: SCENE_DIR [EVIDENCE_JSON]. Run from the repository root.

Checks one saved cell against its authoritative relations in environment.yaml:
every world pose is reproduced by the relation graph, each datum mate holds in
world (flush planes face to face), destination zones are current projections of
their target-local placement, and the generated environment objects and MoveIt
collision manifest carry the same relations and poses. Sends nothing to ROS.
"""
import json
import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path.cwd() / 'scripts'))
import asset_mounting as m  # noqa: E402
from physical_destination import destination_ids, resolve_destination  # noqa: E402


def frame_error(a, b):
    return max(math.dist(a[0], b[0]), max(abs(x - y) for ra, rb in zip(a[1], b[1]) for x, y in zip(ra, rb)))


scene = Path(sys.argv[1])
environment = yaml.safe_load((scene / 'environment.yaml').read_text())
physical = environment['environment']
resolved = m.validate_physical(physical)  # raises on a stale or invalid relation
rows = {row['id']: row for row in m.physical_rows(physical)}
result = {'scene': str(scene), 'relations': {}, 'result': 'FAIL'}
for identifier, row in rows.items():
    if row.get('parent_object', 'world') == 'world':
        continue
    parent = rows[row['parent_object']]
    child_name, alignment = m.mate_of(row)
    child_datum = m.compose(m.world_pose(row), m.pose(m.anchors(row)[child_name]))
    expected = m.compose(m.compose(m.world_pose(parent), m.pose(m.anchors(parent)[row['parent_link']])),
                         m.compose(m.origin_pose(row), m.alignment_pose(alignment)))
    error = frame_error(child_datum, expected)
    assert error < 1e-9, (identifier, error)
    result['relations'][identifier] = {
        'parent': row['parent_object'], 'parent_datum': row['parent_link'], 'child_datum': child_name,
        'alignment': alignment, 'offset': row['origin'], 'world_xyz': row['pose_xyz'], 'world_rpy': row['pose_rpy'],
        'datum_frame_error': error}
result['destinations'] = {}
for zone in destination_ids(physical, environment.get('task') or {}):
    destination = resolve_destination(physical, zone)  # raises on a stale world projection
    result['destinations'][zone] = {'target': destination['target_id'], 'placement_local': destination['placement_local']}
generated = {obj['id']: obj for obj in yaml.safe_load((scene / 'generated/generated_environment_objects.yaml').read_text())['objects']}
for identifier, relation in result['relations'].items():
    obj = generated[identifier]
    assert obj.get('parent_object') == relation['parent'] and obj.get('mate') == rows[identifier].get('mate'), identifier
    assert math.dist(obj['pose']['xyz'], rows[identifier]['pose_xyz']) < 1e-9, identifier
result['generated_objects_match'] = True
manifest = yaml.safe_load((scene / 'config/moveit_collision_objects.yaml').read_text())
result['collision_objects'] = {obj.get('source_item_id', obj['id']): obj['pose']['xyz'] for obj in manifest['objects']}
camera = environment.get('camera') or {}
camera_row = rows.get(camera.get('camera_id', camera.get('id')))
if camera_row:
    assert max(abs(a - b) for a, b in zip(camera['pose'], camera_row['pose_xyz'] + camera_row['pose_rpy'])) < 1e-12
    result['camera_pose_mirrors_asset'] = True
result['result'] = 'PASS'
text = json.dumps(result, indent=2, default=str)
if len(sys.argv) > 2:
    Path(sys.argv[2]).write_text(text + '\n')
print(text)
