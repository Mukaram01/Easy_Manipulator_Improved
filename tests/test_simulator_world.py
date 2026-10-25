"""Physics world derived from an authored cell and the qualified Stage-A world template."""
import copy
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import simulator_world as sw  # noqa: E402

REFERENCE_OBJECT = {'object_id': 'part_04', 'shape': 'box', 'dimensions': [0.025, 0.025, 0.025],
                    'pose': {'frame_id': 'world', 'xyz': [0.40, -0.20, 0.0125], 'rpy': [0, 0, 0.10]}}


def cell(tmp_path, obj=None, edit=None):
    """An authored cell as Create New Cell saves it: environment + manual test object + v2 task."""
    environment = yaml.safe_load((ROOT / 'scenes/ur5_2f_test/environment.yaml').read_text())
    doc = {'environment': environment, 'workcell_studio': {'pick_zone': {'object_source': {
        'mode': 'manual_simulated', 'manual_observation': copy.deepcopy(obj or REFERENCE_OBJECT)}}}}
    if edit:
        edit(doc)
    (tmp_path / 'config').mkdir(parents=True, exist_ok=True)
    (tmp_path / 'environment.yaml').write_text(yaml.safe_dump(doc))
    intent = {'schema': 'workcell_builder_task_intent/v2',
              'place': {'target': {'asset_ref': 'target_bin_default', 'region_ref': 'default_drop_zone'}}}
    (tmp_path / 'config/workcell_builder_task_intent.yaml').write_text(json.dumps(intent, indent=2))
    return tmp_path


def boxes(text, model):
    world = ET.fromstring(text).find('world')
    m = next(x for x in world.findall('model') if x.get('name') == model)
    return {c.get('name'): ([float(v) for v in (c.findtext('pose') or '0 0 0 0 0 0').split()][:3], [float(v) for v in c.findtext('geometry/box/size').split()])
            for c in m.iter('collision') if c.find('geometry/box') is not None}


def test_the_reference_cell_derives_the_qualified_world_model_for_model(tmp_path):
    text, report = sw.derive_world(cell(tmp_path))
    assert sw.equivalent_models(text, sw.TEMPLATE.read_text())
    assert report['destination']['floor_top_z'] == pytest.approx(0.1)
    assert report['objects'][0]['mass_kg'] == pytest.approx(0.020) and report['material']['spawn_clearance_m'] == pytest.approx(0.0003)
    template = ET.fromstring(sw.TEMPLATE.read_bytes()).find('world')
    derived = ET.fromstring(text).find('world')
    for tag in ('physics', 'gravity', 'scene'):                       # qualified world settings kept verbatim
        assert ET.tostring(derived.find(tag)) == ET.tostring(template.find(tag))
    assert [p.get('name') for p in derived.findall('plugin')] == [p.get('name') for p in template.findall('plugin')]


def test_authored_object_and_destination_edits_reach_the_physics_world(tmp_path):
    moved = copy.deepcopy(REFERENCE_OBJECT); moved['pose']['xyz'] = [0.42, -0.18, 0.015]; moved['dimensions'] = [0.03, 0.02, 0.03]
    def move_bin(doc):
        target = next(a for a in doc['environment']['assets'] if a['id'] == 'target_bin_default')
        target['pose_xyz'] = [0.50, 0.25, 0.1]
    text, report = sw.derive_world(cell(tmp_path, moved, move_bin))
    assert not sw.equivalent_models(text, sw.TEMPLATE.read_text())
    bin_boxes = boxes(text, 'target_bin_physics')
    floor_xyz, floor_size = bin_boxes['floor']
    assert floor_xyz[:2] == pytest.approx([0.50, 0.25]) and floor_xyz[2] + floor_size[2] / 2 == pytest.approx(0.1)
    assert floor_size == pytest.approx([0.26, 0.14, 0.01])
    assert bin_boxes['wall_x_plus'][0][0] - bin_boxes['wall_x_minus'][0][0] == pytest.approx(0.24 + 0.01)
    part = boxes(text, 'part_04')['collision']
    assert part[1] == pytest.approx([0.03, 0.02, 0.03])
    obj = report['objects'][0]
    assert obj['pose'][:3] == pytest.approx([0.42, -0.18, 0.0153]) and obj['mass_kg'] == pytest.approx(1280 * 0.03 * 0.02 * 0.03)


@pytest.mark.parametrize('change,message', [
    (lambda obj, doc: obj.update(shape='cylinder'), 'only box'),
    (lambda obj, doc: obj['pose'].update(xyz=[0.40, -0.20, 0.05]), 'not resting'),
    (lambda obj, doc: obj['pose'].update(rpy=[0.3, 0, 0]), 'resting flat'),
    (lambda obj, doc: obj['pose'].update(frame_id='tool0'), 'world frame'),
    (lambda obj, doc: obj.update(object_id='bad name'), 'valid model name'),
    (lambda obj, doc: next(a for a in doc['environment']['assets'] if a['id'] == 'target_bin_default').update(pose_rpy=[0, 0, 0.5]), 'axis-aligned'),
    (lambda obj, doc: next(s for s in doc['environment']['support_surfaces']).update(surface_z_m=0.2), 'support plane'),
    (lambda obj, doc: doc['workcell_studio']['pick_zone']['object_source'].update(mode='live_perception'), 'manual test object'),
])
def test_cells_the_template_cannot_represent_fail_closed(tmp_path, change, message):
    obj = copy.deepcopy(REFERENCE_OBJECT)
    def edit(doc):
        change(doc['workcell_studio']['pick_zone']['object_source']['manual_observation'], doc)
    with pytest.raises(sw.SimulatorWorldError, match=message):
        sw.derive_world(cell(tmp_path, obj, edit))


def test_cli_writes_world_and_report_or_blocks(tmp_path):
    scene = cell(tmp_path / 'cell')
    world, report = tmp_path / 'world.sdf', tmp_path / 'report.json'
    assert sw.main([str(scene), '--output', str(world), '--report', str(report)]) == 0
    data = json.loads(report.read_text())
    assert data['equivalent_to_qualified_template'] is True and world.read_text().startswith('<sdf')
    (scene / 'config/workcell_builder_task_intent.yaml').write_text('{"schema": "workcell_builder_task_intent/v1"}')
    assert sw.main([str(scene), '--output', str(tmp_path / 'other.sdf')]) == 2 and not (tmp_path / 'other.sdf').exists()
