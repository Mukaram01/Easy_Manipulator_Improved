"""The physics world is composed from each authored cell's own saved and generated geometry."""
import copy
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import simulator_world as sw  # noqa: E402

pytestmark = pytest.mark.skipif(not Path('/opt/ros/humble/bin/xacro').exists(), reason='Real Humble xacro required')


@pytest.fixture(scope='module')
def cells(tmp_path_factory):
    from r4_1b_cells import author_cell_a, author_cell_b
    root = tmp_path_factory.mktemp('r4_1b_world')
    return {'a': author_cell_a(root)[0], 'b': author_cell_b(root)[0]}


def models(text):
    return {m.get('name'): m for m in ET.fromstring(text).find('world').findall('model')}


def boxes(model):
    return {c.get('name'): ([float(v) for v in (c.findtext('pose') or '0 0 0 0 0 0').split()][:3],
                            [float(v) for v in c.findtext('geometry/box/size').split()])
            for c in model.iter('collision') if c.find('geometry/box') is not None}


def copy_cell(cell, tmp_path):
    target = tmp_path / cell.name
    shutil.copytree(cell, target)
    return target


def test_each_cell_world_comes_from_its_own_planning_geometry_and_authored_objects(cells):
    reports = {}
    for name, cell in cells.items():
        text, report = sw.derive_world(cell)
        reports[name] = report
        manifest = yaml.safe_load((cell / 'config/moveit_collision_objects.yaml').read_text())
        support = next(o for o in manifest['objects'] if o['semantic_role'] == 'support_surface')
        world = models(text)
        assert set(world) == {'pick_support', 'target_bin_physics', report['objects'][0]['object_id']}   # nothing inherited
        pose = [float(v) for v in world['pick_support'].findtext('pose').split()]
        assert pose[:3] == pytest.approx(support['pose']['xyz'])
        assert boxes(world['pick_support'])['support_collision'][1] == pytest.approx(support['collision_geometry']['dimensions_m'])
        assert report['planning_manifest_sha256'] == hashlib.sha256((cell / 'config/moveit_collision_objects.yaml').read_bytes()).hexdigest()
        assert sw.engine_equivalent(text)                                   # qualified engine block kept verbatim
        assert report['robot_model'] == 'ur5' and report['end_effector_capability'] == 'robotiq_2f_85'
    a, b = reports['a'], reports['b']
    assert a['objects'][0]['object_id'] == 'part_04' and a['objects'][0]['class_id'] == 'part'
    assert a['objects'][0]['mass_kg'] == pytest.approx(0.020) and a['objects'][0]['within_qualified_dimensions']
    assert b['objects'][0]['object_id'] == 'bracket_b' and b['objects'][0]['class_id'] == 'bracket'
    assert b['objects'][0]['dimensions_m'] == [0.03, 0.02, 0.04] and b['objects'][0]['spawn_pose'][:3] == pytest.approx([0.46, -0.10, 0.0203])
    assert b['objects'][0]['mass_kg'] == pytest.approx(1280 * 0.03 * 0.02 * 0.04)
    # The destination follows the moved bin; its measured interior moves with it, its floor stays the mesh floor.
    shift = [0.36 - 0.45, 0.25 - 0.22]
    for axis, delta in zip('xy', shift):
        assert [v - delta for v in b['destination']['interior_world'][axis]] == pytest.approx(a['destination']['interior_world'][axis])
    assert a['destination']['floor_top_z'] == pytest.approx(0.003, abs=1e-6) == b['destination']['floor_top_z']
    assert a['world_sha256'] != b['world_sha256']


def test_the_destination_container_is_measured_from_the_bin_mesh(cells):
    text, report = sw.derive_world(cells['a'])
    measured = report['destination']['measured_local']
    assert measured['floor_top_z'] == pytest.approx(-0.097, abs=1e-6)              # sorting_bin.stl floor, not the origin
    assert measured['wall_top_z']['x_minus'] < measured['wall_top_z']['x_plus']     # the bin's open front is lower
    walls = boxes(models(text)['target_bin_physics'])
    floor_xyz, floor_size = walls['floor']
    assert floor_xyz[2] + floor_size[2] / 2 == pytest.approx(report['destination']['floor_top_z'])
    for side, (xyz, size) in walls.items():
        if side.startswith('wall_'):
            assert xyz[2] + size[2] / 2 == pytest.approx(report['destination']['wall_top_world_z'][side[5:]])
    # destination_binding re-measures the same planning mesh and accepts exactly this floor.
    from destination_binding import bind_destination_geometry
    binding = bind_destination_geometry(text, cells['a'] / 'config/moveit_collision_objects.yaml', measured_floor=True)
    assert binding['floor_z'] == pytest.approx(0.003) and binding['floor_contract'] == 'planning_mesh_floor'
    with pytest.raises(ValueError, match='destination origin'):
        bind_destination_geometry(text, cells['a'] / 'config/moveit_collision_objects.yaml')   # reference contract


def test_profile_destination_regions_reach_the_measured_floor(cells):
    from container_geometry import measure_container
    profile = yaml.safe_load((ROOT / 'catalog/workcell_studio_profiles/ur5_2f_workbench.yaml').read_text())['destination_regions']
    manifest = yaml.safe_load((cells['a'] / 'config/moveit_collision_objects.yaml').read_text())
    mesh = next(o for o in manifest['objects'] if o['source_item_id'] == profile['asset'])['collision_geometry']
    zone = profile['zones']['default_drop_zone']
    measured = measure_container(mesh['vertices_m'], mesh['triangles'], zone['pose_xyz'][:2], zone['pose_xyz'][2], zone['dimensions'][:2])
    assert measured['floor_top_z'] == pytest.approx(profile['measured_floor_top_local_z'], abs=1e-6)
    for region in (profile['usable_placement'], *profile['zones'].values()):
        # a part seated on the floor keeps the authored placement clearance on its bottom face too
        assert region['pose_xyz'][2] - region['dimensions'][2] / 2 <= measured['floor_top_z'] - 0.02 + 1e-9


def test_the_qualified_catalog_matches_the_pinned_stage_a_world():
    catalog = sw.load_catalog()
    world = ET.fromstring(sw.TEMPLATE.read_bytes()).find('world')
    assert world.get('name') == catalog['engine']['world_name']
    part = world.find("model[@name='part_04']")
    material = catalog['materials']['qualified_test_polymer']
    size = [float(v) for v in part.findtext('link/collision/geometry/box/size').split()]
    assert material['qualified_dimensions_m'] == size
    assert material['density_kg_m3'] == pytest.approx(float(part.findtext('link/inertial/mass')) / (size[0] * size[1] * size[2]))
    assert material['friction_mu'] == float(part.findtext('link/collision/surface/friction/ode/mu'))
    for key, value in material['contact'].items():
        assert value == float(part.findtext(f'link/collision/surface/contact/ode/{key}'))
    pose = [float(v) for v in part.findtext('pose').split()]
    assert catalog['construction']['object_spawn_clearance_m'] == pytest.approx(pose[2] - size[2] / 2)
    assert catalog['surfaces']['support']['friction_mu'] == float(world.find("model[@name='pick_support']").findtext('link/collision/surface/friction/ode/mu'))
    bin_floor = world.find("model[@name='target_bin_physics']/link/collision[@name='floor']")
    assert catalog['surfaces']['destination']['friction_mu'] == float(bin_floor.findtext('surface/friction/ode/mu'))
    assert catalog['construction']['container_floor_thickness_m'] == float(bin_floor.findtext('geometry/box/size').split()[2])


def edit_environment(cell, change):
    doc = yaml.safe_load((cell / 'environment.yaml').read_text())
    change(doc)
    (cell / 'environment.yaml').write_text(yaml.safe_dump(doc, sort_keys=False))


def manual(doc):
    return doc['workcell_studio']['pick_zone']['object_source']['manual_observation']


@pytest.mark.parametrize('change,message', [
    (lambda doc: manual(doc).update(material='steel'), "material 'steel' is not qualified"),
    (lambda doc: manual(doc).pop('material'), 'material None is not qualified'),
    (lambda doc: manual(doc).update(shape='cylinder'), 'only box'),
    (lambda doc: manual(doc)['pose'].update(xyz=[0.40, -0.20, 0.05]), 'does not rest'),
    (lambda doc: manual(doc)['pose'].update(rpy=[0.3, 0, 0]), 'resting flat'),
    (lambda doc: manual(doc).update(object_id='bad name'), 'valid, unique model name'),
    (lambda doc: doc['robot'].update(model='ur10'), "robot model 'ur10' is not commissioned"),
    (lambda doc: doc['end_effector'].update(capability='robotiq_3f'), "capability 'robotiq_3f' is not commissioned"),
    (lambda doc: doc['workcell_studio']['pick_zone']['object_source'].update(mode='live_epd_realsense'), 'manual test objects'),
])
def test_cells_outside_the_qualified_envelope_are_refused(cells, tmp_path, change, message):
    cell = copy_cell(cells['a'], tmp_path)
    manifest = yaml.safe_load((cell / 'config/moveit_collision_objects.yaml').read_text())
    edit_environment(cell, change)
    manifest['source']['physical_state_sha256'] = hashlib.sha256((cell / 'environment.yaml').read_bytes()).hexdigest()
    (cell / 'config/moveit_collision_objects.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
    with pytest.raises(sw.SimulatorWorldError, match=message):
        sw.derive_world(cell)


def test_stale_generated_files_and_floating_drop_regions_are_refused(cells, tmp_path):
    cell = copy_cell(cells['a'], tmp_path)
    edit_environment(cell, lambda doc: manual(doc)['pose'].update(xyz=[0.41, -0.20, 0.0125]))     # saved, not generated
    with pytest.raises(sw.SimulatorWorldError, match='stale'):
        sw.derive_world(cell)
    # The R4.1 beginner cell kept the reference drop region 47 mm above the measured bin floor.
    reference = yaml.safe_load((ROOT / 'scenes/ur5_2f_test/environment.yaml').read_text())['environment']
    def floating(doc):
        for key in ('assets', 'task_zones'):
            for item in doc['environment'][key]:
                old = next((r for r in reference[key] if r['id'] == item['id']), None)
                for field in ('usable_placement', 'placement_local', 'pose_xyz', 'dimensions'):
                    if old is not None and field in old and item['id'] in ('target_bin_default', 'default_drop_zone', 'place_zone_default'):
                        item[field] = copy.deepcopy(old[field])
    cell = copy_cell(cells['a'], tmp_path / 'floating')
    edit_environment(cell, floating)
    manifest = yaml.safe_load((cell / 'config/moveit_collision_objects.yaml').read_text())
    manifest['source']['physical_state_sha256'] = hashlib.sha256((cell / 'environment.yaml').read_bytes()).hexdigest()
    (cell / 'config/moveit_collision_objects.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
    with pytest.raises(sw.SimulatorWorldError, match='0.0470 m above the measured container floor'):
        sw.derive_world(cell)


def test_cli_reports_blocked_with_the_reason(tmp_path, capsys):
    assert sw.main([str(tmp_path), '--output', str(tmp_path / 'w.sdf')]) == 2
    assert json.loads(capsys.readouterr().out)['status'] == 'BLOCKED'


def test_a_routed_cell_renders_but_never_becomes_a_physics_world(cells, tmp_path):
    """R2.5A: the render-only world skips the physics task qualification; derive_world keeps it."""
    cell = copy_cell(cells['a'], tmp_path)
    path = cell / 'config/workcell_builder_task_intent.yaml'
    intent = yaml.safe_load(path.read_text())
    intent['routing'] = {'mode': 'class', 'unrouted_policy': 'leave_in_place', 'rules': [
        {'id': 'route_1', 'when': {'attribute': 'class', 'equals': 'cup'}, 'destination': intent['place']['target']['region_ref']}]}
    path.write_text(json.dumps(intent))
    with pytest.raises(sw.SimulatorWorldError, match='TASK_ROUTING_PHYSICS_UNQUALIFIED'):
        sw.derive_world(cell)
    text, report = sw.derive_render_world(cell)
    assert report['render_only'] and not report['physics_execution'] and report['world_name'] == sw.RENDER_WORLD_NAME
    assert ET.fromstring(text).find('world').get('name') == sw.RENDER_WORLD_NAME != sw.load_catalog()['engine']['world_name']
    assert not sw.engine_equivalent(text)                       # no qualified physics entry point accepts it
    assert set(models(text)) == {'pick_support', 'target_bin_physics', 'part_04'}
    assert not any(m.find('include') is not None or 'ur5' in m.get('name') for m in models(text).values())   # no robot
