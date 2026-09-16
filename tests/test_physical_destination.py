"""Physical ownership, rigid transforms and fail-closed projection checks."""
import copy
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def scene():
    return {'assets': [{'id': 'bin', 'frame': 'world',
                        'pose_xyz': [1., 2., 3.], 'pose_rpy': [0., 0., math.pi / 2],
                        'collision': {'enabled': True, 'mode': 'mesh'},
                        'usable_placement': {'pose_xyz': [0., 0., .1], 'pose_rpy': [0., 0., 0.],
                                             'dimensions': [.4, .3, .2]}}],
            'task_zones': [{'id': 'drop', 'target_ref': 'bin', 'frame': 'world',
                            'placement_local': {'pose_xyz': [.05, 0., .1], 'pose_rpy': [0., 0., 0.],
                                                'dimensions': [.2, .1, .1]},
                            'pose_xyz': [1., 2.05, 3.1], 'pose_rpy': [0., 0., math.pi / 2],
                            'dimensions': [.2, .1, .1]}]}


def test_resolves_rotated_target_local_region():
    from physical_destination import resolve_destination
    result = resolve_destination(scene(), 'drop')
    assert result['pose_xyz'] == pytest.approx([1, 2.05, 3.1])
    assert result['target_id'] == 'bin'
    assert result['pose_rpy'] == pytest.approx([0, 0, math.pi / 2])


@pytest.mark.parametrize('mutation,reason', [
    (lambda e: e['assets'].clear(), 'target'),
    (lambda e: e['assets'][0].pop('usable_placement'), 'usable'),
    (lambda e: e['task_zones'][0].pop('placement_local'), 'placement_local'),
    (lambda e: e['task_zones'][0]['placement_local'].update(pose_xyz=[5, 0, .1]), 'outside'),
    (lambda e: e['assets'][0].update(pose_xyz=[5, 2, 3]), 'stale'),
    (lambda e: e['task_zones'][0].update(pose_xyz=[5, 2, 3]), 'stale'),
    (lambda e: e['assets'][0]['usable_placement'].update(dimensions=[float('nan'), 1, 1]), 'finite'),
    (lambda e: e['assets'][0]['collision'].update(enabled=False), 'collision'),
])
def test_rejects_invalid_physical_destination(mutation, reason):
    from physical_destination import resolve_destination
    env = scene()
    mutation(env)
    with pytest.raises(ValueError, match=reason):
        resolve_destination(env, 'drop')


def test_rotated_object_must_fit_region():
    from physical_destination import resolve_destination, check_object_containment
    destination = resolve_destination(scene(), 'drop')
    check_object_containment(destination, [1, 2.05, 3.1, 0, 0, math.sqrt(.5), math.sqrt(.5)], [.18, .08, .08])
    with pytest.raises(ValueError, match='outside'):
        check_object_containment(destination, [1, 2.05, 3.1, 0, 0, 0, 1], [.18, .08, .08])


def test_runtime_rejects_legacy_layout_only(tmp_path):
    from perceived_object_grasp_execute import load_canonical_place_target
    import yaml
    (tmp_path / 'layout').mkdir()
    (tmp_path / 'layout/workcell_studio_layout.yaml').write_text(yaml.safe_dump({'items': [{'id': 'target_bin_default', 'pose': {'xyz': [1, 2, 3]}}]}))
    with pytest.raises(RuntimeError, match='handoff'):
        load_canonical_place_target(tmp_path)


def test_runtime_and_validation_reject_disjoint_projection(tmp_path):
    import yaml
    from perceived_object_grasp_execute import load_canonical_place_target
    from validate_cell_definition import validate_cell_definition
    env = scene()
    env['task_zones'][0]['pose_xyz'] = [5, 6, 7]
    cell = {'environment': env, 'task': {'place': {'target_ref': 'drop'}}}
    path = tmp_path / 'cell_definition.yaml'
    path.write_text(yaml.safe_dump(cell))
    with pytest.raises((ValueError, RuntimeError), match='stale'):
        load_canonical_place_target(tmp_path)
    result = validate_cell_definition(cell, path, 'yaml', [])
    assert any('stale' in error for error in result.errors)


def test_save_target_motion_carries_region_and_zone_edit_updates_local(tmp_path):
    import yaml
    from workcell_studio_layout_merge import merge
    from physical_destination import resolve_destination
    env = scene()
    env['task_zones'][0].update(layout_item_ref='overlay', category='zone')
    items = []
    for record in [env['assets'][0], env['task_zones'][0]]:
        item = copy.deepcopy(record)
        item['id'] = record.get('layout_item_ref', record['id'])
        item['pose'] = {'xyz': item.pop('pose_xyz'), 'rpy': item.pop('pose_rpy')}
        items.append(item)
    (tmp_path / 'layout').mkdir()
    (tmp_path / 'environment.yaml').write_text(yaml.safe_dump({'environment': env}))
    lp = tmp_path / 'layout/workcell_studio_layout.yaml'
    items[0]['pose']['xyz'][0] += .2
    lp.write_text(yaml.safe_dump({'items': items}))
    merge(tmp_path, save_authored=True)
    saved = yaml.safe_load((tmp_path / 'environment.yaml').read_text())['environment']
    assert resolve_destination(saved, 'drop')['pose_xyz'] == pytest.approx([1.2, 2.05, 3.1])
    layout = yaml.safe_load(lp.read_text())
    zone = next(i for i in layout['items'] if i['id'] == 'overlay')
    zone['pose']['xyz'][1] += .01
    lp.write_text(yaml.safe_dump(layout))
    merge(tmp_path, save_authored=True)
    saved = yaml.safe_load((tmp_path / 'environment.yaml').read_text())['environment']
    assert saved['task_zones'][0]['placement_local']['pose_xyz'] == pytest.approx([.06, 0, .1])
    assert resolve_destination(saved, 'drop')['pose_xyz'] == pytest.approx([1.2, 2.06, 3.1])


def test_product_view_matches_runtime_volume_and_rejects_stale_state():
    from export_workcell_studio_web_scene_impl import _normalise_active_place_zone, BlockingExportError
    from physical_destination import resolve_destination
    env = scene()
    env['task_zones'][0]['layout_item_ref'] = 'overlay'
    data = {'environment': {'environment': env, 'task': {'place': {'target_ref': 'drop'}}},
            'layout': {'items': [{'id': 'overlay'}]}}
    _normalise_active_place_zone(data)
    shown = data['layout']['items'][0]
    runtime = resolve_destination(env, 'drop')
    assert shown['pose']['xyz'] == runtime['pose_xyz']
    assert shown['pose']['rpy'] == runtime['pose_rpy']
    assert shown['dimensions'] == runtime['dimensions']
    env['task_zones'][0]['pose_xyz'][0] += 1
    with pytest.raises(BlockingExportError, match='stale'):
        _normalise_active_place_zone(data)


def test_missing_target_ref_rejected_by_validation(tmp_path):
    from validate_cell_definition import validate_cell_definition
    env = scene()
    env['task_zones'][0].pop('target_ref')
    env['task_zones'][0].pop('placement_local')
    env['task_zones'][0]['type'] = 'place_zone'
    result = validate_cell_definition({'environment': env}, tmp_path/'cell.yaml', 'yaml', [])
    assert any('physical target' in error for error in result.errors)


def test_validate_scene_rejects_authored_mismatch_without_cached_export(tmp_path):
    import yaml
    from validate_builder_generated_scene import validate_scene
    env = scene()
    env['task_zones'][0]['pose_xyz'][0] += 1
    (tmp_path / 'environment.yaml').write_text(yaml.safe_dump({'environment': env}))
    result = validate_scene(tmp_path)
    assert any('stale' in error for error in result['errors'])


def test_deleted_active_zone_rejected_by_validate_save_and_generation(tmp_path):
    import yaml
    from validate_cell_definition import validate_cell_definition
    from validate_builder_generated_scene import validate_scene
    from workcell_studio_layout_merge import merge
    env = scene()
    env['task_zones'] = []
    task = {'place': {'target_ref': 'drop'}}
    cell = {'environment': env, 'task': task}
    result = validate_cell_definition(cell, tmp_path/'cell.yaml', 'yaml', [])
    assert any('drop' in error and 'missing' in error for error in result.errors)
    (tmp_path/'layout').mkdir()
    (tmp_path/'layout/workcell_studio_layout.yaml').write_text('items: []\n')
    (tmp_path/'environment.yaml').write_text(yaml.safe_dump(cell))
    assert any('drop' in error and 'missing' in error for error in validate_scene(tmp_path)['errors'])
    with pytest.raises(ValueError, match='drop.*missing'):
        merge(tmp_path, save_authored=True)


def test_resolve_local_destination_uses_target_frame_and_region_bounds():
    from physical_destination import resolve_local_destination
    env = scene()
    original = copy.deepcopy(env)
    requested = {'xyz_m': [.08, 0., .1], 'rpy_rad': [0., 0., .2]}
    result = resolve_local_destination(env, 'bin', 'drop', requested)
    assert result['target_id'] == 'bin'
    assert result['id'] == 'drop'
    assert result['placement_local']['pose_xyz'] == pytest.approx([.08, 0., .1])
    assert result['pose_xyz'] == pytest.approx([1., 2.08, 3.1])
    assert result['pose_rpy'] == pytest.approx([0., 0., math.pi / 2 + .2])
    assert result['physical_destination_contract'] == 'target_local/v1'
    assert env == original
    assert requested == {'xyz_m': [.08, 0., .1], 'rpy_rad': [0., 0., .2]}
    result['region_local']['pose_xyz'][0] = 99
    result['usable_placement']['dimensions'][0] = 99
    assert env == original


def test_resolve_local_destination_rejects_pose_outside_named_region():
    from physical_destination import resolve_local_destination, RequestedLocalPoseError
    with pytest.raises(RequestedLocalPoseError, match='outside'):
        resolve_local_destination(scene(), 'bin', 'drop',
                                  {'xyz_m': [.16, 0., .1], 'rpy_rad': [0., 0., 0.]})


def test_local_destination_full_object_uses_fixed_rotated_region():
    from physical_destination import resolve_local_destination, check_object_containment
    env = scene()
    # Target yaw pi/2 plus region yaw pi/2 gives world yaw pi.
    env['task_zones'][0]['placement_local']['pose_rpy'][2] = math.pi / 2
    env['task_zones'][0]['pose_rpy'][2] = math.pi
    result = resolve_local_destination(env, 'bin', 'drop',
                                      {'xyz_m': [.05, .08, .1], 'rpy_rad': [0., 0., 0.]})
    assert result['pose_xyz'] == pytest.approx([.92, 2.05, 3.1])
    # The center fits, but a 6 cm object extends beyond the fixed region.
    with pytest.raises(ValueError, match='outside'):
        check_object_containment(result, [.92, 2.05, 3.1, 0, 0, 0, 1], [.06, .02, .02])
    check_object_containment(result, [.92, 2.05, 3.1, 0, 0, 0, 1], [.02, .02, .02])
    # An object at the authored center still fits despite the selected offset.
    check_object_containment(result, [1., 2.05, 3.1, 0, 0, 0, 1], [.18, .08, .08])


@pytest.mark.parametrize('requested', [None, {}, {'xyz_m': [0, 0], 'rpy_rad': [0, 0, 0]},
    {'xyz_m': [True, 0, .1], 'rpy_rad': [0, 0, 0]},
    {'xyz_m': [.05, 0, .1], 'rpy_rad': [0, float('nan'), 0]},
    {'xyz_m': [.05, 0, .1], 'rpy_rad': [0, 0, float('inf')]}])
def test_local_destination_invalid_pose_is_distinguishable(requested):
    from physical_destination import resolve_local_destination, RequestedLocalPoseError
    with pytest.raises(RequestedLocalPoseError):
        resolve_local_destination(scene(), 'bin', 'drop', requested)


@pytest.mark.parametrize('mutation', [
    lambda e: e['assets'].clear(),
    lambda e: e['task_zones'].clear(),
    lambda e: e['assets'][0].update(pose_xyz=[4, 2, 3]),
    lambda e: e['assets'][0].update(usable_placement=None),
    lambda e: e['assets'][0].update(pose=None),
    lambda e: e.update(assets=None),
    lambda e: e['assets'][0]['usable_placement'].update(dimensions=[-1, .3, .2]),
])
def test_local_destination_structural_errors_precede_invalid_pose(mutation):
    from physical_destination import resolve_local_destination, PhysicalDestinationContextError
    env = scene()
    mutation(env)
    with pytest.raises(PhysicalDestinationContextError):
        resolve_local_destination(env, 'bin', 'drop', None)


def test_local_destination_target_mismatch_is_structural():
    from physical_destination import resolve_local_destination, PhysicalDestinationContextError
    with pytest.raises(PhysicalDestinationContextError, match='mismatch'):
        resolve_local_destination(scene(), 'other', 'drop',
                                  {'xyz_m': [.05, 0, .1], 'rpy_rad': [0, 0, 0]})
