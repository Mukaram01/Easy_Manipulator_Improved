import copy
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def model():
    return [
        {'id': 'table', 'pose_xyz': [1., 2., 0.], 'pose_rpy': [0., 0., 0.],
         'mounting_anchors': {'surface': {'pose_xyz': [0., 0., .8], 'pose_rpy': [0., 0., 0.]}}},
        {'id': 'bin', 'pose_xyz': [1.2, 2., .8], 'pose_rpy': [.1, .2, .3]},
        {'id': 'camera', 'pose_xyz': [1.2, 2., 1.8], 'pose_rpy': [.2, -.4, .6]},
    ]


def test_reparent_rotate_chain_detach_and_yaml_roundtrip():
    import asset_mounting as m
    rows = model()
    original = copy.deepcopy(rows)
    rows = m.edit(rows, 'bin', parent_object='table', parent_link='surface')
    assert rows[1]['pose_xyz'] == pytest.approx(original[1]['pose_xyz'])
    assert rows[1]['origin']['x'] == pytest.approx(.2)
    rows = m.edit(rows, 'camera', parent_object='bin', parent_link='origin')
    rows = m.edit(rows, 'table', world_pose={'xyz': [2., 3., 1.], 'rpy': [.3, -.2, 1.2]})
    assert rows[1]['pose_xyz'] != pytest.approx(original[1]['pose_xyz'])
    m.validate(rows)
    assert yaml.safe_load(yaml.safe_dump(rows)) == rows
    mounted = copy.deepcopy(rows)
    rows = m.edit(rows, 'bin', parent_object='world', parent_link='world')
    assert rows[1]['pose_xyz'] == pytest.approx(mounted[1]['pose_xyz'])
    assert rows[2]['pose_xyz'] == pytest.approx(mounted[2]['pose_xyz'])
    rows = m.edit(rows, 'table', world_pose={'xyz': [0., 0., 0.], 'rpy': [0., 0., 0.]})
    assert rows[1]['pose_xyz'] == pytest.approx(mounted[1]['pose_xyz'])
    assert rows[2]['pose_xyz'] == pytest.approx(mounted[2]['pose_xyz'])


@pytest.mark.parametrize('parent,anchor,message', [
    ('absent', 'origin', 'missing parent'), ('table', 'absent', 'invalid anchor'),
    ('bin', 'origin', 'cycle')])
def test_invalid_mount_is_atomic(parent, anchor, message):
    import asset_mounting as m
    rows = model()
    before = copy.deepcopy(rows)
    with pytest.raises(ValueError, match=message):
        m.edit(rows, 'bin', parent_object=parent, parent_link=anchor)
    assert rows == before


def test_duplicate_cycle_nonfixed_and_stale_projections_rejected():
    import asset_mounting as m
    rows = model()
    with pytest.raises(ValueError, match='duplicate'):
        m.validate(rows + [copy.deepcopy(rows[0])])
    rows = m.edit(rows, 'bin', parent_object='table', parent_link='surface')
    with pytest.raises(ValueError, match='cycle'):
        m.edit(rows, 'table', parent_object='bin', parent_link='origin')
    rows[1]['joint_type'] = 'revolute'
    with pytest.raises(ValueError, match='fixed'):
        m.validate(rows)
    rows[1]['joint_type'] = 'fixed'
    rows[0]['pose_xyz'][0] += 1
    with pytest.raises(ValueError, match='stale world pose'):
        m.validate(rows)


def test_relative_pose_composes_with_named_anchor():
    import asset_mounting as m
    rows = m.edit(model(), 'bin', parent_object='table', parent_link='surface',
                  relative_pose={'xyz': [.3, .4, .1], 'rpy': [.2, .1, .5]})
    assert rows[1]['pose_xyz'] == pytest.approx([1.3, 2.4, .9])
    assert rows[1]['pose_rpy'] == pytest.approx([.2, .1, .5])


def test_save_reopen_generation_and_collision_keep_mounts(tmp_path):
    import asset_mounting as m
    from workcell_studio_layout_merge import merge
    from generate_moveit_collision_manifest import load_and_build
    from generate_workcell_from_cell_definition import _build_environment_objects
    rows = model()[:2]
    for row in rows:
        row.update(type='object', geometry_type='box', dimensions=[.2, .3, .4],
                   collision={'enabled': True, 'mode': 'box_proxy'})
    rows = m.edit(rows, 'bin', parent_object='table', parent_link='surface')
    rows = m.edit(rows, 'table', world_pose={'xyz': [2., 3., 1.], 'rpy': [.2, -.1, .7]})
    scene = tmp_path / 'cell'
    (scene / 'layout').mkdir(parents=True)
    (scene / 'environment.yaml').write_text(yaml.safe_dump({'environment': {'assets': model()[:2]}}))
    layout = scene / 'layout/workcell_studio_layout.yaml'
    layout.write_text(yaml.safe_dump({'items': [dict(row, pose={'xyz': row['pose_xyz'], 'rpy': row['pose_rpy']}) for row in rows]}))
    merge(scene, save_authored=True)
    saved = yaml.safe_load((scene / 'environment.yaml').read_text())
    m.validate_physical(saved['environment'])
    assert saved['environment']['assets'][1]['origin'] == rows[1]['origin']
    merge(scene)
    objects = _build_environment_objects(saved)['objects']
    assert objects[1]['parent_object'] == 'table'
    assert objects[1]['pose']['xyz'] == pytest.approx(rows[1]['pose_xyz'])
    manifest = load_and_build(layout)
    child = next(obj for obj in manifest['objects'] if obj['source_item_id'] == 'bin')
    assert child['pose']['xyz'] == pytest.approx(rows[1]['pose_xyz'])
    # A bad edit cannot partially change authored environment state.
    before = (scene / 'environment.yaml').read_bytes()
    rows[1]['parent_object'] = 'missing'
    layout.write_text(yaml.safe_dump({'items': rows}))
    with pytest.raises(ValueError, match='missing parent'):
        merge(scene, save_authored=True)
    assert (scene / 'environment.yaml').read_bytes() == before


def test_save_translates_editor_parent_to_stable_physical_id(tmp_path):
    import asset_mounting as m
    from workcell_studio_layout_merge import merge
    rows = model()[:2]
    rows[0]['id'] = 'editor_table'
    physical = copy.deepcopy(rows)
    physical[0].update(id='physical_table', layout_item_ref='editor_table')
    rows = m.edit(rows, 'bin', parent_object='editor_table', parent_link='surface')
    scene = tmp_path / 'cell'
    (scene / 'layout').mkdir(parents=True)
    (scene / 'environment.yaml').write_text(yaml.safe_dump({'environment': {'assets': physical}}))
    (scene / 'layout/workcell_studio_layout.yaml').write_text(yaml.safe_dump({'items': [dict(row, pose={'xyz': row['pose_xyz'], 'rpy': row['pose_rpy']}) for row in rows]}))
    merge(scene, save_authored=True)
    saved = yaml.safe_load((scene / 'environment.yaml').read_text())['environment']
    assert saved['assets'][1]['parent_object'] == 'physical_table'
    m.validate_physical(saved)


def test_save_stages_catalog_support_in_existing_scene_assets(tmp_path):
    from workcell_studio_layout_merge import merge
    source = tmp_path / 'support.stl'
    source.write_bytes(b'solid support\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid support\n')
    scene = tmp_path / 'cell'
    (scene / 'layout').mkdir(parents=True)
    item = dict(id='stand', catalog_asset_id='camera_stand', geometry_type='mesh',
                mesh={'path': str(source)}, pose={'xyz': [0,0,0], 'rpy': [0,0,0]})
    layout = scene / 'layout/workcell_studio_layout.yaml'
    layout.write_text(yaml.safe_dump({'items': [item]}))
    merge(scene, save_authored=True)
    saved = yaml.safe_load(layout.read_text())['items'][0]
    staged = scene / saved['mesh']['path']
    assert saved['mesh']['path'].startswith('assets/imported/catalog/')
    from generate_moveit_collision_manifest import read_stl_mesh
    assert read_stl_mesh(staged) == read_stl_mesh(source)
    assert len(staged.read_bytes()) == 134
    assert source.read_bytes().startswith(b'solid')
    from workcell_studio_layout_mesh_preview_node import resolve_mesh_resource
    assert resolve_mesh_resource(saved['mesh']['path'], 'cell') == 'package://cell/' + saved['mesh']['path']
    merge(scene, save_authored=True)
    assert len(list((scene / 'assets/imported/catalog').iterdir())) == 1


def test_browser_save_exports_assets_without_read_only_destination_projections():
    import json
    import subprocess
    source = (Path(__file__).resolve().parents[1] / 'workcell_studio_web/viewer/viewer.js').read_text()
    derived = source[source.index('function derivedTransformTargetId('):source.index('function inspectionSelectionRendered(')]
    function = source[source.index('function buildEditPatch()'):source.index('function exportEditPatch()')]
    authority = source[source.index('function editAuthoritySource('):source.index('const TRANSFORM_FIELD_SPECS')]
    script = """const EDIT_PATCH_SCHEMA_VERSION='test',VIEWER_VERSION='test';
const state={objects:[{item:{id:'bin',editable:true,locked:false,source_layer:'editable_layout'}},{item:{id:'destination',role:'place_zone',target_ref:'bin',editable:true,locked:false,source_layer:'editable_layout'}}],dirtyTransforms:new Map([['bin',{oldTransform:{},newTransform:{}}],['destination',{oldTransform:{},newTransform:{}}]])};
const sceneId=()=> 'cell',itemLabel=()=>'',itemType=()=>'',isDiagnosticOnlyItem=()=>false;
""" + derived + authority + function + "console.log(JSON.stringify(buildEditPatch()));"
    patch = json.loads(subprocess.check_output(['node', '-e', script], text=True))
    assert [edit['item_id'] for edit in patch['edits']] == ['bin']


def test_browser_save_rebases_derived_destinations_with_persisted_target():
    import json
    import subprocess
    source = (Path(__file__).resolve().parents[1] / 'workcell_studio_web/viewer/viewer.js').read_text()
    derived = source[source.index('function derivedTransformTargetId('):source.index('function inspectionSelectionRendered(')]
    rebase = source[source.index('function rebasePersistedDependents('):source.index('// Runs inside the viewer module;')]
    script = """const destination={item:{id:'zone',role:'place_zone',target_ref:'bin'},object3d:{pose:2},originalTransform:{pose:1}};
const state={objects:[destination],dirtyTransforms:new Map([['zone',{}],['bin',{}]])};
const cloneTransform=x=>JSON.parse(JSON.stringify(x)),transformFromObject=x=>x;
""" + derived + rebase + """
rebasePersistedDependents(['bin']);
if(!state.dirtyTransforms.has('zone')) throw Error('newer parent edit was lost');
state.dirtyTransforms.delete('bin');
rebasePersistedDependents(['bin']);
console.log(JSON.stringify({dirty:state.dirtyTransforms.size,baseline:destination.originalTransform}));
"""
    result = json.loads(subprocess.check_output(['node', '-e', script], text=True))
    assert result == {'dirty': 0, 'baseline': {'pose': 2}}
