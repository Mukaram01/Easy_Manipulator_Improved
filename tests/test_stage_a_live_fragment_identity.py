"""CPU-only tests of the live capture gate; records are NOT runtime evidence."""
import copy
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_fragment_identity as api
from test_stage_a_visual_collision_identity import records
from test_stage_a_fragment_identity import capture


def live():
    owner,renderer,context=records();report,rgb,ids=capture()
    # The native capture inventory must describe ALL visuals and ID-bearing items.
    inv=owner['identity_inventory'];box=inv['visuals'][0]['geometry']
    inv['models'].append(dict(id=12,parent=1));inv['links'].append(dict(id=13,parent=12))
    inv['visuals'].append(dict(id=14,parent=13,geometry=box,opaque=True))
    inv['collisions'].append(dict(id=15,parent=13,geometry=box))
    owner['shapes'].append(dict(collision_id=15,physics_shape_id=150,shape_node_identity='node150',shape_type='BoxShape'))
    renderer['visuals'][0]['renderer_id']=16777217
    renderer['visuals'].append(dict(visual_id=14,link_id=13,model_id=12,renderer_id=4000000022))
    context['scene_fingerprint']=renderer['scene_fingerprint']=api.identity_scene_fingerprint(owner)
    report.update(session='s',frame=1,render_pass_count=1,render_owner='GazeboRenderUtil',
        owner_inventory_equal=True,renderer=renderer,acquisition=dict(session='s',world_entity=1,
        step=9,stamp_ns=9000,scene_id=7,frame=1,update_epoch=2),
        scene_updates=[dict(epoch=1,step=8,stamp_ns=8000),dict(epoch=2,step=9,stamp_ns=9000)],
        geometry_source='SceneManager::VisualById::GeometryByIndex::OgreObject',
        rgb_profile='opaque_unlit_diffuse_MRT',draw_bindings=[
            dict(visual_id=4,renderer_id=16777217,item_id=10,original_item_reused=True),
            dict(visual_id=14,renderer_id=4000000022,item_id=11,original_item_reused=True)])
    return owner,report,rgb,ids


def test_actual_visual_draw_join_keeps_timing_and_contact_blocked():
    o,r,rgb,ids=live();result=api.validate_live_fragment_capture(o,r,rgb,ids)
    assert result['visual_collision_draw_binding']=='PASS'
    assert [m['collision_id'] for m in result['mappings']]==[5,15]
    assert result['timing_authority']=='BLOCKED' and result['contact_authority'] is False


@pytest.mark.parametrize('kind',['stale_update','missing_update','wrong_scene','duplicate_item',
    'missing_item','replacement_item','unexpected_geometry','wrong_id','rgb_changed','missing_node','bad_inventory'])
def test_live_gate_rejects_unproven_draw_or_context(kind):
    o,r,rgb,ids=live()
    if kind=='stale_update':r['acquisition']['update_epoch']=1
    if kind=='missing_update':r['scene_updates']=[]
    if kind=='wrong_scene':r['acquisition']['scene_id']=8
    if kind=='duplicate_item':r['draw_bindings'][1]['item_id']=10
    if kind=='missing_item':r['draw_bindings'].pop()
    if kind=='replacement_item':r['draw_bindings'][0]['original_item_reused']=False
    if kind=='unexpected_geometry':r['draw_bindings'].append(dict(visual_id=99,renderer_id=22,item_id=12,original_item_reused=True))
    if kind=='wrong_id':r['draw_bindings'][0]['renderer_id']=4000000022
    if kind=='rgb_changed':rgb[0,0]=2
    if kind=='missing_node':o['shapes'][0]['shape_node_identity']=''
    if kind=='bad_inventory':r['owner_inventory_equal']=False
    with pytest.raises(ValueError):api.validate_live_fragment_capture(o,r,rgb,ids)


def test_incomplete_native_mapping_is_retained_before_rejection_and_hash_pinned():
    root=Path(__file__).resolve().parents[1]
    source=(root/'scripts/stage_a_rgbd/gazebo_fragment_capture.cpp').read_text()
    assert source.index('record["renderer"]=mapping')<source.index('if(!mapping["complete"].asBool())')
    runner=(root/'scripts/stage_a_gazebo_fragment.py').read_text()
    assert "'scripts/stage_a_rgbd/renderer_identity.hh'" in runner
    assert "'scripts/stage_a_rgbd/renderer_identity_check.hh'" in runner


def test_native_gl_context_reacquisition_is_witnessed_and_strict():
    root=Path(__file__).resolve().parents[1]
    source=(root/'scripts/stage_a_rgbd/gazebo_fragment_capture.cpp').read_text()
    assert source.index('["before_reacquire"]=GlContextWitness()')<source.index('rs->postExtraThreadsStarted()')
    assert source.index('rs->postExtraThreadsStarted()')<source.index('["after_reacquire"]=GlContextWitness()')
    assert source.index('diagnostics.flush()')<source.index('if(glState!="PASS_CURRENT_GL45")')
    assert 'renderThread!=std::this_thread::get_id()' in source
    runner=(root/'scripts/stage_a_gazebo_fragment.py').read_text()
    for name in ('gl_context_witness.hh','gl_context_check.hh'):
        assert "'scripts/stage_a_rgbd/"+name+"'" in runner


def test_both_material_witnesses_precede_native_material_rejection():
    root=Path(__file__).resolve().parents[1]
    source=(root/'scripts/stage_a_rgbd/gazebo_fragment_capture.cpp').read_text()
    assert source.index('record["materials"].append(evidence)')<source.index('unsupported native material: ')
    assert 'materials.at(visualId)' in source
    assert 'const auto &colour=material["diffuse"]' in source
    runner=(root/'scripts/stage_a_gazebo_fragment.py').read_text()
    for name in ('material_contract.hh','material_witness.hh'):
        assert "'scripts/stage_a_rgbd/"+name+"'" in runner
