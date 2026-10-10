"""Identity-join tests only; these fixtures are not live Gazebo or DART evidence."""
import copy
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_fragment_identity as api


def records():
    box=dict(type='BOX',size=[.025,.025,.025])
    inv=dict(worlds=[dict(id=1,parent=0)],models=[dict(id=2,parent=1)],
        links=[dict(id=3,parent=2)],visuals=[dict(id=4,parent=3,geometry=box,opaque=True)],
        collisions=[dict(id=5,parent=3,geometry=box)],complete=True)
    owner=dict(session='s',world_entity=1,step=9,stamp_ns=9000,complete=True,
        identity_inventory=inv,shapes=[dict(collision_id=5,physics_shape_id=50,shape_node_identity='node50',shape_type='BoxShape')])
    context={k:owner[k] for k in ('session','world_entity','step','stamp_ns')}
    context['scene_fingerprint']=api.identity_scene_fingerprint(owner)
    context['scene_id']=7
    renderer=dict(**context,lookup_source='GazeboSceneManager::VisualById',complete=True,
        visuals=[dict(visual_id=4,link_id=3,model_id=2,renderer_id=4000000022)])
    return owner,renderer,context


def test_unique_entity_parent_and_owner_join_preserves_uint32():
    o,r,c=records();result=api.bind_visual_collisions(o,r,c)
    assert result['mappings']==[dict(visual_id=4,link_id=3,model_id=2,collision_id=5,
        physics_shape_id=50,shape_node_identity='node50',renderer_id=4000000022)]
    assert not result['contact_authority'] and result['mrt_binding']=='BLOCKED_NOT_WITNESSED'


def test_two_object_join_uses_full_width_entity_ids_and_not_enumeration_order():
    o,r,c=records();inv=o['identity_inventory'];base=2**40
    inv['models'].insert(0,dict(id=base+2,parent=1))
    inv['links'].insert(0,dict(id=base+3,parent=base+2))
    inv['visuals'].append(dict(inv['visuals'][0],id=base+4,parent=base+3))
    inv['collisions'].insert(0,dict(inv['collisions'][0],id=base+5,parent=base+3))
    o['shapes'].insert(0,dict(collision_id=base+5,physics_shape_id=51,shape_node_identity='node51',shape_type='BoxShape'))
    r['visuals'].insert(0,dict(visual_id=base+4,link_id=base+3,model_id=base+2,renderer_id=16777217))
    c['scene_fingerprint']=r['scene_fingerprint']=api.identity_scene_fingerprint(o)
    result=api.bind_visual_collisions(o,r,c)
    assert [(m['visual_id'],m['collision_id'],m['renderer_id']) for m in result['mappings']]==[
        (4,5,4000000022),(base+4,base+5,16777217)]


def test_snapshot_and_lookup_are_wired_to_existing_authorities():
    root=Path(__file__).resolve().parents[1]/'scripts/stage_a_rgbd'
    owner=(root/'physics_owner/owner_private.inc').read_text()
    adapter=(root/'renderer_identity.hh').read_text()
    assert 'workcell::IdentityInventory(ecm)' in owner
    assert 'entityWorldMap.Map().begin()->first' in owner
    assert 'manager.VisualById(id)' in adapter and 'RendererNodeWitness(visual)' in adapter
    assert 'Json::UInt64(node->Id())' in adapter
    assert 'UserData' not in adapter and 'Name()' not in adapter


@pytest.mark.parametrize('kind',['duplicate_visual','missing_visual','two_visuals','two_collisions',
    'missing_node','missing_shape','duplicate_shape','stale_session','stale_step','stale_world',
    'changed_scene','unsupported_shape','unsupported_geometry','bad_parent','transparent',
    'incomplete_inventory','incomplete_renderer','missing_renderer','duplicate_renderer','wrong_link','lost_model','stale_scene','wrong_collision'])
def test_identity_join_rejects_unsafe_or_incomplete_records(kind):
    o,r,c=records();inv=o['identity_inventory']
    if kind=='duplicate_visual':inv['visuals'].append(copy.deepcopy(inv['visuals'][0]))
    if kind=='missing_visual':inv['visuals']=[]
    if kind=='two_visuals':inv['visuals'].append(dict(inv['visuals'][0],id=6))
    if kind=='two_collisions':inv['collisions'].append(dict(inv['collisions'][0],id=6))
    if kind=='missing_node':o['shapes'][0]['shape_node_identity']=''
    if kind=='missing_shape':o['shapes']=[]
    if kind=='duplicate_shape':o['shapes'].append(copy.deepcopy(o['shapes'][0]))
    if kind=='stale_session':r['session']='other'
    if kind=='stale_step':r['step']=8
    if kind=='stale_world':r['world_entity']=2
    if kind=='stale_scene':r['scene_id']=8
    if kind=='changed_scene':inv['visuals'][0]['geometry']=dict(type='BOX',size=[.03,.03,.03])
    if kind=='unsupported_shape':o['shapes'][0]['shape_type']='MeshShape'
    if kind=='unsupported_geometry':inv['visuals'][0]['geometry']=dict(type='MESH')
    if kind=='bad_parent':inv['visuals'][0]['parent']=2
    if kind=='transparent':inv['visuals'][0]['opaque']=False
    if kind=='incomplete_inventory':inv['complete']=False
    if kind=='incomplete_renderer':r['complete']=False
    if kind=='missing_renderer':r['visuals']=[]
    if kind=='duplicate_renderer':r['visuals'].append(copy.deepcopy(r['visuals'][0]))
    if kind=='wrong_link':r['visuals'][0]['link_id']=2
    if kind=='wrong_collision':o['shapes'][0]['collision_id']=6
    if kind=='lost_model':inv['models']=[]
    if kind!='changed_scene':
        # Exercise the actual rejection, not merely a stale fingerprint.
        c['scene_fingerprint']=r['scene_fingerprint']=api.identity_scene_fingerprint(o)
    with pytest.raises(ValueError):api.bind_visual_collisions(o,r,c)
