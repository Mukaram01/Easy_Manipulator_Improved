"""Same-fragment fixture checks; renderer-local identity only, never contact authority.

No masks are generated here. Callers must preserve genuine EPD masks and verify
its preprocessing/capture provenance before using mask_identity in a live path.
"""
import hashlib
import numpy as np
from stage_a_physics_contact import require


def validate_fragment_capture(report,rgb,ids,session,frame):
    require(bool(session) and report.get('session')==session and report.get('frame')==frame,'stale capture')
    shape=(report.get('height'),report.get('width'))
    require(rgb.dtype==np.uint8 and rgb.shape==(*shape,3) and
            ids.dtype==np.uint32 and ids.shape==shape,'unsupported image buffers')
    require(report.get('shared_scene_pass') is True and report.get('samples')==1 and
            report.get('opaque') is True and report.get('blending') is False and
            report.get('depth_test') is True and report.get('depth_write') is True,
            'unsupported fragment visibility contract')
    require(report.get('rgb_format')=='RGBA8_UNORM' and report.get('id_format')=='R32_UINT','unsupported attachments')
    require(report.get('inventory_complete') is True,'incomplete visual inventory')
    inventory=report.get('inventory',[])
    values=[v.get('id') for v in inventory];entities=[v.get('native_entity') for v in inventory]
    require(values and all(type(v) is int and 0<v<4294967295 for v in values) and
            len(set(values))==len(values) and all(isinstance(v,str) and v for v in entities) and
            len(set(entities))==len(entities),'invalid or duplicate visual inventory')
    require(set(np.unique(ids))=={0,*values},'missing, invalid or unexpected visible IDs')
    require(report.get('rgb_sha256')==hashlib.sha256(rgb.tobytes()).hexdigest() and
            report.get('id_sha256')==hashlib.sha256(ids.astype('<u4').tobytes()).hexdigest(),
            'changed RGB/ID correspondence')
    return dict(ids=sorted(values),contact_authority=False,scope='renderer-local fixture only; physics mapping unknown')


def mask_identity(report,rgb,ids,mask,session,frame,epd_rgb_sha256):
    validate_fragment_capture(report,rgb,ids,session,frame)
    require(epd_rgb_sha256==report['rgb_sha256'],'EPD RGB image mismatch')
    require(isinstance(mask,np.ndarray) and mask.dtype==np.bool_ and mask.shape==ids.shape and np.any(mask),'invalid mask')
    values=np.unique(ids[mask])
    require(len(values)==1 and int(values[0]) not in (0,4294967295),'mixed, missing or ambiguous mask ID')
    return int(values[0])


def identity_scene_fingerprint(owner):
    """Topology/geometry/lifetime digest; no dynamic simulator poses exported."""
    import json
    inventory=owner.get('identity_inventory',{})
    # Sorting removes enumeration-order dependence; duplicates remain detectable.
    canonical={k:sorted(v,key=lambda row:row['id']) if isinstance(v,list) else v
               for k,v in inventory.items()}
    shapes=sorted(({k:s.get(k) for k in ('collision_id','physics_shape_id','shape_node_identity','shape_type')}
                   for s in owner.get('shapes',[])),key=lambda s:s['collision_id'])
    return hashlib.sha256(json.dumps(dict(inventory=canonical,shapes=shapes),sort_keys=True,
        separators=(',',':'),allow_nan=False).encode()).hexdigest()


def bind_visual_collisions(owner,renderer,expected):
    """Strict read-only identity metadata join, NOT MRT draw/EPD/contact authority.

    Renderer records must come from the live Gazebo SceneManager lookup adapter.
    This function cannot prove that an ID attachment used those same objects.
    """
    for key in ('session','world_entity','step','stamp_ns'):
        require(key in expected and owner.get(key)==renderer.get(key)==expected[key],'changed world/session/step')
    require(isinstance(expected['session'],str) and expected['session'] and
            all(type(expected[k]) is int and expected[k]>=0 for k in ('world_entity','step','stamp_ns')),'invalid capture context')
    require(owner.get('complete') is True and renderer.get('complete') is True,'incomplete owner/renderer')
    require(renderer.get('lookup_source')=='GazeboSceneManager::VisualById' and
            type(renderer.get('scene_id')) is int and renderer['scene_id']>=0 and
            renderer['scene_id']==expected.get('scene_id'),'missing/changed native scene lookup')
    fingerprint=identity_scene_fingerprint(owner)
    require(expected.get('scene_fingerprint')==renderer.get('scene_fingerprint')==fingerprint,'untracked scene change')
    inv=owner.get('identity_inventory',{});require(inv.get('complete') is True,'incomplete ECM inventory')
    groups={};seen=set()
    for kind in ('worlds','models','links','visuals','collisions'):
        rows=inv.get(kind,[]);group={}
        require(isinstance(rows,list) and bool(rows),'missing ECM inventory category')
        for row in rows:
            identity=row.get('id');parent=row.get('parent')
            require(type(identity) is int and identity>0 and identity not in seen and
                    type(parent) is int and parent>=0,'duplicate/missing entity identity')
            seen.add(identity);group[identity]=row
        groups[kind]=group
    worlds,models,links,visuals,collisions=(groups[k] for k in ('worlds','models','links','visuals','collisions'))
    require(set(worlds)=={expected['world_entity']} and next(iter(worlds.values()))['parent']==0,'wrong world entity')
    require(all(m['parent'] in worlds for m in models.values()) and
            all(l['parent'] in models for l in links.values()) and
            {l['parent'] for l in links.values()}==set(models),'unsupported model/link topology')
    for rows in (visuals,collisions):
        require(all(row['parent'] in links for row in rows.values()),'unsupported visual/collision parent')
        for row in rows.values():
            geometry=row.get('geometry',{});size=np.asarray(geometry.get('size',[]),dtype=float)
            require(geometry.get('type')=='BOX' and size.shape==(3,) and np.all(np.isfinite(size)) and
                    np.all(size>0),'unsupported geometry')
    require(all(v.get('opaque') is True for v in visuals.values()),'unsupported material')
    require({v['parent'] for v in visuals.values()}=={v['parent'] for v in collisions.values()}==set(links),
            'incomplete physical link inventory')
    shapes=owner.get('shapes',[]);shape_map={s.get('collision_id'):s for s in shapes}
    require(len(shape_map)==len(shapes)==len(collisions) and set(shape_map)==set(collisions),'incomplete or duplicate owner collision inventory')
    nodes=[s.get('shape_node_identity') for s in shapes];physics=[s.get('physics_shape_id') for s in shapes]
    require(all(isinstance(n,str) and n for n in nodes) and len(set(nodes))==len(nodes) and
            all(type(n) is int and n>=0 for n in physics) and len(set(physics))==len(physics) and
            all(s.get('shape_type')=='BoxShape' for s in shapes),'missing/unsupported ShapeNodes')
    rendered=renderer.get('visuals',[]);rendered_map={v.get('visual_id'):v for v in rendered}
    require(len(rendered_map)==len(rendered)==len(visuals) and set(rendered_map)==set(visuals),'missing/duplicate renderer visuals')
    native_ids=[v.get('renderer_id') for v in rendered]
    require(all(type(i) is int and 0<i<4294967295 for i in native_ids) and
            len(set(native_ids))==len(native_ids),'invalid/duplicate renderer-local IDs')
    mappings=[]
    for visual_id,visual in sorted(visuals.items()):
        link=visual['parent'];siblings=[v for v in visuals.values() if v['parent']==link]
        physical=[c for c in collisions.values() if c['parent']==link]
        require(len(siblings)==len(physical)==1,'ambiguous visual-to-collision ownership')
        collision=physical[0]['id'];shape=shape_map[collision];record=rendered_map[visual_id]
        require(record.get('link_id')==link and record.get('model_id')==links[link]['parent'],'renderer/ECM parent mismatch')
        mappings.append(dict(visual_id=visual_id,link_id=link,model_id=links[link]['parent'],
            collision_id=collision,physics_shape_id=shape['physics_shape_id'],
            shape_node_identity=shape['shape_node_identity'],renderer_id=record['renderer_id']))
    return dict(mappings=mappings,scene_fingerprint=fingerprint,
        identity_metadata_consistency='PASS',mrt_binding='BLOCKED_NOT_WITNESSED',
        contact_authority=False,execution_goals=0)
