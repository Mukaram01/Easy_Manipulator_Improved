#!/usr/bin/env python3
"""Evaluate isolated Fortress contact state, never replace EPD planning geometry.

Exact rational arithmetic encloses all BOX corners against horizontal support.
ECM and live-owner readbacks have separate authority scopes. Neither bypasses
unqualified detector numerics. No caller scalar can grant backend authority.
"""
import argparse
import hashlib
import itertools
import json
import math
from fractions import Fraction as F
from pathlib import Path

import numpy as np
from stage_a_relative_support import _rotation


def require(condition, reason):
    if not condition:
        raise ValueError('BLOCKED: physics contact: '+reason)


def vector(value, size):
    require(isinstance(value,list) and len(value)==size and
            all(type(v) in (int,float) and math.isfinite(v) for v in value),'finite vector required')
    return [F(v) for v in value]


def outward(value, upper):
    result=float(value)
    require(math.isfinite(result),'numeric range exceeded')
    if (upper and F(result)<value) or (not upper and F(result)>value):
        result=math.nextafter(result,math.inf if upper else -math.inf)
    return result


def corners(pair):
    require(pair.get('shape')=='BOX' and pair.get('support_shape')=='PLANE','unsupported collision geometry')
    require(type(pair.get('collision_id')) is int and pair['collision_id']>0 and
            type(pair.get('support_collision_id')) is int and pair['support_collision_id']>0 and
            pair['support_collision_id']!=pair['collision_id'],'distinct exact collision IDs required')
    dims=vector(pair.get('dimensions_m'),3)
    require(all(d>0 for d in dims),'positive BOX dimensions required')
    centre=vector(pair.get('centre_world'),3)
    axes=[vector(row,3) for row in pair.get('axes_world',[])]
    require(len(axes)==3,'complete rotation matrix required')
    # A validity check, not a backend accuracy bound. Numeric geometry uses the
    # supplied finite binary matrix exactly; no orthogonality is assumed there.
    matrix=np.asarray(axes,dtype=float)
    require(np.max(np.abs(matrix.T@matrix-np.eye(3)))<1e-10 and
            np.linalg.det(matrix)>0,'unsupported non-rigid rotation matrix')
    require(vector(pair.get('support_normal_world'),3)==[F(0),F(0),F(1)],
            'only exactly horizontal upward support plane implemented')
    origin=vector(pair.get('support_point_world'),3)
    result=[]
    for signs in itertools.product((-1,1),repeat=3):
        result.append([centre[i]+sum(axes[i][j]*signs[j]*dims[j]/2 for j in range(3)) for i in range(3)])
    return result,origin


def enclose_box_plane(pair):
    points,origin=corners(pair)
    gap=min(p[2]-origin[2] for p in points)
    low,high=outward(gap,False),outward(gap,True)
    penetration=outward(max(F(0),-gap),True)
    return dict(evaluated_corner_count=8,minimum_gap_interval_m=[low,high],
        conditional_penetration_upper_m=penetration,
        arithmetic_enclosure_width_m=outward(F(high)-F(low),True),
        arithmetic_authority='exact rational evaluation of reported binary ECM centre/matrix/dimensions',
        backend_state_error_bound_m=None,physical_penetration_upper_m=None,
        decision='BLOCKED_EXCESSIVE_PENETRATION' if penetration>.0001 else 'BLOCKED_BACKEND_STATE_ERROR',
        reason='DART state to cached ECM pose conversion/notification error not qualified; '
               'even shallow conditional geometry cannot authorize contact')


def enclose_owner_geometry(owner, observer, session, step):
    """Enclose actual stored DART affine BOX geometry, without detector authority.

    IEEE binary readback is verified by hexadecimal doubles. Rational arithmetic
    adds zero arithmetic error; final JSON endpoints are rounded outwards.
    ODE's matrix -> quaternion -> detector rotation conversion is NOT covered.
    """
    require(owner.get('complete') is True and observer.get('complete') is True,
            'incomplete backend inventory')
    require(owner.get('session')==observer.get('run_id')==session and
            owner.get('world')==observer.get('world') and
            owner.get('frame_id')==observer.get('frame_id')=='world', 'owner scene/frame identity')
    require(type(step) is int and step>0 and owner.get('step')==observer.get('step')==step and
            owner.get('dart_frames')==step and owner.get('pre_step_dart_frames')==step-1 and
            observer.get('contact_request_step')==step and
            owner.get('geometry_state_phase')=='post_ForwardStep_position_integration', 'stale physics step')
    require(owner.get('dt_ns')==observer.get('dt_ns') and type(owner.get('dt_ns')) is int and
            owner['dt_ns']>0 and owner.get('stamp_ns')==observer.get('stamp_ns')==step*owner['dt_ns'],
            'physics time mismatch; only fixed-dt campaign supported')
    shapes=owner.get('shapes',[]);ids=[v.get('collision_id') for v in shapes]
    physics=[v.get('physics_shape_id') for v in shapes];nodes=[v.get('shape_node_identity') for v in shapes]
    requested=observer.get('diagnostic_requested_ids',[]);inventory=observer.get('diagnostic_collision_ids',[])
    require(bool(ids) and all(type(v) is int and v>0 for v in ids+physics) and
            len(ids)==len(set(ids))==len(set(physics))==len(set(nodes)) and all(nodes) and
            len(requested)==len(set(requested)) and len(inventory)==len(set(inventory)) and
            set(ids)==set(inventory)==set(requested),'non-bijective collision inventory')
    by_id={v['collision_id']:v for v in shapes}
    support_id=observer.get('support_collision_id');support=by_id.get(support_id)
    require(support is not None and support.get('mobile') is False and
            support.get('collision_name')==observer.get('support_collision_name'),'wrong support mapping')
    pairs=observer.get('pairs',[]);cubes=[v.get('collision_id') for v in pairs]
    require(bool(cubes) and len(cubes)==len(set(cubes)) and
            set(cubes)=={v['collision_id'] for v in shapes if v.get('mobile') is True},
            'partial or ambiguous dynamic inventory')
    node_ids={v['shape_node_identity']:v['collision_id'] for v in shapes}
    contacts=owner.get('contacts',[]);seen=set()
    require(bool(contacts),'missing contact data')
    for c in contacts:
        a,b=c.get('collision1'),c.get('collision2')
        require(a!=b and a in by_id and b in by_id and
                node_ids.get(c.get('node1'))==a and node_ids.get(c.get('node2'))==b,
                'incorrect contact ShapeNode mapping')
        vector(c.get('position_m'),3);vector(c.get('normal'),3)
        require(type(c.get('depth_m')) in (int,float) and math.isfinite(c['depth_m']), 'missing contact depth')
        seen.add(frozenset((a,b)))
    require(all(frozenset((c,support_id)) in seen for c in cubes),'missing cube/support contact')

    def exact(values, hexes):
        vals=vector(values,len(values))
        require(isinstance(hexes,list) and len(hexes)==len(vals), 'missing exact binary geometry')
        require(all(isinstance(h,str) and math.isfinite(float.fromhex(h)) and
                    F(float.fromhex(h))==v for h,v in zip(hexes,vals)), 'unqualified numeric conversion')
        return vals

    def geometry(shape,key):
        require(shape.get('shape_type')=='BoxShape', 'unsupported backend shape')
        dims=exact(shape.get('dimensions_m',[]),shape.get('dimensions_hex'))
        require(len(dims)==3 and all(v>0 for v in dims), 'invalid backend BOX')
        mat=shape.get(key,[]);hexes=shape.get(key+'_hex',[])
        require(len(mat)==len(hexes)==4 and all(len(row)==4 for row in mat), 'incomplete transform')
        matrix=[exact(row,h) for row,h in zip(mat,hexes)]
        require(matrix[3]==[0,0,0,1], 'invalid affine transform')
        return dims,matrix

    # This finite campaign supports only a complete BOX collision inventory.
    for shape in shapes:
        require(type(shape.get('mobile')) is bool, 'missing skeleton mobility')
        for key in ('pre_step_world_transform','world_transform'):
            geometry(shape,key)
    results=[]
    for observed in pairs:
        cube=by_id[observed['collision_id']]
        require(cube.get('collision_name')==observed.get('collision_name') and
                cube.get('dimensions_m')==observed.get('dimensions_m'), 'cube geometry provenance mismatch')
        result=dict(collision_id=cube['collision_id'],support_collision_id=support_id,
                    shape_node_identity=cube['shape_node_identity'],support_node_identity=support['shape_node_identity'])
        for phase,key in [('contact_evaluation_input','pre_step_world_transform'),('post_step','world_transform')]:
            sd,st=geometry(support,key);cd,ct=geometry(cube,key)
            require([row[:3] for row in st[:3]]==[[1,0,0],[0,1,0],[0,0,1]],
                    'unsupported constructed support rotation')
            top=st[2][3]+sd[2]/2
            require(sd==[F(2100)]*3 and top==0 and st[0][3]==st[1][3]==0,
                    'constructed support differs from qualified two-cube campaign')
            pair=dict(shape='BOX',support_shape='PLANE',collision_id=cube['collision_id'],
                support_collision_id=support_id,dimensions_m=cube['dimensions_m'],
                centre_world=[float(ct[j][3]) for j in range(3)],
                axes_world=[[float(v) for v in row[:3]] for row in ct[:3]],
                support_normal_world=[0.,0.,1.],support_point_world=[0.,0.,0.])
            points,_=corners(pair)
            require(all(st[j][3]-sd[j]/2<=p[j]<=st[j][3]+sd[j]/2 for p in points for j in (0,1)),
                    'BOX exceeds finite support footprint')
            gap=min(p[2]-top for p in points);penetration=max(F(0),-gap)
            require(penetration<=F(1,10000), 'whole-shape penetration exceeds unchanged 0.1 mm limit')
            lo,hi=outward(gap,False),outward(gap,True)
            result[phase]=dict(evaluated_corner_count=8,minimum_gap_interval_m=[lo,hi],
                stored_dart_penetration_upper_m=outward(penetration,True),
                exact_arithmetic_error_m=0,output_enclosure_width_m=outward(F(hi)-F(lo),True),
                support_top_interval_m=[outward(top,False),outward(top,True)],
                support_dimensions_m=support['dimensions_m'])
        results.append(result)
    return dict(scope='instrumented_runtime_stored_DART_affine_geometry_only',step=step,
        stamp_ns=owner['stamp_ns'],session=session,stored_geometry_decision='PASS',pairs=results,
        contact_authority=False,physical_penetration_upper_m=None,backend_conversion_error_bound_m=None,
        decision='BLOCKED_COLLISION_DETECTOR_TRANSFORM_ERROR',
        outstanding_proof='bound DART matrix-to-quaternion and ODE detector transform conversion; '
            'exact stored ShapeNode geometry does not attest the detector representation')


def projection_bounds(pair,source):
    points,_=corners(pair)
    camera=vector(source.get('camera_world_pose'),7)
    q=[float(v) for v in camera[3:]]
    # This matrix computation is deliberately unqualified. Rational projection
    # encloses its reported binary coefficients, not the renderer camera state.
    rotation=_rotation(q)@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    matrix=[[F(float(v)) for v in row] for row in rotation]
    fx,fy,cx,cy=vector(source.get('intrinsics'),4)
    require(fx>0 and fy>0,'positive camera focal lengths required')
    pixels=[]
    for point in points:
        delta=[point[i]-camera[i] for i in range(3)]
        optical=[sum(delta[i]*matrix[i][j] for i in range(3)) for j in range(3)]
        require(optical[2]>0,'BOX crosses camera projection singularity')
        pixels.append([fx*optical[0]/optical[2]+cx,fy*optical[1]/optical[2]+cy])
    return [outward(min(p[0] for p in pixels),False),outward(min(p[1] for p in pixels),False),
            outward(max(p[0] for p in pixels),True),outward(max(p[1] for p in pixels),True)]


def bind_contact_observations(record,snapshot,masks):
    source=snapshot['source'];stamp=snapshot['timestamp'];binding=source.get('physics_measurement',{})
    require(snapshot.get('frame_id')=='stage_a_camera_optical_frame' and
            source.get('clock_domain')=='gazebo_simulation' and
            type(source.get('acquisition_clock_ns')) is int and
            type(stamp) is int and 0<=source['acquisition_clock_ns']-stamp<=1000000000,
            'fresh simulation capture and optical frame required')
    require(record.get('schema')=='workcell_physics_contact_state/v1' and
            record.get('frame_id')=='world' and record.get('complete') is True and
            record.get('state_source')=='Fortress_PostUpdate_ECM','complete supported provider record required')
    require(type(record.get('step')) is int and record['step']>0 and
            type(record.get('dt_ns')) is int and record['dt_ns']>0 and
            record.get('stamp_ns')==stamp and
            all(source.get(k)==stamp for k in ('rgb_stamp_ns','depth_stamp_ns','info_stamp_ns')) and
            all(binding.get(k)==record.get(k) for k in ('run_id','world','step','stamp_ns',
                'support_collision_id','support_collision_name')),
            'exact capture-step/session binding required; no extrapolation allowed')
    require(source.get('camera_is_static') is True and
            source.get('camera_pose_source')=='live_scene_info' and
            source.get('camera_definition_source')=='live_generate_world_sdf',
            'verified static camera definition required')
    loaded=source.get('simulation_asset_geometry',{})
    require(loaded.get('complete') is True and loaded.get('world')==record['world'] and
            loaded.get('capture_stamp_ns')==stamp,'loaded scene identity mismatch')
    dynamic=[m for m in loaded.get('models',[]) if m.get('static') is False]
    pairs=record.get('pairs',[])
    require(pairs and len(pairs)==len(dynamic) and
            {p.get('model_name') for p in pairs}=={m.get('name') for m in dynamic} and
            len({p.get('collision_id') for p in pairs})==len(pairs),'incomplete collision inventory')
    objects=snapshot.get('objects',[])
    ids=[o.get('object_id') for o in objects]
    require(ids and len(set(ids))==len(ids) and set(masks)==set(ids),'exact EPD mask inventory required')
    for mask in masks.values():
        require(isinstance(mask,np.ndarray) and mask.ndim==2 and np.any(mask) and
                mask.shape==(source.get('height'),source.get('width')),'matching nonempty EPD mask required')
    results=[];owners={i:[] for i in ids}
    for pair in pairs:
        model=next(m for m in dynamic if m['name']==pair['model_name'])
        expected='::'.join((record['world'],model['name'],model.get('link',''),model.get('collision','')))
        require(model.get('supported') is True and pair.get('collision_name')==expected and
                pair.get('dimensions_m')==model.get('dimensions_m'),'collision identity/geometry mismatch')
        support=pair.get('support_collision_name','').split('::')
        require(len(support)==4 and support[0]==record['world'] and
                pair.get('support_collision_id')==record.get('support_collision_id') and
                pair.get('support_collision_name')==record.get('support_collision_name') and
                any(m.get('name')==support[1] and m.get('static') is True for m in loaded.get('models',[])),
                'support identity mismatch')
        bounds=projection_bounds(pair,source)
        candidates=[]
        for identity,mask in masks.items():
            v,u=np.nonzero(mask)
            # Conservative candidate graph uses pixel cells intersecting the
            # enclosing projection rectangle. It cannot certify mask accuracy.
            if np.any((u+.5>=bounds[0])&(u-.5<=bounds[2])&(v+.5>=bounds[1])&(v-.5<=bounds[3])):
                candidates.append(identity);owners[identity].append(pair['collision_id'])
        require(len(candidates)==1,'ambiguous or missing EPD projection association')
        results.append(dict(collision_id=pair['collision_id'],collision_name=pair['collision_name'],
            support_collision_id=pair['support_collision_id'],support_collision_name=pair['support_collision_name'],
            dimensions_m=pair['dimensions_m'],support_plane_world=dict(point=pair['support_point_world'],
                normal=pair['support_normal_world']),contact_normal_source='geometric support normal; no force/contact assertion',
            geometry=enclose_box_plane(pair),candidate_object_ids=candidates,
            conditional_projection_bounds_uv=bounds,binding='BLOCKED_PROJECTION_UNQUALIFIED',
            binding_reason='binary-state projection only; backend/renderer state, intrinsics, '
                'mask error and within-exposure synchronization bounds unavailable'))
    require(all(len(v)==1 for v in owners.values()),'EPD association is not bijective')
    return dict(schema='workcell_physics_contact_evidence/v1',scope='simulation_only_replay',
        run_id=record['run_id'],world=record['world'],step=record['step'],stamp_ns=stamp,
        contact_authority=False,decision='BLOCKED',pairs=results,
        outstanding_proof='qualified DART-to-ECM shape state error and renderer-to-physics capture-step '
            'projection/mask association; no numeric scalar override is accepted')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import cv2
    record_path=args.capture/'physics_measurement.json'
    snapshot_path=args.capture/'snapshot.json'
    inputs=[record_path,snapshot_path]
    try:
        record=json.loads(record_path.read_text());snapshot=json.loads(snapshot_path.read_text())
        masks={}
        for obj in snapshot['objects']:
            filename=obj.get('attributes',{}).get('mask_file')
            require(isinstance(filename,str) and Path(filename).name==filename,'capture-owned mask filename required')
            inputs.append(args.capture/filename)
            masks[obj['object_id']]=cv2.imread(str(inputs[-1]),cv2.IMREAD_GRAYSCALE)
        report=bind_contact_observations(record,snapshot,masks)
    except (OSError,ValueError,KeyError,TypeError) as exc:
        report=dict(schema='workcell_physics_contact_evidence/v1',scope='simulation_only_replay',
            contact_authority=False,decision='BLOCKED',failure_reason=str(exc))
    report['input_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in inputs if p.is_file()}
    with args.output.open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
    print('BLOCKED: '+report.get('failure_reason','whole-BOX conditional enclosure computed; backend state and EPD projection unqualified'))
    return 2


if __name__=='__main__':
    raise SystemExit(main())
