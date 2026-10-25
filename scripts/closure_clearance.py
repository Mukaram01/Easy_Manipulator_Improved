"""Conservative candidate rejection, never physical contact permission.

Reuse the native BOX predicate for target/neighbour geometry. The actual full
robot closing trajectory is separately certified by the existing continuous
MoveIt adapter against a privately expanded copy of every dynamic neighbour.
The expansion covers the existing observation-admission bounds and maximum
planned fingertip intrusion; it is a screening allowance, not a dynamics bound.
"""
import copy
import math
from grasp_capture import DEFAULTS, target_observation_agreement
from pile_extraction import ExtractionFailure, _inputs, _geometry, _predicate, TOLERANCE_M


class ClosureClearanceFailure(ExtractionFailure):
    def __init__(self, code, **details):
        super().__init__(code, **details)
        self.reason_code='CLOSURE_CLEARANCE_REJECTED'
        self.details['failure_kind']='collision'


def check_closure_pose_bounds(boxes, measured_poses):
    by_id={item['id']:item for item in boxes}
    if len(by_id)!=len(boxes) or set(measured_poses)!=set(by_id):
        raise ValueError('missing/duplicate measured closure pose')
    evidence={}
    for name,pose in measured_poses.items():
        agreement=target_observation_agreement(pose,by_id[name]['pose'])
        if not agreement['accepted']:
            raise ValueError('measured closure pose differs from bound observation: '+name)
        evidence[name]=agreement
    return evidence


def screen_closure_neighbors(target, neighbors, measured_contacts, config=None, *, measured_poses=None):
    target,neighbors=_inputs(target,neighbors)
    cfg=dict(DEFAULTS)
    if config is not None:
        if not isinstance(config,dict) or set(config)-set(cfg):
            raise ValueError('unsupported closure uncertainty configuration')
        cfg.update(config)
    if any(isinstance(v,bool) or not math.isfinite(v) or v<=0 for v in cfg.values()):
        raise ValueError('invalid closure uncertainty configuration')
    if not isinstance(measured_contacts,dict) or set(measured_contacts)-{n['id'] for n in neighbors}:
        raise ValueError('unknown measured closure neighbour')
    boxes={item['id']:item for item in [target,*neighbors]}
    if measured_poses is None:measured_poses={name:item['pose'] for name,item in boxes.items()}
    pose_bounds=check_closure_pose_bounds([target,*neighbors],measured_poses)

    limits=target_observation_agreement(target['pose'],target['pose'])
    rotation=4*math.asin(limits['quaternion_distance_limit']/2)

    def uncertainty(box):
        # Quaternion chord d=2*sin(angle/4). Corner radius bounds movement of
        # the entire BOX under the existing pose admission, not just its centre.
        radius=math.hypot(*box['dimensions'])/2
        return limits['translation_limit_m']+2*radius*math.sin(rotation/2)

    checks=[];inflation={}
    for neighbor in neighbors:
        name=neighbor['id'];geometry=_geometry(target,neighbor)
        required=uncertainty(target)+uncertainty(neighbor)+cfg['max_penetration_m']
        physical=name in measured_contacts
        if physical:
            points=measured_contacts[name]
            if (not isinstance(points,list) or not points or any(
                    len(p)!=3 or not all(math.isfinite(x) for x in p) for p in points)):
                raise ClosureClearanceFailure('CLOSURE_CONTACT_EVIDENCE_INVALID',neighbor=name)
            import ctypes
            def array(v):return (ctypes.c_double*len(v))(*v)
            output=(ctypes.c_double*8)()
            valid=_predicate()(array(target['dimensions']),array(measured_poses[target['id']]),
                array(neighbor['dimensions']),array(measured_poses[name]),
                array([x for p in points for x in p]),len(points),output)
            if not valid or output[0]>TOLERANCE_M:
                raise ClosureClearanceFailure('CLOSURE_CONTACT_GEOMETRY_INVALID',neighbor=name,**geometry)
        elif geometry['gap_m']<=required:
            raise ClosureClearanceFailure('CLOSURE_NEIGHBOR_CLEARANCE',neighbor=name,
                                          required_clearance_m=required,**geometry)
        checks.append(dict(neighbor=name,measured_initial_contact=physical,
                           required_clearance_m=required,**geometry))
        # An existing target-support contact never removes the neighbour from
        # the full gripper sweep or grants any gripper/neighbor allowance.
        inflation[name]=required
    return dict(accepted=True,claim='REJECTION_SCREEN_ONLY',configuration=cfg,
                observation_translation_limit_m=limits['translation_limit_m'],
                observation_rotation_limit_rad=rotation,
                measured_pose_bounds=pose_bounds,
                checks=checks,neighbor_inflation_m=inflation)


def expanded_closure_scene(scene, margins):
    """Private BOX Minkowski superset; preserve all other scene/ACM data."""
    result=copy.deepcopy(scene);found=set()
    for obj in result.world.collision_objects:
        if obj.id not in margins:continue
        margin=margins[obj.id]
        if (obj.id in found or isinstance(margin,bool) or not math.isfinite(margin) or margin<=0 or
                len(obj.primitives)!=1 or obj.primitives[0].type!=1 or
                len(obj.primitives[0].dimensions)!=3 or len(obj.primitive_poses)!=1 or
                obj.meshes or obj.planes or obj.mesh_poses or obj.plane_poses):
            raise ValueError('unsupported closure neighbour expansion: '+obj.id)
        dims=list(obj.primitives[0].dimensions)
        if any(not math.isfinite(v) or v<=0 for v in dims):raise ValueError('invalid closure BOX')
        expanded=[math.nextafter(v+2*margin,math.inf) for v in dims]
        if not all(math.isfinite(v) for v in expanded):raise ValueError('unbounded closure expansion')
        obj.primitives[0].dimensions=expanded
        found.add(obj.id)
    if found!=set(margins):raise ValueError('missing closure neighbour geometry')
    return result
