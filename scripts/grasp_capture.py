"""Facet-based geometric pinch eligibility. No force or physical-retention claim.

Only BOX targets and BOX/binary-STL tool collisions are supported. A common
interior disk must fit inside two actual clipped opposing collision facets;
whole-mesh projections, hulls and contact-link membership cannot certify it.
"""
import hashlib
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

DEFAULTS = dict(patch_radius_m=.001, translation_sample_m=.00025,
                rotation_sample_rad=math.radians(.5), opposition_angle_rad=math.radians(15),
                max_penetration_m=.001, open_clearance_m=.001, mimic_tolerance_rad=.02, mimic_tolerance_m=.0001,
                joint_sample_rad=.001, joint_sample_m=.00005)


def digest(value):
    return hashlib.sha256(value).hexdigest()


def numbers(values, count):
    result=np.asarray(values,dtype=float)
    if result.shape!=(count,) or not np.isfinite(result).all():
        raise ValueError('incomplete/nonfinite geometric evidence')
    return result


def transform(pose):
    p=numbers(pose,7)
    if abs(np.linalg.norm(p[3:])-1)>1e-6:
        raise ValueError('nonunit quaternion / frame error')
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(p[3:]).as_matrix();t[:3,3]=p[:3]
    return t


def target_observation_agreement(measured_pose, observation_pose):
    """Recheck the established observation-admission limits, not capture proof."""
    transform(measured_pose);transform(observation_pose)
    measured=numbers(measured_pose,7);observed=numbers(observation_pose,7)
    translation=float(np.linalg.norm(measured[:3]-observed[:3]))
    quaternion_distance=float(min(np.linalg.norm(measured[3:]-observed[3:]),
                                  np.linalg.norm(measured[3:]+observed[3:])))
    return dict(reference='bound_observation',translation_m=translation,
                quaternion_distance=quaternion_distance,
                translation_limit_m=.00025,quaternion_distance_limit=.00025,
                accepted=translation<=.00025 and quaternion_distance<=.00025)


def origin(element):
    t=np.eye(4)
    if element is not None:
        t[:3,3]=numbers(element.get('xyz','0 0 0').split(),3)
        t[:3,:3]=Rotation.from_euler('xyz',numbers(element.get('rpy','0 0 0').split(),3)).as_matrix()
    return t


def box_triangles(size):
    half=numbers(size,3)/2
    if min(half)<=0:raise ValueError('nonpositive collision scale')
    result=[]
    for axis in range(3):
        u,v=[i for i in range(3) if i!=axis]
        for sign in (-1,1):
            vertices=[]
            for a,b in ((-1,-1),(1,-1),(1,1),(-1,1)):
                p=np.zeros(3);p[axis]=sign*half[axis];p[u]=a*half[u];p[v]=b*half[v];vertices.append(p)
            for ids in ((0,1,2),(0,2,3)):
                tri=np.array([vertices[i] for i in ids])
                if np.cross(tri[1]-tri[0],tri[2]-tri[0])[axis]*sign<0:tri=tri[::-1]
                result.append(tri)
    return np.array(result)


class CaptureModel:
    def __init__(self, urdf, touch_links):
        self.sha256=digest(urdf.encode());self.robot=ET.fromstring(urdf)
        self.links=list(touch_links);self.meshes=[];self.facets={}
        self.parents={j.find('child').get('link'):j for j in self.robot.findall('joint')}
        roots={l.get('name') for l in self.robot.findall('link')}-set(self.parents)
        if roots!={'world'}:raise ValueError('unsupported URDF root frame; explicit world root required')
        by_name={l.get('name'):l for l in self.robot.findall('link')}
        for name in self.links:
            if name not in by_name:raise ValueError('unsupported/missing contact link '+name)
            shapes=[]
            for collision in by_name[name].findall('collision'):
                geometry=collision.find('geometry');mesh=geometry.find('mesh');box=geometry.find('box')
                if box is not None:
                    triangles=box_triangles(box.get('size','').split())
                    self.meshes.append(dict(link=name,shape='BOX',size=box.get('size')))
                elif mesh is not None:
                    uri=mesh.get('filename','')
                    if uri.startswith('package://'):
                        from ament_index_python.packages import get_package_share_directory
                        package,relative=uri[10:].split('/',1);path=Path(get_package_share_directory(package))/relative
                    else:path=Path(uri.removeprefix('file://'))
                    path=path.resolve(strict=True);data=path.read_bytes()
                    if len(data)<84:raise ValueError('unsupported collision mesh; binary STL required')
                    count=struct.unpack_from('<I',data,80)[0]
                    if not count or len(data)!=84+50*count:raise ValueError('unsupported collision mesh; binary STL required')
                    triangles=np.array([struct.unpack_from('<12fH',data,84+50*i)[3:12] for i in range(count)]).reshape(-1,3,3)
                    scale=numbers(mesh.get('scale','1 1 1').split(),3)
                    if min(scale)<=0:raise ValueError('nonpositive collision mesh scale')
                    triangles*=scale
                    self.meshes.append(dict(link=name,path=str(path),sha256=digest(data),scale=scale.tolist(),triangles=count))
                else:raise ValueError('unsupported collision shape')
                if not np.isfinite(triangles).all():raise ValueError('nonfinite mesh')
                t=origin(collision.find('origin'));shapes.append(triangles@t[:3,:3].T+t[:3,3])
            if not shapes:raise ValueError('missing collision evidence: '+name)
            self.facets[name]=np.concatenate(shapes)
        if len(set(self.links))!=2:raise ValueError('unsupported contact configuration: two distinct links required')

    def fk(self, joints):
        result={'world':np.eye(4)}
        def visit(name):
            if name in result:return result[name]
            joint=self.parents[name];t=visit(joint.find('parent').get('link'))@origin(joint.find('origin'))
            kind=joint.get('type');motion=np.eye(4)
            if kind!='fixed':
                value=float(joints[joint.get('name')])
                if not math.isfinite(value):raise ValueError('nonfinite joint evidence')
                axis_element=joint.find('axis')
                axis=numbers((axis_element.get('xyz','1 0 0') if axis_element is not None else '1 0 0').split(),3)
                if abs(np.linalg.norm(axis)-1)>1e-6:raise ValueError('joint axis scale error')
                if kind in ('revolute','continuous'):motion[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix()
                elif kind=='prismatic':motion[:3,3]=axis*value
                else:raise ValueError('unsupported joint '+kind)
            result[name]=t@motion;return result[name]
        for name in self.links:visit(name)
        return result

    def triangles(self,joints,target_pose):
        fk=self.fk(joints);inverse=np.linalg.inv(transform(target_pose))
        return {name:facets@(inverse@fk[name])[:3,:3].T+(inverse@fk[name])[:3,3]
                for name,facets in self.facets.items()}

    def closing_axis(self, opened, closed, target_pose):
        # A mounted tool need not close along any particular TCP axis. Relative
        # fingertip motion removes shared translation and uses the same raw
        # URDF FK/mimic residuals as the collision facets. This selects faces;
        # the actual opposing facets, aperture and depth must still qualify.
        left,right=self.links
        open_fk=self.fk(opened);closed_fk=self.fk(closed)
        before=open_fk[right][:3,3]-open_fk[left][:3,3]
        after=closed_fk[right][:3,3]-closed_fk[left][:3,3]
        motion=before-after;distance=float(np.linalg.norm(motion))
        if not math.isfinite(distance) or distance<=1e-12:
            raise ValueError('unsupported aperture: no relative fingertip closing motion')
        world=motion/distance;target=transform(target_pose)[:3,:3].T@world
        axis=int(np.argmax(abs(target)))
        return dict(method='relative_fingertip_fk_open_to_closed',links=list(self.links),
                    open_separation_world_m=before.tolist(),closed_separation_world_m=after.tolist(),
                    relative_motion_m=distance,direction_world=world.tolist(),direction_target=target.tolist(),
                    face_axis=axis,face_angle_rad=float(math.acos(np.clip(abs(target[axis]),-1,1))))


def clip(poly, normal, offset):
    """Clip a convex polygon to dot(normal,p)<=offset in 2D or 3D."""
    if len(poly)==0:return np.empty((0,len(normal)))
    result=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        da,db=np.dot(normal,a)-offset,np.dot(normal,b)-offset
        if da<=0:result.append(a)
        if (da<0<db) or (db<0<da):result.append(a+(b-a)*da/(da-db))
    return np.array(result).reshape(-1,len(normal))


def halfplanes(poly):
    area=sum(np.cross(a,b) for a,b in zip(poly,np.roll(poly,-1,axis=0)))
    if abs(area)<1e-16:return []
    if area<0:poly=poly[::-1]
    result=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        delta=b-a;norm=np.linalg.norm(delta)
        if norm<=1e-12:continue
        normal=np.array([delta[1],-delta[0]])/norm
        result.append((normal,float(np.dot(normal,a))))
    return result


def patches(triangles, axis, sign, half, config):
    normal_axis=np.eye(3)[axis]*sign;tangents=[i for i in range(3) if i!=axis]
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    lengths=np.linalg.norm(normals,axis=1);good=lengths>1e-14
    normals[good]/=lengths[good,None]
    good &= normals@normal_axis <= -math.cos(config['opposition_angle_rad'])
    rows=[]
    for index in np.flatnonzero(good):
        polygon=clip(triangles[index],normal_axis,half[axis])
        polygon=clip(polygon,-normal_axis,-half[axis]+config['max_penetration_m'])
        for dim in tangents:
            polygon=clip(polygon,np.eye(3)[dim],half[dim])
            polygon=clip(polygon,-np.eye(3)[dim],half[dim])
        if len(polygon)<3:continue
        projected=polygon[:,tangents];planes=halfplanes(projected)
        if len(planes)<3:continue
        depths=half[axis]-polygon@normal_axis
        rows.append(dict(polygon=projected,planes=planes,normal=normals[index],facet=int(index),
                         depths=depths,min=projected.min(axis=0),max=projected.max(axis=0)))
    return rows


def sample_capture(closed, opened, closing_direction, dimensions, config):
    half=numbers(dimensions,3)/2
    if min(half)<=0:raise ValueError('nonpositive target scale')
    closing=numbers(closing_direction,3)
    axis=int(np.argmax(abs(closing)))
    if abs(closing[axis])<math.cos(config['opposition_angle_rad']):
        raise ValueError('unsupported closing-axis opposition to target faces')
    tangent=[i for i in range(3) if i!=axis];radius=config['patch_radius_m']
    links=list(closed);best=None;has_contact=False;has_aperture=False;excess_depth=None
    for left,right in (links,links[::-1]):
        negative=patches(closed[left],axis,-1,half,config);positive=patches(closed[right],axis,1,half,config)
        if not negative or not positive:continue
        has_contact=True
        # A selected good facet cannot conceal a deeper tool protrusion. This
        # whole-shape half-space bound is conservative even for concave meshes.
        deepest=max(closed[left][:,:,axis].max()+half[axis],half[axis]-closed[right][:,:,axis].min())
        if deepest>config['max_penetration_m']:
            excess_depth=float(deepest);continue
        gap_left=-half[axis]-opened[left][:,:,axis].max()
        gap_right=opened[right][:,:,axis].min()-half[axis]
        if min(gap_left,gap_right)<config['open_clearance_m']:continue
        has_aperture=True
        for a in negative:
            for b in positive:
                if np.dot(a['normal'],b['normal'])>-math.cos(config['opposition_angle_rad']):continue
                if min(np.minimum(a['max'],b['max'])-np.maximum(a['min'],b['min']))<2*radius:continue
                planes=a['planes']+b['planes']
                # Target edge/depth margins are enforced independently of facets.
                for dim in range(2):
                    n=np.eye(2)[dim];planes.extend([(n,half[tangent[dim]]),(-n,half[tangent[dim]])])
                poly=np.array([[-half[tangent[0]],-half[tangent[1]]],[half[tangent[0]],-half[tangent[1]]],
                               [half[tangent[0]],half[tangent[1]]],[-half[tangent[0]],half[tangent[1]]]])
                for n,d in planes:
                    poly=clip(poly,n,d-radius)
                    if not len(poly):break
                if not len(poly):continue
                center=poly.mean(axis=0);margin=min(d-n@center for n,d in planes)
                row=dict(links=[left,right],facets=[a['facet'],b['facet']],axis=axis,
                         patch_radius_m=float(margin),center_tangent_m=center.tolist(),
                         edge_margin_m=float(min(half[tangent]-abs(center))),
                         penetration_range_m=[float(min(a['depths'].min(),b['depths'].min())),float(max(a['depths'].max(),b['depths'].max()))],
                         open_clearance_m=float(min(gap_left,gap_right)),
                         opposition_angle_rad=float(math.acos(np.clip(-a['normal']@b['normal'],-1,1))))
                if best is None or row['patch_radius_m']>best['patch_radius_m']:best=row
    if best is None:
        if excess_depth is not None:
            raise ValueError(f'closed tool penetration {excess_depth:.9g} m exceeds {config["max_penetration_m"]:.9g} m')
        raise ValueError('no compatible opposing contact regions' if not has_contact else
                         'open aperture lacks target clearance' if not has_aperture else
                         'edge/depth mismatch: no common interior contact patch')
    return best


def qualify_capture(*,model,target,candidate_pose,measured,planned_open,planned_closed,binding,now_ns,config=None):
    result=dict(accepted=False,claim='GEOMETRIC_ONLY',robustness='SAMPLED_NOT_CONTINUOUS_PROOF',
                binding=dict(binding),measured=measured,mesh_identity=model.meshes,samples=[])
    try:
        cfg=dict(DEFAULTS)
        if config is not None:
            if not isinstance(config,dict):raise ValueError('geometric configuration must be a mapping')
            if set(config)-set(cfg):raise ValueError('unknown geometric tolerance')
            cfg.update(config)
        if any(isinstance(v,bool) or not math.isfinite(v) or v<=0 for v in cfg.values()):
            raise ValueError('invalid geometric tolerance')
        if cfg['opposition_angle_rad']>=math.pi/2:raise ValueError('invalid opposition tolerance')
        result['configuration']=cfg
        for key in ('scene_sha256','observation_sha256','robot_model_sha256','srdf_sha256','receipt_sha256'):
            value=binding.get(key)
            if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):
                raise ValueError('missing identity evidence: '+key)
        if binding['robot_model_sha256']!=model.sha256 or not binding.get('candidate_id') or binding.get('target_id')!=target['id']:
            raise ValueError('stale or mismatched model/candidate evidence')
        if target.get('shape')!='BOX':raise ValueError('unsupported target shape; explicit BOX evidence required')
        for mesh in model.meshes:
            if 'path' in mesh and digest(Path(mesh['path']).read_bytes())!=mesh['sha256']:
                raise ValueError('stale collision mesh evidence')
        if target['frame_id']!='world':raise ValueError('target frame must be world')
        if (measured.get('kind')!='MEASURED' or not isinstance(measured.get('wall_ns'),int) or
                type(measured.get('monotonic_raw_ns')) is not int):
            raise ValueError('missing measured evidence')
        # now_ns is CLOCK_MONOTONIC_RAW: a wall-clock step must not age or refresh evidence.
        if not 0<=now_ns-measured['monotonic_raw_ns']<=250_000_000:raise ValueError('stale measured evidence (>250 ms)')
        # Facets are evaluated at the immutable bound observation, not at a
        # substitute simulator pose. Admission at entry and completion uses
        # the same original reference; sampled robustness is not a drift proof.
        result['target_reference_pose']=list(target['pose'])
        result['measurement_target_agreement']=target_observation_agreement(
            measured['target_pose'],result['target_reference_pose'])
        if not result['measurement_target_agreement']['accepted']:
            raise ValueError('target geometry changed since bound observation')
        raw=measured['joints'];measured_fk=model.fk(raw)
        frame_errors={}
        for name in model.links:
            observed=transform(measured['link_poses'][name])
            delta=np.linalg.inv(measured_fk[name])@observed
            translation=float(np.linalg.norm(delta[:3,3]))
            angle=float(Rotation.from_matrix(delta[:3,:3]).magnitude())
            frame_errors[name]=dict(translation_m=translation,angle_rad=angle)
            if translation>cfg['translation_sample_m'] or angle>cfg['rotation_sample_rad']:
                raise ValueError(f'measured frame disagrees with raw joint FK: {name}, {translation:.9g} m, {angle:.9g} rad')
        result['measured_frame_errors']=frame_errors
        opened=dict(planned_open);closed=dict(planned_closed);residuals={}
        mimic_joints={j.get('name'):j for j in model.robot.findall('joint') if j.find('mimic') is not None}
        for name,joint in mimic_joints.items():
            mimic=joint.find('mimic');parent=mimic.get('joint')
            scale=float(mimic.get('multiplier','1'));offset=float(mimic.get('offset','0'))
            residual=float(raw[name])-float(raw[parent])*scale-offset
            if not math.isfinite(residual) or abs(residual)>cfg['mimic_tolerance_m' if joint.get('type')=='prismatic' else 'mimic_tolerance_rad']:
                raise ValueError('measured mimic discrepancy '+name+': '+str(residual))
            residuals[name]=residual
            for planned in (planned_open,planned_closed):
                if abs(float(planned[name])-float(planned[parent])*scale-offset)>1e-9:
                    raise ValueError('inconsistent planned mimic state: '+name)
        # Evaluate dependencies in order, retaining every measured discrepancy.
        def apply_residual(name, state, visiting, done):
            if name in done or name not in mimic_joints:return
            if name in visiting:raise ValueError('cyclic mimic evidence')
            visiting.add(name);mimic=mimic_joints[name].find('mimic');parent=mimic.get('joint')
            apply_residual(parent,state,visiting,done)
            state[name]=state[parent]*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))+residuals[name]
            visiting.remove(name);done.add(name)
        for state in (opened,closed):
            done=set()
            for name in mimic_joints:apply_residual(name,state,set(),done)
        result['planned']=dict(kind='PLANNED',open_joints=planned_open,closed_joints=planned_closed,
                               raw_mimic_residuals=residuals,evaluated_open_joints=opened,evaluated_closed_joints=closed)
        pose=list(target['pose']);transform(pose);transform(candidate_pose);model.fk(opened);model.fk(closed)
        variants=[('nominal',pose,closed)]
        for dim in range(3):
            for sign in (-1,1):
                moved=list(pose);moved[dim]+=sign*cfg['translation_sample_m'];variants.append((f'translation_{dim}_{sign}',moved,closed))
                rotated=list(pose);delta=np.eye(3)[dim]*sign*cfg['rotation_sample_rad']
                rotated[3:]=(Rotation.from_rotvec(delta)*Rotation.from_quat(pose[3:])).as_quat().tolist()
                variants.append((f'rotation_{dim}_{sign}',rotated,closed))
        # Sample each moving joint on the two tool branches beyond arm/common joints.
        def branch(link):
            names=[]
            while link in model.parents:
                joint=model.parents[link];names.append(joint.get('name'))
                link=joint.find('parent').get('link')
            return set(names)
        branches=[branch(link) for link in model.links]
        tool_joints=set.union(*branches)-set.intersection(*branches)
        moving={j.get('name'):j for j in model.robot.findall('joint')
                if j.get('name') in tool_joints and j.get('type')!='fixed'}
        if not moving:raise ValueError('unsupported aperture: no moving tool branches')
        for name,joint in sorted(moving.items()):
            for sign in (-1,1):
                perturbed=dict(closed);perturbed[name]+=sign*cfg['joint_sample_m' if joint.get('type')=='prismatic' else 'joint_sample_rad']
                variants.append((f'joint_{name}_{sign}',pose,perturbed))
        for label,p,q in variants:
            closed_facets=model.triangles(q,p);open_facets=model.triangles(opened,p)
            axis_evidence={}
            try:
                axis_evidence=model.closing_axis(opened,q,p)
                margins=sample_capture(closed_facets,open_facets,axis_evidence['direction_target'],target['dimensions'],cfg)
                result['samples'].append(dict(sample=label,accepted=True,closing_axis=axis_evidence,**margins))
            except ValueError as exc:
                result['samples'].append(dict(sample=label,accepted=False,reason=str(exc),closing_axis=axis_evidence,
                    closed_bounds_in_target_m={name:[facets.reshape(-1,3).min(axis=0).tolist(),
                        facets.reshape(-1,3).max(axis=0).tolist()] for name,facets in closed_facets.items()},
                    target_dimensions_m=list(target['dimensions'])))
                raise ValueError(label+': '+str(exc)) from exc
        result.update(accepted=True,reason='opposing interior facet regions qualify for every sampled perturbation',
                      minimum_patch_radius_m=min(s['patch_radius_m'] for s in result['samples']))
    except (ValueError,KeyError,TypeError,OSError) as exc:
        result['reason']=str(exc)
    return result


def confirm_capture(result, current, binding, now_ns):
    """Confirm fresh evidence for the unchanged geometric calculation inputs.

    Joints/frames retain numeric equality because they supply FK and mimic
    evidence. The target must still agree with the original observation used
    for the facets. This admission limit does not prove continuous robustness.
    The original sample is never re-stamped or replaced by the completion one.
    """
    result['completion_measurement']=current
    result['completion_checked_monotonic_raw_ns']=now_ns
    try:
        if not result.get('accepted'):return result
        if (current.get('kind')!='MEASURED' or type(current.get('monotonic_raw_ns')) is not int or
                not 0<=now_ns-current['monotonic_raw_ns']<=250_000_000):
            raise ValueError('stale completion measurement evidence')
        for key in ('robot_model_sha256','srdf_sha256'):
            if binding.get(key)!=result['binding'][key]:
                raise ValueError('model changed during geometric qualification: '+key)
        previous=result['measured']
        if set(current['joints'])!=set(previous['joints']):
            raise ValueError('measured joint evidence changed during qualification')
        for name,value in previous['joints'].items():
            if not math.isfinite(current['joints'][name]) or abs(current['joints'][name]-value)>1e-9:
                raise ValueError('measured state changed during qualification: '+name)
        for name,pose in previous['link_poses'].items():
            if not np.allclose(transform(current['link_poses'][name]),transform(pose),atol=1e-9,rtol=0):
                raise ValueError('measured frame changed during qualification: '+name)
        result['completion_target_agreement']=target_observation_agreement(
            current['target_pose'],result['target_reference_pose'])
        if not result['completion_target_agreement']['accepted']:
            raise ValueError('target geometry changed since bound observation')
        result['confirmed_monotonic_raw_ns']=now_ns
    except (ValueError,KeyError,TypeError) as exc:
        result.update(accepted=False,reason=str(exc))
    return result
