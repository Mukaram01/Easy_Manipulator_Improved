#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, re, shutil, subprocess, math
from pathlib import Path
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = ROOT / "scenes"
EXTRACTOR_VERSION = "2.3"
PLACEHOLDER_RE = re.compile(r"(\$\{[^}]+\}|\$\(arg\s+[^)]+\)|\$\(find\s+[^)]+\))")

def read_yaml(path):
    try:return yaml.safe_load(Path(path).read_text()) if Path(path).exists() else None
    except Exception:return None

def parse_vec(text,n,default=0.0):
    vals=[default]*n
    if not text:return vals
    for i,p in enumerate(re.split(r"\s+",str(text).strip())[:n]):
        try:vals[i]=float(p)
        except Exception:pass
    return vals

def identity_tf(): return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
def matmul4(a,b): return [[sum(a[r][k]*b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]
def tf_from_xyz_rpy(xyz,rpy):
    x,y,z=(xyz+[0,0,0])[:3]; rr,pp,yy=(rpy+[0,0,0])[:3]
    cr,sr=math.cos(rr),math.sin(rr); cp,sp=math.cos(pp),math.sin(pp); cy,sy=math.cos(yy),math.sin(yy)
    rot=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]; out=identity_tf()
    for r in range(3):
        for c in range(3): out[r][c]=rot[r][c]
    out[0][3],out[1][3],out[2][3]=x,y,z; return out

def xyz_rpy_from_tf(tf):
    x,y,z=tf[0][3],tf[1][3],tf[2][3]; sy=-tf[2][0]; pitch=math.asin(max(-1,min(1,sy))); cp=math.cos(pitch)
    if abs(cp)>1e-8: roll=math.atan2(tf[2][1],tf[2][2]); yaw=math.atan2(tf[1][0],tf[0][0])
    else: roll=math.atan2(-tf[1][2],tf[1][1]); yaw=0.0
    return {"xyz":[x,y,z],"rpy":[roll,pitch,yaw]}

def contains_placeholder(v): return bool(PLACEHOLDER_RE.search(json.dumps(v) if isinstance(v,(list,tuple,dict)) else str(v)))
def sanitize(s): return re.sub(r'[^A-Za-z0-9_]+','_',str(s or 'unnamed')).strip('_') or 'unnamed'
def tag_name(e): return e.tag.split('}')[-1]

def discover_package_map(scene_dir, workspace_root=None):
    out={}
    diagnostics={'resolved_packages':[], 'shadowed_packages':[], 'resolution_paths':[]}
    ws = Path(workspace_root) if workspace_root else None
    roots=[
        ('scene_generated_package', scene_dir/'generated'),
        ('workspace_install_share', (ws/'install/share') if ws else None),
        ('workspace_src_easy_manipulation_deployment_assets', (ws/'src/easy_manipulation_deployment/assets') if ws else None),
        ('workspace_src_assets', (ws/'src/assets') if ws else None),
        ('repo_assets', ROOT/'assets'),
        ('ros_humble_share', Path('/opt/ros/humble/share')),
    ]
    for tier, root in roots:
        if root is None or not root.exists():
            continue
        for pkg_xml in root.rglob('package.xml'):
            pkg_dir = pkg_xml.parent
            pkg_name = pkg_dir.name
            parse_error = ''
            try:
                xml_root = ET.parse(pkg_xml).getroot()
                name_node = xml_root.find('name')
                if name_node is not None and (name_node.text or '').strip():
                    pkg_name = name_node.text.strip()
            except Exception as e:
                parse_error = str(e)
            candidate={'package_name':pkg_name,'root_path':str(pkg_dir),'source_tier':tier,'package_xml':str(pkg_xml)}
            if parse_error:
                candidate['name_fallback']='directory_name'
                candidate['parse_error']=parse_error
            if pkg_name in out:
                diagnostics['shadowed_packages'].append({
                    **candidate,
                    'shadowed_by':str(out[pkg_name]),
                })
                continue
            out[pkg_name]=pkg_dir
            diagnostics['resolved_packages'].append(candidate)
            diagnostics['resolution_paths'].append({'package_name':pkg_name,'root_path':str(pkg_dir),'source_tier':tier})
    return out, diagnostics

def xacro_env(scene_dir, workspace_root=None):
    rp=[str(ROOT),str(ROOT/'assets'),str(ROOT/'workcell_builder/workcell_builder/assets'),str(ROOT/'scenes')]
    if workspace_root:
        ws = Path(workspace_root); rp += [str(ws/'install'), str(ws/'install/share'), str(ws/'src'), str(ws/'src/easy_manipulation_deployment/assets'), str(ws/'src/assets')]
    if Path('/opt/ros/humble/share').exists(): rp.append('/opt/ros/humble/share')
    old=os.environ.get('ROS_PACKAGE_PATH','')
    if old: rp.append(old)
    env=dict(os.environ); env['ROS_PACKAGE_PATH']=':'.join(dict.fromkeys([p for p in rp if p])); env['AMENT_PREFIX_PATH']=os.environ.get('AMENT_PREFIX_PATH',''); return env

def discover_xacro_command():
    if shutil.which('xacro'): return ['xacro'], True, ''
    if Path('/opt/ros/humble/bin/xacro').exists(): return ['/opt/ros/humble/bin/xacro'], True, ''
    mod = subprocess.run(['python3', '-m', 'xacro', '--help'], capture_output=True, text=True)
    if mod.returncode == 0: return ['python3', '-m', 'xacro'], True, ''
    return None, False, "xacro executable unavailable"

def expand_xacro(urdf_path, scene_dir=None, xacro_args=None, workspace_root=None):
    scene_dir = scene_dir or Path(urdf_path).parents[1]; xacro_args = xacro_args or {}
    xacro_base, xacro_available, reason = discover_xacro_command()
    if not xacro_available: return None,False,reason,None
    cmd=xacro_base+[str(urdf_path),'-o',str(scene_dir/'generated'/'expanded_scene_preview.urdf')]
    for k,v in xacro_args.items(): cmd.append(f'{k}:={v}')
    try:
        (scene_dir/'generated').mkdir(parents=True,exist_ok=True)
        p=subprocess.run(cmd,capture_output=True,text=True,timeout=45,env=xacro_env(scene_dir, workspace_root=workspace_root))
        if p.returncode!=0: return None,True,(p.stderr or p.stdout or 'xacro failed').strip(),cmd
        out=scene_dir/'generated'/'expanded_scene_preview.urdf'; return out.read_text(errors='ignore'),True,'',cmd
    except Exception as e: return None,True,f'xacro expansion failed: {e}',cmd

def resolve_mesh_uri(uri, package_map):
    u=(uri or '').strip()
    if not u: return '', 'file_not_found'
    if u.startswith('package://'):
        rel=u[len('package://'):]; pkg, _, tail = rel.partition('/')
        if pkg not in package_map: return '', 'package_not_found'
        p=package_map[pkg]/tail; return str(p), ('file_not_found' if not p.exists() else '')
    p=Path(u); return str(p), ('file_not_found' if not p.exists() else '')

def extract_from_urdf(xml_text, package_map):
    root=ET.fromstring(xml_text); items=[]; idx=0
    links={l.attrib.get('name',''):l for l in root.iter() if tag_name(l)=='link'}
    joints=[]
    for j in root.iter():
        if tag_name(j)!='joint': continue
        parent=next((c for c in list(j) if tag_name(c)=='parent'),None); child=next((c for c in list(j) if tag_name(c)=='child'),None)
        origin=next((c for c in list(j) if tag_name(c)=='origin'),None)
        if parent is None or child is None: continue
        xyz=parse_vec(origin.attrib.get('xyz') if origin is not None else '',3,0.0); rpy=parse_vec(origin.attrib.get('rpy') if origin is not None else '',3,0.0)
        joints.append({'name':j.attrib.get('name',''),'type':j.attrib.get('type','fixed'),'parent':parent.attrib.get('link',''),'child':child.attrib.get('link',''),'origin_xyz':xyz,'origin_rpy':rpy})
    child_to_joint={j['child']:j for j in joints if j['child']}
    roots=[n for n in links.keys() if n and n not in child_to_joint]
    cache={}
    def link_world_tf(link_name):
        if link_name in cache: return cache[link_name]
        if link_name not in links: return None,'missing_link',[]
        chain=[]; seen=set(); tf=identity_tf(); cur=link_name; unresolved=''
        while cur in child_to_joint:
            if cur in seen: unresolved='cycle'; break
            seen.add(cur); j=child_to_joint[cur]
            chain.append(f"{j['parent']}->{cur}({j['type']})")
            tf=matmul4(tf_from_xyz_rpy(j['origin_xyz'],j['origin_rpy']),tf)
            cur=j['parent']
            if not cur or cur not in links: unresolved=f'missing_parent:{cur}'; break
        if not unresolved and cur not in roots and cur in child_to_joint: unresolved='unresolved_chain'
        status='resolved' if not unresolved else 'unresolved'
        result=(tf,status,list(reversed(chain)),cur,unresolved)
        cache[link_name]=result
        return result
    for lname,link in links.items():
        for visual in [c for c in list(link) if tag_name(c)=='visual']:
            geom=next((c for c in list(visual) if tag_name(c)=='geometry'),None)
            if geom is None: continue
            vname=visual.attrib.get('name',f'visual_{idx}'); item_id=f'urdf_visual_{idx}_{sanitize(lname)}_{sanitize(vname)}'
            origin=next((c for c in list(visual) if tag_name(c)=='origin'), None)
            vxyz=parse_vec((origin.attrib.get('xyz') if origin is not None else ''),3,0.0); vrpy=parse_vec((origin.attrib.get('rpy') if origin is not None else ''),3,0.0)
            link_tf, link_status, chain, root_link, unresolved = link_world_tf(lname)
            if link_tf is None: link_tf=identity_tf()
            final_tf=matmul4(link_tf, tf_from_xyz_rpy(vxyz,vrpy)); pose=xyz_rpy_from_tf(final_tf)
            common={'id':item_id,'link':lname,'visual':vname,'parent_link':root_link or '','pose':pose,'link_transform_status':link_status,'transform_status':link_status,'transform_chain':chain,'render_expected':True}
            mesh=next((c for c in list(geom) if tag_name(c)=='mesh'),None)
            box=next((c for c in list(geom) if tag_name(c)=='box'),None)
            cyl=next((c for c in list(geom) if tag_name(c)=='cylinder'),None)
            sph=next((c for c in list(geom) if tag_name(c)=='sphere'),None)
            warning=unresolved
            if mesh is not None:
                mesh_uri=mesh.attrib.get('filename',''); resolved_path, skip_reason=resolve_mesh_uri(mesh_uri, package_map)
                scale=parse_vec(mesh.attrib.get('scale','1 1 1'),3,1.0)
                items.append({**common,'geometry_type':'mesh','package_uri':mesh_uri if mesh_uri.startswith('package://') else '','source_path':mesh_uri,'resolved_source_path':resolved_path,'resolved':bool(resolved_path and not skip_reason),'mesh_scale':scale,'render_expected':not bool(skip_reason),'render_skip_reason':skip_reason,'warning':(skip_reason or warning or '')})
            elif box is not None:
                items.append({**common,'geometry_type':'box','size':parse_vec(box.attrib.get('size','0.1 0.1 0.1'),3,0.1),'resolved':True,'warning':warning or ''})
            elif cyl is not None:
                items.append({**common,'geometry_type':'cylinder','radius':float(cyl.attrib.get('radius','0.05') or 0.05),'length':float(cyl.attrib.get('length','0.1') or 0.1),'resolved':True,'warning':warning or ''})
            elif sph is not None:
                items.append({**common,'geometry_type':'sphere','radius':float(sph.attrib.get('radius','0.05') or 0.05),'resolved':True,'warning':warning or ''})
            idx+=1
    return items


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--scene'); ap.add_argument('--all',action='store_true'); ap.add_argument('--prefer-xacro-expanded',action='store_true',default=True); ap.add_argument('--fallback-best-effort',action='store_true',default=True); ap.add_argument('--fail-on-unexpanded',action='store_true'); ap.add_argument('--xacro-arg',action='append',default=[]); ap.add_argument('--use-fake-hardware',default='true'); ap.add_argument('--robot-prefix',default=''); ap.add_argument('--tool-prefix',default=''); ap.add_argument('--no-write',action='store_true'); ap.add_argument('--prefer-xacro',action='store_true'); ap.add_argument('--require-xacro',action='store_true'); ap.add_argument('--workspace-root',default=os.environ.get('WORKSPACE_ROOT',''))
    a=ap.parse_args()
    scenes=[SCENES_ROOT/a.scene] if a.scene else sorted([p for p in SCENES_ROOT.iterdir() if p.is_dir()])
    report={'scene_count':0,'visual_count':0,'resolved':0,'unresolved':0,'xacro_expanded_count':0,'best_effort_count':0,'scenes':[]}
    for scene_dir in scenes:
        manifest=read_yaml(scene_dir/'scene_manifest.yaml') or {}
        urdf_path=scene_dir/(((manifest.get('files') or {}).get('urdf_xacro')) or 'urdf/scene.urdf.xacro')
        xargs={'use_fake_hardware':a.use_fake_hardware,'robot_prefix':a.robot_prefix,'tool_prefix':a.tool_prefix}
        for ent in a.xacro_arg:
            if '=' in ent:
                k,v=ent.split('=',1); xargs[k.strip()]=v.strip()
        mode='best_effort_recursive'; fallback_reason=''; _,xacro_avail,missing_reason=discover_xacro_command(); expanded_path=''; xacro_cmd=[]; xml_text=''
        if (a.prefer_xacro or a.prefer_xacro_expanded or a.require_xacro):
            xml_text,_,err,xacro_cmd=expand_xacro(urdf_path,scene_dir,xargs,workspace_root=(a.workspace_root or None))
            if xml_text: mode='xacro_expanded'; expanded_path='generated/expanded_scene_preview.urdf'
            else: fallback_reason=err or missing_reason
        if a.require_xacro and not xacro_avail:
            print('xacro executable unavailable (required)'); return 2
        if not xml_text: xml_text=(urdf_path.read_text(errors='ignore') if urdf_path.exists() else '<robot/>')
        package_map, package_diagnostics = discover_package_map(scene_dir, workspace_root=(a.workspace_root or None))
        try: items=extract_from_urdf(xml_text, package_map)
        except Exception: items=[]
        unresolved=[i for i in items if any(contains_placeholder(i.get(k,'')) for k in ('id','link','parent_link'))]
        safe=bool(items) and (len(unresolved)==0) and (mode=='xacro_expanded')
        payload={'scene_name':scene_dir.name,'visual_count':len(items),'resolved':sum(1 for i in items if i.get('resolved')),'unresolved':sum(1 for i in items if not i.get('resolved')),'generated_at':datetime.now(timezone.utc).isoformat(),'extractor_version':EXTRACTOR_VERSION,'extraction_mode':mode,'xacro_available':xacro_avail,'source_expanded_urdf_path':expanded_path,'fallback_reason':fallback_reason,'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'stale_index':False,'stale_reasons':[],'visual_items':items,'xacro_command':xacro_cmd,'package_resolution_diagnostics':package_diagnostics}
        idx_path=scene_dir/'generated/scene_visual_mesh_index.json'
        if not a.no_write:
            idx_path.parent.mkdir(parents=True,exist_ok=True)
            idx_path.write_text(json.dumps(payload,indent=2)+'\n')
        report['scene_count']+=1
        report['visual_count']+=len(items)
        report['resolved']+=sum(1 for i in items if i.get('resolved'))
        report['unresolved']+=sum(1 for i in items if not i.get('resolved'))
        report['xacro_expanded_count']+=int(mode=='xacro_expanded')
        report['best_effort_count']+=int(mode!='xacro_expanded')
        report['scenes'].append({'scene':scene_dir.name,'extraction_mode':mode,'xacro_available':payload['xacro_available'],'expanded_urdf_written':bool(expanded_path),'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'mesh_backed_count':sum(1 for i in items if i.get('geometry_type')=='mesh'),'skipped_count':sum(1 for i in items if i.get('render_skip_reason')),'fallback_reason':fallback_reason,'primitive_fallback_count':sum(1 for i in items if i.get('geometry_type') in ('box','cylinder','sphere')),'stale_index':False,'status':'PASS' if safe else 'WARN'})
        if a.require_xacro and mode != 'xacro_expanded': return 2
    (ROOT/'build').mkdir(exist_ok=True)
    (ROOT/'build/workcell_studio_urdf_visual_mesh_index_report.json').write_text(json.dumps(report,indent=2)+'\n')
    if a.fail_on_unexpanded and report['best_effort_count']>0: return 3
    return 0

if __name__=='__main__': raise SystemExit(main())
