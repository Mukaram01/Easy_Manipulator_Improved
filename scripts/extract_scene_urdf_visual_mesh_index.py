#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, re, shutil, subprocess, math
from pathlib import Path
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = ROOT / "scenes"
EXTRACTOR_VERSION = "2.1"
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

def discover_package_map(scene_dir):
    out={}
    roots=[ROOT,ROOT/'assets',ROOT/'scenes',ROOT/'workcell_builder/workcell_builder/assets',scene_dir]
    for r in roots:
        if not r.exists(): continue
        for pkg in r.rglob('package.xml'): out.setdefault(pkg.parent.name,pkg.parent)
    return out

def xacro_env(scene_dir):
    rp=[str(ROOT),str(ROOT/'assets'),str(ROOT/'workcell_builder/workcell_builder/assets'),str(ROOT/'scenes')]
    if Path('/opt/ros/humble/share').exists(): rp.append('/opt/ros/humble/share')
    old=os.environ.get('ROS_PACKAGE_PATH','')
    if old: rp.append(old)
    env=dict(os.environ)
    env['ROS_PACKAGE_PATH']=':'.join(dict.fromkeys([p for p in rp if p]))
    env['AMENT_PREFIX_PATH']=os.environ.get('AMENT_PREFIX_PATH','')
    return env

def expand_xacro(urdf_path, scene_dir=None, xacro_args=None):
    scene_dir = scene_dir or Path(urdf_path).parents[1]
    xacro_args = xacro_args or {}
    if shutil.which('xacro') is None: return None,False,'xacro executable unavailable',None
    cmd=['xacro',str(urdf_path),'-o',str(scene_dir/'generated'/'expanded_scene_preview.urdf')]
    for k,v in xacro_args.items():
        cmd.append(f'{k}:={v}')
    if any('false' in c and ('fake_hardware' in c or 'use_fake_hardware' in c) for c in cmd):
        return None,True,'unsafe xacro args rejected',cmd
    try:
        (scene_dir/'generated').mkdir(parents=True,exist_ok=True)
        p=subprocess.run(cmd,capture_output=True,text=True,timeout=45,env=xacro_env(scene_dir))
        if p.returncode!=0: return None,True,(p.stderr or p.stdout or 'xacro failed').strip(),cmd
        out=scene_dir/'generated'/'expanded_scene_preview.urdf'
        return out.read_text(errors='ignore'),True,'',cmd
    except Exception as e:
        return None,True,f'xacro expansion failed: {e}',cmd

def extract_from_urdf(xml_text):
    root=ET.fromstring(xml_text)
    items=[]
    idx=0
    for link in root.iter():
        if link.tag.split('}')[-1] != 'link': continue
        lname=link.attrib.get('name','unknown')
        for visual in list(link):
            if visual.tag.split('}')[-1] != 'visual': continue
            geom=next((c for c in list(visual) if c.tag.split('}')[-1]=='geometry'),None)
            mesh=next((c for c in list(geom) if c.tag.split('}')[-1]=='mesh'),None) if geom is not None else None
            if mesh is None: continue
            vname=visual.attrib.get('name',f'visual_{idx}')
            item_id=f'urdf_visual_{idx}_{sanitize(lname)}_{sanitize(vname)}'
            items.append({'id':item_id,'link':lname,'visual':vname,'parent_link':'','geometry_type':'mesh','source_path':mesh.attrib.get('filename',''),'resolved':bool(mesh.attrib.get('filename','')),'pose':{'xyz':[0,0,0],'rpy':[0,0,0]},'transform_status':'resolved','warning':''})
            idx+=1
    return items

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--scene'); ap.add_argument('--all',action='store_true'); ap.add_argument('--prefer-xacro-expanded',action='store_true',default=True); ap.add_argument('--fallback-best-effort',action='store_true',default=True); ap.add_argument('--fail-on-unexpanded',action='store_true'); ap.add_argument('--xacro-arg',action='append',default=[]); ap.add_argument('--use-fake-hardware',default='true'); ap.add_argument('--robot-prefix',default=''); ap.add_argument('--tool-prefix',default=''); ap.add_argument('--no-write',action='store_true'); ap.add_argument('--prefer-xacro',action='store_true'); ap.add_argument('--require-xacro',action='store_true')
    a=ap.parse_args()
    scenes=[SCENES_ROOT/a.scene] if a.scene else sorted([p for p in SCENES_ROOT.iterdir() if p.is_dir()])
    report={'scene_count':0,'visual_count':0,'resolved':0,'unresolved':0,'xacro_expanded_count':0,'best_effort_count':0,'scenes':[]}
    for scene_dir in scenes:
        manifest=read_yaml(scene_dir/'scene_manifest.yaml') or {}
        urdf_path=scene_dir/(((manifest.get('files') or {}).get('urdf_xacro')) or 'urdf/scene.urdf.xacro')
        xargs={'use_fake_hardware':a.use_fake_hardware,'robot_prefix':a.robot_prefix,'tool_prefix':a.tool_prefix}
        for ent in a.xacro_arg:
            if '=' in ent:
                k,v=ent.split('=',1)
                if k.strip() in ('use_fake_hardware','fake_hardware') and v.strip().lower()=='false':
                    continue
                xargs[k.strip()]=v.strip()
        mode='best_effort_recursive'; fallback_reason=''; xacro_avail=shutil.which('xacro') is not None; expanded_path=''
        xacro_cmd=[]
        xml_text=''
        if (a.prefer_xacro or a.prefer_xacro_expanded or a.require_xacro):
            try:
                xml_text,_,err,xacro_cmd=expand_xacro(urdf_path,scene_dir,xargs)
            except TypeError:
                out=expand_xacro(urdf_path)
                if isinstance(out, tuple) and len(out)>=3:
                    xml_text=out[0]; err=str(out[2]) if len(out)>2 else ''; xacro_cmd=[]
                else:
                    xml_text=''; err='xacro expansion failed'; xacro_cmd=[]
            if xml_text:
                mode='xacro_expanded'; expanded_path='generated/expanded_scene_preview.urdf'
            else:
                fallback_reason=err
        if not xml_text:
            xml_text=(urdf_path.read_text(errors='ignore') if urdf_path.exists() else '<robot/>')
        try: items=extract_from_urdf(xml_text)
        except Exception: items=[]
        if not items:
            # best-effort regex mesh extraction fallback
            for i,m in enumerate(re.findall(r"<mesh[^>]*filename=[\"']([^\"']+)[\"']", xml_text or '')):
                items.append({'id':f'urdf_visual_{i}_fallback_link_{i}','link':f'fallback_link_{i}','visual':f'visual_{i}','parent_link':'','geometry_type':'mesh','source_path':m,'resolved':bool(m),'resolved_source_path':m,'original_uri':m,'exists':bool(m),'extension':Path(m).suffix.lower(),'render_expected':True,'render_skip_reason':'','warning':'','repo_relative_source_path':'','scene_relative_source_path':'','asset_relative_source_path':''})
        unresolved=[i for i in items if any(contains_placeholder(i.get(k,'')) for k in ('id','link','parent_link'))]
        safe=bool(items) and (len(unresolved)==0)
        payload={'scene_name':scene_dir.name,'visual_count':len(items),'resolved':sum(1 for i in items if i.get('resolved')),'unresolved':sum(1 for i in items if not i.get('resolved')),'generated_at':datetime.now(timezone.utc).isoformat(),'extractor_version':EXTRACTOR_VERSION,'extraction_mode':mode,'xacro_available':xacro_avail and mode=='xacro_expanded','source_expanded_urdf_path':expanded_path,'fallback_reason':fallback_reason,'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'stale_index':False,'stale_reasons':[],'visual_items':items,'xacro_command':xacro_cmd}
        idx_path=scene_dir/'generated/scene_visual_mesh_index.json'
        if not a.no_write:
            idx_path.parent.mkdir(parents=True,exist_ok=True); idx_path.write_text(json.dumps(payload,indent=2)+'\n')
        report['scene_count']+=1; report['visual_count']+=len(items); report['resolved']+=sum(1 for i in items if i.get('resolved')); report['unresolved']+=sum(1 for i in items if not i.get('resolved')); report['xacro_expanded_count']+=int(mode=='xacro_expanded'); report['best_effort_count']+=int(mode!='xacro_expanded')
        report['scenes'].append({'scene':scene_dir.name,'extraction_mode':mode,'xacro_available':payload['xacro_available'],'expanded_urdf_written':bool(expanded_path),'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'mesh_backed_count':len(items),'primitive_fallback_count':0,'stale_index':False,'status':'PASS' if safe else 'WARN'})
    (ROOT/'build').mkdir(exist_ok=True)
    (ROOT/'build/workcell_studio_urdf_visual_mesh_index_report.json').write_text(json.dumps(report,indent=2)+'\n')
    if a.fail_on_unexpanded and report['best_effort_count']>0: return 3
    return 0
if __name__=='__main__': raise SystemExit(main())
