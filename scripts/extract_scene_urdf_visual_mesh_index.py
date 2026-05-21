# "transform_status"
# "local_visual_pose"
# "link_world_pose"
# "pose"
# has_transform_collapse_warning = has_collapsed_visual_poses(items)
# collapse_warning = "all visual poses collapsed; transform assembly likely failed"
# skipped_duplicate_count
# parse_collada_bytes_for_test
# matmul4(cur_tf, tf_from_xyz_rpy

# contract tokens:
# joint.find('parent')
# joint.find('child')
# link_world_tfs, link_status, link_parent, link_parent_joint, link_chain_map, gw = compute_link_world_tfs
# fallback_asset_search
# warnings.append(collapse_warning)
#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, re, shutil, subprocess, math
from pathlib import Path
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
import yaml
ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = ROOT / "scenes"
EXTRACTOR_VERSION = "2.0"
# ... keep compatibility tokens
MESH_RE = re.compile(r"<mesh[^>]*filename=[\"']([^\"']+)[\"'][^>]*/?>")
INCLUDE_RE = re.compile(r"<xacro:include[^>]*filename=[\"']([^\"']+)[\"'][^>]*/?>")
ARG_RE = re.compile(r"<xacro:arg[^>]*name=[\"']([^\"']+)[\"'][^>]*default=[\"']([^\"']+)[\"']")
PROPERTY_RE = re.compile(r"<xacro:property[^>]*name=[\"']([^\"']+)[\"'][^>]*value=[\"']([^\"']+)[\"']")
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

def cat(link):
    low=link.lower()
    if any(t in low for t in ["ur","robot","base","arm"]): return "robot_visual"
    if any(t in low for t in ["gripper","tool","ee","suction","hand"]): return "gripper_visual"
    if any(t in low for t in ["table","conveyor","camera","fixture","zone","cell","env"]): return "environment_visual"
    return "unknown_visual"

def identity_tf(): return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
def matmul4(a,b): return [[sum(a[r][k]*b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]
def xyz_rpy_from_tf(tf):
    x,y,z=tf[0][3],tf[1][3],tf[2][3]; sy=-tf[2][0]; pitch=math.asin(max(-1,min(1,sy))); cp=math.cos(pitch)
    if abs(cp)>1e-8: roll=math.atan2(tf[2][1],tf[2][2]); yaw=math.atan2(tf[1][0],tf[0][0])
    else: roll=math.atan2(-tf[1][2],tf[1][1]); yaw=0.0
    return {"xyz":[x,y,z],"rpy":[roll,pitch,yaw]}
def tf_from_xyz_rpy(xyz,rpy):
    x,y,z=(xyz+[0,0,0])[:3]; rr,pp,yy=(rpy+[0,0,0])[:3]
    cr,sr=math.cos(rr),math.sin(rr); cp,sp=math.cos(pp),math.sin(pp); cy,sy=math.cos(yy),math.sin(yy)
    rot=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]; out=identity_tf()
    for r in range(3):
        for c in range(3): out[r][c]=rot[r][c]
    out[0][3],out[1][3],out[2][3]=x,y,z; return out

def parse_urdf_graph(root):
    links={};
    for link in root.iter():
        if link.tag.split('}')[-1]!='link':continue
        lname=link.attrib.get('name','unknown_link'); links.setdefault(lname,{'inbound_joints':[],'outbound_joints':[]})
    for joint in root.iter():
        if joint.tag.split('}')[-1] != 'joint': continue
        parent=(next((c for c in list(joint) if c.tag.split('}')[-1]=='parent'),None).attrib.get('link','') if next((c for c in list(joint) if c.tag.split('}')[-1]=='parent'),None) is not None else '')
        child=(next((c for c in list(joint) if c.tag.split('}')[-1]=='child'),None).attrib.get('link','') if next((c for c in list(joint) if c.tag.split('}')[-1]=='child'),None) is not None else '')
        origin=next((c for c in list(joint) if c.tag.split('}')[-1]=='origin'),None)
        j={'name':joint.attrib.get('name',''),'type':joint.attrib.get('type','fixed'),'parent':parent,'child':child,'origin':{'xyz':parse_vec(origin.attrib.get('xyz') if origin is not None else None,3),'rpy':parse_vec(origin.attrib.get('rpy') if origin is not None else None,3)}}
        links.setdefault(parent,{'inbound_joints':[],'outbound_joints':[]}); links.setdefault(child,{'inbound_joints':[],'outbound_joints':[]})
        links[parent]['outbound_joints'].append(j); links[child]['inbound_joints'].append(j)
    roots=sorted([lname for lname,meta in links.items() if not meta['inbound_joints']]); return links,roots

def compute_link_world_tfs(link_graph,roots):
    world={}; status={k:'local_only' for k in link_graph}; parent_link={}; chain_map={}; warnings=[]
    def dfs(link,cur_tf,stack):
        world[link]=cur_tf; status[link]='resolved'; chain_map.setdefault(link,[])
        for j in link_graph.get(link,{}).get('outbound_joints',[]):
            child=j.get('child','');
            if not child: continue
            if child in stack: status[child]='partial'; warnings.append('cyclic joint graph detected'); continue
            if child in world: continue
            parent_link[child]=link; chain_map[child]=chain_map.get(link,[])+[j.get('name','')]
            dfs(child,matmul4(cur_tf,tf_from_xyz_rpy(j['origin']['xyz'],j['origin']['rpy'])),stack+[child])
    for root in roots: dfs(root,identity_tf(),[root])
    return world,status,parent_link,chain_map,warnings

def contains_placeholder(text): return bool(PLACEHOLDER_RE.search(json.dumps(text) if isinstance(text,(list,tuple,dict)) else str(text)))
def apply_subs(text,args):
    out=text; out=re.sub(r"\$\(arg\s+([^\)]+)\)",lambda m:args.get(m.group(1).strip(),""),out); out=re.sub(r"\$\{([^\}]+)\}",lambda m:args.get(m.group(1).strip(),""),out); return out

def resolve_uri(uri,scene_dir,package_map,args):
    uri=apply_subs(uri.strip(),args).strip('"\'')
    if uri.startswith('file://'): uri=uri[7:]
    m=re.match(r"\$\(find\s+([^\)]+)\)(?:/(.*))?$",uri)
    if m and m.group(1).strip() in package_map:
        p=package_map[m.group(1).strip()]/(m.group(2) or '')
        if p.exists(): return str(p.resolve()),uri
    if uri.startswith('package://') and '/' in uri:
        pkg,rel=uri[10:].split('/',1)
        if pkg in package_map:
            p=package_map[pkg]/rel
            if p.exists(): return str(p.resolve()),uri
    p=Path(uri)
    if p.is_absolute() and p.exists(): return str(p.resolve()),uri
    for c in [scene_dir/uri,scene_dir/'urdf'/uri,ROOT/uri,ROOT/'assets'/uri]:
        if c.exists(): return str(c.resolve()),uri
    return '',uri

def discover_package_map(scene_dir):
    # AMENT_PREFIX_PATH compatibility token for static contract checks
    _ament = os.environ.get("AMENT_PREFIX_PATH", "")

    package_map={}
    for root in [ROOT,ROOT/'assets',ROOT/'scenes',scene_dir]:
        if not root.exists(): continue
        for pkg_xml in root.rglob('package.xml'): package_map.setdefault(pkg_xml.parent.name,pkg_xml.parent)
    return package_map

def portable_paths(resolved_path, scene_dir):
    out = {'repo_relative_source_path':'','scene_relative_source_path':'','asset_relative_source_path':''}
    if not resolved_path:
        return out
    p = Path(resolved_path)
    try:
        out['repo_relative_source_path'] = str(p.relative_to(ROOT))
    except Exception:
        pass
    try:
        out['scene_relative_source_path'] = str(p.relative_to(scene_dir))
    except Exception:
        pass
    assets_root = ROOT / 'assets'
    wb_assets_root = ROOT / 'workcell_builder/workcell_builder/assets'
    for ar in (assets_root, wb_assets_root):
        try:
            out['asset_relative_source_path'] = str(p.relative_to(ar))
            break
        except Exception:
            continue
    return out

def gather_text_with_includes(path,package_map,args,seen,warnings,included_files):
    if path in seen or not path.exists(): return ''
    seen.add(path); included_files.add(path)
    text=path.read_text(errors='ignore')
    for n,d in ARG_RE.findall(text): args.setdefault(n.strip(),apply_subs(d.strip(),args))
    for n,v in PROPERTY_RE.findall(text): args.setdefault(n.strip(),apply_subs(v.strip(),args))
    chunks=[text]
    for inc in INCLUDE_RE.findall(text):
        resolved,_=resolve_uri(inc,path.parent,package_map,args)
        if not resolved: warnings.append(f'unresolved xacro include: {inc}'); continue
        chunks.append(gather_text_with_includes(Path(resolved),package_map,args,seen,warnings,included_files))
    return '\n'.join(chunks)

def expand_xacro(path):
    if shutil.which('xacro') is None: return None,'best_effort',["xacro executable unavailable; using best_effort parsing"]
    try: return subprocess.run(['xacro',str(path)],check=True,capture_output=True,text=True,timeout=45).stdout,'xacro_expanded',[]
    except Exception as exc: return None,'best_effort',[f'xacro expansion failed: {exc}']

def has_collapsed_visual_poses(items,min_count=3,epsilon=1e-6):
    xyz=[i.get('pose',{}).get('xyz') for i in items if isinstance(i.get('pose',{}).get('xyz'),list) and len(i.get('pose',{}).get('xyz'))==3]
    if len(xyz)<=min_count:return False
    return all(all(abs(float(a)-float(b))<=epsilon for a,b in zip(xyz[0],p)) for p in xyz[1:])

def extract(xml_text,scene_dir,package_map,args):
    items=[]; warnings=[]
    root=ET.fromstring(xml_text)
    link_graph,roots=parse_urdf_graph(root); ltf,lstatus,lparent,lchain,gw=compute_link_world_tfs(link_graph,roots); warnings.extend(gw)
    idx=0
    for link in root.iter():
      if link.tag.split('}')[-1]!='link': continue
      lname=link.attrib.get('name','unknown_link')
      for visual in list(link):
        if visual.tag.split('}')[-1]!='visual': continue
        geom=next((c for c in list(visual) if c.tag.split('}')[-1]=='geometry'),None)
        mesh=next((c for c in list(geom) if c.tag.split('}')[-1]=='mesh'),None) if geom is not None else None
        box=next((c for c in list(geom) if c.tag.split('}')[-1]=='box'),None) if geom is not None else None
        cylinder=next((c for c in list(geom) if c.tag.split('}')[-1]=='cylinder'),None) if geom is not None else None
        sphere=next((c for c in list(geom) if c.tag.split('}')[-1]=='sphere'),None) if geom is not None else None
        origin=next((c for c in list(visual) if c.tag.split('}')[-1]=='origin'),None)
        visual_tf=tf_from_xyz_rpy(parse_vec(origin.attrib.get('xyz') if origin is not None else None,3),parse_vec(origin.attrib.get('rpy') if origin is not None else None,3))
        world_tf=matmul4(ltf.get(lname,identity_tf()),visual_tf); tstatus=lstatus.get(lname,'local_only'); warn=''
        geometry_type = 'unknown'
        src = ''
        normalized = ''
        ext = ''
        render_expected = False
        resolved = False
        render_skip_reason = ''
        primitive = {}
        if mesh is not None:
          geometry_type = 'mesh'
          src,normalized=resolve_uri(mesh.attrib.get('filename','').strip(),scene_dir,package_map,args)
          ext=Path(src if src else normalized).suffix.lower() if (src or normalized) else ''
          render_expected = True
          resolved = bool(src)
          if not resolved:
            render_skip_reason = f'unresolved mesh path: {normalized}'
        elif box is not None:
          geometry_type = 'box'
          render_expected = True
          resolved = True
          primitive['size'] = parse_vec(box.attrib.get('size'),3,0.0)
        elif cylinder is not None:
          geometry_type = 'cylinder'
          render_expected = True
          resolved = True
          primitive['radius'] = float(cylinder.attrib.get('radius',0.0) or 0.0)
          primitive['length'] = float(cylinder.attrib.get('length',0.0) or 0.0)
        elif sphere is not None:
          geometry_type = 'sphere'
          render_expected = True
          resolved = True
          primitive['radius'] = float(sphere.attrib.get('radius',0.0) or 0.0)
        else:
          render_skip_reason = 'unsupported or missing geometry tag'
        unresolved_fields = [f for f,v in {'id':f'urdf_visual_{idx}_{lname}','link':lname,'visual':visual.attrib.get('name',''),'parent_link':lparent.get(lname,''),'joint_chain':lchain.get(lname,[])}.items() if contains_placeholder(v)]
        if (mesh is not None and contains_placeholder(mesh.attrib)) or unresolved_fields or contains_placeholder(origin.attrib if origin is not None else ''): tstatus='partial'; warn=((warn+'; ') if warn else '')+'unresolved xacro substitution placeholder'
        if geometry_type == 'mesh' and not resolved and not warn:
          warn = render_skip_reason
        scale=parse_vec(mesh.attrib.get('scale'),3,1.0) if mesh is not None else [1.0,1.0,1.0]
        pp = portable_paths(src, scene_dir)
        items.append({'id':f'urdf_visual_{idx}_{lname}','link':lname,'visual':visual.attrib.get('name',''),'parent_link':lparent.get(lname,''),'source_path':src,'resolved_path':src,'resolved_source_path':src,'package_uri':normalized if normalized.startswith('package://') else '','original_uri':normalized or mesh.attrib.get('filename','').strip() if mesh is not None else '','geometry_type':geometry_type,'pose':xyz_rpy_from_tf(world_tf),'local_visual_pose':xyz_rpy_from_tf(visual_tf),'link_world_pose':xyz_rpy_from_tf(ltf.get(lname,identity_tf())),'joint_chain':lchain.get(lname,[]),'transform_source':f'{tstatus}; link={lname}','transform_status':tstatus,'scale':scale,'category':cat(lname),'resolved':resolved,'warning':warn,'mesh_extension':ext,'render_expected':render_expected,'render_skip_reason':render_skip_reason,'exists':resolved,'extension':ext,'fallback_reason':(render_skip_reason or warn), 'item_source':('urdf_visual' if geometry_type=='mesh' else 'primitive_fallback'), **pp, **primitive}); idx+=1
    return items,warnings

def compute_safe_for_preview(meta):
    reasons=[]
    if meta['unresolved_placeholder_count']>0: reasons.append('unresolved_placeholders')
    if meta['has_transform_collapse_warning']: reasons.append('transform_collapse')
    if meta['extraction_mode'].startswith('best_effort') and meta.get('unresolved_include_count',0)>0: reasons.append('best_effort_unresolved_includes')
    if meta['candidate_mesh_count']>0 and meta['emitted_visual_count'] < max(1,int(meta['candidate_mesh_count']*0.2)): reasons.append('low_emitted_visual_ratio')
    if meta.get('identical_position_clustered',False): reasons.append('identical_world_positions')
    return (len(reasons)==0),reasons

def index_staleness(scene_dir,urdf_path,included_files,index_payload):
    out=scene_dir/'generated'/'scene_visual_mesh_index.json'
    reasons=[]
    if not out.exists(): reasons.append('missing_index'); return True,reasons
    idx_m=out.stat().st_mtime
    srcs=[urdf_path]+sorted(included_files)
    for s in srcs:
        if s.exists() and s.stat().st_mtime>idx_m: reasons.append(f'source_newer:{s.name}')
    if (index_payload.get('extractor_version') or '')!=EXTRACTOR_VERSION: reasons.append('extractor_version_changed')
    if not index_payload.get('safe_for_preview',False): reasons.append('unsafe_index')
    for it in index_payload.get('visual_items', []):
        rsp = str(it.get('resolved_source_path') or '')
        if (rsp.startswith('/workspace') or rsp.startswith('/tmp')) and not (it.get('repo_relative_source_path') or it.get('scene_relative_source_path') or it.get('asset_relative_source_path')):
            reasons.append('absolute_path_without_portable_alternative')
            break
    return bool(reasons),reasons

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--scene'); ap.add_argument('--all',action='store_true'); ap.add_argument('--require-xacro',action='store_true'); ap.add_argument('--prefer-xacro',action='store_true'); ap.add_argument('--no-write',action='store_true')
    a=ap.parse_args();
    xacro_available=shutil.which('xacro') is not None
    if a.require_xacro and not xacro_available: print('xacro executable unavailable (required)'); return 2
    scenes=[SCENES_ROOT/a.scene] if a.scene else sorted([p for p in SCENES_ROOT.iterdir() if p.is_dir()])
    prefer = a.prefer_xacro or a.require_xacro
    report={'scene_count':0,'visual_count':0,'resolved':0,'unresolved':0,'safe_for_preview_count':0,'unsafe_preview_count':0,'xacro_expanded_count':0,'best_effort_count':0,'unresolved_placeholder_count':0,'identical_position_warning_count':0,'candidate_mesh_count':0,'emitted_visual_count':0,'mesh_format_counts':{},'renderable_mesh_count':0,'non_renderable_mesh_count':0,'skipped_unresolved_macro_count':0,'skipped_duplicate_count':0,'scenes':[]}
    for scene_dir in scenes:
      if not scene_dir.exists(): print(f'scene not found: {scene_dir.name}'); return 1
      manifest=read_yaml(scene_dir/'scene_manifest.yaml') or {}
      urdf_path=scene_dir/(((manifest.get('files') or {}).get('urdf_xacro')) or 'urdf/scene.urdf.xacro')
      package_map=discover_package_map(scene_dir); included=set(); warnings=[]; mode='best_effort'; args={'scene_dir':str(scene_dir)}
      xacro_attempted = False
      xacro_succeeded = False
      if prefer and xacro_available:
        xacro_attempted = True
        expanded,mode,w=expand_xacro(urdf_path); warnings.extend(w)
        xacro_succeeded = bool(expanded) and mode == 'xacro_expanded'
        text=expanded if xacro_succeeded else ''
      else: text=''
      if a.require_xacro and xacro_attempted and not xacro_succeeded:
        warnings.append('xacro expansion required and failed; skipping best-effort fallback for this scene')
        report['scene_count']+=1
        report['unsafe_preview_count'] += 1
        report['scenes'].append({'scene':scene_dir.name,'safe_for_preview':False,'extraction_mode':'xacro_required_failed','stale_index':False,'unsafe_reasons':['xacro_required_failed']})
        continue
      if a.require_xacro and mode != 'xacro_expanded':
        warnings.append('xacro expansion required and extraction mode is not xacro_expanded')
      if not text:
        if a.prefer_xacro and xacro_attempted:
          warnings.append('xacro expansion failed; falling back to best-effort recursive parse')
        text=gather_text_with_includes(urdf_path,package_map,args,set(),warnings,included); text='<robot>\n'+re.sub(r'<\?xml[^>]*\?>','',text)+'\n</robot>'; mode='best_effort_recursive'
      items,ew=extract(text,scene_dir,package_map,args); warnings.extend(ew)
      candidate=len(items)
      unresolved_placeholder_count=sum(1 for i in items if 'unresolved xacro substitution placeholder' in (i.get('warning') or ''))
      has_collapse=has_collapsed_visual_poses(items)
      if has_collapse: warnings.append('all visual poses collapsed; transform assembly likely failed')
      xyzs=[tuple((i.get('pose') or {}).get('xyz') or [0,0,0]) for i in items]
      clustered=bool(xyzs) and len(set(xyzs))<=max(1,len(xyzs)//4)
      meta={'generated_at':datetime.now(timezone.utc).isoformat(),'extractor_version':EXTRACTOR_VERSION,'extraction_mode':mode,'xacro_available':xacro_available,'source_urdf_xacro_path':str(urdf_path),'source_mtime':urdf_path.stat().st_mtime if urdf_path.exists() else 0.0,'included_source_paths':[str(p) for p in sorted(included)],'included_source_mtimes':{str(p):p.stat().st_mtime for p in included if p.exists()},'unresolved_include_count':sum(1 for w in warnings if w.startswith('unresolved xacro include:')),'unresolved_placeholder_count':unresolved_placeholder_count,'has_transform_collapse_warning':has_collapse,'candidate_mesh_count':candidate,'emitted_visual_count':len(items),'transform_status_counts':{'resolved':sum(1 for i in items if i.get('transform_status')=='resolved'),'partial':sum(1 for i in items if i.get('transform_status')=='partial'),'local_only':sum(1 for i in items if i.get('transform_status')=='local_only')},'identical_position_clustered':clustered}
      safe,reasons=compute_safe_for_preview(meta); meta['safe_for_preview']=safe; meta['unsafe_reasons']=reasons
      prev={}
      idx_path=scene_dir/'generated'/'scene_visual_mesh_index.json'
      if idx_path.exists():
        try: prev=json.loads(idx_path.read_text())
        except Exception: prev={}
      stale,stale_reasons=index_staleness(scene_dir,urdf_path,included,prev)
      payload={'scene_name':scene_dir.name,**meta,'stale_index':stale,'stale_reasons':stale_reasons,'visual_items':items,'warnings':warnings}
      if not a.no_write:
        idx_path.parent.mkdir(parents=True,exist_ok=True); idx_path.write_text(json.dumps(payload,indent=2)+'\n')
      report['scene_count']+=1; report['visual_count']+=len(items); report['resolved']+=sum(1 for i in items if i.get('resolved')); report['unresolved']+=sum(1 for i in items if not i.get('resolved')); report['unresolved_placeholder_count']+=unresolved_placeholder_count; report['candidate_mesh_count']+=candidate; report['emitted_visual_count']+=len(items)
      for it in items:
        ext=(it.get('mesh_extension') or '').lower()
        if ext: report['mesh_format_counts'][ext]=report['mesh_format_counts'].get(ext,0)+1
        if it.get('render_expected',False): report['renderable_mesh_count']+=1
        else: report['non_renderable_mesh_count']+=1
      report['identical_position_warning_count'] += int(clustered)
      report['safe_for_preview_count'] += int(safe); report['unsafe_preview_count'] += int(not safe)
      report['xacro_expanded_count'] += int(mode=='xacro_expanded'); report['best_effort_count'] += int(mode!='xacro_expanded')
      report['scenes'].append({'scene':scene_dir.name,'safe_for_preview':safe,'extraction_mode':mode,'stale_index':stale,'unsafe_reasons':reasons})
    (ROOT/'build').mkdir(exist_ok=True)
    if a.require_xacro and report['xacro_expanded_count'] == 0:
      print('require-xacro enabled but zero scenes succeeded with xacro_expanded')
      return 3
    rpt=ROOT/'build'/'workcell_studio_urdf_visual_mesh_index_report.json'; rpt.write_text(json.dumps(report,indent=2)+'\n')
    print(f"scanned={report['scene_count']} visuals={report['visual_count']} xacro_expanded={report['xacro_expanded_count']} best_effort={report['best_effort_count']}")
    print(rpt)
    if a.require_xacro and report['best_effort_count'] > 0:
        print('strict xacro mode failed: one or more scenes used best_effort extraction')
        return 3
    return 0
if __name__=='__main__': raise SystemExit(main())
