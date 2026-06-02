#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, re, shutil, subprocess, math, collections, copy, sys
from pathlib import Path
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = ROOT / "scenes"
EXTRACTOR_VERSION = "2.4"
PLACEHOLDER_RE = re.compile(r"(\$\{[^}]+\}|\$\(arg\s+[^)]+\)|\$\(find\s+[^)]+\))")
FIND_RE = re.compile(r"\$\(find\s+([^)\s]+)\)")
ARG_RE = re.compile(r"\$\(arg\s+([^)\s]+)\)")
EXPR_RE = re.compile(r"\$\{([^}]+)\}")

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

def _pythonpath_candidates():
    candidates=[str(ROOT), str(ROOT/'scripts')]
    for base in [Path('/opt/ros/humble/lib/python3.10/site-packages'), Path('/opt/ros/humble/local/lib/python3.10/dist-packages')]:
        if base.exists(): candidates.append(str(base))
    old=os.environ.get('PYTHONPATH','')
    if old: candidates.extend(old.split(os.pathsep))
    return [p for p in dict.fromkeys(candidates) if p]

def xacro_env(scene_dir, workspace_root=None, package_map=None):
    rp=[str(ROOT),str(ROOT/'assets'),str(ROOT/'workcell_builder/workcell_builder/assets'),str(ROOT/'scenes')]
    if workspace_root:
        ws = Path(workspace_root); rp += [str(ws/'install'), str(ws/'install/share'), str(ws/'src'), str(ws/'src/easy_manipulation_deployment/assets'), str(ws/'src/assets')]
    if package_map:
        rp += [str(p.parent) for p in package_map.values()]
    if Path('/opt/ros/humble/share').exists(): rp.append('/opt/ros/humble/share')
    old=os.environ.get('ROS_PACKAGE_PATH','')
    if old: rp.append(old)
    env=dict(os.environ)
    env['ROS_PACKAGE_PATH']=':'.join(dict.fromkeys([p for p in rp if p]))
    env['PYTHONPATH']=os.pathsep.join(_pythonpath_candidates())
    env['AMENT_PREFIX_PATH']=os.environ.get('AMENT_PREFIX_PATH','')
    return env

def discover_xacro_command():
    candidates=[]
    if shutil.which('xacro'): candidates.append([shutil.which('xacro')])
    if Path('/opt/ros/humble/bin/xacro').exists(): candidates.append(['/opt/ros/humble/bin/xacro'])
    candidates.append([sys.executable, '-m', 'xacro'])
    candidates.append(['python3', '-m', 'xacro'])
    reasons=[]
    for cmd in candidates:
        try:
            mod = subprocess.run(cmd + ['--help'], capture_output=True, text=True, timeout=10, env=xacro_env(ROOT))
            if mod.returncode == 0:
                return cmd, True, ''
            reasons.append(' '.join(cmd)+': '+(mod.stderr or mod.stdout or f'rc={mod.returncode}').strip())
        except Exception as e:
            reasons.append(' '.join(cmd)+f': {e}')
    return None, False, 'xacro executable unavailable; ' + '; '.join(reasons[:3])

def _resolve_find_text(text, package_map):
    missing=[]
    def repl(m):
        pkg=m.group(1)
        if pkg in package_map:
            return str(package_map[pkg])
        missing.append(pkg)
        return m.group(0)
    return FIND_RE.sub(repl, text or ''), missing

def _eval_expr(expr, context):
    expr=str(expr).strip()
    safe={'pi':math.pi, 'true':True, 'false':False}
    safe.update({k:v for k,v in context.items() if re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', k)})
    try:
        return str(eval(expr, {'__builtins__':{}}, safe))
    except Exception:
        return str(context.get(expr, '${'+expr+'}'))

def _subst_text(value, context, package_map):
    text, _ = _resolve_find_text(str(value or ''), package_map)
    text = ARG_RE.sub(lambda m: str(context.get(m.group(1), '')), text)
    text = EXPR_RE.sub(lambda m: _eval_expr(m.group(1), context), text)
    return text

def _is_xacro(elem, local=None):
    name=tag_name(elem)
    if local and name != local: return False
    return elem.tag.startswith('{') and 'xacro' in elem.tag.split('}')[0]

def _clone_regular(elem, context, package_map, macros, warnings, blocks=None):
    if _is_xacro(elem, 'insert_block'):
        name=elem.attrib.get('name','')
        block=(blocks or {}).get(name)
        return [copy.deepcopy(block)] if block is not None else []
    if _is_xacro(elem, 'if'):
        val=_subst_text(elem.attrib.get('value',''), context, package_map).strip().lower()
        if val not in {'true','1','yes'}: return []
        out=[]
        for child in list(elem): out.extend(_clone_regular(child, context, package_map, macros, warnings, blocks))
        return out
    if _is_xacro(elem):
        macro=macros.get(tag_name(elem))
        if macro is None:
            warnings.append(f'skipped unsupported xacro tag/macros call: {tag_name(elem)}')
            return []
        return _expand_macro(elem, macro, context, package_map, macros, warnings)
    new=ET.Element(tag_name(elem), {k:_subst_text(v, context, package_map) for k,v in elem.attrib.items()})
    if elem.text and elem.text.strip(): new.text=_subst_text(elem.text, context, package_map)
    for child in list(elem):
        for expanded in _clone_regular(child, context, package_map, macros, warnings, blocks): new.append(expanded)
    return [new]

def _expand_macro(call, macro, parent_context, package_map, macros, warnings):
    params, children = macro
    context=dict(parent_context)
    block_names=[]
    for token in params.split():
        if token.startswith('*'):
            block_names.append(token[1:]); continue
        name, _, default = token.partition(':=')
        if name:
            context[name]=_subst_text(call.attrib.get(name, default), parent_context, package_map)
    for key, value in call.attrib.items(): context[key]=_subst_text(value, parent_context, package_map)
    blocks={}
    for child in list(call):
        lname=tag_name(child)
        if lname in block_names or lname == 'origin':
            blocks[lname]=_clone_regular(child, context, package_map, macros, warnings)[0]
    out=[]
    for child in children:
        out.extend(_clone_regular(child, context, package_map, macros, warnings, blocks))
    return out

def _collect_macros(path, package_map, context, macros, visited, warnings):
    path=Path(path)
    if path in visited: return
    visited.add(path)
    if not path.exists():
        warnings.append(f'include not found: {path}')
        return
    try:
        root=ET.fromstring(path.read_text(errors='ignore'))
    except Exception as e:
        warnings.append(f'include parse failed: {path}: {e}')
        return
    for child in list(root):
        if _is_xacro(child, 'arg'):
            context.setdefault(child.attrib.get('name',''), _subst_text(child.attrib.get('default',''), context, package_map))
        elif _is_xacro(child, 'include'):
            inc=_subst_text(child.attrib.get('filename',''), context, package_map)
            if '$(' in inc:
                warnings.append(f'include has unresolved substitution: {inc}')
            else:
                _collect_macros(Path(inc), package_map, context, macros, visited, warnings)
        elif _is_xacro(child, 'macro'):
            macros[child.attrib.get('name','')]=(child.attrib.get('params',''), [copy.deepcopy(c) for c in list(child)])

def expand_xacro_repo_local(urdf_path, scene_dir, xacro_args, workspace_root=None):
    package_map, diagnostics = discover_package_map(scene_dir, workspace_root=workspace_root)
    warnings=[]; context={'pi':str(math.pi)}
    context.update({k:str(v) for k,v in (xacro_args or {}).items()})
    try:
        root=ET.fromstring(Path(urdf_path).read_text(errors='ignore'))
    except Exception as e:
        return None, f'repo-local xacro parse failed: {e}', diagnostics
    for child in list(root):
        if _is_xacro(child, 'arg'):
            name=child.attrib.get('name','')
            context.setdefault(name, _subst_text(child.attrib.get('default',''), context, package_map))
    macros={}; visited=set()
    for child in list(root):
        if _is_xacro(child, 'include'):
            inc=_subst_text(child.attrib.get('filename',''), context, package_map)
            if '$(' in inc:
                warnings.append(f'include has unresolved substitution: {inc}')
            else:
                _collect_macros(Path(inc), package_map, context, macros, visited, warnings)
    out_root=ET.Element('robot', {'name': root.attrib.get('name', scene_dir.name)})
    for child in list(root):
        if _is_xacro(child, 'arg') or _is_xacro(child, 'include') or _is_xacro(child, 'macro'):
            continue
        for expanded in _clone_regular(child, context, package_map, macros, warnings): out_root.append(expanded)
    xml_text=ET.tostring(out_root, encoding='unicode')
    return xml_text, '; '.join(warnings), diagnostics

def expand_xacro(urdf_path, scene_dir=None, xacro_args=None, workspace_root=None):
    scene_dir = scene_dir or Path(urdf_path).parents[1]; xacro_args = xacro_args or {}
    package_map, _ = discover_package_map(scene_dir, workspace_root=workspace_root)
    xacro_base, xacro_available, reason = discover_xacro_command()
    cmd=None; real_error=''
    if xacro_available:
        try:
            (scene_dir/'generated').mkdir(parents=True,exist_ok=True)
            for stale in [scene_dir/'generated'/'expanded_scene_preview.urdf', scene_dir/'generated'/'scene_preview.find_resolved.xacro']:
                if stale.exists(): stale.unlink()
            source_text=Path(urdf_path).read_text(errors='ignore')
            resolved_text, missing_pkgs = _resolve_find_text(source_text, package_map)
            input_path=scene_dir/'generated'/'scene_preview.find_resolved.xacro'
            input_path.write_text(resolved_text, encoding='utf-8')
            out=scene_dir/'generated'/'expanded_scene_preview.urdf'
            cmd=xacro_base+[str(input_path),'-o',str(out)]
            for k,v in xacro_args.items(): cmd.append(f'{k}:={v}')
            p=subprocess.run(cmd,capture_output=True,text=True,timeout=45,env=xacro_env(scene_dir, workspace_root=workspace_root, package_map=package_map))
            if p.returncode==0:
                return out.read_text(errors='ignore'),True,'',cmd,'xacro_expanded'
            real_error=(p.stderr or p.stdout or 'xacro failed').strip()
            if input_path.exists(): input_path.unlink()
            if missing_pkgs:
                real_error += '; unresolved packages in local package map: ' + ', '.join(sorted(set(missing_pkgs)))
        except Exception as e:
            real_error=f'xacro expansion failed: {e}'
    xml_text, local_warning, _ = expand_xacro_repo_local(urdf_path, scene_dir, xacro_args, workspace_root=workspace_root)
    if xml_text:
        reason_text='repo-local xacro fallback used'
        if real_error: reason_text += f' after xacro failure: {real_error}'
        elif reason: reason_text += f': {reason}'
        if local_warning: reason_text += f'; fallback warnings: {local_warning}'
        return xml_text, bool(xacro_available), reason_text, (cmd or []), 'repo_local_xacro_expanded'
    return None,bool(xacro_available),(real_error or reason or local_warning or 'xacro failed'),(cmd or []),'best_effort_recursive'

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
    global_materials={}
    for mat in [m for m in list(root) if tag_name(m)=='material']:
        name=mat.attrib.get('name','')
        color=next((c for c in list(mat) if tag_name(c)=='color'),None)
        if name and color is not None:
            global_materials[name]=parse_vec(color.attrib.get('rgba',''),4,1.0)
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
            pose=xyz_rpy_from_tf(link_tf)
            material_node=next((c for c in list(visual) if tag_name(c)=='material'),None)
            material={'name':'','color':None}
            if material_node is not None:
                material['name']=material_node.attrib.get('name','')
                color_node=next((c for c in list(material_node) if tag_name(c)=='color'),None)
                if color_node is not None:
                    material['color']=parse_vec(color_node.attrib.get('rgba',''),4,1.0)
                elif material['name'] in global_materials:
                    material['color']=global_materials[material['name']]
            common={'id':item_id,'link':lname,'visual':vname,'parent_link':root_link or '','pose':pose,'visual_origin':{'xyz':vxyz,'rpy':vrpy},'material':material,'link_transform_status':link_status,'transform_status':link_status,'transform_chain':chain,'render_expected':True}
            mesh=next((c for c in list(geom) if tag_name(c)=='mesh'),None)
            box=next((c for c in list(geom) if tag_name(c)=='box'),None)
            cyl=next((c for c in list(geom) if tag_name(c)=='cylinder'),None)
            sph=next((c for c in list(geom) if tag_name(c)=='sphere'),None)
            cap=next((c for c in list(geom) if tag_name(c)=='capsule'),None)
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
            elif cap is not None:
                items.append({**common,'geometry_type':'capsule','radius':float(cap.attrib.get('radius','0.05') or 0.05),'length':float(cap.attrib.get('length',cap.attrib.get('height','0.1')) or 0.1),'resolved':True,'warning':warning or ''})
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
            try:
                expanded_result = expand_xacro(urdf_path,scene_dir,xargs,workspace_root=(a.workspace_root or None))
            except TypeError:
                expanded_result = expand_xacro(urdf_path)
            if isinstance(expanded_result, tuple) and len(expanded_result) >= 5:
                xml_text,_,err,xacro_cmd,mode_hint=expanded_result[:5]
                if mode_hint: mode=str(mode_hint)
            elif isinstance(expanded_result, tuple) and len(expanded_result) >= 4:
                xml_text,_,err,xacro_cmd=expanded_result[:4]
            elif isinstance(expanded_result, tuple) and len(expanded_result) == 3:
                xml_text,mode_hint,err=expanded_result; xacro_cmd=[]
                if isinstance(err, list): err='; '.join(str(e) for e in err)
                if mode_hint and not xml_text: mode=str(mode_hint)
            else:
                xml_text,err,xacro_cmd='', 'xacro expansion failed: unexpected extractor result', []
            if xml_text:
                if mode == 'best_effort_recursive': mode='xacro_expanded'
                expanded_path='generated/expanded_scene_preview.urdf' if (scene_dir/'generated'/'expanded_scene_preview.urdf').exists() else ''
                if mode != 'xacro_expanded': fallback_reason=err or missing_reason
            else: fallback_reason=err or missing_reason
        if a.require_xacro and not xacro_avail and mode != 'repo_local_xacro_expanded':
            print('xacro executable unavailable and repo-local xacro expansion failed (required)'); return 2
        if not xml_text: xml_text=(urdf_path.read_text(errors='ignore') if urdf_path.exists() else '<robot/>')
        package_map, package_diagnostics = discover_package_map(scene_dir, workspace_root=(a.workspace_root or None))
        try: items=extract_from_urdf(xml_text, package_map)
        except Exception: items=[]
        unresolved=[i for i in items if any(contains_placeholder(i.get(k,'')) for k in ('id','link','parent_link'))]
        safe=bool(items) and (len(unresolved)==0) and (mode in {'xacro_expanded','repo_local_xacro_expanded'})
        mesh_format_counts=dict(collections.Counter((Path(i.get('resolved_source_path') or i.get('source_path') or '').suffix.lower() or 'unknown') for i in items if i.get('geometry_type')=='mesh'))
        transform_status_counts=dict(collections.Counter(str(i.get('transform_status') or 'unknown') for i in items))
        renderable_count=sum(1 for i in items if i.get('render_expected', True))
        renderable_mesh_count=sum(1 for i in items if i.get('geometry_type')=='mesh' and i.get('render_expected', True))
        source_mtime=urdf_path.stat().st_mtime if urdf_path.exists() else None
        has_transform_collapse_warning=bool(items) and len({tuple((i.get('pose') or {}).get('xyz') or []) for i in items}) <= 1 and len(items) > 1
        payload={'scene_name':scene_dir.name,'visual_count':len(items),'resolved':sum(1 for i in items if i.get('resolved')),'unresolved':sum(1 for i in items if not i.get('resolved')),'generated_at':datetime.now(timezone.utc).isoformat(),'extractor_version':EXTRACTOR_VERSION,'extraction_mode':mode,'xacro_available':xacro_avail,'source_urdf_xacro_path':str(urdf_path),'source_mtime':source_mtime,'source_expanded_urdf_path':expanded_path,'fallback_reason':fallback_reason,'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'has_transform_collapse_warning':has_transform_collapse_warning,'candidate_mesh_count':len(items),'emitted_visual_count':len(items),'transform_status_counts':transform_status_counts,'mesh_format_counts':mesh_format_counts,'renderable_mesh_count':renderable_mesh_count,'renderable_item_count':renderable_count,'stale_index':False,'stale_reasons':[],'visual_items':items,'xacro_command':xacro_cmd,'package_resolution_diagnostics':package_diagnostics}
        idx_path=scene_dir/'generated/scene_visual_mesh_index.json'
        if not a.no_write:
            idx_path.parent.mkdir(parents=True,exist_ok=True)
            idx_path.write_text(json.dumps(payload,indent=2)+'\n')
        report['scene_count']+=1
        report['visual_count']+=len(items)
        report['resolved']+=sum(1 for i in items if i.get('resolved'))
        report['unresolved']+=sum(1 for i in items if not i.get('resolved'))
        report['xacro_expanded_count']+=int(mode in {'xacro_expanded','repo_local_xacro_expanded'})
        report['best_effort_count']+=int(mode not in {'xacro_expanded','repo_local_xacro_expanded'})
        report.setdefault('mesh_format_counts', {})
        for ext,count in mesh_format_counts.items(): report['mesh_format_counts'][ext]=report['mesh_format_counts'].get(ext,0)+count
        report['renderable_mesh_count']=report.get('renderable_mesh_count',0)+renderable_mesh_count
        report['renderable_item_count']=report.get('renderable_item_count',0)+renderable_count
        report['candidate_mesh_count']=report.get('candidate_mesh_count',0)+len(items)
        report['emitted_visual_count']=report.get('emitted_visual_count',0)+len(items)
        report['unresolved_placeholder_count']=report.get('unresolved_placeholder_count',0)+len(unresolved)
        report['scenes'].append({'scene':scene_dir.name,'extraction_mode':mode,'xacro_available':payload['xacro_available'],'expanded_urdf_written':bool(expanded_path),'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'mesh_backed_count':sum(1 for i in items if i.get('geometry_type')=='mesh'),'skipped_count':sum(1 for i in items if i.get('render_skip_reason')),'fallback_reason':fallback_reason,'urdf_primitive_count':sum(1 for i in items if i.get('geometry_type') in ('box','cylinder','sphere','capsule')),'primitive_fallback_count':0,'stale_index':False,'status':'PASS' if safe else 'WARN'})
        if a.require_xacro and mode not in {'xacro_expanded','repo_local_xacro_expanded'}: return 2
    (ROOT/'build').mkdir(exist_ok=True)
    (ROOT/'build/workcell_studio_urdf_visual_mesh_index_report.json').write_text(json.dumps(report,indent=2)+'\n')
    if a.fail_on_unexpanded and report['best_effort_count']>0: return 3
    return 0

if __name__=='__main__': raise SystemExit(main())
