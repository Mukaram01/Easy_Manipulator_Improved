#!/usr/bin/env python3
from __future__ import annotations
import argparse, copy, json, os, re, shutil, subprocess, math, collections
from pathlib import Path
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = ROOT / "scenes"
EXTRACTOR_VERSION = "2.5"
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

def _package_search_paths(scene_dir=None, workspace_root=None):
    roots=[ROOT, ROOT/'assets', ROOT/'workcell_builder/workcell_builder/assets', ROOT/'scenes']
    if scene_dir:
        roots.append(Path(scene_dir))
    if workspace_root:
        ws=Path(workspace_root)
        roots += [ws/'install/share', ws/'src', ws/'src/easy_manipulation_deployment/assets', ws/'src/assets']
    if Path('/opt/ros/humble/share').exists():
        roots.append(Path('/opt/ros/humble/share'))
    package_dirs=[]
    seen=set()
    for root in roots:
        if not root or not Path(root).exists():
            continue
        for pkg_xml in Path(root).rglob('package.xml'):
            pkg_dir=pkg_xml.parent.resolve()
            if pkg_dir not in seen:
                seen.add(pkg_dir); package_dirs.append(pkg_dir)
    return roots, package_dirs

def _dedupe_path_entries(entries):
    out=[]
    seen=set()
    for entry in entries:
        entry=str(entry).strip() if entry else ''
        if not entry or entry in seen:
            continue
        seen.add(entry)
        out.append(entry)
    return out

def xacro_env(scene_dir, workspace_root=None):
    roots, package_dirs = _package_search_paths(scene_dir, workspace_root)
    rp=[str(p) for p in [*roots, *package_dirs, *(p.parent for p in package_dirs)] if p]
    old=os.environ.get('ROS_PACKAGE_PATH','')
    if old: rp.extend(old.split(os.pathsep))
    env=dict(os.environ)
    env['ROS_PACKAGE_PATH']=os.pathsep.join(_dedupe_path_entries(rp))

    ament_prefix_entries=[]
    inherited_ament=os.environ.get('AMENT_PREFIX_PATH','')
    if inherited_ament:
        ament_prefix_entries.extend(inherited_ament.split(os.pathsep))
    if workspace_root:
        workspace_install=Path(workspace_root)/'install'
        if workspace_install.exists():
            ament_prefix_entries.append(str(workspace_install))
    ros_humble=Path('/opt/ros/humble')
    if ros_humble.exists():
        ament_prefix_entries.append(str(ros_humble))
    env['AMENT_PREFIX_PATH']=os.pathsep.join(_dedupe_path_entries(ament_prefix_entries))
    return env

def discover_xacro_command():
    if shutil.which('xacro'): return ['xacro'], True, ''
    if Path('/opt/ros/humble/bin/xacro').exists(): return ['/opt/ros/humble/bin/xacro'], True, ''
    try:
        mod = subprocess.run(['python3', '-m', 'xacro', '--help'], capture_output=True, text=True)
        if mod.returncode == 0: return ['python3', '-m', 'xacro'], True, ''
    except Exception as e:
        return None, False, f"xacro executable unavailable ({e})"
    return None, False, "xacro executable unavailable"

def _xacro_tag(e):
    name=tag_name(e)
    if e.tag.startswith('{'):
        ns=e.tag.split('}',1)[0].strip('{')
        if 'xacro' in ns:
            return name
    if ':' in e.tag:
        prefix, local=e.tag.split(':',1)
        if prefix == 'xacro': return local
    return ''

def _truthy(value):
    return str(value).strip().lower() not in {'', '0', 'false', 'none', 'no'}

def _eval_xacro_expr(expr, variables):
    expr=str(expr).strip()
    safe={'pi':math.pi, **{k:v for k,v in variables.items() if re.match(r'^[A-Za-z_]\w*$', str(k))}}
    try:
        return str(eval(expr, {'__builtins__':{}}, safe))
    except Exception:
        return str(variables.get(expr, '${'+expr+'}'))

def _substitute_xacro_text(value, args, package_map):
    if value is None:
        return value
    out=str(value)
    def repl_arg(m): return str(args.get(m.group(1).strip(), m.group(0)))
    def repl_find(m):
        pkg=m.group(1).strip()
        return str(package_map.get(pkg, m.group(0)))
    out=re.sub(r"\$\(arg\s+([^)]+)\)", repl_arg, out)
    out=re.sub(r"\$\(find\s+([^)]+)\)", repl_find, out)
    def repl_brace(m): return _eval_xacro_expr(m.group(1), args)
    out=re.sub(r"\$\{([^}]+)\}", repl_brace, out)
    return out

def _clean_xacro_tree(node, args, package_map):
    node=copy.deepcopy(node)
    if _xacro_tag(node):
        return []
    node.tag=tag_name(node)
    node.attrib={k:_substitute_xacro_text(v,args,package_map) for k,v in node.attrib.items()}
    if node.text:
        node.text=_substitute_xacro_text(node.text,args,package_map)
    cleaned=[]
    for child in list(node):
        xt=_xacro_tag(child)
        if xt == 'if':
            if _truthy(_substitute_xacro_text(child.attrib.get('value',''),args,package_map)):
                for grand in list(child):
                    cleaned.extend(_clean_xacro_tree(grand,args,package_map))
        elif xt:
            continue
        else:
            cleaned.extend(_clean_xacro_tree(child,args,package_map))
    node[:]=cleaned
    return [node]

def _resolve_include_path(filename, package_map):
    resolved=_substitute_xacro_text(filename, {}, package_map)
    p=Path(resolved)
    return p if p.exists() else None

def _collect_lite_macros(path, package_map, macros, seen):
    path=Path(path).resolve()
    if path in seen or not path.exists():
        return
    seen.add(path)
    try:
        root=ET.parse(path).getroot()
    except Exception:
        return
    for child in list(root):
        xt=_xacro_tag(child)
        if xt == 'include':
            inc=_resolve_include_path(child.attrib.get('filename',''), package_map)
            if inc: _collect_lite_macros(inc, package_map, macros, seen)
        elif xt == 'macro' and child.attrib.get('name'):
            macros[child.attrib['name']]={'params':child.attrib.get('params','').split(), 'body':list(child)}

def _expand_lite_node(node, args, package_map, macros):
    xt=_xacro_tag(node)
    if xt in {'arg','include','macro'}:
        return []
    if xt == 'if':
        if not _truthy(_substitute_xacro_text(node.attrib.get('value',''),args,package_map)):
            return []
        out=[]
        for child in list(node): out.extend(_expand_lite_node(child,args,package_map,macros))
        return out
    if xt and xt in macros:
        macro=macros[xt]
        local=dict(args)
        blocks={}
        positional=[]
        for param in macro['params']:
            if param.startswith('*'):
                blocks[param[1:]]=None
            else:
                positional.append(param.split(':=',1)[0])
                if ':=' in param:
                    k,d=param.split(':=',1); local[k]=_substitute_xacro_text(d,local,package_map)
        for k,v in node.attrib.items(): local[k]=_substitute_xacro_text(v,args,package_map)
        for child in list(node):
            blocks[tag_name(child)] = child
        out=[]
        for child in macro['body']:
            if _xacro_tag(child) == 'insert_block':
                block=blocks.get(child.attrib.get('name',''))
                if block is not None: out.extend(_clean_xacro_tree(block,local,package_map))
            else:
                out.extend(_expand_lite_node(child,local,package_map,macros))
        return out
    if xt:
        return []
    node=copy.deepcopy(node)
    node.tag=tag_name(node)
    node.attrib={k:_substitute_xacro_text(v,args,package_map) for k,v in node.attrib.items()}
    if node.text: node.text=_substitute_xacro_text(node.text,args,package_map)
    children=[]
    for child in list(node): children.extend(_expand_lite_node(child,args,package_map,macros))
    node[:]=children
    return [node]

def expand_xacro_lite(urdf_path, scene_dir=None, xacro_args=None, workspace_root=None):
    scene_dir = scene_dir or Path(urdf_path).parents[1]; xacro_args = dict(xacro_args or {})
    package_map, diagnostics = discover_package_map(scene_dir, workspace_root=workspace_root)
    try:
        root=ET.parse(urdf_path).getroot()
    except Exception as e:
        return None, False, f'xacro lite parse failed: {e}', None
    args=dict(xacro_args)
    for child in list(root):
        if _xacro_tag(child) == 'arg':
            name=child.attrib.get('name')
            if name and name not in args:
                args[name]=_substitute_xacro_text(child.attrib.get('default',''), args, package_map)
    macros={}
    seen=set()
    _collect_lite_macros(urdf_path, package_map, macros, seen)
    expanded_root=ET.Element('robot', {'name': root.attrib.get('name', scene_dir.name)})
    skipped=[]
    for child in list(root):
        xt=_xacro_tag(child)
        if xt and xt not in {'if'} and xt not in macros:
            if xt not in {'arg','include','macro'}:
                skipped.append(xt)
            continue
        for out in _expand_lite_node(child,args,package_map,macros):
            expanded_root.append(out)
    xml_text=ET.tostring(expanded_root, encoding='unicode')
    reason='xacro executable unavailable; used safe repo-local xacro-lite expansion'
    if skipped:
        reason += '; skipped unresolved macros: ' + ', '.join(sorted(set(skipped)))
    return xml_text, True, reason, ['xacro-lite', str(urdf_path)]

def expand_xacro(urdf_path, scene_dir=None, xacro_args=None, workspace_root=None):
    scene_dir = scene_dir or Path(urdf_path).parents[1]; xacro_args = xacro_args or {}
    xacro_base, xacro_available, reason = discover_xacro_command()
    if not xacro_available:
        return expand_xacro_lite(urdf_path, scene_dir, xacro_args, workspace_root)
    cmd=xacro_base+[str(urdf_path),'-o',str(scene_dir/'generated'/'expanded_scene_preview.urdf')]
    for k,v in xacro_args.items(): cmd.append(f'{k}:={v}')
    try:
        (scene_dir/'generated').mkdir(parents=True,exist_ok=True)
        p=subprocess.run(cmd,capture_output=True,text=True,timeout=45,env=xacro_env(scene_dir, workspace_root=workspace_root))
        if p.returncode!=0:
            lite_xml, _, lite_reason, lite_cmd = expand_xacro_lite(urdf_path, scene_dir, xacro_args, workspace_root)
            if lite_xml:
                return lite_xml, True, f"xacro failed ({(p.stderr or p.stdout or 'xacro failed').strip()}); {lite_reason}", lite_cmd
            return None,True,(p.stderr or p.stdout or 'xacro failed').strip(),cmd
        out=scene_dir/'generated'/'expanded_scene_preview.urdf'; return out.read_text(errors='ignore'),True,'',cmd
    except Exception as e:
        lite_xml, _, lite_reason, lite_cmd = expand_xacro_lite(urdf_path, scene_dir, xacro_args, workspace_root)
        if lite_xml:
            return lite_xml, True, f'xacro expansion failed: {e}; {lite_reason}', lite_cmd
        return None,True,f'xacro expansion failed: {e}',cmd



def append_static_robot_primitive_fallbacks(items, urdf_text, fallback_reason):
    """Add bounded robot primitives when xacro-lite cannot expand a known robot macro.

    These entries are preview-only visual fallbacks for Scene3D. They do not alter
    launch files, controllers, or runtime robot behaviour.
    """
    reason = (fallback_reason or '').lower()
    if 'ur_robot' not in reason and '<xacro:ur_robot' not in (urdf_text or ''):
        return 0
    existing_ids = {str(i.get('id') or '') for i in items}
    specs = [
        ('ur5_static_base', 'base_link', 'cylinder', [0.0, 0.0, 0.08], [0.0, 0.0, 0.0], {'radius': 0.18, 'length': 0.16}),
        ('ur5_static_shoulder', 'shoulder_link', 'box', [0.0, 0.0, 0.34], [0.0, 0.0, 0.0], {'size': [0.18, 0.18, 0.52]}),
        ('ur5_static_upper_arm', 'upper_arm_link', 'box', [0.27, 0.0, 0.58], [0.0, 0.35, 0.0], {'size': [0.54, 0.11, 0.13]}),
        ('ur5_static_forearm', 'forearm_link', 'box', [0.58, 0.0, 0.43], [0.0, -0.45, 0.0], {'size': [0.48, 0.10, 0.12]}),
        ('ur5_static_wrist_1', 'wrist_1_link', 'box', [0.82, 0.0, 0.31], [0.0, 0.0, 0.0], {'size': [0.12, 0.10, 0.16]}),
        ('ur5_static_wrist_2', 'wrist_2_link', 'box', [0.92, 0.0, 0.28], [0.0, 0.0, 0.0], {'size': [0.12, 0.10, 0.12]}),
        ('ur5_static_tool0', 'tool0', 'box', [1.02, 0.0, 0.28], [0.0, 0.0, 0.0], {'size': [0.08, 0.08, 0.08]}),
    ]
    added = 0
    for stable_id, link, geom, xyz, rpy, dims in specs:
        item_id = f'urdf_static_fallback_{stable_id}'
        if item_id in existing_ids:
            continue
        common = {
            'id': item_id,
            'link': link,
            'parent_link': 'world',
            'category': 'robot_static_primitive_fallback',
            'geometry_type': geom,
            'pose': {'xyz': xyz, 'rpy': rpy},
            'chain_pose': {'xyz': xyz, 'rpy': rpy},
            'world_pose': {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]},
            'visual_origin': {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]},
            'link_transform_status': 'static_fallback',
            'transform_status': 'static_fallback',
            'transform_chain': [],
            'render_expected': True,
            'resolved': True,
            'primitive_fallback': True,
            'fallback_reason': 'ur_robot xacro macro unavailable; emitted bounded Scene3D-only robot primitive fallback',
            'source_path': '',
            'resolved_source_path': '',
            'package_uri': '',
            'mesh_scale': [1.0, 1.0, 1.0],
            'material': {'name': 'scene3d_static_robot_fallback', 'color': [0.72, 0.76, 0.82, 0.62]},
        }
        common.update(dims)
        items.append(common)
        existing_ids.add(item_id)
        added += 1
    return added


def resolve_static_tool0_children(items):
    """Lift tool-attached visuals out of origin when the robot macro was statically approximated."""
    has_static_tool0 = any(str(i.get('id') or '') == 'urdf_static_fallback_ur5_static_tool0' for i in items)
    if not has_static_tool0:
        return 0
    fixed = 0
    tool_xyz = [1.06, 0.0, 0.28]
    for item in items:
        warning = str(item.get('warning') or '') + ' ' + str(item.get('render_skip_reason') or '')
        if 'missing_parent:tool0' not in warning:
            continue
        pose = item.get('pose') if isinstance(item.get('pose'), dict) else {}
        xyz = pose.get('xyz') if isinstance(pose.get('xyz'), list) else [0.0, 0.0, 0.0]
        while len(xyz) < 3:
            xyz.append(0.0)
        shifted = [float(xyz[0]) + tool_xyz[0], float(xyz[1]) + tool_xyz[1], float(xyz[2]) + tool_xyz[2]]
        pose['xyz'] = shifted
        item['pose'] = pose
        item['chain_pose'] = {'xyz': shifted, 'rpy': pose.get('rpy', [0.0, 0.0, 0.0])}
        item['transform_status'] = 'static_fallback_parent'
        item['transform_chain'] = item.get('transform_chain') if isinstance(item.get('transform_chain'), list) else []
        item['render_skip_reason'] = ''
        item['warning'] = 'static_parent_resolved:tool0 via Scene3D UR5 primitive fallback'
        item['static_parent_resolved'] = True
        fixed += 1
    return fixed

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
            visual_tf=tf_from_xyz_rpy(vxyz, vrpy)
            pose=xyz_rpy_from_tf(matmul4(link_tf, visual_tf))
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
            if isinstance(expanded_result, tuple) and len(expanded_result) >= 4:
                xml_text,_,err,xacro_cmd=expanded_result[:4]
            elif isinstance(expanded_result, tuple) and len(expanded_result) == 3:
                xml_text,mode_hint,err=expanded_result; xacro_cmd=[]
                if isinstance(err, list): err='; '.join(str(e) for e in err)
                if mode_hint and not xml_text: mode=str(mode_hint)
            else:
                xml_text,err,xacro_cmd='', 'xacro expansion failed: unexpected extractor result', []
            if xml_text:
                mode='xacro_lite_expanded' if xacro_cmd and xacro_cmd[0] == 'xacro-lite' else 'xacro_expanded'
                expanded_path='' if mode == 'xacro_lite_expanded' else 'generated/expanded_scene_preview.urdf'
                fallback_reason=err if mode == 'xacro_lite_expanded' else ''
            else: fallback_reason=err or missing_reason
        if a.require_xacro and not xacro_avail:
            print('xacro executable unavailable (required)'); return 2
        if not xml_text: xml_text=(urdf_path.read_text(errors='ignore') if urdf_path.exists() else '<robot/>')
        package_map, package_diagnostics = discover_package_map(scene_dir, workspace_root=(a.workspace_root or None))
        try: items=extract_from_urdf(xml_text, package_map)
        except Exception: items=[]
        static_robot_fallback_count = append_static_robot_primitive_fallbacks(items, xml_text, fallback_reason)
        static_parent_resolved_count = resolve_static_tool0_children(items)
        unresolved=[i for i in items if any(contains_placeholder(i.get(k,'')) for k in ('id','link','parent_link'))]
        safe=bool(items) and (len(unresolved)==0) and (mode in ('xacro_expanded','xacro_lite_expanded'))
        mesh_format_counts=dict(collections.Counter((Path(i.get('resolved_source_path') or i.get('source_path') or '').suffix.lower() or 'unknown') for i in items if i.get('geometry_type')=='mesh'))
        transform_status_counts=dict(collections.Counter(str(i.get('transform_status') or 'unknown') for i in items))
        renderable_count=sum(1 for i in items if i.get('render_expected', True))
        renderable_mesh_count=sum(1 for i in items if i.get('geometry_type')=='mesh' and i.get('render_expected', True))
        source_mtime=urdf_path.stat().st_mtime if urdf_path.exists() else None
        has_transform_collapse_warning=bool(items) and len({tuple((i.get('pose') or {}).get('xyz') or []) for i in items}) <= 1 and len(items) > 1
        payload={'scene_name':scene_dir.name,'visual_count':len(items),'resolved':sum(1 for i in items if i.get('resolved')),'unresolved':sum(1 for i in items if not i.get('resolved')),'generated_at':datetime.now(timezone.utc).isoformat(),'extractor_version':EXTRACTOR_VERSION,'extraction_mode':mode,'xacro_available':xacro_avail,'source_urdf_xacro_path':str(urdf_path),'source_mtime':source_mtime,'source_expanded_urdf_path':expanded_path,'fallback_reason':fallback_reason,'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'has_transform_collapse_warning':has_transform_collapse_warning,'candidate_mesh_count':len(items),'emitted_visual_count':len(items),'transform_status_counts':transform_status_counts,'mesh_format_counts':mesh_format_counts,'renderable_mesh_count':renderable_mesh_count,'renderable_item_count':renderable_count,'static_robot_primitive_fallback_count':static_robot_fallback_count,'static_parent_resolved_count':static_parent_resolved_count,'stale_index':False,'stale_reasons':[],'visual_items':items,'xacro_command':xacro_cmd,'package_resolution_diagnostics':package_diagnostics}
        idx_path=scene_dir/'generated/scene_visual_mesh_index.json'
        if not a.no_write:
            idx_path.parent.mkdir(parents=True,exist_ok=True)
            idx_path.write_text(json.dumps(payload,indent=2)+'\n')
        report['scene_count']+=1
        report['visual_count']+=len(items)
        report['resolved']+=sum(1 for i in items if i.get('resolved'))
        report['unresolved']+=sum(1 for i in items if not i.get('resolved'))
        report['xacro_expanded_count']+=int(mode in ('xacro_expanded','xacro_lite_expanded'))
        report['best_effort_count']+=int(mode not in ('xacro_expanded','xacro_lite_expanded'))
        report.setdefault('mesh_format_counts', {})
        for ext,count in mesh_format_counts.items(): report['mesh_format_counts'][ext]=report['mesh_format_counts'].get(ext,0)+count
        report['renderable_mesh_count']=report.get('renderable_mesh_count',0)+renderable_mesh_count
        report['renderable_item_count']=report.get('renderable_item_count',0)+renderable_count
        report['candidate_mesh_count']=report.get('candidate_mesh_count',0)+len(items)
        report['emitted_visual_count']=report.get('emitted_visual_count',0)+len(items)
        report['unresolved_placeholder_count']=report.get('unresolved_placeholder_count',0)+len(unresolved)
        report['scenes'].append({'scene':scene_dir.name,'extraction_mode':mode,'xacro_available':payload['xacro_available'],'expanded_urdf_written':bool(expanded_path),'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'mesh_backed_count':sum(1 for i in items if i.get('geometry_type')=='mesh'),'skipped_count':sum(1 for i in items if i.get('render_skip_reason')),'fallback_reason':fallback_reason,'urdf_primitive_count':sum(1 for i in items if i.get('geometry_type') in ('box','cylinder','sphere','capsule')),'primitive_fallback_count':static_robot_fallback_count,'stale_index':False,'status':'PASS' if safe else 'WARN'})
        if a.require_xacro and mode not in ('xacro_expanded','xacro_lite_expanded'): return 2
    (ROOT/'build').mkdir(exist_ok=True)
    (ROOT/'build/workcell_studio_urdf_visual_mesh_index_report.json').write_text(json.dumps(report,indent=2)+'\n')
    if a.fail_on_unexpanded and report['best_effort_count']>0: return 3
    return 0

if __name__=='__main__': raise SystemExit(main())
