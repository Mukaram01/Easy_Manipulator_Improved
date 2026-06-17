#!/usr/bin/env python3
from __future__ import annotations
import argparse, collections, copy, importlib, importlib.util, json, math, os, re, shutil, subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENES_ROOT = ROOT / "scenes"
EXTRACTOR_VERSION = "2.10"
UR5_INITIAL_POSITIONS_PATH = ROOT / "assets/robots/universal_robot/ur5_moveit_config/config/initial_positions.yaml"
UR5_INITIAL_JOINT_DEFAULTS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": 0.0,
    "elbow_joint": 0.0,
    "wrist_1_joint": 0.0,
    "wrist_2_joint": 0.0,
    "wrist_3_joint": 0.0,
}
UR5_VISUAL_MESH_URI_PREFIX = "package://ur_description/meshes/ur5/visual/"
PLACEHOLDER_RE = re.compile(r"(\$\{[^}]+\}|\$\(arg\s+[^)]+\)|\$\(find\s+[^)]+\))")
REPO_LOCAL_ASSET_PRECEDENCE_PACKAGES = {
    "robotiq_85_description",
    "workbench_description",
    "realsense2_description",
    "ur_description",
}

UR5_STATIC_VISUAL_SPECS = [
    {'stable_id': 'ur5_static_base', 'link': 'base_link', 'mesh': 'base.dae', 'geom': 'cylinder', 'xyz': [0.0, 0.0, 0.08], 'rpy': [0.0, 0.0, 0.0], 'dims': {'radius': 0.18, 'length': 0.16}},
    {'stable_id': 'ur5_static_shoulder', 'link': 'shoulder_link', 'mesh': 'shoulder.dae', 'geom': 'box', 'xyz': [0.0, 0.0, 0.34], 'rpy': [0.0, 0.0, 0.0], 'dims': {'size': [0.18, 0.18, 0.52]}},
    {'stable_id': 'ur5_static_upper_arm', 'link': 'upper_arm_link', 'mesh': 'upperarm.dae', 'geom': 'box', 'xyz': [0.27, 0.0, 0.58], 'rpy': [0.0, 0.35, 0.0], 'dims': {'size': [0.54, 0.11, 0.13]}},
    {'stable_id': 'ur5_static_forearm', 'link': 'forearm_link', 'mesh': 'forearm.dae', 'geom': 'box', 'xyz': [0.58, 0.0, 0.43], 'rpy': [0.0, -0.45, 0.0], 'dims': {'size': [0.48, 0.10, 0.12]}},
    {'stable_id': 'ur5_static_wrist_1', 'link': 'wrist_1_link', 'mesh': 'wrist1.dae', 'geom': 'box', 'xyz': [0.82, 0.0, 0.31], 'rpy': [0.0, 0.0, 0.0], 'dims': {'size': [0.12, 0.10, 0.16]}},
    {'stable_id': 'ur5_static_wrist_2', 'link': 'wrist_2_link', 'mesh': 'wrist2.dae', 'geom': 'box', 'xyz': [0.92, 0.0, 0.28], 'rpy': [0.0, 0.0, 0.0], 'dims': {'size': [0.12, 0.10, 0.12]}},
    {'stable_id': 'ur5_static_wrist_3', 'link': 'wrist_3_link', 'mesh': 'wrist3.dae', 'geom': 'box', 'xyz': [0.98, 0.0, 0.28], 'rpy': [0.0, 0.0, 0.0], 'dims': {'size': [0.10, 0.10, 0.10]}},
    {'stable_id': 'ur5_static_tool0', 'link': 'tool0', 'mesh': '', 'geom': 'box', 'xyz': [1.02, 0.0, 0.28], 'rpy': [0.0, 0.0, 0.0], 'dims': {'size': [0.08, 0.08, 0.08]}},
]

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
def tf_from_axis_angle(axis, angle):
    ax = parse_vec(" ".join(str(v) for v in (axis or [])), 3, 0.0) if not isinstance(axis, str) else parse_vec(axis, 3, 0.0)
    x, y, z = (ax + [0.0, 0.0, 0.0])[:3]
    norm = math.sqrt(x*x + y*y + z*z)
    if norm <= 1e-12:
        x, y, z = 1.0, 0.0, 0.0
    else:
        x, y, z = x/norm, y/norm, z/norm
    c = math.cos(float(angle or 0.0)); s = math.sin(float(angle or 0.0)); t = 1.0 - c
    out = identity_tf()
    rot = [
        [t*x*x + c, t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c, t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c],
    ]
    for r in range(3):
        for col in range(3): out[r][col] = rot[r][col]
    return out

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



def _repo_relative_path(path_value):
    """Return a repository-relative path for repo-local filesystem paths.

    Generated mesh indexes are committed artifacts, so they must not embed the
    transient checkout directory (for example /workspace/...).  Keep external
    absolute paths untouched, but make repo-local paths portable.
    """
    if path_value is None:
        return ''
    text = str(path_value)
    if not text:
        return ''
    if text.startswith(('package://', 'http://', 'https://', 'model://')):
        return text
    path = Path(text)
    if not path.is_absolute():
        return text
    try:
        return path.resolve().relative_to(ROOT).as_posix()
    except Exception:
        return text


def _portable_source_metadata(value):
    """Recursively strip the repo checkout prefix from generated metadata."""
    if isinstance(value, dict):
        return {key: _portable_source_metadata(child) for key, child in value.items()}
    if isinstance(value, list):
        return [_portable_source_metadata(child) for child in value]
    if isinstance(value, str):
        return _repo_relative_path(value)
    return value

def contains_placeholder(v): return bool(PLACEHOLDER_RE.search(json.dumps(v) if isinstance(v,(list,tuple,dict)) else str(v)))
def sanitize(s): return re.sub(r'[^A-Za-z0-9_]+','_',str(s or 'unnamed')).strip('_') or 'unnamed'
def tag_name(e): return e.tag.split('}')[-1]

def _package_name_from_xml(pkg_xml):
    pkg_dir = Path(pkg_xml).parent
    pkg_name = pkg_dir.name
    parse_error = ''
    try:
        xml_root = ET.parse(pkg_xml).getroot()
        name_node = xml_root.find('name')
        if name_node is not None and (name_node.text or '').strip():
            pkg_name = name_node.text.strip()
    except Exception as e:
        parse_error = str(e)
    return pkg_name, parse_error

def _ros_resolution_env(workspace_root=None):
    env=dict(os.environ)
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
    if ament_prefix_entries:
        env['AMENT_PREFIX_PATH']=os.pathsep.join(_dedupe_path_entries(ament_prefix_entries))
    return env

def resolve_ros_package_share(package_name, workspace_root=None):
    """Resolve a ROS package share directory through installed ROS mechanisms.

    Returns ``(share_path, source_tier, diagnostics)``. ``share_path`` is ``None``
    when neither ament index nor ``ros2 pkg prefix`` can resolve the package.
    """
    diagnostics={'package_name':package_name,'attempts':[]}
    if not package_name:
        diagnostics['error']='empty_package_name'
        return None, '', diagnostics

    has_ament_index = importlib.util.find_spec('ament_index_python') is not None
    has_ament_packages = has_ament_index and importlib.util.find_spec('ament_index_python.packages') is not None
    if has_ament_packages:
        ament_packages = importlib.import_module('ament_index_python.packages')
        try:
            share = Path(ament_packages.get_package_share_directory(package_name))
            diagnostics['attempts'].append({'source_tier':'ament_index','root_path':str(share),'status':'resolved'})
            if share.exists():
                return share, 'ament_index', diagnostics
            diagnostics['attempts'][-1]['status']='missing_path'
        except Exception as e:
            diagnostics['attempts'].append({'source_tier':'ament_index','status':'unresolved','error':str(e)})
    else:
        diagnostics['attempts'].append({'source_tier':'ament_index','status':'unavailable','error':'ament_index_python.packages import unavailable'})

    ros2 = shutil.which('ros2') or ('/opt/ros/humble/bin/ros2' if Path('/opt/ros/humble/bin/ros2').exists() else '')
    if ros2:
        try:
            proc=subprocess.run(
                [ros2,'pkg','prefix',package_name],
                capture_output=True,
                text=True,
                timeout=10,
                env=_ros_resolution_env(workspace_root),
            )
            if proc.returncode == 0 and proc.stdout.strip():
                share = Path(proc.stdout.strip())/'share'/package_name
                diagnostics['attempts'].append({'source_tier':'ros2_pkg_prefix','prefix':proc.stdout.strip(),'root_path':str(share),'status':'resolved' if share.exists() else 'missing_path'})
                if share.exists():
                    return share, 'ros2_pkg_prefix', diagnostics
            else:
                diagnostics['attempts'].append({'source_tier':'ros2_pkg_prefix','status':'unresolved','error':(proc.stderr or proc.stdout or '').strip()})
        except Exception as e:
            diagnostics['attempts'].append({'source_tier':'ros2_pkg_prefix','status':'unresolved','error':str(e)})
    else:
        diagnostics['attempts'].append({'source_tier':'ros2_pkg_prefix','status':'unavailable','error':'ros2 executable unavailable'})

    return None, '', diagnostics

def _fallback_package_candidates(scene_dir, workspace_root=None):
    ws = Path(workspace_root) if workspace_root else None
    roots=[
        ('scene_generated_package', scene_dir/'generated'),
        ('workspace_install_share', (ws/'install/share') if ws else None),
        ('workspace_src_easy_manipulation_deployment_assets', (ws/'src/easy_manipulation_deployment/assets') if ws else None),
        ('workspace_src_assets', (ws/'src/assets') if ws else None),
        ('repo_assets', ROOT/'assets'),
        ('ros_humble_share', Path('/opt/ros/humble/share')),
    ]
    candidates=[]
    for tier, root in roots:
        if root is None or not root.exists():
            continue
        for pkg_xml in root.rglob('package.xml'):
            pkg_dir = pkg_xml.parent
            pkg_name, parse_error = _package_name_from_xml(pkg_xml)
            candidate={'package_name':pkg_name,'root_path':str(pkg_dir),'source_tier':tier,'package_xml':str(pkg_xml)}
            if parse_error:
                candidate['name_fallback']='directory_name'
                candidate['parse_error']=parse_error
            candidates.append(candidate)
    return sorted(candidates, key=lambda c: (str(c.get('package_name', '')), str(c.get('root_path', '')), str(c.get('source_tier', ''))))

def _record_package_resolution(out, diagnostics, candidate):
    pkg_name = candidate['package_name']
    if pkg_name in out:
        diagnostics['shadowed_packages'].append({
            **candidate,
            'shadowed_by':str(out[pkg_name]),
        })
        return False
    out[pkg_name]=Path(candidate['root_path'])
    diagnostics['resolved_packages'].append(candidate)
    diagnostics['resolution_paths'].append({'package_name':pkg_name,'root_path':candidate['root_path'],'source_tier':candidate['source_tier']})
    return True

def extract_referenced_package_names(text):
    text = text or ''
    names=set(re.findall(r"package://([^/\s\"\']+)", text))
    names.update(n.strip() for n in re.findall(r'\$\(find\s+([^)]+)\)', text) if n.strip())
    return sorted(names)

def discover_package_map(scene_dir, workspace_root=None, package_names=None):
    out={}
    diagnostics={'resolved_packages':[], 'shadowed_packages':[], 'resolution_paths':[], 'ros_resolution_attempts':[], 'unresolved_packages':[]}
    fallback_candidates=_fallback_package_candidates(Path(scene_dir), workspace_root=workspace_root)
    by_name=collections.OrderedDict()
    for candidate in fallback_candidates:
        by_name.setdefault(candidate['package_name'], []).append(candidate)
    for pkg_name in package_names or []:
        by_name.setdefault(pkg_name, [])

    # Some developer checkouts keep UR description assets under the repository's
    # Universal Robot asset bundle instead of an installed ROS package. Treat
    # those directories as package candidates only when the expected mesh tree is
    # present so package://ur_description/... can become mesh-backed without
    # hardcoding a scene name.
    if 'ur_description' in by_name:
        ur_asset_candidates = [
            ROOT/'assets/robots/universal_robot/ur_description',
            ROOT/'assets/robots/universal_robot',
        ]
        for ur_root in ur_asset_candidates:
            if (ur_root/'meshes/ur5/visual').exists():
                by_name['ur_description'].append({
                    'package_name':'ur_description',
                    'root_path':str(ur_root),
                    'source_tier':'repo_assets_ur_description',
                    'package_xml':str(ur_root/'package.xml'),
                })
                break

    for pkg_name, candidates in by_name.items():
        repo_local_candidates=[c for c in candidates if c['source_tier'] in {'repo_assets','repo_assets_ur_description','workspace_src_easy_manipulation_deployment_assets','workspace_src_assets'}]
        if pkg_name in REPO_LOCAL_ASSET_PRECEDENCE_PACKAGES and repo_local_candidates:
            _record_package_resolution(out, diagnostics, repo_local_candidates[0])
            for candidate in candidates:
                if candidate is not repo_local_candidates[0]:
                    diagnostics['shadowed_packages'].append({**candidate,'shadowed_by':str(out[pkg_name])})
            continue

        share_path, source_tier, ros_diag = resolve_ros_package_share(pkg_name, workspace_root=workspace_root)
        diagnostics['ros_resolution_attempts'].append(ros_diag)
        if share_path:
            ros_candidate={'package_name':pkg_name,'root_path':str(share_path),'source_tier':source_tier,'package_xml':str(share_path/'package.xml')}
            _record_package_resolution(out, diagnostics, ros_candidate)
            for candidate in candidates:
                if Path(candidate['root_path']).resolve() != share_path.resolve():
                    diagnostics['shadowed_packages'].append({**candidate,'shadowed_by':str(out[pkg_name])})
            continue

        if not candidates:
            diagnostics['unresolved_packages'].append({'package_name':pkg_name,'source_tier':'unresolved','reason':'not_found_by_ament_ros2_or_package_xml_scan'})
            continue
        _record_package_resolution(out, diagnostics, candidates[0])
        for candidate in candidates[1:]:
            diagnostics['shadowed_packages'].append({**candidate,'shadowed_by':str(out[pkg_name])})
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
    diagnostics=[]

    path_xacro = shutil.which('xacro')
    if path_xacro:
        return [str(Path(path_xacro).resolve())], True, ''
    diagnostics.append('PATH xacro not found')

    ros_humble_xacro = Path('/opt/ros/humble/bin/xacro')
    if ros_humble_xacro.exists():
        return [str(ros_humble_xacro.resolve())], True, ''
    diagnostics.append('/opt/ros/humble/bin/xacro missing')

    try:
        if importlib.util.find_spec('xacro') is not None:
            return ['python3', '-m', 'xacro'], True, ''
        diagnostics.append('python xacro module import failed: module spec not found')
    except Exception as e:
        diagnostics.append(f'python xacro module import failed: {e}')

    return None, False, '; '.join(diagnostics)

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

def expand_xacro_lite(urdf_path, scene_dir=None, xacro_args=None, workspace_root=None, reason_prefix=None):
    scene_dir = scene_dir or Path(urdf_path).parents[1]; xacro_args = dict(xacro_args or {})
    source_text = Path(urdf_path).read_text(errors='ignore') if Path(urdf_path).exists() else ''
    package_map, diagnostics = discover_package_map(scene_dir, workspace_root=workspace_root, package_names=extract_referenced_package_names(source_text))
    try:
        root=ET.fromstring(source_text) if source_text else ET.parse(urdf_path).getroot()
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
    reason=reason_prefix or 'xacro executable unavailable; used safe repo-local xacro-lite expansion'
    if skipped:
        reason += '; skipped unresolved macros: ' + ', '.join(sorted(set(skipped)))
    return xml_text, True, reason, ['xacro-lite', str(urdf_path)]

def _explicit_lite_fallback_requested(explicit_lite_fallback=False):
    env_value = os.environ.get('WORKCELL_ALLOW_XACRO_LITE_FALLBACK', '')
    return bool(explicit_lite_fallback) or _truthy(env_value)


def _xacro_failure_diagnostic(status, command, returncode=None, stdout='', stderr='', reason=''):
    diagnostic = {
        'xacro_status': status,
        'xacro_command': list(command or []),
        'xacro_returncode': returncode,
        'xacro_stdout': stdout or '',
        'xacro_stderr': stderr or '',
        'xacro_diagnostic': reason or '',
    }
    parts = [status.replace('_', ' ')]
    if returncode is not None:
        parts.append(f'returncode={returncode}')
    if stderr:
        parts.append('stderr: ' + stderr.strip())
    if stdout:
        parts.append('stdout: ' + stdout.strip())
    if reason and not (stderr or stdout):
        parts.append(reason)
    diagnostic['fallback_reason'] = '; '.join(p for p in parts if p)
    return diagnostic


def expand_xacro(urdf_path, scene_dir=None, xacro_args=None, workspace_root=None, explicit_lite_fallback=False):
    scene_dir = scene_dir or Path(urdf_path).parents[1]; xacro_args = xacro_args or {}
    xacro_base, xacro_available, reason = discover_xacro_command()
    if not xacro_available:
        lite = expand_xacro_lite(urdf_path, scene_dir, xacro_args, workspace_root)
        if isinstance(lite, tuple) and len(lite) >= 4:
            xml_text, ok, lite_reason, lite_cmd = lite[:4]
            diag = _xacro_failure_diagnostic('real_xacro_unavailable', lite_cmd, reason=lite_reason or reason)
            diag['xacro_lite_command'] = list(lite_cmd or [])
            return xml_text, ok, lite_reason, lite_cmd, diag
        return lite
    cmd=xacro_base+[str(urdf_path),'-o',str(scene_dir/'generated'/'expanded_scene_preview.urdf')]
    for k,v in xacro_args.items(): cmd.append(f'{k}:={v}')
    try:
        (scene_dir/'generated').mkdir(parents=True,exist_ok=True)
        p=subprocess.run(cmd,capture_output=True,text=True,timeout=45,env=xacro_env(scene_dir, workspace_root=workspace_root))
        if p.returncode!=0:
            diag = _xacro_failure_diagnostic('real_xacro_failed', cmd, returncode=p.returncode, stdout=p.stdout, stderr=p.stderr)
            if _explicit_lite_fallback_requested(explicit_lite_fallback):
                lite_xml, _, lite_reason, lite_cmd = expand_xacro_lite(urdf_path, scene_dir, xacro_args, workspace_root, reason_prefix='explicit xacro-lite fallback requested after real xacro failure')
                diag['xacro_status'] = 'explicit_xacro_lite_fallback' if lite_xml else 'real_xacro_failed_lite_fallback_failed'
                diag['xacro_lite_command'] = list(lite_cmd or [])
                diag['xacro_lite_reason'] = lite_reason
                if lite_xml:
                    return lite_xml, True, f"{diag['fallback_reason']}; {lite_reason}", cmd, diag
            return None, True, diag['fallback_reason'], cmd, diag
        out=scene_dir/'generated'/'expanded_scene_preview.urdf'
        diag = _xacro_failure_diagnostic('real_xacro_succeeded', cmd, returncode=p.returncode, stdout=p.stdout, stderr=p.stderr)
        return out.read_text(errors='ignore'),True,'',cmd,diag
    except Exception as e:
        diag = _xacro_failure_diagnostic('real_xacro_failed', cmd, reason=f'xacro expansion failed: {e}')
        if _explicit_lite_fallback_requested(explicit_lite_fallback):
            lite_xml, _, lite_reason, lite_cmd = expand_xacro_lite(urdf_path, scene_dir, xacro_args, workspace_root, reason_prefix='explicit xacro-lite fallback requested after real xacro exception')
            diag['xacro_status'] = 'explicit_xacro_lite_fallback' if lite_xml else 'real_xacro_failed_lite_fallback_failed'
            diag['xacro_lite_command'] = list(lite_cmd or [])
            diag['xacro_lite_reason'] = lite_reason
            if lite_xml:
                return lite_xml, True, f"{diag['fallback_reason']}; {lite_reason}", cmd, diag
        return None, True, diag['fallback_reason'], cmd, diag





def preview_degraded_fallback_warning(fallback_reason, extraction_mode=''):
    text = str(fallback_reason or '')
    if 'skipped unresolved macros' in text.lower() and _contains_unresolved_ur_robot(text):
        return 'xacro-lite skipped robot macro ur_robot; preview uses degraded fallback geometry'
    if str(extraction_mode or '') in {'xacro_lite_expanded', 'xacro_lite_fallback'} and _contains_unresolved_ur_robot(text):
        return 'xacro-lite skipped robot macro ur_robot; preview uses degraded fallback geometry'
    return ''

def is_preview_fully_healthy(items, unresolved, extraction_mode, fallback_reason, renderable_mesh_count=None):
    if renderable_mesh_count is None:
        renderable_mesh_count = sum(1 for item in items or [] if item.get('geometry_type') == 'mesh' and item.get('render_expected', True))
    if preview_degraded_fallback_warning(fallback_reason, extraction_mode):
        return False
    return bool(items) and len(unresolved or []) == 0 and extraction_mode == 'xacro_expanded' and renderable_mesh_count > 0

def _contains_unresolved_ur_robot(text):
    text = text or ''
    lowered = text.lower()
    if 'ur_robot' not in lowered:
        return False
    return bool(re.search(r"<\s*(?:[A-Za-z_][\w.-]*:)?ur_robot\b", text, re.IGNORECASE)) or 'ur_robot' in lowered

def _has_ur5_visual_mesh_uri(items):
    for item in items or []:
        uri_candidates = [item.get('package_uri'), item.get('source_path')]
        if any(str(uri or '').startswith(UR5_VISUAL_MESH_URI_PREFIX) for uri in uri_candidates):
            return True
    return False

def _ur5_visual_mesh_diagnostics(package_map, package_diagnostics, generated_count, fallback_count):
    diag = {
        'package_name': 'ur_description',
        'source': 'missing',
        'root_path': '',
        'visual_folder': '',
        'visual_folder_exists': False,
        'mesh_files_found': 0,
        'mesh_items_generated': generated_count,
        'static_fallback_items_generated': fallback_count,
        'status': 'ur_description package missing',
    }
    root = package_map.get('ur_description')
    if not root:
        return diag
    diag['root_path'] = _repo_relative_path(root)
    resolved = [
        r for r in (package_diagnostics or {}).get('resolution_paths', [])
        if r.get('package_name') == 'ur_description'
    ]
    source_tier = str(resolved[-1].get('source_tier', '')) if resolved else ''
    diag['source'] = 'repo_assets/ur_description' if source_tier in {'repo_assets', 'repo_assets_ur_description'} else (source_tier or 'package_map')
    visual_folder = Path(root) / 'meshes/ur5/visual'
    diag['visual_folder'] = _repo_relative_path(visual_folder)
    if not visual_folder.exists():
        alternatives = sorted(str(p.relative_to(root)) for p in (Path(root) / 'meshes').glob('*/visual') if p.is_dir()) if (Path(root) / 'meshes').exists() else []
        diag['alternate_visual_folders'] = alternatives
        diag['status'] = 'UR5 visual folder missing'
        return diag
    mesh_files = sorted(p for p in visual_folder.iterdir() if p.is_file() and p.suffix.lower() in {'.dae', '.stl', '.obj'})
    diag['visual_folder_exists'] = True
    diag['mesh_files_found'] = len(mesh_files)
    expected = ['base.dae','shoulder.dae','upperarm.dae','forearm.dae','wrist1.dae','wrist2.dae','wrist3.dae']
    missing = [name for name in expected if not (visual_folder / name).exists()]
    diag['missing_expected_files'] = missing
    if missing:
        diag['status'] = 'UR5 expected visual mesh files missing'
    elif generated_count <= 0:
        diag['status'] = 'UR5 mesh files found but visual items were not generated'
    else:
        diag['status'] = 'resolved'
    return diag

def _log_ur5_visual_mesh_diagnostics(scene_name, diag):
    print(f"[scene_visual_mesh_index] {scene_name}: UR5 visual source: {diag.get('source')}")
    print(f"[scene_visual_mesh_index] {scene_name}: UR5 visual mesh files found: {diag.get('mesh_files_found', 0)}")
    print(f"[scene_visual_mesh_index] {scene_name}: UR5 visual mesh items generated: {diag.get('mesh_items_generated', 0)}")
    print(f"[scene_visual_mesh_index] {scene_name}: UR5 static fallback items generated: {diag.get('static_fallback_items_generated', 0)}")
    if diag.get('status') != 'resolved':
        print(f"[scene_visual_mesh_index] {scene_name}: UR5 visual diagnostic: {diag.get('status')}")
        if diag.get('alternate_visual_folders'):
            print(f"[scene_visual_mesh_index] {scene_name}: UR5 alternate visual folders detected: {', '.join(diag.get('alternate_visual_folders') or [])}")

def append_static_ur5_mesh_visuals(items, package_map):
    """Emit Scene3D UR5 mesh visuals when package://ur_description meshes resolve.

    This is a scene-agnostic recovery path for lightweight xacro expansion: if the
    xacro macro cannot be expanded but repository/ROS package assets can resolve,
    emit mesh visual metadata instead of immediately falling back to primitives.
    """
    if _has_ur5_visual_mesh_uri(items):
        return 0
    existing_ids = {str(i.get('id') or '') for i in items}
    added = 0
    for spec in UR5_STATIC_VISUAL_SPECS:
        mesh_name = spec.get('mesh') or ''
        if not mesh_name:
            continue
        item_id = f"urdf_static_mesh_{spec['stable_id']}"
        if item_id in existing_ids:
            continue
        package_uri = UR5_VISUAL_MESH_URI_PREFIX + mesh_name
        resolved_path, error = resolve_mesh_uri(package_uri, package_map)
        if error or not resolved_path:
            continue
        xyz = list(spec['xyz'])
        rpy = list(spec['rpy'])
        items.append({
            'id': item_id,
            'link': spec['link'],
            'visual': 'static_mesh_visual',
            'parent_link': 'world',
            'category': 'robot_static_mesh_visual',
            'geometry_type': 'mesh',
            'pose': {'xyz': xyz, 'rpy': rpy},
            'chain_pose': {'xyz': xyz, 'rpy': rpy},
            'world_pose': {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]},
            'visual_origin': {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]},
            'link_transform_status': 'static_mesh_resolved',
            'transform_status': 'static_mesh_resolved',
            'transform_chain': [],
            'render_expected': True,
            'resolved': True,
            'primitive_fallback': False,
            'fallback_reason': '',
            'source_path': package_uri,
            'resolved_source_path': _repo_relative_path(resolved_path),
            'resolved_source_path_is_repo_relative': bool(resolved_path and _repo_relative_path(resolved_path) != str(resolved_path)),
            'package_uri': package_uri,
            'mesh_scale': [1.0, 1.0, 1.0],
            'material': {'name': 'scene3d_static_robot_mesh', 'color': None},
            'warning': 'ur_robot xacro macro unavailable; used resolved UR mesh asset with static preview pose',
        })
        existing_ids.add(item_id)
        added += 1
    return added


def append_static_robot_primitive_fallbacks(items, urdf_text, fallback_reason, extraction_mode='best_effort_recursive', source_xacro_text=''):
    """Add bounded robot primitives when real xacro UR mesh expansion is unavailable.

    These entries are preview-only visual fallbacks for Scene3D. They do not alter
    launch files, controllers, or runtime robot behaviour.
    """
    fallback_eligible_modes = {'xacro_lite_expanded', 'best_effort_recursive', 'best_effort', 'raw_fallback', 'raw'}
    if extraction_mode not in fallback_eligible_modes:
        return 0
    if not (
        _contains_unresolved_ur_robot(fallback_reason)
        or _contains_unresolved_ur_robot(source_xacro_text)
        or _contains_unresolved_ur_robot(urdf_text)
    ):
        return 0
    if _has_ur5_visual_mesh_uri(items):
        return 0
    existing_ids = {str(i.get('id') or '') for i in items}
    specs = UR5_STATIC_VISUAL_SPECS
    added = 0
    existing_mesh_links = {str(i.get('link') or '') for i in items if i.get('geometry_type') == 'mesh' and str(i.get('package_uri') or '').startswith(UR5_VISUAL_MESH_URI_PREFIX) and i.get('resolved')}
    for spec in specs:
        stable_id = spec['stable_id']; link = spec['link']; geom = spec['geom']; xyz = spec['xyz']; rpy = spec['rpy']; dims = spec['dims']
        if link in existing_mesh_links:
            continue
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


def read_ur5_initial_joint_positions(path=UR5_INITIAL_POSITIONS_PATH):
    defaults = dict(UR5_INITIAL_JOINT_DEFAULTS)
    data = read_yaml(path)
    positions = data.get('initial_positions') if isinstance(data, dict) else None
    if not isinstance(positions, dict):
        print('[scene_visual_mesh_index] UR5 initial joint source: default')
        return defaults, 'default'
    resolved = dict(defaults)
    for name, value in positions.items():
        try:
            resolved[str(name)] = float(value)
        except Exception:
            pass
    source = _repo_relative_path(path)
    print(f'[scene_visual_mesh_index] UR5 initial joint source: {source}')
    return resolved, source

def extract_from_urdf(xml_text, package_map, include_diagnostics=False):
    root=ET.fromstring(xml_text); items=[]; idx=0
    initial_joint_positions, initial_joint_source = read_ur5_initial_joint_positions()
    diagnostics={
        'root_links': [],
        'visual_parent_link_counts': {},
        'missing_parent_links': [],
        'transform_chain_diagnostics': [],
        'initial_joint_source': initial_joint_source,
    }
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
        axis=next((c for c in list(j) if tag_name(c)=='axis'),None)
        if parent is None or child is None: continue
        xyz=parse_vec(origin.attrib.get('xyz') if origin is not None else '',3,0.0); rpy=parse_vec(origin.attrib.get('rpy') if origin is not None else '',3,0.0)
        axis_xyz=parse_vec(axis.attrib.get('xyz') if axis is not None else '1 0 0',3,0.0)
        joint_name=j.attrib.get('name','')
        joint_type=j.attrib.get('type','fixed')
        joint_value=0.0 if joint_type == 'fixed' else float(initial_joint_positions.get(joint_name, 0.0))
        joints.append({'name':joint_name,'type':joint_type,'parent':parent.attrib.get('link',''),'child':child.attrib.get('link',''),'origin_xyz':xyz,'origin_rpy':rpy,'axis':axis_xyz,'value':joint_value})
    child_to_joint={j['child']:j for j in joints if j['child']}
    roots=sorted(n for n in links.keys() if n and n not in child_to_joint)
    diagnostics['root_links'] = roots
    missing_parent_names=collections.Counter()
    visual_parent_counts=collections.Counter()
    chain_diagnostics_by_link={}
    cache={}
    def link_world_tf(link_name):
        if link_name in cache: return cache[link_name]
        if link_name not in links: return None,'missing_link',[],link_name,'missing_link',None
        if link_name in roots:
            result=(identity_tf(),'resolved',[],link_name,'',None)
            cache[link_name]=result
            return result
        seen=set()
        def resolve(cur):
            if cur in cache: return cache[cur]
            if cur in seen:
                return identity_tf(),'unresolved',[],cur,'cycle',None
            seen.add(cur)
            j=child_to_joint.get(cur)
            if not j:
                unresolved='' if cur in roots else 'unresolved_chain'
                return identity_tf(),('resolved' if not unresolved else 'unresolved'),[],cur,unresolved,None
            parent_tf,parent_status,parent_chain,root_link,unresolved,_ = resolve(j['parent'])
            joint_origin=tf_from_xyz_rpy(j['origin_xyz'],j['origin_rpy'])
            if j['type'] == 'fixed':
                joint_tf = joint_origin
            else:
                joint_tf = matmul4(joint_origin, tf_from_axis_angle(j['axis'], j['value']))
            tf=matmul4(parent_tf,joint_tf)
            chain=[*parent_chain, f"{j['parent']}->{cur}({j['type']})"]
            status=parent_status
            if j['parent'] not in links:
                unresolved=f"missing_parent:{j['parent']}"
                status='unresolved'
            elif unresolved:
                status='unresolved'
            result=(tf,status,chain,root_link,unresolved,j)
            cache[cur]=result
            return result
        result=resolve(link_name)
        tf,status,chain,cur,unresolved,joint=result
        if unresolved:
            diagnostic={
                'link': link_name,
                'status': status,
                'terminal_link': cur or '',
                'unresolved_reason': unresolved,
                'chain': chain,
            }
            if unresolved.startswith('missing_parent:'):
                missing_name=unresolved.split(':',1)[1]
                diagnostic['missing_parent_link'] = missing_name
                diagnostic['missing_parent_joint'] = joint.get('name','') if joint else ''
                missing_parent_names[missing_name] += 1
            chain_diagnostics_by_link[link_name]=diagnostic
        cache[link_name]=result
        return result
    for lname,link in links.items():
        for visual in [c for c in list(link) if tag_name(c)=='visual']:
            geom=next((c for c in list(visual) if tag_name(c)=='geometry'),None)
            if geom is None: continue
            vname=visual.attrib.get('name',f'visual_{idx}'); item_id=f'urdf_visual_{idx}_{sanitize(lname)}_{sanitize(vname)}'
            origin=next((c for c in list(visual) if tag_name(c)=='origin'), None)
            vxyz=parse_vec((origin.attrib.get('xyz') if origin is not None else ''),3,0.0); vrpy=parse_vec((origin.attrib.get('rpy') if origin is not None else ''),3,0.0)
            link_tf, link_status, chain, root_link, unresolved, joint_meta = link_world_tf(lname)
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
            visual_parent_counts[root_link or ''] += 1
            link_pose=xyz_rpy_from_tf(link_tf)
            joint_name=(joint_meta or {}).get('name','')
            joint_value=(joint_meta or {}).get('value',0.0)
            joint_axis=(joint_meta or {}).get('axis',[1.0,0.0,0.0])
            common={'id':item_id,'link':lname,'visual':vname,'parent_link':root_link or '','pose':pose,'chain_pose':pose,'world_pose':link_pose,'visual_origin':{'xyz':vxyz,'rpy':vrpy},'joint_name':joint_name,'joint_value':joint_value,'joint_axis':joint_axis,'material':material,'link_transform_status':link_status,'transform_status':link_status,'transform_chain':chain,'render_expected':True}
            mesh=next((c for c in list(geom) if tag_name(c)=='mesh'),None)
            box=next((c for c in list(geom) if tag_name(c)=='box'),None)
            cyl=next((c for c in list(geom) if tag_name(c)=='cylinder'),None)
            sph=next((c for c in list(geom) if tag_name(c)=='sphere'),None)
            cap=next((c for c in list(geom) if tag_name(c)=='capsule'),None)
            warning=unresolved
            if mesh is not None:
                mesh_uri=mesh.attrib.get('filename',''); resolved_path, skip_reason=resolve_mesh_uri(mesh_uri, package_map)
                scale=parse_vec(mesh.attrib.get('scale','1 1 1'),3,1.0)
                items.append({**common,'geometry_type':'mesh','package_uri':mesh_uri if mesh_uri.startswith('package://') else '','source_path':mesh_uri,'resolved_source_path':_repo_relative_path(resolved_path),'resolved_source_path_is_repo_relative': bool(resolved_path and _repo_relative_path(resolved_path) != str(resolved_path)),'resolved':bool(resolved_path and not skip_reason),'mesh_scale':scale,'render_expected':not bool(skip_reason),'render_skip_reason':skip_reason,'warning':(skip_reason or warning or '')})
            elif box is not None:
                items.append({**common,'geometry_type':'box','size':parse_vec(box.attrib.get('size','0.1 0.1 0.1'),3,0.1),'resolved':True,'warning':warning or ''})
            elif cyl is not None:
                items.append({**common,'geometry_type':'cylinder','radius':float(cyl.attrib.get('radius','0.05') or 0.05),'length':float(cyl.attrib.get('length','0.1') or 0.1),'resolved':True,'warning':warning or ''})
            elif sph is not None:
                items.append({**common,'geometry_type':'sphere','radius':float(sph.attrib.get('radius','0.05') or 0.05),'resolved':True,'warning':warning or ''})
            elif cap is not None:
                items.append({**common,'geometry_type':'capsule','radius':float(cap.attrib.get('radius','0.05') or 0.05),'length':float(cap.attrib.get('length',cap.attrib.get('height','0.1')) or 0.1),'resolved':True,'warning':warning or ''})
            idx+=1
    diagnostics['visual_parent_link_counts'] = dict(sorted(visual_parent_counts.items()))
    diagnostics['missing_parent_links'] = [
        {'link': name, 'visual_count': count}
        for name, count in sorted(missing_parent_names.items())
    ]
    diagnostics['transform_chain_diagnostics'] = list(chain_diagnostics_by_link.values())
    if include_diagnostics:
        return items, diagnostics
    return items


def supported_robot_root_diagnostics(scene_name, items, urdf_diagnostics):
    supported_prefixes=('ur5_', 'ur3_', 'ur10_')
    supported_names={'suction_test'}
    if not (scene_name.startswith(supported_prefixes) or scene_name in supported_names):
        return [], []
    warnings=[]; blockers=[]
    counts=collections.Counter(str(i.get('parent_link') or '') for i in items)
    total=sum(counts.values())
    tool0_count=counts.get('tool0', 0)
    expected_roots={'world','base','base_link','base_link_inertia'}
    root_links=set(urdf_diagnostics.get('root_links') or [])
    has_expected_root=bool(root_links & expected_roots)
    if total and tool0_count / total >= 0.5:
        msg=f"Most supported robot scene visuals terminate at tool0 ({tool0_count}/{total}); expected robot/world roots may be missing or collapsed."
        blockers.append(msg)
    if root_links and not has_expected_root:
        msg=f"Supported robot scene root links do not include an expected world/base root: {sorted(root_links)}"
        warnings.append(msg)
    elif not root_links:
        warnings.append('Supported robot scene URDF diagnostics found no root links.')
    return warnings, blockers


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--scene'); ap.add_argument('--all',action='store_true'); ap.add_argument('--prefer-xacro-expanded',action='store_true',default=True); ap.add_argument('--fallback-best-effort',action='store_true',default=True); ap.add_argument('--fail-on-unexpanded',action='store_true'); ap.add_argument('--xacro-arg',action='append',default=[]); ap.add_argument('--use-fake-hardware',default='true'); ap.add_argument('--robot-prefix',default=''); ap.add_argument('--tool-prefix',default=''); ap.add_argument('--no-write',action='store_true'); ap.add_argument('--prefer-xacro',action='store_true'); ap.add_argument('--require-xacro',action='store_true'); ap.add_argument('--workspace-root',default=os.environ.get('WORKSPACE_ROOT','')); ap.add_argument('--allow-xacro-lite-fallback',action='store_true',default=False,help='Allow explicit xacro-lite fallback after a real xacro command is available but fails')
    a=ap.parse_args()
    scenes=[SCENES_ROOT/a.scene] if a.scene else sorted([p for p in SCENES_ROOT.iterdir() if p.is_dir()])
    report={'scene_count':0,'visual_count':0,'resolved':0,'unresolved':0,'xacro_expanded_count':0,'best_effort_count':0,'real_xacro_expanded_count':0,'scenes':[]}
    for scene_dir in scenes:
        manifest=read_yaml(scene_dir/'scene_manifest.yaml') or {}
        urdf_path=scene_dir/(((manifest.get('files') or {}).get('urdf_xacro')) or 'urdf/scene.urdf.xacro')
        xargs={'use_fake_hardware':a.use_fake_hardware,'robot_prefix':a.robot_prefix,'tool_prefix':a.tool_prefix}
        for ent in a.xacro_arg:
            if '=' in ent:
                k,v=ent.split('=',1); xargs[k.strip()]=v.strip()
        mode='best_effort_recursive'; fallback_reason=''; _,xacro_avail,missing_reason=discover_xacro_command(); expanded_path=''; xacro_cmd=[]; xml_text=''; real_xacro_command_succeeded=False; xacro_diagnostics={}; source_xacro_text=urdf_path.read_text(errors='ignore') if urdf_path.exists() else ''
        if (a.prefer_xacro or a.prefer_xacro_expanded or a.require_xacro):
            try:
                expanded_result = expand_xacro(urdf_path,scene_dir,xargs,workspace_root=(a.workspace_root or None),explicit_lite_fallback=a.allow_xacro_lite_fallback)
            except TypeError:
                expanded_result = expand_xacro(urdf_path)
            if isinstance(expanded_result, tuple) and len(expanded_result) >= 4:
                xml_text,_,err,xacro_cmd=expanded_result[:4]
                if len(expanded_result) >= 5 and isinstance(expanded_result[4], dict): xacro_diagnostics=expanded_result[4]
            elif isinstance(expanded_result, tuple) and len(expanded_result) == 3:
                xml_text,mode_hint,err=expanded_result; xacro_cmd=[]
                if isinstance(err, list): err='; '.join(str(e) for e in err)
                if mode_hint and not xml_text: mode=str(mode_hint)
            else:
                xml_text,err,xacro_cmd='', 'xacro expansion failed: unexpected extractor result', []
            if xml_text:
                mode='xacro_lite_expanded' if xacro_cmd and xacro_cmd[0] == 'xacro-lite' else ('xacro_lite_fallback' if xacro_diagnostics.get('xacro_status') == 'explicit_xacro_lite_fallback' else 'xacro_expanded')
                expanded_path='' if mode in ('xacro_lite_expanded','xacro_lite_fallback') else 'generated/expanded_scene_preview.urdf'
                fallback_reason=err if mode in ('xacro_lite_expanded','xacro_lite_fallback') else ''
                real_xacro_command_succeeded=bool(mode == 'xacro_expanded' and xacro_cmd and xacro_cmd[0] != 'xacro-lite')
            else: fallback_reason=err or missing_reason
        if a.require_xacro and not xacro_avail:
            print('xacro executable unavailable (required)'); return 2
        if not xml_text: xml_text=(source_xacro_text if source_xacro_text else '<robot/>')
        referenced_packages = sorted(set(extract_referenced_package_names(xml_text)) | set(extract_referenced_package_names(source_xacro_text)))
        package_map, package_diagnostics = discover_package_map(scene_dir, workspace_root=(a.workspace_root or None), package_names=referenced_packages)
        urdf_diagnostics={'root_links': [], 'visual_parent_link_counts': {}, 'missing_parent_links': [], 'transform_chain_diagnostics': []}
        try:
            items, urdf_diagnostics = extract_from_urdf(xml_text, package_map, include_diagnostics=True)
        except Exception:
            items=[]
        static_robot_fallback_count = 0
        static_robot_mesh_count = 0
        fallback_ur_robot_unresolved = (
            _contains_unresolved_ur_robot(fallback_reason)
            or _contains_unresolved_ur_robot(source_xacro_text)
            or _contains_unresolved_ur_robot(xml_text)
        )
        if mode in {'xacro_lite_expanded', 'xacro_lite_fallback', 'best_effort_recursive', 'best_effort', 'raw_fallback', 'raw'} and fallback_ur_robot_unresolved:
            static_robot_mesh_count = append_static_ur5_mesh_visuals(items, package_map)
            static_robot_fallback_count = append_static_robot_primitive_fallbacks(items, xml_text, fallback_reason, mode, source_xacro_text)
        ur5_visual_diagnostics = _ur5_visual_mesh_diagnostics(
            package_map,
            package_diagnostics,
            sum(1 for i in items if str(i.get('package_uri') or '').startswith(UR5_VISUAL_MESH_URI_PREFIX)),
            static_robot_fallback_count,
        )
        if scene_dir.name.startswith('ur5_') or _contains_unresolved_ur_robot(source_xacro_text):
            _log_ur5_visual_mesh_diagnostics(scene_dir.name, ur5_visual_diagnostics)
        static_parent_resolved_count = resolve_static_tool0_children(items)
        unresolved=[i for i in items if any(contains_placeholder(i.get(k,'')) for k in ('id','link','parent_link'))]
        mesh_format_counts=dict(collections.Counter((Path(i.get('resolved_source_path') or i.get('source_path') or '').suffix.lower() or 'unknown') for i in items if i.get('geometry_type')=='mesh'))
        transform_status_counts=dict(collections.Counter(str(i.get('transform_status') or 'unknown') for i in items))
        renderable_count=sum(1 for i in items if i.get('render_expected', True))
        renderable_mesh_count=sum(1 for i in items if i.get('geometry_type')=='mesh' and i.get('render_expected', True))
        preview_warning=preview_degraded_fallback_warning(fallback_reason, mode)
        preview_blockers=[]
        preview_warnings=[]
        if preview_warning:
            preview_blockers.append(preview_warning)
            preview_warnings.append(preview_warning)
        if mode in ('xacro_lite_expanded','xacro_lite_fallback') and not preview_warning:
            preview_warnings.append('xacro-lite preview is degraded and is not equivalent to real xacro-expanded output')
        root_warnings, root_blockers = supported_robot_root_diagnostics(scene_dir.name, items, urdf_diagnostics)
        preview_warnings.extend(root_warnings)
        preview_blockers.extend(root_blockers)
        safe=is_preview_fully_healthy(items, unresolved, mode, fallback_reason, renderable_mesh_count)
        source_mtime=urdf_path.stat().st_mtime if urdf_path.exists() else None
        has_transform_collapse_warning=bool(items) and len({tuple((i.get('pose') or {}).get('xyz') or []) for i in items}) <= 1 and len(items) > 1
        payload={'scene_name':scene_dir.name,'visual_count':len(items),'resolved':sum(1 for i in items if i.get('resolved')),'unresolved':sum(1 for i in items if not i.get('resolved')),'generated_at':datetime.now(timezone.utc).isoformat(),'extractor_version':EXTRACTOR_VERSION,'path_reference_root':'repository','extraction_mode':mode,'xacro_available':xacro_avail,'source_urdf_xacro_path':_repo_relative_path(urdf_path),'source_mtime':source_mtime,'source_expanded_urdf_path':_repo_relative_path(expanded_path),'fallback_reason':fallback_reason,'xacro_real_command_succeeded':real_xacro_command_succeeded,'xacro_status':xacro_diagnostics.get('xacro_status', 'not_attempted' if not xacro_avail else ('real_xacro_succeeded' if real_xacro_command_succeeded else 'real_xacro_failed')),'xacro_diagnostics':_portable_source_metadata(xacro_diagnostics),'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'has_transform_collapse_warning':has_transform_collapse_warning,'candidate_mesh_count':len(items),'emitted_visual_count':len(items),'root_links':urdf_diagnostics.get('root_links', []),'visual_parent_link_counts':urdf_diagnostics.get('visual_parent_link_counts', {}),'missing_parent_links':urdf_diagnostics.get('missing_parent_links', []),'transform_chain_diagnostics':urdf_diagnostics.get('transform_chain_diagnostics', []),'initial_joint_source':urdf_diagnostics.get('initial_joint_source', ''),'transform_status_counts':transform_status_counts,'mesh_format_counts':mesh_format_counts,'renderable_mesh_count':renderable_mesh_count,'renderable_item_count':renderable_count,'static_robot_primitive_fallback_count':static_robot_fallback_count,'static_robot_mesh_visual_count':static_robot_mesh_count,'ur5_visual_mesh_diagnostics':_portable_source_metadata(ur5_visual_diagnostics),'static_parent_resolved_count':static_parent_resolved_count,'stale_index':False,'stale_reasons':[],'blockers':preview_blockers,'warnings':preview_warnings,'visual_items':items,'xacro_command':_portable_source_metadata(xacro_cmd),'package_resolution_diagnostics':_portable_source_metadata(package_diagnostics)}
        idx_path=scene_dir/'generated/scene_visual_mesh_index.json'
        if not a.no_write:
            idx_path.parent.mkdir(parents=True,exist_ok=True)
            idx_path.write_text(json.dumps(payload,indent=2)+'\n')
        report['scene_count']+=1
        report['visual_count']+=len(items)
        report['resolved']+=sum(1 for i in items if i.get('resolved'))
        report['unresolved']+=sum(1 for i in items if not i.get('resolved'))
        report['xacro_expanded_count']+=int(mode in ('xacro_expanded','xacro_lite_expanded','xacro_lite_fallback'))
        report['real_xacro_expanded_count']+=int(real_xacro_command_succeeded)
        report['best_effort_count']+=int(mode not in ('xacro_expanded','xacro_lite_expanded','xacro_lite_fallback'))
        report.setdefault('mesh_format_counts', {})
        for ext,count in mesh_format_counts.items(): report['mesh_format_counts'][ext]=report['mesh_format_counts'].get(ext,0)+count
        report['renderable_mesh_count']=report.get('renderable_mesh_count',0)+renderable_mesh_count
        report['renderable_item_count']=report.get('renderable_item_count',0)+renderable_count
        report['candidate_mesh_count']=report.get('candidate_mesh_count',0)+len(items)
        report['emitted_visual_count']=report.get('emitted_visual_count',0)+len(items)
        report['unresolved_placeholder_count']=report.get('unresolved_placeholder_count',0)+len(unresolved)
        report['scenes'].append({'scene':scene_dir.name,'extraction_mode':mode,'xacro_available':payload['xacro_available'],'xacro_real_command_succeeded':real_xacro_command_succeeded,'xacro_status':xacro_diagnostics.get('xacro_status', 'not_attempted' if not xacro_avail else ('real_xacro_succeeded' if real_xacro_command_succeeded else 'real_xacro_failed')),'xacro_diagnostics':_portable_source_metadata(xacro_diagnostics),'expanded_urdf_written':bool(expanded_path),'safe_for_preview':safe,'unresolved_placeholder_count':len(unresolved),'mesh_backed_count':sum(1 for i in items if i.get('geometry_type')=='mesh'),'skipped_count':sum(1 for i in items if i.get('render_skip_reason')),'fallback_reason':fallback_reason,'urdf_primitive_count':sum(1 for i in items if i.get('geometry_type') in ('box','cylinder','sphere','capsule')),'primitive_fallback_count':static_robot_fallback_count,'stale_index':False,'status':'PASS' if safe else 'WARN'})
        if a.require_xacro and mode not in ('xacro_expanded','xacro_lite_expanded','xacro_lite_fallback'): return 2
    (ROOT/'build').mkdir(exist_ok=True)
    (ROOT/'build/workcell_studio_urdf_visual_mesh_index_report.json').write_text(json.dumps(report,indent=2)+'\n')
    if a.fail_on_unexpanded and report['best_effort_count']>0: return 3
    return 0

if __name__=='__main__': raise SystemExit(main())
