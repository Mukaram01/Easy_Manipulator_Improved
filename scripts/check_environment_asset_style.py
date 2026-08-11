#!/usr/bin/env python3
import argparse, json, re
from pathlib import Path
import xml.etree.ElementTree as ET

REQ_DIRS=("launch","meshes","rviz","urdf")
INSTALL_RE=re.compile(r"install\s*\(\s*DIRECTORY\s+launch\s+meshes\s+rviz\s+urdf",re.I|re.S)
AMENT_FIND_RE=re.compile(r"find_package\s*\(\s*ament_cmake\s+REQUIRED\s*\)",re.I)
AMENT_PACKAGE_RE=re.compile(r"ament_package\s*\(\s*\)",re.I)
BUILD_TYPE_ERROR=(
    "package.xml must export build_type=ament_cmake; otherwise colcon may "
    "classify the environment asset as ros.catkin and ROS 2 package:// "
    "resources will not be discoverable."
)

def classify(p:Path):
    if p.name.endswith('_description') and (p/'package.xml').exists():
        if 'camera' in p.name or 'realsense' in p.name: return 'camera/sensor description package'
        if any(k in p.name for k in ['mount','stand','frame','gantry','bracket']): return 'support/mount package'
        return 'ROS 2 description package'
    if (p/'package.xml').exists(): return 'generated asset package'
    return 'legacy/non-package asset'

def has_valid_object_yaml(pkg:Path):
    ys=list(pkg.glob('*.yaml'))
    for y in ys:
        t=y.read_text(errors='ignore')
        if all(k in t for k in ['link','joint','visual','collision']):
            return True,y.name
    return False, ys[0].name if ys else None

def check_ament_package(package_xml:Path, cmake_text:str):
    errors=[]
    try:
        root=ET.parse(package_xml).getroot()
    except Exception as e:
        return [f'package.xml parse error: {e}']

    name=(root.findtext('name') or '').strip()
    if not name:
        errors.append('package.xml missing package name')
    elif name!=package_xml.parent.name:
        errors.append(f'package.xml name mismatch ({name})')

    buildtools=[(node.text or '').strip() for node in root.findall('buildtool_depend')]
    if 'ament_cmake' not in buildtools:
        errors.append('package.xml missing buildtool_depend on ament_cmake')

    build_type=root.find('export/build_type')
    if build_type is None or (build_type.text or '').strip()!='ament_cmake':
        errors.append(BUILD_TYPE_ERROR)

    if not AMENT_FIND_RE.search(cmake_text):
        errors.append('CMakeLists.txt missing find_package(ament_cmake REQUIRED)')
    if not AMENT_PACKAGE_RE.search(cmake_text):
        errors.append('CMakeLists.txt missing ament_package()')
    return errors

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--root',default='assets/environment')
    ap.add_argument('--strict',action='store_true')
    ap.add_argument('--json-out')
    args=ap.parse_args()
    root=Path(args.root)
    report=[]; fails=0
    for p in sorted([x for x in root.iterdir() if x.is_dir()]):
        row={'name':p.name,'path':str(p),'classification':classify(p),'warnings':[],'errors':[]}
        if (p/'package.xml').exists() or (p/'CMakeLists.txt').exists():
            for d in REQ_DIRS:
                if not (p/d).exists(): row['errors'].append(f'missing {d}')
            cm=None
            if not (p/'CMakeLists.txt').exists(): row['errors'].append('missing CMakeLists.txt')
            else:
                cm=(p/'CMakeLists.txt').read_text(errors='ignore')
                if not INSTALL_RE.search(cm): row['errors'].append('CMakeLists missing install(DIRECTORY launch meshes rviz urdf ...) pattern')
            if (p/'package.xml').exists():
                if cm is not None:
                    row['errors'].extend(check_ament_package(p/'package.xml',cm))
            else: row['errors'].append('missing package.xml')
            valid_yaml,y=has_valid_object_yaml(p)
            if list(p.glob('*.yaml')) and not valid_yaml:
                row['warnings'].append('yaml wrapper present but failed simple LoadObjects field check')
            if not list(p.glob('*.yaml')) and p.name not in ['realsense2_description']:
                row['warnings'].append('no top-level object yaml wrapper')
        else:
            row['warnings'].append('legacy/non-package folder kept as-is')
        fails += len(row['errors'])
        report.append(row)
    for r in report:
        status='FAIL' if r['errors'] else ('WARN' if r['warnings'] else 'PASS')
        print(f"[{status}] {r['name']} :: {r['classification']}")
        for e in r['errors']: print('  - ERROR:',e)
        for w in r['warnings']: print('  - WARN :',w)
    if args.json_out: Path(args.json_out).write_text(json.dumps({'root':str(root),'results':report},indent=2))
    return 1 if (args.strict and fails) else 0

if __name__=='__main__': raise SystemExit(main())
