#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, shutil, subprocess, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPORTER = ROOT / 'scripts' / 'export_builder_scene_to_cell_definition.py'
GENERATOR = ROOT / 'scripts' / 'generate_workcell_from_cell_definition.py'
PARITY = ROOT / 'scripts' / 'validate_scene_builder_canvas_generated_parity.py'

REQ_FILES = [
    'scene_manifest.yaml','launch/demo.launch.py','generated/generated_workcell_summary.json',
    'generated/generated_environment_objects.yaml','urdf/generated_asset_metadata.yaml','layout/workcell_studio_layout.yaml'
]

def _run(cmd:list[str])->tuple[int,dict,str]:
    p=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,check=False)
    data={}
    if p.stdout.strip():
        try:data=json.loads(p.stdout)
        except Exception:pass
    return p.returncode,data,p.stderr.strip() or p.stdout.strip()

def _scene_dirs(root:Path)->list[Path]:
    if not root.exists(): return []
    return sorted([p for p in root.iterdir() if p.is_dir()])

def evaluate_scene(scene:Path,strict:bool,in_place:bool)->dict:
    warnings=[]; blockers=[]
    work_scene=scene if in_place else Path(tempfile.mkdtemp(prefix='scene_repro_'))/scene.name
    if not in_place: shutil.copytree(scene, work_scene)
    layout=(work_scene/'layout/workcell_studio_layout.yaml').is_file()
    legacy=(work_scene/'environment_layout.yaml').is_file()
    preview_fallback_count=1 if legacy and not layout else 0
    starter=False
    if not layout and legacy:
        (work_scene/'layout').mkdir(parents=True,exist_ok=True)
        shutil.copy2(work_scene/'environment_layout.yaml', work_scene/'layout/workcell_studio_layout.yaml')
        starter=True
        warnings.append('preview-only; starter editable layout can be created')

    export_code, export_report, export_msg = _run(['python3', str(EXPORTER), str(work_scene), '--output-dir', str(work_scene/'generated'), '--json'])
    if export_code != 0:
        blockers.append(f'export failed: {export_msg[:200]}')
    cell_def = work_scene/'generated/cell_definition.yaml'
    if not cell_def.is_file(): blockers.append('missing generated/cell_definition.yaml')

    gen_code, _, gen_msg = _run(['python3', str(GENERATOR), str(cell_def), '--output-dir', str(work_scene/'generated_package'), '--package-name', scene.name, '--force']) if cell_def.is_file() else (1,{},'skipped generation')
    if gen_code != 0: blockers.append(f'generation failed: {gen_msg[:200]}')

    parity_status='SKIPPED'
    if gen_code == 0:
        _, parity_report, _ = _run(['python3', str(PARITY), str(work_scene/'generated_package'/scene.name), '--json'])
        parity_status = parity_report.get('status','SKIPPED') if isinstance(parity_report,dict) else 'SKIPPED'

    generated_dir = work_scene/'generated_package'/scene.name
    gen_files={f:(generated_dir/f).is_file() for f in REQ_FILES}
    if strict and not all(gen_files.values()): blockers.append('missing required generated files')

    fake_cmd=f'ros2 launch {scene.name} demo.launch.py use_fake_hardware:=true'
    rec='Create editable layout from preview' if preview_fallback_count else ('Generate Scene Package' if not gen_files.get('scene_manifest.yaml',False) else 'Validate / Plan & Simulate')
    status='BLOCKED' if blockers else ('WARN' if warnings else 'PASS')
    return {
      'scene_name':scene.name,'source_scene_dir':str(scene),'working_scene_dir':str(work_scene),
      'editable_layout_count':1 if (work_scene/'layout/workcell_studio_layout.yaml').is_file() else 0,
      'preview_fallback_count':preview_fallback_count,'starter_layout_created':starter,
      'generated_package_dir':str(generated_dir),'generated_files_present':gen_files,'parity_status':parity_status,
      'blocker_count':len(blockers),'warning_count':len(warnings),'fake_hardware_launch_command':fake_cmd,
      'recommended_next_action':rec,'status':status,'warnings':warnings,'blockers':blockers,
    }

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--scenes-root', default=str(ROOT/'tests/fixtures/all_scene_builder_reproducibility'))
    ap.add_argument('--output')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--max-scenes', type=int)
    ap.add_argument('--strict', action='store_true')
    ap.add_argument('--in-place', action='store_true')
    a=ap.parse_args()
    scenes=_scene_dirs(Path(a.scenes_root))
    if a.max_scenes: scenes=scenes[:a.max_scenes]
    per=[evaluate_scene(s,a.strict,a.in_place) for s in scenes]
    report={'total_scenes':len(per),'passed':sum(p['status']=='PASS' for p in per),'warnings':sum(p['status']=='WARN' for p in per),'blocked':sum(p['status']=='BLOCKED' for p in per),'skipped':sum(p['status']=='SKIPPED' for p in per),'per_scene':per}
    text=json.dumps(report,indent=2,sort_keys=True)
    if a.output:
        Path(a.output).parent.mkdir(parents=True,exist_ok=True); Path(a.output).write_text(text,encoding='utf-8')
    if a.json or not per: print(text)
    return 1 if a.strict and report['blocked'] else 0

if __name__=='__main__':
    raise SystemExit(main())
