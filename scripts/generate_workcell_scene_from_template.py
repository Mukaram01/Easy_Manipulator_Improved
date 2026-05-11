#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess
from pathlib import Path


def _yaml(d, i=0):
    sp='  '*i
    if isinstance(d, dict):
        out=[]
        for k,v in d.items():
            if isinstance(v,(dict,list)): out.append(f"{sp}{k}:") ; out.append(_yaml(v,i+1))
            else: out.append(f"{sp}{k}: {str(v).lower() if isinstance(v,bool) else v}")
        return '\n'.join(out)
    if isinstance(d, list):
        out=[]
        for v in d:
            if isinstance(v,(dict,list)): out.append(f"{sp}-") ; out.append(_yaml(v,i+1))
            else: out.append(f"{sp}- {v}")
        return '\n'.join(out)
    return f"{sp}{d}"

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--template',required=True); ap.add_argument('--scene-name',required=True); ap.add_argument('--output-dir',required=True)
    ap.add_argument('--validate',action='store_true'); ap.add_argument('--print-summary',action='store_true')
    a=ap.parse_args()
    catalog=Path('workcell_builder/workcell_builder/config/scene_templates/scene_templates.json')
    data=json.loads(catalog.read_text())
    tpl=next((t for t in data['templates'] if t['template_id']==a.template),None)
    if not tpl: raise SystemExit('unknown template')
    scene=Path(a.output_dir)/a.scene_name; (scene/'config').mkdir(parents=True,exist_ok=True)
    env={"schema_version":"workcell_scene/v1","scene":{"name":a.scene_name},"robot":{"name":tpl['recommended_robot']},"tool":{"name":tpl['recommended_tool']},"camera":{"profile":tpl.get('camera_profile')},"workspace":tpl['workspace_bounds'],"placed_objects":tpl['placed_objects'],"task":tpl['task_defaults'],"safety":tpl['safety_flags'],"metadata":{"template_id":tpl['template_id'],"template_category":tpl['category'],"required_assets":tpl['required_assets'],"optional_assets":tpl['optional_assets']}}
    (scene/'environment.yaml').write_text(_yaml(env)+'\n',encoding='utf-8')
    (scene/'config'/'task_recipe.yaml').write_text(_yaml({"schema_version":"workcell_task/v1","task":tpl['task_defaults'],"safety":tpl['safety_flags']})+'\n',encoding='utf-8')
    summary={"scene_dir":str(scene),"template_id":tpl['template_id'],"validation_command":["python3","scripts/validate_workcell_scene.py","--scene-dir",str(scene)],"fake_hardware_smoke":["python3","scripts/run_workcell_fake_hardware_smoke.py","--scene-dir",str(scene),"--skip-launch","--print-summary"]}
    (scene/'workcell_template_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    (scene/'workcell_template_summary.md').write_text(f"# {a.scene_name}\n\nTemplate: {tpl['template_id']}\n\nSafe guidance: fake-hardware only, launch is optional.\n",encoding='utf-8')
    if a.validate:
      subprocess.run(["python3","scripts/validate_workcell_scene.py","--scene-dir",str(scene)],check=False)
    if a.print_summary: print(json.dumps(summary,indent=2))

if __name__=='__main__': main()
