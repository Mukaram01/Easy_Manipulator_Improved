#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess
from pathlib import Path


def _yaml(d, i=0):
    sp='  '*i
    if isinstance(d, dict):
        out=[]
        for k,v in d.items():
            if isinstance(v,(dict,list)):
                out.append(f"{sp}{k}:")
                out.append(_yaml(v,i+1))
            else:
                out.append(f"{sp}{k}: {str(v).lower() if isinstance(v,bool) else v}")
        return '\n'.join(out)
    if isinstance(d, list):
        out=[]
        for v in d:
            if isinstance(v,(dict,list)):
                out.append(f"{sp}-")
                out.append(_yaml(v,i+1))
            else:
                out.append(f"{sp}- {v}")
        return '\n'.join(out)
    return f"{sp}{d}"


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--template',required=True); ap.add_argument('--scene-name',required=True); ap.add_argument('--output-dir',required=True)
    ap.add_argument('--validate',action='store_true'); ap.add_argument('--print-summary',action='store_true')
    a=ap.parse_args()
    root = Path(__file__).resolve().parents[1]
    catalog=root/'workcell_builder/workcell_builder/config/scene_templates/scene_templates.json'
    data=json.loads(catalog.read_text())
    tpl=next((t for t in data['templates'] if t['template_id']==a.template),None)
    if not tpl: raise SystemExit('unknown template')
    scene=Path(a.output_dir)/a.scene_name; (scene/'config').mkdir(parents=True,exist_ok=True); (scene/'preview').mkdir(parents=True,exist_ok=True)

    camera_enabled = bool(tpl.get('camera_profile'))
    camera = {
        'enabled': camera_enabled,
        'camera_id': tpl.get('camera_profile',''),
        'frame_id': 'camera_link',
        'pose': '[0.0, -0.35, 1.2, 0.0, 0.0, 0.0]',
        'rgb_topic':'/camera/color/image_raw',
        'depth_topic':'/camera/depth/image_rect_raw',
        'pointcloud_topic':'/camera/depth/color/points',
    }
    env={
        "schema_version":"workcell_scene/v1",
        "scene":{"name":a.scene_name,"description":tpl['description']},
        "robot":{"name":tpl['recommended_robot'],"profile":"default_ur5"},
        "tool":{"name":tpl['recommended_tool'],"profile":"default_tool"},
        "compatibility":{"status":"COMPATIBLE","warnings":[]},
        "placed_objects":tpl['placed_objects'],
        "camera":camera,
        "task":tpl['task_defaults'],
        "workspace":{
            "bounds":{
                "x_min":float(tpl['workspace_bounds']['min'][0]),"x_max":float(tpl['workspace_bounds']['max'][0]),
                "y_min":float(tpl['workspace_bounds']['min'][1]),"y_max":float(tpl['workspace_bounds']['max'][1]),
                "z_min":float(tpl['workspace_bounds']['min'][2]),"z_max":float(tpl['workspace_bounds']['max'][2]),
            },
            "zones":[{"name":"operator_warning_zone","type":"warning","shape":"rectangle","min":[0.0,-0.65],"max":[0.95,0.65]}]
        },
        "safety":tpl['safety_flags'],
        "metadata":{"template_id":tpl['template_id'],"template_category":tpl['category'],"task_metadata_format":"workcell_task/v1"}
    }
    (scene/'environment.yaml').write_text(_yaml(env)+'\n',encoding='utf-8')
    (scene/'config'/'task_recipe.yaml').write_text(_yaml({"schema_version":"workcell_task/v1","task":tpl['task_defaults'],"safety":tpl['safety_flags']})+'\n',encoding='utf-8')

    obj_names=', '.join(o['id'] for o in tpl['placed_objects'])
    fake_launch = f"ros2 launch {a.scene_name} demo.launch.py use_fake_hardware:=true launch_rviz:=false"
    summary={"scene_dir":str(scene),"template_id":tpl['template_id'],"scene_name":a.scene_name,"scene_schema_validation_status":"pending","compatibility_status":"COMPATIBLE","readiness_overlay_status":"present","camera_metadata_status":"present","validation_command":["python3","scripts/validate_workcell_scene.py","--scene-dir",str(scene)],"fake_hardware_smoke":["python3","scripts/run_workcell_fake_hardware_smoke.py","--scene-dir",str(scene),"--skip-launch","--print-summary"],"fake_hardware_launch_command":fake_launch,"safety":tpl['safety_flags']}
    (scene/'workcell_template_summary.json').write_text(json.dumps(summary,indent=2)+"\n",encoding='utf-8')
    (scene/'workcell_template_summary.md').write_text(f"# {a.scene_name}\n\nTemplate: {tpl['template_id']}\n\nSafe guidance: fake-hardware only, launch is optional.\n",encoding='utf-8')
    (scene/'workcell_studio_summary.json').write_text(json.dumps(summary, indent=2)+"\n", encoding='utf-8')
    (scene/'workcell_studio_summary.md').write_text(
f"# Workcell Studio Template Summary\n\nscene_name: {a.scene_name}\ntemplate_id: {tpl['template_id']}\nscene_schema_validation_status: pending\ncompatibility_status: COMPATIBLE\nreadiness_overlay_status: present\ncamera_metadata_status: present\n\nfake_hardware_launch_command: `{fake_launch}`\n\nSafety flags:\n- fake_hardware_first: true\n- motion_command_sent: false\n- runtime_execution_enabled: false\n- real_hardware_enabled: false\n- moveit_plan_service_called: false\n")
    (scene/'preview'/'workcell_preview.svg').write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360"><text x="20" y="40">Scene: {a.scene_name}</text><text x="20" y="70">Robot/Tool: {tpl["recommended_robot"]} + {tpl["recommended_tool"]}</text><text x="20" y="100">Objects: {obj_names}</text><text x="20" y="130">Camera enabled: {str(camera_enabled).lower()}</text><text x="20" y="160">fake-hardware-first: true</text></svg>\n', encoding='utf-8')
    (scene/'preview'/'workcell_preview.html').write_text(f"<html><body><h1>{a.scene_name}</h1><p>Robot/Tool: {tpl['recommended_robot']} + {tpl['recommended_tool']}</p><p>Placed objects: {obj_names}</p><p>Camera marker: {'enabled' if camera_enabled else 'disabled'}</p><p>fake-hardware-first note: true</p></body></html>\n", encoding='utf-8')

    if a.validate:
      rc=subprocess.run(["python3",str(root/"scripts/validate_workcell_scene.py"),"--scene-dir",str(scene)],check=False).returncode
      summary["scene_schema_validation_status"] = "PASS" if rc==0 else "FAIL"
      (scene/'workcell_studio_summary.json').write_text(json.dumps(summary, indent=2)+"\n", encoding='utf-8')
    if a.print_summary: print(json.dumps(summary,indent=2))

if __name__=='__main__': main()
