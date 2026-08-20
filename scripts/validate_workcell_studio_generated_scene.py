#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, re
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:
    yaml = None

STATUS_PASS="PASS"; STATUS_WARN="WARNINGS"; STATUS_BLOCKED="BLOCKED"; STATUS_PREVIEW="PREVIEW_ONLY"
ROS_NAME_RE=re.compile(r"^[a-z][a-z0-9_]*$")


def _load_yaml(path: Path)->dict[str, Any]:
    if not path.is_file(): return {}
    text=path.read_text(encoding='utf-8')
    if yaml is not None:
        out=yaml.safe_load(text)
        return out if isinstance(out, dict) else {}
    return {}

def _has(scene:Path, rel:str)->bool: return (scene/rel).is_file()

def validate(scene:Path)->dict[str,Any]:
    checks=[]; blockers=[]; warnings=[]
    req=["package.xml","CMakeLists.txt","environment.yaml","scene_manifest.yaml","config/task_recipe.yaml","config/workcell_builder_task_intent.yaml"]
    for r in req:
        ok=_has(scene,r); checks.append({"name":f"{r} exists","ok":ok});
        if not ok: blockers.append(f"Missing required file: {r}")

    preview_ok=_has(scene,"preview/static_preview.html") or _has(scene,"preview/static_preview.svg")
    checks.append({"name":"preview/static_preview.html or static_preview.svg exists","ok":preview_ok})
    if not preview_ok: warnings.append("Preview artifact missing")

    smoke_ok=_has(scene,"smoke/offline_smoke_summary.txt")
    checks.append({"name":"smoke/offline_smoke_summary.txt exists","ok":smoke_ok})
    if not smoke_ok: warnings.append("Offline smoke summary missing")

    summary_ok=_has(scene,"workcell_studio_summary.json") or _has(scene,"workcell_template_summary.json")
    checks.append({"name":"generated summary exists","ok":summary_ok})
    if not summary_ok: warnings.append("Generated summary missing")

    name_ok=bool(ROS_NAME_RE.match(scene.name))
    checks.append({"name":"scene name is valid ROS package name","ok":name_ok})
    if not name_ok: blockers.append("Scene directory name is not a valid ROS package name")

    env=_load_yaml(scene/"environment.yaml")
    intent=_load_yaml(scene/"config/workcell_builder_task_intent.yaml")
    recipe=_load_yaml(scene/"config/task_recipe.yaml")
    safety=((intent.get("safety") if isinstance(intent.get("safety"),dict) else {}) or (recipe.get("safety") if isinstance(recipe.get("safety"),dict) else {}))
    for key, expected in [("fake_hardware_first",True),("runtime_execution_enabled",False),("motion_command_sent",False)]:
        ok=safety.get(key)==expected
        checks.append({"name":f"{key} == {expected}","ok":ok})
        if not ok: blockers.append(f"Safety flag {key} expected {expected} got {safety.get(key)!r}")

    launch=scene/"launch/demo.launch.py"
    launch_ready=launch.is_file()
    checks.append({"name":"launch/demo.launch.py exists for launch-ready scene","ok":launch_ready,"optional":True})
    launch_text=launch.read_text(encoding='utf-8') if launch_ready else ""
    if launch_ready:
        fake_arg_ok=("use_fake_hardware" in launch_text)
        checks.append({"name":"launch file contains use_fake_hardware argument","ok":fake_arg_ok})
        if not fake_arg_ok: blockers.append("Launch file missing use_fake_hardware argument")
        no_real_default=("use_fake_hardware:=false" not in launch_text and "real_hardware:=true" not in launch_text)
        checks.append({"name":"launch file does not default to real hardware","ok":no_real_default})
        if not no_real_default: blockers.append("Launch file appears to default to real hardware")
        no_runtime_default=("runtime_execution_enabled:=true" not in launch_text)
        checks.append({"name":"launch file does not enable runtime execution by default","ok":no_runtime_default})
        if not no_runtime_default: blockers.append("Launch file enables runtime execution by default")
    else:
        warnings.append("Launch file missing; preview-only or scaffold scene")

    urdf_scene=(scene/"urdf/scene.urdf.xacro").is_file() or (scene/"urdf/environment.urdf.xacro").is_file()
    srdf_ok=(scene/"urdf/arm_hand.srdf.xacro").is_file()
    checks.append({"name":"scene.urdf.xacro or environment.urdf.xacro exists","ok":urdf_scene,"optional":True})
    checks.append({"name":"arm_hand.srdf.xacro exists","ok":srdf_ok,"optional":True})

    xacro_text=""
    for p in [scene/"urdf/scene.urdf.xacro", scene/"urdf/environment.urdf.xacro", scene/"environment.yaml"]:
        if p.is_file(): xacro_text += p.read_text(encoding='utf-8')+"\n"
    if "robotiq" in xacro_text.lower() or "2f" in xacro_text.lower() or "gripper" in xacro_text.lower():
        rpy_ok="-1.5708 -1.5708 0" in xacro_text
        checks.append({"name":"generated gripper mount RPY is -1.5708 -1.5708 0","ok":rpy_ok})
        if not rpy_ok: warnings.append("Expected gripper mount RPY not found")


    merge_report = {}
    acceptance_layout_stale=False
    if (scene/"generated/workcell_studio_layout_merge_report.json").is_file():
        merge_report = json.loads((scene/"generated/workcell_studio_layout_merge_report.json").read_text(encoding="utf-8"))
    merge_exists=_has(scene,"generated/workcell_studio_layout_merge_report.json")
    checks.append({"name":"generated/workcell_studio_layout_merge_report.json exists","ok":merge_exists,"optional":True})
    if not merge_exists: warnings.append("Layout merge report missing; run Generate Scene to apply layout")
    layout=_load_yaml(scene/"layout/workcell_studio_layout.yaml")
    if layout.get("saved_at_utc") and merge_exists:
        mt=(scene/"generated/workcell_studio_layout_merge_report.json").stat().st_mtime
        lt=(scene/"layout/workcell_studio_layout.yaml").stat().st_mtime
        stale=lt>mt
        checks.append({"name":"layout merged after last save","ok":not stale,"optional":True})
        if stale: warnings.append("Generated files stale: saved layout newer than merge artifacts"); acceptance_layout_stale=True

    cmd=f"ros2 launch {scene.name} demo.launch.py use_fake_hardware:=true"
    checks.append({"name":"launch command contains use_fake_hardware:=true","ok":True})

    # mirror browser high-level statuses
    has_env=_has(scene,"environment.yaml"); has_launch=launch_ready
    if not has_env: status="MISSING_ENVIRONMENT_YAML"
    elif not has_launch and preview_ok: status=STATUS_PREVIEW
    elif blockers: status=STATUS_BLOCKED
    elif warnings: status=STATUS_WARN
    else: status=STATUS_PASS

    acceptance={
        "scene_name":scene.name,"scene_path":str(scene),"status":status,"checks":checks,
        "blockers":blockers,"warnings":warnings,
        "safety_flags":{"fake_hardware_first":safety.get("fake_hardware_first"),"runtime_execution_enabled":safety.get("runtime_execution_enabled"),"motion_command_sent":safety.get("motion_command_sent")},
        "next_commands":[f"colcon build --symlink-install --packages-select {scene.name}","source install/setup.bash",cmd],
        "notes":["Command not executed by validator","No robot motion commanded"],
        "layout_applied": merge_report.get("layout_applied", False),
        "generated_from_saved_layout": merge_report.get("generated_from_saved_layout", False),
        "merge_report_path": str(scene/"generated/workcell_studio_layout_merge_report.json"),
        "layout_stale": acceptance_layout_stale,
        "merge_warnings": merge_report.get("warnings", []),
        "merge_blockers": merge_report.get("blockers", []),
    }
    out_dir=scene/"acceptance"; out_dir.mkdir(exist_ok=True)
    (out_dir/"generated_scene_acceptance.json").write_text(json.dumps(acceptance,indent=2)+"\n",encoding='utf-8')
    next_commands_html = "\n".join(acceptance["next_commands"])
    html = (
        f"<html><body><h1>Generated Scene Acceptance</h1>"
        f"<p>scene={scene.name}</p><p>status={status}</p>"
        f"<p>no robot motion commanded</p><h2>blockers</h2>"
        f"<pre>{json.dumps(blockers,indent=2)}</pre>"
        f"<h2>warnings</h2><pre>{json.dumps(warnings,indent=2)}</pre>"
        f"<h2>next commands</h2><pre>{next_commands_html}</pre></body></html>"
    )
    (out_dir/"generated_scene_acceptance.html").write_text(html,encoding='utf-8')
    summary='\n'.join([f"scene={scene.name}",f"status={status}","no_robot_motion_commanded=true",f"launch_command={cmd}"])
    (out_dir/"generated_scene_acceptance_summary.txt").write_text(summary+"\n",encoding='utf-8')
    return acceptance


def main()->int:
    ap=argparse.ArgumentParser(); ap.add_argument('scene_dir',type=Path); ap.add_argument('--json',action='store_true')
    a=ap.parse_args(); report=validate(a.scene_dir)
    if a.json: print(json.dumps(report,indent=2))
    else: print(f"{report['status']}: {report['scene_name']}")
    return 0 if report['status'] in {STATUS_PASS, STATUS_WARN, STATUS_PREVIEW} else 1

if __name__=='__main__': raise SystemExit(main())
