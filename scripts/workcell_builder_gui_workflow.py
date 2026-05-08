from __future__ import annotations
import json, shutil, subprocess, sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _run(cmd:list[str], cwd:Path|None=None)->dict[str,Any]:
    p=subprocess.run(cmd,cwd=str(cwd) if cwd else None,capture_output=True,text=True,check=False)
    return {"ok":p.returncode==0,"returncode":p.returncode,"stdout":p.stdout,"stderr":p.stderr,"command":" ".join(cmd)}


def _error_payload(run:dict[str,Any], output_dir:Path|None=None)->dict[str,Any]:
    return {
        "ok": False,
        "error": {
            "command": run.get("command",""),
            "returncode": run.get("returncode",-1),
            "stderr": (run.get("stderr","") or "").strip()[:500],
            "output_dir": str(output_dir) if output_dir else "",
        },
    }


def validate_manual_cell_state(state:dict[str,Any])->dict[str,Any]:
    issues=[]
    selected=state.get("selected",{})
    for key,code,msg in [
        ("robot","missing_robot","Robot must be selected"),
        ("tool","missing_tool","Gripper/tool must be selected"),
        ("support_surface","missing_support_surface","Support surface/table must be selected"),
        ("pick_area","missing_pick_area","Pick area must be selected"),
        ("place_target","missing_place_target","Place target must be selected"),
        ("grasp_strategy","missing_grasp_strategy","Grasp strategy must be selected"),
    ]:
        if not selected.get(key): issues.append({"severity":"FAIL","code":code,"message":msg})
    roi=state.get("camera_pointcloud_roi") or {}
    if roi and roi.get("enabled",False):
        if roi.get("x_min",0)>=roi.get("x_max",0) or roi.get("y_min",0)>=roi.get("y_max",0) or roi.get("z_min",0)>=roi.get("z_max",0):
            issues.append({"severity":"FAIL","code":"invalid_pointcloud_roi","message":"Pointcloud ROI bounds are invalid"})
    if selected.get("robot") and selected.get("tool"):
        compat=state.get("compatibility",{}).get("robot_tool",True)
        if not compat: issues.append({"severity":"FAIL","code":"robot_tool_incompatible","message":"Selected robot/tool are incompatible"})
    if state.get("preview_only_assets"):
        issues.append({"severity":"WARN","code":"preview_only_assets","message":"Selection includes preview-only assets"})
    if state.get("custom_stl_collision_mode") == "visual_only":
        issues.append({"severity":"WARN","code":"custom_stl_visual_only","message":"Custom STL collision mode is visual-only"})
    if state.get("fake_hardware_default") is False:
        issues.append({"severity":"WARN","code":"fake_hw_not_default","message":"Fake hardware should remain default"})
    has_fail=any(i["severity"]=="FAIL" for i in issues)
    status="FAIL" if has_fail else ("WARN" if issues else "OK")
    return {"status":status,"issues":issues,"fake_hardware_default": state.get("fake_hardware_default",True)}


def generate_canonical_files(state:dict[str,Any], output_dir:Path)->dict[str,Any]:
    output_dir.mkdir(parents=True,exist_ok=True)
    val=validate_manual_cell_state(state)
    files={
        "cell_definition.yaml": "cell_definition: {}\n",
        "environment_layout.yaml": "environment_layout: {}\n",
        "task_recipe.yaml": "task_recipe: {}\n",
    }
    for name,content in files.items(): (output_dir/name).write_text(content,encoding="utf-8")
    (output_dir/"selected_assets.json").write_text(json.dumps(state.get("selected",{}),indent=2),encoding="utf-8")
    (output_dir/"compatibility_report.json").write_text(json.dumps({"fake_hardware_default":True,"preview_only":bool(state.get("preview_only_assets"))},indent=2),encoding="utf-8")
    (output_dir/"builder_export_summary.json").write_text(json.dumps({"validation_status":val["status"],"generated_files":sorted([p.name for p in output_dir.iterdir()])},indent=2),encoding="utf-8")
    return {"ok":True,"validation":val,"output_dir":str(output_dir),"generated_files":sorted([p.name for p in output_dir.iterdir()])}


def generate_studio_pack(state:dict[str,Any], output_dir:Path)->dict[str,Any]:
    result=generate_canonical_files(state,output_dir)
    (output_dir/"readiness_summary.md").write_text("# Readiness\n- Final readiness: **%s**\n- Safety: offline/fake hardware first\n"%result["validation"]["status"],encoding="utf-8")
    (output_dir/"readiness_summary.html").write_text("<html><body><h1>Readiness</h1></body></html>",encoding="utf-8")
    (output_dir/"environment_preview.svg").write_text("<svg xmlns='http://www.w3.org/2000/svg'></svg>",encoding="utf-8")
    (output_dir/"environment_preview.html").write_text("<html><body>Preview</body></html>",encoding="utf-8")
    launch=copy_fake_hardware_launch_command(state)
    (output_dir/"generated_launch_commands.md").write_text(launch.get("message",""),encoding="utf-8")
    return {"ok":True,"output_dir":str(output_dir),"readiness":result["validation"]["status"]}


def copy_fake_hardware_launch_command(state:dict[str,Any])->dict[str,Any]:
    if state.get("preview_only_assets"):
        return {"ok":False,"message":"No runtime launch command available for preview-only cell"}
    return {"ok":True,"message":"ros2 launch <scene_package> demo.launch.py use_fake_hardware:=true"}
