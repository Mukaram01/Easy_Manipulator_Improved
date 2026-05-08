from __future__ import annotations
import json, shutil, subprocess, sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

DEFAULTS={
    "robot":{"role":"robot_base","pose":{"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},"collision_mode":"mesh_collision","support_status":"supported"},
    "end_effector":{"role":"end_effector","pose":{"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},"collision_mode":"mesh_collision","support_status":"supported"},
    "support_surface":{"role":"support_surface","pose":{"x":0.6,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},"collision_mode":"bounding_box","support_status":"supported"},
    "pick_object":{"role":"pick_object","pose":{"x":0.45,"y":0,"z":0.2,"roll":0,"pitch":0,"yaw":0},"collision_mode":"bounding_box","support_status":"supported"},
    "place_target":{"role":"place_target","pose":{"x":0.5,"y":-0.35,"z":0.2,"roll":0,"pitch":0,"yaw":0},"collision_mode":"bounding_box","support_status":"supported"},
    "camera":{"role":"camera","pose":{"x":0.5,"y":0,"z":0.8,"roll":0,"pitch":-0.6,"yaw":0},"collision_mode":"visual_only","support_status":"supported"},
    "conveyor":{"role":"conveyor","pose":{"x":0.7,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},"collision_mode":"bounding_box","support_status":"preview_only"},
}

def add_asset_to_cell(state:dict[str,Any], asset_id:str, category:str, role:str|None=None, name:str|None=None)->dict[str,Any]:
    assets=state.setdefault("current_cell_assets",[])
    key=role or category
    defaults=DEFAULTS.get(key, {"role": role or "visual_object", "pose": {"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0}, "collision_mode":"visual_only", "support_status":"supported"})
    entry={"asset_id":asset_id,"name":name or asset_id,"category":category,"role":defaults["role"],"pose":dict(defaults["pose"]),"collision_mode":defaults["collision_mode"],"support_status":defaults["support_status"],"source":"catalog"}
    assets.append(entry)
    selected=state.setdefault("selected",{})
    selected_map={"robot_base":"robot","end_effector":"tool","support_surface":"support_surface","pick_object":"pick_area","place_target":"place_target","camera":"camera"}
    if entry["role"] in selected_map:
        selected[selected_map[entry["role"]]]=entry["asset_id"]
    return entry

def duplicate_selected_asset(state:dict[str,Any], index:int)->dict[str,Any]:
    src=state.setdefault("current_cell_assets",[])[index]
    dup=json.loads(json.dumps(src))
    dup["name"]=f"{src['name']}_copy"
    state["current_cell_assets"].append(dup)
    return dup

def remove_selected_asset(state:dict[str,Any], index:int)->None:
    state.setdefault("current_cell_assets",[]).pop(index)

def import_custom_stl(state:dict[str,Any], filepath:str, collision_mode:str="visual_only")->dict[str,Any]:
    return add_asset_to_cell(state, asset_id=Path(filepath).stem, category="custom_stl", role="visual_object", name=Path(filepath).name) | {"source_file":filepath,"collision_mode":collision_mode}


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
    if state.get("preview_only_assets") or any(a.get("support_status")=="preview_only" for a in state.get("current_cell_assets",[])):
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
    assets=state.get("current_cell_assets",[])
    files={
        "cell_definition.yaml": "cell_definition:\n  assets: %d\n"%len(assets),
        "environment_layout.yaml": "environment_layout:\n  placed_assets: %d\n"%len(assets),
        "task_recipe.yaml": "task_recipe: {}\n",
    }
    for name,content in files.items(): (output_dir/name).write_text(content,encoding="utf-8")
    (output_dir/"selected_assets.json").write_text(json.dumps({"selected":state.get("selected",{}),"current_cell_assets":assets},indent=2),encoding="utf-8")
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
