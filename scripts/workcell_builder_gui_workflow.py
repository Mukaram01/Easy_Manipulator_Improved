from __future__ import annotations
import json, shutil, subprocess, sys, traceback
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
    robot_name=str(selected.get("robot") or "").lower()
    tool_name=str(selected.get("tool") or "").lower()
    if "generic" in robot_name or "placeholder" in robot_name:
        issues.append({"severity":"WARN","code":"placeholder_robot","message":"Selected robot is placeholder family"})
    if ("generic" in robot_name or "placeholder" in robot_name) and ("suction" in tool_name or "vacuum" in tool_name):
        issues.append({"severity":"WARN","code":"preview_combo","message":"Robot/tool combination is preview-only"})
    if state.get("fake_hardware_default") is not True:
        issues.append({"severity":"WARN","code":"fake_hw_required","message":"use_fake_hardware must remain true by default"})
    for a in state.get("current_cell_assets",[]):
        if a.get("category")=="custom_stl" and not a.get("source_file"):
            issues.append({"severity":"WARN","code":"missing_custom_stl_path","message":"Custom STL source path is missing"})
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
        "cell_definition.yaml": _render_cell_definition_v1(state),
        "environment_layout.yaml": _render_environment_layout_v1(state),
        "task_recipe.yaml": "task_recipe: {}\n",
    }
    for name,content in files.items(): (output_dir/name).write_text(content,encoding="utf-8")
    selected_payload={"selected":state.get("selected",{}),"current_cell_assets":assets,
        "catalog_selection":{
            "robot":state.get("selected",{}).get("robot"),
            "end_effector":state.get("selected",{}).get("tool"),
            "sensor":state.get("selected",{}).get("camera"),
            "task":state.get("selected",{}).get("task"),
        }}
    (output_dir/"selected_assets.json").write_text(json.dumps(selected_payload,indent=2),encoding="utf-8")
    (output_dir/"compatibility_report.json").write_text(json.dumps({"fake_hardware_default":True,"preview_only":bool(state.get("preview_only_assets"))},indent=2),encoding="utf-8")
    scene_manifest={"schema_version":"scene_manifest/v1","generated_by":"workcell_builder","selected_assets":selected_payload["selected"],"fake_hardware_default":state.get("fake_hardware_default",True)}
    (output_dir/"scene_manifest.yaml").write_text("\n".join(["schema_version: scene_manifest/v1", "generated_by: workcell_builder"])+"\n",encoding="utf-8")
    (output_dir/"builder_export_summary.json").write_text(json.dumps({"validation_status":val["status"],"generated_files":sorted([p.name for p in output_dir.iterdir()]),"scene_manifest":scene_manifest},indent=2),encoding="utf-8")
    return {"ok":True,"validation":val,"output_dir":str(output_dir),"generated_files":sorted([p.name for p in output_dir.iterdir()])}


def generate_studio_pack(state:dict[str,Any], output_dir:Path)->dict[str,Any]:
    try:
        result=generate_canonical_files(state,output_dir)
    except Exception as exc:
        return {"ok":False,"error":{"code":"generation_failure","message":str(exc)}}
    (output_dir/"readiness_summary.md").write_text("# Readiness\n- Final readiness: **%s**\n- Safety: offline/fake hardware first\n"%result["validation"]["status"],encoding="utf-8")
    (output_dir/"readiness_summary.html").write_text("<html><body><h1>Readiness</h1></body></html>",encoding="utf-8")
    (output_dir/"environment_preview.svg").write_text("<svg xmlns='http://www.w3.org/2000/svg'></svg>",encoding="utf-8")
    (output_dir/"environment_preview.html").write_text("<html><body>Preview</body></html>",encoding="utf-8")
    launch=copy_fake_hardware_launch_command(state)
    (output_dir/"generated_launch_commands.md").write_text(launch.get("message",""),encoding="utf-8")
    return {"ok":True,"output_dir":str(output_dir),"readiness":result["validation"]["status"],"runtime_classification":_runtime_classification(state,result["validation"]) }


def _runtime_classification(state:dict[str,Any], validation:dict[str,Any])->str:
    if validation.get("status") == "FAIL":
        return "blocked_by_validation_errors"
    if state.get("preview_only_assets"):
        return "preview_only"
    if state.get("fake_hardware_default") is not True:
        return "requires_real_hardware_review"
    return "runtime_ready"


def _render_cell_definition_v1(state:dict[str,Any])->str:
    selected=state.get("selected",{})
    payload={"schema_version":"cell_definition/v1","cell_name":state.get("scene_name","builder_scene"),"robot":{"id":selected.get("robot")},"tool":{"id":selected.get("tool")},"sensor":{"id":selected.get("camera")},"task":{"id":selected.get("task")},"safety":{"use_fake_hardware":state.get("fake_hardware_default",True)},"assets":[{"id":a.get("asset_id"),"role":a.get("role"),"category":a.get("category"),"pose":a.get("pose",{})} for a in state.get("current_cell_assets",[])]}
    return json.dumps(payload,indent=2)+"\n"


def _render_environment_layout_v1(state:dict[str,Any])->str:
    assets=[]
    for a in state.get("current_cell_assets",[]):
        assets.append({"id":a.get("asset_id"),"type":a.get("category"),"xyz":[a.get("pose",{}).get("x",0),a.get("pose",{}).get("y",0),a.get("pose",{}).get("z",0)],"rpy":[a.get("pose",{}).get("roll",0),a.get("pose",{}).get("pitch",0),a.get("pose",{}).get("yaw",0)],"scale":a.get("scale",[1.0,1.0,1.0]),"source_file":a.get("source_file")})
    payload={"schema_version":"environment_layout/v1","layout_id":state.get("scene_name","builder_layout"),"assets":assets}
    return json.dumps(payload,indent=2)+"\n"


def copy_fake_hardware_launch_command(state:dict[str,Any])->dict[str,Any]:
    if state.get("preview_only_assets"):
        return {"ok":False,"message":"No runtime launch command available for preview-only cell"}
    return {"ok":True,"message":"ros2 launch <scene_package> demo.launch.py use_fake_hardware:=true"}


def _marker_type(asset:dict[str,Any])->str:
    role=(asset.get("role") or "").lower()
    category=(asset.get("category") or "").lower()
    if "robot" in role or category == "robot":
        return "robot_base"
    if "end_effector" in role or "gripper" in category:
        return "tool"
    if "support_surface" in role or category in {"table","workbench"}:
        return "table"
    if "bin" in role or category == "bin":
        return "bin"
    if "camera" in role or "camera" in category:
        return "camera"
    if "conveyor" in role or category == "conveyor":
        return "conveyor"
    return "object"


def build_visual_layout_canvas_model(state:dict[str,Any])->dict[str,Any]:
    markers=[]
    warnings=[]
    for asset in state.get("current_cell_assets",[]):
        pose=asset.get("pose",{})
        markers.append({
            "asset_id":asset.get("asset_id"),
            "label":asset.get("name") or asset.get("asset_id"),
            "marker_type":_marker_type(asset),
            "x":float(pose.get("x",0.0)),
            "y":float(pose.get("y",0.0)),
            "z":float(pose.get("z",0.0)),
            "support_status":asset.get("support_status","supported"),
            "warning_badge":asset.get("support_status")=="preview_only",
        })
    roi=state.get("camera_pointcloud_roi") or {}
    roi_model=None
    if roi.get("enabled"):
        valid=roi.get("x_min",0)<roi.get("x_max",0) and roi.get("y_min",0)<roi.get("y_max",0) and roi.get("z_min",0)<roi.get("z_max",0)
        roi_model={**roi,"valid":valid}
        if not valid:
            warnings.append("ROI invalid")
    reach_helpers=[]
    for m in markers:
        if (m["asset_id"] or "").lower().startswith("ur5") or "ur5" in (m["label"] or "").lower():
            reach_helpers.append({"asset_id":m["asset_id"],"x":m["x"],"y":m["y"],"radius_m":0.85,"tooltip":"Approximate visual reach only — not a safety or reachability certificate."})
    return {"grid":{"enabled":True,"step_m":0.1},"origin":{"x":0.0,"y":0.0},"markers":markers,"roi":roi_model,"reach_helpers":reach_helpers,"warnings":warnings}


def update_asset_xy_from_canvas_move(state:dict[str,Any], asset_id:str, x:float, y:float)->dict[str,Any]:
    for asset in state.get("current_cell_assets",[]):
        if asset.get("asset_id")==asset_id:
            asset.setdefault("pose",{})["x"]=x
            asset["pose"]["y"]=y
            return asset
    raise KeyError(f"Asset not found: {asset_id}")


def export_layout_preview(state:dict[str,Any], output_dir:Path)->dict[str,Any]:
    output_dir.mkdir(parents=True,exist_ok=True)
    model=build_visual_layout_canvas_model(state)
    svg=["<svg xmlns='http://www.w3.org/2000/svg' width='900' height='700'>"]
    svg.append("<text x='12' y='20'>Workcell Layout Preview</text>")
    for idx,marker in enumerate(model["markers"]):
        cx=450+int(marker["x"]*250)
        cy=350-int(marker["y"]*250)
        color="#f59e0b" if marker["warning_badge"] else "#2563eb"
        svg.append(f"<circle cx='{cx}' cy='{cy}' r='8' fill='{color}'/>")
        svg.append(f"<text x='{cx+10}' y='{cy}'>{marker['label']}</text>")
    if model["roi"]:
        roi=model["roi"]
        color="#dc2626" if not roi["valid"] else "#16a34a"
        x=450+int(roi["x_min"]*250)
        y=350-int(roi["y_max"]*250)
        w=max(1,int((roi["x_max"]-roi["x_min"])*250))
        h=max(1,int((roi["y_max"]-roi["y_min"])*250))
        svg.append(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' fill='none' stroke='{color}' stroke-width='2'/>")
    svg.append("</svg>")
    (output_dir/"layout_preview.svg").write_text("\n".join(svg),encoding="utf-8")
    (output_dir/"layout_preview.html").write_text("<html><body><h1>Layout Preview</h1><object data='layout_preview.svg' type='image/svg+xml'></object></body></html>",encoding="utf-8")
    return {"ok":True,"files":["layout_preview.svg","layout_preview.html"],"warnings":model["warnings"]}


def register_custom_stl_asset(state:dict[str,Any], source_path:str, destination_dir:Path, *, xyz:list[float]|None=None, rpy:list[float]|None=None, scale:list[float]|None=None)->dict[str,Any]:
    src=Path(source_path)
    destination_dir.mkdir(parents=True, exist_ok=True)
    copied=destination_dir/src.name
    shutil.copy2(src,copied)
    asset=add_asset_to_cell(state, asset_id=src.stem, category="custom_stl", role="visual_object", name=src.name)
    asset["source_file"]=str(src)
    asset["copied_asset_path"]=str(copied)
    asset["collision_mode"]="visual_only"
    asset["pose"]={"x":(xyz or [0,0,0])[0],"y":(xyz or [0,0,0])[1],"z":(xyz or [0,0,0])[2],"roll":(rpy or [0,0,0])[0],"pitch":(rpy or [0,0,0])[1],"yaw":(rpy or [0,0,0])[2]}
    asset["scale"]=scale or [1.0,1.0,1.0]
    return asset
