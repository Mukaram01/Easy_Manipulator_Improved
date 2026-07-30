from __future__ import annotations
import json, os, shutil, subprocess, sys, traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any
import re
import yaml

from scripts.export_builder_scene_to_cell_definition import export_scene
from scripts.workcell_studio_path_resolver import resolve_repo_root

DEFAULTS={
    "robot":{"role":"robot_base","pose":{"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},"collision_mode":"mesh_collision","support_status":"supported"},
    "end_effector":{"role":"end_effector","pose":{"x":0,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},"collision_mode":"mesh_collision","support_status":"supported"},
    "support_surface":{"role":"support_surface","pose":{"x":0.6,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},"collision_mode":"bounding_box","support_status":"supported"},
    "pick_object":{"role":"pick_object","pose":{"x":0.45,"y":0,"z":0.2,"roll":0,"pitch":0,"yaw":0},"collision_mode":"bounding_box","support_status":"supported"},
    "place_target":{"role":"place_target","pose":{"x":0.5,"y":-0.35,"z":0.2,"roll":0,"pitch":0,"yaw":0},"collision_mode":"bounding_box","support_status":"supported"},
    "camera":{"role":"camera","pose":{"x":0.5,"y":0,"z":0.8,"roll":0,"pitch":-0.6,"yaw":0},"collision_mode":"visual_only","support_status":"supported"},
    "conveyor":{"role":"conveyor","pose":{"x":0.7,"y":0,"z":0,"roll":0,"pitch":0,"yaw":0},"collision_mode":"bounding_box","support_status":"preview_only"},
}
TASK_TEMPLATES={"pick_place","sorting_placeholder","inspection_placeholder","machine_tending_placeholder"}
GRASP_STRATEGIES={"auto","finger_top","finger_side","suction_top","suction_side"}
APPROACH_AXES={"x_plus","x_minus","y_plus","y_minus","z_up","z_down"}
PLACEHOLDER_TEMPLATES={"sorting_placeholder","inspection_placeholder","machine_tending_placeholder"}



def default_perception_profile(state:dict[str,Any])->dict[str,Any]:
    return {
        "perception": {
            "enabled": False,
            "provider": "epd",
            "camera": {
                "model": "realsense_d435i",
                "frame_id": "camera_color_optical_frame",
                "rgb_topic": "/camera/camera/color/image_raw",
                "depth_topic": "/camera/camera/depth/image_rect_raw",
                "camera_info_topic": "/camera/camera/color/camera_info",
                "pointcloud_topic": "/camera/camera/depth/color/points",
            },
            "epd": {
                "mode": "localization",
                "localization_topic": "/easy_perception_deployment/epd_localize_output",
                "tracking_topic": "/easy_perception_deployment/epd_tracking_output",
                "object_detection_topic": "/processor/epd_p2_output",
                "object_tracking_topic": "/processor/epd_p3_output",
                "qos": {"reliability": "best_effort", "depth": 1, "durability": "volatile"},
            },
            "object_mapping": {"default_pick_object_ref": "object_1", "class_to_task_target": {}},
            "dry_run": {"enabled": True, "snapshot_output": "detected_objects_snapshot.json", "bridge_payload_output": "runtime_bridge_payload.json"},
        }
    }


def default_sample_detected_objects()->dict[str,Any]:
    return {"schema_version":"detected_objects/v1","source":"epd","frame_id":"camera_color_optical_frame","timestamp":None,"objects":[{"id":"obj_001","label":"cube","confidence":0.92,"pose":{"xyz":[0.45,0.10,0.22],"rpy":[0.0,0.0,0.0]},"dimensions_xyz":[0.05,0.05,0.05],"tracking_id":None}]}

def default_task_grasp_config()->dict[str,Any]:
    return {"task":{"template":"pick_place","pick":{"source_ref":"","object_ref":""},"place":{"target_ref":""},"routing":{"mode":"direct"},"release":{"strategy":"open_gripper_or_disable_suction"},"runtime_ready":True},"grasp":{"strategy":"auto","approach_axis":"z_down","orientation_mode":"vertical","approach_distance_m":0.12,"retreat_distance_m":0.1,"allowed_roll_angles_deg":[0.0],"allowed_yaw_angles_deg":[0.0,90.0,180.0,270.0],"tool_offset_xyz":[0.0,0.0,0.0],"tool_offset_rpy":[0.0,0.0,0.0],"suction_cups":None}}

def ensure_task_grasp_config(state:dict[str,Any])->dict[str,Any]:
    payload=state.setdefault("task_grasp_composer", default_task_grasp_config())
    return payload

def _tool_supports_suction(tool_id:str)->bool:
    name=tool_id.lower()
    return "suction" in name or "vacuum" in name or "airpick" in name

def _tool_supports_finger(tool_id:str)->bool:
    name=tool_id.lower()
    return "robotiq" in name or "2f" in name or "gripper" in name or "finger" in name

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

def edit_asset_pose(state:dict[str,Any], index:int, *, x:float|None=None, y:float|None=None, z:float|None=None, roll:float|None=None, pitch:float|None=None, yaw:float|None=None, scale:list[float]|None=None, source_file:str|None=None, name:str|None=None, category:str|None=None)->dict[str,Any]:
    asset=state.setdefault("current_cell_assets",[])[index]
    pose=asset.setdefault("pose",{})
    for key,val in [("x",x),("y",y),("z",z),("roll",roll),("pitch",pitch),("yaw",yaw)]:
        if val is not None:
            pose[key]=float(val)
    if scale is not None:
        asset["scale"]=[float(v) for v in scale]
    if source_file is not None:
        asset["source_file"]=source_file
    if name is not None:
        asset["name"]=name
    if category is not None:
        asset["category"]=category
    return asset

def remove_selected_asset(state:dict[str,Any], index:int)->None:
    state.setdefault("current_cell_assets",[]).pop(index)

def import_custom_stl(state:dict[str,Any], filepath:str, collision_mode:str="visual_only")->dict[str,Any]:
    return add_asset_to_cell(state, asset_id=Path(filepath).stem, category="custom_stl", role="visual_object", name=Path(filepath).name) | {"source_file":filepath,"collision_mode":collision_mode}


def repo_root() -> Path:
    return resolve_repo_root()


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
    composer=ensure_task_grasp_config(state)
    task=composer.get("task",{})
    grasp=composer.get("grasp",{})
    template=task.get("template")
    if not template: issues.append({"severity":"FAIL","code":"TASK_INCOMPLETE","message":"Task template is required"})
    elif template not in TASK_TEMPLATES: issues.append({"severity":"FAIL","code":"TASK_INCOMPLETE","message":"Task template is invalid"})
    if not ((task.get("pick") or {}).get("source_ref")): issues.append({"severity":"FAIL","code":"TASK_INCOMPLETE","message":"Pick source is required"})
    if not ((task.get("place") or {}).get("target_ref")): issues.append({"severity":"FAIL","code":"TASK_INCOMPLETE","message":"Place target is required"})
    if not grasp.get("strategy"): issues.append({"severity":"FAIL","code":"TASK_INCOMPLETE","message":"Grasp strategy is required"})
    if grasp.get("strategy") and grasp.get("strategy") not in GRASP_STRATEGIES: issues.append({"severity":"FAIL","code":"GRASP_INCOMPATIBLE","message":"Invalid grasp strategy"})
    for fld in ["approach_distance_m","retreat_distance_m"]:
        if float(grasp.get(fld,0)) < 0: issues.append({"severity":"FAIL","code":"TASK_INCOMPLETE","message":f"{fld} must be non-negative"})
    if grasp.get("approach_axis") and grasp.get("approach_axis") not in APPROACH_AXES:
        issues.append({"severity":"FAIL","code":"TASK_INCOMPLETE","message":"Invalid approach axis"})
    selected_tool=str(state.get("selected",{}).get("tool") or "")
    strategy=str(grasp.get("strategy") or "")
    if strategy.startswith("suction") and not _tool_supports_suction(selected_tool):
        issues.append({"severity":"WARN","code":"GRASP_INCOMPATIBLE","message":"Suction grasp selected with non-suction tool"})
    if strategy.startswith("finger") and not _tool_supports_finger(selected_tool):
        issues.append({"severity":"WARN","code":"GRASP_INCOMPATIBLE","message":"Finger grasp selected with non-finger tool"})
    if template in PLACEHOLDER_TEMPLATES and task.get("runtime_ready",True):
        issues.append({"severity":"WARN","code":"PREVIEW_ONLY","message":"Placeholder task is preview-only and runtime unsupported"})
    has_fail=any(i["severity"]=="FAIL" for i in issues)
    status="FAIL" if has_fail else ("WARN" if issues else "OK")
    return {"status":status,"issues":issues,"fake_hardware_default": state.get("fake_hardware_default",True)}


def generate_canonical_files(state:dict[str,Any], output_dir:Path)->dict[str,Any]:
    output_dir.mkdir(parents=True,exist_ok=True)
    val=validate_manual_cell_state(state)
    assets=state.get("current_cell_assets",[])
    composer=ensure_task_grasp_config(state)
    scene_doc=_build_workcell_scene_v1(state, state.get("scene_name") or output_dir.parent.name)
    files={
        "cell_definition.yaml": _render_cell_definition_v1(state),
        "environment_layout.yaml": _render_environment_layout_v1(state),
        "task_recipe.yaml": json.dumps({"schema_version":"task_recipe/v1","task":composer.get("task",{}),"grasp":composer.get("grasp",{})},indent=2)+"\n",
        "grasp_strategy.yaml": json.dumps({"schema_version":"grasp_strategy/v1","grasp":composer.get("grasp",{})},indent=2)+"\n",
    }
    for name,content in files.items(): (output_dir/name).write_text(content,encoding="utf-8")
    (output_dir/"scene_schema_validator_command.txt").write_text("python3 scripts/validate_workcell_scene.py --scene-dir <generated_scene_dir>\n", encoding="utf-8")
    selected_payload={"selected":state.get("selected",{}),"current_cell_assets":assets,
        "catalog_selection":{
            "robot":state.get("selected",{}).get("robot"),
            "end_effector":state.get("selected",{}).get("tool"),
            "sensor":state.get("selected",{}).get("camera"),
            "task":state.get("selected",{}).get("task"),
        }}
    (output_dir/"selected_assets.json").write_text(json.dumps(selected_payload,indent=2),encoding="utf-8")
    (output_dir/"compatibility_report.json").write_text(json.dumps({"fake_hardware_default":True,"preview_only":bool(state.get("preview_only_assets"))},indent=2),encoding="utf-8")
    scene_manifest={"schema_version":"scene_manifest/v1","generated_by":"workcell_builder","selected_assets":selected_payload["selected"],"task":composer.get("task",{}),"grasp":composer.get("grasp",{}),"fake_hardware_default":state.get("fake_hardware_default",True)}
    (output_dir/"scene_manifest.yaml").write_text(json.dumps(scene_manifest,indent=2)+"\n",encoding="utf-8")
    (output_dir/"perception_profile.yaml").write_text(json.dumps(default_perception_profile(state),indent=2)+"\n",encoding="utf-8")
    (output_dir/"sample_detected_objects.yaml").write_text(json.dumps(default_sample_detected_objects(),indent=2)+"\n",encoding="utf-8")
    (output_dir/"runtime_bridge_payload.sample.json").write_text(json.dumps({"schema_version":"emd_grasp_bridge_payload/v1","source":"perception_replay","dry_run_only":True,"targets":[]},indent=2)+"\n",encoding="utf-8")
    (output_dir/"builder_export_summary.json").write_text(json.dumps({"validation_status":val["status"],"generated_files":sorted([p.name for p in output_dir.iterdir()]),"scene_manifest":scene_manifest,"task_grasp_summary":{"template":(composer.get("task",{}).get("template")),"grasp_strategy":(composer.get("grasp",{}).get("strategy"))},"perception_profile_generated":True,"scene_schema_version":"workcell_scene/v1","scene_schema_validation_status":("FAIL" if scene_doc["metadata"]["scene_schema_blockers"] else ("WARN" if scene_doc["metadata"]["scene_schema_warnings"] else "PASS")),"scene_schema_warnings":len(scene_doc["metadata"]["scene_schema_warnings"]),"scene_schema_blockers":len(scene_doc["metadata"]["scene_schema_blockers"])},indent=2),encoding="utf-8")
    return {"ok":True,"validation":val,"output_dir":str(output_dir),"generated_files":sorted([p.name for p in output_dir.iterdir()])}


def generate_studio_pack(state:dict[str,Any], output_dir:Path)->dict[str,Any]:
    try:
        result=generate_canonical_files(state,output_dir)
    except Exception as exc:
        return {"ok":False,"error":{"code":"generation_failure","message":str(exc)}}
    replay = generate_perception_replay_preview(output_dir)
    replay_status = replay.get("status","NOT_RUN")
    selected = (((replay.get("selected_target") or {}).get("selected_object")) or {})
    (output_dir/"readiness_summary.md").write_text(
        "# Readiness\n"
        f"- Final readiness: **{result['validation']['status']}**\n"
        "- Safety: offline/fake hardware first\n\n"
        "## Perception Replay\n"
        "- profile: config/perception_profile.yaml\n"
        "- input snapshot: config/sample_detected_objects.yaml\n"
        f"- selected object: {selected.get('label')}/{selected.get('id')}/{selected.get('confidence')}\n"
        f"- mapped place target: {((replay.get('selected_target') or {}).get('mapping') or {}).get('place_target_ref')}\n"
        f"- generated bridge payload preview: {replay_status}\n"
        "- live EPD launched automatically: false\n- runtime execution called: false\n- motion command sent: false\n",
        encoding="utf-8")
    (output_dir/"readiness_summary.html").write_text("<html><body><h1>Readiness</h1></body></html>",encoding="utf-8")
    (output_dir/"environment_preview.svg").write_text("<svg xmlns='http://www.w3.org/2000/svg'></svg>",encoding="utf-8")
    (output_dir/"environment_preview.html").write_text("<html><body>Preview</body></html>",encoding="utf-8")
    launch=copy_fake_hardware_launch_command(state)
    (output_dir/"generated_launch_commands.md").write_text(launch.get("message",""),encoding="utf-8")
    return {"ok":True,"output_dir":str(output_dir),"readiness":result["validation"]["status"],"runtime_classification":_runtime_classification(state,result["validation"]), "perception_replay": replay }

def generate_perception_replay_preview(scene_dir:Path)->dict[str,Any]:
    cfg = scene_dir
    profile = cfg/"perception_profile.yaml"
    snap = cfg/"sample_detected_objects.yaml"
    out = cfg/"runtime_bridge_payload.preview.json"
    markers = cfg/"perception_replay_markers.json"
    summary = cfg/"perception_replay_summary.json"
    selected = cfg/"selected_target_summary.json"
    task = scene_dir.parent/"task_recipe.yaml"
    grasp = scene_dir.parent/"grasp_strategy.yaml"
    env = scene_dir.parent/"environment_layout.yaml"
    if not (profile.exists() and snap.exists()):
        return {"status":"PERCEPTION_REPLAY_BLOCKED","reason":"missing_profile_or_snapshot"}
    cmd=[sys.executable, str(repo_root()/ "scripts"/"epd_snapshot_adapter.py"), "--profile", str(profile), "--input", str(snap), "--output", str(out), "--markers", str(markers), "--summary", str(summary), "--selected-summary", str(selected)]
    if task.exists(): cmd += ["--task", str(task)]
    if grasp.exists(): cmd += ["--grasp", str(grasp)]
    if env.exists(): cmd += ["--environment", str(env)]
    run=_run(cmd, cwd=repo_root())
    if not run["ok"]:
        return {"status":"PERCEPTION_REPLAY_BLOCKED","error":run.get("stderr","").strip()}
    sp = _load_json_safe(summary)
    st = _load_json_safe(selected)
    return {"status":"PERCEPTION_REPLAY_READY" if sp.get("status")=="READY" else "PERCEPTION_REPLAY_WARN","summary":sp,"selected_target":st,"bridge_payload_preview_ready":out.exists()}


def normalize_perception_config_from_environment(environment_payload:dict[str,Any]|None)->dict[str,Any]:
    """Normalize legacy/partial environment.yaml perception payloads safely."""
    payload = environment_payload if isinstance(environment_payload, dict) else {}
    raw = payload.get("perception")
    if raw is None:
        return {"status": "MISSING_PERCEPTION", "enabled": False, "config": {}}
    if isinstance(raw, str):
        token = raw.strip().lower()
        if token in {"disabled", "none", "false", "off", ""}:
            return {"status": "PERCEPTION_DISABLED", "enabled": False, "config": {"enabled": False}}
        return {"status": "PERCEPTION_LEGACY_SCALAR", "enabled": False, "config": {"enabled": False}}
    if isinstance(raw, bool):
        return {"status": "PERCEPTION_DISABLED" if raw is False else "PERCEPTION_ENABLED_NO_DETAILS", "enabled": bool(raw), "config": {"enabled": bool(raw)}}
    if not isinstance(raw, dict):
        return {"status": "PERCEPTION_LEGACY_SCALAR", "enabled": False, "config": {"enabled": False}}
    if not raw:
        return {"status": "PERCEPTION_EMPTY_CONFIG", "enabled": False, "config": {}}
    enabled = bool(raw.get("enabled", True))
    return {"status": "PERCEPTION_ENABLED" if enabled else "PERCEPTION_DISABLED", "enabled": enabled, "config": raw}


def parse_environment_yaml_safely(environment_yaml_text:str)->dict[str,Any]:
    try:
        loaded = yaml.safe_load(environment_yaml_text) or {}
    except Exception as exc:
        return {"ok": False, "error": f"malformed_environment_yaml: {exc}", "environment": {}}
    if not isinstance(loaded, dict):
        return {"ok": False, "error": "malformed_environment_yaml: root must be mapping", "environment": {}}
    return {"ok": True, "environment": loaded}


def build_epd_snapshot_adapter_command(*, profile:Path|None, input_snapshot:Path|None, output_payload:Path|None)->dict[str,Any]:
    if not profile or not input_snapshot or not output_payload:
        return {"ok": False, "error": "missing required args for epd_snapshot_adapter.py", "command": []}
    return {"ok": True, "command": [sys.executable, str(repo_root()/ "scripts"/"epd_snapshot_adapter.py"), "--profile", str(profile), "--input", str(input_snapshot), "--output", str(output_payload)]}


def build_perception_bridge_preview_command(*, perception_profile:Path|None, detected_objects:Path|None, task_intent:Path|None, output_payload:Path|None, output_report:Path|None)->dict[str,Any]:
    if not perception_profile or not detected_objects or not task_intent or not output_payload or not output_report:
        return {"ok": False, "error": "missing required args for generate_perception_bridge_preview.py", "command": []}
    return {"ok": True, "command": [sys.executable, str(repo_root()/ "scripts"/"generate_perception_bridge_preview.py"), "--perception-profile", str(perception_profile), "--detected-objects", str(detected_objects), "--task-intent", str(task_intent), "--output-payload", str(output_payload), "--output-report", str(output_report)]}

def _load_json_safe(path:Path)->dict[str,Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _runtime_classification(state:dict[str,Any], validation:dict[str,Any])->str:
    if state.get("preview_only_assets"):
        return "preview_only"
    if validation.get("status") == "FAIL":
        return "blocked_by_validation_errors"
    if state.get("fake_hardware_default") is not True:
        return "requires_real_hardware_review"
    return "runtime_ready"

def build_readiness_status_panel(state:dict[str,Any], validation:dict[str,Any]|None=None)->dict[str,Any]:
    validation=validation or validate_manual_cell_state(state)
    badges={"VALID":False,"WARN":False,"BLOCKED":False,"PREVIEW_ONLY":False,"TASK_INCOMPLETE":False,"GRASP_INCOMPATIBLE":False,"PERCEPTION_SOURCE_UNCONFIGURED":False,"RUNTIME_UNSUPPORTED":False,"MISSING_ASSET":False,"FAKE_HARDWARE":state.get("fake_hardware_default",True),"REAL_HARDWARE_REVIEW_REQUIRED":False}
    messages=[]
    if validation["status"]=="OK":
        badges["VALID"]=True
    elif validation["status"]=="WARN":
        badges["WARN"]=True
    else:
        badges["BLOCKED"]=True
    selected=state.get("selected",{})
    if not selected.get("robot") or not selected.get("tool"):
        badges["MISSING_ASSET"]=True
    preview_combo=False
    robot_name=str(selected.get("robot") or "").lower()
    if "generic" in robot_name or "placeholder" in robot_name:
        badges["PREVIEW_ONLY"]=True
        badges["RUNTIME_UNSUPPORTED"]=True
        preview_combo=True
    if state.get("preview_only_assets") or any(a.get("support_status")=="preview_only" for a in state.get("current_cell_assets",[])):
        badges["PREVIEW_ONLY"]=True
    if state.get("fake_hardware_default") is not True:
        badges["REAL_HARDWARE_REVIEW_REQUIRED"]=True
    if preview_combo:
        messages.append("Placeholder robot family is preview-only and runtime unsupported.")
    for issue in validation.get("issues",[]):
        if issue.get("code") in badges: badges[issue.get("code")] = True
        messages.append(issue.get("message",""))
    perception_profile = default_perception_profile(state)
    perception = perception_profile.get("perception", {})
    p_enabled = bool(perception.get("enabled", False))
    p_status = "PERCEPTION_READY_CONFIG_ONLY"
    if not perception_profile:
        p_status = "PERCEPTION_PROFILE_MISSING"
    elif not perception.get("camera", {}).get("frame_id"):
        p_status = "CAMERA_FRAME_MISSING"
    elif not perception.get("epd", {}).get("localization_topic"):
        p_status = "EPD_TOPIC_MISSING"
    elif not perception.get("object_mapping", {}).get("default_pick_object_ref"):
        p_status = "OBJECT_MAPPING_MISSING"
    elif not p_enabled:
        p_status = "PERCEPTION_DISABLED"
    if p_status == "PERCEPTION_PROFILE_MISSING" or p_status == "CAMERA_FRAME_MISSING":
        badges["PERCEPTION_SOURCE_UNCONFIGURED"] = True
    messages.append("Config generated. Live EPD not launched automatically.")
    messages.append(f"Perception status: {p_status}")
    return {"validation_status":validation["status"],"badges":badges,"messages":messages,"perception_status":p_status,"perception_profile":perception_profile}

def build_preview_launch_plan(state:dict[str,Any], scene_package:str|None=None)->dict[str,Any]:
    validation=validate_manual_cell_state(state)
    readiness=build_readiness_status_panel(state,validation)
    pkg=scene_package or "<scene_package>"
    cmd=f"ros2 launch {pkg} demo.launch.py use_fake_hardware:=true"
    return {"validation":validation,"readiness":readiness,"command":cmd,"manual_launch_only":True,"auto_execute":False}


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


def _sanitize_scenario_name(name:str)->str:
    return re.sub(r"[^a-zA-Z0-9_]+", "_", (name or "").strip()).strip("_").lower()


def _gripper_suffix(gripper_type:str)->str:
    normalized=(gripper_type or "").strip().lower()
    mapping={"2f":"2f","3f":"3f","suction":"suction","airpick":"airpick4","airpick4":"airpick4"}
    if normalized not in mapping:
        raise ValueError(f"Unsupported gripper type: {gripper_type}")
    return mapping[normalized]




def load_emd_planner_defaults(params_2f_path:Path|None=None)->dict[str,Any]:
    defaults={
        "easy_perception_deployment":{
            "epd_enabled":True,
            "epd_localization_topic":"/easy_perception_deployment/epd_localize_output",
            "epd_tracking_topic":"/easy_perception_deployment/epd_tracking_output",
            "epd_tracking_enabled":True,
            "epd_perception_service":"epd_perception_service",
            "epd_subscription_reliability":"best_effort",
        },
        "camera_parameters":{
            "point_cloud_topic":"/camera/camera/depth/color/points",
            "camera_frame":"camera_depth_optical_frame",
            "robot_base_frame":"base_link",
            "point_cloud_subscription_reliability":"best_effort",
            "fx":1.0,"fy":1.0,"ppx":0.0,"ppy":0.0,
        },
        "point_cloud_params":{},
        "end_effectors":{},
        "visualization_params":{"point_cloud_visualization":False},
    }
    if params_2f_path and Path(params_2f_path).exists():
        payload=yaml.safe_load(Path(params_2f_path).read_text(encoding='utf-8')) or {}
        params=((payload.get('grasp_planning_node') or {}).get('ros__parameters') or {})
        for k in defaults:
            if isinstance(defaults[k],dict):
                defaults[k].update(params.get(k) or {})
    return defaults

def generate_emd_planner_files(state:dict[str,Any], scenario_name:str, gripper_type:str, planner_values:dict[str,Any], planner_dir:Path|None=None, overwrite:bool=False, generate_execution_wrapper:bool=True)->dict[str,Any]:
    scenario=_sanitize_scenario_name(scenario_name)
    suffix=_gripper_suffix(gripper_type)
    root=(planner_dir or (repo_root()/"easy_manipulation_deployment"/"emd_demo_nodes"/"run_grasp_planner"))
    cfg_dir=root/"config"; launch_dir=root/"launch"
    cfg_dir.mkdir(parents=True,exist_ok=True); launch_dir.mkdir(parents=True,exist_ok=True)
    config_name=f"params_{scenario}_{suffix}.yaml"
    launch_name=f"grasp_planner_{scenario}.launch.py"
    config_path=cfg_dir/config_name; launch_path=launch_dir/launch_name
    params=planner_values
    yaml_payload={"grasp_planning_node":{"ros__parameters":params}}
    config_text="# Generated by Workcell Studio. Do not edit manually.\n" + yaml.safe_dump(yaml_payload, sort_keys=False)
    launch_text=f"# Generated by Workcell Studio. Do not edit manually.\nimport os\nfrom ament_index_python.packages import get_package_share_directory\nfrom launch import LaunchDescription\nfrom launch_ros.actions import Node\n\ndef generate_launch_description():\n    config = os.path.join(get_package_share_directory('run_grasp_planner'), 'config', '{config_name}')\n    return LaunchDescription([\n        Node(\n            package='run_grasp_planner',\n            name='grasp_planning_node',\n            executable='demo_node',\n            output='screen',\n            parameters=[config],\n        )\n    ])\n"
    existing=[]
    for p,t in ((config_path,config_text),(launch_path,launch_text)):
        if p.exists() and not overwrite:
            existing.append(str(p))
            continue
        p.write_text(t, encoding='utf-8')
    result={
        "config_path": str(config_path),
        "launch_path": str(launch_path),
        "planner_command": f"ros2 launch run_grasp_planner {launch_name}",
        "execution_command": f"ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:={scenario}",
        "warnings": ["generated files exist and will not be overwritten unless user confirms"] if existing else [],
        "missing_fields": [],
        "existing_files": existing,
    }
    if generate_execution_wrapper:
        ex_root=root.parent/'run_grasp_execution'/'launch'
        ex_root.mkdir(parents=True,exist_ok=True)
        ex_name=f'grasp_execution_{scenario}.launch.py'
        ex_path=ex_root/ex_name
        ex_text=f"# Generated by Workcell Studio. Do not edit manually.\nfrom launch import LaunchDescription\nfrom launch.actions import IncludeLaunchDescription\nfrom launch.launch_description_sources import PythonLaunchDescriptionSource\nfrom launch.substitutions import PathJoinSubstitution\nfrom launch_ros.substitutions import FindPackageShare\n\ndef generate_launch_description():\n    launch_file = PathJoinSubstitution([FindPackageShare('run_grasp_execution'),'launch','grasp_execution.launch.py'])\n    return LaunchDescription([\n        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file), launch_arguments={{'scene_package':'{scenario}'}}.items())\n    ])\n"
        if not ex_path.exists() or overwrite:
            ex_path.write_text(ex_text, encoding='utf-8')
        result['execution_wrapper_path']=str(ex_path)
        result['execution_wrapper_command']=f'ros2 launch run_grasp_execution {ex_name}'
    return result


def validate_generated_emd_planner_config(payload:dict[str,Any])->dict[str,Any]:
    errors=[]; warnings=[]
    root=((payload.get("grasp_planning_node") or {}).get("ros__parameters") or {})
    if not root: errors.append("missing grasp_planning_node.ros__parameters")
    pc=(root.get("point_cloud_params") or {})
    cam=(root.get("camera_parameters") or {})
    epd=(root.get("easy_perception_deployment") or {})
    ee=(root.get("end_effectors") or {})
    if not ee.get("selected_end_effector"): errors.append("no end effector selected")
    if not pc.get("point_cloud_topic") and not cam.get("point_cloud_topic"): errors.append("missing point_cloud_topic")
    if not cam.get("camera_frame"): errors.append("missing camera_frame")
    if not cam.get("robot_base_frame"): errors.append("missing robot_base_frame")
    if epd.get("epd_enabled") and not epd.get("epd_localization_topic"): errors.append("epd_enabled true but localization topic empty")
    for axis in ("x","y","z"):
        if pc.get(f"passthrough_{axis}_min",0) > pc.get(f"passthrough_{axis}_max",0): errors.append("invalid passthrough min/max")
    if cam.get("fx",0) in (0,1.0) or cam.get("fy",0) in (0,1.0): warnings.append("camera intrinsics are still default")
    if not epd.get("epd_tracking_enabled",True): warnings.append("tracking disabled")
    if ((root.get("visualization_params") or {}).get("point_cloud_visualization")): warnings.append("point_cloud_visualization enabled")
    return {"status": "ERROR" if errors else ("WARN" if warnings else "OK"), "errors": errors, "warnings": warnings}


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
        marker_type=_marker_type(asset)
        markers.append({
            "asset_id":asset.get("asset_id"),
            "label":asset.get("name") or asset.get("asset_id"),
            "marker_type":marker_type,
            "x":float(pose.get("x",0.0)),
            "y":float(pose.get("y",0.0)),
            "z":float(pose.get("z",0.0)),
            "roll":float(pose.get("roll",0.0)),
            "pitch":float(pose.get("pitch",0.0)),
            "yaw":float(pose.get("yaw",0.0)),
            "scale":asset.get("scale",[1.0,1.0,1.0]),
            "source_file":asset.get("source_file"),
            "support_status":asset.get("support_status","supported"),
            "warning_badge":asset.get("support_status")=="preview_only",
            "shape":"mesh_label" if marker_type=="object" and asset.get("category")=="custom_stl" else "box",
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
    composer=ensure_task_grasp_config(state)
    task=composer.get("task",{})
    grasp=composer.get("grasp",{})
    pick_id=((task.get("pick") or {}).get("source_ref"))
    place_id=((task.get("place") or {}).get("target_ref"))
    marker_map={m["asset_id"]:m for m in markers}
    task_flow={}
    if pick_id in marker_map and place_id in marker_map:
        p0=marker_map[pick_id]; p1=marker_map[place_id]
        task_flow={"pick_source":pick_id,"place_target":place_id,"arrow":{"from":[p0["x"],p0["y"]],"to":[p1["x"],p1["y"]]},"approach_vector":{"axis":grasp.get("approach_axis"),"distance_m":grasp.get("approach_distance_m")},"retreat_vector":{"axis":"z_up","distance_m":grasp.get("retreat_distance_m")},"label":f"{task.get('template')} / {grasp.get('strategy')}"}
    else:
        warnings.append("Task source/target missing from scene markers")
    if task.get("template") in PLACEHOLDER_TEMPLATES:
        warnings.append("Task template is preview-only")
    return {"grid":{"enabled":True,"step_m":0.1},"origin":{"x":0.0,"y":0.0},"markers":markers,"roi":roi_model,"reach_helpers":reach_helpers,"warnings":warnings,"task_flow":task_flow}


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
    if model.get("task_flow",{}).get("arrow"):
        a=model["task_flow"]["arrow"]; x1=450+int(a["from"][0]*250); y1=350-int(a["from"][1]*250); x2=450+int(a["to"][0]*250); y2=350-int(a["to"][1]*250)
        svg.insert(2,f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='#059669' stroke-width='3' marker-end='url(#arrow)'/>")
        svg.insert(1,"<defs><marker id='arrow' markerWidth='10' markerHeight='10' refX='8' refY='3' orient='auto'><polygon points='0 0, 10 3, 0 6' fill='#059669'/></marker></defs>")
        svg.insert(3,f"<text x='12' y='40'>Task: {model['task_flow'].get('label')}</text>")
    svg.append("</svg>")
    svg_text="\n".join(svg)
    (output_dir/"layout_preview.svg").write_text(svg_text,encoding="utf-8")
    (output_dir/"layout_preview.html").write_text("<html><body><h1>Layout Preview</h1><object data='layout_preview.svg' type='image/svg+xml'></object></body></html>",encoding="utf-8")
    (output_dir/"static_preview.svg").write_text(svg_text,encoding="utf-8")
    (output_dir/"preview_markers.json").write_text(json.dumps({"markers":model["markers"],"warnings":model["warnings"],"task_flow":model.get("task_flow",{})},indent=2),encoding="utf-8")
    return {"ok":True,"files":["layout_preview.svg","layout_preview.html","static_preview.svg","preview_markers.json"],"warnings":model["warnings"]}

def load_existing_scene_into_state(scene_dir:Path, state:dict[str,Any]|None=None)->dict[str,Any]:
    state=state or {}
    generated=scene_dir/"generated"/"environment_layout.yaml"
    selected_json=scene_dir/"generated"/"selected_assets.json"
    if generated.exists():
        payload=json.loads(generated.read_text(encoding="utf-8"))
        assets=[]
        for item in payload.get("assets",[]):
            xyz=item.get("xyz",[0,0,0]); rpy=item.get("rpy",[0,0,0])
            assets.append({"asset_id":item.get("id"),"name":item.get("id"),"category":item.get("type","object"),"role":item.get("role",item.get("type","object")),"pose":{"x":xyz[0],"y":xyz[1],"z":xyz[2],"roll":rpy[0],"pitch":rpy[1],"yaw":rpy[2]},"scale":item.get("scale",[1,1,1]),"source_file":item.get("source_file"),"support_status":"preview_only" if "placeholder" in str(item.get("type","")).lower() else "supported"})
        state["current_cell_assets"]=assets
    if selected_json.exists():
        selected_payload=json.loads(selected_json.read_text(encoding="utf-8"))
        state["selected"]=selected_payload.get("selected",{})
    state["scene_name"]=scene_dir.name
    state["canvas_model"]=build_visual_layout_canvas_model(state)
    return state


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



ASSET_CATEGORIES={
    "Robots":["ur3","ur5","ur10","fanuc","panda"],
    "Grippers & Tools":["robotiq_2f","robotiq_3f","single_suction","onrobot_airpick4"],
    "Cameras & Sensors":["realsense_d435i"],
    "Tables & Workbenches":["table","workbench"],
    "Bins & Totes":["bin","tote"],
    "Conveyors":["conveyor"],
    "Fixtures & Jigs":["fixture","jig"],
    "Objects":["cube","object"],
    "Custom STL":["custom_stl"],
}

def list_catalog_assets_by_category(category:str)->list[str]:
    return list(ASSET_CATEGORIES.get(category, []))

def resolve_output_folder(path:str|Path|None=None)->dict[str,Any]:
    root=Path(path).expanduser() if path else _default_scenes_root()
    root=root.resolve()
    return {"path":str(root),"writable":root.exists() and os.access(root, os.W_OK) if root.exists() else os.access(root.parent, os.W_OK)}

def generation_prerequisites(state:dict[str,Any], scene_dir:str|Path|None=None)->dict[str,Any]:
    issues=[]
    if not state.get("scene_name"):
        issues.append("Create or open a cell first")
    selected=state.get("selected",{})
    if not selected.get("robot"):
        issues.append("Select a robot")
    if not selected.get("tool"):
        issues.append("Select an end effector")
    if not state.get("current_cell_assets"):
        issues.append("Add at least one asset to the cell")
    yaml_ready=not issues
    files_ready=False
    if scene_dir:
        files_ready=(Path(scene_dir)/"environment.yaml").exists()
    return {"generate_yaml_enabled":yaml_ready,"generate_yaml_tooltip":"Ready" if yaml_ready else "Disabled: " + "; ".join(issues),"generate_files_enabled":files_ready,"generate_files_tooltip":"Ready" if files_ready else "Disabled: Generate YAML files for scene first (environment.yaml missing)"}

def create_golden_demo_cell(scene_name:str, output_folder:str|Path|None=None)->dict[str,Any]:
    created=create_new_cell(scene_name, output_folder)
    if not created.get("ok"):
        return created
    state={"scene_name":created["cell_name"],"fake_hardware_default":True,"selected":{"robot":"ur5","tool":"robotiq_2f","camera":"realsense_d435i","support_surface":"workbench","pick_area":"bin","place_target":"pick_area","task":"pick_place","grasp_strategy":"finger_top"},"current_cell_assets":[]}
    add_asset_to_cell(state,'ur5','robot','robot_base','UR5')
    add_asset_to_cell(state,'robotiq_2f','gripper','end_effector','Robotiq 2F')
    add_asset_to_cell(state,'realsense_d435i','camera','camera','RealSense D435i')
    add_asset_to_cell(state,'workbench','workbench','support_surface','Workbench')
    add_asset_to_cell(state,'bin','bin','pick_object','Bin')
    add_asset_to_cell(state,'cube','object','pick_object','Cube')
    add_asset_to_cell(state,'pick_area','object','place_target','Pick Area')
    scene_dir=Path(created['scene_dir'])
    yaml_result=generate_yaml_files_for_scene(state, scene_dir)
    package_result=generate_files_from_yaml(scene_dir)
    launch_cmd=f"ros2 launch {created['cell_name']} demo.launch.py use_fake_hardware:=true"
    status=(f"Golden Demo: PASS. Created {scene_dir}. Next: Generate files from YAML or build package with {package_result.get('build_command','')}")
    return {"ok":True,"state":state,"scene_dir":str(scene_dir),"yaml":yaml_result,"package":package_result,"build_command":package_result.get("build_command",""),"launch_command":launch_cmd,"status":status}
ROS_PACKAGE_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def evaluate_robot_tool_compatibility(robot:str|None, tool:str|None):
    if not robot or not tool:
        return None
    r=str(robot).lower(); t=str(tool).lower()
    if "airpick" in t and "ur5" not in r:
        return False
    return True


def validate_cell_name(cell_name:str)->dict[str,Any]:
    name=(cell_name or "").strip()
    if not name:
        return {"ok":False,"error":"Cell name is required"}
    if not ROS_PACKAGE_RE.match(name):
        return {"ok":False,"error":"Cell name must be lower-case ROS package style (start with letter, use letters/numbers/underscores only)"}
    return {"ok":True,"cell_name":name}


def _default_scenes_root()->Path:
    return Path.home()/"workcell_ws"/"src"/"scenes"




def _build_workcell_scene_v1(state:dict[str,Any], scene_name:str)->dict[str,Any]:
    selected=state.get("selected",{}) if isinstance(state.get("selected",{}),dict) else {}
    composer=ensure_task_grasp_config(state)
    compat=evaluate_robot_tool_compatibility(selected.get("robot"), selected.get("tool"))
    compat_status="COMPATIBLE"
    blockers=[]
    warnings=[]
    if not selected.get("robot") or not selected.get("tool"):
        compat_status="UNKNOWN_COMPATIBILITY"
        warnings.append("missing robot/tool selection")
    elif compat is False:
        compat_status="INCOMPATIBLE"
        blockers.append("known incompatible robot/tool pair")
    elif compat is None:
        compat_status="UNKNOWN_COMPATIBILITY"
        warnings.append("compatibility profile missing")

    placed=[]
    for idx,a in enumerate(state.get("current_cell_assets",[]) or []):
        if not isinstance(a,dict):
            continue
        pose=a.get("pose") if isinstance(a.get("pose"),dict) else {}
        xyz=pose.get("xyz",[0.0,0.0,0.0]); rpy=pose.get("rpy",[0.0,0.0,0.0])
        pose6=[float(xyz[0]) if len(xyz)>0 else 0.0,float(xyz[1]) if len(xyz)>1 else 0.0,float(xyz[2]) if len(xyz)>2 else 0.0,float(rpy[0]) if len(rpy)>0 else 0.0,float(rpy[1]) if len(rpy)>1 else 0.0,float(rpy[2]) if len(rpy)>2 else 0.0]
        name=str(a.get("name") or a.get("asset_id") or f"asset_{idx}")
        mesh_path=str(a.get("source_file") or "")
        collision_enabled=str(a.get("collision_mode","visual_only")).lower() != "visual_only"
        parent_frame=str(a.get("parent_frame") or "world")
        scale=a.get("scale",[1.0,1.0,1.0])
        if not isinstance(scale,list):
            scale=[1.0,1.0,1.0]
        scale=[float(scale[0]) if len(scale)>0 else 1.0,float(scale[1]) if len(scale)>1 else 1.0,float(scale[2]) if len(scale)>2 else 1.0]
        placed.append({
            "id":a.get("asset_id") or f"asset_{idx}",
            "name":name,
            "category":a.get("category","unknown"),
            "role":a.get("role","unknown"),
            "source":a.get("source","asset_stl"),
            "pose":pose6,
            "mesh_path":mesh_path,
            "collision_enabled":collision_enabled,
            "parent_frame":parent_frame,
            "scale":scale,
        })

    camera_id=selected.get("camera") or "UNKNOWN_CAMERA"
    camera_enabled=bool(selected.get("camera"))
    if not camera_enabled:
        warnings.append("camera disabled: no camera selection")

    validation=validate_manual_cell_state(state)
    for issue in validation.get("issues",[]):
        if issue.get("severity")=="FAIL": blockers.append(issue.get("message","validation blocker"))
        elif issue.get("severity")=="WARN": warnings.append(issue.get("message","validation warning"))

    return {
        "schema_version":"workcell_scene/v1",
        "scene":{"id":scene_name,"name":scene_name},
        "robot":{"id":selected.get("robot") or "UNKNOWN_ROBOT","profile":selected.get("robot") or "missing_profile"},
        "tool":{"id":selected.get("tool") or "UNKNOWN_TOOL","profile":selected.get("tool") or "missing_profile"},
        "compatibility":{"status":compat_status,"robot":selected.get("robot") or "UNKNOWN_ROBOT","tool":selected.get("tool") or "UNKNOWN_TOOL"},
        "placed_objects":placed,
        "camera":{"enabled":camera_enabled,"camera_id":camera_id,"frame_id":"camera_color_optical_frame" if camera_enabled else "UNKNOWN_FRAME","pose":[-0.55,0.55,0.40,0.0,0.0,0.0],"rgb_topic":"/camera/color/image_raw" if camera_enabled else "","depth_topic":"/camera/depth/image_rect_raw" if camera_enabled else "","pointcloud_topic":"/camera/depth/color/points" if camera_enabled else ""},
        "task":{"template":composer.get("task",{}).get("template") or "UNKNOWN_TASK","pick":(composer.get("task",{}).get("pick") or {}),"place":(composer.get("task",{}).get("place") or {}),"grasp":composer.get("grasp",{})},
        "workspace":{"bounds":{"x_min":-1.0,"x_max":1.0,"y_min":-1.0,"y_max":1.0,"z_min":0.0,"z_max":1.8},"zones":[{"id":"robot_base_exclusion","type":"exclusion","shape":"circle"}]},
        "safety":{"fake_hardware_first":True,"real_hardware_enabled":False,"runtime_execution_enabled":False,"motion_command_sent":False},
        "metadata":{"generated_by":"workcell_builder","scene_schema_version":"workcell_scene/v1","scene_schema_warnings":warnings,"scene_schema_blockers":blockers}
    }

def _scene_stub(cell_name:str, state:dict[str,Any]|None=None)->str:
    payload=_build_workcell_scene_v1(state or {}, cell_name)
    return "\n".join(f"{line}" for line in _to_yaml_lines(payload))+"\n"


def _to_yaml_lines(data, indent:int=0):
    sp="  "*indent
    if isinstance(data,dict):
        lines=[]
        for k,v in data.items():
            if isinstance(v,(dict,list)):
                lines.append(f"{sp}{k}:")
                lines.extend(_to_yaml_lines(v, indent+1))
            else:
                if isinstance(v,bool): sval="true" if v else "false"
                elif v is None: sval="null"
                elif isinstance(v,(int,float)): sval=str(v)
                else: sval=str(v)
                lines.append(f"{sp}{k}: {sval}")
        return lines
    if isinstance(data,list):
        lines=[]
        for item in data:
            if isinstance(item,(dict,list)):
                lines.append(f"{sp}-")
                lines.extend(_to_yaml_lines(item, indent+1))
            else:
                lines.append(f"{sp}- {item}")
        return lines
    return [f"{sp}{data}"]


def create_new_cell(cell_name:str, output_folder:str|Path|None=None)->dict[str,Any]:
    valid=validate_cell_name(cell_name)
    if not valid["ok"]:
        return valid
    root=Path(output_folder).expanduser() if output_folder else _default_scenes_root()
    root.mkdir(parents=True, exist_ok=True)
    if not root.is_dir() or not os.access(root, os.W_OK):
        return {"ok":False,"error":f"Output folder is not writable: {root}"}
    scene_dir=root/valid["cell_name"]
    scene_dir.mkdir(parents=True, exist_ok=True)
    (scene_dir/"environment.yaml").write_text(_scene_stub(valid["cell_name"]), encoding="utf-8")
    (scene_dir/"scene_manifest.yaml").write_text(json.dumps({"scene":valid["cell_name"],"fake_hardware_default":True}, indent=2)+"\n", encoding="utf-8")
    (scene_dir/"package.xml").write_text(f"<package><name>{valid['cell_name']}</name><version>0.0.1</version><description>Generated scene</description><maintainer email='builder@example.com'>builder</maintainer><license>Apache-2.0</license></package>\n", encoding="utf-8")
    (scene_dir/"CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.8)\nproject({})\nfind_package(ament_cmake REQUIRED)\ninstall(FILES environment.yaml scene_manifest.yaml DESTINATION share/${{PROJECT_NAME}})\nament_package()\n".format(valid["cell_name"]), encoding="utf-8")
    (scene_dir/"README.builder.md").write_text(f"# {valid['cell_name']}\nGenerated by workcell_builder.\n", encoding="utf-8")
    return {"ok":True,"scene_dir":str(scene_dir),"cell_name":valid["cell_name"],"output_folder":str(root)}


def repair_scene_yaml(scene_dir:str|Path, state:dict[str,Any]|None=None)->dict[str,Any]:
    sdir=Path(scene_dir)
    env=sdir/"environment.yaml"
    if env.exists():
        return {"ok":True,"repaired":False,"path":str(env)}
    scene_name=sdir.name
    if state and state.get("scene_name"):
        scene_name=str(state["scene_name"])
    env.write_text(_scene_stub(scene_name, state), encoding="utf-8")
    return {"ok":True,"repaired":True,"path":str(env)}


def generate_yaml_files_for_scene(state:dict[str,Any], scene_dir:str|Path)->dict[str,Any]:
    sdir=Path(scene_dir)
    sdir.mkdir(parents=True, exist_ok=True)
    scene_name=state.get("scene_name") or sdir.name
    state["scene_name"]=scene_name
    generated=[]
    (sdir/"environment.yaml").write_text(_scene_stub(scene_name, state), encoding="utf-8"); generated.append("environment.yaml")
    manifest={"scene":scene_name,"selected":state.get("selected",{}),"fake_hardware_default":state.get("fake_hardware_default",True)}
    (sdir/"scene_manifest.yaml").write_text(json.dumps(manifest,indent=2)+"\n", encoding="utf-8"); generated.append("scene_manifest.yaml")
    canonical=generate_canonical_files(state, sdir/"generated")
    generated.extend([f for f in canonical.get("generated_files",[]) if f in {"selected_assets.json","cell_definition.yaml","environment_layout.yaml"}])
    return {"ok":True,"generated":generated,"scene_dir":str(sdir)}


def generate_files_from_yaml(scene_dir:str|Path)->dict[str,Any]:
    """Regenerate an existing authored scene through the canonical exporter.

    This entry point deliberately does not scaffold a package or repair authored
    YAML.  Existing-scene regeneration consumes the two authored sources and
    writes derived artifacts only below ``generated``.
    """
    requested = Path(scene_dir).expanduser()
    try:
        sdir = requested.resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        return {"ok": False, "error": f"Cannot resolve scene directory '{requested}': {exc}"}
    if not sdir.is_dir():
        return {"ok": False, "error": f"Scene path is not a directory: {sdir}"}

    authored_paths = (
        sdir / "environment.yaml",
        sdir / "layout" / "workcell_studio_layout.yaml",
    )
    authored_bytes: dict[Path, bytes] = {}
    for path in authored_paths:
        if not path.is_file():
            return {"ok": False, "error": f"Required authored file is missing: {path}"}
        try:
            raw = path.read_bytes()
            parsed = yaml.safe_load(raw)
        except (OSError, yaml.YAMLError, UnicodeError) as exc:
            return {"ok": False, "error": f"Failed to parse authored file '{path}': {exc}"}
        if not isinstance(parsed, dict):
            return {"ok": False, "error": f"Authored file must contain a YAML mapping: {path}"}
        authored_bytes[path] = raw

    output_dir = sdir / "generated"
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        summary = export_scene(sdir, output_dir, validate=True)
    except Exception as exc:
        return {"ok": False, "error": f"Canonical export failed for authored scene '{sdir}': {exc}"}

    for path, before in authored_bytes.items():
        try:
            after = path.read_bytes()
        except OSError as exc:
            return {"ok": False, "error": f"Cannot verify authored file after export '{path}': {exc}"}
        if after != before:
            return {"ok": False, "error": f"Canonical export modified authored file unexpectedly: {path}"}

    validation = summary.get("validation") if isinstance(summary, dict) else None
    if not isinstance(validation, dict):
        return {"ok": False, "error": f"Canonical export returned no validation results for authored scene: {sdir}"}
    failed = [name for name, result in validation.items()
              if not isinstance(result, dict) or result.get("result") == "FAIL"]
    if failed:
        return {"ok": False, "error": f"Canonical export validation failed for authored file '{authored_paths[0]}': {', '.join(sorted(failed))}"}

    required_names = (
        "cell_definition.yaml",
        "environment_layout.yaml",
        "task_recipe_from_builder_intent.yaml",
        "offline_plan_preview_request.yaml",
        "selected_assets.json",
        "compatibility_report.json",
        "builder_export_summary.json",
    )
    generated_files = [output_dir / name for name in required_names]
    missing = [path for path in generated_files if not path.is_file()]
    if missing:
        return {"ok": False, "error": f"Canonical export of authored file '{authored_paths[0]}' did not produce required artifact: {missing[0]}"}

    cmd=f"cd {Path.home()/ 'workcell_ws'} && colcon build --symlink-install --packages-select {sdir.name}"
    return {
        "ok": True,
        "scene_dir": str(sdir),
        "output_dir": str(output_dir),
        "generated_files": [str(path) for path in generated_files],
        "validation": validation,
        "build_command": cmd,
    }
