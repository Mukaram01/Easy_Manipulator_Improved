#!/usr/bin/env python3
from __future__ import annotations
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _scene_yaml(scene_name: str) -> str:
    return f"""schema_version: workcell_scene/v1
scene:
  name: {scene_name}
robot:
  id: ur5
tool:
  id: robotiq_2f85
compatibility:
  status: COMPATIBLE
  profile_pair: ur5__robotiq_2f85
  tcp_frame: tool0
  tool_mount_link: ee_link
placed_objects:
  - name: table_01
    asset_id: table_small
    pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - name: pick_object_01
    asset_id: pick_box
    pose: [0.45, 0.0, 0.78, 0.0, 0.0, 0.0]
  - name: bin_left
    asset_id: bin_small
    pose: [0.55, 0.25, 0.75, 0.0, 0.0, 0.0]
  - name: bin_right
    asset_id: bin_large
    pose: [0.55, -0.25, 0.75, 0.0, 0.0, 0.0]
  - name: camera_stand_01
    asset_id: camera_stand
    optional: true
    pose: [0.25, -0.45, 0.0, 0.0, 0.0, 0.0]
  - name: safety_panel_01
    asset_id: safety_fence_panel
    optional: true
    pose: [1.0, -0.8, 0.0, 0.0, 0.0, 0.0]
  - name: robot_base_01
    asset_id: robot_base_plate
    optional: true
    pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
camera:
  enabled: true
  camera_id: realsense_d435i
  frame_id: camera_link
  pose: [0.25, -0.45, 1.25, -2.6, 0.0, -1.57]
  rgb_topic: /camera/color/image_raw
  depth_topic: /camera/depth/image_rect_raw
  pointcloud_topic: /camera/depth/color/points
task:
  schema_version: workcell_task/v1
  pick_source: pick_object_01
  place_target: bin_left
  grasp_strategy: finger_top
  release_strategy: open_gripper
workspace:
  bounds:
    x_min: 0.10
    x_max: 0.80
    y_min: -0.50
    y_max: 0.50
    z_min: 0.65
    z_max: 1.20
  zones:
    - name: robot_base_exclusion
      type: exclusion
      shape: circle
      center: [0.0, 0.0]
      radius: 0.22
    - name: operator_warning_zone
      type: warning
      shape: rectangle
      min: [0.0, -0.65]
      max: [0.95, 0.65]
safety:
  fake_hardware_first: true
  motion_command_sent: false
  runtime_execution_enabled: false
  real_hardware_enabled: false
  moveit_plan_service_called: false
metadata:
  golden_demo: true
  generated_by: generate_golden_workcell_demo.py
  object_placement_metadata: present
  visual_layout_metadata: present
  camera_metadata_status: present
  compatibility_status: COMPATIBLE
  readiness_overlay_status: present
"""


def main() -> int:
    p = argparse.ArgumentParser(description="Generate deterministic golden Workcell Studio demo pack")
    p.add_argument("--output-dir", default="/tmp/workcell_golden_demo")
    p.add_argument("--scene-name", default="ur5_2f_golden_demo")
    p.add_argument("--validate", action="store_true")
    p.add_argument("--strict", action="store_true")
    p.add_argument("--overwrite", action="store_true")
    p.add_argument("--print-summary", action="store_true")
    p.add_argument("--no-rviz", action="store_true")
    p.add_argument("--install-into-scenes", action="store_true")
    a = p.parse_args()

    root = Path(__file__).resolve().parents[1]
    output_base = (root / "scenes") if a.install_into_scenes else Path(a.output_dir)
    scene_dir = output_base / a.scene_name
    if scene_dir.exists() and not a.overwrite:
        print(f"Refusing to overwrite existing scene: {scene_dir}")
        print("Use --overwrite to replace it.")
        return 1
    if scene_dir.exists():
        shutil.rmtree(scene_dir)

    _write(scene_dir / "environment.yaml", _scene_yaml(a.scene_name))
    _write(scene_dir / "config/task_recipe.yaml", """schema_version: workcell_task/v1
pick:
  source: pick_object_01
place:
  target: bin_left
grasp_strategy: finger_top
release_strategy: open_gripper
tool_type: robotiq_2f85
tcp_frame: tool0
safety:
  fake_hardware_first: true
  motion_command_sent: false
  runtime_execution_enabled: false
  real_hardware_enabled: false
  moveit_plan_service_called: false
""")
    _write(scene_dir / "config/perception_metadata.json", json.dumps({"camera_id": "realsense_d435i", "metadata_only": True}, indent=2) + "\n")
    _write(scene_dir / "config/compatibility_metadata.json", json.dumps({"status": "COMPATIBLE", "robot": "ur5", "tool": "robotiq_2f85"}, indent=2) + "\n")
    _write(scene_dir / "config/readiness_overlay_metadata.json", json.dumps({"status": "ready_for_offline_preview", "runtime_execution_enabled": False}, indent=2) + "\n")
    _write(scene_dir / "config/visual_layout_metadata.json", json.dumps({"layout": "top_down", "grid_enabled": True}, indent=2) + "\n")
    _write(scene_dir / "preview/workcell_preview.svg", '<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360"><text x="20" y="40">ur5_2f_golden_demo preview</text></svg>\n')
    _write(scene_dir / "preview/workcell_preview.html", "<html><body><h1>Golden Demo Preview</h1><p>Offline-only reference pack. Curated assets: table_small, bin_small, bin_large, pick_box, camera_stand, robot_base_plate.</p></body></html>\n")

    fake_launch = f"ros2 launch {a.scene_name} demo.launch.py use_fake_hardware:=true launch_rviz:={'false' if a.no_rviz else 'true'}"
    summary = {
        "golden_demo": True,
        "generated_by": "generate_golden_workcell_demo.py",
        "scene_name": a.scene_name,
        "scene_schema_validation_status": "pending",
        "compatibility_status": "COMPATIBLE",
        "readiness_overlay_status": "present",
        "camera_metadata_status": "present",
        "fake_hardware_launch_command": fake_launch,
        "placed_assets": [
      {"asset_id":"table_small","label":"Table Small","category":"Tables / Workbenches","mesh_path":"workcell_builder/workcell_builder/assets/environment/table_small_description/meshes/table_small.stl","urdf_path":"workcell_builder/workcell_builder/assets/environment/table_small_description/urdf/table_small.urdf.xacro","dimensions":[1.2,0.8,0.75],"default_pose":[0,0,0,0,0,0],"catalog_validation_status":"PASS"},
      {"asset_id":"bin_small","label":"Bin Small","category":"Bins / Trays / Totes","mesh_path":"workcell_builder/workcell_builder/assets/environment/bin_small_description/meshes/bin_small.stl","urdf_path":"workcell_builder/workcell_builder/assets/environment/bin_small_description/urdf/bin_small.urdf.xacro","dimensions":[0.4,0.3,0.2],"default_pose":[0.55,0.25,0.75,0,0,0],"catalog_validation_status":"PASS"}
    ],
    "safety": {
            "fake_hardware_first": True,
            "motion_command_sent": False,
            "runtime_execution_enabled": False,
            "real_hardware_enabled": False,
            "moveit_plan_service_called": False,
        },
    }
    _write(scene_dir / "workcell_studio_summary.json", json.dumps(summary, indent=2) + "\n")
    _write(scene_dir / "workcell_studio_summary.md", f"""# Workcell Studio Golden Demo Summary

golden_demo: true

generated_by: generate_golden_workcell_demo.py

scene_schema_validation_status: pending
compatibility_status: COMPATIBLE
readiness_overlay_status: present
camera_metadata_status: present

fake_hardware_launch_command: `{fake_launch}`

Safety flags:
- fake_hardware_first: true
- motion_command_sent: false
- runtime_execution_enabled: false
- real_hardware_enabled: false
- moveit_plan_service_called: false

Build/launch (fake hardware only):
- colcon build --symlink-install --packages-select {a.scene_name}
- source install/setup.bash
- {fake_launch}
""")
    _write(scene_dir / "readiness_report.json", json.dumps({"status": "PASS", "offline_only": True}, indent=2) + "\n")

    if a.validate:
        cmd = [sys.executable, str(root / "scripts/validate_workcell_scene.py"), "--scene-dir", str(scene_dir)]
        if a.strict:
            cmd.append("--strict")
        rc = subprocess.run(cmd, check=False).returncode
        summary["scene_schema_validation_status"] = "PASS" if rc == 0 else "FAIL"
        _write(scene_dir / "workcell_studio_summary.json", json.dumps(summary, indent=2) + "\n")
        if rc != 0:
            return rc

    if a.print_summary:
        print(json.dumps(summary, indent=2))
    print(f"Generated golden demo pack at: {scene_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
