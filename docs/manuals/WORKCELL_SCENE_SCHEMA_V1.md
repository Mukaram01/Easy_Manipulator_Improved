# Workcell Scene Schema v1

Schema version token: `workcell_scene/v1`.

Purpose: define one stable offline scene contract for `workcell_builder` generated scenes.

## Top-level sections
Required: `scene`, `robot`, `tool`, `compatibility`, `placed_objects`, `camera`, `task`, `safety`, `metadata`.

## Safety
- fake_hardware_first: true
- motion_command_sent: false
- runtime_execution_enabled: false
- real_hardware_enabled: false
- moveit_plan_service_called: false

## Compatibility statuses
- COMPATIBLE
- COMPATIBLE_WITH_WARNINGS
- UNKNOWN_COMPATIBILITY
- INCOMPATIBLE
- MISSING_TCP
- MISSING_MOUNT_LINK
- MISSING_CONTROLLER_METADATA

Unknown compatibility warns by default. Incompatible pair blocks generation.

## Example YAML
```yaml
schema_version: workcell_scene/v1
scene:
  name: ur5_2f_test
  package_name: ur5_2f_test
  generated_at: 2026-05-11T00:00:00Z
  workspace_hint: ~/workcell_ws
robot:
  robot_id: ur5
  label: UR5
  description_package: ur_description
  moveit_config_package: ur5_moveit_config
  base_link: base_link
  planning_group: manipulator
  base_pose: [0, 0, 0, 0, 0, 0]
tool:
  tool_id: robotiq_2f
  label: Robotiq 2F
  tool_type: finger
  description_package: robotiq_2f_description
  moveit_config_package: robotiq_2f_moveit_config
  mount_link: tool0
  tcp_frame: tool0
  tcp_xyz_rpy: [0, 0, 0, 0, 0, 0]
  controller_hint: ur_ros2_control
  requires_io: true
  grasp_strategy_default: finger_top
  release_strategy_default: open_gripper
compatibility:
  status: COMPATIBLE
  warnings: []
  blockers: []
  robot_profile: ur5
  tool_profile: robotiq_2f
  pair_profile: ur5__robotiq_2f
placed_objects:
  - name: table_01
    source: asset_stl
    mesh: package://easy_manipulation_deployment/assets/environment/table/meshes/table.stl
    pose: [0, 0, 0, 0, 0, 0]
camera:
  enabled: true
  camera_id: realsense_d435i
  label: RealSense D435i
  frame_id: camera_link
  optical_frame_id: camera_color_optical_frame
  pose: [x, y, z, roll, pitch, yaw]
  rgb_topic: /camera/camera/color/image_raw
  depth_topic: /camera/camera/depth/image_rect_raw
  camera_info_topic: /camera/camera/color/camera_info
  pointcloud_topic: /camera/camera/depth/color/points
  mount_type: fixed
  perception_hint: rgbd_object_detection
  epd_input_hint: external_epd_adapter
task:
  task_type: pick_place
  pick_source: pick_bin
  place_target: place_bin
  grasp_strategy: finger_top
  release_strategy: open_gripper
  approach_distance: 0.1
  retreat_distance: 0.1
  safety flags: offline_only
safety:
  fake_hardware_first: true
  motion_command_sent: false
  runtime_execution_enabled: false
  real_hardware_enabled: false
  moveit_plan_service_called: false
metadata:
  visual_layout_editor_used: true
  object_placement_manager_used: true
  generated_by: workcell_builder
  warnings: []
```

Validation examples:
- `python3 scripts/validate_workcell_scene.py --scene-dir scenes/ur5_2f_test`
- `python3 scripts/validate_workcell_scene.py --scene-file scenes/ur5_2f_test/environment.yaml --strict`

Operator flow: Open builder -> select scene -> robot/tool -> compatibility -> objects -> visual editor -> task/grasp -> generate -> Scene Schema Validation -> fix blockers/warnings -> build -> launch with `use_fake_hardware:=true`.


Camera validation notes: camera metadata is optional; when enabled camera_id/frame_id and six-value finite pose are required. Missing pointcloud topic is a warning; missing rgb/depth can be warning or blocker in strict mode. EPD metadata is export hint only.
