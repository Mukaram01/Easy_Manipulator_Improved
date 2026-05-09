# Workcell Studio EPD Perception Profile

Generated scenes now include `config/perception_profile.yaml` (config-only), `config/sample_detected_objects.yaml`, and dry-run bridge payload sample artifacts.

## Safety
- Config/adapter level integration only.
- Live EPD and RealSense are **not** auto-launched.
- Fake hardware remains default.

## Manual commands
- RealSense:
`ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=424x240x15 depth_module.depth_profile:=424x240x15 align_depth.enable:=true pointcloud.enable:=true`
- EPD:
`ros2 launch easy_perception_deployment run.launch.py`
- Adapter dry-run:
`python3 scripts/epd_snapshot_adapter.py --profile <scene>/config/perception_profile.yaml --input <scene>/config/sample_detected_objects.yaml --output /tmp/runtime_bridge_payload.json --summary /tmp/epd_adapter_summary.json`

## Perception Replay Preview (Offline Dry-Run)
- Uses `config/perception_profile.yaml` and `config/sample_detected_objects.yaml` to map detected_objects/v1 into task pick intent.
- Generates `selected_target_summary.json`, `runtime_bridge_payload.preview.json`, `perception_replay_markers.json`, and `perception_replay_summary.json`.
- Launch preview with `publish_perception_replay:=true` to visualize replay markers; live EPD/RealSense are not auto-launched.
- Safety limits: dry-run only, no robot motion, no MoveIt planning service calls, not a safety certificate.

```bash
python3 scripts/epd_snapshot_adapter.py \
  --profile ~/workcell_ws/src/scenes/new_scene/config/perception_profile.yaml \
  --input ~/workcell_ws/src/scenes/new_scene/config/sample_detected_objects.yaml \
  --task ~/workcell_ws/src/scenes/new_scene/task_recipe.yaml \
  --grasp ~/workcell_ws/src/scenes/new_scene/grasp_strategy.yaml \
  --environment ~/workcell_ws/src/scenes/new_scene/environment_layout.yaml \
  --output ~/workcell_ws/src/scenes/new_scene/config/runtime_bridge_payload.preview.json \
  --markers ~/workcell_ws/src/scenes/new_scene/config/perception_replay_markers.json \
  --summary ~/workcell_ws/src/scenes/new_scene/config/perception_replay_summary.json

ros2 launch new_scene demo.launch.py \
  use_fake_hardware:=true \
  publish_workcell_markers:=true \
  publish_perception_replay:=true
```
