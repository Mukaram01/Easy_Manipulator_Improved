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
