# EMD Grasp Planner Point-Cloud/Grasp Visualization

This document covers **EMD grasp planner visualization** only.
It is **not** EPD CloudViewer, and does not change Workcell Studio/Builder behavior.

## Defaults and safety
- `visualization_params.point_cloud_visualization` defaults to `false`.
- Visualization is best-effort: if no display is available, planning continues.
- No robot motion, MoveIt execution, or gripper commands are issued by this visualization path.

## Parameters
```yaml
visualization_params:
  point_cloud_visualization: false
  point_cloud_visualization_backend: "pcl"
  point_cloud_visualization_blocking: false
  point_cloud_visualization_timeout_ms: 2000
  point_cloud_visualization_save_debug_pcd: false
  point_cloud_visualization_debug_dir: "/tmp/emd_grasp_visualization"
```

## Enabling in 2F config
Edit `params_2f.yaml` and set:
```yaml
visualization_params:
  point_cloud_visualization: true
  point_cloud_visualization_blocking: false
```

## Debug launch
Use:
```bash
ros2 launch run_grasp_planner grasp_planner_2f_visual_debug.launch.py
```

## Headless-safe artifact mode
If `DISPLAY`/`WAYLAND_DISPLAY` are unavailable and `point_cloud_visualization_save_debug_pcd: true`, debug artifacts are saved under `point_cloud_visualization_debug_dir`:
- `object_cloud_<timestamp>.pcd`
- `scene_cloud_<timestamp>.pcd`
- `grasp_debug_summary_<timestamp>.yaml`

## Common failure causes
- Missing `DISPLAY` (for example SSH without X11 forwarding)
- Wayland/Qt/OpenGL incompatibility
- Empty object point cloud
- Invalid RealSense point-cloud topic
- Camera frame/TF mismatch

## Relation to Workcell Studio
Workcell Studio uses readiness/previews. This viewer remains a planner-side debug utility only.
