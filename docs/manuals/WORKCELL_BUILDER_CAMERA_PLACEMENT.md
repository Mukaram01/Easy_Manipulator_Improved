# Workcell Builder Camera Placement

Workcell Builder now supports camera placement metadata using `realsense2_description` models (d415, d435, d435i, r410, r430).

## Metadata
`environment.yaml` supports:
- `cameras[].package: realsense2_description`
- `cameras[].model`
- `cameras[].xacro`
- `cameras[].parent_object`
- `cameras[].parent_link`
- `cameras[].pose.xyz` and `cameras[].pose.rpy`
- `cameras[].optical_frame` and `cameras[].depth_frame`
- `cameras[].role`
- `cameras[].runtime_driver: metadata_only`

This metadata only attaches camera models in generated URDF/Xacro and does not start RealSense hardware drivers.

## Work zones metadata preview
Detection/pick/place zones and conveyor_flow metadata are preserved in environment YAML and scene bundles. This is metadata/preview only (not safety-certified). Conveyor runtime tracking and robot wait logic are future work.
