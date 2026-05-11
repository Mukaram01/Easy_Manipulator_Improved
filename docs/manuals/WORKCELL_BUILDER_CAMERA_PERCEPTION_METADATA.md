# Workcell Builder Camera + Perception Metadata

Purpose: offline-only camera placement and perception metadata export for Workcell Scene Schema v1.

EPD remains separate/external. This feature does not launch EPD, import EPD GUI, or run runtime inference.

## Default profile
- RealSense D435i (`realsense_d435i`)
- topics: rgb/depth/camera_info/pointcloud
- frame names: `camera_link`, `camera_color_optical_frame`

## Operator flow
1. Open workcell_builder
2. Select/create scene
3. Select robot and end effector
4. Add/import/place objects
5. Open Camera / Perception
6. Select RealSense D435i
7. Apply camera defaults
8. Edit camera pose/topics
9. Validate Camera
10. Save/generate scene
11. Review readiness/schema validation
12. Launch with use_fake_hardware:=true
13. Run EPD separately when needed

## Visual layout
Camera marker appears in top-down layout with label and heading cue.

## Schema v1 integration
Camera metadata is written under `camera:` in `workcell_scene/v1`.

## EPD adapter metadata
`config/epd_adapter_metadata.json` is export metadata only (scene name, camera profile, topics, frames, notes).

## Safety note
fake-hardware-first applies; camera checks are generation-time validation only.
