# Workcell Builder Camera Support Assets

## Summary

The `realsense2_description` package remains the existing camera package used by current projects.
This change adds reusable **physical camera support / mount** description packages so scenes can represent mounted cameras (instead of floating sensors).

## Added camera support package types

- `tslot_camera_frame_description` (`tslot_camera_frame`)
- `vslot_camera_frame_description` (`vslot_camera_frame`)
- `pipe_camera_stand_description` (`pipe_camera_stand`)
- `flat_plate_camera_bracket_description` (`flat_plate_camera_bracket`)
- `overhead_camera_gantry_description` (`overhead_camera_gantry`)
- `table_clamp_camera_mount_description` (`table_clamp_camera_mount`)

Each package follows the same ROS 2 description package structure pattern as existing environment description assets and includes:

- `CMakeLists.txt`, `package.xml`
- `launch/view_<asset>.launch.py`
- `meshes/visual/<asset>.stl`
- `meshes/collision/<asset>.stl`
- `rviz/view_<asset>.rviz`
- `urdf/<asset>.urdf.xacro`
- `urdf/test_<asset>.urdf.xacro`
- `<asset>.yaml` wrapper for object loading workflows

## View each support package

After building and sourcing:

```bash
ros2 launch tslot_camera_frame_description view_tslot_camera_frame.launch.py
ros2 launch vslot_camera_frame_description view_vslot_camera_frame.launch.py
ros2 launch pipe_camera_stand_description view_pipe_camera_stand.launch.py
ros2 launch flat_plate_camera_bracket_description view_flat_plate_camera_bracket.launch.py
ros2 launch overhead_camera_gantry_description view_overhead_camera_gantry.launch.py
ros2 launch table_clamp_camera_mount_description view_table_clamp_camera_mount.launch.py
```

## Load into Workcell Builder

1. Start `workcell_builder`.
2. Open or create a scene.
3. Click **Load Existing Object**.
4. Select one of:
   - `tslot_camera_frame`
   - `vslot_camera_frame`
   - `pipe_camera_stand`
   - `flat_plate_camera_bracket`
   - `overhead_camera_gantry`
   - `table_clamp_camera_mount`

These support assets are intended to be used alongside the existing RealSense description package.

## Out of scope for this change

Detection zones, pick/place zones, conveyor tracking, and EPD runtime behavior are intentionally not changed here and should be handled in a follow-up PR.
