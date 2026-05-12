# Workcell Builder Environment Asset Style

Environment asset packages under `assets/environment` should follow the same style as `realsense2_description`:

- `CMakeLists.txt`
- `package.xml`
- `launch/`
- `meshes/` (prefer `meshes/visual` and `meshes/collision`)
- `rviz/`
- `urdf/`

## Why this style
This layout is proven to be discoverable, buildable, and previewable with ROS 2 tooling and Workcell Builder.

## Environment vs robot/gripper assets
This convention is for **environment assets only**. Robot and end-effector MoveIt packages are intentionally out-of-scope.

## Add a new environment asset
1. Create `assets/environment/<asset_name>_description`.
2. Add package files (`package.xml`, `CMakeLists.txt`) with `ament_cmake`.
3. Add `urdf/<asset>.urdf.xacro` and `urdf/test_<asset>.urdf.xacro`.
4. Add `meshes/visual` and `meshes/collision`.
5. Add `launch/view_<asset>.launch.py` and `rviz/view_<asset>.rviz`.
6. If it should show in **Load Existing Object**, add `<asset>.yaml` wrapper with link/joint/visual/collision references.

## Checker
Run:

```bash
python3 scripts/check_environment_asset_style.py --root assets/environment --strict
python3 scripts/check_environment_asset_style.py --root assets/environment --json-out /tmp/environment_asset_style_report.json
```

The checker prints PASS/WARN/FAIL per environment folder and returns non-zero in strict mode when required layout checks fail.
