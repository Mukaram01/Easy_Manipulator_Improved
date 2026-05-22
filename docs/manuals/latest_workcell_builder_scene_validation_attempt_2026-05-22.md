# Workcell Builder scene validation attempt (2026-05-22)

## Commands requested
- `colcon build --symlink-install --packages-select workcell_builder`
- `source install/setup.bash`
- Scene runtime acceptance + scene contract checks for:
  - `ur5_2f_test`
  - `ur5_2f_sorting_test`
- GUI smoke screenshots:
  - `build/workcell_studio/scene3d_gui_smoke_ur5_2f_test.png`
  - `build/workcell_studio/scene3d_gui_smoke_ur5_2f_sorting_test.png`
  - `build/workcell_studio/scene3d_gui_smoke_new_cell.png` (if supported)

## Environment blockers
1. `colcon` is not installed in this runner (`/bin/bash: colcon: command not found`).
2. `install/setup.bash` is not present because build did not run.
3. `workcell_builder` executable cannot be resolved by GUI smoke (`status=FAIL; blockers=[unable to resolve workcell_builder executable: checked install/workcell_builder first, then PATH]`).
4. ROS package discovery is unavailable for contract checks (`RESULT: SKIP` with E1 requesting sourced ROS + workspace overlays).

## Produced artifacts
- Runtime acceptance summary was still generated and captured missing smoke evidence:
  - `build/workcell_studio/scene3d_runtime_acceptance.json`
  - `build/workcell_studio/scene3d_runtime_acceptance.md`

## Requested screenshot paths
The following paths are the expected reviewer targets, but screenshots were **not** generated in this environment due blockers above:
- `build/workcell_studio/scene3d_gui_smoke_ur5_2f_test.png`
- `build/workcell_studio/scene3d_gui_smoke_ur5_2f_sorting_test.png`
- `build/workcell_studio/scene3d_gui_smoke_new_cell.png`

## Smoke JSON snippets
### `build/workcell_studio/scene3d_runtime_acceptance.json`
```json
{
  "schema": "workcell_studio_scene3d_runtime_acceptance/v1",
  "scenes": [
    {
      "scene": "ur5_2f_test",
      "blockers": [
        "missing runtime smoke evidence JSON: /workspace/Easy_Manipulator_Improved/build/workcell_studio/scene3d_gui_smoke_ur5_2f_test.json"
      ]
    }
  ]
}
```
