# Workcell Task Recipe RViz Preview

Offline dry-run preview visualizer for generated task_recipe.yaml.

- Publishes passive markers on `/workcell_studio/task_plan_markers`.
- Writes `task_plan_preview.json` and `task_plan_preview.md`.
- **Does not** command robot motion, call MoveIt planning, or enable real hardware.

Launch args: `launch_task_preview`, `task_recipe_path`, `task_preview_output_dir`, `task_preview_markers`, `launch_rviz`, `use_fake_hardware`.

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch <scene_name> demo.launch.py \
  use_fake_hardware:=true \
  launch_rviz:=true \
  launch_task_preview:=true
```

Headless:

```bash
ros2 launch <scene_name> demo.launch.py \
  use_fake_hardware:=true \
  launch_rviz:=false \
  launch_task_preview:=true
```
