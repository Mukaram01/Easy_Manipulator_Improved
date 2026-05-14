# Workcell Studio Generated Scene Acceptance

This preflight validates template-generated scene packages before local build/launch preflight. It is offline/static and **does not execute ROS launch** and **does not command robot motion**.

## Checks
- Required files: `package.xml`, `CMakeLists.txt`, `environment.yaml`, `scene_manifest.yaml`, `config/task_recipe.yaml`, `config/workcell_builder_task_intent.yaml`
- Preview/smoke artifacts and generated summary
- ROS package name validity (scene directory)
- Safety flags: `fake_hardware_first=true`, `runtime_execution_enabled=false`, `motion_command_sent=false`
- Launch contract checks for `launch/demo.launch.py`
- Static xacro/URDF markers where available, including gripper mount RPY `-1.5708 -1.5708 0`

## Statuses
- `PASS`: complete and consistent.
- `WARNINGS`: usable, but non-blocking issues exist.
- `BLOCKED`: critical acceptance failures.
- `PREVIEW_ONLY`: preview artifacts exist but launch/runtime package is intentionally incomplete.

## Outputs
- `acceptance/generated_scene_acceptance.json`
- `acceptance/generated_scene_acceptance.html`
- `acceptance/generated_scene_acceptance_summary.txt`

## Run
```bash
python3 scripts/validate_workcell_studio_generated_scene.py <scene_dir> --json
```

## Manual preflight commands (emitted only; not executed)
```bash
colcon build --symlink-install --packages-select <scene_name>
source install/setup.bash
ros2 launch <scene_name> demo.launch.py use_fake_hardware:=true
```

## Relation to offline smoke check
- Offline smoke check confirms smoke/readiness artifacts from generation.
- Generated-scene acceptance focuses on package completeness + launch/safety contract compliance.
