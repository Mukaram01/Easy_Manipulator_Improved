# Workcell Studio: New Cell from Scratch Flow (Audit Map)

## End-to-end UI flow
1. **Dashboard → New Cell/New Scene**
   - Owner: `SceneSelect::on_add_scene_clicked`
   - Output: new scene folder under configured scenes root.
   - Next: Scene Builder.
   - Failure: invalid/duplicate name, missing workspace/scenes root.

2. **Use Recommended Layout**
   - Owner: `SceneSelect::on_use_recommended_layout_clicked`
   - Output: `config/recommended_layout.yaml`, starter layout intent.
   - Next: open/edit scene canvas.

3. **Add to Canvas / Edit on Canvas / Save Layout**
   - Owner: `MainWindow::add_asset_to_canvas_from_catalog`, `MainWindow::save_layout_changes`
   - Output: `environment_layout.yaml` updates.
   - Next: task intent step.
   - Failure: unsaved/invalid layout -> prompt and log guidance.

4. **Generate/Update Task Intent**
   - Owner: `MainWindow::generate_or_update_task_intent_for_selected_scene`
   - Output: `config/workcell_builder_task_intent.yaml`.
   - Next: generate scene/package files.
   - Failure: missing helper script -> searched paths are logged.

5. **Generate Files / Generate Scene Package**
   - Owner: `SceneSelect::on_generate_files_clicked` and command-bar Generate Scene action.
   - Output: launch/config/package artifacts (`launch/demo.launch.py`, package files, scene outputs).
   - Next: validation.

6. **Run Offline Validation**
   - Owner: `MainWindow::run_offline_validation`, `SceneSelect::on_validate_scene_button_clicked`
   - Output: validation reports/dashboard files.
   - Next: Plan & Simulate.

7. **Plan & Simulate (Open RViz2 / MoveIt, Run Fake-Hardware Simulation)**
   - Owner: `MainWindow::run_preview_build`, `MainWindow::run_fake_hardware_preview`
   - Command contract: `ros2 launch <scene> demo.launch.py use_fake_hardware:=true launch_rviz:=true`.
   - Failure: safety-blocking checks and clear blocker messages.

8. **Refresh Existing Scenes**
   - Owner: `SceneSelect::on_refresh_scenes_button_clicked`, browser refresh actions.
   - Output: scene appears in Existing Scenes list and status views.

## Expected files in new scene
- `environment.yaml`
- `environment_layout.yaml`
- `config/workcell_builder_task_intent.yaml`
- `cell_definition.yaml` (if supported by scripts/workflow)
- `scene_manifest.yaml` (if used)
- `launch/demo.launch.py` after generation

## New-cell defaults
- Robot: `UR5`
- End effector: `Robotiq 2F`
- Environment: `table + pick zone + place zone + camera`
- Task: `pick_place`
- Launch defaults: `use_fake_hardware:=true`, `launch_rviz:=true`

## Failure states and recovery
- **Scene already exists**: create safe suffix name and continue.
- **Invalid scene name**: reject and ask for lowercase alnum/underscore.
- **Missing scripts**: show searched script paths and next fix.
- **Missing assets/layout blockers**: show blocker + recommended fix action.
- **Missing workspace/scenes root**: no crash; prompt to choose scenes folder/workspace.

## Scratch Cell Acceptance Test

Use the acceptance generator to create a default scratch scene package using UR5 + Robotiq 2F + pick_place with fake hardware defaults.

```bash
python3 scripts/generate_scratch_cell_acceptance.py \
  --scene-name scratch_ur5_2f_acceptance \
  --output-root /tmp/workcell_studio_scratch_acceptance
```

Generated outputs include:
- `cell_definition.yaml`
- `environment_layout.yaml`
- `environment.yaml` (legacy compatibility)
- `config/workcell_builder_task_intent.yaml`
- `scene_manifest.yaml`
- `package.xml`
- `CMakeLists.txt`
- `launch/demo.launch.py`
- `urdf/environment.urdf.xacro` or equivalent URDF/Xacro entrypoint

Validation command:

```bash
python3 scripts/validate_cell_definition.py <scene_dir>/cell_definition.yaml
python3 scripts/validate_environment_layout.py <scene_dir>/environment_layout.yaml
```

Build and launch contract:

```bash
colcon build --symlink-install --packages-select scratch_ur5_2f_acceptance
source install/setup.bash
ros2 launch scratch_ur5_2f_acceptance demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

Known limitations:
- Offline generation and validation only; it does not guarantee real robot execution readiness.
- RViz/MoveIt launch is not required during automated tests.
- Missing UR5/Robotiq assets are reported as blockers with searched paths.


## Point 2: File-output Audit

Run a dedicated audit after `Generate Scene Package` and before Plan & Simulate:

```bash
python3 scripts/audit_new_cell_file_outputs.py \
  --scene-dir /tmp/workcell_studio_file_audit/scratch_ur5_2f_file_audit \
  --scene-name scratch_ur5_2f_file_audit \
  --json-out /tmp/workcell_studio_file_audit/file_output_audit.json
```

The audit checks expected file locations and minimum content coherence for:
- metadata files: `environment.yaml`, `environment_layout.yaml`, `scene_manifest.yaml`, `cell_definition.yaml`, `config/workcell_builder_task_intent.yaml`
- generated package files: `package.xml`, `CMakeLists.txt`, `launch/demo.launch.py`, and URDF/Xacro entrypoints where present
- task/layout/package cross-references (scene-name, pick/place IDs, and launch package target)

Status meanings:
- `PASS`: required files exist, required tokens are present, and no blockers were found.
- `WARNINGS`: no hard blockers, but cross-reference or metadata quality warnings were found.
- `BLOCKED`: missing required files, malformed required content, or scene/package mismatch that blocks readiness.

## Point 3: State-transition Audit

State names:
- `NO_WORKSPACE`
- `WORKSPACE_READY`
- `CELL_DRAFT_CREATED`
- `LAYOUT_CREATED`
- `LAYOUT_SAVED`
- `TASK_INTENT_CREATED`
- `SCENE_PACKAGE_GENERATED`
- `FILE_OUTPUTS_CHECKED`
- `VALIDATION_READY`
- `VALIDATION_PASSED`
- `VALIDATION_BLOCKED`
- `PLAN_SIMULATE_READY`
- `SIMULATION_RUNNING`
- `SIMULATION_STOPPED`

Required files/conditions by progression:
- Layout milestone: `environment_layout.yaml`
- Task intent milestone: `config/workcell_builder_task_intent.yaml`
- Scene package milestone: `package.xml`, `CMakeLists.txt`, `launch/demo.launch.py`
- File-output audit milestone: `file_output_audit.json`
- Validation milestone: `smoke/offline_smoke_report.json`
- Simulation milestone: fake-hardware process running/stopped only

Allowed next actions:
- `Save Layout`
- `Generate/Update Task Intent`
- `Generate Scene Package`
- `Run Offline Validation`
- `Open Plan & Simulate`

Blocked conditions and recovery:
- Missing `environment_layout.yaml` → recover with `Save Layout`.
- Missing `config/workcell_builder_task_intent.yaml` → recover with `Generate/Update Task Intent`.
- Missing package outputs (`package.xml`, `CMakeLists.txt`, `launch/demo.launch.py`) → recover with `Generate Scene Package`.
- Plan & Simulate remains blocked until launch/package outputs exist.

Audit script:
```bash
python3 scripts/audit_new_cell_state_transitions.py \
  --scene-dir <path> \
  --scene-name <name> \
  --json-out <path>
```


## Point 4: Error-message Audit

Standard message format fields:
- code
- severity (ERROR/BLOCKER/WARNING/INFO)
- title
- detail
- why_it_matters
- next_action
- recovery_command
- related_file
- related_page

Common blockers include missing workspace, invalid scene name, missing layout, missing task intent, missing package files, missing launch file, missing fake-hardware arg, and validation blocked.

Interpretation:
- BLOCKED: Must resolve blocker before Plan & Simulate.
- WARNINGS: Flow can continue, but quality/safety issues should be fixed.

Good message example:
- Missing launch/demo.launch.py. Generate Scene Package before opening Plan & Simulate.
- Task intent references pick_zone_01, but that id was not found in environment_layout.yaml. Open Scene Builder and bind a pick zone.
