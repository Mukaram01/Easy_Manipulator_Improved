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
