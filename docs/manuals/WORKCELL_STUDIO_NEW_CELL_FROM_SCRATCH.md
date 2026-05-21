# Workcell Studio New Cell from Scratch

This guide documents the **existing** Workcell Studio New Cell flow. It does not introduce a parallel generator path.

## Guaranteed flow stages (existing New Cell tab)
Workspace -> New Cell -> Layout -> Task Intent -> Generate Scene Package -> Validate -> Plan & Simulate.

## What the existing New Cell flow now guarantees
- Step-by-step guidance for Cell Basics, Layout, Task Intent, and Generate/Validate/Plan.
- Manual XYZ/RPY editing for robot base, tool mount, environment placement, pick/place zones, and camera pose.
- Safe defaults for blank numeric fields.
- Fake-hardware-first outputs and simulation-safe launch guidance.

## Required steps
1. Choose workspace and create/select New Cell.
2. Enter Cell Basics: scene name, robot, tool/end-effector, base/tool poses.
3. Build Layout: environment/table/workbench/bin/fixture and pick/place zones.
4. Set Task Intent and grasp strategy.
5. Generate Scene Package.
6. Validate.
7. Open Plan & Simulate (fake hardware).

## Optional advanced layout fields
- Camera pose (XYZ/RPY) and camera FOV metadata.
- Conveyor placeholder/spawn-line metadata for conveyor-style flows.
- Primitive fallback when optional environment asset meshes are missing.

## Pick/place zones
Pick and place zones are first-class layout metadata used by task intent generation, validation checks, and downstream dry-run planning artifacts.

## Camera pose/FOV
When camera fields are configured, New Cell stores camera pose plus FOV/frustum metadata for preview and adapter metadata handoff.

## Conveyor placeholder metadata
When conveyor options are configured, New Cell includes conveyor placeholder/spawn-line metadata for preview and intent scaffolding.

## Primitive fallback behavior
- Optional missing mesh/asset: WARN + primitive fallback.
- Missing required robot/tool package: FAIL with clear blocker.

## Fake-hardware-first safety note
Generated New Cell scenes are simulation/fake-hardware-first and **must not be treated as real-hardware approval**.

## Generated files (scene contract)
- `environment.yaml`
- `cell_definition.yaml`
- `scene_manifest.yaml`
- `layout/workcell_studio_layout.yaml`
- `task/workcell_builder_task_intent.yaml`
- `task/task_recipe_from_builder_intent.yaml` (when supported)
- `plan_preview/offline_plan_preview_request.yaml` (when supported)
- scene visual metadata for existing visual mesh index path
- `launch/demo.launch.py` (or explicit blocker)

## Button and action map
Choose Workspace → New Cell → New Scene → Use Recommended Layout → Add to Canvas → Save Layout → Remove Selected Layout Item → Validate Layout → Generate/Update Task Intent → Open Task File → Copy Task Summary → Generate Scene Package → Refresh Existing Scenes → Run Offline Validation → Generate Readiness Pack → Open Readiness Dashboard → Open Plan & Simulate → Open RViz2 / MoveIt → Run Fake-Hardware Simulation → Stop Simulation → Copy Launch Command.

## Recommended manual validation command
```bash
python3 scripts/run_workcell_studio_scene_readiness_gate.py --dry-run-launches
```


## Runtime generation behavior

The **existing New Cell flow** now drives runtime scene-contract generation directly from UI state (no parallel generator path).

- **Use Recommended Layout** populates starter environment/layout metadata (workbench or primitive fallback, pick/place zones, robot base marker, camera marker/FOV when configured, and conveyor placeholder/spawn line when configured in the existing layout step).
- **Save Layout / Generate Task Intent / Generate Scene Package** update the scene contract files used by Workcell Studio, including:
  - `environment.yaml`
  - `cell_definition.yaml`
  - `scene_manifest.yaml`
  - `layout/workcell_studio_layout.yaml`
  - `task/workcell_builder_task_intent.yaml`
  - `task/task_recipe_from_builder_intent.yaml` (where supported)
  - `plan_preview/offline_plan_preview_request.yaml` (where supported)
  - `launch/demo.launch.py` (or explicit blockers)
- Primitive fallback behavior is recorded as warning metadata so preview stays usable even when optional meshes are unavailable.
- Validation path remains the same existing readiness workflow. Recommended command:
  - `python3 scripts/run_workcell_studio_scene_readiness_gate.py --dry-run-launches`

> Safety policy: generated New Cell scenes are simulation/fake-hardware-first scaffolds and are **not** real-hardware execution approval.

## Existing New Cell action wiring

The existing **New Cell** flow now wires runtime actions directly:
- **Use Recommended Layout** populates starter layout items from current preview metadata.
- **Save Layout** writes both `layout/workcell_studio_layout.yaml` and `environment.yaml` metadata (including pick/place task zones).
- **Generate/Update Task Intent** writes task-intent artifacts under `task/` and creates an offline plan preview request.
- The existing canvas refresh path reads saved layout metadata so editable items show up without manual YAML repair.

This remains the existing New Cell workflow and does **not** introduce a separate generator workflow.

## 3D canvas behavior

- The existing Scene Builder canvas panel now renders saved `layout/workcell_studio_layout.yaml` metadata in the same in-app canvas area using the 3D preview path.
- Mouse controls in 3D mode:
  - Left drag: orbit
  - Shift + left drag (or middle drag): pan
  - Mouse wheel: zoom
- The viewport shows world grid and XYZ orientation cues and uses perspective/depth so occlusion is visually correct.
- Editable layout items (from layout metadata) are visually separated from locked/generated preview items (for example URDF/mesh-index sourced visuals).
- When mesh files are missing or unsupported, preview logs warnings and uses primitive fallback geometry so canvas inspection remains usable.
- Camera FOV and conveyor metadata are represented with placeholder/frustum/primitive visuals when available.
- This canvas is for layout/preview only: it is not physics simulation and not real-hardware approval.

## 3D selection and transform editing

- Click an item in the existing Scene3D canvas to select it and highlight it.
- The existing inspector/details area shows ID, type, source, editable/locked status, XYZ/RPY, and dimensions where applicable.
- Editable layout items can be changed via inspector XYZ/RPY and dimensions fields.
- Locked/generated preview items remain selectable for inspection but are read-only in the inspector.
- Use the existing **Save Layout** action to persist edits to `layout/workcell_studio_layout.yaml`.
- Pick/place zone edits are propagated to `environment.yaml` task-zone metadata.
- This remains preview/layout editing only (no physics and no real-hardware approval).
