# SCENE_CREATION_WORKFLOW

This manual documents the **current intended flow** for creating/importing/editing environments using the existing repository tooling.

## Scope and boundaries

- `workcell_builder` remains the source GUI for scene creation/editing and task authoring (`pick_place`).
- `scenes/` stores scene packages/files.
- `assets/` stores physical meshes/models/descriptions.
- `environment_layout/v1` is metadata/placement helper only.
- `cell_definition/v1` describes intended cell/task contract.
- No duplicate scene system is required.

## Practical workflow (current)

1. Open or create a workcell project.
2. Use existing `workcell_builder` GUI.
3. Use the existing `scenes/` directory structure.
4. Import STL/mesh through existing builder flow.
5. Add visual geometry for each imported object.
6. Add collision geometry (required for practical planning safety).
7. Place assets in `world` frame (or intended parent link frame).
8. Add robot package/model to the scene.
9. Add gripper/end effector configuration.
10. Add camera/sensor frame and model references.
11. Save scene package.
12. Author task intent in `workcell_builder`: select pick source, place target, grasp/release strategy, and routing rule.
13. Export generated task intent (`generated/workcell_builder_task_intent.yaml`) from builder-authored selections.
14. Validate scene readiness (`scripts/check_scene_readiness.py`).
15. Optionally generate/preview cell definition artifacts.
16. Simulate.
17. Commission/deploy in later stages.

## Relationship between metadata layers

- `environment_layout/v1`: placement/zones checklist metadata for operators.
- `cell_definition/v1`: high-level cell intent (robot/tool/sensor/task).
- `task_recipe/v1`: offline task logic intent (`pick_place`, `sort_by_colour`, `garbage_sorting`, etc.).
- Scene package (`scenes/<name>`): concrete files used by existing workflow and validation.

## Helper commands

```bash
python3 scripts/report_workcell_builder_paths.py
python3 scripts/check_scene_readiness.py --workcell-root .
python3 scripts/generate_scene_import_checklist.py path/to/fixture.stl
python3 scripts/environment_layout_to_scene_checklist.py tests/fixtures/environment_layouts/ur5_table_bins_existing_assets.layout.yaml
```

## Create your own environment using cell definitions

Use the existing Workcell Builder + generation flow (no duplicate builder/system).

1. Start from `cell_definitions/demo_ur5_sorting_cell.yaml`.
2. Edit robot/base scene metadata, table/bin/box/camera frames, and task destinations/rules.
3. Validate:
   - `python3 scripts/validate_cell_definition.py --cell-definition cell_definitions/demo_ur5_sorting_cell.yaml --json`
4. Generate staged workcell package:
   - `python3 scripts/generate_workcell_from_cell_definition.py --cell-definition cell_definitions/demo_ur5_sorting_cell.yaml --output-dir /tmp/generated_workcells/demo_ur5_sorting_cell --json`
5. Run preflight and gated dry-run with generated artifacts (no robot motion required).
6. Only after offline checks pass, continue with simulation/replay commissioning steps.

## Do not do this

- Do **not** copy the same mesh into multiple folders unnecessarily.
- Do **not** create duplicate scene package names.
- Do **not** hardcode absolute `/home/user/...` paths into reusable scenes.
- Do **not** create a new top-level scene format outside existing `workcell_builder`.
- Do **not** bypass collision geometry.

## Runtime boundary

This workflow/manual improves authoring readiness only.

It does not change runtime ROS 2, MoveIt, grasp, perception, or controller behavior.

## Generate and test your own workcell bundle
Use `generate_workcell_from_cell_definition.py` then run `run_generated_workcell_bundle.py --gated-dry-run --json` to execute preflight + gated dry-run with generated defaults only; robot motion remains disabled.

## Visual preview of a generated workcell

Use `python3 scripts/preview_generated_workcell_bundle.py --workcell <path> --json` to produce `generated/visual_preview_summary.json`, then optionally publish markers to `/generated_workcell/markers` for RViz checks of tables, bins, objects, destinations, and task pick/release highlights. Motion remains disabled.

## Visual task-flow preview
1. Generate the workcell bundle.
2. Run gated dry-run (`run_generated_workcell_bundle.py --gated-dry-run --json`).
3. Open visual preview (`preview_generated_workcell_bundle.py --show-task-flow --task-flow-preview <dry-run-output>/task_flow_preview.json --json`).
4. Review selected object, pick pose, destination, and release pose in the trace.
5. Check warnings/blockers before any future execution workflow.
6. This is dry-run intent preview only (`safe_for_robot_motion=false`) and does not move the robot.
7. Future milestone: MoveIt simulation trajectory preview.


## Studio Lite

Studio Lite (`scripts/studio_lite.py`) is the first operator workflow window for generated bundles. It reuses existing scripts and does not replace Workcell Builder.

- Uses `validate_cell_definition.py` and `generate_workcell_from_cell_definition.py` for generation.
- Uses `preview_generated_workcell_bundle.py` for JSON and marker preview.
- Uses `run_generated_workcell_bundle.py --gated-dry-run --json` for safe dry-run checks.
- Shows report paths, blockers, warnings, and selected object/destination fields.
- Keeps robot motion disabled (`safe_for_robot_motion: false`).

Use Studio Lite for generate/preview/preflight/report inspection while backend authoring stays in Workcell Builder.


## Developer Studio vs Operator Runtime

- **Developer Studio (developer/integrator role):** validate cell definitions, generate workcell bundles, preview environment/task flow, run preflight and gated dry-runs, inspect reports.
- **Operator Runtime (operator role):** load generated bundles (including approved bundles), view summaries, preview environment/task flow, run preflight and gated dry-runs, inspect warnings/blockers/reports.
- Operator Runtime does **not** expose editing or generation controls and never enables robot motion/replay/controller publishing.
- Both roles use the same generated bundle format and the same backend scripts (no duplicate scene system, no second planning backend).
- Bundle approval is an internal workflow marker only, not a safety certification. Physical robot execution remains disabled.

## Scenario packs
See `docs/manuals/INDUSTRIAL_SCENARIO_PACKS.md` for generated-cell scenario validation flows.


## Persisting placed objects to real generated scenes
1. Add/import STL object.
2. Open STL preview.
3. Open interactive RViz preview.
4. Drag/rotate object.
5. Import RViz Pose Feedback.
6. Review/apply valid updates.
7. **Save Placed Objects to Scene YAML**.
8. Generate YAML / Generate Files.
9. Build scene package.
10. Launch generated scene in RViz/MoveIt fake hardware mode.

Preview files are temporary. `environment.yaml` is the scene persistence point. Real scene files are updated only by explicit save/generate actions. This workflow introduces no controller execution and no real hardware execution.
