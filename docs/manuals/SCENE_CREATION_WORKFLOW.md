# SCENE_CREATION_WORKFLOW

This manual documents the **current intended flow** for creating/importing/editing environments using the existing repository tooling.

## Scope and boundaries

- `workcell_builder` remains the source GUI for scene creation/editing.
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
12. Validate scene readiness (`scripts/check_scene_readiness.py`).
13. Optionally generate/preview cell definition artifacts.
14. Simulate.
15. Commission/deploy in later stages.

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
