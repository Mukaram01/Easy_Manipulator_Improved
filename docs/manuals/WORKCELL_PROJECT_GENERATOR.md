# Workcell Project Generator (Offline)

## Why this exists

`create_workcell_project.py` provides a one-command offline orchestration layer to generate a complete, reviewable workcell project package from either:
- an existing `cell_definition/v1` YAML, or
- a template-driven cell definition flow.

It reuses existing validators/generators and does not modify runtime robot behavior.

Generated outputs now also preserve optional validated `grasp_strategy/v1` metadata from `cell_definition/v1` into manifests, task recipes, summaries, and bundle summary JSON for offline review. This remains metadata-only and does not change runtime execution.

## Terminology

- **Template**: Preset metadata inputs used to synthesize a cell definition.
- **Cell definition**: `cell_definition/v1` YAML source of truth for offline generation.
- **Generated workcell package**: ROS 2 package produced from the cell definition (`package.xml`, `scene_manifest.yaml`, previews).
- **Workcell project**: End-to-end offline project folder with generated package, reports, bundle, README, next commands, and JSON manifest.
- **Commissioning bundle**: Handover/export artifact containing validation/dry-run/plan/checklist files.

## Quick usage

### 1) Create project from fixture

```bash
python3 scripts/create_workcell_project.py \
  --cell-definition tests/fixtures/cell_definition_sort_by_colour.yaml \
  --output-dir dist/workcell_projects \
  --force
```

### 2) Create project from template

```bash
python3 scripts/create_workcell_project.py \
  --template sort_by_colour \
  --cell-name "Colour Sorting Demo" \
  --cell-id colour_sorting_demo \
  --robot ur5 \
  --end-effector robotiq_2f \
  --camera realsense_d435i \
  --output-dir dist/workcell_projects \
  --force
```

### 3) Inspect generated files

```bash
cat dist/workcell_projects/<cell_id>/project_manifest.json
cat dist/workcell_projects/<cell_id>/reports/validation_summary.md
cat dist/workcell_projects/<cell_id>/next_commands.md
```

### 4) Copy generated package into workspace

```bash
cp -r dist/workcell_projects/<cell_id>/generated_workcell/<package_name> ~/workcell_ws/src/
```

### 5) Build and launch later

```bash
cd ~/workcell_ws
colcon build --packages-select <package_name>
source install/setup.bash
ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=<package_name>
```

## Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| `FAIL: missing cell definition file` | wrong input path | verify `--cell-definition` path |
| `invalid cell definition` | schema errors in YAML | run `python3 scripts/validate_cell_definition.py <yaml>` |
| `project already exists` | no overwrite allowed by default | rerun with `--force` |
| bundle WARN | optional bundle step warning | inspect `reports/validation_summary.md`; use `--strict` to enforce failure |
| no execution plan | dry-run non-PASS or skipped step | inspect `reports/task_recipe_dry_run.md`; use `--skip-execution-plan` intentionally if needed |

## Safety notes

- Offline generation is not a safety certificate.
- Runtime commissioning, collision review, hardware interlocks, and operator sign-off are still required.
- Generated launch example is for later staged validation after workspace build.

## Future direction

The project manifest and bundle layout are designed to support future GUI import/export workflows and deterministic commissioning handover pipelines.

## Importing a workcell_builder scene into Workcell Studio

Use Workcell Studio as an orchestration/import layer on top of `workcell_builder` exports. The builder stays the visual authoring tool; Workcell Studio consumes `generated/cell_definition.yaml` + `generated/environment_layout.yaml` and produces offline validation/demo artifacts by default. Runtime execution remains guarded and fake-hardware-first.

```bash
python3 scripts/workcell_studio.py import-builder-scene \
  --scene-package /path/to/generated_scene \
  --output-dir /tmp/workcell_studio_import_demo \
  --project-name demo_from_builder \
  --validate \
  --generate-project
```
