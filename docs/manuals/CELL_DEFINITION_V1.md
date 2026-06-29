# Cell Definition v1

## What a cell definition is

A **Cell Definition** is a single high-level YAML contract (`schema_version: cell_definition/v1`) that captures robot-cell intent for commissioning and documentation workflows.

It includes:

- cell identity and description
- robot and home behavior metadata
- end-effector/tool metadata
- camera metadata
- environment and objects
- task type, destinations, and rules
- commissioning/export expectations

## Why it exists

The software supports delivery of the real product: the **physical robotic cell**.
Cell Definition v1 makes cell setup more repeatable by creating one source of truth that can be validated, preview-generated, and reviewed by operators before runtime integration.

## How this supports future click-driven generation

Cell Definition v1 is designed as a bridge to future workcell-builder integration:

1. operator/engineer defines cell metadata once
2. tooling validates contract quality offline
3. tooling generates scene/task previews
4. tooling generates operator-readable commissioning summary
5. reviewed metadata can be transformed into future click-driven builders/exporters

No GUI is introduced in this phase.

## Cell definition vs scene manifest vs task recipe

- **Cell Definition v1**: high-level commissioning contract for a whole cell.
- **Scene manifest**: lower-level scene/runtime-oriented metadata contract.
- **Task recipe**: task intent and routing rules (destinations + decision rules).

This PR adds preview generation from cell definition into scene-manifest/task-recipe shaped artifacts.

## Validate a cell definition

```bash
python3 scripts/validate_cell_definition.py tests/fixtures/cell_definition_sort_by_colour.yaml
```

Validator behavior:

- FAIL for malformed YAML and required contract violations
- WARN for non-blocking concerns (example: unknown end-effector type)
- PASS when checks are satisfied without warnings
- exit non-zero only on failures

## Generate preview files

```bash
python3 scripts/generate_scene_from_cell_definition.py \
  tests/fixtures/cell_definition_sort_by_colour.yaml \
  --output-dir /tmp/emd_cell_preview
```

Generated files:

- `scene_manifest.preview.yaml`
- `task_recipe.preview.yaml`
- `commissioning_summary.md`

If a `grasp` strategy is present and validated, preview/generated outputs also include a `grasp_strategy` metadata block in the scene manifest and task recipe, and a grasp section in summaries. This is offline metadata only and does not change runtime motion/planning behavior.

## Review commissioning summary

`commissioning_summary.md` is written for operators and includes:

- cell name/id
- robot, end-effector, camera
- task type and destinations
- object count and warnings
- next commissioning steps
- explicit limitation note

## Supported non-simple task use cases

Cell Definition v1 supports metadata for:

- `pick_place`
- `sort_by_colour`
- `sort_by_shape`
- `sort_by_class`
- `garbage_sorting`
- `inspection_then_place`
- `custom`

In this phase, task types only drive **offline preview generation** (no runtime behavior changes).

## Workflow command

```bash
./scripts/check_cell_definitions.sh
```

This runs validation and preview generation for all `tests/fixtures/cell_definition_*.yaml` files.

## Limitations

- Not a safety certificate.
- Not proof of robot reachability.
- Not proof of collision-free execution.
- Generated outputs must be engineering-reviewed before runtime use.


## Generate a workcell package from a cell definition

```bash
python3 scripts/generate_workcell_from_cell_definition.py   tests/fixtures/cell_definition_sort_by_colour.yaml   --output-dir /tmp/generated_workcells   --package-name generated_colour_sorting_cell   --force

python3 scripts/validate_scene_contract.py /tmp/generated_workcells/generated_colour_sorting_cell/scene_manifest.yaml
./scripts/check_generated_workcells.sh
```

Generated package folder (example):

- `/tmp/generated_workcells/generated_colour_sorting_cell/package.xml`
- `/tmp/generated_workcells/generated_colour_sorting_cell/CMakeLists.txt`
- `/tmp/generated_workcells/generated_colour_sorting_cell/scene_manifest.yaml`
- `/tmp/generated_workcells/generated_colour_sorting_cell/workcell.yaml`
- `/tmp/generated_workcells/generated_colour_sorting_cell/README.md`
- `/tmp/generated_workcells/generated_colour_sorting_cell/generated/commissioning_summary.md`
- `/tmp/generated_workcells/generated_colour_sorting_cell/generated/validation_report.md`

Safety note: generated package is for offline review and commissioning preparation. It is not proof of physical reachability, collision-free runtime behavior, or machine safety compliance.

## Create a cell definition with the wizard

Use the guided wizard to build valid `cell_definition/v1` YAML without hand-authoring YAML.

### Interactive mode

```bash
python3 scripts/create_cell_definition_wizard.py
```

### Non-interactive sort-by-colour

```bash
python3 scripts/create_cell_definition_wizard.py \
  --template sort_by_colour \
  --cell-name "Colour Sorting Demo" \
  --cell-id colour_sorting_demo \
  --robot ur5 \
  --end-effector robotiq_2f \
  --camera realsense_d435i \
  --output /tmp/colour_sorting_demo.cell.yaml \
  --force
```

### Generate a workcell directly from the wizard

```bash
python3 scripts/create_cell_definition_wizard.py \
  --template sort_by_shape \
  --cell-name "Shape Sorting Demo" \
  --cell-id shape_sorting_demo \
  --robot ur5 \
  --end-effector robotiq_2f \
  --camera realsense_d435i \
  --output /tmp/shape_sorting_demo.cell.yaml \
  --generate-workcell \
  --workcell-output-dir /tmp/generated_workcells \
  --package-name generated_shape_sorting_demo \
  --force
```

### Validate generated YAML and export a bundle

```bash
python3 scripts/validate_cell_definition.py /tmp/shape_sorting_demo.cell.yaml
python3 scripts/export_workcell_bundle.py generated_shape_sorting_demo --force
```

## Workcell project orchestration

A validated `cell_definition/v1` can now be used directly with:

```bash
python3 scripts/create_workcell_project.py --cell-definition <path/to/cell.yaml> --output-dir dist/workcell_projects --force
```

This orchestrates existing offline tooling (validation, package generation, direct manifest validation, dry-run, execution plan, and commissioning bundle) into one project folder and emits `project_manifest.json`.

## Optional capability references (offline)

`cell_definition/v1` now optionally supports capability IDs for robot, end effector, sensors, task, and environment assets. Existing direct fields remain valid and required; capability references only add extra offline validation and metadata propagation.

```yaml
robot:
  capability: ur5
end_effector:
  capability: robotiq_2f_85
sensors:
  - capability: intel_realsense_d435i
task:
  capability: sort_by_colour
environment:
  assets:
    - capability: table_standard_1200
    - capability: bin_blue_large
```

Validation behavior:
- By default, unresolved capability IDs are checked against `catalog/capabilities/` (shipping Workcell Studio catalog).
- `--capabilities-dir` overrides the registry path for tests or experiments (for example `tests/fixtures/capabilities`).
- Non-strict mode: unresolved capability IDs report WARN.
- Strict mode (`--strict`): unresolved capability IDs report FAIL.
- Compatibility checks are best-effort and conservative to preserve backward compatibility.

## Optional `environment.layout` reference

`cell_definition/v1` now supports an optional layout reference under `environment`:

```yaml
environment:
  layout: tests/fixtures/environment_layouts/ur5_table_bins_existing_assets.layout.yaml
```

`environment.layout` is a placement-layer input (`environment_layout/v1`) that references existing assets.
It does not duplicate assets and does not replace the existing asset folder structure.

Validation behavior:
- Missing layout path: WARN by default, FAIL in strict mode.
- Valid layout: summary included in validator JSON output.
- Backward compatibility: cell definitions without `environment.layout` remain valid.

See `docs/manuals/ENVIRONMENT_LAYOUT_V1.md`.

## Optional grasp strategy

Cell definitions may include an optional `grasp` block for metadata-only grasp intent. This is resolved from `catalog/grasp_strategies/` and does not change runtime motion, MoveIt planning, or execution behaviour yet.

Reference form:
```yaml
grasp:
  strategy_ref: suction_top_basic
```

Inline form:
```yaml
grasp:
  strategy:
    schema_version: grasp_strategy/v1
    grasp_strategy:
      id: inline_suction_top
      label: Inline suction top
      strategy: suction_top
      approach_axis: z_down
      orientation_mode: vertical
      approach_distance_m: 0.1
      retreat_distance_m: 0.1
      contact: {type: suction}
      release: {type: vacuum_off}
```


## Builder scene exports for Workcell Studio

Qt Scene3D in `workcell_builder` is an experimental/legacy Debug 3D Preview, not the source of truth. Generated scenes can now export portable Workcell Studio source files using `scripts/export_builder_scene_to_cell_definition.py`. The export writes `generated/cell_definition.yaml`, `generated/environment_layout.yaml`, and `generated/builder_export_summary.json`. Generated scene files and ROS package outputs remain the backend contract, and RViz/MoveIt remains the planning and visualization truth for simulation validation. These files are for offline commissioning and backend tooling, and are not proof of reachability or runtime safety. Keep fake-hardware-first defaults and runtime send disabled unless separately commissioned.
