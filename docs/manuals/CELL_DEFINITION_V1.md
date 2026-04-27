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
