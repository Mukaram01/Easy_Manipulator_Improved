# TASK_RECIPE_V1

`task_recipe/v1` is an **offline schema** describing industrial task intent and decision logic.

It is:
- not runtime ROS/MoveIt/grasp/perception/controller code,
- not a replacement for runtime motion execution,
- a bridge between user task selection and future generated runtime behaviour.

## Perception input compatibility

`scripts/run_task_recipe_adapter.py` accepts:
- legacy runtime object fixtures (`tests/fixtures/runtime_objects`), and
- real/recorded `detected_objects/v1` snapshots (`tests/fixtures/detected_objects`).

For production direction, `detected_objects/v1` is the first-class input, while mock fixtures remain test-only.

## Supported task types

- `pick_place`
- `sort_by_colour`
- `sort_by_shape`
- `sort_by_class`
- `garbage_sorting`
- `inspection_then_place`
- `custom`

## Required concepts

- source object or perception source
- object attributes: `colour`, `shape`, `class`, `material`, `confidence`, `inspection_result`
- destinations with:
  - `id`
  - `frame`
  - `pose_xyz`
  - `pose_rpy`
  - `bin` / `type` / `label`
- decision rules:
  - attribute equals value
  - confidence threshold routing (`confidence_below`)
  - fallback/default rule
  - reject/unknown destination routing

## Safety boundary

Task recipe metadata does **not** prove:
- reachability,
- collision safety,
- real I/O safety,
- production commissioning readiness.

Commissioning validation is still required.

## Schema sketch

```yaml
schema_version: task_recipe/v1
task:
  id: colour_sort_task
  type: sort_by_colour
  source: detected_object
  perception_source: overhead_rgbd
  object_attributes: [colour, shape, class, material, confidence, inspection_result]
  destinations: []
  decision_rules: []
```

## Example A: simple pick and place

```yaml
schema_version: task_recipe/v1
task:
  id: pick_place_demo
  type: pick_place
  source: source_part
  destinations:
    - id: place_bin
      frame: world
      pose_xyz: [0.35, -0.20, 0.10]
      pose_rpy: [0.0, 0.0, 0.0]
      bin: place
  decision_rules:
    - id: default_place
      when: {default: true}
      destination: place_bin
```

## Example B: sort red/blue/green by colour

```yaml
schema_version: task_recipe/v1
task:
  id: colour_sort_rgb
  type: sort_by_colour
  source: detected_object
  perception_source: overhead_rgbd
  destinations:
    - {id: red_bin, frame: world, pose_xyz: [0.30, -0.30, 0.10], pose_rpy: [0, 0, 0], label: red}
    - {id: blue_bin, frame: world, pose_xyz: [0.30, 0.00, 0.10], pose_rpy: [0, 0, 0], label: blue}
    - {id: green_bin, frame: world, pose_xyz: [0.30, 0.30, 0.10], pose_rpy: [0, 0, 0], label: green}
    - {id: unknown_reject_bin, frame: world, pose_xyz: [0.15, 0.00, 0.10], pose_rpy: [0, 0, 0], label: reject}
  decision_rules:
    - {id: to_red, when: {attribute: colour, equals: red}, destination: red_bin}
    - {id: to_blue, when: {attribute: colour, equals: blue}, destination: blue_bin}
    - {id: to_green, when: {attribute: colour, equals: green}, destination: green_bin}
    - {id: fallback_reject, when: {default: true}, destination: unknown_reject_bin}
```

## Example C: sort by shape

```yaml
schema_version: task_recipe/v1
task:
  id: shape_sort
  type: sort_by_shape
  source: detected_object
  destinations:
    - {id: box_bin, frame: world, pose_xyz: [0.35, -0.20, 0.10], pose_rpy: [0, 0, 0], type: shape}
    - {id: cylinder_bin, frame: world, pose_xyz: [0.35, 0.20, 0.10], pose_rpy: [0, 0, 0], type: shape}
    - {id: reject_bin, frame: world, pose_xyz: [0.15, 0.00, 0.10], pose_rpy: [0, 0, 0], label: reject}
  decision_rules:
    - {id: to_box, when: {attribute: shape, equals: box}, destination: box_bin}
    - {id: to_cylinder, when: {attribute: shape, equals: cylinder}, destination: cylinder_bin}
    - {id: fallback_reject, when: {default: true}, destination: reject_bin}
```

## Example D: garbage sorting plastic/metal/paper/unknown

```yaml
schema_version: task_recipe/v1
task:
  id: garbage_sort
  type: garbage_sorting
  source: waste_item
  destinations:
    - {id: plastic_bin, frame: world, pose_xyz: [0.45, -0.30, 0.10], pose_rpy: [0, 0, 0], type: plastic}
    - {id: metal_bin, frame: world, pose_xyz: [0.45, -0.10, 0.10], pose_rpy: [0, 0, 0], type: metal}
    - {id: paper_bin, frame: world, pose_xyz: [0.45, 0.10, 0.10], pose_rpy: [0, 0, 0], type: paper}
    - {id: unknown_reject_bin, frame: world, pose_xyz: [0.45, 0.30, 0.10], pose_rpy: [0, 0, 0], label: reject}
  decision_rules:
    - {id: to_plastic, when: {attribute: material, equals: plastic}, destination: plastic_bin}
    - {id: to_metal, when: {attribute: material, equals: metal}, destination: metal_bin}
    - {id: to_paper, when: {attribute: material, equals: paper}, destination: paper_bin}
    - {id: low_confidence_reject, when: {confidence_below: 0.50}, destination: unknown_reject_bin}
    - {id: fallback_reject, when: {default: true}, destination: unknown_reject_bin}
```

## Example E: inspection then place pass/fail

```yaml
schema_version: task_recipe/v1
task:
  id: inspect_then_place
  type: inspection_then_place
  source: inspected_part
  destinations:
    - {id: pass_bin, frame: world, pose_xyz: [0.30, -0.10, 0.10], pose_rpy: [0, 0, 0], label: pass}
    - {id: fail_bin, frame: world, pose_xyz: [0.30, 0.10, 0.10], pose_rpy: [0, 0, 0], label: fail}
  decision_rules:
    - {id: pass_route, when: {attribute: inspection_result, equals: pass}, destination: pass_bin}
    - {id: fail_route, when: {attribute: inspection_result, equals: fail}, destination: fail_bin}
    - {id: fallback_fail, when: {default: true}, destination: fail_bin}
```

## Tools

```bash
python3 scripts/validate_task_recipe.py tests/fixtures/task_recipes
python3 scripts/validate_task_recipe.py tests/fixtures/task_recipes --json
python3 scripts/generate_task_recipe_from_cell_definition.py tests/fixtures/cell_definition_sort_by_colour.yaml --output /tmp/task_recipe.preview.yaml
```
