# DETECTED_OBJECTS_V1

`detected_objects/v1` is the production-facing perception snapshot contract used between D435i/EPD perception and the task recipe adapter.

## Intent

- **Primary path**: live Intel RealSense D435i + EPD localization/tracking output.
- **Secondary path**: offline replay from saved snapshots.
- **Test-only path**: mock objects in `tests/fixtures/runtime_objects` (CI/offline tests only).

## Schema (summary)

```yaml
schema_version: detected_objects/v1
source:
  type: epd_localization
  topic: /easy_perception_deployment/epd_localize_output
  camera: intel_realsense_d435i
  frame_id: camera_depth_optical_frame
  captured_at: 2026-04-28T00:00:00Z
objects:
  - object_id: obj_001
    name: mouse
    class_id: mouse
    confidence: 0.92
    pose:
      frame_id: camera_depth_optical_frame
      xyz: [-0.56, 0.09, 0.13]
      rpy: [0.0, 0.0, 0.0]
    centroid: {x: -0.56, y: 0.09, z: 0.13}
    dimensions: {x: 0.08, y: 0.04, z: 0.03}
    shape: {type: box}
    attributes: {colour: unknown, shape: box, material: unknown}
    raw: {source_message_type: epd_msgs/msg/EPDObjectLocalization}
```

## Validation

```bash
python3 scripts/validate_detected_objects.py tests/fixtures/detected_objects/valid_epd_single_box.yaml --json
python3 scripts/validate_detected_objects.py tests/fixtures/detected_objects/missing_dimensions_warn.yaml --json
python3 scripts/validate_detected_objects.py tests/fixtures/detected_objects/missing_dimensions_warn.yaml --json --strict
```

Status meanings:
- `PASS`: valid and usable.
- `WARN`: usable, but missing/unknown non-blocking fields.
- `FAIL`: invalid for routing/bridge.

## Adapter usage

```bash
python3 scripts/run_task_recipe_adapter.py \
  --task-recipe tests/fixtures/task_recipes/valid_sort_by_colour.yaml \
  --objects tests/fixtures/detected_objects/valid_epd_colour_sorting.yaml \
  --json
```


## Workcell Studio normalized snapshot contract

Workcell Studio now validates EPD payloads through the machine-readable `workcell_perception_snapshot/v1` contract before task binding. EPD remains responsible for RealSense camera processing, detection, localization, and tracking; Workcell Studio owns scene identity checks, camera identity checks, task binding, and adapter configuration.

Required normalized fields are `scene_id`, `camera_id`, `timestamp`, `frame_id`, and `objects`. Each object requires an `object_id` or `track_id`, `label`, `confidence`, optional `attributes`, and either a finite pose (`position` plus normalized `orientation_xyzw`) or a finite `centroid`. Validation rejects duplicate object IDs, non-finite poses, non-normalized quaternions, confidence outside `[0, 1]`, and scene/camera mismatches.

Generated scene packages write `generated/perception_adapter_config.yaml`. Perception-backed scenes receive camera identity, expected frames, EPD input topic/message type, required object classes, confidence threshold, task binding, and the normalized output contract. Scenes with perception disabled report `NOT_APPLICABLE`; incomplete perception-backed metadata fails generation clearly.
