# EMD Grasp Bridge Payload (offline-first)

## What this bridge does

`runtime_execution_plan/v1 -> emd_grasp_bridge_payload/v1` provides a conservative adapter from offline task routing output into an EMD-shaped grasp payload that mirrors the existing `emd_msgs/msg/GraspTask` and `emd_msgs/srv/GraspRequest` interfaces.

It is additive and does **not** replace `run_grasp_execution`.
It is now paired with `detected_objects/v1` so real D435i/EPD captures can feed the same runtime bridge path.

## Why it exists

Recent workflow layers produce deterministic offline plans, but `run_grasp_execution` consumes EMD grasp messages. This bridge keeps the runtime stack unchanged while adding a reviewable conversion stage.

## Mapping to existing EMD interfaces

- Input: `runtime_execution_plan/v1`
- Output contract: `emd_grasp_bridge_payload/v1`
- Upstream recommended input to runtime plan: `detected_objects/v1`
- Runtime boundary references:
  - topic: `grasp_tasks` (`emd_msgs/msg/GraspTask`)
  - service: `grasp_requests` (`emd_msgs/srv/GraspRequest`)

Per-step mapping:
- each `pick*` step -> one `grasp_target`
- object pose -> `target_pose`
- object dimensions/shape -> `target_shape`
- `preferred_end_effector` -> `grasp_methods[].ee_id`
- destination metadata preserved as bridge metadata for future place/release logic
- destination-aware release metadata is preserved in each bridge target as:
  - `destination_id`, `destination_name`, `destination_label`
  - `destination_pose.frame_id`, `destination_pose.xyz`, `destination_pose.rpy`, `destination_pose.quaternion_xyzw`
  - optional `destination_approach` / `destination_retreat`
  - `destination_safety_warnings`

## Offline mode (default)

Default mode is dry-run/offline and does not require ROS:

```bash
python3 scripts/convert_runtime_plan_to_emd_grasp.py \
  --runtime-plan tests/fixtures/emd_grasp_bridge/valid_single_box_runtime_plan.json \
  --json
```

## Optional guarded ROS mode

ROS mode is opt-in and lazy-imported. It is only attempted when `--mode ros` is passed.

```bash
python3 scripts/convert_runtime_plan_to_emd_grasp.py \
  --runtime-plan tests/fixtures/emd_grasp_bridge/valid_single_box_runtime_plan.json \
  --mode ros \
  --dry-run false \
  --ros-interface service \
  --service-name grasp_requests \
  --json
```

Topic variant:

```bash
python3 scripts/convert_runtime_plan_to_emd_grasp.py \
  --runtime-plan tests/fixtures/emd_grasp_bridge/valid_single_box_runtime_plan.json \
  --mode ros \
  --dry-run false \
  --ros-interface topic \
  --topic-name grasp_tasks \
  --json
```

If ROS Python modules are unavailable, status is `FAIL` with an explicit error message.

## Perception replay to bridge preview (offline-only)

Golden demo readiness now includes an **offline adapter preview**:

- input replay snapshot: `detected_objects_snapshot/v1` (EPD-style)
- output preview payload: `generated/emd_bridge_payload_preview.json`
- output report: `generated/perception_bridge_preview_report.json`

Run manually:

```bash
python3 scripts/generate_perception_bridge_preview.py \
  --perception-profile <scene>/generated/perception_profile.yaml \
  --detected-objects tests/fixtures/perception/detected_objects_snapshot_golden.yaml \
  --task-intent <scene>/generated/workcell_builder_task_intent.yaml \
  --environment-layout <pack>/exported/environment_layout.yaml \
  --output-payload <scene>/generated/emd_bridge_payload_preview.json \
  --output-report <scene>/generated/perception_bridge_preview_report.json \
  --json
```

This artifact is **preview-only** and explicitly sets: dry-run, no robot motion, no runtime execution, no MoveIt plan service calls, and no real hardware enablement. It is not published to ROS topics/services and does not execute grasp runtime behavior.

## Limitations

- The current EMD runtime request (`emd_msgs/srv/GraspRequest`) and target message (`emd_msgs/msg/GraspTarget`) do not carry an explicit release/place destination pose field.
- Bridge payload now records a `runtime_release_adapter_boundary` block and emits explicit warnings for:
  - using explicit destination release pose metadata (offline bridge level),
  - destination frame mismatch with planning frame,
  - destination missing/malformed pose, causing fallback to `release_x_offset` / `release_use_grasp_z`.
- `run_grasp_execution` now supports a **non-breaking runtime adapter** for explicit destination release poses:
  - keep `release_x_offset` / `release_use_grasp_z` as default legacy behavior,
  - opt-in with `use_explicit_release_pose:=true`,
  - point `explicit_release_pose_bridge_payload_path` to the generated `emd_grasp_bridge_payload/v1` JSON,
  - keep `fallback_to_legacy_release:=true` to safely recover when destination pose is missing, malformed, wrong-frame, or unreachable.
- synthesized grasp poses are conservative first-pass placeholders.
- this bridge is not full production sequencing or a safety certification layer.
