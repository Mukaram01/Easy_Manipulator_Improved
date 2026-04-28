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

## Limitations

- The current EMD runtime request (`emd_msgs/srv/GraspRequest`) and target message (`emd_msgs/msg/GraspTarget`) do not carry an explicit release/place destination pose field.
- Bridge payload now records a `runtime_release_adapter_boundary` block and emits explicit warnings for:
  - using explicit destination release pose metadata (offline bridge level),
  - destination frame mismatch with planning frame,
  - destination missing/malformed pose, causing fallback to `release_x_offset` / `release_use_grasp_z`.
- Because of this interface boundary, `run_grasp_execution` keeps existing release behavior unless runtime interfaces are extended in a future non-breaking step.
- synthesized grasp poses are conservative first-pass placeholders.
- this bridge is not full production sequencing or a safety certification layer.
