# EMD Grasp Bridge Payload (offline-first)

## What this bridge does

`runtime_execution_plan/v1 -> emd_grasp_bridge_payload/v1` provides a conservative adapter from offline task routing output into an EMD-shaped grasp payload that mirrors the existing `emd_msgs/msg/GraspTask` and `emd_msgs/srv/GraspRequest` interfaces.

It is additive and does **not** replace `run_grasp_execution`.

## Why it exists

Recent workflow layers produce deterministic offline plans, but `run_grasp_execution` consumes EMD grasp messages. This bridge keeps the runtime stack unchanged while adding a reviewable conversion stage.

## Mapping to existing EMD interfaces

- Input: `runtime_execution_plan/v1`
- Output contract: `emd_grasp_bridge_payload/v1`
- Runtime boundary references:
  - topic: `grasp_tasks` (`emd_msgs/msg/GraspTask`)
  - service: `grasp_requests` (`emd_msgs/srv/GraspRequest`)

Per-step mapping:
- each `pick*` step -> one `grasp_target`
- object pose -> `target_pose`
- object dimensions/shape -> `target_shape`
- `preferred_end_effector` -> `grasp_methods[].ee_id`
- destination metadata preserved as bridge metadata for future place/release logic

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

- destination pose is preserved for sorting/place metadata, but current grasp execution release behavior is unchanged unless later extended.
- synthesized grasp poses are conservative first-pass placeholders.
- this bridge is not full production sequencing or a safety certification layer.
