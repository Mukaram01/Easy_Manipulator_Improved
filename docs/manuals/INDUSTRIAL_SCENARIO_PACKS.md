# Industrial Scenario Packs

Scenario packs (`scenario_pack/v1`) are validation wrappers around existing Cell Definition generation, preview, and gated dry-run tooling.

- They **do not** replace Workcell Builder.
- They **do not** replace task recipes.
- They keep `safe_for_robot_motion: false` and never execute physical robot motion.

## Run one scenario

```bash
python3 scripts/run_scenario_pack.py \
  --scenario scenario_packs/ur5_2f_garbage_sorting.yaml \
  --output-root /tmp/scenario_runs \
  --json
```

## Run scenario matrix

```bash
python3 scripts/run_scenario_matrix.py \
  --scenario-dir scenario_packs \
  --output-root /tmp/scenario_matrix \
  --json
```

PASS/WARN/FAIL/SKIPPED are reported per scenario and aggregated in `scenario_matrix_report.json`.

## MVP live scenario (single coherent path)

Use `scenario_packs/ur5_2f_live_garbage_sorting.yaml` for one aligned UR5 + Robotiq 2F + D435i path with runtime scene package `ur5_2f_test`.

```bash
python3 scripts/run_scenario_pack.py \
  --scenario scenario_packs/ur5_2f_live_garbage_sorting.yaml \
  --output-root /tmp/scenario_runs \
  --json
```

Then run live dry-run (still no physical motion):

```bash
python3 scripts/run_generated_workcell_bundle.py \
  --workcell /tmp/scenario_runs/ur5_2f_live_garbage_sorting/generated_workcell/ur5_2f_live_garbage_sorting \
  --output-dir /tmp/ur5_2f_live_run \
  --capture-live \
  --epd-topic /easy_perception_deployment/epd_localize_output \
  --epd-qos-reliability best_effort \
  --target-frame world \
  --require-transform \
  --gated-dry-run \
  --dry-run \
  --no-replay \
  --preflight-live \
  --preflight-check-tf \
  --preflight-check-ros-topics \
  --preview-task-flow \
  --json
```
