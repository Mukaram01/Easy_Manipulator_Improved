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
