# Cell Readiness Preflight

## 1) Offline preflight
Run:

```bash
python3 scripts/run_cell_readiness_check.py \
  --scene-package ur5_2f_test \
  --task-recipe tests/fixtures/task_recipes/valid_garbage_sorting.yaml \
  --detected-objects tests/fixtures/detected_objects/valid_epd_garbage_sorting.yaml \
  --json
```

Offline mode validates scene discovery, task recipe, detected objects (if provided), and generated-cell dry-run validation. ROS camera/EPD topics are not required.

## 2) Live D435i/EPD preflight
Run:

```bash
python3 scripts/run_cell_readiness_check.py \
  --scene-package ur5_2f_test \
  --task-recipe tests/fixtures/task_recipes/valid_garbage_sorting.yaml \
  --live --check-ros-topics --json
```

Live mode requires D435i camera topics and the EPD output topic.

## 3) TF camera-to-world check
Add TF enforcement for live readiness:

```bash
--check-tf --target-frame world --camera-frame camera_depth_optical_frame
```

If TF camera → world is missing, preflight returns `FAIL` with a blocker.

## 4) PASS / WARN / FAIL
- **PASS**: ready for the next requested stage.
- **WARN**: usable for diagnostics only; do not trust for motion.
- **FAIL**: blocked; resolve blockers before replay/simulation/commissioning.

## 5) Before enabling replay or motion
- Keep generated-cycle checks in dry-run/no-motion mode first.
- Resolve all preflight blockers.
- Resolve warnings for any stage involving physical motion.
- Do not publish robot motion commands from preflight.

The operator panel includes a **Run Preflight Check** button that runs this script and prints blockers/warnings.

## One-click gated dry-run
1. Select scene
2. Select task recipe
3. Select detected objects or live EPD
4. Press Run Gated Dry-Run
5. Read PASS/WARN/FAIL
6. Do not proceed to replay/motion unless all blockers are clear

This path is dry-run only and never executes robot motion.
