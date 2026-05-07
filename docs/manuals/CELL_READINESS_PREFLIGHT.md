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

## 6) Golden builder-to-readiness demo + safe preview handoff
Run the canonical golden flow:

```bash
python3 scripts/run_golden_builder_readiness_demo.py \
  --scene-package scenes/ur5_2f_test \
  --output-dir /tmp/golden_builder_demo \
  --force --json
```

Then inspect `/tmp/golden_builder_demo/golden_builder_demo_summary.json` for:
- readiness pack path,
- static preview path,
- dashboard path,
- RViz/MoveIt fake-hardware preview command (when launch metadata is available),
- explicit no-motion safety flags.

If a preview command is present, run it manually only after sourcing ROS/workspace and only in fake hardware mode. The golden demo itself does not launch ROS, does not call MoveIt planning services, and does not command real robot motion.

This preview demonstrates visualization/metadata readiness only. It is not proof of real hardware commissioning readiness.

## 7) Golden demo visual markers and preview meaning

The golden readiness demo now emits `preview/visual_markers.json` and records a `visual_preview` section in `golden_builder_demo_summary.json`.

Marker groups:
- **layout markers**: robot base, table/support surface, camera pose.
- **task-flow markers**: pick zone/source, grasp point, place zone/target, release point, approach vector, retreat vector, and pick→place flow arrow.
- **safety markers/metadata**: fake hardware + offline preview + no motion + no runtime execution constraints.

Open the static preview and dashboard manually:
- `preview/static_preview.html`
- `readiness_dashboard.html`

Use the RViz/MoveIt preview command manually from the summary only after sourcing ROS 2/workspace. The demo itself does **not** auto-launch RViz, does **not** call MoveIt planning services, and does **not** command robot motion.

Visual preview artifacts are communication aids and do **not** certify real-hardware commissioning readiness.
