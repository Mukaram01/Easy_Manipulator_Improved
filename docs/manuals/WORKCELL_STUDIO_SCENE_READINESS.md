# Workcell Studio Scene Readiness Matrix

Use the scene readiness matrix when you need a catalog-backed view of whether
supported Workcell Studio scenes have enough evidence to proceed through the
safe simulation-first readiness path.

The matrix is broader than a visual screenshot audit. Scene3D GUI smoke and
screenshot outputs are optional debug/legacy evidence for Workcell Builder
triage; they are not required to prove production 3D editor readiness and they
must not introduce visual-topology or screenshot-quality gates. Production
readiness remains grounded in package structure, generated artifacts, schema
validation, manifest references, package build checks, generated-scene
validation, fake-hardware RViz/MoveIt launch validation, and whether
ROS-dependent checks were skipped or evaluated in the current environment.
RViz/MoveIt remains the planning and visual truth for the simulation-first
readiness path.

## Run the readiness matrix

Run the matrix from the repository root:

```bash
python3 scripts/run_workcell_studio_scene_readiness_matrix.py --supported-scenes scenes/supported_scenes.yaml --output-dir build/workcell_studio_scene_readiness
```

The command reads `scenes/supported_scenes.yaml` as the supported-scene catalog
and writes the readiness report artifacts under
`build/workcell_studio_scene_readiness`.

## Generated visual cache and Web 3D export policy

`generated/scene_visual_mesh_index.json` is generated cache/build output for Scene3D/Web preview paths. It should not be committed under `scenes/*/generated/`; the GUI/Web viewer refresh path regenerates it automatically when needed. Keep canonical scene state in tracked source-of-truth inputs such as YAML, xacro, URDF, layout, manifest, and task/config files.

Web scene JSON exports are also build/viewer outputs. Write them under `build/workcell_studio_web_scene/` or another ignored output location instead of `scenes/*/generated/`. Readiness evidence may cite regenerated outputs from a real workspace, but those generated cache/export files should not become canonical scene files.

## `ur5_2f_test` fake-hardware-only canary readiness

Use this focused canary when a PR or release check needs `ur5_2f_test`
evidence from an actual ROS 2 Humble Workcell Studio workspace. This is a
**fake-hardware-only** sequence. It must not be used as proof of real robot
motion, real controller readiness, or safe commissioning. Real-hardware work
requires separate guarded flags, preflight checks, and operator safety review.

Run the canary exactly from the expected ROS workspace layout:

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder ur5_2f_test
source install/setup.bash
cd /home/user/workcell_ws/src/easy_manipulation_deployment
python3 scripts/validate_builder_generated_scene.py scenes/ur5_2f_test --json
python3 scripts/run_workcell_studio_scene_readiness_matrix.py --supported-scenes scenes/supported_scenes.yaml --output-dir build/workcell_studio_scene_readiness
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

If a PR explicitly needs optional Scene3D GUI debug/legacy evidence, collect it
separately after the required validation commands:

```bash
cd /home/user/workcell_ws/src/easy_manipulation_deployment
python3 scripts/run_workcell_builder_scene3d_gui_smoke.py --repo-root "$PWD" --workspace-root /home/user/workcell_ws --scene ur5_2f_test --output /tmp/ur5_2f_scene3d_smoke.json --screenshot /tmp/ur5_2f_scene3d_smoke.png
```

Canary status meanings:

- **PASS** requires actual runtime evidence from a real ROS 2 Humble workspace:
  successful package build, generated-scene validation output,
  readiness-matrix output, and a fake-hardware RViz/MoveIt launch attempt for
  `ur5_2f_test`. Optional Scene3D GUI smoke JSON/screenshot artifacts may be
  attached as debug/legacy evidence, but they are not PASS requirements and do
  not replace generated scene validation, package build checks, or
  fake-hardware RViz/MoveIt launch validation.
- **BLOCKED** means required runtime or environment evidence is missing, such as
  ROS Humble not being sourced, the workspace not being built, the
  `ur5_2f_test` package not being available, generated-scene validation not
  being run, or the fake-hardware launch not being evaluated. Missing
  GUI/screenshot capture alone should be reported as an optional debug evidence
  limitation, not as a production-readiness blocker.
- **FAIL** means the required evidence was collected but one or more required
  commands reported a real validation, build, readiness, or fake-hardware launch
  failure. Optional Scene3D GUI smoke failures should be recorded as debug
  limitations unless the PR explicitly scoped that smoke path as the thing under
  test.

When optional Scene3D GUI smoke artifacts are attached, the repository copy of
`scenes/ur5_2f_test/generated/scene3d_gui_smoke.json` should be regenerated from
the real ROS 2 Humble workspace before it is cited as debug evidence. A stale
checked-in JSON file, a local non-ROS checkout run, or a hand-edited evidence
file is not sufficient to claim current GUI smoke coverage, but these optional
artifacts are not required for canary PASS.

## Output files

The readiness matrix writes these files:

- `build/workcell_studio_scene_readiness/scene_readiness_summary.json`
- `build/workcell_studio_scene_readiness/scene_readiness_summary.md`

Use the JSON output for CI, dashboards, and machine-readable regression checks.
Use the Markdown output for quick human triage in PRs, demo-readiness reviews,
and scene owner handoffs.

## `ur5_2f_test` optional real-workspace Scene3D GUI debug evidence

Use this focused smoke path only when a PR needs optional real-workspace
Workcell Builder Scene3D debug/legacy evidence for the supported `ur5_2f_test`
scene. Run it from the ROS workspace layout used for Workcell Studio validation,
not from an unrelated temporary checkout. This path is not a production 3D
editor readiness gate and must not be treated as a visual-topology or
screenshot-quality requirement.

Build and source the builder package first:

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder
source install/setup.bash
```

Then run the Scene3D GUI smoke capture against `ur5_2f_test`:

```bash
cd /home/user/workcell_ws/src/easy_manipulation_deployment
python3 scripts/run_workcell_builder_scene3d_gui_smoke.py --repo-root "$PWD" --workspace-root /home/user/workcell_ws --scene ur5_2f_test --output "$PWD/scenes/ur5_2f_test/generated/scene3d_gui_smoke.json" --screenshot "$PWD/scenes/ur5_2f_test/generated/scene3d_gui_smoke.png" --timeout-sec 30
```

Optional debug evidence files:

- `scenes/ur5_2f_test/generated/scene3d_gui_smoke.json`
- `scenes/ur5_2f_test/generated/scene3d_gui_smoke.png`
- stdout and stderr tail log paths recorded in the JSON evidence file

This optional evidence confirms only that the Workcell Builder Scene3D GUI smoke
path could open the selected scene and record visual/log artifacts in that
workspace. It does **not** launch real hardware, does **not** enable real robot
motion, does **not** prove production 3D editor readiness, and does **not** prove
fake-hardware RViz/MoveIt readiness by itself. Treat it as debug/legacy evidence
that may complement, but never replace, generated scene validation, package build
checks, the broader scene readiness matrix, or fake-hardware RViz/MoveIt launch
validation. RViz/MoveIt remains the planning and visual truth.

## Local `ur5_2f_test` real-workspace evidence runner

Use the local runner when you want one repeatable command to collect the
`ur5_2f_test` Workcell Builder build log, optional Scene3D GUI smoke
JSON/screenshot debug evidence, and readiness-matrix logs from a real ROS 2
Humble workspace. The runner is
fake-hardware-first: it sources ROS Humble when available, builds only
`workcell_builder`, runs the existing Scene3D GUI smoke runner, runs the
offline readiness matrix, and does **not** invoke `ros2 launch` or start real
robot drivers.

From the repository checkout inside the ROS workspace, run:

```bash
cd /home/user/workcell_ws/src/easy_manipulation_deployment
bash scripts/run_local_ur5_2f_workcell_validation.sh \
  --workspace-root /home/user/workcell_ws \
  --scene ur5_2f_test \
  --output-dir build/workcell_studio/local_validation/ur5_2f_test
```

Runner output files under the output directory include:

- `build_workcell_builder.log`
- `scene3d_gui_smoke.json`
- `scene3d_gui_smoke.png`
- `readiness_matrix_stdout.log`
- `readiness_matrix_stderr.log`
- `readiness_summary.json` when the readiness matrix produces JSON
- `validation_summary.md`

The runner reports `BLOCKED` when ROS Humble or the built `workcell_builder`
executable is unavailable and `FAIL` when required builder build or readiness
commands fail. `PASS` should be interpreted as the required build and
readiness-matrix checks completing; Scene3D smoke JSON/screenshot artifacts
remain optional debug/legacy evidence, and their absence should not be used as a
production 3D editor readiness failure.
After a successful automated run, complete the
manual GUI persistence check printed by the script: open Workcell Builder, open
`ur5_2f_test`, confirm the robot/gripper/environment are visible, create an
editable layout from preview, move an editable item, save, close/reopen, and
confirm the moved item persists.

## Status definitions

The matrix uses these status values:

- **PASS**: required evidence is present and valid.
- **FAIL**: required evidence exists but is invalid or internally inconsistent.
- **BLOCKED**: readiness cannot be evaluated because required inputs, generated
  artifacts, ROS environment, generated-scene validation, package build checks,
  or fake-hardware RViz/MoveIt launch evidence are missing.

`BLOCKED` is intentionally distinct from `FAIL`: it means Workcell Studio cannot
make a trustworthy readiness judgment yet because a prerequisite is absent or the
current environment cannot evaluate it. Do not hide a broken supported scene by
removing it from the supported-scene catalog; keep the blocker visible until the
missing input or environment is restored.

## Readiness categories

The readiness matrix combines several categories so that a scene is not treated
as ready based on one signal alone:

- package structure expected by the supported-scene catalog;
- required authoring files and generated artifacts;
- `cell_definition.yaml` schema validation;
- `scene_manifest.yaml` references and internal consistency;
- optional Scene3D GUI smoke/screenshot debug evidence when a PR explicitly
  requests legacy visual triage artifacts;
- fake-hardware RViz/MoveIt launch derivation from catalog/package metadata;
- ROS skip/evaluation state, including whether ROS-dependent checks were
  actually evaluated or explicitly skipped because the environment was missing.

Scene3D screenshots can help with visual debugging and layout triage, but they
are optional legacy/debug artifacts. They do not prove production 3D editor
readiness, do not replace package contracts, generated artifacts, manifest
references, generated-scene validation, package build checks, or fake-hardware
RViz/MoveIt launch validation, and must not create new screenshot-quality or
visual-topology gates. RViz/MoveIt remains the planning and visual truth.

## Recommended triage loop

After the first failing scene, fix issues in dependency order instead of jumping
straight to RViz/MoveIt launch debugging:

1. Fix missing authoring and generated files first.
2. Validate `cell_definition.yaml` and correct schema or consistency errors.
3. Fix `scene_manifest.yaml` references so the package index points at real,
   current artifacts.
4. If a PR explicitly needs legacy Scene3D GUI triage, collect optional smoke
   JSON/screenshot debug artifacts without treating them as production-readiness
   gates.
5. Evaluate fake-hardware RViz/MoveIt readiness as the planning and visual truth.

This order keeps the earliest source-of-truth blockers visible and prevents ROS
launch failures from masking simpler missing-file, schema, or manifest-reference
problems.

## Safety notes

The readiness matrix is a simulation-first evidence report. Fake hardware
remains the default readiness path, and the matrix does not invoke real robot
motion. Any later real-hardware commissioning work must remain behind explicit
guarded flags, preflight checks, and clear operator warnings.
