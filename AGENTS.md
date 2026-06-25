# AGENTS.md — Workcell Studio / Easy Manipulation Deployment

## Project identity

This repository is the primary Workcell Studio / Easy Manipulation Deployment workspace.

Workcell Studio is an internal configurable robotic-cell platform for authoring, generating, validating, simulating, and eventually commissioning industrial robotic cells.

This is not only a sorting demo. Sorting, inspection, machine tending, conveyor picking, binning, palletising, and similar workflows are scenario templates inside Workcell Studio.

The primary product UI is `workcell_builder`. Improve `workcell_builder` directly.

Do not replace the product with Streamlit, dashboards, notebooks, or separate demo UIs. Those are allowed only as optional reporting, export, validation summary, or demo artifacts when explicitly useful.

Keep EMD / Workcell Builder separate from EPD. Do not merge the EPD GUI into Workcell Builder.

## Product goal

Workcell Studio should support:

- configurable robots
- configurable tools and grippers
- configurable environments
- configurable task logic
- configurable grasp strategy
- generated ROS 2 scene/workcell packages
- RViz/MoveIt simulation
- fake-hardware-first validation
- EPD/RealSense perception integration
- guarded real-hardware readiness later
- clear logs, reports, dashboards, and demo/investor-friendly outputs

The target workflow is:

```text
Workspace -> New Cell -> Layout -> Task Intent -> Generate Scene Package -> Validate -> Plan & Simulate
```

## Current tactical focus

Current priority is not broad cleanup. Current priority is making the real supported Scene3D/Product View path reliable.

Canonical M1 scene:

```text
scenes/ur5_2f_test
```

M1-clean means:

- robot is visible
- robot is connected, not exploded
- gripper/tool is visible where assets exist
- table/workbench is visible
- camera is visible
- Product View opens from a useful angle
- Product View frames physical product geometry, not overlay helper bounds
- helper boxes are hidden by default
- stale primitive robot boxes are hidden when generated robot meshes exist
- smoke/diagnostic evidence is honest
- fake-hardware RViz/MoveIt remains the validation foundation

After M1 is clean, generalise to other supported scenes.

## Priority order

When choosing between fixes, prefer this order:

1. Fix known breakage from the latest merged PR.
2. Fix failing tests caused by real product drift.
3. Stop Workcell Builder breakage.
4. Make `ur5_2f_test` Scene3D/Product View clean.
5. Make scene generation repeatable.
6. Make supported scenes reproducible.
7. Improve RViz/MoveIt fake-hardware simulation.
8. Improve true 3D scene editing and digital-twin-style preview.
9. Improve grasp planner and grasp execution wiring.
10. Improve EPD/RealSense bridge integration.
11. Improve investor/demo visuals.
12. Prepare guarded real-hardware readiness only after simulation and gates are stable.

Do not start broad optimisation while an obvious product mismatch or failing regression exists.

## Repository ownership rules

Easy_Manipulator_Improved owns:

- Workcell Studio product shell
- Workcell Builder
- scene authoring
- scene/package generation
- validation and readiness reports
- RViz/MoveIt orchestration
- grasp planner/execution wiring
- fake-hardware simulation
- runtime readiness checks
- EPD bridge consumption

`epd_Improved` owns:

- RealSense input
- detection
- localization
- tracking
- classification
- perception algorithms
- EPD GUI and perception-specific configuration

Boundary:

```text
EPD produces perception results.
Workcell Studio consumes perception results.
Workcell Studio owns cell authoring, generation, validation, planning, and simulation orchestration.
```

Do not make EPD own cell definition, scene generation, task planning, or Workcell Studio UI state.

## Safety rules

Never weaken safety gates.

Defaults must remain safe:

- fake hardware by default
- no real robot motion by default
- no automatic runtime send by default
- no uncontrolled service/topic publishing
- real-hardware mode requires explicit guarded flags
- generated validation reports are not safety certificates
- any real-hardware readiness work includes blockers, preflight checks, and clear warnings

Generated packages may support `use_fake_hardware:=true/false`, but real hardware must never become the default path.

If a change touches launch files, execution nodes, controllers, runtime services, or hardware parameters, verify that fake hardware remains the default and real robot motion remains explicitly guarded.

## Workcell Builder rules

Improve the existing Workcell Builder instead of creating a separate product UI.

Keep the UI clean:

- do not add more main-page buttons unless essential
- move secondary actions into menus, dropdowns, side panels, or advanced panels
- no silent no-op buttons
- every visible primary action must either work or clearly explain why unavailable
- user-facing errors should explain what failed, which file or command was involved, and what to do next
- prefer a clear selected-scene action path: Validate, Generate, Plan/Simulate, Open Logs, Export
- keep demo/test tools away from the primary user flow unless clearly marked advanced/developer

## Scene generation rules

Generated scenes must be reproducible.

Supported scene packages should consistently handle:

- `environment.yaml`
- `cell_definition.yaml`
- `scene_manifest.yaml`
- `layout/workcell_studio_layout.yaml` where applicable
- `urdf/scene.urdf.xacro`
- `launch/demo.launch.py`
- generated readiness or validation reports where applicable

Prefer source-of-truth clarity:

- `environment.yaml` is authoring/layout-adjacent scene data
- `layout/workcell_studio_layout.yaml` is editor/UI state
- `cell_definition.yaml` is the canonical generated exchange model
- `scene_manifest.yaml` is the scene package index and contract summary
- task recipes define task semantics
- EPD bridge payloads are perception/runtime contracts, not authoring state

Avoid scene-specific hacks. Do not make one scene work by breaking the broader supported scene catalog.

## Supported scene expectations

Important supported scenes include:

- `ur5_2f_test`
- `ur5_3f_test`
- `ur3_suction_test`
- `ur10_2f_test`
- `ur5_airpick4_test`
- `suction_test`
- generated builder pick/place demo scenes where applicable
- sorting or conveyor scenes where applicable

Each supported scene should eventually answer:

- Does it have required authoring files?
- Does it have required generated files?
- Can it be validated?
- Can it be regenerated?
- Can it build as a ROS 2 package?
- Can it launch in RViz/MoveIt fake hardware?
- What is the current blocker if it cannot?

## 3D canvas and Product View rules

The 3D canvas should become a trustworthy scene editor.

True 3D means:

- mesh-backed visuals when meshes exist
- primitive fallback only when meshes are missing
- reliable camera controls
- stable selection/picking
- inspector-based XYZ/RPY editing
- deterministic transform save/load
- clear distinction between editable layout items and locked generated URDF preview items
- scene hierarchy synchronized with 3D selection
- right-side inspector synchronized with selection

Near-term canvas model:

```text
Editable layout items = user can select, move, rotate, edit, save.
Generated URDF preview items = locked visual preview, not directly edited.
Inspector = authoritative manual XYZ/RPY editing surface.
Layout YAML = editor state.
Generated scene files = regenerated from source-of-truth data.
```

Product View should show physical/product content first:

- generated robot visuals
- mesh previews
- editable physical layout items
- table/workbench
- camera body
- gripper/tool meshes
- real authored environment items

Product View must hide by default:

- helper overlays
- warning labels
- diagnostic labels
- safety/pick/place zones
- reachability heatmaps
- collision warnings
- work envelope
- task route
- approach/retreat arrows
- camera FOV
- pick coverage
- EPD detections
- detection labels
- primitive fallback robot boxes when generated robot meshes exist

Overlay/helper content should remain available through explicit overlay/diagnostic controls, but must not dominate the initial Product View.

When deciding whether a preview item is physical or helper/overlay, inspect all relevant identity fields:

- `source_layer`
- `active_visual_source`
- `role`
- `category`
- `id`
- `display_name`
- `status`
- `warnings`
- `mesh_load_warning`
- source path / mesh metadata where relevant

Do not rely only on `role` and `category`.

Canonical helper tokens include:

```text
overlay
helper
diagnostic
safety_zone
pick_zone
place_zone
robot_reach
warning_anchor
warning_badge
camera_fov
fov
pick_coverage
reachability
collision
work_envelope
task_route
approach_retreat
epd_detection
detection_label
bounds_box
bounding_box
```

These should require explicit overlay/diagnostic visibility unless the item is clearly physical product geometry.

No canvas action should silently fail. If save, load, transform, mesh preview, or selection cannot work, show a clear warning with the responsible file or missing metadata.

## Robot, tool, and asset rules

Robot/tool/environment support should be capability-based, not hardcoded per scene.

Initial practical combinations:

- UR5 + Robotiq 2F
- UR5 + suction
- UR3 + suction
- UR10 + 2F
- UR5 + AirPick-style suction where supported
- simple delta/cartesian placeholder + suction later

Future support may include Fanuc, ABB, SCARA, gantry/cartesian, delta robots, custom arms, tool changers, and custom end effectors.

Do not add hardcoded logic that prevents future robot/tool swaps.

Tool behavior should be described through capability/config metadata where practical.

Gripper orientation defaults and offsets must be handled consistently through scene generation and validation. Avoid one-off manual xacro fixes where a generator-level default or capability metadata fix is appropriate.

## Grasp planner and execution rules

The grasp planner and execution path should support simulation before real hardware.

A generated scene is not manipulation-ready unless it can prove:

- planning group is valid
- robot base link is valid
- end-effector link is valid
- tool metadata is valid
- object/task input is available or replayable
- fake-hardware launch path works
- RViz/MoveIt planning scene loads
- simulated plan can be produced where practical
- real execution remains guarded

Prefer generated or scene-local grasp planner config that can be inspected and edited.

## EPD/RealSense bridge rules

Keep the bridge explicit and clean.

Workcell Studio consumes normalized perception outputs from EPD; it does not absorb EPD responsibilities.

Bridge/config work should support:

- camera ID
- scene ID
- frame ID
- timestamp
- object ID or track ID
- class/label
- pose or localized centroid
- confidence/quality
- task binding
- replayed snapshot mode
- live EPD mode

The bridge should support:

```text
perception: off
perception: replayed_snapshot
perception: live_epd
```

A missing perception runtime should block perception-backed execution clearly. It must not crash the builder or corrupt scene metadata.

## ROS and platform rules

Current supported baseline:

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 / RViz as the primary planning and visualization foundation

Do not migrate ROS distributions unless explicitly requested.

Do not make Gazebo, Ignition/Gazebo Sim, or Isaac Sim mandatory for the core workflow yet.

The core path remains RViz/MoveIt-first. Gazebo and Isaac can be added later as optional advanced backends.

## Testing and validation commands

Expected user VM layout is usually `/home/user/workcell_ws` or `/home/ubuntu/workcell_ws`. Codex containers may instead use `/workspace/Easy_Manipulator_Improved` for static repo inspection.

Before running commands, confirm the actual checkout/workspace path. Use a sourced ROS workspace only for commands that need `colcon`, `ros2`, or launch files.

For quick Python/static tests:

```bash
python3 -m pytest tests/test_scene3d_product_overlay_filtering.py
python3 -m pytest tests/test_workcell_builder_product_view_defaults.py
```

For builder changes in a ROS workspace:

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder
source install/setup.bash
```

For generated scene validation:

```bash
cd /home/user/workcell_ws/src/easy_manipulation_deployment
python3 scripts/validate_builder_generated_scene.py scenes/ur5_2f_test --json
```

For fake-hardware launch validation:

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

For Scene3D GUI smoke when workspace/display are available:

```bash
cd /home/user/workcell_ws/src/easy_manipulation_deployment
python3 scripts/run_workcell_builder_scene3d_gui_smoke.py \
  --repo-root "$PWD" \
  --workspace-root /home/user/workcell_ws \
  --scene ur5_2f_test \
  --output "$PWD/scenes/ur5_2f_test/generated/scene3d_gui_smoke.json" \
  --screenshot "$PWD/scenes/ur5_2f_test/generated/scene3d_gui_smoke.png" \
  --timeout-sec 45
```

When validating broader generation logic, validate more than one supported scene. If a script or scene is missing, do not invent success; report the blocker and propose the smallest fix.

## PR rules

Use small incremental PRs.

For normal Codex development tasks, create or use a branch named like `codex/<short-task-name>` unless the user requests otherwise.

Each PR should:

- target one repo only unless explicitly required
- avoid broad refactors
- preserve ROS 2 Humble compatibility
- preserve current working demos
- preserve safety gates
- include tests or validation commands
- update docs only if user-facing behavior changes
- include risks and rollback notes

PR summaries should include:

```text
Summary:
- what changed
- why it matters for Workcell Studio
- affected files/packages

Validation:
- commands run
- results
- commands not run and why

Safety:
- confirms fake hardware remains default where relevant
- confirms no real-hardware path was enabled accidentally

Risks / rollback:
- known limitations
- how to revert safely
```

## Codex task behavior

When working as Codex in this repo:

1. Read the relevant files before editing.
2. Inspect recent PRs before choosing the next task.
3. Keep the task scoped.
4. Do not perform broad cleanup unless requested.
5. Prefer tests/validators that protect real product fixes.
6. Prefer clear error messages over silent fallback.
7. Preserve existing working scenes.
8. Do not hide failures by marking broken scenes unsupported unless explicitly requested.
9. Do not claim success without validation evidence.
10. If blocked, explain the exact blocker and smallest next step.
11. Keep changes practical for fast PR iteration.

If asked “what next?”, answer:

1. Current state.
2. What the latest PR changed.
3. What is still broken.
4. The next exact PR.
5. Exact files to change.
6. Validation command.

Do not give a Codex prompt unless the user explicitly asks for one.

## Current next-fix selection logic

Choose the next PR using this order:

1. Fix known failing tests caused by recent product changes.
2. Fix actual Product View behavior mismatch.
3. Fix `ur5_2f_test` visibility/framing/robot/table/camera/gripper issues.
4. Fix generated robot/gripper mesh identity.
5. Fix scene package generation only when needed for M1.
6. Fix all supported scenes only after the canonical scene is clean.
7. Optimise large files only when it does not delay M1.

A current example is Product View defaults drift: if a test expects helpers off but code still enables a helper overlay by default, fix the product code first, then update stale assertions to match intended behavior.

## Anti-goals

Do not:

- rewrite the whole architecture
- merge EPD into Workcell Builder
- combine the EPD GUI into the EMD/workcell_builder UI
- replace `workcell_builder` with Streamlit
- make Isaac mandatory
- make Gazebo mandatory before the core workflow is stable
- enable real robot motion by default
- add UI clutter
- create scene-specific hacks
- weaken fake-hardware-first safety
- claim simulation or execution works unless validated
- treat one passing demo scene as proof that the platform is stable
- add placeholder buttons to the main UI
- hide missing files or broken metadata behind vague success messages
- move business-critical scene state into random UI-only files
- make perception own task planning or scene generation

## Definition of done

A change is only done when:

- the requested scope is complete
- relevant files are updated
- the change is validated with the smallest meaningful command set
- safety defaults are preserved
- user-facing behavior is documented when needed
- PR summary explains what changed and why
- failures or skipped validations are clearly stated

For Workcell Studio, “done” means the change moves the platform toward:

```text
reliable Workcell Builder
+ repeatable scene generation
+ clean true 3D scene editing
+ RViz/MoveIt fake-hardware simulation
+ grasp planner/execution readiness
+ clean EPD/RealSense bridge
+ guarded real-hardware future
```
