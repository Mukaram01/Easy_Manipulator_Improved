# AGENTS.md

## Project identity

This repository is the primary Workcell Studio / Easy Manipulation Deployment workspace.

Workcell Studio is an internal configurable robotic-cell platform for authoring, generating, validating, simulating, and eventually commissioning industrial robotic cells.

This is not only a sorting demo. Sorting, inspection, machine tending, conveyor picking, binning, palletising, and similar workflows are scenario templates inside Workcell Studio.

## Product goal

The goal is to make Workcell Studio capable of:

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

## Priority order

When choosing between possible fixes or features, prefer this order:

1. Stop Workcell Builder breakage.
2. Make scene generation repeatable.
3. Make all supported scenes reproducible.
4. Improve RViz/MoveIt fake-hardware simulation.
5. Improve true 3D scene editing and digital-twin-style preview.
6. Improve grasp planner and grasp execution wiring.
7. Improve EPD/RealSense bridge integration.
8. Improve investor/demo visuals.
9. Prepare real-hardware readiness only after simulation and safety gates are stable.

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

epd_Improved owns:

- RealSense input
- detection
- localization
- tracking
- classification
- perception algorithms
- EPD GUI and perception-specific configuration

Do not merge EPD into Workcell Builder.

Do not make EPD the owner of cell definition, scene generation, task planning, or Workcell Studio UI state.

Do not combine the EPD GUI into the EMD/workcell_builder UI.

The correct boundary is:

```text
EPD produces perception results.
Workcell Studio consumes perception results.
Workcell Studio owns cell authoring, generation, validation, planning, and simulation orchestration.
```

## Safety rules

Never weaken safety gates.

Default behavior must remain safe:

- fake hardware by default
- no real robot motion by default
- no automatic runtime send by default
- no uncontrolled service/topic publishing
- real-hardware mode must require explicit guarded flags
- generated validation reports are not safety certificates
- any real-hardware readiness work must include blockers, preflight checks, and clear warnings

Generated packages should support `use_fake_hardware:=true/false` where supported, but real hardware must never become the default path.

If a change touches launch files, execution nodes, controllers, runtime services, or hardware parameters, verify that fake hardware remains the default and real robot motion remains explicitly guarded.

## Workcell Builder rules

The main UI direction is `workcell_builder`.

Do not replace `workcell_builder` with Streamlit.

Streamlit, dashboards, or static HTML reports may be used only for optional reporting, export, validation summaries, or demo artifacts. They must not become a parallel replacement for the main Workcell Builder product UI.

Improve the existing Workcell Builder instead of creating a separate product UI.

Keep the UI clean:

- do not add more main-page buttons unless essential
- move secondary actions into menus, dropdowns, side panels, or advanced panels
- no silent no-op buttons
- every visible primary action must either work or clearly explain why it is unavailable
- user-facing errors should explain what failed, which file or command was involved, and what to do next
- prefer a clear selected-scene action path: Validate, Generate, Plan/Simulate, Open Logs, Export
- keep demo/test tools away from the primary user flow unless they are clearly marked as advanced or developer tools

The Workcell Builder should help the user understand:

```text
Workspace -> New Cell -> Layout -> Task Intent -> Generate Scene Package -> Validate -> Plan & Simulate
```

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

Avoid scene-specific hacks.

Do not make only one scene work while breaking the broader supported scene catalog.

Do not treat `suction_test` or `ur5_2f_test` passing as proof that the whole platform works.

Prefer source-of-truth clarity:

- `environment.yaml` is authoring/layout-adjacent scene data
- `layout/workcell_studio_layout.yaml` is editor/UI state
- `cell_definition.yaml` is the canonical generated exchange model
- `scene_manifest.yaml` is the scene package index and contract summary
- task recipes define task semantics
- EPD bridge payloads are perception/runtime contracts, not authoring state

Generated files should include enough provenance to identify:

- source inputs
- generator version or script
- timestamp where appropriate
- validation status
- known blockers

When touching generation logic, prefer validating multiple supported scenes, not only one hand-picked scene.

## Supported scene expectations

The platform should eventually make the supported scene catalog machine-readable.

At minimum, changes should preserve and improve these common scene types when present:

- `ur5_2f_test`
- `ur5_3f_test`
- `ur3_suction_test`
- `ur10_2f_test`
- `ur5_airpick4_test`
- `suction_test`
- generated builder pick/place demo scenes where applicable
- sorting or conveyor scenes where applicable

Each supported scene should be able to answer:

- Does it have required authoring files?
- Does it have required generated files?
- Can it be validated?
- Can it be regenerated?
- Can it build as a ROS 2 package?
- Can it launch in RViz/MoveIt fake hardware?
- What is the current blocker if it cannot?

## EPD/RealSense bridge rules

Keep the bridge explicit and clean.

Workcell Studio should consume normalized perception outputs from EPD, not absorb EPD responsibilities.

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

A missing perception runtime should block perception-backed execution clearly.

It must not crash the builder or corrupt scene metadata.

The bridge should support three practical modes:

```text
perception: off
perception: replayed_snapshot
perception: live_epd
```

EPD/RealSense work should preserve this separation:

```text
RealSense + EPD -> detection/localization/tracking/classification
Workcell Studio -> task binding, grasp planning input, simulation/execution readiness
```

## 3D canvas rules

The 3D canvas should become a trustworthy scene editor.

True 3D means:

- mesh-backed visuals when meshes exist
- primitive fallback when meshes are missing
- reliable camera controls
- stable selection/picking
- inspector-based XYZ/RPY editing
- deterministic transform save/load
- clear distinction between editable layout items and locked generated URDF preview items
- scene hierarchy synchronized with 3D selection
- right-side inspector synchronized with selection
- camera FOV, pick/place zones, safety overlays, reachability, and collision overlays later

Do not detour into a full game-engine or physics rewrite.

Do not make Gazebo or Isaac mandatory yet.

The correct near-term canvas model is:

```text
Editable layout items = user can select, move, rotate, edit, save.
Generated URDF preview items = locked visual preview, not directly edited.
Inspector = authoritative manual XYZ/RPY editing surface.
Layout YAML = editor state.
Generated scene files = regenerated from source-of-truth data.
```

No canvas action should silently fail. If save, load, transform, mesh preview, or selection cannot work, show a clear warning with the file or missing metadata responsible.

## Robot, tool, and asset rules

Robot/tool/environment support should be capability-based, not hardcoded per scene.

Initial practical combinations:

- UR5 + Robotiq 2F
- UR5 + suction
- UR3 + suction
- UR10 + 2F
- UR5 + AirPick-style suction where supported
- simple delta/cartesian placeholder + suction later

Future support may include:

- Fanuc
- ABB
- SCARA
- gantry/cartesian
- delta robots
- custom 3D printed robots
- other ROS 2 compatible arms
- tool changers
- custom end effectors

Do not add hardcoded logic that prevents future robot/tool swaps.

Tool behavior should be described through capability/config metadata where practical.

Gripper orientation defaults and offsets must be handled consistently through scene generation and validation. Avoid one-off manual xacro fixes where a generator-level default or capability metadata fix is appropriate.

## Grasp planner and execution rules

The grasp planner and grasp execution path should support simulation before real hardware.

A generated scene should not be considered manipulation-ready unless it can prove:

- planning group is valid
- robot base link is valid
- end-effector link is valid
- tool metadata is valid
- object/task input is available or replayable
- fake-hardware launch path works
- RViz/MoveIt planning scene loads
- simulated plan can be produced where practical
- real execution remains guarded

For grasp planner configuration, prefer generated or scene-local config that can be inspected and edited.

Do not force users to manually hunt through unrelated config files when Workcell Studio already knows the robot, tool, camera, zones, and task intent.

## ROS and platform rules

Current supported baseline:

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 / RViz as the primary planning and visualization foundation

Do not migrate to a newer ROS 2 distribution unless explicitly requested.

Do not make Gazebo, Ignition/Gazebo Sim, or Isaac Sim mandatory for the core workflow yet.

Gazebo can be added later for ROS-native simulation, conveyors, simple physics, and sensor simulation.

Isaac Sim can be added later for investor-grade visuals, synthetic data, and photorealistic digital twin demos.

The core 3-month path should remain RViz/MoveIt-first.

## Testing and validation commands

For builder changes, run at minimum:

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder
source install/setup.bash
```

For generated scene validation, run:

```bash
cd /home/user/workcell_ws/src/easy_manipulation_deployment
python3 scripts/validate_builder_generated_scene.py scenes/ur5_2f_test --json
```

For fake-hardware launch validation, prefer:

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

When validating a generated scene package, use the scene name explicitly:

```bash
cd /home/user/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select <scene_package>
source install/setup.bash
ros2 launch <scene_package> demo.launch.py use_fake_hardware:=true launch_rviz:=true
```

When validating builder-generated scene artifacts, prefer:

```bash
cd /home/user/workcell_ws/src/easy_manipulation_deployment
python3 scripts/validate_builder_generated_scene.py scenes/<scene_name> --json
```

When touching broader generation logic, validate more than one scene.

Useful validation targets include:

```bash
python3 scripts/validate_builder_generated_scene.py scenes/ur5_2f_test --json
python3 scripts/validate_builder_generated_scene.py scenes/suction_test --json
python3 scripts/validate_builder_generated_scene.py scenes/ur3_suction_test --json
python3 scripts/validate_builder_generated_scene.py scenes/ur10_2f_test --json
python3 scripts/validate_builder_generated_scene.py scenes/ur5_airpick4_test --json
```

If a script does not exist or a scene is missing, do not invent success. Report the blocker clearly and propose the smallest fix.

## PR rules

Use small incremental PRs.

Each PR should:

- target one repo only unless explicitly required
- use a branch name like `codex/<short-task-name>`
- avoid broad refactors
- preserve ROS 2 Humble compatibility
- preserve current working demos
- preserve safety gates
- include tests or validation commands
- update docs if user-facing behavior changes
- include a clear PR summary
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
2. Keep the task scoped.
3. Do not perform broad cleanup unless requested.
4. Prefer adding tests/validators for regressions.
5. Prefer clear error messages over silent fallback.
6. Preserve existing working scenes.
7. Do not hide failures by marking broken scenes as unsupported unless explicitly requested.
8. Do not claim success without validation evidence.
9. If blocked, explain the exact blocker and smallest next step.
10. Keep changes practical for fast PR iteration.

## Current development strategy

Prefer this sequence:

1. Stabilize current build and validators.
2. Add or improve all-scenes reproducibility reporting.
3. Clean Workcell Builder UX and remove no-op primary actions.
4. Fix scene save/load persistence.
5. Improve 3D canvas selection, inspector editing, and transform round-tripping.
6. Validate fake-hardware RViz/MoveIt launch for supported scenes.
7. Add or improve grasp planner/execution smoke paths.
8. Harden EPD/RealSense bridge with replay/live modes.
9. Add advanced simulation backends later.
10. Add guarded real-hardware commissioning only after simulation is reliable.

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
- claim simulation or execution works unless it is validated
- treat one passing demo scene as proof that the platform is stable
- add placeholder buttons to the main UI
- hide missing files or broken metadata behind vague success messages
- move business-critical scene state into random UI-only files
- make perception own task planning or scene generation

## Definition of done

A change is only done when:

- the requested scope is complete
- the relevant files are updated
- the change is validated with the smallest meaningful command set
- safety defaults are preserved
- user-facing behavior is documented when needed
- PR summary explains what changed and why
- failures or skipped validations are clearly stated

For Workcell Studio, “done” means the change moves the platform toward:

```text
reliable Workcell Builder
+ repeatable scene generation
+ true 3D scene editing
+ RViz/MoveIt fake-hardware simulation
+ grasp planner/execution readiness
+ clean EPD/RealSense bridge
+ guarded real-hardware future
```
