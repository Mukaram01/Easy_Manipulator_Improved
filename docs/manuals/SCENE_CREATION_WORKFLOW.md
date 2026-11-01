# SCENE_CREATION_WORKFLOW

This manual documents the **current intended flow** for creating/importing/editing environments using the existing repository tooling.

## Scope and boundaries

- `workcell_builder` remains the source GUI for scene creation/editing and task authoring (`pick_place`).
- `scenes/` stores scene packages/files.
- `assets/` stores physical meshes/models/descriptions.
- `environment_layout/v1` is metadata/placement helper only.
- `cell_definition/v1` describes intended cell/task contract.
- No duplicate scene system is required.

## Where cells are saved (Git policy)

- **New cells:** New Cell saves every cell in the repository's `scenes/<name>/` folder. Workcell Builder lists, edits (3D Scene Builder), generates, validates and simulates cells there.
  - **Other folders:** refused with the reason. They are never redirected.
  - **Existing names:** blocked, never overwritten.
  - **Older folders:** cells created in an earlier custom output folder stay listed.
- **Not in Git:** new cells are **local to this PC**. Git ignores everything under `scenes/` except the curated reference library listed in `.gitignore`, so a new cell is never uploaded to GitHub and does not sync to other computers. To use it on another PC, copy its folder or export it.
- **Scene-local assets:** a cell's own assets are saved inside it, and its generated package resolves them inside the cell, whether or not Git tracks them. Examples:
  - the test-object visuals that New Cell > Sorting copies;
  - meshes that Save copies when an asset is placed: imported CAD goes to `assets/imported/catalog/`, and meshes from the tracked repository library go to `assets/library/`.
- **Imported CAD:** Import Mesh adds a mesh to this PC's imported-asset library (`workcell_builder/workcell_builder/assets/imported/`). The library holds the files and their one catalog, `imported_environment_assets.json`, and Git ignores both, so customer or proprietary CAD is never committed by default. Every cell's Asset Library lists the library, and Save copies a placed mesh into the cell's `assets/imported/`.
  - **Ignored in every scene:** Git ignores `assets/imported/` in every scene, reference scenes included.
  - **Reference scenes:** a reference scene that uses an imported part shows a `Not portable` validation warning until the part is promoted with licence approval (`WORKCELL_EXTERNAL_ASSET_IMPORT_WIZARD.md`). See `WORKCELL_EXTERNAL_ASSET_IMPORT_WIZARD.md`.
- **Tracked:** the core library (`assets/`, robot/tool/environment descriptions, `catalog/`) and the reference scenes. A reference scene depends only on tracked files, so a GitHub checkout works without copying assets.
- **Promoting a cell to a reference scene:** a deliberate review. Add its `!scenes/<name>/` line to `.gitignore`, make sure every asset it references is tracked and licensed, then commit it.

## Practical workflow (current)

1. Open or create a workcell project.
2. Use existing `workcell_builder` GUI.
3. Use the existing `scenes/` directory structure.
4. Import STL/mesh through existing builder flow.
5. Add visual geometry for each imported object.
6. Add collision geometry (required for practical planning safety).
7. Place assets in `world` frame (or intended parent link frame).
8. Add robot package/model to the scene.
9. Add gripper/end effector configuration.
10. Add camera/sensor frame and model references.
11. Save scene package.
12. Author task intent in `workcell_builder`: select pick source, place target, grasp/release strategy, and routing rule.
13. Export generated task intent (`generated/workcell_builder_task_intent.yaml`) from builder-authored selections.
14. Validate scene readiness (`scripts/check_scene_readiness.py`).
15. Optionally generate/preview cell definition artifacts.
16. Simulate.
17. Commission/deploy in later stages.

## Choosing the robot and tool (New Cell)

New Cell lists the robots and tools of the capability catalog (`catalog/capabilities/robots`, `end_effectors`) and checks each against the models installed under `assets/`. The wizard runs `scripts/instantiate_workcell_studio_profile.py --equipment` and shows the result.

**Readiness of a robot + tool pair.** The tool list names it, and the Robot and End Effector pages explain it:

| Status | Meaning | New Cell |
|---|---|---|
| **PLAN_CAPABLE** | A reviewed New Cell profile (`catalog/workcell_studio_profiles/*.yaml`) binds this exact pair to its MoveIt and controller configuration. | Creates the cell from the profile. Generate, Validate and fake-hardware Plan Only are available. |
| **CONFIGURABLE** | Both models are installed and the tool declares the robot's family compatible, but no reviewed profile binds them. | Creates an authoring-only cell: equipment identities and frames, no task. Generate and Plan / Simulate are blocked with this reason (`EQUIPMENT_NOT_PLAN_CAPABLE`). |
| **UNCOMMISSIONED** | The catalog lists the equipment, but its model is not installed in this workspace. | Create is disabled; the reason names what is missing. |
| **UNSUPPORTED** | The tool does not declare the robot's family compatible, or the record is preview-only. | Create is disabled; the reason names the capability record. |

Catalog presence never implies runtime readiness, and real hardware stays locked for every pair. A new pair never inherits G4/G6 qualification.

**Frames and Advanced overrides.**
- **Defaults:** frames, planning group and controller come from the reviewed profile when the pair has one. Otherwise they come from the capability records and the installed model: SRDF groups and chains, and the tool macro's root link.
- **Advanced overrides:** Advanced Robot/Tool Frames accepts only values the record or the installed model declares. A typo blocks Create and names the valid choices.
- **Saved:** the cell stores the capability ids (`workcell_studio.robot.capability`, `workcell_studio.tool.capability`) and `workcell_studio.equipment` (status, reason, profile).

A new installed model without a capability record is listed as not selectable.

## Giving a new cell its object input (Plan Only)

A new cell plans only from measured object observations. Configure them in **Scene Builder › Task › Object input**; no YAML editing is needed.

1. **Object class:** under *What / Pick eligibility*, type the class to pick, for example `cup`. Generate refuses observation input without one, and the Object input status says so.
2. **Perception mode:**
   - **Recorded observations (replay):** plans from a saved `detected_objects/v1` recording (`source.mode: replayed_snapshot`). No camera is needed. Use this for Plan Only on fake hardware.
   - **Live EPD camera observations:** plans from a capture of the selected camera (**Capture live EPD objects**). The capture must declare that camera, give world-frame poses and still be fresh when planning starts. A recording is never used instead.
   - **Off — simulator-measured objects (physics only):** the physics simulator measures the objects. That simulator is locked for new cells, and fake-hardware planning refuses a recording in this mode.
3. **Existing input:** lists the recordings, or the saved EPD captures in Live mode, found in the cell and in the repository's `config/runtime/`. The reviewed commissioning recording is `config/runtime/r1_4_replay.yaml`: a cup and a bottle in the UR5 + 2F workbench pick zone. **Choose observation file…** selects any other file.
   - **Stored path:** a file of the same repository is stored relative to the cell.
4. **Status box:** below the chooser, it shows:
   - the input's declared source and objects;
   - its freshness;
   - whether it passes the planner's input checks: `planning_input`, plus the executor's producer and freshness admission (`task_intent_v2.observation_age_cap`);
   - the next step.
5. **Maximum age:** switching to a mode with a lower freshness limit lowers a larger maximum observation age to that limit, and the status says so. A recording's limit is 2 s; it is re-timed when planning starts.
6. **Save task**, Generate, then Validate. Plan / Simulate › **Plan Saved Task (Fake Hardware)** is enabled only when the saved input passes those checks. Otherwise it shows the reason and what to do.

Objects are never derived from scene assets. The Sorting wizard's simulated test objects are not perception input.

## Choosing where the robot places (visual destination binding)

A task's destination is a **physical target** (a bin or other collision-modelled asset) together with one of
its **placement regions**. A region is reviewed free space in the target's own frame (`placement_local`), so it moves
and rotates with its target; it is never a separately movable bin. The **place pose** is where the planner puts the
object inside that region (AUTO: the region centre, or a free place per object in a sequence; PREFERRED/EXACT: the
requested target-local pose).

1. **Select the bin** in the 3D view. **Inspector › Task destination** shows:
   - its name, ID and world pose;
   - its placement regions;
   - whether the task places there, with the resolved place pose in world.
2. **Use Selected as Task Destination** sets the task's physical target, and its region when the target has exactly
   one, in the TaskIntent draft. Nothing generated is written. **Save task**.
3. **Task › Where** shows the destination as the validator resolves it:
   - the physical target's world pose;
   - the region's target-local offset and size;
   - the place pose, in target-local and world coordinates.

   **Show physical target in 3D** selects the target in Product View.
4. **Moving the bin:** move or rotate it in the 3D view and **Save**. The region follows the target. The task's
   resolved place pose updates at once, and Generate/Validate become stale. Before Save, both panels say the
   coordinates are the saved ones.
5. **Generate, Validate, then Plan / Simulate › Plan Saved Task.** MoveIt plans to the new place pose with the moved
   bin in its collision scene.

A target without a placement region, a region of another target, or stale geometry blocks the task. The block names
the code (`PLACE_REGION_TARGET_MISMATCH`, `R19_DESTINATION_INVALID`, ...) and the next step. Nothing is moved or
substituted to make a plan pass, and an unreachable place fails in the collision-aware MoveIt plan.

## Relationship between metadata layers

- `environment_layout/v1`: placement/zones checklist metadata for operators.
- `cell_definition/v1`: high-level cell intent (robot/tool/sensor/task).
- `task_recipe/v1`: offline task logic intent (`pick_place`, `sort_by_colour`, `garbage_sorting`, etc.).
- Scene package (`scenes/<name>`): concrete files used by existing workflow and validation.

## Helper commands

```bash
python3 scripts/report_workcell_builder_paths.py
python3 scripts/check_scene_readiness.py --workcell-root .
python3 scripts/generate_scene_import_checklist.py path/to/fixture.stl
python3 scripts/environment_layout_to_scene_checklist.py tests/fixtures/environment_layouts/ur5_table_bins_existing_assets.layout.yaml
```

## Create your own environment using cell definitions

Use the existing Workcell Builder + generation flow (no duplicate builder/system).

1. Start from `cell_definitions/demo_ur5_sorting_cell.yaml`.
2. Edit robot/base scene metadata, table/bin/box/camera frames, and task destinations/rules.
3. Validate:
   - `python3 scripts/validate_cell_definition.py --cell-definition cell_definitions/demo_ur5_sorting_cell.yaml --json`
4. Generate staged workcell package:
   - `python3 scripts/generate_workcell_from_cell_definition.py --cell-definition cell_definitions/demo_ur5_sorting_cell.yaml --output-dir /tmp/generated_workcells/demo_ur5_sorting_cell --json`
5. Run preflight and gated dry-run with generated artifacts (no robot motion required).
6. Only after offline checks pass, continue with simulation/replay commissioning steps.

## Do not do this

- Do **not** copy the same mesh into multiple folders unnecessarily.
- Do **not** create duplicate scene package names.
- Do **not** hardcode absolute `/home/user/...` paths into reusable scenes.
- Do **not** create a new top-level scene format outside existing `workcell_builder`.
- Do **not** bypass collision geometry.

## Runtime boundary

This workflow/manual improves authoring readiness only.

It does not change runtime ROS 2, MoveIt, grasp, perception, or controller behavior.

## Generate and test your own workcell bundle
Use `generate_workcell_from_cell_definition.py` then run `run_generated_workcell_bundle.py --gated-dry-run --json` to execute preflight + gated dry-run with generated defaults only; robot motion remains disabled.

## Visual preview of a generated workcell

Use `python3 scripts/preview_generated_workcell_bundle.py --workcell <path> --json` to produce `generated/visual_preview_summary.json`, then optionally publish markers to `/generated_workcell/markers` for RViz checks of tables, bins, objects, destinations, and task pick/release highlights. Motion remains disabled.

## Visual task-flow preview
1. Generate the workcell bundle.
2. Run gated dry-run (`run_generated_workcell_bundle.py --gated-dry-run --json`).
3. Open visual preview (`preview_generated_workcell_bundle.py --show-task-flow --task-flow-preview <dry-run-output>/task_flow_preview.json --json`).
4. Review selected object, pick pose, destination, and release pose in the trace.
5. Check warnings/blockers before any future execution workflow.
6. This is dry-run intent preview only (`safe_for_robot_motion=false`) and does not move the robot.
7. Future milestone: MoveIt simulation trajectory preview.


## Studio Lite

Studio Lite (`scripts/studio_lite.py`) is the first operator workflow window for generated bundles. It reuses existing scripts and does not replace Workcell Builder.

- Uses `validate_cell_definition.py` and `generate_workcell_from_cell_definition.py` for generation.
- Uses `preview_generated_workcell_bundle.py` for JSON and marker preview.
- Uses `run_generated_workcell_bundle.py --gated-dry-run --json` for safe dry-run checks.
- Shows report paths, blockers, warnings, and selected object/destination fields.
- Keeps robot motion disabled (`safe_for_robot_motion: false`).

Use Studio Lite for generate/preview/preflight/report inspection while backend authoring stays in Workcell Builder.


## Developer Studio vs Operator Runtime

- **Developer Studio (developer/integrator role):** validate cell definitions, generate workcell bundles, preview environment/task flow, run preflight and gated dry-runs, inspect reports.
- **Operator Runtime (operator role):** load generated bundles (including approved bundles), view summaries, preview environment/task flow, run preflight and gated dry-runs, inspect warnings/blockers/reports.
- Operator Runtime does **not** expose editing or generation controls and never enables robot motion/replay/controller publishing.
- Both roles use the same generated bundle format and the same backend scripts (no duplicate scene system, no second planning backend).
- Bundle approval is an internal workflow marker only, not a safety certification. Physical robot execution remains disabled.

## Scenario packs
See `docs/manuals/INDUSTRIAL_SCENARIO_PACKS.md` for generated-cell scenario validation flows.


## Persisting placed objects to real generated scenes
1. Add/import STL object.
2. Open STL preview.
3. Open interactive RViz preview.
4. Drag/rotate object.
5. Import RViz Pose Feedback.
6. Review/apply valid updates.
7. **Save Placed Objects to Scene YAML**.
8. Generate YAML / Generate Files.
9. Build scene package.
10. Launch generated scene in RViz/MoveIt fake hardware mode.

Preview files are temporary. `environment.yaml` is the scene persistence point. Real scene files are updated only by explicit save/generate actions. This workflow introduces no controller execution and no real hardware execution.

## Placed object end-to-end generation

1. Import an STL file in Object Placement Manager (or select an existing managed STL asset).
2. Confirm the STL is stored in the managed asset path and place it into the scene with initial XYZ/RPY pose values.
3. Open RViz STL Preview to generate temporary preview artifacts under `/tmp/workcell_builder_preview/<scene_name>/...`.
4. Optionally refine object pose in interactive RViz preview and export feedback to `placed_objects_feedback.yaml` in the same temporary preview location.
5. Import RViz pose feedback into Object Placement Manager and review/apply valid pose updates by object name.
6. Save placed objects to scene YAML so `environment.yaml` is updated as the persistence source of record.
7. Run Generate YAML / Generate Files to regenerate scene artifacts from persisted data.
8. Confirm generated placed-object URDF/Xacro outputs are produced; these generated URDF/Xacro files are the runtime source consumed by RViz/MoveIt visualization flows.
9. Build the generated scene package/workspace so runtime launch files resolve the updated generated artifacts.
10. Launch the demo/preview runtime (typically fake hardware mode) and verify the placed objects appear correctly in RViz/MoveIt.

**Important runtime boundary**

- Preview files are temporary and live under `/tmp/workcell_builder_preview/...`; they are not persistent scene truth.
- `environment.yaml` is the persistence source for placed object data.
- Generated URDF/Xacro is the runtime source used by RViz/MoveIt after generation/build.
- This feature does not enable robot motion, controller execution, or real hardware execution.



## Camera placement and frustum preview
1. Add camera.
2. Set XYZ/RPY.
3. Save Cameras to Scene YAML.
4. Open Camera Frustum Preview.
5. Generate YAML.
6. Confirm cell_definition.yaml camera block.
7. Later use metadata for EPD/RealSense integration.

This is visual/configuration only: it does not start RealSense hardware, does not start EPD, and does not enable robot motion.
