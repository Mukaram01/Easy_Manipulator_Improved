# Workcell Builder Generation Pipeline

Qt Scene3D in `workcell_builder` is a Debug 3D Preview for inspecting generated packages, not the source of truth for EMD/Workcell Studio package generation. Generated scene files and ROS package outputs remain the backend contract, and RViz/MoveIt remains the planning and visualization truth for simulation validation.

## Pipeline
1. Builder scene + metadata are exported to canonical artifacts:
   - `cell_definition.yaml`
   - `environment_layout.yaml`
   - `task_recipe_from_builder_intent.yaml` (when task intent exists)
   - `selected_assets.json`
   - `compatibility_report.json`
   - `builder_export_summary.json`
2. Workcell Studio import consumes canonical artifacts and emits preview/readiness bundles.
3. Generated project/package stays fake-hardware-first by default.

## Supported vs preview-only
- Supported UR5 paths preserve generated package/runtime behavior.
- Preview-only robots/tools/tasks generate metadata + previews and explicit WARN notes.
- Real hardware commands are not generated as default output.

## Custom STL handling
- Builder object meshes are propagated into `environment_layout.yaml` and `selected_assets.json`.
- Custom STL assets are copied into studio pack output when present.
- Visual-only/placeholder warnings are carried through readiness summaries.

## EPD separation
EPD remains separate by design:
- no EPD GUI panel added to builder
- no EPD launch controls added to builder
- only passive camera/perception metadata is preserved for future adapters

## Safety/readiness limitations
Generation is metadata/export only:
- no robot motion is commanded
- no MoveIt planning service is called
- no real hardware is enabled by default
- generated package/readiness output is not a safety certificate
