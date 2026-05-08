# Workcell Builder GUI: Validate, Generate, Preview

This workflow keeps manual authoring in `workcell_builder` as the primary flow.

## Validate Cell
Use **Validate Cell** to check required ingredients: robot, tool, support surface, pick area, place target, grasp strategy, ROI validity, compatibility, preview-only flags, and fake-hardware default.

## Generate Canonical Files
Use **Generate Canonical Files** to export:
- `cell_definition.yaml`
- `environment_layout.yaml`
- `task_recipe.yaml`
- `selected_assets.json`
- `compatibility_report.json`
- `builder_export_summary.json`

## Generate Workcell Package
When supported and fake-hardware-ready, generate package scaffolding and launch/config artifacts with fake hardware default.
Preview-only combinations remain metadata-only and should show a preview-only runtime warning.

## Generate Studio Pack
Use **Generate Studio Pack** to produce canonical files, readiness summaries, HTML/SVG preview, launch command notes, and compatibility metadata.

## Open Preview / Output Folder
- **Open Preview** opens generated HTML/SVG if available.
- **Open Output Folder** opens the most recent output directory.

## Readiness Report
Readiness should summarize final status (`OK/WARN/FAIL`), selected ingredients, runtime eligibility, preview-only warnings, and safety notes.

## Copy Fake-Hardware Launch Command
Use **Copy Fake-Hardware Launch Command** to copy only safe fake-hardware launch commands by default.
Preview-only cells should return: `No runtime launch command available for preview-only cell`.

## Safety and separation
- Real hardware is never the default.
- EPD remains separate.
- No EPD controls, RealSense launch, or real hardware drivers are added to workcell_builder GUI.
