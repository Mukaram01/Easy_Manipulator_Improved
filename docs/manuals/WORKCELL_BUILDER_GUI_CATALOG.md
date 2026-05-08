# Workcell Builder GUI Catalog

- Generate GUI catalog JSON:
  - `python3 scripts/workcell_builder_gui_catalog.py`
  - Output: `workcell_studio_catalog/generated/workcell_builder_gui_catalog.json`
- Renderer metadata command:
  - `python3 scripts/render_workcell_builder_metadata.py --help`

## How Qt GUI loads it
Current integration keeps legacy loaders and uses generated catalog as additive source for robot/gripper/object/environment visibility in selection flows.
If catalog JSON is missing, legacy folder scanning is still used and GUI remains functional.

## Refresh/Rescan
Run catalog generation command again (no rebuild required), then reopen selection dialogs.

## Add new entries
1. Add/modify items in `workcell_studio_catalog/catalog.yaml`.
2. Re-run `scripts/workcell_builder_gui_catalog.py`.
3. Ensure entries include support/runtime status and tags.

## Preview-only
`preview_only` entries are visible for planning/manual authoring, but may not be fully runtime-enabled.

## Known limitation
EPD GUI remains separate and is not merged into workcell_builder.
