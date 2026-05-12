# Workcell Builder GUI Acceptance / Self-Test

## Purpose
Use the **real C++/Qt `workcell_builder` executable** to confirm visible Workcell Studio actions, catalog visibility, and GUI readiness controls.

## Run self-test
```bash
QT_QPA_PLATFORM=offscreen workcell_builder --self-test-gui
```

Also supported:
```bash
QT_QPA_PLATFORM=offscreen workcell_builder --gui-acceptance-check
```

This mode is GUI-safe and non-interactive:
- does **not** launch MoveIt
- does **not** launch ROS control/hardware
- does **not** command motion
- exits with code **0** on pass and **1** on failure

## Generated report
The self-test writes:

`/tmp/workcell_builder_gui_acceptance_report.json`

Report fields:
- `checked_actions`
- `checked_catalog_entries`
- `checked_editor_fields`
- `missing_items`
- `pass`
- `timestamp`
- `catalog_path_used`

Inspect with:
```bash
cat /tmp/workcell_builder_gui_acceptance_report.json
```

## Expected visible actions
- Validate Cell
- Generate Canonical Files
- Generate Workcell Package
- Generate Studio Pack
- Open Preview
- Open Output Folder
- Show Readiness Report
- Copy Fake-Hardware Launch Command
- Refresh Asset Catalog

## Expected visible catalog coverage
- Robots: UR3, UR5, UR10, Fanuc, Panda
- Grippers/tools: Robotiq 85/2F, Robotiq 3F, Single Suction, OnRobot AirPick4
- Environment: Table, Workbench, cube/bin/conveyor placeholders
- Sensor: RealSense D435i

`preview_only` entries are acceptable as long as they remain visible in the GUI-facing catalog.

## EPD separation
EPD remains separate from `workcell_builder` GUI acceptance. No EPD launch/control is required for this test.

## Manual validation commands
```bash
cd ~/workcell_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

QT_QPA_PLATFORM=offscreen workcell_builder --self-test-gui
cat /tmp/workcell_builder_gui_acceptance_report.json
```

```bash
cd ~/workcell_ws/src/easy_manipulation_deployment

PYTHONPATH=$PWD:$PYTHONPATH \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q \
  tests/test_workcell_builder_gui_acceptance_self_test.py \
  tests/test_workcell_builder_gui_visible_actions.py \
  tests/test_workcell_builder_gui_visible_catalog_entries.py \
  tests/test_workcell_builder_gui_visible_editor_fields.py \
  tests/test_workcell_builder_gui_launch_commands.py
```

## 2026 compact-layout acceptance additions
- Verify key dialogs fit on 1366x768.
- Verify long path/status text wraps without forcing horizontal growth.
- Confirm no runtime/safety/launch behavior changes; UI-only polish.

- Scene Select now includes a Workcell Studio Scene Status panel with Refresh Status, Validate Scene, Copy Build Command, and Copy Launch Command controls.
