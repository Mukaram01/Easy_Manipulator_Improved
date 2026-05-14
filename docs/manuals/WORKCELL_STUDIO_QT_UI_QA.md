# Workcell Studio Qt UI Manual QA Checklist

This checklist validates the hardened Workcell Studio Qt shell wiring without changing generation/runtime semantics.

## Build and Run

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder
source install/setup.bash
workcell_builder
```

## Click-through checklist

- App opens to **Workcell Studio** shell.
- Dark theme loads; if missing, app stays usable and reports fallback in status/log.
- Dashboard and left navigation appear.
- Open **Full Screen** and confirm **Escape** returns to normal mode.
- Click each visible action and verify behavior:
  - **New Cell**: clear placeholder message appears.
  - **Open Existing Scene**: transitions into existing scene workflow.
  - **Scene Builder**: transitions into existing scene workflow.
  - **Asset Browser**: placeholder message appears.
  - **Scenario Templates**: placeholder message appears.
  - **Validate**: request is logged and no crash.
  - **Preview**: placeholder message appears.
  - **Generate Scene**: request is logged and no crash.
  - **Export**: placeholder message appears.
- Verify placeholder text includes:
  - "This Workcell Studio action is not wired yet. No files changed and no robot motion was commanded."

## Existing workflow preservation checks

After opening Scene Builder workflow:

- Existing scenes are discovered from default scenes folder.
- **Add New Scene** still works.
- **Edit Scene** still works.
- **Generate YAML** still works.
- **Generate Files From YAML** still works.
- Scene generation outputs remain unchanged in intent (URDF/Xacro/launch semantics unchanged).

## Safety checks

- Bottom action/status log updates for user actions (navigation + button clicks).
- Confirm logs include concise action messages and explicit safety note for placeholders.
- Confirm no runtime execution is auto-triggered and no robot motion is commanded.

## Scope note

- Workcell Builder remains the primary Workcell Studio product UI.
- EPD GUI remains separate and is not merged into this shell in this change.
