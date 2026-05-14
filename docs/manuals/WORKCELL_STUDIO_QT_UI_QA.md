# Workcell Studio Qt UI Visual QA Checklist

## Build and run

```bash
cd ~/workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select workcell_builder
source install/setup.bash
workcell_builder
```

## Visual polish checklist

- Dark digital-twin theme is loaded with readable contrast.
- Spacious cards are visible for dashboard, scene overview, selected scene, readiness, and preview command console.
- Left navigation shows: Dashboard, New Cell, Scene Builder, Existing Scenes, Scenario Templates, Asset Browser, Demo Mode, Preview Launch, Validation, Export.
- Top command bar shows: New Cell, Open Scene, Validate, Demo Mode, Preview Launch, Generate Scene, Export, Full Screen.
- Right safety indicator is visible: **Fake Hardware | No Robot Motion**.
- Status badges/tokens are visible in UI copy: READY, PASS, WARNINGS, BLOCKED, PREVIEW_ONLY, ACCEPTED, BUILD REQUIRED.

## 16:9 presentation checklist

- Enter full screen and confirm **Press Esc to exit full screen** message is visible.
- Dashboard table remains readable at 16:9.
- Scene Builder preview panel keeps digital twin preview hierarchy.
- Demo Mode cards and command actions stay clear and uncluttered.
- Preview Launch presents a safe console with clear stop control and fake-hardware banner.

## Investor demo screenshot checklist

Capture screenshots for:
1. Dashboard overview
2. Scene Builder digital twin preview and inspector
3. Demo Mode readiness summary
4. Preview Launch safe console

Each screenshot should visibly include:
- Fake Hardware safety language
- No Robot Motion safety language
- Current selected scene/status context
- Command/readiness information
