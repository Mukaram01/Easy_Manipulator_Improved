# Workcell Builder Task Intent Readiness

Task Intent Readiness adds an offline preview layer that converts detection mapping, work zones, conveyor preview metadata, robot metadata, and end-effector metadata into `task_intent_preview.yaml/json`.

## Scope
- Preview-only runtime mode (`preview_only`).
- No robot motion execution.
- No MoveIt planning calls.
- No gripper commands.
- No EPD runtime launch.
- No RealSense runtime launch.
- No conveyor hardware command.
- Fake hardware remains the default recommendation.

Future PRs may connect this readiness layer to dry-run planning/fake execution.


## live_epd_feed_bridge update
- Added live EPD feed bridge preview path (metadata-only).
- EPD GUI remains separate.
- No robot/gripper/conveyor command, no MoveIt call, no forced RealSense launch.
