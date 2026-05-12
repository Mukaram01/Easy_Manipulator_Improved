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


## Planning readiness follow-up
Task intent preview can now be validated with dry-run planning readiness artifacts. This remains non-executing: no robot motion, no MoveIt execute call, no gripper/conveyor command.


## Class-to-Place-Zone Routing
Preview-only class-to-place-zone routing maps EPD class labels from snapshot/live bridge to destination place zones, with unknown/default typically routed to reject bin. No robot motion, no MoveIt execution, no gripper or conveyor hardware commands, EPD GUI remains separate, real hardware later.


## EMD Grasp Request Contract (preview-only)
- EPD provides perception only.
- Existing EMD planner remains downstream planner.
- Workcell Studio now emits grasp_strategy.yaml, emd_grasp_planner_request.yaml/json, and readiness reports.
- No planner execution or robot motion is called in this stage.
