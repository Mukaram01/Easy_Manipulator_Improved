# Workcell Builder EPD Adapter (Offline Snapshot Contract)

This adapter adds an offline EPD-compatible detection snapshot schema for Workcell Studio preview.

- EPD GUI remains separate.
- No EPD runtime is launched.
- No RealSense driver is launched.
- No robot motion is commanded.
- No MoveIt planning is called.
- No conveyor hardware is commanded.

Runtime mode is `adapter_metadata_only` for snapshot input and `preview_only` for mapping output.

## Task Intent Readiness note
This layer is preview/readiness only and does not execute robot motion, MoveIt planning, or gripper runtime commands.
