# Roadmap

The following items are pending implementation:

1. Dynamic Safety & Visualizer enhancements (distance-based checks — initial minimum-distance reporting added).
2. MoveIt Collision Checker plugin loading and multi-axis joint support.
3. Tesseract Collision Checker plugin mapping and self-collision checks.
4. Replanner parameterization and joint-limit handling improvements.
5. Scheduler workflow prerequisites and queue robustness.
6. Grasp Execution Interface documentation and options expansion.
7. MoveIt-based execution improvements for multi-axis joints and path constraints.
8. Context loading via `rcl_yaml_param_parser` with schema validation.
9. Demo node lifecycle conversion.
10. Finger-gripper sampling strategies for multi-finger configurations.
11. Additional testing, CI workflows, documentation, and Docker support.

These tasks are outlined for future contributions.

## Completed

- Gripper driver interface hardened with explicit result codes
- End effector execution context supports optional delays for attach and detach operations.
- Demo node grasp method selection.
- Default executor watchdog and graceful shutdown.
