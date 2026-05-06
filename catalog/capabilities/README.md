# Workcell Studio capability catalog

This directory is the **Workcell Studio product-facing capability registry**.

It ships offline-first metadata used to describe selectable workcell components:

- `robots/`: robot capability records.
- `end_effectors/`: gripper, suction, and tooling capability records.
- `sensors/`: perception and sensing capability records.
- `tasks/`: task-family capability records.
- `environment_assets/`: environment asset capability records.

These files are metadata-only and do **not** change ROS 2 launch, MoveIt planning, grasp execution, EPD flows, or any robot-motion runtime behavior.
