# Workcell Sim-to-Real and EPD Compatibility

This document defines simulation-to-real metadata only.

- Real hardware is **not enabled** by this update.
- EPD remains external/separate and metadata export-only.
- fake_hardware_first safety flow remains required.

## Required future real hardware setup
- Robot driver requirements (driver package, controller, ros2_control, network).
- Tool I/O mapping requirements (digital output/vacuum/gripper controller).
- Camera calibration requirements (frames/topics/calibration).

## Delta robot status
`generic_delta_robot` is metadata-only unless user provides URDF/MoveIt/driver/kinematics.
Not launch-ready.

## Safety defaults
- fake_hardware_first: true
- real_hardware_enabled: false
- runtime_execution_enabled: false
- motion_command_sent: false
- moveit_plan_service_called: false
