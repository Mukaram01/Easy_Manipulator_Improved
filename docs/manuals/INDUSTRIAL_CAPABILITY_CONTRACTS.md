# Industrial Capability Contracts (Offline)\

This manual defines an offline capability layer for heterogeneous robotic cells.

## Why capability contracts exist

The platform is moving from scene-specific examples toward a fully customisable industrial cell builder. Capability contracts separate **what a cell component can do** from scene/runtime implementation details. This allows future tooling to compose cells for different robot families, tools, sensors, tasks, and environment assets without hard-coding around UR5 + Robotiq + table + box assumptions.

These contracts are intentionally offline-first metadata. They do not alter ROS launch flow, MoveIt planning behavior, grasp execution runtime behavior, perception runtime behavior, or existing GUI/runtime tools.

## Contract families

All contracts include `schema_version` and use versioned identifiers:

- `robot_capability/v1`
- `end_effector_capability/v1`
- `sensor_capability/v1`
- `task_capability/v1`
- `environment_asset/v1`

Shipping Workcell Studio catalog lives under `catalog/capabilities/` and is the default capability registry for offline tooling.
Regression fixtures remain under `tests/fixtures/capabilities/` for backward-compatible tests.

---


## Capability catalog layout

`catalog/capabilities/` is now the shipping Workcell Studio catalog path:

- `catalog/capabilities/robots/`
- `catalog/capabilities/end_effectors/`
- `catalog/capabilities/sensors/`
- `catalog/capabilities/tasks/`
- `catalog/capabilities/environment_assets/`

`tests/fixtures/capabilities/` remains a regression-fixture set for validator and tooling tests.

The capability loader scans recursively, so nested folders like `catalog/capabilities/robots/*.yaml` and legacy flat fixtures like `tests/fixtures/capabilities/*.yaml` are both supported.

All capability files remain offline metadata only and do not alter ROS 2 launch, MoveIt planning, grasp execution, EPD, or robot-motion runtime behavior.

---

## 1) robot_capability/v1

Key fields:

- `schema_version`
- `robot.id`, `robot.label`, `robot.brand`
- `robot.family` (`serial_6_axis`, `collaborative_6_axis`, `scara`, `delta`, `gantry`, `mobile_manipulator`, `custom`)
- `robot.mounting` (`floor`, `table`, `wall`, `ceiling`, `overhead`, `custom`)
- planning groups, base/tool frames, default tool frame
- named targets, payload, reach/workspace description
- controller type
- supported interfaces (`moveit`, `ros2_control`, `joint_trajectory_controller`, `cartesian_path`, `named_targets`)
- supported task families
- limitations/warnings

Examples:

- UR5 serial 6-axis: `tests/fixtures/capabilities/robot_ur5.yaml`
- Generic delta: `tests/fixtures/capabilities/robot_generic_delta.yaml`
- Generic SCARA: `tests/fixtures/capabilities/robot_generic_scara.yaml`
- Generic gantry: `tests/fixtures/capabilities/robot_generic_gantry.yaml`

### Delta robot support

`robot.family: delta` + overhead mounting + high-speed/conveyor task family metadata allows future generators to pick delta-oriented defaults (short cycle windows, top-down tools, conveyor targeting) without changing runtime today.

---

## 2) end_effector_capability/v1

Key fields:

- `id`, `label`
- `family` (`finger_gripper`, `three_finger_gripper`, `suction`, `vacuum_array`, `magnetic`, `tool_changer`, `custom`)
- compatible robot families
- required frames, grasp frames
- contact links, allowed touch links
- max object mass and opening width/suction area
- supported grasp strategies (`pinch`, `enveloping`, `top_down_suction`, `side_pick`, `magnetic_pick`, `custom`)
- required IO signals
- release behaviour
- limitations/warnings

Examples:

- Robotiq 2F: `tests/fixtures/capabilities/ee_robotiq_2f.yaml`
- Robotiq 3F: `tests/fixtures/capabilities/ee_robotiq_3f.yaml`
- OnRobot Airpick style suction: `tests/fixtures/capabilities/ee_airpick_suction.yaml`
- Generic vacuum array: `tests/fixtures/capabilities/ee_vacuum_array.yaml`
- Generic magnetic gripper (manual example):

```yaml
schema_version: end_effector_capability/v1
end_effector:
  id: generic_magnetic_head
  label: Generic Magnetic Gripper
  family: magnetic
  compatible_robot_families: [gantry, serial_6_axis]
  required_frames: [tool0]
  grasp_frames: [magnet_face]
  contact_links: [magnet_face]
  allowed_touch_links: [magnet_face]
  max_object_mass_kg: 3.0
  supported_grasp_strategies: [magnetic_pick]
  required_io_signals: [magnet_enable]
  release_behaviour: demagnetize_then_clear
  limitations_warnings: [Ferromagnetic-only payloads]
```

---

## 3) sensor_capability/v1

Key fields:

- `id`, `label`
- `family` (`rgbd_camera`, `2d_camera`, `depth_camera`, `barcode_reader`, `force_torque`, `proximity`, `custom`)
- frames, topics, detection outputs
- supported object attributes (`class`, `colour`, `shape`, `pose`, `size`, `confidence`, `barcode`)
- mounting (`fixed`, `wrist`, `overhead`, `custom`)
- calibration requirements
- limitations/warnings

Examples:

- Intel RealSense D435i: `tests/fixtures/capabilities/sensor_realsense_d435i.yaml`
- Generic overhead RGBD: `tests/fixtures/capabilities/sensor_overhead_rgbd.yaml`
- Generic wrist camera (manual example):

```yaml
schema_version: sensor_capability/v1
sensor:
  id: generic_wrist_cam
  label: Generic Wrist Camera
  family: 2d_camera
  frames: [wrist_camera_link, wrist_camera_optical_frame]
  topics: [/wrist/image_raw]
  detection_outputs: [class, pose, confidence]
  supported_object_attributes: [class, pose, confidence]
  mounting: wrist
  calibration_requirements: [hand_eye_extrinsics]
  limitations_warnings: [Field of view changes with wrist pose]
```

- Barcode reader (manual example):

```yaml
schema_version: sensor_capability/v1
sensor:
  id: industrial_barcode_reader
  label: Industrial Barcode Reader
  family: barcode_reader
  frames: [barcode_reader_link]
  topics: [/barcode/readings]
  detection_outputs: [barcode, confidence]
  supported_object_attributes: [barcode, confidence, pose]
  mounting: fixed
  calibration_requirements: [target_distance_calibration]
  limitations_warnings: [Requires clear line of sight]
```

---

## 4) task_capability/v1

Key fields:

- `task.task_family` (`pick_place`, `sort_by_colour`, `sort_by_shape`, `garbage_sorting`, `machine_tending`, `palletising`, `inspection_routing`, `conveyor_sorting`, `custom`)
- required robot capabilities
- required end-effector capabilities
- required sensor attributes
- required destinations
- rule model
- expected validation checks
- runtime requirements
- limitations/warnings

Examples:

- Simple pick/place: `tests/fixtures/capabilities/task_pick_place.yaml`
- Colour sorting: `tests/fixtures/capabilities/task_colour_sorting.yaml`
- Shape sorting: `tests/fixtures/capabilities/task_shape_sorting.yaml`
- Garbage sorting: `tests/fixtures/capabilities/task_garbage_sorting.yaml`
- Conveyor sorting: `tests/fixtures/capabilities/task_conveyor_sorting.yaml`
- Machine tending (manual sketch): same schema with `task_family: machine_tending`, destination stations, and door/chuck IO requirements.

---

## 5) environment_asset/v1

Key fields:

- `asset.id`, `asset.label`
- `asset.family` (`table`, `workbench`, `bin`, `tray`, `conveyor`, `fixture`, `machine`, `safety_fence`, `camera_mount`, `custom`)
- dimensions, frame, pose
- collision behaviour
- movable/static/attachable state
- import source and mesh path
- limitations/warnings

Examples:

- Table: `tests/fixtures/capabilities/asset_table.yaml`
- Conveyor: `tests/fixtures/capabilities/asset_conveyor.yaml`
- Bin: `tests/fixtures/capabilities/asset_bin.yaml`
- CNC/machine tending fixture: `tests/fixtures/capabilities/asset_machine_fixture.yaml`
- Tray (manual example): `family: tray`
- Safety fence (manual example): `family: safety_fence`

---

## Offline validator and checker

Validator:

```bash
python3 scripts/validate_capability_contracts.py catalog/capabilities
python3 scripts/validate_capability_contracts.py catalog/capabilities --json
python3 scripts/validate_capability_contracts.py tests/fixtures/capabilities
python3 scripts/validate_capability_contracts.py tests/fixtures/capabilities --strict
```

Checker wrapper:

```bash
./scripts/check_capability_contracts.sh
```

Preflight integration:

```bash
./scripts/preflight_workcell.sh
```

This stage remains offline and metadata-only.

## How future GUI and generators will use these contracts

Planned direction:

1. GUI catalogs load robot/tool/sensor/asset/task capability records.
2. GUI constrains options using compatibility fields (e.g., tool family vs robot family).
3. workcell_builder emits selected capability IDs into generated workcell metadata.
4. Generated project validators ensure required capability combinations are complete before launch.
5. Runtime adapters can then map validated capabilities to concrete planners/controllers/drivers.

This keeps the **physical robotic cell** as the commercial product while software remains the configurable engineering pipeline used to define, validate, and commission that cell.

## Cell-definition integration

Capability contracts can now be referenced directly by `cell_definition/v1` tooling. This enables offline compatibility hints between:
- robot family and supported task families
- end-effector compatibility with robot family
- task requirements and selected sensor attributes
- optional environment asset references

This is intentionally metadata-only and does not alter runtime launch/planning/perception behavior.

## Capability contracts + asset manifests + environment layouts

- Capability contracts describe **what an asset/component can do**.
- `assets/` folders store the physical meshes/models/descriptions.
- `asset_manifest.yaml` provides optional sidecar metadata for existing assets.
- `environment_layout/v1` describes where those existing assets are placed in a specific cell.

This separation prepares future GUI import/export workflows while keeping runtime behavior unchanged in this phase.


## Cell definition validation examples

```bash
python3 scripts/validate_cell_definition.py tests/fixtures/cell_definition_sort_by_colour.yaml
python3 scripts/validate_cell_definition.py tests/fixtures/cell_definition_sort_by_colour.yaml --capabilities-dir catalog/capabilities
python3 scripts/validate_cell_definition.py tests/fixtures/cell_definition_sort_by_colour.yaml --capabilities-dir tests/fixtures/capabilities
```
