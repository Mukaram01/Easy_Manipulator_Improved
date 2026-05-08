# Workcell Builder User-Friendly GUI

The Workcell Builder GUI now follows a guided Workcell Studio workflow:
Start → Ingredients → Layout → Task → Perception ROI → Grasp → Validate & Generate → Status.

- **Start**: New/Open/Save cell, set cell name and output folder.
- **Ingredients**: choose robots, grippers/tools, cameras/sensors, environment assets, and objects.
- **Layout**: place assets and edit XYZ/RPY, role, dimensions, and collision mode.
- **Task**: pick/place and sorting, with clear **Preview only — no runtime launch yet** messaging for unsupported runtime flows.
- **Perception ROI**: shown as **Camera Pick Area / Pointcloud Crop Box** with plain boundaries.
- **Grasp**: auto/finger/suction strategies with approach-retreat and TCP offset guidance.
- **Validate & Generate**: grouped actions for validation, canonical exports, package output, studio pack, preview/report commands.
- **Status**: safety-first banner shows fake hardware default.

## Recommended Starter Set
- UR5
- Robotiq 2F
- Workbench/Table
- Cube Small
- Small Bin
- RealSense D435i visual asset

## Search/Filter
Use the search box in the asset browser and type values like `ur`, `fanuc`, `bin`, `suction`, `conveyor`, `camera`, `box`.

## EPD Separation
EPD controls are intentionally not part of workcell_builder. EPD remains a separate application boundary.

## Fake-Hardware First
Default launch command guidance is safe simulation (fake hardware). Real hardware launch is not default.
