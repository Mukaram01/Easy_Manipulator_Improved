# Workcell Curated Asset Library

This repository includes a curated built-in environment asset library for Workcell Studio.

Included assets: table_small, table_large, workbench, bin_small, bin_large, tray, tote_box, conveyor_placeholder, fixture_plate, pallet, pedestal, camera_stand, safety_fence_panel, robot_base_plate, calibration_cube, pick_box, cylinder_object.

Browse in workcell_builder by category groups:
- Tables / Workbenches
- Bins / Trays / Totes
- Conveyors
- Fixtures
- Safety / Fencing
- Camera Mounts
- Robot Bases
- Pick Objects

Placement: select an asset in picker/discovery, then add in Object Placement Manager using default pose/dimensions.

Replacement policy: these are lightweight generated placeholders. Replace with real licensed CAD later by importing user meshes into a workspace asset package.

License/source policy: built-ins are generated_internal or placeholder/user_replaceable only. No proprietary vendor CAD is bundled.

User STL import path recommendation: place imported meshes under a workspace-owned package path (for example, `assets/environment/<your_asset>_description/meshes/`) and keep package discovery flat.

Fake-hardware-first: this library is for offline preview, validation, and scene generation; not for real hardware execution.
