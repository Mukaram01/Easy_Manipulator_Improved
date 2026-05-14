# Workcell Studio Qt UI QA

## Canvas checks
- Select/create scene and open Scene Builder.
- Confirm polished Digital Twin Canvas and overlays.
- Click items and verify inspector updates with id/type/pose/source.
- Verify Fit Cell, Reset View, Zoom In/Out, Toggle Grid/Labels/Warnings.
- Export Canvas Snapshot and confirm SVG output path.
- Confirm Fake Hardware and No Robot Motion badges remain visible.


## Asset-to-canvas loop
- Open Asset Catalog in Scene Builder and add assets directly to the digital-twin canvas.
- Use task binding shortcuts (Use Selected as Pick Source/Place Target/Pick Zone/Place Zone/Camera).
- Save Layout explicitly, then rerun validation/acceptance when `Layout changed since last acceptance` appears.
- Safety contract remains: fake_hardware_first=true, runtime_execution_enabled=false, motion_command_sent=false.
