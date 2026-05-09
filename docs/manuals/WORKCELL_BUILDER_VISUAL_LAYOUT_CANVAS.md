# Workcell Builder Visual Layout Canvas

The Workcell Builder **Layout** tab now includes a top-down **Visual Layout Canvas** for manual cell authoring.

## What it is
- A simple 2D planning view for placing assets.
- It is **offline authoring support** and does not replace RViz/MoveIt.
- EPD stays separate.

## How to use
1. Add assets into **Current Cell Assets**.
2. Open **Layout** tab.
3. Use canvas tools:
   - Fit Cell
   - Reset View
   - Toggle Grid
   - Toggle Reach
   - Toggle ROI
   - Snap to Grid
   - Export Layout Preview
4. Move/update assets and regenerate canonical files.

## Visual helpers
- Asset markers and labels are shown for robot, tool, table, bin, object, camera, conveyor and other scene items.
- UR5 gets an approximate reach circle.
- Camera pointcloud ROI appears as a top-down rectangle.

## Validation overlays
- Green/amber/red style cues indicate OK/WARN/FAIL style checks.
- Preview-only assets show warning badges.
- Invalid ROI is highlighted.
- Overlap/bounds checks are lightweight guidance only.

## Safety note
The reach circle and overlays are **not a safety certificate** and do not prove collision-free motion.
Use RViz/MoveIt and standard safety processes for real deployment.
