# Workcell Studio Industrial Scenario Catalog

Workcell Studio is currently **preview/fake-hardware-first**. This catalog tracks real industrial scenarios and readiness.

## Scope and constraints

- This catalog is the source of truth for scenario readiness and missing capabilities.
- Live EPD feed is required before claiming full perception scenario support.
- Real hardware readiness is tracked separately and is later-phase work.
- EPD GUI remains a separate application and is not merged into Workcell Builder.
- Current EPD adapter support in Workcell Studio is offline/snapshot-compatible.

## Scenario list

### static_table_pick_place
- Static table pick and place.
- Preview-ready using snapshot detections and task intent preview.
- Needs live EPD feed before full perception scenario support.

### conveyor_upstream_detection_downstream_pick
- Conveyor upstream detection, downstream pick.
- Preview-ready for layout, zones, conveyor flow preview, and mapping.
- Runtime wait-until-arrival pick execution is not complete.
- Needs live EPD feed before full perception scenario support.

### conveyor_sorting_by_class
- Conveyor sorting by class.
- Partial preview support.
- Needs class-to-place routing completion and live EPD feed.

### bin_tote_picking
- Bin/tote picking.
- Planned.
- Needs grasp strategy and depth-aware object pose support.

### kitting_tray_loading
- Kitting / tray loading.
- Planned.

### machine_tending_load_unload
- Machine tending load/unload.
- Planned.

### inspection_and_reject
- Inspection and reject flow.
- Planned and can reuse sorting primitives.

### palletizing_depalletizing_light
- Palletizing / depalletizing light objects.
- Planned.

### fixture_loading_assembly_assist
- Fixture loading / assembly assist.
- Planned.

### multi_bin_sorting_cell
- Multi-bin sorting cell.
- Planned.

### dual_camera_workcell
- Dual-camera workcell.
- Planned.

### mobile_portable_demo_cell
- Mobile/portable demo cell.
- Mostly supported in preview/fake-hardware workflows.

## Readiness definitions

- **supported_preview**: Scenario can be configured and previewed in Workcell Studio fake-hardware workflow.
- **partial_preview**: Significant pieces are previewable, but required links/rules are missing.
- **needs_live_epd**: Preview exists, but live perception feed is required for full scenario claim.
- **planned**: Scenario documented but not yet sufficiently implemented.
- **blocked**: Scenario cannot proceed due to a prerequisite dependency.
- **complete_simulation**: Scenario complete for simulation-grade workflow.
- **real_hardware_later**: Intentionally deferred to hardware-readiness phase.
