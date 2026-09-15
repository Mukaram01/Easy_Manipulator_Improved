# Physical destination implementation plan

**Goal:** Make authored target-local placement geometry resolve identically for generation, preview, validation and fake-hardware execution.
**Architecture:** environment.yaml owns target transforms, reviewed usable geometry and target-local placement definitions. World poses are checked projections. Save converts Inspector world edits into local geometry; target movement reprojects dependent zones. A shared Python resolver rejects missing metadata, ambiguous ownership, stale projections and containment failures. Collision-aware complete-cycle planning remains mandatory before execution.
**Tech stack:** Python/YAML, existing Qt Builder, Web3D, ROS 2 Humble/MoveIt.
**Spec:** User's approved R1.9 request; baseline evidence in docs/manuals/evidence/r19/baseline_mismatch.json.

## Constraints

Fake hardware only. Preserve build/install/log and existing dependencies. No camera/EPD, suction, simulator additions, broad refactors or weaker collision/ACM gates.

## Tasks

- [x] Inspect clean main, remote branch availability, source ownership and measure original STL/world destination mismatch.
- [ ] Add failing physical destination tests for rigid transforms, missing metadata/targets, stale projections, disjoint regions and rotated object clearance.
- [ ] Implement scripts/physical_destination.py with resolve_destination(environment, zone_id), checked target-local box containment and world pose resolution.
- [ ] Connect Save in workcell_studio_layout_merge.py, generated cell export and validate_cell_definition.py to the contract; preserve semantic aliases.
- [ ] Replace preview exporter snapping with the shared resolver; retain editable zone poses and metadata.
- [ ] Connect load_canonical_place_target and final/preplan object containment; preserve nine-stage collision-aware planning.
- [ ] Author reviewed bin-local usable geometry and canonical placement; prove original-bin path failure before moving bin through authored save path.
- [ ] Run targeted Python/Qt tests and affected-package builds, negative live acceptance, positive fake-hardware acceptance and same-session two-cycle regression. Inspect JSON and process cleanup.
- [ ] Update roadmap and evidence accurately; diff check; commit, push and open PR to main.
