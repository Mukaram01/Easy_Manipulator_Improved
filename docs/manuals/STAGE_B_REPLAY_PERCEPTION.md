# Stage-B: camera-free replayed EPD perception in the qualified physics cycle

Simulation only; real robot motion stays locked. No live camera is used.

## Pipeline

1. **Record** (`scripts/perception_replay.py record`): a render-only Gazebo process built from the cell's derived
   physics world (static models at authored rest poses, each object's authored `visual` mesh, the cell camera)
   writes an EPD replay fixture and `recording.json`. Provenance is always `SIMULATOR_RENDERED_SYNTHETIC`.
   The qualified physics world is not modified.
2. **Detect** (`perception_replay.py detect`): EPD's production node (tracking use case, hybrid RGB-D refinement),
   its replay node and its Workcell contract bridge (`--runtime-mode replay`) replay the recording; the result is a
   `detected_objects/v1` snapshot (`source.mode: replayed_snapshot`) saved as the Task's replay input.
3. **Run** (Builder *Run Physics Simulation Cycle*): every session detects from the recording again (`PERCEIVING`),
   measures the physical objects and binds each to the one EPD detection inside it
   (`simulator_observations.py --perception`), then resolves, revalidates and executes the qualified full cycle.

## Rules

- **Replay time**: ages are measured on the recording clock only (last frame stamp minus the result's frame stamp,
  2 s replay ceiling). Live EPD output, stamps that are not recording frames, and stale detections are BLOCKED;
  nothing is re-stamped as fresh. The live camera path keeps its host-clock guard.
- **Calibration**: camera-frame detections become world observations only through the recording's
  `simulation_exact` camera-to-world record, valid while it equals the cell's current camera pose. A real camera
  needs a qualified extrinsic calibration (not available): BLOCKED.
- **Identity**: classes come only from EPD. A measured object takes the class/confidence of the one detection whose
  centroid lies inside its box (else within 20 mm of only it); others are `unperceived` and never targets. A
  target-class detection with no single physical object blocks the cycle. Task exclusions apply as usual.
- **Freshness of what is planned**: planning consumes the measured physical object (simulator-measurement policy,
  `replayed_perception_in_simulation`); the detection that classified it keeps the 2 s replay ceiling, re-checked by
  the executor.
- **Envelope**: the qualified 2F top grasp accepts ~30 mm wide, <=45 mm tall bodies (closed-tool penetration <= 1 mm);
  larger bodies are refused by grasp qualification, never forced.

## Reproduce (LENLEG)

```bash
cd <workspace>/src/easy_manipulation_deployment
source /opt/ros/humble/setup.bash && source <workspace>/install/setup.bash
# EPD runtime registered once: python3 scripts/live_perception_session.py register --epd-setup <epd ws>/install/setup.bash ...
python3 tests/stage_b_cells.py <out> <workspace> nominal      # author (Builder entry points) + record + detect + Task input
python3 scripts/physics_cycle_preflight.py --workspace <workspace> <out>/stage_b_cell
# Builder GUI: open the cell, Run Physics Simulation Cycle; or unattended:
workcell_builder --physics-cycle-acceptance --scene-path <out>/stage_b_cell --smoke-output <dir> --workspace <workspace>
```

Variants for negative tests: `unreachable`, `collision` (`tests/stage_b_cells.py <out> <workspace> <variant>`).
