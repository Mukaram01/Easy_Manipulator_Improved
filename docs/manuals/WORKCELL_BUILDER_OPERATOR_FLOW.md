# Workcell Builder Operator Flow

Recommended operator flow:
1. Scene
2. Robot + End Effector
3. Objects / Layout
4. Camera / Perception Metadata
5. Task / Grasp Strategy
6. Validate (Run Offline Validation)
7. Generate Files

## Validation status meaning
- PASS: check is good, proceed.
- WARN: proceed allowed, but operator should review.
- FAIL/BLOCKED: must fix before generation succeeds.

## Validation Dashboard checks
- Scene Schema
- Asset Catalog
- Robot/Tool Compatibility
- Object Placement
- Camera Metadata
- Task Recipe
- Readiness Overlay
- Fake-Hardware Smoke (static readiness status)

## Warning vs blocker policy
- Warnings are acceptable for generation when no blockers exist.
- Blockers must be fixed before Generate Files can report success.

## Safety and tooling boundaries
- Fake-hardware-first is the default operator path.
- No MoveIt planning/execution or robot motion is triggered by offline validation.
- Golden Demo is developer/test tooling only, not main operator workflow.

## Validation status meanings
- **PASS**: check passed with no blockers.
- **WARN**: check completed with warnings; generation may proceed if blockers are zero.
- **FAIL**: blocker detected; fix before Generate Files.
- **SKIP**: check intentionally not executed (for example static-only fake-hardware smoke row without ROS launch).
- **UNKNOWN**: check has not run yet.

## Warnings vs blockers
- Warnings are advisory and shown in the dashboard warning count.
- Blockers are gating issues and shown in blocker count; Generate Files is blocked when blocker count > 0.

## Why Run Offline Validation is safe
- Run Offline Validation aggregates only offline/static checks and dashboard summaries.
- It does not launch ROS, call MoveIt planning, execute robot motion, or enable real hardware.
- Fake-hardware-first remains the default safety posture for operator workflow.
