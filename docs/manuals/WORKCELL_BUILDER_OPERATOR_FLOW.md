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
