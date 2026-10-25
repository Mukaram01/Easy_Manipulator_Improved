# Stage A1 canonical retirement evidence

This directory preserves the minimal evidence required to reproduce and understand
the current Stage-A1 recovery qualification state before retiring the development PC.

It intentionally excludes superseded runs, build products, caches, temporary
worktrees, and old diagnostics.

## Current recovery blocker

The current blocker is the measured physical follower state -> controller-start
binding for the Robotiq mimic joints.

The installed gz_ros2_control / DART runtime does not instantaneously project the
measured follower state onto the nominal mimic manifold. The historical recovery
opening therefore remains unqualified until a conservative bound on physical
follower evolution through the first controller interval is established.

## Preserved evidence

### start_binding/

Current start-binding investigation and machine-readable result.

### continuous_jtc/

Final continuous JTC interval-audit provenance plus the exact historical
withdrawal fixture used by the detached-contact and start-binding diagnostics.

The fixture preserves:
- measured scene
- URDF
- SRDF
- leader identity
- provenance

### detached_separation/

Machine-readable evidence for the qualified detached
ROBOT_LINK/WORLD_OBJECT geometric separation certificate.

### release_failure/

Evidence from the physical grasp-retention failure that initiated the recovery work.

### release_lifecycle/

Evidence from the release-lifecycle replay qualification.

## Supporting source

The tracked repository also contains:

- docs/manuals/STAGE_A1_CONTINUOUS_CONTROLLER_AUDIT.md
- docs/manuals/STAGE_A1_DETACHED_SEPARATION.md
- third_party/patches/moveit_humble_2.5.10/

The MoveIt patches preserve the two qualified Humble 2.5.10 lifecycle fixes.

This evidence does not constitute physical robot safety certification.
