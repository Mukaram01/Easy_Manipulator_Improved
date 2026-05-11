# Workcell Builder Offline Readiness Overlay

This overlay provides **offline approximate readiness only** for layout sanity checks. It is not MoveIt planning, not certified safety validation, and does not command robot motion.

- Approximate reach envelope: top-down annulus from robot base (example UR5 radius ~0.85 m, inner exclusion ~0.15 m).
- Workspace bounds: checks against configured bounds; defaults are inferred as x/y [-1.5, 1.5] and z [0.0, 2.0] with `workspace_bounds_source: default`.
- Safety/exclusion zones: supports warning/exclusion with circle/rectangle approximations.
- Object overlap approximation: simple 2D footprint overlap warnings.
- Camera placement warnings: outside workspace or suspicious z-height warnings.
- Task pick/place reach warnings: flags pick/place targets outside estimated reach.

Safety constraints:
- No MoveIt planning calls.
- No trajectory execution.
- No hardware enablement.
- Fake-hardware-first guidance is preserved (`use_fake_hardware:=true`).

## Operator flow
1. Open workcell_builder.
2. Select/create scene.
3. Select robot/tool and check compatibility.
4. Add/import/place objects.
5. Add camera metadata if needed.
6. Open Visual Layout Editor.
7. Enable Readiness Overlay.
8. Review reach/workspace/overlap/camera/task warnings.
9. Fix object/camera positions.
10. Generate Files.
11. Review schema/readiness summary.
12. Launch with `use_fake_hardware:=true`.
