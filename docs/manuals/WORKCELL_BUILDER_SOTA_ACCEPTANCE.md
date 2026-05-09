# Workcell Builder SOTA Acceptance

This checklist defines acceptance gates for builder-generated workflows.

- **Level 0: files exist only** — Generated scene files exist but no build/launch checks.
- **Level 1: scene builds** — Generated package metadata and structure are buildable.
- **Level 2: scene launches in RViz/MoveIt fake hardware** — Fake-hardware launch boots without SRDF/controller extraction failures.
- **Level 3: visual composer + task/grasp preview works** — Builder can generate YAML/package/preview/readiness artifacts with headless acceptance checks.
- **Level 4: EPD/RealSense profile generated and dry-run compatible** — Perception profile output exists and passes dry-run validations.
- **Level 5: guarded real-hardware readiness checklist exists** — Explicit guarded checklist exists for hardware onboarding.

Current required builder baseline: **Level 3**.

Safety gates:
- Fake hardware stays default (`use_fake_hardware:=true`).
- No runtime execution, MoveIt planning service calls, or robot command publication in acceptance checks.
- Placeholder robot/task combinations remain preview-only.
