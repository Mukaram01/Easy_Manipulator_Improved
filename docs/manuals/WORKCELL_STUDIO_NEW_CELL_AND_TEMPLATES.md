# Workcell Studio New Cell and Templates

`New Cell` now instantiates a real scene package backend rather than UI-only scaffolding.

Supported behavior:
- Template + robot + tool + layout metadata produce generated files under scene root.
- UR5 + finger tool uses finger grasp defaults.
- UR5 + suction tool defaults to `suction_top` strategy.
- Placeholder/non-UR5 robots are generated as metadata-first `PREVIEW_ONLY` scenes.

Safety and runtime constraints:
- Fake-hardware-first defaults are preserved.
- Runtime execution remains disabled by default.
- No MoveIt/live motion commands are executed during generation.
- EPD GUI remains separate.
