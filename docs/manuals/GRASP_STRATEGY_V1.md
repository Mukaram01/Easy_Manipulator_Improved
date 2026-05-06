# Grasp Strategy v1

`grasp_strategy/v1` defines **offline metadata only** for grasp intent in Workcell Studio.

It does **not** prove reachability, collision safety, suction reliability, IO safety, or runtime readiness. It does **not** change MoveIt planning, EMD execution, or robot runtime behaviour in this phase.

A strategy describes how a tool should approach, orient, contact, retreat, and release for later YAML/CLI/generator/Streamlit workflows.

Schema core:
- `schema_version: grasp_strategy/v1`
- `grasp_strategy.id`, `label`, `strategy`
- optional compatibility hints (`compatible_tool_families`, `compatible_end_effector_capabilities`)
- approach/orientation metadata (`approach_axis`, `orientation_mode`, distances, optional angle sets)
- optional tool frame offsets
- required `contact` and `release` mappings
- optional `limitations_warnings`

Validate:
```bash
python3 scripts/validate_grasp_strategy.py catalog/grasp_strategies
```
