# Supported Asset Pack v1

- Golden path: UR5 + Robotiq 2F + table/workbench + bins + cube object, fake hardware first.
- WARN path: UR5 + AirPick suction (metadata/generation ready; runtime may be partial).
- Preview only: generic delta/gantry plus placeholder industrial templates (Fanuc/ABB/KUKA/conveyor/CNC/safety).
- Real drivers remain metadata-only (`driver_status: optional_later`).

## Validation

Run:

```bash
PYTHONPATH=$PWD:$PYTHONPATH python3 scripts/audit_workcell_assets.py --root . --json
PYTHONPATH=$PWD:$PYTHONPATH python3 -m pytest -q tests/test_supported_asset_pack_v1.py tests/test_builder_golden_ur5_2f_template.py tests/test_builder_ur5_suction_template.py tests/test_builder_preview_only_templates.py
```
