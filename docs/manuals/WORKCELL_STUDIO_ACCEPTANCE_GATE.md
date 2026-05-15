# Workcell Studio Acceptance Gate

Unified Workcell Studio Acceptance Gate for scratch and existing scenes.

## Included audits
- file-output audit
- state-transition audit
- error-message audit
- real build/run smoke
- regression audit

## Commands
```bash
python3 scripts/run_workcell_studio_acceptance_gate.py --mode scratch --workspace ~/workcell_ws --output /tmp/workcell_studio_acceptance
python3 scripts/generate_scratch_cell_acceptance.py --workspace ~/workcell_ws --output /tmp/workcell_studio_acceptance
python3 scripts/audit_workcell_studio_regressions.py --workspace ~/workcell_ws --output /tmp/workcell_studio_acceptance
python3 scripts/smoke_test_scratch_cell_workspace.py --workspace ~/workcell_ws --output /tmp/workcell_studio_acceptance
```

Output includes `acceptance_summary.md` plus JSON report artifacts.
