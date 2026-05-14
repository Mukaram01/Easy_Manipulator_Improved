# Workcell Studio Diagnostics

Use **Diagnostics → Run Self-Test** for offline checks. Status values:
- PASS
- WARN
- BLOCKED
- NOT CHECKED

Reports are generated under `diagnostics/`:
- `workcell_studio_diagnostics_report.json`
- `workcell_studio_diagnostics_summary.txt`
- `workcell_studio_diagnostics_dashboard.html`

Safety: diagnostics never commands robot motion and requires fake-hardware-first defaults.
