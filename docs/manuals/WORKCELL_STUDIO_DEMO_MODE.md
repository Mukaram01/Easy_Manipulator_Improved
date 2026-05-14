# Workcell Studio Layout-Aware Flow

Generate Scene now runs layout merge automatically when `layout/workcell_studio_layout.yaml` exists and no unsaved canvas edits are pending.

- Run path: Scene Builder / Command Bar / Demo Mode can trigger **Run Layout Merge**.
- Reports:
  - `generated/workcell_studio_layout_merge_report.json`
  - `generated/workcell_studio_layout_merge_summary.txt`
- Acceptance and Demo read these merge reports and surface `layout_applied`, `generated_from_saved_layout`, stale status, warnings, and blockers.
- Preview Launch blocks stale layouts with: `Layout changed since last generation. Run Generate Scene / Layout Merge before preview.`

## Safety

No robot motion is commanded by merge/generate flows.

Defaults remain:
- `fake_hardware_first: true`
- `runtime_execution_enabled: false`
- `motion_command_sent: false`
- gripper mount RPY: `-1.5708 -1.5708 0`
