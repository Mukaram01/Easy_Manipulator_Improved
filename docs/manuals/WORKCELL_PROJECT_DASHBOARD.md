# Workcell Project Dashboard

## What this is

The Workcell Project Dashboard is a single static HTML page generated from a workcell project's `project_manifest.json`.

It gives operators, reviewers, and managers a quick offline summary of:

- project identity and status,
- cell/robot/task/environment overview,
- validation + commissioning status,
- generated artifacts and checksums,
- operator next steps.

**The dashboard is a read-only offline project summary. It does not replace the existing GUI and does not control the robot.**

## What this is not

- It does **not** launch ROS nodes.
- It does **not** control robot motion or execution.
- It does **not** replace MoveIt, controller, perception, EPD, or runtime tools.
- It does **not** replace existing GUI behavior.

## Generate the dashboard

Dashboard generation is now included by default in project generation:

```bash
python3 scripts/create_workcell_project.py \
  --cell-definition tests/fixtures/cell_definition_pick_place.yaml \
  --output-dir /tmp/workcell_projects \
  --force
```

The generated file is typically:

```text
/tmp/workcell_projects/<cell_id>/dashboard/index.html
```

Manual generation command:

```bash
python3 scripts/generate_workcell_dashboard.py \
  --project-dir /tmp/workcell_projects/<cell_id>
```

Direct manifest mode:

```bash
python3 scripts/generate_workcell_dashboard.py \
  --manifest /tmp/workcell_projects/<cell_id>/project_manifest.json \
  --output /tmp/workcell_projects/<cell_id>/dashboard/index.html
```

Optional flags:

- `--json`: print machine-readable summary.
- `--strict`: convert warnings to non-zero exit.
- `--quiet`: reduce console output.

## Open the dashboard

Open locally in any browser (offline):

```bash
xdg-open /tmp/workcell_projects/<cell_id>/dashboard/index.html
```

No network, CDN, external JS, or external CSS is required.

## Integration with existing GUI/runtime

Use this dashboard for offline review and handover checks before runtime launch.

Keep normal runtime workflow unchanged:

1. validate cell + scene contracts,
2. review recipe/plan/commissioning artifacts,
3. then run normal launch/runtime tools.

The dashboard intentionally stays read-only and does not alter existing GUI/runtime behavior.

## Future screenshots/pictures

Future versions can embed locally generated screenshots or pictures by writing additional image files into the generated project folder and linking them from the dashboard HTML.

This can be added without changing runtime logic, launch behavior, planner behavior, controller behavior, perception behavior, or grasp execution logic.
