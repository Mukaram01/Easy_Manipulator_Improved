"""Two independently authored cells for R4.1b, built only through the Workcell Builder's entry points.

Create New Cell (the wizard's staged environment + instantiate profile), Studio Save (edit the saved
layout, then workcell_studio_layout_merge.py --save-authored, as MainWindow does), the Task Editor (its
authoring helper and save format), then Generate and Validate. No generated or runtime file is edited.
"""
import copy
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_r4_1_authored_cell_flow import SIMULATE, create_cell, generate_and_validate, task_editor_save  # noqa: E402

# Cell A: the reference-style beginner cell (the qualified 25 mm test part where Stage-A placed it).
CELL_A_OBJECT = {'object_id': 'part_04', 'class_id': 'part', 'material': 'qualified_test_polymer', 'shape': 'box',
                 'dimensions': [0.025, 0.025, 0.025],
                 'pose': {'frame_id': 'world', 'xyz': [0.40, -0.20, 0.0125], 'rpy': [0, 0, 0]}}

# Cell B: another object (size, pose, class), pick zone and destination bin location.
CELL_B_OBJECT = {'object_id': 'bracket_b', 'class_id': 'bracket', 'material': 'qualified_test_polymer', 'shape': 'box',
                 'dimensions': [0.03, 0.02, 0.04],
                 'pose': {'frame_id': 'world', 'xyz': [0.46, -0.10, 0.02], 'rpy': [0, 0, 0]}}
CELL_B_STUDIO_MOVES = {'target_bin_default': [0.36, 0.25, 0.1],     # destination bin moved on the bench
                       'pick_zone_main': [0.45, -0.12, 0.3]}         # pick zone moved with the part


def studio_save(scene, moves):
    """Studio: move items in the saved layout, then the Save projection MainWindow runs."""
    from workcell_studio_layout_merge import merge
    path = scene / 'layout/workcell_studio_layout.yaml'
    layout = yaml.safe_load(path.read_text())
    for item in layout['items']:
        if item.get('id') in moves:
            item['pose'] = dict(item.get('pose') or {}, xyz=list(moves[item['id']]))
    missing = set(moves) - {item.get('id') for item in layout['items']}
    if missing:
        raise ValueError(f'layout has no items {sorted(missing)}')
    path.write_text(yaml.safe_dump(layout, sort_keys=False))
    report = merge(scene, save_authored=True)
    if report['status'] != 'READY':
        raise RuntimeError(report)
    return report


def author_cell(root, name, manual_object, studio_moves=None, task_edits=()):
    """Create -> (Studio Save) -> Task Editor save -> reopen -> Generate -> Validate; returns the cell and steps."""
    from task_intent_authoring import load_authoring
    root = Path(root)
    scene = create_cell(root / 'staging' / name, root / name, manual_object)
    created = load_authoring(scene)
    studio = studio_save(scene, studio_moves) if studio_moves else None
    saved = task_editor_save(scene, [*SIMULATE, *task_edits])
    reopened = load_authoring(scene)
    rc, validation = generate_and_validate(scene)
    return scene, dict(created=created, studio=studio, saved=saved, reopened=reopened, validation_rc=rc,
                       validation=validation)


def author_cell_a(root):
    return author_cell(root, 'cell_a', CELL_A_OBJECT, task_edits=[(('pick', 'selection', 'object_filter', 'class_id'), 'part')])


def author_cell_b(root):
    return author_cell(root, 'cell_b', CELL_B_OBJECT, CELL_B_STUDIO_MOVES,
                       task_edits=[(('pick', 'selection', 'object_filter', 'class_id'), 'bracket')])


if __name__ == '__main__':
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=False)
    record = {}
    for label, author in (('cell_a', author_cell_a), ('cell_b', author_cell_b)):
        scene, steps = author(out)
        record[label] = dict(scene=str(scene), validation_rc=steps['validation_rc'],
                             saved_status=steps['saved']['status'], reopened_status=steps['reopened']['status'],
                             reopen_identical=steps['reopened']['task_intent'] == steps['saved']['task_intent'],
                             normalized_intent_sha256=steps['reopened']['normalized_intent_sha256'],
                             studio=steps['studio'])
    (out / 'authoring.json').write_text(json.dumps(record, indent=2, default=str) + '\n')
    print(json.dumps(record, indent=2, default=str))
