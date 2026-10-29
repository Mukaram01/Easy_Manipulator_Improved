"""R2.5B: a two-bin cell authored only through the Workcell Builder's entry points.

Create New Cell (UR5 + 2F-85 workbench), Studio: the sorting bin and its placement region duplicated as Bin B,
Task Editor save (simulator backend) placing into Bin A or Bin B, Generate and Validate. Used for the
multi-destination physics world (simulator_world.derive_world physical_destinations='all') and its plan-only gates.
"""
import copy
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from r4_1b_cells import CELL_A_OBJECT  # noqa: E402
from test_r4_1_authored_cell_flow import SIMULATE, create_cell, generate_and_validate, task_editor_save  # noqa: E402

# The default bin moved out to x 0.58 and Bin B beside it at x 0.25: their 0.30 m x 0.20 m interiors plus 10 mm proxy
# walls leave a 10 mm gap. Bin B at x 0.12 (beside the bin's default x 0.45) sits over the robot base: every transfer
# to it drove the forearm into the bin (LENLEG 2026-10-29 G2, -27).
XYZ_A = [0.58, 0.22, 0.1]
BIN_B, ZONE_B, XYZ_B = 'target_bin_b', 'place_zone_b', [0.25, 0.26, 0.1]
TO_BIN_B = [(('place', 'target', 'asset_ref'), BIN_B), (('place', 'target', 'region_ref'), ZONE_B)]


def add_bin_b(scene):
    """Studio: move the sorting bin out, duplicate it and its placement region as Bin B beside it, then Save."""
    from workcell_studio_layout_merge import merge
    path = scene / 'layout/workcell_studio_layout.yaml'
    layout = yaml.safe_load(path.read_text())
    items = {i['id']: i for i in layout['items']}
    items['target_bin_default']['pose']['xyz'] = list(XYZ_A)
    items['place_zone_default']['pose']['xyz'] = [XYZ_A[0], XYZ_A[1], items['place_zone_default']['pose']['xyz'][2]]
    b = copy.deepcopy(items['target_bin_default'])
    b.update(id=BIN_B, task_zone_ref=ZONE_B, display_name='Destination Bin B')
    b['pose']['xyz'] = list(XYZ_B)
    z = copy.deepcopy(items['place_zone_default'])
    z.update(id=ZONE_B, target_ref=BIN_B, display_name='Bin B placement')
    z['pose']['xyz'] = [XYZ_B[0], XYZ_B[1], z['pose']['xyz'][2]]
    layout['items'] += [b, z]
    path.write_text(yaml.safe_dump(layout, sort_keys=False))
    report = merge(scene, save_authored=True)
    if report['status'] != 'READY':
        raise RuntimeError(report)
    env = yaml.safe_load((scene / 'environment.yaml').read_text())['environment']
    if not any(a['id'] == BIN_B for a in env['assets']) or not any(t['id'] == ZONE_B for t in env['task_zones']):
        raise RuntimeError('Studio Save did not project Bin B')
    return report


def author_two_bin_cell(root, name, manual_object=CELL_A_OBJECT, destination='a', task_edits=()):
    """Create -> Studio Bin B -> Task Editor save (Bin A or B) -> Generate -> Validate; returns (cell, rc)."""
    root = Path(root)
    scene = create_cell(root / 'staging' / name, root / name, manual_object)
    add_bin_b(scene)
    edits = [*SIMULATE, *task_edits, *(TO_BIN_B if destination == 'b' else [])]
    task_editor_save(scene, edits)
    rc, _ = generate_and_validate(scene)
    return scene, rc
