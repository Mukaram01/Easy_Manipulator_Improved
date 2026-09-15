#!/usr/bin/env python3
"""Record real canonical contract rejection and serialized local/world evidence.

Runs production validation, Product View export normalization and runtime input
loading against isolated copies of the actual authored scene. This is static
contract acceptance; the separate Studio/MoveIt evidence proves motion.
"""
import argparse
import copy
import json
from pathlib import Path
import tempfile

import yaml
from authored_yaml import write_preserving
from export_workcell_studio_web_scene_impl import _normalise_active_place_zone
from perceived_object_grasp_execute import load_canonical_place_target
from physical_destination import apply, resolve_destination, rotation
from validate_builder_generated_scene import validate_scene
from workcell_studio_layout_merge import merge


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    env = yaml.safe_load((args.scene/'environment.yaml').read_text())
    layout = yaml.safe_load((args.scene/'layout/workcell_studio_layout.yaml').read_text())
    cell = yaml.safe_load((args.scene/'cell_definition.yaml').read_text())
    zone_id = env['task']['place']['target_ref']
    baseline = resolve_destination(env['environment'], zone_id)
    report = {'scope': 'canonical static contract acceptance; motion evidence recorded separately',
              'fake_hardware_only': True, 'camera_epd_used': False,
              'canonical_resolved_destination': baseline, 'negative_cases': {}}
    for name in ('old_disjoint_bin', 'moved_target_stale_world', 'missing_placement_metadata', 'invalid_usable_metadata', 'missing_active_zone', 'region_outside_usable'):
        authored = copy.deepcopy(env)
        physical = authored['environment']
        target = next(t for t in physical['assets'] if t['id'] == baseline['target_id'])
        zone = next(z for z in physical['task_zones'] if z['id'] == zone_id)
        if name == 'old_disjoint_bin':
            target['pose_xyz'] = [.94, -.28, .1]
        elif name == 'moved_target_stale_world':
            target['pose_xyz'][0] += .05
        elif name == 'missing_placement_metadata':
            zone.pop('placement_local')
        elif name == 'missing_active_zone':
            physical['task_zones'] = [z for z in physical['task_zones'] if z['id'] != zone_id]
        elif name == 'region_outside_usable':
            zone['placement_local']['pose_xyz'][0] = 2.0
        else:
            target['usable_placement']['dimensions'][0] = -1
        with tempfile.TemporaryDirectory(prefix='r19-contract-') as temp:
            root = Path(temp)
            (root/'layout').mkdir()
            (root/'environment.yaml').write_text(yaml.safe_dump(authored))
            invalid_cell = copy.deepcopy(cell)
            invalid_cell['environment'] = physical
            (root/'cell_definition.yaml').write_text(yaml.safe_dump(invalid_cell))
            (root/'layout/workcell_studio_layout.yaml').write_text(yaml.safe_dump(layout))
            errors = validate_scene(root)['errors']
            physical_errors = [e for e in errors if 'physical destination' in e]
            assert physical_errors, (name, errors)
            result = {'validation': 'REJECTED', 'validation_reasons': physical_errors}
            for consumer, fn in [('runtime', lambda: load_canonical_place_target(root)),
                                 ('product_view', lambda: _normalise_active_place_zone({'environment': authored, 'layout': copy.deepcopy(layout)})),
                                 ('save', lambda: merge(root, save_authored=True))]:
                try:
                    fn()
                except (ValueError, RuntimeError) as exc:
                    result[consumer] = {'status': 'REJECTED', 'reason': str(exc)}
                else:
                    raise AssertionError(f'{name}: {consumer} accepted invalid destination')
            report['negative_cases'][name] = result
    with tempfile.TemporaryDirectory(prefix='r19-transform-') as temp:
        root = Path(temp)
        (root/'layout').mkdir()
        (root/'environment.yaml').write_text(yaml.safe_dump(env))
        lp = root/'layout/workcell_studio_layout.yaml'
        edited = copy.deepcopy(layout)
        target = next(i for i in edited['items'] if i['id'] == baseline['target_id'])
        target['pose']['xyz'][0] += .03
        target['pose']['rpy'] = [.1, -.1, .37]
        lp.write_text(yaml.safe_dump(edited))
        merge(root, save_authored=True)
        saved = yaml.safe_load((root/'environment.yaml').read_text())
        moved = resolve_destination(saved['environment'], zone_id)
        assert moved['placement_local'] == baseline['placement_local']
        assert moved['pose_xyz'] != baseline['pose_xyz']
        edited = yaml.safe_load(lp.read_text())
        layout_id = next(z for z in saved['environment']['task_zones'] if z['id'] == zone_id)['layout_item_ref']
        zone = next(i for i in edited['items'] if i['id'] == layout_id)
        delta = apply(rotation(target['pose']['rpy']), [.01, 0, 0])
        zone['pose']['xyz'] = [a+b for a, b in zip(zone['pose']['xyz'], delta)]
        write_preserving(lp, edited)
        merge(root, save_authored=True)
        saved = yaml.safe_load((root/'environment.yaml').read_text())
        in_bin_edit = resolve_destination(saved['environment'], zone_id)
        assert abs(in_bin_edit['placement_local']['pose_xyz'][0] - baseline['placement_local']['pose_xyz'][0] - .01) < 1e-8
        report['serialized_transform_proof'] = {'before': baseline, 'translated_and_rotated_target': moved,
                                               'one_cm_local_x_edit': in_bin_edit, 'status': 'PASS'}
    report['status'] = 'PASS'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(args.output)


if __name__ == '__main__':
    main()
