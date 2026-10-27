#!/usr/bin/env python3
"""Disposable multi-selection fixture: ur5_2f_test plus two more sorting bins.

Usage: build_fixture.py SCENE_DIR   (a scratch copy of scenes/ur5_2f_test; edited in place)

The extra bins reuse the calibrated sorting_bin mesh of target_bin_default and are
placed on the workbench at deliberately uneven positions, so alignment and
distribution have something to do. Nothing outside SCENE_DIR is touched.
"""
import copy
import sys
from pathlib import Path

import yaml

EXTRA = (('reject_bin', 'Reject Bin', [0.80, -0.25, 0.1]), ('spare_bin', 'Spare Bin', [1.0, 0.10, 0.1]))


def main(scene_dir):
    scene = Path(scene_dir)
    layout_path = scene / 'layout' / 'workcell_studio_layout.yaml'
    env_path = scene / 'environment.yaml'
    layout = yaml.safe_load(layout_path.read_text())
    env = yaml.safe_load(env_path.read_text())
    template = next(item for item in layout['items'] if item['id'] == 'target_bin_default')
    asset = next(item for item in env['environment']['assets'] if item['id'] == 'target_bin_default')
    for identifier, name, xyz in EXTRA:
        if any(item['id'] == identifier for item in layout['items']):
            continue
        item = copy.deepcopy(template)
        for key in ('task_zone_ref', 'usable_placement', 'support_surface_ref'):
            item.pop(key, None)
        item.update(id=identifier, layout_item_ref=identifier, display_name=name, type='bin', role='object',
                    pose={'xyz': list(xyz), 'rpy': [0, 0, 0]})
        layout['items'].append(item)
        record = copy.deepcopy(asset)
        for key in ('task_zone_ref', 'usable_placement', 'support_surface_ref'):
            record.pop(key, None)
        record.update(id=identifier, layout_item_ref=identifier, display_name=name, type='bin', role='object',
                      pose_xyz=list(xyz), pose_rpy=[0, 0, 0])
        env['environment']['assets'].append(record)
    layout_path.write_text(yaml.safe_dump(layout, sort_keys=False))
    env_path.write_text(yaml.safe_dump(env, sort_keys=False))


if __name__ == '__main__':
    main(sys.argv[1])
