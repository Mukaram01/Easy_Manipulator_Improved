#!/usr/bin/env python3
from __future__ import annotations

import unittest

from scripts import studio_lite


class StudioLiteTests(unittest.TestCase):
    def test_command_builders_use_expected_scripts(self):
        self.assertIn('validate_cell_definition.py', ' '.join(studio_lite.build_validate_cell_definition_command('cell.yaml')))
        self.assertIn('generate_workcell_from_cell_definition.py', ' '.join(studio_lite.build_generate_workcell_command('cell.yaml', '/tmp/out', 'pkg')))
        self.assertIn('preview_generated_workcell_bundle.py', ' '.join(studio_lite.build_preview_workcell_command('/tmp/w')))
        self.assertIn('preview_generated_workcell_bundle.py', ' '.join(studio_lite.build_publish_preview_markers_command('/tmp/w')))
        gated = studio_lite.build_run_gated_bundle_command('/tmp/w')
        self.assertIn('run_generated_workcell_bundle.py', ' '.join(gated))
        self.assertIn('--gated-dry-run', gated)
        self.assertIn('--json', gated)
        self.assertIn('preview_generated_workcell_bundle.py', ' '.join(studio_lite.build_preview_task_flow_command('/tmp/w', '/tmp/tf.json')))

    def test_no_physical_robot_execution_flags(self):
        commands = [
            studio_lite.build_validate_cell_definition_command('cell.yaml'),
            studio_lite.build_generate_workcell_command('cell.yaml', '/tmp/out', 'pkg'),
            studio_lite.build_preview_workcell_command('/tmp/w'),
            studio_lite.build_publish_preview_markers_command('/tmp/w'),
            studio_lite.build_run_gated_bundle_command('/tmp/w'),
            studio_lite.build_preview_task_flow_command('/tmp/w', '/tmp/tf.json'),
        ]
        flattened = ' '.join(' '.join(c) for c in commands)
        self.assertNotIn('--execute', flattened)
        self.assertNotIn('--replay', flattened)
        self.assertNotIn('controller', flattened)
        self.assertNotIn('follow_joint_trajectory', flattened)

    def test_no_motion_constants_present(self):
        self.assertIn('NO ROBOT MOTION', studio_lite.NO_MOTION_BANNER)
        self.assertIn('safe_for_robot_motion: false', studio_lite.SAFE_FOR_ROBOT_MOTION_TEXT)

    def test_module_imports_headless_without_gui_start(self):
        self.assertTrue(hasattr(studio_lite, 'build_validate_cell_definition_command'))
        self.assertFalse(hasattr(studio_lite, 'tk'))


if __name__ == '__main__':
    unittest.main()
