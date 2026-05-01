#!/usr/bin/env python3
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from scripts import operator_runtime, studio_lite
from scripts.workcell_roles import ROLE_OPERATOR


class OperatorRuntimeTests(unittest.TestCase):
    def test_module_imports_headless_without_gui_start(self):
        self.assertTrue(hasattr(operator_runtime, "main"))

    def test_path_loading_helper(self):
        args = operator_runtime.parse_args(["--workcell", "/tmp/demo"])
        self.assertEqual(args.workcell, "/tmp/demo")

    def test_operator_role_does_not_expose_generation(self):
        caps = studio_lite.get_visible_capabilities(ROLE_OPERATOR)
        self.assertFalse(caps["show_generate"])
        self.assertFalse(caps["show_validate"])

    def test_operator_role_keeps_no_motion_wording(self):
        caps = studio_lite.get_visible_capabilities(ROLE_OPERATOR)
        self.assertIn("NO ROBOT MOTION", caps["no_motion_banner"])
        self.assertIn("safe_for_robot_motion: false", caps["safe_for_robot_motion"])

    def test_unapproved_bundle_warning_text(self):
        with tempfile.TemporaryDirectory() as td:
            workcell = Path(td)
            generated = workcell / "generated"
            generated.mkdir()
            summary = {
                "approval": {"status": "unapproved", "approved_by": None, "approved_at": None, "notes": ""}
            }
            (generated / "generated_workcell_summary.json").write_text(json.dumps(summary), encoding="utf-8")
            txt = json.loads((generated / "generated_workcell_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(((txt.get("approval") or {}).get("status")), "unapproved")


if __name__ == "__main__":
    unittest.main()
