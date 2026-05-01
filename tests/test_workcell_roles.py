#!/usr/bin/env python3
from __future__ import annotations

import unittest

from scripts.workcell_roles import ROLE_DEVELOPER, ROLE_OPERATOR, can_role_do


class WorkcellRolesTests(unittest.TestCase):
    def test_developer_can_generate_workcell(self):
        self.assertTrue(can_role_do(ROLE_DEVELOPER, "generate_workcell"))

    def test_operator_cannot_generate_workcell(self):
        self.assertFalse(can_role_do(ROLE_OPERATOR, "generate_workcell"))

    def test_both_can_preview_bundle(self):
        self.assertTrue(can_role_do(ROLE_DEVELOPER, "preview_bundle"))
        self.assertTrue(can_role_do(ROLE_OPERATOR, "preview_bundle"))

    def test_both_can_run_gated_dry_run(self):
        self.assertTrue(can_role_do(ROLE_DEVELOPER, "run_gated_dry_run"))
        self.assertTrue(can_role_do(ROLE_OPERATOR, "run_gated_dry_run"))

    def test_execute_motion_false_for_both_roles(self):
        self.assertFalse(can_role_do(ROLE_DEVELOPER, "execute_motion"))
        self.assertFalse(can_role_do(ROLE_OPERATOR, "execute_motion"))


if __name__ == "__main__":
    unittest.main()
