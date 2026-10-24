#include <gtest/gtest.h>

#include "gui/semantic_role_edit.hpp"

using workcell_builder::semantic_role_actually_changed;

TEST(SemanticRoleEdit, ReapplyingTheDisplayedAliasIsNotAnEdit)
{
  // ur5_2f_test: layout says "target_bin", the inspector shows "place_target_bin".
  EXPECT_FALSE(semantic_role_actually_changed("place_target_bin", "target_bin", "place_target_bin"));
  EXPECT_FALSE(semantic_role_actually_changed("target_bin", "target_bin", "place_target_bin"));
  EXPECT_FALSE(semantic_role_actually_changed("", "target_bin", "place_target_bin"));
}

TEST(SemanticRoleEdit, ChoosingAnotherRoleIsAnEdit)
{
  EXPECT_TRUE(semantic_role_actually_changed("pick_object", "target_bin", "place_target_bin"));
  EXPECT_TRUE(semantic_role_actually_changed("fixture", "", ""));
}
