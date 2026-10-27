#include <gtest/gtest.h>

#include "gui/semantic_role_edit.hpp"

using workcell_builder::semantic_role_actually_changed;
using workcell_builder::semantic_role_to_persist;

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

TEST(SemanticRoleEdit, SaveKeepsAuthoredRoleUnlessReallyChanged)
{
  // A mount/mate/align commit carries the display alias; Save must keep "target_bin".
  EXPECT_EQ(semantic_role_to_persist("place_target_bin", "target_bin", "place_target_bin"), "target_bin");
  EXPECT_EQ(semantic_role_to_persist("support_surface_table", "support_surface", "support_surface_table"), "support_surface");
  EXPECT_EQ(semantic_role_to_persist("pick_source_zone", "pick_zone", "pick_source_zone"), "pick_zone");
  EXPECT_EQ(semantic_role_to_persist("fixture", "target_bin", "place_target_bin"), "fixture");
  EXPECT_EQ(semantic_role_to_persist("object", "", ""), "object");
}
