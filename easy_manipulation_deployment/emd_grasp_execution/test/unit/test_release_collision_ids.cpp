#include <gtest/gtest.h>

#include "emd/grasp_execution/moveit2/moveit_cpp_if.hpp"

TEST(ReleaseCollisionIds, IncludesTargetAndAttachedTarget)
{
  const auto ids = grasp_execution::moveit2::detail::get_attached_object_acm_ids("box_1");

  ASSERT_EQ(ids.size(), 2u);
  EXPECT_EQ(ids[0], "box_1");
  EXPECT_EQ(ids[1], "#box_1");
}

TEST(ReleaseCollisionIds, DoesNotIncludeRobotOrSupportLinks)
{
  const auto ids = grasp_execution::moveit2::detail::get_attached_object_acm_ids("picked_object");

  for (const auto & id : ids) {
    EXPECT_NE(id, "table_");
    EXPECT_NE(id, "upper_arm_link");
    EXPECT_NE(id, "forearm_link");
    EXPECT_NE(id, "tool0");
  }
}
