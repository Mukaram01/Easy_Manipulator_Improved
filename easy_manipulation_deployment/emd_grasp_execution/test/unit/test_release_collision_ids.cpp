#include <gtest/gtest.h>

#include <set>

#include "emd/grasp_execution/moveit2/moveit_cpp_if.hpp"

TEST(ReleaseCollisionIds, NormalizesUnprefixedTargetIdIntoUniqueAliases)
{
  const auto ids = grasp_execution::moveit2::detail::get_attached_object_acm_ids("box-abc");
  const std::set<std::string> unique_ids(ids.begin(), ids.end());

  EXPECT_EQ(ids.size(), unique_ids.size());
  EXPECT_EQ(unique_ids.count("box-abc"), 1u);
  EXPECT_EQ(unique_ids.count("#box-abc"), 1u);
  EXPECT_EQ(unique_ids.count("##box-abc"), 0u);
}

TEST(ReleaseCollisionIds, NormalizesPrefixedTargetIdWithoutDoubleHashAliases)
{
  const auto ids = grasp_execution::moveit2::detail::get_attached_object_acm_ids("#box-abc");
  const std::set<std::string> unique_ids(ids.begin(), ids.end());

  EXPECT_EQ(ids.size(), unique_ids.size());
  EXPECT_EQ(unique_ids.count("#box-abc"), 1u);
  EXPECT_EQ(unique_ids.count("box-abc"), 1u);
  EXPECT_EQ(unique_ids.count("##box-abc"), 0u);
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
