#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "emd/grasp_execution/moveit2/moveit_cpp_if.hpp"

namespace
{

bool contains_pair(
  const std::vector<std::pair<std::string, std::string>> & pairs,
  const std::string & first,
  const std::string & second)
{
  return std::find(pairs.begin(), pairs.end(), std::make_pair(first, second)) != pairs.end();
}

bool is_always_allowed(
  const collision_detection::AllowedCollisionMatrix & acm,
  const std::string & first,
  const std::string & second)
{
  collision_detection::AllowedCollision::Type allowed_type =
    collision_detection::AllowedCollision::NEVER;
  if (!acm.getEntry(first, second, allowed_type)) {
    return false;
  }
  return allowed_type == collision_detection::AllowedCollision::ALWAYS;
}

}  // namespace

TEST(ReleasePlanningAllowedContacts, IncludesOnlyExpectedPairs)
{
  const std::vector<std::string> fingertip_links = {
    "gripper_finger1_finger_tip_link",
    "gripper_finger2_finger_tip_link",
  };
  const std::vector<std::string> release_links = {"table_"};

  const auto pairs = grasp_execution::moveit2::detail::get_release_planning_allowed_pairs(
    "box_demo", fingertip_links, release_links);

  EXPECT_TRUE(contains_pair(pairs, "<octomap>", "gripper_finger1_finger_tip_link"));
  EXPECT_TRUE(contains_pair(pairs, "<octomap>", "gripper_finger2_finger_tip_link"));
  EXPECT_TRUE(contains_pair(pairs, "box_demo", "<octomap>"));
  EXPECT_TRUE(contains_pair(pairs, "#box_demo", "<octomap>"));
  EXPECT_TRUE(contains_pair(pairs, "box_demo", "table_"));
  EXPECT_TRUE(contains_pair(pairs, "#box_demo", "table_"));

  EXPECT_FALSE(contains_pair(pairs, "forearm_link", "table_"));
  EXPECT_FALSE(contains_pair(pairs, "upper_arm_link", "table_"));
  EXPECT_FALSE(contains_pair(pairs, "forearm_link", "<octomap>"));
  EXPECT_FALSE(contains_pair(pairs, "wrist_2_link", "camera_link"));
}

TEST(ReleasePlanningAllowedContacts, PrefixedTargetDoesNotCreateDoubleHashAliases)
{
  const auto pairs = grasp_execution::moveit2::detail::get_release_planning_allowed_pairs(
    "#box-abc",
    {"gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"},
    {"table_"});

  EXPECT_TRUE(contains_pair(pairs, "<octomap>", "gripper_finger1_finger_tip_link"));
  EXPECT_TRUE(contains_pair(pairs, "<octomap>", "gripper_finger2_finger_tip_link"));
  EXPECT_TRUE(contains_pair(pairs, "#box-abc", "<octomap>"));
  EXPECT_TRUE(contains_pair(pairs, "box-abc", "<octomap>"));
  EXPECT_TRUE(contains_pair(pairs, "#box-abc", "table_"));
  EXPECT_TRUE(contains_pair(pairs, "box-abc", "table_"));

  EXPECT_FALSE(contains_pair(pairs, "##box-abc", "<octomap>"));
  EXPECT_FALSE(contains_pair(pairs, "##box-abc", "table_"));
  EXPECT_FALSE(contains_pair(pairs, "forearm_link", "table_"));
  EXPECT_FALSE(contains_pair(pairs, "upper_arm_link", "table_"));
  EXPECT_FALSE(contains_pair(pairs, "forearm_link", "<octomap>"));
  EXPECT_FALSE(contains_pair(pairs, "wrist_2_link", "camera_link"));
}

TEST(ReleasePlanningAllowedContacts, ACMAllowsExpectedAndRejectsDisallowedPairs)
{
  collision_detection::AllowedCollisionMatrix acm;

  const auto pairs = grasp_execution::moveit2::detail::get_release_planning_allowed_pairs(
    "box_demo",
    {"gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"},
    {"table_"});
  grasp_execution::moveit2::detail::set_allowed_collision_pairs(acm, pairs, true);

  EXPECT_TRUE(is_always_allowed(acm, "<octomap>", "gripper_finger1_finger_tip_link"));
  EXPECT_TRUE(is_always_allowed(acm, "<octomap>", "gripper_finger2_finger_tip_link"));
  EXPECT_TRUE(is_always_allowed(acm, "box_demo", "<octomap>"));
  EXPECT_TRUE(is_always_allowed(acm, "#box_demo", "<octomap>"));
  EXPECT_TRUE(is_always_allowed(acm, "box_demo", "table_"));
  EXPECT_TRUE(is_always_allowed(acm, "#box_demo", "table_"));

  EXPECT_FALSE(is_always_allowed(acm, "forearm_link", "table_"));
  EXPECT_FALSE(is_always_allowed(acm, "upper_arm_link", "table_"));
  EXPECT_FALSE(is_always_allowed(acm, "forearm_link", "<octomap>"));
  EXPECT_FALSE(is_always_allowed(acm, "wrist_2_link", "camera_link"));
}
