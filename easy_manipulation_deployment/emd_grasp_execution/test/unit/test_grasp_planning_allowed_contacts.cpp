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

}  // namespace

TEST(GraspPlanningAllowedContacts, IncludesOctomapAndTargetPairsForFingertips)
{
  const std::vector<std::string> touch_links = {
    "gripper_finger1_finger_tip_link",
    "gripper_finger2_finger_tip_link",
  };

  const auto pairs =
    grasp_execution::moveit2::detail::get_grasp_planning_allowed_pairs("#box_demo", touch_links);

  EXPECT_TRUE(contains_pair(pairs, "<octomap>", "gripper_finger1_finger_tip_link"));
  EXPECT_TRUE(contains_pair(pairs, "<octomap>", "gripper_finger2_finger_tip_link"));
  EXPECT_TRUE(contains_pair(pairs, "#box_demo", "gripper_finger1_finger_tip_link"));
  EXPECT_TRUE(contains_pair(pairs, "#box_demo", "gripper_finger2_finger_tip_link"));
}

TEST(GraspPlanningAllowedContacts, DoesNotIncludeArmOrSupportContactPairs)
{
  const std::vector<std::string> touch_links = {
    "gripper_finger1_finger_tip_link",
    "gripper_finger2_finger_tip_link",
  };

  const auto pairs =
    grasp_execution::moveit2::detail::get_grasp_planning_allowed_pairs("#box_demo", touch_links);

  EXPECT_FALSE(contains_pair(pairs, "forearm_link", "table_"));
  EXPECT_FALSE(contains_pair(pairs, "upper_arm_link", "table_"));
  EXPECT_FALSE(contains_pair(pairs, "forearm_link", "<octomap>"));
}
