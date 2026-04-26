#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "run_grasp_execution/grasp_candidate_utils.hpp"

namespace
{

geometry_msgs::msg::PoseStamped make_candidate(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "camera_color_optical_frame";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

}  // namespace

TEST(GraspCandidateUtils, ExpandsWhenCountBelowMinimumAndKeepsFirstCandidate)
{
  const std::vector<geometry_msgs::msg::PoseStamped> input = {
    make_candidate(-0.55, 0.11, 0.04)};

  const auto expanded = run_grasp_execution::expand_grasp_candidates_with_fallbacks(input, 8);

  EXPECT_GE(expanded.size(), 8u);
  EXPECT_EQ(expanded.front().pose.position.x, input.front().pose.position.x);
  EXPECT_EQ(expanded.front().pose.position.y, input.front().pose.position.y);
  EXPECT_EQ(expanded.front().pose.position.z, input.front().pose.position.z);
  EXPECT_EQ(expanded.front().pose.orientation.w, input.front().pose.orientation.w);
}

TEST(GraspCandidateUtils, FallbackGenerationAvoidsDuplicatePoses)
{
  const std::vector<geometry_msgs::msg::PoseStamped> input = {
    make_candidate(-0.55, 0.11, 0.04)};

  const auto expanded = run_grasp_execution::expand_grasp_candidates_with_fallbacks(input, 12);

  std::set<std::string> dedup;
  for (const auto & pose : expanded) {
    dedup.insert(run_grasp_execution::pose_dedup_key(pose));
  }
  EXPECT_EQ(dedup.size(), expanded.size());
}

TEST(GraspCandidateUtils, CollisionReasonCategorizedAsRobotTable)
{
  const std::string reason =
    "IK solution in collision: forearm_link<->table_, table_<->upper_arm_link";
  EXPECT_EQ(
    run_grasp_execution::categorize_candidate_rejection_reason(reason),
    "broad robot/table collision");
}
