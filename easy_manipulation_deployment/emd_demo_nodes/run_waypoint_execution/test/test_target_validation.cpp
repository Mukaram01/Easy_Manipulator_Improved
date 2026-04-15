#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "emd_msgs/msg/grasp_method.hpp"
#include "emd_msgs/msg/grasp_target.hpp"
#include "run_waypoint_execution/target_validation.hpp"

namespace
{

TEST(TargetValidation, RejectsNullTarget)
{
  const emd_msgs::msg::GraspMethod * grasp_method = nullptr;
  const geometry_msgs::msg::PoseStamped * grasp_pose = nullptr;

  EXPECT_FALSE(run_waypoint_execution::validate_grasp_target_selection(
      rclcpp::get_logger("test_target_validation"),
      nullptr,
      "target-null",
      grasp_method,
      grasp_pose));
  EXPECT_EQ(grasp_method, nullptr);
  EXPECT_EQ(grasp_pose, nullptr);
}

TEST(TargetValidation, RejectsEmptyGraspMethods)
{
  auto target = std::make_shared<emd_msgs::msg::GraspTarget>();

  const emd_msgs::msg::GraspMethod * grasp_method = nullptr;
  const geometry_msgs::msg::PoseStamped * grasp_pose = nullptr;

  EXPECT_FALSE(run_waypoint_execution::validate_grasp_target_selection(
      rclcpp::get_logger("test_target_validation"),
      target,
      "target-empty-methods",
      grasp_method,
      grasp_pose));
  EXPECT_EQ(grasp_method, nullptr);
  EXPECT_EQ(grasp_pose, nullptr);
}

TEST(TargetValidation, RejectsEmptyGraspPoses)
{
  auto target = std::make_shared<emd_msgs::msg::GraspTarget>();
  target->grasp_methods.emplace_back();
  target->grasp_methods[0].ee_id = "test-ee";

  const emd_msgs::msg::GraspMethod * grasp_method = nullptr;
  const geometry_msgs::msg::PoseStamped * grasp_pose = nullptr;

  EXPECT_FALSE(run_waypoint_execution::validate_grasp_target_selection(
      rclcpp::get_logger("test_target_validation"),
      target,
      "target-empty-poses",
      grasp_method,
      grasp_pose));
  EXPECT_NE(grasp_method, nullptr);
  EXPECT_EQ(grasp_pose, nullptr);
}

TEST(TargetValidation, AcceptsValidFirstSelection)
{
  auto target = std::make_shared<emd_msgs::msg::GraspTarget>();
  target->grasp_methods.emplace_back();
  target->grasp_methods[0].ee_id = "test-ee";
  target->grasp_methods[0].grasp_poses.emplace_back();

  const emd_msgs::msg::GraspMethod * grasp_method = nullptr;
  const geometry_msgs::msg::PoseStamped * grasp_pose = nullptr;

  EXPECT_TRUE(run_waypoint_execution::validate_grasp_target_selection(
      rclcpp::get_logger("test_target_validation"),
      target,
      "target-valid",
      grasp_method,
      grasp_pose));
  ASSERT_NE(grasp_method, nullptr);
  ASSERT_NE(grasp_pose, nullptr);
  EXPECT_EQ(grasp_method->ee_id, "test-ee");
}

}  // namespace
