#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "emd/grasp_execution/context.hpp"
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

TEST(TargetValidation, RejectsUnknownEndEffectorMapping)
{
  grasp_execution::WorkcellContext workcell_context;
  workcell_context.load_ee(
    "manipulator_a", "known_ee", "known-brand", "known_link", 0.01, "driver_plugin",
    "driver_controller");

  std::string planning_group = "stale_group";
  std::string ee_link = "stale_link";
  double clearance = 0.42;

  EXPECT_FALSE(run_waypoint_execution::resolve_end_effector_mapping(
      workcell_context, "unknown-ee-id", planning_group, ee_link, clearance));
  EXPECT_TRUE(planning_group.empty());
  EXPECT_TRUE(ee_link.empty());
  EXPECT_DOUBLE_EQ(clearance, 0.0);
}

}  // namespace
