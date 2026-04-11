#include <gtest/gtest.h>

#include <limits>

#include "emd/grasp_execution/moveit2/cartesian_planning_options.hpp"

namespace grasp_execution::moveit2
{

TEST(CartesianPlanningValidation, RejectsEmptyWaypoints)
{
  const auto result = validate_cartesian_request(0U, true, 0.01, 0.0);
  EXPECT_EQ(result.status, CartesianPlanStatus::kInvalidInput);
  EXPECT_FALSE(result.message.empty());
}

TEST(CartesianPlanningValidation, RejectsInvalidLink)
{
  const auto result = validate_cartesian_request(1U, false, 0.01, 0.0);
  EXPECT_EQ(result.status, CartesianPlanStatus::kInvalidInput);
}

TEST(CartesianPlanningValidation, RejectsNonPositiveStep)
{
  const auto result = validate_cartesian_request(1U, true, 0.0, 0.0);
  EXPECT_EQ(result.status, CartesianPlanStatus::kInvalidInput);
}

TEST(CartesianPlanningValidation, RejectsNonFiniteJumpThreshold)
{
  const auto result = validate_cartesian_request(1U, true, 0.01, std::numeric_limits<double>::infinity());
  EXPECT_EQ(result.status, CartesianPlanStatus::kInvalidInput);
}

TEST(CartesianPlanningConstraintConfig, UnconstrainedWithoutCollisionAvoidance)
{
  CartesianPlanningOptions options;
  options.avoid_collisions = false;
  const auto config = build_cartesian_constraint_config(options);
  EXPECT_FALSE(config.has_path_constraints);
  EXPECT_FALSE(config.build_constraint_fn);
}

TEST(CartesianPlanningConstraintConfig, ConstrainedWhenPathConstraintsProvided)
{
  CartesianPlanningOptions options;
  options.avoid_collisions = false;
  moveit_msgs::msg::JointConstraint jc;
  jc.joint_name = "joint_1";
  jc.position = 0.0;
  jc.tolerance_above = 0.01;
  jc.tolerance_below = 0.01;
  jc.weight = 1.0;
  options.path_constraints.joint_constraints.push_back(jc);

  const auto config = build_cartesian_constraint_config(options);
  EXPECT_TRUE(config.has_path_constraints);
  EXPECT_TRUE(config.build_constraint_fn);
}

TEST(CartesianPlanningConstraintConfig, ConstrainedWhenCollisionAvoidanceEnabled)
{
  CartesianPlanningOptions options;
  options.avoid_collisions = true;
  const auto config = build_cartesian_constraint_config(options);
  EXPECT_FALSE(config.has_path_constraints);
  EXPECT_TRUE(config.build_constraint_fn);
}

}  // namespace grasp_execution::moveit2
