#include <vector>
#include <string>

#include "gtest/gtest.h"
#include "emd/dynamic_safety/replanner_flatten_utils.hpp"

namespace
{
using dynamic_safety::has_monotonic_timestamps;
using dynamic_safety::reorder_joint;
using dynamic_safety::set_start_kinematics;

TEST(ReplannerFlattenUtils, ReorderJointStateReordersVelocitiesAndAccelerations)
{
  std::vector<std::string> reference{"j1", "j2", "j3"};
  std::vector<std::string> current{"j3", "j1", "j2"};

  trajectory_msgs::msg::JointTrajectoryPoint state;
  state.positions = {3.0, 1.0, 2.0};
  state.velocities = {0.3, 0.1, 0.2};
  state.accelerations = {3.0, 1.0, 2.0};

  ASSERT_TRUE(reorder_joint(reference, current, state));
  EXPECT_EQ(current, reference);
  EXPECT_EQ(state.positions, (std::vector<double>{1.0, 2.0, 3.0}));
  EXPECT_EQ(state.velocities, (std::vector<double>{0.1, 0.2, 0.3}));
  EXPECT_EQ(state.accelerations, (std::vector<double>{1.0, 2.0, 3.0}));
}

TEST(ReplannerFlattenUtils, SetStartKinematicsUsesNonZeroInitialVelocity)
{
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = {0.0, 0.0};

  trajectory_msgs::msg::JointTrajectoryPoint current;
  current.positions = {0.1, -0.2};
  current.velocities = {0.5, -0.1};
  current.accelerations = {1.2, -0.3};

  set_start_kinematics(start, current);
  EXPECT_EQ(start.velocities, current.velocities);
  EXPECT_EQ(start.accelerations, current.accelerations);
}

TEST(ReplannerFlattenUtils, MonotonicTimestampValidationHandlesShortSegments)
{
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> short_points(2);
  short_points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
  short_points[1].time_from_start = rclcpp::Duration::from_seconds(0.0);
  EXPECT_TRUE(has_monotonic_timestamps(short_points));

  short_points[1].time_from_start = rclcpp::Duration::from_seconds(-0.1);
  EXPECT_FALSE(has_monotonic_timestamps(short_points));
}

}  // namespace
