#include <gtest/gtest.h>

#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_command_language/waypoint.h>

#include <Eigen/Core>

using namespace tesseract_planning;

TEST(SimplePlannerInterpolationUnit, ReordersMismatchedJointWaypointNames)  // NOLINT
{
  std::vector<std::string> names1{ "joint_a", "joint_b", "joint_c" };
  std::vector<std::string> names2{ "joint_b", "joint_c", "joint_a" };

  Eigen::VectorXd start_position(3);
  start_position << 0.0, 1.0, 2.0;
  WaypointPoly start(JointWaypoint(names1, start_position));

  Eigen::VectorXd stop_position(3);
  stop_position << 10.0, 20.0, 30.0;  // Aligned with names2 order
  WaypointPoly stop(JointWaypoint(names2, stop_position));

  const auto results = interpolate_waypoint(start, stop, 2);

  ASSERT_EQ(results.size(), 3U);
  for (const auto& waypoint : results)
  {
    ASSERT_TRUE(waypoint.isJointWaypoint());
    EXPECT_EQ(waypoint.as<JointWaypointPoly>().getNames(), names1);
  }

  const Eigen::VectorXd& stop_aligned = results.back().as<JointWaypointPoly>().getPosition();
  Eigen::VectorXd expected_stop(3);
  expected_stop << 30.0, 10.0, 20.0;  // Reordered to match names1
  EXPECT_TRUE(stop_aligned.isApprox(expected_stop, 1e-8));

  const Eigen::VectorXd& middle = results.at(1).as<JointWaypointPoly>().getPosition();
  Eigen::VectorXd expected_middle(3);
  expected_middle << 15.0, 5.5, 11.0;
  EXPECT_TRUE(middle.isApprox(expected_middle, 1e-8));
}

TEST(SimplePlannerInterpolationUnit, ThrowsWhenJointNamesDoNotMatch)  // NOLINT
{
  std::vector<std::string> names1{ "joint_a", "joint_b" };
  std::vector<std::string> names2{ "joint_b", "joint_c" };

  WaypointPoly start(JointWaypoint(names1, Eigen::Vector2d(0.0, 1.0)));
  WaypointPoly stop(JointWaypoint(names2, Eigen::Vector2d(10.0, 20.0)));

  EXPECT_THROW(interpolate_waypoint(start, stop, 1), std::runtime_error);
}

