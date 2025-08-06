#include <gtest/gtest.h>
#include "emd/waypoint_execution/waypoint_parser.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

TEST(WaypointParserTest, AddRPYToPose)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  ASSERT_TRUE(emd::WayPointParser::internal::add_rpy_to_pose(pose, 0.0, 0.0, M_PI / 2));
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  EXPECT_NEAR(q.x(), 0.0, 1e-6);
  EXPECT_NEAR(q.y(), 0.0, 1e-6);
  EXPECT_NEAR(q.z(), std::sin(M_PI / 4), 1e-6);
  EXPECT_NEAR(q.w(), std::cos(M_PI / 4), 1e-6);
}

TEST(WaypointParserTest, LoadWaypoints)
{
  std::vector<emd::WayPoint> waypoints;
  std::string path = std::string(TEST_DIRECTORY) + "/waypoints.json";
  ASSERT_TRUE(emd::WayPointParser::load_waypoints(waypoints, path));
  ASSERT_EQ(waypoints.size(), 3u);
  EXPECT_EQ(waypoints[0].action_type, "move");
  EXPECT_EQ(waypoints[1].action_type, "move");
  EXPECT_EQ(waypoints[2].action_type, "collision");
}
