#include <gtest/gtest.h>

#include "robot_tool_compatibility.hpp"

namespace {

Scene make_scene(const std::string & ee_name)
{
  Scene scene;
  scene.robot_loaded = true;
  Robot robot;
  robot.name = "ur5";
  scene.robot_vector.push_back(robot);

  scene.ee_loaded = true;
  EndEffector ee;
  ee.name = ee_name;
  scene.ee_vector.push_back(ee);
  return scene;
}

}  // namespace

TEST(RobotToolCompatibilityInferenceTest, InfersRobotiqFromNameVariants)
{
  const std::string config_root = "./assets/compatibility";

  auto result = evaluate_robot_tool_compatibility(make_scene("robotiq_85"), config_root);
  EXPECT_EQ(result.tool_id, "robotiq_2f85");

  result = evaluate_robot_tool_compatibility(make_scene("robotiq_2f"), config_root);
  EXPECT_EQ(result.tool_id, "robotiq_2f85");
}

TEST(RobotToolCompatibilityInferenceTest, InfersSuctionFromNameVariants)
{
  const std::string config_root = "./assets/compatibility";

  auto result = evaluate_robot_tool_compatibility(make_scene("onrobot_airpick4"), config_root);
  EXPECT_EQ(result.tool_id, "onrobot_airpick");

  result = evaluate_robot_tool_compatibility(make_scene("single_suction"), config_root);
  EXPECT_EQ(result.tool_id, "onrobot_airpick");
}

TEST(RobotToolCompatibilityInferenceTest, FallsBackToUnknown)
{
  const std::string config_root = "./assets/compatibility";
  const auto result = evaluate_robot_tool_compatibility(make_scene("unknown"), config_root);
  EXPECT_EQ(result.tool_id, "generic_unknown_tool");
}
