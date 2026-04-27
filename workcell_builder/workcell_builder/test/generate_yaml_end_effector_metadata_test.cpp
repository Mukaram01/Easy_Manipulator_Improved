#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include "attributes/end_effector.h"
#include "attributes/robot.h"
#include "attributes/scene.h"
#include "yaml_parser/generate_yaml.h"

namespace {

Scene make_base_scene()
{
  Scene scene;

  Robot robot;
  robot.name = "ur5";
  robot.brand = "universal_robot";
  robot.filepath = "$(find ur_description)/urdf/ur5.urdf.xacro";
  robot.base_link = "base_link";
  robot.ee_link = "tool0";
  robot.robot_links = {"base_link", "tool0"};
  scene.robot_vector.push_back(robot);

  return scene;
}

YAML::Node generate_environment_yaml(const Scene & scene)
{
  const boost::filesystem::path temp_dir =
    boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("workcell_yaml_test_%%%%-%%%%-%%%%");
  boost::filesystem::create_directories(temp_dir);

  const bool generated = GenerateYAML::generate_yaml(scene, temp_dir.string(), temp_dir, temp_dir);
  EXPECT_TRUE(generated);

  const boost::filesystem::path environment_yaml_path = temp_dir / "environment.yaml";
  EXPECT_TRUE(boost::filesystem::exists(environment_yaml_path));

  YAML::Node root = YAML::LoadFile(environment_yaml_path.string());

  boost::filesystem::remove_all(temp_dir);
  return root;
}

}  // namespace

TEST(GenerateYAMLMetadataTest, FingerGripperIncludesNormalizedAndLegacyKeys)
{
  Scene scene = make_base_scene();

  EndEffector ee;
  ee.name = "robotiq_85";
  ee.brand = "robotiq";
  ee.filepath = "$(find robotiq_85_description)/urdf/robotiq_85_gripper.urdf.xacro";
  ee.base_link = "robotiq_85_base_link";
  ee.robot_link = "tool0";
  ee.ee_type = "finger";
  ee.attribute_1 = 2;
  ee.ee_links = {"robotiq_85_base_link", "left_finger", "right_finger"};
  scene.ee_vector.push_back(ee);

  const YAML::Node root = generate_environment_yaml(scene);
  const YAML::Node end_effector = root["end_effector"];

  ASSERT_TRUE(end_effector);
  EXPECT_EQ(end_effector["name"].as<std::string>(), "robotiq_85");
  EXPECT_EQ(end_effector["brand"].as<std::string>(), "robotiq");
  EXPECT_EQ(end_effector["ee_type"].as<std::string>(), "finger");
  EXPECT_EQ(end_effector["attributes"]["fingers"].as<int>(), 2);
  ASSERT_TRUE(end_effector["links"]);

  EXPECT_EQ(end_effector["planner_id"].as<std::string>(), "robotiq");
  EXPECT_EQ(end_effector["grasp_frame"].as<std::string>(), "robotiq_85_base_link");
  EXPECT_EQ(end_effector["tcp_link"].as<std::string>(), "robotiq_85_base_link");
  EXPECT_EQ(end_effector["gripper_type"].as<std::string>(), "finger");
  EXPECT_TRUE(end_effector["spawn_gripper_controller"].as<bool>());
  EXPECT_EQ(end_effector["finger_count"].as<int>(), 2);
}

TEST(GenerateYAMLMetadataTest, SuctionGripperIncludesNormalizedKeysWithoutFingerCount)
{
  Scene scene = make_base_scene();

  EndEffector ee;
  ee.name = "single_suction_gripper";
  ee.brand = "airpick_vendor";
  ee.filepath = "$(find single_suction_description)/urdf/single_suction_gripper.urdf.xacro";
  ee.base_link = "single_suction_gripper_base_link";
  ee.robot_link = "tool0";
  ee.ee_type = "suction";
  ee.gripper_type = "airpick";
  ee.attribute_1 = 1;
  ee.attribute_2 = 1;
  ee.ee_links = {"single_suction_gripper_base_link"};
  scene.ee_vector.push_back(ee);

  const YAML::Node root = generate_environment_yaml(scene);
  const YAML::Node end_effector = root["end_effector"];

  ASSERT_TRUE(end_effector);
  EXPECT_EQ(end_effector["ee_type"].as<std::string>(), "suction");
  EXPECT_EQ(end_effector["attributes"]["array_width"].as<int>(), 1);
  EXPECT_EQ(end_effector["attributes"]["array_height"].as<int>(), 1);

  EXPECT_EQ(end_effector["planner_id"].as<std::string>(), "airpick_vendor");
  EXPECT_EQ(end_effector["gripper_type"].as<std::string>(), "airpick");
  EXPECT_FALSE(end_effector["spawn_gripper_controller"].as<bool>());
  EXPECT_FALSE(end_effector["finger_count"]);
}
