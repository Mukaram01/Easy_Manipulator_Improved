#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>

#include "attributes/end_effector.h"
#include "attributes/robot.h"
#include "attributes/scene.h"
#include "scene_xacro_parser.h"

TEST(SceneXacroToolAttachmentTest, EmitsConfiguredEndEffectorMountRpySnippet)
{
  Scene scene;
  scene.name = "demo_scene";
  scene.robot_loaded = true;
  scene.ee_loaded = true;

  Robot robot;
  robot.name = "custom_arm";
  robot.brand = "custom";
  robot.filepath = "$(find custom_description)/urdf/custom_arm.urdf.xacro";
  robot.base_link = "base_link";
  scene.robot_vector.push_back(robot);

  EndEffector ee;
  ee.name = "robotiq_85";
  ee.brand = "robotiq_85_gripper";
  ee.filepath = "$(find robotiq_85_description)/urdf/robotiq_85_gripper.urdf.xacro";
  ee.base_link = "gripper_base_link";
  ee.robot_link = "tool0";
  ee.origin.is_origin = true;
  ee.origin.x = 0.0F;
  ee.origin.y = 0.0F;
  ee.origin.z = 0.0F;
  ee.origin.roll = -1.5708F;
  ee.origin.pitch = -1.5708F;
  ee.origin.yaw = 0.0F;
  scene.ee_vector.push_back(ee);

  const boost::filesystem::path out_dir =
    boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("scene_xacro_test_%%%%-%%%%");
  boost::filesystem::create_directories(out_dir);
  const boost::filesystem::path out_file = out_dir / "scene.urdf.xacro";

  generate_scene_xacro(scene, out_file.string());

  std::ifstream file(out_file.string());
  ASSERT_TRUE(file.is_open());
  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::string content = buffer.str();

  EXPECT_NE(content.find("rpy=\"-1.5708 -1.5708 0.0\""), std::string::npos);

  boost::filesystem::remove_all(out_dir);
}
