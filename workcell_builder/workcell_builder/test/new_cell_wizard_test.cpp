#include <gtest/gtest.h>

#include "gui/new_cell_wizard.h"

TEST(NewCellWizard, ScenePackageNameValidation)
{
  EXPECT_TRUE(NewCellWizard::is_valid_package_name("ur5_pick_place"));
  EXPECT_FALSE(NewCellWizard::is_valid_package_name("UR5_bad"));
  EXPECT_FALSE(NewCellWizard::is_valid_package_name("bad-name"));
  EXPECT_FALSE(NewCellWizard::is_valid_package_name("1bad"));
}

TEST(NewCellWizard, DefaultGripperRpy)
{
  EXPECT_EQ(NewCellWizard::default_gripper_rpy_text().toStdString(), "-1.5708, -1.5708, 0");
}

TEST(NewCellWizard, RecommendedLayoutDefaults)
{
  const auto items = NewCellWizard::recommended_environment_assets();
  EXPECT_TRUE(items.contains("workbench_01"));
  EXPECT_TRUE(items.contains("source_bin_01"));
  EXPECT_TRUE(items.contains("place_fixture_01"));
  EXPECT_TRUE(items.contains("camera_01"));
  EXPECT_TRUE(items.contains("safety_zone_01"));
}

TEST(NewCellWizard, UrDefaults)
{
  const QString family = "Universal Robots / UR";

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, "UR5").toStdString(), "base_link");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, "UR5").toStdString(), "ee_link");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, "UR5").toStdString(), "manipulator");

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, "UR3").toStdString(), "base_link");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, "UR3").toStdString(), "ee_link");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, "UR3").toStdString(), "manipulator");

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, "UR10").toStdString(), "base_link");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, "UR10").toStdString(), "ee_link");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, "UR10").toStdString(), "manipulator");
}

TEST(NewCellWizard, FrankaDefaults)
{
  const QString family = "Franka / Panda";

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, "Panda").toStdString(), "panda_link0");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, "Panda").toStdString(), "panda_hand");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, "Panda").toStdString(), "panda_arm");
}

TEST(NewCellWizard, PlaceholderDefaults)
{
  const QString family = "Delta";
  const QString model = "delta_placeholder";

  EXPECT_EQ(NewCellWizard::default_robot_base_link(family, model).toStdString(), "base_link");
  EXPECT_EQ(NewCellWizard::default_robot_tip_link(family, model).toStdString(), "tool0_placeholder");
  EXPECT_EQ(NewCellWizard::default_robot_planning_group(family, model).toStdString(), "preview_group");
}

TEST(NewCellWizard, RobotiqDefaults)
{
  EXPECT_EQ(NewCellWizard::default_end_effector_attach_link("robotiq_85").toStdString(), "gripper_base_link");
  EXPECT_EQ(NewCellWizard::default_end_effector_tcp_link("robotiq_85").toStdString(), "ee_palm");
}
