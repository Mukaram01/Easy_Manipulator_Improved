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
  EXPECT_TRUE(items.contains("Work table"));
  EXPECT_TRUE(items.contains("Source bin"));
  EXPECT_TRUE(items.contains("Place fixture"));
  EXPECT_TRUE(items.contains("RealSense D435i camera"));
  EXPECT_TRUE(items.contains("Safety zone"));
}
