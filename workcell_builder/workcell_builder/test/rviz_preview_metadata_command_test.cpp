#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <fstream>

#include "rviz_preview_runner.hpp"
#include "workcell_studio_scene_browser.hpp"

namespace fs = boost::filesystem;

namespace {

workcell_builder::WorkcellStudioSceneInfo runnable_scene()
{
  workcell_builder::WorkcellStudioSceneInfo scene;
  scene.scene_name = "ur5_2f_test";
  scene.has_package_xml = true;
  scene.has_launch_demo = true;
  scene.has_task_intent = true;
  scene.status = "READY";
  return scene;
}

}  // namespace

TEST(RvizPreviewMetadataCommandTest, UsesMetadataDerivedLaunchPackageAndFile)
{
  auto scene = runnable_scene();
  scene.launch_metadata_present = true;
  scene.launch_package = "custom_generated_pkg";
  scene.launch_file = "custom_preview.launch.py";
  scene.launch_metadata_file_exists = true;

  const QString command = workcell_builder::build_command(scene);

  EXPECT_EQ(
    command,
    "ros2 launch custom_generated_pkg custom_preview.launch.py use_fake_hardware:=true launch_rviz:=true");
}

TEST(RvizPreviewMetadataCommandTest, FallsBackToSceneNameAndDemoLaunch)
{
  const auto scene = runnable_scene();

  const QString command = workcell_builder::build_command(scene);

  EXPECT_EQ(command, "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true");
}

TEST(RvizPreviewMetadataCommandTest, BlocksMissingMetadataLaunchFileClearly)
{
  auto scene = runnable_scene();
  scene.launch_metadata_present = true;
  scene.launch_package = "custom_generated_pkg";
  scene.launch_file = "missing_preview.launch.py";
  scene.launch_metadata_file_exists = false;
  scene.launch_metadata_warning = "scene_manifest.yaml launch file missing: /tmp/scene/launch/missing_preview.launch.py";

  const auto status = workcell_builder::validate_readiness(scene, fs::current_path());

  EXPECT_FALSE(status.ready);
  EXPECT_NE(status.blocker_reason.indexOf("scene_manifest.yaml launch file missing"), -1);
  EXPECT_NE(status.blocker_reason.indexOf("missing_preview.launch.py"), -1);
}

TEST(RvizPreviewMetadataCommandTest, DiscoversManifestLaunchMetadata)
{
  const fs::path root = fs::temp_directory_path() / fs::unique_path("workcell_scene_metadata_test_%%%%%%");
  const fs::path scene_dir = root / "src" / "easy_manipulation_deployment" / "scenes" / "metadata_scene";
  fs::create_directories(scene_dir / "launch");
  std::ofstream(scene_dir / "environment.yaml") << "robot: {name: ur5}\n";
  std::ofstream(scene_dir / "package.xml") << "<package format=\"3\"><name>metadata_pkg</name></package>\n";
  std::ofstream(scene_dir / "launch" / "custom.launch.py") << "# launch\n";
  std::ofstream(scene_dir / "scene_manifest.yaml")
    << "scene:\n"
    << "  name: metadata_scene\n"
    << "  package: metadata_pkg\n"
    << "files:\n"
    << "  launch: launch/custom.launch.py\n";

  const auto result = workcell_builder::discover_workcell_studio_scenes(root);

  ASSERT_EQ(result.scenes.size(), 1U);
  EXPECT_EQ(result.scenes[0].launch_package, "metadata_pkg");
  EXPECT_EQ(result.scenes[0].launch_file, "custom.launch.py");
  EXPECT_TRUE(result.scenes[0].launch_metadata_file_exists);

  fs::remove_all(root);
}

TEST(RvizPreviewMetadataCommandTest, FakeHardwareAndRvizRemainDefaultWithoutRealHardwareFlags)
{
  auto scene = runnable_scene();
  scene.launch_package = "custom_generated_pkg";
  scene.launch_file = "custom_preview.launch.py";

  const QString command = workcell_builder::build_command(scene);

  EXPECT_NE(command.indexOf("use_fake_hardware:=true"), -1);
  EXPECT_NE(command.indexOf("launch_rviz:=true"), -1);
  EXPECT_EQ(command.indexOf("use_fake_hardware:=false"), -1);
  EXPECT_EQ(command.indexOf("real_hardware:=true"), -1);
  EXPECT_EQ(command.indexOf("runtime_execution_enabled:=true"), -1);
}

TEST(RvizPreviewMetadataCommandTest, BuildsMetadataSelectedPackageDependencyClosureInDerivedWorkspace)
{
  auto scene = runnable_scene();
  scene.launch_package = "metadata_selected_scene";
  const QString command = workcell_builder::build_selected_package_command(scene, "/home/user/workcell_ws");

  EXPECT_EQ(
    command,
    "source /opt/ros/humble/setup.bash && cd '/home/user/workcell_ws' && "
    "colcon build --symlink-install --packages-up-to 'metadata_selected_scene'");
  EXPECT_EQ(command.indexOf("--packages-select"), -1);
  EXPECT_EQ(command.indexOf("ur5_2f_test"), -1);
  EXPECT_EQ(command.indexOf("--allow-overriding"), -1);
}

TEST(RvizPreviewMetadataCommandTest, DiscoveryAndLaunchSourceBuiltOverlay)
{
  const auto scene = runnable_scene();

  EXPECT_EQ(
    workcell_builder::package_prefix_check_command(scene, "/home/user/workcell_ws"),
    "source /opt/ros/humble/setup.bash && source '/home/user/workcell_ws/install/setup.bash' && "
    "ros2 pkg prefix 'ur5_2f_test'");
  const QString launch = workcell_builder::build_launch_shell_command(scene, "/home/user/workcell_ws");
  EXPECT_NE(launch.indexOf("source /opt/ros/humble/setup.bash"), -1);
  EXPECT_NE(launch.indexOf("source '/home/user/workcell_ws/install/setup.bash'"), -1);
  EXPECT_NE(launch.indexOf("exec ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true"), -1);
  EXPECT_TRUE(workcell_builder::launch_command_is_safe(launch));
}

TEST(RvizPreviewMetadataCommandTest, RejectsRealHardwareLaunchTokens)
{
  QString reason;
  EXPECT_FALSE(workcell_builder::launch_command_is_safe(
    "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=false launch_rviz:=true", &reason));
  EXPECT_FALSE(workcell_builder::launch_command_is_safe(
    "ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true ur_robot_driver:=true",
    &reason));
}
