#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include "workcell_scene_status.hpp"

namespace fs = boost::filesystem;

TEST(WorkcellSceneStatus, ValidSceneOk)
{
  fs::path root = fs::temp_directory_path() / "wc_status_ok";
  fs::remove_all(root);
  fs::create_directories(root / "scenes" / "demo" / "launch");
  fs::create_directories(root / "assets" / "environment" / "table_description");
  std::ofstream(root / "scenes" / "demo" / "environment.yaml") << "robot:\n  name: ur5\nobject:\n  - name: table\n";
  std::ofstream(root / "scenes" / "demo" / "scene_manifest.yaml") << "scene: demo\n";
  std::ofstream(root / "scenes" / "demo" / "package.xml") << "<package/>\n";
  std::ofstream(root / "scenes" / "demo" / "CMakeLists.txt") << "cmake_minimum_required(VERSION 3.5)\n";
  std::ofstream(root / "scenes" / "demo" / "launch" / "demo.launch.py") << "# fake\n";

  auto report = workcell_builder::inspect_scene_status(root, root / "scenes", root / "assets", "demo");
  EXPECT_TRUE(report.environment_yaml_ok);
  EXPECT_TRUE(report.generated_files_ok);
  EXPECT_TRUE(report.blockers.empty());
}

TEST(WorkcellSceneStatus, MissingEnvironmentIsBlocker)
{
  fs::path root = fs::temp_directory_path() / "wc_status_missing_env";
  fs::remove_all(root);
  fs::create_directories(root / "scenes" / "demo");
  auto report = workcell_builder::inspect_scene_status(root, root / "scenes", root / "assets", "demo");
  EXPECT_FALSE(report.environment_yaml_ok);
  EXPECT_FALSE(report.blockers.empty());
}

TEST(WorkcellSceneStatus, MissingAssetWarns)
{
  fs::path root = fs::temp_directory_path() / "wc_status_missing_asset";
  fs::remove_all(root);
  fs::create_directories(root / "scenes" / "demo" / "launch");
  std::ofstream(root / "scenes" / "demo" / "environment.yaml") << "robot:\n  name: ur5\nobject:\n  - name: table\n";
  std::ofstream(root / "scenes" / "demo" / "package.xml") << "<package/>\n";
  std::ofstream(root / "scenes" / "demo" / "CMakeLists.txt") << "cmake_minimum_required(VERSION 3.5)\n";
  std::ofstream(root / "scenes" / "demo" / "launch" / "demo.launch.py") << "# fake\n";
  auto report = workcell_builder::inspect_scene_status(root, root / "scenes", root / "assets", "demo");
  EXPECT_FALSE(report.warnings.empty());
}

TEST(WorkcellSceneStatus, FakeHardwareCommand)
{
  fs::path root = fs::temp_directory_path() / "wc_status_cmd";
  fs::remove_all(root);
  fs::create_directories(root / "scenes" / "demo");
  auto report = workcell_builder::inspect_scene_status(root, root / "scenes", root / "assets", "demo");
  ASSERT_GE(report.next_commands.size(), 3U);
  EXPECT_NE(report.next_commands[2].find("use_fake_hardware:=true"), std::string::npos);
  EXPECT_FALSE(report.safety_notes.empty());
}

TEST(WorkcellSceneStatus, ScalarRobotAndMalformedYamlDoNotCrash)
{
  fs::path root = fs::temp_directory_path() / "wc_status_scalar_and_bad";
  fs::remove_all(root);
  fs::create_directories(root / "scenes" / "demo" / "launch");
  std::ofstream(root / "scenes" / "demo" / "environment.yaml") << "robot: ur5\nend_effector: rg2\nobject:\n  - table\n";
  std::ofstream(root / "scenes" / "demo" / "package.xml") << "<package/>\n";
  std::ofstream(root / "scenes" / "demo" / "CMakeLists.txt") << "cmake_minimum_required(VERSION 3.5)\n";
  std::ofstream(root / "scenes" / "demo" / "launch" / "demo.launch.py") << "# fake\n";
  auto report = workcell_builder::inspect_scene_status(root, root / "scenes", root / "assets", "demo");
  EXPECT_TRUE(report.environment_yaml_ok);
  EXPECT_FALSE(report.items.empty());
}

TEST(WorkcellSceneStatus, SnapshotMissingShowsSingleInfoWithoutContradiction)
{
  fs::path root = fs::temp_directory_path() / "wc_status_snapshot_info";
  fs::remove_all(root);
  fs::create_directories(root / "scenes" / "demo" / "launch");
  std::ofstream(root / "scenes" / "demo" / "environment.yaml") << "robot:\n  name: ur5\n";
  std::ofstream(root / "scenes" / "demo" / "package.xml") << "<package/>\n";
  std::ofstream(root / "scenes" / "demo" / "CMakeLists.txt") << "cmake_minimum_required(VERSION 3.5)\n";
  std::ofstream(root / "scenes" / "demo" / "launch" / "demo.launch.py") << "# fake\n";

  const auto report = workcell_builder::inspect_scene_status(root, root / "scenes", root / "assets", "demo");
  int snapshot_items = 0;
  for (const auto & item : report.items) {
    if (item.name == "Detection snapshot available") {
      snapshot_items++;
      EXPECT_EQ(item.status, "INFO");
      EXPECT_EQ(item.message, "EPD-compatible offline metadata");
    }
  }
  EXPECT_EQ(snapshot_items, 1);
}

TEST(WorkcellSceneStatus, GroupedPerceptionCameraAndPreviewLinesPresent)
{
  fs::path root = fs::temp_directory_path() / "wc_status_grouped_lines";
  fs::remove_all(root);
  fs::create_directories(root / "scenes" / "demo" / "launch");
  std::ofstream(root / "scenes" / "demo" / "environment.yaml") << "robot:\n  name: ur5\ncameras:\n  - name: camera_01\n    package: realsense2_description\n";
  std::ofstream(root / "scenes" / "demo" / "package.xml") << "<package/>\n";
  std::ofstream(root / "scenes" / "demo" / "CMakeLists.txt") << "cmake_minimum_required(VERSION 3.5)\n";
  std::ofstream(root / "scenes" / "demo" / "launch" / "demo.launch.py") << "# fake\n";

  const auto report = workcell_builder::inspect_scene_status(root, root / "scenes", root / "assets", "demo");
  auto has_line = [&](const std::string &name) {
    return std::any_of(report.items.begin(), report.items.end(), [&](const auto &item) { return item.name == name; });
  };
  EXPECT_TRUE(has_line("Detection mapping available"));
  EXPECT_TRUE(has_line("Camera configured"));
  EXPECT_TRUE(has_line("Task intent preview"));
}
