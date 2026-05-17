#include <gtest/gtest.h>

#include <filesystem>

#include <fstream>
#include <sstream>

#include "placed_object_preview_writer.hpp"

TEST(PlacedObjectPreviewWriter, WritesPreviewFilesWithCameraEntries)
{
  const auto temp = std::filesystem::temp_directory_path() / "wb_preview_test";
  std::filesystem::create_directories(temp);

  workcell_builder::PlacedObject object;
  object.name = "camera_target";
  object.source_type = "external";
  object.mesh_path = "package://example/mesh.stl";

  workcell_builder::PlacedObjectPreviewWriter writer;
  std::string out_dir;
  std::vector<std::string> warnings;
  ASSERT_TRUE(writer.write_preview("test_scene", {object}, &out_dir, &warnings));

  const std::filesystem::path xacro_path = std::filesystem::path(out_dir) / "placed_objects_preview.urdf.xacro";
  const std::filesystem::path camera_yaml_path = std::filesystem::path(out_dir) / "camera_frustum_preview.yaml";

  ASSERT_TRUE(std::filesystem::exists(xacro_path));
  ASSERT_TRUE(std::filesystem::exists(camera_yaml_path));

  std::ifstream xacro_file(xacro_path.string());
  std::stringstream xacro_buffer;
  xacro_buffer << xacro_file.rdbuf();
  EXPECT_NE(xacro_buffer.str().find("camera_01_link"), std::string::npos);

  std::ifstream camera_yaml_file(camera_yaml_path.string());
  std::stringstream camera_yaml_buffer;
  camera_yaml_buffer << camera_yaml_file.rdbuf();
  EXPECT_NE(camera_yaml_buffer.str().find("camera_placements"), std::string::npos);
}
