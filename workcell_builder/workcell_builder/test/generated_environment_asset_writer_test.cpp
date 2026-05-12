#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include "generated_environment_asset_writer.hpp"
#include "gui/loadobjects.h"

TEST(GeneratedEnvironmentAssetWriter, CreatesExpectedAssetFiles)
{
  const auto temp = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("wb_assets_%%%%%%");
  boost::filesystem::create_directories(temp / "environment");

  for (auto t : {workcell_builder::PrimitiveType::kBox, workcell_builder::PrimitiveType::kTable,
    workcell_builder::PrimitiveType::kBinTray, workcell_builder::PrimitiveType::kConveyorPlaceholder,
    workcell_builder::PrimitiveType::kFixturePlate})
  {
    workcell_builder::PrimitiveSpec spec;
    spec.object_name = "gen_obj_" + std::to_string(static_cast<int>(t));
    spec.type = t;
    auto res = workcell_builder::write_generated_environment_asset(spec, temp, false);
    ASSERT_TRUE(res.success);
    EXPECT_TRUE(boost::filesystem::exists(res.yaml_path));
    EXPECT_TRUE(boost::filesystem::exists(res.urdf_xacro_path));
    EXPECT_TRUE(boost::filesystem::exists(res.visual_mesh_path));
    EXPECT_TRUE(boost::filesystem::exists(res.collision_mesh_path));

    auto duplicate = workcell_builder::write_generated_environment_asset(spec, temp, false);
    EXPECT_FALSE(duplicate.success);
    EXPECT_TRUE(duplicate.already_exists);
  }
}

TEST(GeneratedEnvironmentAssetWriter, DiscoveredByLoadObjects)
{
  const auto temp = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("wb_assets_%%%%%%");
  boost::filesystem::create_directories(temp / "environment");
  workcell_builder::PrimitiveSpec spec;
  spec.object_name = "discover_me";
  ASSERT_TRUE(workcell_builder::write_generated_environment_asset(spec, temp, false).success);

  LoadObjects load;
  load.assets_path = temp;
  load.get_all_objects();
  ASSERT_NE(std::find(load.available_objects.begin(), load.available_objects.end(), "discover_me"), load.available_objects.end());
}
