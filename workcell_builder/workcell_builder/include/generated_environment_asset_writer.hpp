#pragma once

#include <boost/filesystem/path.hpp>
#include <string>

#include "generated_stl_writer.hpp"

namespace workcell_builder
{
struct GeneratedEnvironmentAssetResult
{
  bool success = false;
  std::string object_name;
  boost::filesystem::path asset_root;
  boost::filesystem::path yaml_path;
  boost::filesystem::path urdf_xacro_path;
  boost::filesystem::path visual_mesh_path;
  boost::filesystem::path collision_mesh_path;
  std::string error_message;
  std::string suggested_name;
  bool already_exists = false;
};

GeneratedEnvironmentAssetResult write_generated_environment_asset(
  const PrimitiveSpec & spec,
  const boost::filesystem::path & assets_path,
  bool overwrite = false);
}  // namespace workcell_builder
