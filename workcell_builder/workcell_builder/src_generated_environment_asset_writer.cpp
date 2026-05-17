#include "generated_environment_asset_writer.hpp"
#include "placed_object_urdf_render.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>

namespace workcell_builder
{
namespace
{
std::string primitive_type_to_string(const PrimitiveType type)
{
  switch (type) {
    case PrimitiveType::kBox: return "box";
    case PrimitiveType::kTable: return "table";
    case PrimitiveType::kBinTray: return "bin";
    case PrimitiveType::kConveyorPlaceholder: return "conveyor_placeholder";
    case PrimitiveType::kFixturePlate: return "fixture_plate";
  }
  return "unknown";
}

std::string suggest_next_name(const boost::filesystem::path & environment_dir, const std::string & name)
{
  for (int i = 2; i < 1000; ++i) {
    const std::string candidate = name + "_" + std::to_string(i);
    if (!boost::filesystem::exists(environment_dir / (candidate + "_description"))) {
      return candidate;
    }
  }
  return name + "_new";
}
}  // namespace

GeneratedEnvironmentAssetResult write_generated_environment_asset(
  const PrimitiveSpec & raw_spec,
  const boost::filesystem::path & assets_path,
  bool overwrite)
{
  GeneratedEnvironmentAssetResult result;
  PrimitiveSpec spec = raw_spec;
  spec.object_name = sanitize_object_name(spec.object_name);
  result.object_name = spec.object_name;

  std::string reason;
  if (!validate_primitive_spec(spec, &reason)) {
    result.error_message = "Invalid primitive specification: " + reason;
    return result;
  }

  const auto environment_dir = assets_path / "environment";
  const auto asset_root = environment_dir / (spec.object_name + "_description");
  result.asset_root = asset_root;
  result.suggested_name = suggest_next_name(environment_dir, spec.object_name);

  if (boost::filesystem::exists(asset_root) && !overwrite) {
    result.already_exists = true;
    result.error_message = "Asset already exists: " + asset_root.string();
    return result;
  }

  const auto visual_dir = asset_root / "meshes" / "visual";
  const auto collision_dir = asset_root / "meshes" / "collision";
  const auto urdf_dir = asset_root / "urdf";
  result.visual_mesh_path = visual_dir / (spec.object_name + ".stl");
  result.collision_mesh_path = collision_dir / (spec.object_name + ".stl");
  result.yaml_path = asset_root / (spec.object_name + ".yaml");
  result.urdf_xacro_path = urdf_dir / (spec.object_name + ".urdf.xacro");

  boost::filesystem::create_directories(visual_dir);
  boost::filesystem::create_directories(collision_dir);
  boost::filesystem::create_directories(urdf_dir);

  if (!write_ascii_stl(spec, result.visual_mesh_path.string(), &reason) ||
    !write_ascii_stl(spec, result.collision_mesh_path.string(), &reason))
  {
    result.error_message = "Failed to write generated mesh: " + reason;
    return result;
  }

  std::ofstream yaml_out(result.yaml_path.string());
  yaml_out << spec.object_name << ":\n"
           << "  child_link: " << spec.object_name << "\n"
           << "  generated_by: workcell_builder\n"
           << "  primitive_type: " << primitive_type_to_string(spec.type) << "\n"
           << "  dimensions_m:\n"
           << "    length: " << spec.length_m << "\n"
           << "    width: " << spec.width_m << "\n"
           << "    height: " << spec.height_m << "\n"
           << "  created_from_ui: true\n"
           << "  visual_mesh: package://" << spec.object_name << "_description/meshes/visual/"
           << spec.object_name << ".stl\n"
           << "  collision_mesh: package://" << spec.object_name << "_description/meshes/collision/"
           << spec.object_name << ".stl\n"
           << "  links:\n"
           << "    " << spec.object_name << ":\n"
           << "      visual:\n"
           << "        name: " << spec.object_name << "\n"
           << "        geometry:\n"
           << "          filepath: package://" << spec.object_name << "_description/meshes/visual/" << spec.object_name << ".stl\n"
           << "      collision:\n"
           << "        name: " << spec.object_name << "_collision\n"
           << "        geometry:\n"
           << "          filepath: package://" << spec.object_name << "_description/meshes/collision/" << spec.object_name << ".stl\n"
           << "  " << spec.object_name << "_base_joint:\n"
           << "    ext_joint_type: fixed\n"
           << "    child_link: " << spec.object_name << "\n";

  std::ofstream urdf_out(result.urdf_xacro_path.string());
  PlacedObject urdf_object;
  urdf_object.name = spec.object_name;
  urdf_object.mesh_path = "package://" + spec.object_name + "_description/meshes/visual/" + spec.object_name + ".stl";
  urdf_object.collision_mesh = "package://" + spec.object_name + "_description/meshes/collision/" + spec.object_name + ".stl";
  urdf_out << "<?xml version=\"1.0\"?>\n"
           << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"" << spec.object_name << "\">\n"
           << render_placed_object_urdf_snippet(urdf_object, spec.object_name)
           << "</robot>\n";

  std::ofstream cmake_out((asset_root / "CMakeLists.txt").string());
  cmake_out << "cmake_minimum_required(VERSION 3.5)\nproject(" << spec.object_name << "_description)\n"
            << "find_package(ament_cmake REQUIRED)\n"
            << "install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})\n"
            << "ament_package()\n";

  std::ofstream pkg_out((asset_root / "package.xml").string());
  pkg_out << "<?xml version=\"1.0\"?>\n<package format=\"3\">\n"
          << "  <name>" << spec.object_name << "_description</name>\n"
          << "  <version>0.0.1</version>\n"
          << "  <description>Generated environment object asset</description>\n"
          << "  <maintainer email=\"noreply@example.com\">workcell_builder</maintainer>\n"
          << "  <license>Apache-2.0</license>\n"
          << "  <buildtool_depend>ament_cmake</buildtool_depend>\n"
          << "</package>\n";

  result.success = true;
  return result;
}
}  // namespace workcell_builder
