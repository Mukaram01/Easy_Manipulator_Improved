#pragma once

#include <string>
#include <vector>

namespace workcell_builder
{

struct PlacedObject
{
  std::string name;
  std::string source_type;  // asset_stl | generated_primitive | external_stl_warning
  std::string mesh_path;
  double x{0.0}, y{0.0}, z{0.0};
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  double scale{1.0};
  std::string status;
};

std::string sanitize_object_name(const std::string & name);
bool validate_placed_object(const PlacedObject & object, std::string * warning);
std::string normalize_mesh_path_for_scene(const std::string & mesh_path);
std::string import_stl_to_asset_library(
  const std::string & stl_path,
  const std::string & repo_root,
  const std::string & managed_folder = "easy_manipulation_deployment/assets/environment/custom_meshes");
std::string serialize_placed_objects_to_environment_yaml(const std::vector<PlacedObject> & objects);
std::vector<PlacedObject> parse_placed_objects_from_environment_yaml(const std::string & content);
bool save_environment_layout(const std::string & output_path, const std::vector<PlacedObject> & objects);
std::vector<PlacedObject> load_environment_layout(const std::string & input_path);

class ObjectPlacementModel
{
public:
  void add_object(const PlacedObject & object);
  bool duplicate_object(const std::string & name);
  bool remove_object(const std::string & name);
  bool update_object_pose(
    const std::string & name, double x, double y, double z, double roll, double pitch, double yaw,
    std::string * warning = nullptr);
  std::vector<PlacedObject> objects() const;

private:
  PlacedObject * find_object_by_name(const std::string & name);
  std::vector<PlacedObject> objects_;
};

}  // namespace workcell_builder
