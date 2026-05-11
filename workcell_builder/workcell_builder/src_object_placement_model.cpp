#include "object_placement_model.hpp"

#include <cmath>
#include <algorithm>
#include <filesystem>

namespace workcell_builder
{

std::string sanitize_object_name(const std::string & name)
{
  std::string out;
  out.reserve(name.size());
  for (const char c : name) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') {
      out.push_back(c);
    } else {
      out.push_back('_');
    }
  }
  return out.empty() ? "object" : out;
}

bool validate_placed_object(const PlacedObject & object, std::string * warning)
{
  if (object.name.empty()) {
    if (warning) *warning = "object name is required";
    return false;
  }
  const bool finite_pose = std::isfinite(object.x) && std::isfinite(object.y) && std::isfinite(object.z) &&
    std::isfinite(object.roll) && std::isfinite(object.pitch) && std::isfinite(object.yaw);
  if (!finite_pose) {
    if (warning) *warning = "pose values must be finite";
    return false;
  }
  if (warning && (std::fabs(object.x) > 100.0 || std::fabs(object.y) > 100.0 || std::fabs(object.z) > 100.0)) {
    *warning = "suspicious object coordinate magnitude";
  }
  return true;
}

std::string normalize_mesh_path_for_scene(const std::string & mesh_path)
{
  if (mesh_path.rfind("package://", 0) == 0 || mesh_path.rfind("meshes/", 0) == 0) {
    return mesh_path;
  }
  std::filesystem::path p(mesh_path);
  return p.lexically_normal().generic_string();
}

std::string import_stl_to_asset_library(const std::string & stl_path, const std::string & repo_root, const std::string & managed_folder)
{
  std::filesystem::path source(stl_path);
  std::filesystem::path target_dir = std::filesystem::path(repo_root) / managed_folder;
  std::filesystem::create_directories(target_dir);
  const std::string safe_name = sanitize_object_name(source.stem().string()) + ".stl";
  std::filesystem::path target = target_dir / safe_name;
  int n = 1;
  while (std::filesystem::exists(target)) {
    target = target_dir / (sanitize_object_name(source.stem().string()) + "_" + std::to_string(n++) + ".stl");
  }
  std::filesystem::copy_file(source, target, std::filesystem::copy_options::overwrite_existing);
  return (std::filesystem::path(managed_folder) / target.filename()).generic_string();
}

void ObjectPlacementModel::add_object(const PlacedObject & object)
{
  auto copy = object;
  copy.name = sanitize_object_name(copy.name);
  if (copy.mesh_path.empty()) {
    copy.status = copy.status.empty() ? "missing mesh path" : copy.status;
  }
  objects_.push_back(copy);
}

bool ObjectPlacementModel::duplicate_object(const std::string & name)
{
  for (const auto & obj : objects_) {
    if (obj.name == name) {
      auto duplicate = obj;
      int suffix = 1;
      std::string candidate;
      do {
        candidate = sanitize_object_name(name + "_copy_" + std::to_string(suffix++));
      } while (std::any_of(objects_.begin(), objects_.end(), [&candidate](const PlacedObject & o) { return o.name == candidate; }));
      duplicate.name = candidate;
      objects_.push_back(duplicate);
      return true;
    }
  }
  return false;
}

bool ObjectPlacementModel::remove_object(const std::string & name)
{
  for (auto it = objects_.begin(); it != objects_.end(); ++it) {
    if (it->name == name) {
      objects_.erase(it);
      return true;
    }
  }
  return false;
}

std::vector<PlacedObject> ObjectPlacementModel::objects() const { return objects_; }

}  // namespace workcell_builder

// managed import default: easy_manipulation_deployment/assets/environment/custom_meshes
