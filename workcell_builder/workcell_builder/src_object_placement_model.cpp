#include "object_placement_model.hpp"

#include <cmath>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace workcell_builder
{
namespace {
std::string trim_copy_local(const std::string & input)
{
  size_t b = 0, e = input.size();
  while (b < e && std::isspace(static_cast<unsigned char>(input[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(input[e - 1]))) --e;
  return input.substr(b, e - b);
}
}  // namespace

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

bool validate_camera_placement(const CameraPlacement & camera, std::string * warning)
{
  if (camera.name.empty()) {
    if (warning) *warning = "camera name is required";
    return false;
  }
  const bool finite_pose = std::isfinite(camera.x) && std::isfinite(camera.y) && std::isfinite(camera.z) &&
    std::isfinite(camera.roll) && std::isfinite(camera.pitch) && std::isfinite(camera.yaw);
  if (!finite_pose) {
    if (warning) *warning = "camera pose values must be finite";
    return false;
  }
  if (warning && (std::fabs(camera.x) > 100.0 || std::fabs(camera.y) > 100.0 || std::fabs(camera.z) > 100.0)) {
    *warning = "suspicious camera coordinate magnitude";
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

bool ObjectPlacementModel::update_object_pose(
  const std::string & name, double x, double y, double z, double roll, double pitch, double yaw,
  std::string * warning)
{
  for (auto & obj : objects_) {
    if (obj.name != name) {
      continue;
    }
    PlacedObject updated = obj;
    updated.x = x;
    updated.y = y;
    updated.z = z;
    updated.roll = roll;
    updated.pitch = pitch;
    updated.yaw = yaw;
    std::string validation_warning;
    if (!validate_placed_object(updated, &validation_warning)) {
      if (warning) *warning = validation_warning;
      return false;
    }
    obj = updated;
    obj.status = validation_warning;
    if (warning) *warning = validation_warning;
    return true;
  }
  if (warning) *warning = "object not found";
  return false;
}

std::vector<PlacedObject> ObjectPlacementModel::objects() const { return objects_; }

std::string serialize_placed_objects_to_environment_yaml(const std::vector<PlacedObject> & objects)
{
  std::ostringstream out;
  out << "placed_objects:\n";
  for (const auto & o : objects) {
    out << "  - name: " << o.name << "\n";
    out << "    source: " << o.source_type << "\n";
    out << "    mesh: " << o.mesh_path << "\n";
    out << "    collision_mesh: " << (o.collision_mesh.empty() ? o.mesh_path : o.collision_mesh) << "\n";
    out << "    collision_enabled: " << (o.collision_enabled ? "true" : "false") << "\n";
    out << "    parent_frame: " << (o.parent_frame.empty() ? "world" : o.parent_frame) << "\n";
    out << "    scale: [" << o.scale_x << ", " << o.scale_y << ", " << o.scale_z << "]\n";
    out << "    pose:\n";
    out << "      xyz: [" << o.x << ", " << o.y << ", " << o.z << "]\n";
    out << "      rpy: [" << o.roll << ", " << o.pitch << ", " << o.yaw << "]\n";
    if (!o.status.empty()) out << "    status: " << o.status << "\n";
  }
  return out.str();
}

std::vector<PlacedObject> parse_placed_objects_from_environment_yaml(const std::string & content)
{
  std::vector<PlacedObject> objs;
  std::istringstream in(content);
  std::string line;
  PlacedObject current;
  while (std::getline(in, line)) {
    if (line.find("  - name:") != std::string::npos) {
      if (!current.name.empty()) objs.push_back(current);
      current = PlacedObject{};
      current.name = sanitize_object_name(line.substr(line.find(":") + 1));
    } else if (line.find("source:") != std::string::npos) current.source_type = trim_copy_local(line.substr(line.find(":") + 1));
    else if (line.find("mesh:") != std::string::npos) current.mesh_path = trim_copy_local(line.substr(line.find(":") + 1));
    else if (line.find("collision_mesh:") != std::string::npos) current.collision_mesh = trim_copy_local(line.substr(line.find(":") + 1));
    else if (line.find("collision_enabled:") != std::string::npos) current.collision_enabled = trim_copy_local(line.substr(line.find(":") + 1)) != "false";
    else if (line.find("parent_frame:") != std::string::npos) current.parent_frame = trim_copy_local(line.substr(line.find(":") + 1));
    else if (line.find("scale:") != std::string::npos) {
      const size_t lb = line.find("[");
      const size_t rb = line.find("]");
      if (lb != std::string::npos && rb != std::string::npos && rb > lb + 1) {
        std::istringstream ss(line.substr(lb + 1, rb - lb - 1));
        std::string token;
        double v[3]{1.0, 1.0, 1.0};
        int i = 0;
        while (i < 3 && std::getline(ss, token, ',')) { std::istringstream value(trim_copy_local(token)); value >> v[i++]; }
        current.scale_x = v[0];
        current.scale_y = v[1];
        current.scale_z = v[2];
      }
    }
    else if (line.find("xyz:") != std::string::npos || line.find("rpy:") != std::string::npos) {
      const size_t lb = line.find("[");
      const size_t rb = line.find("]");
      if (lb == std::string::npos || rb == std::string::npos || rb <= lb + 1) continue;
      std::istringstream ss(line.substr(lb + 1, rb - lb - 1));
      std::string token;
      double v[3]{0.0, 0.0, 0.0};
      int i = 0;
      while (i < 3 && std::getline(ss, token, ',')) {
        std::istringstream value(trim_copy_local(token));
        value >> v[i++];
      }
      if (line.find("xyz:") != std::string::npos) {
        current.x = v[0];
        current.y = v[1];
        current.z = v[2];
      } else {
        current.roll = v[0];
        current.pitch = v[1];
        current.yaw = v[2];
      }
    }
  }
  if (!current.name.empty()) objs.push_back(current);
  return objs;
}

bool save_environment_layout(const std::string & output_path, const std::vector<PlacedObject> & objects)
{
  std::ofstream out(output_path);
  if (!out.good()) return false;
  out << serialize_placed_objects_to_environment_yaml(objects);
  return true;
}

std::vector<PlacedObject> load_environment_layout(const std::string & input_path)
{
  std::ifstream in(input_path);
  if (!in.good()) return {};
  std::stringstream buf;
  buf << in.rdbuf();
  return parse_placed_objects_from_environment_yaml(buf.str());
}

}  // namespace workcell_builder

// managed import default: easy_manipulation_deployment/assets/environment/custom_meshes
