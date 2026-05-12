#include "workcell_camera_model.hpp"

#include <algorithm>
#include <sstream>

namespace workcell_builder
{
WorkcellCamera default_realsense_camera(const std::string & model)
{
  WorkcellCamera c;
  c.model = model;
  c.name = "realsense_" + model + "_1";
  c.xacro_path = "urdf/_" + model + ".urdf.xacro";
  return c;
}

CameraValidationResult validate_camera_attachment(const WorkcellCamera & camera, const std::vector<std::string> & scene_objects, const std::vector<std::string> & parent_links)
{
  CameraValidationResult out;
  if (camera.parent_object == "world") out.warnings.push_back("camera appears floating unless intentionally wall/world mounted");
  if (!camera.parent_object.empty() && camera.parent_object != "world" && std::find(scene_objects.begin(), scene_objects.end(), camera.parent_object) == scene_objects.end()) out.errors.push_back("camera parent object missing");
  if (!camera.parent_link.empty() && std::find(parent_links.begin(), parent_links.end(), camera.parent_link) == parent_links.end()) out.errors.push_back("camera parent link missing");
  out.ok = out.errors.empty();
  return out;
}

void serialize_cameras_to_yaml(YAML::Emitter * out, const std::vector<WorkcellCamera> & cameras)
{
  *out << YAML::Key << "cameras" << YAML::Value << YAML::BeginSeq;
  for (const auto & c : cameras) {
    *out << YAML::BeginMap;
    *out << YAML::Key << "name" << YAML::Value << c.name;
    *out << YAML::Key << "package" << YAML::Value << c.package_name;
    *out << YAML::Key << "model" << YAML::Value << c.model;
    *out << YAML::Key << "xacro" << YAML::Value << c.xacro_path;
    *out << YAML::Key << "parent_object" << YAML::Value << c.parent_object;
    *out << YAML::Key << "parent_link" << YAML::Value << c.parent_link;
    *out << YAML::Key << "pose" << YAML::Value << YAML::BeginMap;
    *out << YAML::Key << "xyz" << YAML::Value << YAML::Flow << YAML::BeginSeq << c.pose.x << c.pose.y << c.pose.z << YAML::EndSeq;
    *out << YAML::Key << "rpy" << YAML::Value << YAML::Flow << YAML::BeginSeq << c.pose.roll << c.pose.pitch << c.pose.yaw << YAML::EndSeq;
    *out << YAML::EndMap;
    *out << YAML::Key << "optical_frame" << YAML::Value << c.optical_frame;
    *out << YAML::Key << "depth_frame" << YAML::Value << c.depth_frame;
    *out << YAML::Key << "role" << YAML::Value << c.role;
    *out << YAML::Key << "runtime_driver" << YAML::Value << c.runtime_driver;
    *out << YAML::EndMap;
  }
  *out << YAML::EndSeq;
}

std::vector<WorkcellCamera> parse_cameras_from_yaml(const YAML::Node & root)
{
  std::vector<WorkcellCamera> out;
  if (!root["cameras"]) return out;
  for (const auto & node : root["cameras"]) {
    WorkcellCamera c;
    if (node["name"]) c.name = node["name"].as<std::string>();
    if (node["package"]) c.package_name = node["package"].as<std::string>();
    if (node["model"]) c.model = node["model"].as<std::string>();
    if (node["xacro"]) c.xacro_path = node["xacro"].as<std::string>();
    if (node["parent_object"]) c.parent_object = node["parent_object"].as<std::string>();
    if (node["parent_link"]) c.parent_link = node["parent_link"].as<std::string>();
    if (node["pose"] && node["pose"]["xyz"] && node["pose"]["xyz"].size() == 3) {
      c.pose.x = node["pose"]["xyz"][0].as<double>(); c.pose.y = node["pose"]["xyz"][1].as<double>(); c.pose.z = node["pose"]["xyz"][2].as<double>();
    }
    if (node["pose"] && node["pose"]["rpy"] && node["pose"]["rpy"].size() == 3) {
      c.pose.roll = node["pose"]["rpy"][0].as<double>(); c.pose.pitch = node["pose"]["rpy"][1].as<double>(); c.pose.yaw = node["pose"]["rpy"][2].as<double>();
    }
    if (node["optical_frame"]) c.optical_frame = node["optical_frame"].as<std::string>();
    if (node["depth_frame"]) c.depth_frame = node["depth_frame"].as<std::string>();
    if (node["role"]) c.role = node["role"].as<std::string>();
    if (node["runtime_driver"]) c.runtime_driver = node["runtime_driver"].as<std::string>();
    out.push_back(c);
  }
  return out;
}

std::string generate_camera_xacro_include(const WorkcellCamera & camera){ return "<xacro:include filename=\"package://" + camera.package_name + "/" + camera.xacro_path + "\"/>"; }
std::string generate_camera_fixed_joint(const WorkcellCamera & camera){ std::ostringstream ss; ss << "<joint name=\""<<camera.name<<"_mount_joint\" type=\"fixed\"><parent link=\""<<camera.parent_link<<"\"/><child link=\""<<camera.name<<"_link\"/><origin xyz=\""<<camera.pose.x<<" "<<camera.pose.y<<" "<<camera.pose.z<<"\" rpy=\""<<camera.pose.roll<<" "<<camera.pose.pitch<<" "<<camera.pose.yaw<<"\"/></joint>"; return ss.str(); }
}  // namespace workcell_builder
