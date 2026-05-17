#include "object_placement_yaml_io.hpp"

#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace workcell_builder
{
namespace
{
YAML::Node to_yaml_pose(const PlacedObject & o)
{
  YAML::Node pose;
  pose["xyz"].push_back(o.x); pose["xyz"].push_back(o.y); pose["xyz"].push_back(o.z);
  pose["rpy"].push_back(o.roll); pose["rpy"].push_back(o.pitch); pose["rpy"].push_back(o.yaw);
  return pose;
}
}

std::vector<PlacedObject> load_placed_objects_from_environment_yaml(const std::string & path, std::vector<std::string> * warnings)
{
  std::vector<PlacedObject> out;
  try {
    YAML::Node root = YAML::LoadFile(path);
    auto arr = root["placed_objects"];
    if (!arr || !arr.IsSequence()) return out;
    for (const auto & n : arr) {
      if (!n.IsMap() || !n["name"] || !n["mesh"]) { if (warnings) warnings->push_back("malformed placed_objects entry skipped"); continue; }
      PlacedObject o;
      o.name = sanitize_object_name(n["name"].as<std::string>(""));
      o.source_type = n["source"].as<std::string>("asset_stl");
      o.mesh_path = n["mesh"].as<std::string>("");
      if (o.mesh_path.rfind("/",0)==0 && o.mesh_path.rfind("package://",0)!=0 && warnings) warnings->push_back("external absolute mesh path: " + o.mesh_path);
      auto pose = n["pose"];
      if (pose && pose.IsMap()) {
        auto xyz = pose["xyz"]; auto rpy = pose["rpy"];
        if (xyz && xyz.IsSequence() && xyz.size()==3) { o.x=xyz[0].as<double>(0.0); o.y=xyz[1].as<double>(0.0); o.z=xyz[2].as<double>(0.0);} 
        if (rpy && rpy.IsSequence() && rpy.size()==3) { o.roll=rpy[0].as<double>(0.0); o.pitch=rpy[1].as<double>(0.0); o.yaw=rpy[2].as<double>(0.0);} 
      }
      out.push_back(o);
    }
  } catch (const std::exception & e) {
    if (warnings) warnings->push_back(std::string("placed_objects yaml parse warning: ") + e.what());
  }
  return out;
}

PlacedObjectYamlWriteResult save_placed_objects_to_environment_yaml(const std::string & path, const std::vector<PlacedObject> & objects)
{
  PlacedObjectYamlWriteResult r; r.path_written = path;
  try {
    YAML::Node root;
    if (std::filesystem::exists(path)) root = YAML::LoadFile(path);
    YAML::Node arr(YAML::NodeType::Sequence);
    for (const auto & o : objects) {
      YAML::Node n;
      n["name"] = o.name;
      n["source"] = o.source_type.empty() ? "asset_stl" : o.source_type;
      n["mesh"] = o.mesh_path;
      n["collision_mesh"] = o.mesh_path;
      n["pose"] = to_yaml_pose(o);
      n["scale"].push_back(1.0); n["scale"].push_back(1.0); n["scale"].push_back(1.0);
      n["parent_frame"] = "world";
      n["collision_enabled"] = true;
      arr.push_back(n);
    }
    root["placed_objects"] = arr;
    std::ofstream out(path);
    out << root;
    r.ok = out.good();
    r.objects_saved = objects.size();
  } catch (const std::exception & e) {
    r.warnings.push_back(std::string("failed to save placed objects: ") + e.what());
  }
  return r;
}

}  // namespace workcell_builder
