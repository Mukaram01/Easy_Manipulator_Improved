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
      o.collision_enabled = n["collision_enabled"].as<bool>(true);
      o.parent_frame = n["parent_frame"].as<std::string>("world");
      o.collision_mesh = n["collision_mesh"].as<std::string>(o.mesh_path);
      auto scale = n["scale"];
      if (scale && scale.IsSequence() && scale.size() == 3) {
        o.scale_x = scale[0].as<double>(1.0);
        o.scale_y = scale[1].as<double>(1.0);
        o.scale_z = scale[2].as<double>(1.0);
      }
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
      n["collision_mesh"] = o.collision_mesh.empty() ? o.mesh_path : o.collision_mesh;
      n["pose"] = to_yaml_pose(o);
      n["scale"].push_back(o.scale_x); n["scale"].push_back(o.scale_y); n["scale"].push_back(o.scale_z);
      n["parent_frame"] = o.parent_frame.empty() ? "world" : o.parent_frame;
      n["collision_enabled"] = o.collision_enabled;
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

std::vector<CameraPlacement> load_camera_placements_from_environment_yaml(const std::string & path, std::vector<std::string> * warnings)
{
  std::vector<CameraPlacement> out;
  try {
    YAML::Node root = YAML::LoadFile(path);
    auto arr = root["camera_placements"];
    if (!arr || !arr.IsSequence()) return out;
    for (const auto & n : arr) {
      if (!n.IsMap() || !n["name"]) { if (warnings) warnings->push_back("malformed camera_placements entry skipped"); continue; }
      CameraPlacement c;
      c.name = sanitize_object_name(n["name"].as<std::string>("camera_01"));
      c.type = n["type"].as<std::string>("realsense_d435i");
      c.source = n["source"].as<std::string>("camera_asset");
      c.parent_frame = n["parent_frame"].as<std::string>("world");
      c.enabled = n["enabled"].as<bool>(true);
      auto pose=n["pose"]; if (pose&&pose.IsMap()) { auto xyz=pose["xyz"]; auto rpy=pose["rpy"]; if(xyz&&xyz.IsSequence()&&xyz.size()==3){c.x=xyz[0].as<double>(0.0); c.y=xyz[1].as<double>(0.0); c.z=xyz[2].as<double>(0.0);} if(rpy&&rpy.IsSequence()&&rpy.size()==3){c.roll=rpy[0].as<double>(0.0); c.pitch=rpy[1].as<double>(0.0); c.yaw=rpy[2].as<double>(0.0);} }
      auto frames=n["frames"]; if(frames&&frames.IsMap()){ c.link_frame=frames["link"].as<std::string>(c.name+"_link"); c.optical_frame=frames["optical_frame"].as<std::string>(c.name+"_color_optical_frame"); c.depth_frame=frames["depth_frame"].as<std::string>(c.name+"_depth_optical_frame"); }
      auto topics=n["topics"]; if(topics&&topics.IsMap()){ c.color_topic=topics["color"].as<std::string>(c.color_topic); c.depth_topic=topics["depth"].as<std::string>(c.depth_topic); c.pointcloud_topic=topics["pointcloud"].as<std::string>(c.pointcloud_topic); c.camera_info_topic=topics["camera_info"].as<std::string>(c.camera_info_topic); }
      auto fr=n["frustum"]; if(fr&&fr.IsMap()){ c.horizontal_fov_deg=fr["horizontal_fov_deg"].as<double>(c.horizontal_fov_deg); c.vertical_fov_deg=fr["vertical_fov_deg"].as<double>(c.vertical_fov_deg); c.near_m=fr["near_m"].as<double>(c.near_m); c.far_m=fr["far_m"].as<double>(c.far_m);}      
      std::string warn; if(!validate_camera_placement(c,&warn)){ if(warnings) warnings->push_back("camera entry rejected: "+c.name+" -> "+warn); continue; }
      if(warnings && !warn.empty()) warnings->push_back("camera warning: "+c.name+" -> "+warn);
      out.push_back(c);
    }
  } catch (const std::exception & e) {
    if (warnings) warnings->push_back(std::string("camera_placements yaml parse warning: ") + e.what());
  }
  return out;
}

PlacedObjectYamlWriteResult save_camera_placements_to_environment_yaml(const std::string & path, const std::vector<CameraPlacement> & cameras)
{
  PlacedObjectYamlWriteResult r; r.path_written=path;
  try {
    YAML::Node root; if (std::filesystem::exists(path)) root = YAML::LoadFile(path);
    YAML::Node arr(YAML::NodeType::Sequence);
    for (const auto & c : cameras) {
      std::string warn; if(!validate_camera_placement(c,&warn)){ r.warnings.push_back("camera entry skipped: "+c.name+" -> "+warn); continue; }
      YAML::Node n; n["name"]=c.name; n["type"]=c.type; n["source"]=c.source; n["parent_frame"]=c.parent_frame; n["enabled"]=c.enabled;
      n["pose"]["xyz"].push_back(c.x); n["pose"]["xyz"].push_back(c.y); n["pose"]["xyz"].push_back(c.z);
      n["pose"]["rpy"].push_back(c.roll); n["pose"]["rpy"].push_back(c.pitch); n["pose"]["rpy"].push_back(c.yaw);
      n["frames"]["link"]=c.link_frame; n["frames"]["optical_frame"]=c.optical_frame; n["frames"]["depth_frame"]=c.depth_frame;
      n["topics"]["color"]=c.color_topic; n["topics"]["depth"]=c.depth_topic; n["topics"]["pointcloud"]=c.pointcloud_topic; n["topics"]["camera_info"]=c.camera_info_topic;
      n["frustum"]["horizontal_fov_deg"]=c.horizontal_fov_deg; n["frustum"]["vertical_fov_deg"]=c.vertical_fov_deg; n["frustum"]["near_m"]=c.near_m; n["frustum"]["far_m"]=c.far_m;
      arr.push_back(n);
    }
    root["camera_placements"]=arr;
    std::ofstream out(path); out<<root; r.ok=out.good(); r.objects_saved=cameras.size();
  } catch (const std::exception & e) { r.warnings.push_back(std::string("failed to save camera placements: ")+e.what()); }
  return r;
}

}  // namespace workcell_builder
