#include "object_placement_yaml_io.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <unordered_set>
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


std::vector<TaskZone> load_task_zones_from_environment_yaml(const std::string & path, std::vector<std::string> * warnings)
{
  std::vector<TaskZone> out;
  std::unordered_set<std::string> seen_ids;
  try {
    YAML::Node root = YAML::LoadFile(path);
    auto arr = root["task_zones"];
    if (!arr) return out;
    if (!arr.IsSequence()) {
      if (warnings) warnings->push_back("task_zones exists but is not a sequence; ignoring");
      return out;
    }

    auto finite_or_warn = [&](double v, const std::string & id, const std::string & field) {
      if (!std::isfinite(v)) {
        if (warnings) warnings->push_back("task_zone rejected: " + id + " -> invalid non-finite " + field);
        return false;
      }
      return true;
    };

    for (const auto & n : arr) {
      if (!n.IsMap() || !n["id"]) {
        if (warnings) warnings->push_back("malformed task_zones entry skipped");
        continue;
      }

      TaskZone z;
      z.id = n["id"].as<std::string>("");
      if (z.id.empty()) {
        if (warnings) warnings->push_back("task_zone skipped: empty id");
        continue;
      }
      if (seen_ids.count(z.id) > 0) {
        if (warnings) warnings->push_back("duplicate task_zone id: " + z.id);
      }
      seen_ids.insert(z.id);

      z.type = n["type"].as<std::string>("");
      z.parent_frame = n["parent_frame"].as<std::string>("world");
      z.frame_id = n["frame_id"].as<std::string>("");
      z.enabled = n["enabled"].as<bool>(true);
      z.status = n["status"].as<std::string>("");

      auto pose = n["pose"];
      if (pose && pose.IsMap()) {
        auto xyz = pose["xyz"]; auto rpy = pose["rpy"];
        if (xyz && xyz.IsSequence() && xyz.size() == 3) {
          z.x = xyz[0].as<double>(0.0); z.y = xyz[1].as<double>(0.0); z.z = xyz[2].as<double>(0.0);
        }
        if (rpy && rpy.IsSequence() && rpy.size() == 3) {
          z.roll = rpy[0].as<double>(0.0); z.pitch = rpy[1].as<double>(0.0); z.yaw = rpy[2].as<double>(0.0);
        }
      }

      auto dims = n["dimensions"];
      if (dims && dims.IsMap()) {
        z.dim_x = dims["x"].as<double>(z.dim_x);
        z.dim_y = dims["y"].as<double>(z.dim_y);
        z.dim_z = dims["z"].as<double>(z.dim_z);
      }

      auto allowed = n["allowed_object_types"];
      if (allowed && allowed.IsSequence()) {
        for (const auto & a : allowed) {
          z.allowed_object_types.push_back(a.as<std::string>(""));
        }
      }

      if (!finite_or_warn(z.x, z.id, "pose.xyz") || !finite_or_warn(z.y, z.id, "pose.xyz") ||
        !finite_or_warn(z.z, z.id, "pose.xyz") || !finite_or_warn(z.roll, z.id, "pose.rpy") ||
        !finite_or_warn(z.pitch, z.id, "pose.rpy") || !finite_or_warn(z.yaw, z.id, "pose.rpy") ||
        !finite_or_warn(z.dim_x, z.id, "dimensions.x") || !finite_or_warn(z.dim_y, z.id, "dimensions.y") ||
        !finite_or_warn(z.dim_z, z.id, "dimensions.z"))
      {
        continue;
      }

      if (z.dim_x <= 0.0 || z.dim_y <= 0.0 || z.dim_z <= 0.0) {
        if (warnings) warnings->push_back("task_zone warning: " + z.id + " has non-positive dimension(s)");
      }
      if (std::fabs(z.x) > 100.0 || std::fabs(z.y) > 100.0 || std::fabs(z.z) > 100.0 ||
          std::fabs(z.dim_x) > 100.0 || std::fabs(z.dim_y) > 100.0 || std::fabs(z.dim_z) > 100.0) {
        if (warnings) warnings->push_back("task_zone warning: " + z.id + " has suspiciously large pose/dimension values");
      }

      out.push_back(z);
    }
  } catch (const std::exception & e) {
    if (warnings) warnings->push_back(std::string("task_zones yaml parse warning: ") + e.what());
  }
  return out;
}

PlacedObjectYamlWriteResult save_task_zones_to_environment_yaml(const std::string & path, const std::vector<TaskZone> & zones)
{
  PlacedObjectYamlWriteResult r; r.path_written = path;
  try {
    YAML::Node root;
    if (std::filesystem::exists(path)) root = YAML::LoadFile(path);

    YAML::Node arr(YAML::NodeType::Sequence);
    for (const auto & z : zones) {
      YAML::Node n;
      n["id"] = z.id;
      n["type"] = z.type;
      n["parent_frame"] = z.parent_frame.empty() ? "world" : z.parent_frame;
      n["pose"]["xyz"].push_back(z.x); n["pose"]["xyz"].push_back(z.y); n["pose"]["xyz"].push_back(z.z);
      n["pose"]["rpy"].push_back(z.roll); n["pose"]["rpy"].push_back(z.pitch); n["pose"]["rpy"].push_back(z.yaw);
      n["dimensions"]["x"] = z.dim_x;
      n["dimensions"]["y"] = z.dim_y;
      n["dimensions"]["z"] = z.dim_z;
      if (!z.frame_id.empty()) n["frame_id"] = z.frame_id;
      if (!z.allowed_object_types.empty()) {
        for (const auto & t : z.allowed_object_types) n["allowed_object_types"].push_back(t);
      }
      n["enabled"] = z.enabled;
      if (!z.status.empty()) n["status"] = z.status;
      arr.push_back(n);
    }

    root["task_zones"] = arr;
    std::ofstream out(path);
    out << root;
    r.ok = out.good();
    r.objects_saved = zones.size();
  } catch (const std::exception & e) {
    r.warnings.push_back(std::string("failed to save task zones: ") + e.what());
  }
  return r;
}

RobotToolPoseConfig load_robot_tool_pose_from_environment_yaml(const std::string & path, std::vector<std::string> * warnings)
{
  RobotToolPoseConfig out;
  try {
    YAML::Node root = YAML::LoadFile(path);
    const auto cfg = root["robot_tool_pose"];
    if (!cfg || !cfg.IsMap()) return out;
    const auto robot = cfg["robot_base_pose"];
    const auto tool = cfg["tool_attach_pose"];
    const auto read_pose = [](const YAML::Node & n, double xyz[3], double rpy[3]) {
      if (!n || !n.IsMap()) return;
      const auto xyz_node = n["xyz"];
      const auto rpy_node = n["rpy"];
      if (xyz_node && xyz_node.IsSequence() && xyz_node.size() == 3) {
        xyz[0] = xyz_node[0].as<double>(0.0); xyz[1] = xyz_node[1].as<double>(0.0); xyz[2] = xyz_node[2].as<double>(0.0);
      }
      if (rpy_node && rpy_node.IsSequence() && rpy_node.size() == 3) {
        rpy[0] = rpy_node[0].as<double>(0.0); rpy[1] = rpy_node[1].as<double>(0.0); rpy[2] = rpy_node[2].as<double>(0.0);
      }
    };
    read_pose(robot, out.robot_base_xyz, out.robot_base_rpy);
    read_pose(tool, out.tool_attach_xyz, out.tool_attach_rpy);
    out.tool_link_id = cfg["tool_link_id"].as<std::string>("");
    out.tool_joint_id = cfg["tool_joint_id"].as<std::string>("");
  } catch (const std::exception & e) {
    if (warnings) warnings->push_back(std::string("robot_tool_pose yaml parse warning: ") + e.what());
  }
  return out;
}

PlacedObjectYamlWriteResult save_robot_tool_pose_to_environment_yaml(const std::string & path, const RobotToolPoseConfig & config)
{
  PlacedObjectYamlWriteResult r; r.path_written = path;
  try {
    YAML::Node root;
    if (std::filesystem::exists(path)) root = YAML::LoadFile(path);
    YAML::Node cfg;
    cfg["robot_base_pose"]["xyz"].push_back(config.robot_base_xyz[0]);
    cfg["robot_base_pose"]["xyz"].push_back(config.robot_base_xyz[1]);
    cfg["robot_base_pose"]["xyz"].push_back(config.robot_base_xyz[2]);
    cfg["robot_base_pose"]["rpy"].push_back(config.robot_base_rpy[0]);
    cfg["robot_base_pose"]["rpy"].push_back(config.robot_base_rpy[1]);
    cfg["robot_base_pose"]["rpy"].push_back(config.robot_base_rpy[2]);
    cfg["tool_attach_pose"]["xyz"].push_back(config.tool_attach_xyz[0]);
    cfg["tool_attach_pose"]["xyz"].push_back(config.tool_attach_xyz[1]);
    cfg["tool_attach_pose"]["xyz"].push_back(config.tool_attach_xyz[2]);
    cfg["tool_attach_pose"]["rpy"].push_back(config.tool_attach_rpy[0]);
    cfg["tool_attach_pose"]["rpy"].push_back(config.tool_attach_rpy[1]);
    cfg["tool_attach_pose"]["rpy"].push_back(config.tool_attach_rpy[2]);
    cfg["tool_link_id"] = config.tool_link_id;
    cfg["tool_joint_id"] = config.tool_joint_id;
    root["robot_tool_pose"] = cfg;
    std::ofstream out(path);
    out << root;
    r.ok = out.good();
    r.objects_saved = 1;
  } catch (const std::exception & e) {
    r.warnings.push_back(std::string("failed to save robot/tool pose: ") + e.what());
  }
  return r;
}

}  // namespace workcell_builder
