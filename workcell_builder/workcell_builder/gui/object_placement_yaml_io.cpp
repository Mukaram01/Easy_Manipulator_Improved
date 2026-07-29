#include "object_placement_yaml_io.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <unordered_set>
#include <yaml-cpp/yaml.h>

namespace workcell_builder
{
namespace
{
constexpr double kSuspiciousPositionMagnitudeMeters = 100.0;

const TaskZone * find_task_zone_by_id(const std::vector<TaskZone> & zones, const std::string & id)
{
  auto it = std::find_if(zones.begin(), zones.end(), [&](const TaskZone & z){ return z.id == id; });
  return it == zones.end() ? nullptr : &*it;
}

YAML::Node to_yaml_pose(const PlacedObject & o)
{
  YAML::Node pose;
  pose["xyz"].push_back(o.x); pose["xyz"].push_back(o.y); pose["xyz"].push_back(o.z);
  pose["rpy"].push_back(o.roll); pose["rpy"].push_back(o.pitch); pose["rpy"].push_back(o.yaw);
  return pose;
}

void append_pose_to_yaml(
  YAML::Node * node, double x, double y, double z, double roll, double pitch, double yaw)
{
  (*node)["pose"]["xyz"].push_back(x);
  (*node)["pose"]["xyz"].push_back(y);
  (*node)["pose"]["xyz"].push_back(z);
  (*node)["pose"]["rpy"].push_back(roll);
  (*node)["pose"]["rpy"].push_back(pitch);
  (*node)["pose"]["rpy"].push_back(yaw);
}

void warn_if_suspicious_pose(
  const std::string & block_name, double x, double y, double z, std::vector<std::string> * warnings)
{
  if (!warnings) {
    return;
  }
  if (std::fabs(x) > kSuspiciousPositionMagnitudeMeters ||
    std::fabs(y) > kSuspiciousPositionMagnitudeMeters ||
    std::fabs(z) > kSuspiciousPositionMagnitudeMeters)
  {
    warnings->push_back(block_name + " warning: suspiciously large xyz magnitude");
  }
}

bool check_finite(const std::string & context, double value, std::vector<std::string> * warnings)
{
  if (std::isfinite(value)) {
    return true;
  }
  if (warnings) warnings->push_back(context + " has non-finite numeric value");
  return false;
}
}

std::vector<TaskAreaDestination> discover_task_area_destinations(
  const std::string & path, std::vector<std::string> * warnings)
{
  std::vector<TaskAreaDestination> result;
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node assets = root["environment"]["assets"];
    if (!assets || !assets.IsSequence()) return result;
    auto normalized = [](const YAML::Node & node, const char * key) {
      std::string value = node[key].as<std::string>("");
      std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      return value;
    };
    const auto contains_any = [](const std::string & value, const std::vector<std::string> & tokens) {
      return std::any_of(tokens.begin(), tokens.end(), [&value](const std::string & token) {
        return value.find(token) != std::string::npos;
      });
    };
    const std::vector<std::string> destination_tokens{
      "bin", "tote", "destination_container", "destination container", "container"};
    const std::vector<std::string> excluded_tokens{
      "overlay", "helper", "diagnostic", "place_zone", "pick_zone", "robot", "tool",
      "gripper", "camera", "generated_urdf", "generated urdf"};
    const std::vector<std::string> excluded_category_tokens{
      "overlay", "helper", "diagnostic", "robot", "tool", "gripper", "camera",
      "generated_urdf", "generated urdf"};
    for (const auto & asset : assets) {
      const std::string id = asset["id"].as<std::string>("");
      const std::string type = normalized(asset, "type");
      const std::string role = normalized(asset, "role");
      const std::string category = normalized(asset, "category");
      const std::string source_layer = normalized(asset, "source_layer");
      const std::string active_visual_source = normalized(asset, "active_visual_source");
      if (id.empty() || contains_any(source_layer, excluded_tokens) ||
        contains_any(active_visual_source, excluded_tokens)) continue;
      const bool semantic_destination = contains_any(type, destination_tokens) ||
        contains_any(role, destination_tokens) || contains_any(category, destination_tokens);
      const bool helper_identity = contains_any(type, excluded_tokens) ||
        contains_any(role, excluded_tokens) || contains_any(category, excluded_category_tokens);
      if (!semantic_destination || helper_identity) continue;
      result.push_back({id, asset["display_name"].as<std::string>(asset["name"].as<std::string>(id))});
    }
    std::sort(result.begin(), result.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.display_name == rhs.display_name ? lhs.id < rhs.id : lhs.display_name < rhs.display_name;
    });
  } catch (const std::exception & e) {
    if (warnings) warnings->push_back(std::string("destination discovery failed: ") + e.what());
  }
  return result;
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
      auto pose_xyz_alias = n["pose_xyz"];
      if (pose_xyz_alias && pose_xyz_alias.IsSequence() && pose_xyz_alias.size() == 3) {
        o.x = pose_xyz_alias[0].as<double>(0.0); o.y = pose_xyz_alias[1].as<double>(0.0); o.z = pose_xyz_alias[2].as<double>(0.0);
      }
      auto pose_rpy_alias = n["pose_rpy"];
      if (pose_rpy_alias && pose_rpy_alias.IsSequence() && pose_rpy_alias.size() == 3) {
        o.roll = pose_rpy_alias[0].as<double>(0.0); o.pitch = pose_rpy_alias[1].as<double>(0.0); o.yaw = pose_rpy_alias[2].as<double>(0.0);
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
      z.role = n["role"].as<std::string>(z.type);
      z.shape = n["shape"].as<std::string>("box");
      z.parent_frame = n["frame"].as<std::string>(n["parent_frame"].as<std::string>("world"));
      z.frame_id = n["frame_id"].as<std::string>("");
      z.camera_id = n["camera_id"].as<std::string>(n["linked_camera"].as<std::string>(""));
      z.robot_id = n["robot_id"].as<std::string>(n["linked_robot"].as<std::string>(""));
      z.support_surface_ref = n["support_surface_ref"].as<std::string>("");
      z.object_ref = n["object_ref"].as<std::string>("");
      z.target_ref = n["target_ref"].as<std::string>("");
      z.enabled = n["enabled"].as<bool>(true);
      z.visible = n["visible"].as<bool>(true);
      z.status = n["status"].as<std::string>("");

      auto pose_xyz_alias = n["pose_xyz"];
      if (pose_xyz_alias && pose_xyz_alias.IsSequence() && pose_xyz_alias.size() == 3) {
        z.x = pose_xyz_alias[0].as<double>(0.0); z.y = pose_xyz_alias[1].as<double>(0.0); z.z = pose_xyz_alias[2].as<double>(0.0);
      }
      auto pose_rpy_alias = n["pose_rpy"];
      if (pose_rpy_alias && pose_rpy_alias.IsSequence() && pose_rpy_alias.size() == 3) {
        z.roll = pose_rpy_alias[0].as<double>(0.0); z.pitch = pose_rpy_alias[1].as<double>(0.0); z.yaw = pose_rpy_alias[2].as<double>(0.0);
      }
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
      } else if (dims && dims.IsSequence() && dims.size() == 3) {
        z.dim_x = dims[0].as<double>(z.dim_x);
        z.dim_y = dims[1].as<double>(z.dim_y);
        z.dim_z = dims[2].as<double>(z.dim_z);
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

std::vector<TaskZoneLink> load_task_zone_links_from_environment_yaml(
  const std::string & path, const std::vector<TaskZone> & zones, std::vector<std::string> * warnings)
{
  std::vector<TaskZoneLink> out;
  try {
    YAML::Node root = YAML::LoadFile(path);
    auto arr = root["task_zone_links"];
    if (!arr) return out;
    if (!arr.IsSequence()) { if (warnings) warnings->push_back("task_zone_links exists but is not a sequence; ignoring"); return out; }
    for (const auto & n : arr) {
      if (!n.IsMap()) { if (warnings) warnings->push_back("malformed task_zone_links entry skipped"); continue; }
      TaskZoneLink link;
      link.id = n["id"].as<std::string>("");
      link.type = n["type"].as<std::string>("");
      link.source_zone_id = n["source_zone_id"].as<std::string>("");
      link.target_zone_id = n["target_zone_id"].as<std::string>("");
      link.enabled = n["enabled"].as<bool>(true);
      link.status = n["status"].as<std::string>("");
      std::string warning;
      link.resolved = validate_observation_to_pick_link(link, zones, out, &warning);
      if (!link.resolved && warnings) warnings->push_back(warning);
      out.push_back(link);
    }
  } catch (const std::exception & e) {
    if (warnings) warnings->push_back(std::string("task_zone_links yaml parse warning: ") + e.what());
  }
  return out;
}

PlacedObjectYamlWriteResult save_task_zone_links_to_environment_yaml(
  const std::string & path, const std::vector<TaskZoneLink> & links)
{
  PlacedObjectYamlWriteResult r; r.path_written = path;
  try {
    YAML::Node root;
    if (std::filesystem::exists(path)) root = YAML::LoadFile(path);
    YAML::Node arr(YAML::NodeType::Sequence);
    for (const auto & l : links) {
      YAML::Node n;
      n["id"] = l.id; n["type"] = l.type;
      n["source_zone_id"] = l.source_zone_id; n["target_zone_id"] = l.target_zone_id;
      n["enabled"] = l.enabled;
      if (!l.status.empty()) n["status"] = l.status;
      arr.push_back(n);
    }
    root["task_zone_links"] = arr;
    std::ofstream out(path); out << root;
    r.ok = out.good(); r.objects_saved = links.size();
  } catch (const std::exception & e) {
    r.warnings.push_back(std::string("failed to save task zone links: ") + e.what());
  }
  return r;
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
      n["role"] = z.role.empty() ? z.type : z.role;
      n["frame"] = z.parent_frame.empty() ? "world" : z.parent_frame;
      n["shape"] = z.shape.empty() ? "box" : z.shape;
      n["pose_xyz"].push_back(z.x); n["pose_xyz"].push_back(z.y); n["pose_xyz"].push_back(z.z);
      n["pose_rpy"].push_back(z.roll); n["pose_rpy"].push_back(z.pitch); n["pose_rpy"].push_back(z.yaw);
      n["dimensions"].push_back(z.dim_x);
      n["dimensions"].push_back(z.dim_y);
      n["dimensions"].push_back(z.dim_z);
      if (!z.frame_id.empty()) n["frame_id"] = z.frame_id;
      if (!z.camera_id.empty()) n["camera_id"] = z.camera_id;
      if (!z.robot_id.empty()) n["robot_id"] = z.robot_id;
      if (!z.support_surface_ref.empty()) n["support_surface_ref"] = z.support_surface_ref;
      if (!z.object_ref.empty()) n["object_ref"] = z.object_ref;
      if (!z.target_ref.empty()) n["target_ref"] = z.target_ref;
      if (!z.allowed_object_types.empty()) {
        for (const auto & t : z.allowed_object_types) n["allowed_object_types"].push_back(t);
      }
      n["enabled"] = z.enabled;
      n["visible"] = z.visible;
      if (!z.status.empty()) n["status"] = z.status;
      arr.push_back(n);
    }

    root["task_zones"] = arr;
    for (const auto & z : zones) {
      const bool is_pick = z.role == "pick" || z.type == "pick" || z.type == "pick_zone";
      const bool is_place = z.role == "place" || z.type == "place" || z.type == "place_zone";
      if (is_pick) {
        root["task"]["pick"]["source_ref"] = z.id;
        if (!z.object_ref.empty()) root["task"]["pick"]["object_ref"] = z.object_ref;
      }
      if (is_place) {
        // task.place.target_ref selects the place zone; zone.target_ref selects its physical bin.
        // Preserve an existing active-zone selection when only the bin association changes.
        if (!root["task"]["place"]["target_ref"]) root["task"]["place"]["target_ref"] = z.id;
        if (!root["task"]["place"]["intent_target_ref"]) root["task"]["place"]["intent_target_ref"] = z.id;
      }
    }
    std::ofstream out(path);
    out << root;
    r.ok = out.good();
    r.objects_saved = zones.size();
  } catch (const std::exception & e) {
    r.warnings.push_back(std::string("failed to save task zones: ") + e.what());
  }
  return r;
}


bool is_camera_observation_zone(const TaskZone & zone)
{
  return zone.type == "camera_observation" || zone.role == "camera_observation";
}

bool is_pick_task_zone(const TaskZone & zone)
{
  return zone.type == "pick" || zone.type == "pick_zone" || zone.role == "pick";
}

bool validate_observation_to_pick_link(
  const TaskZoneLink & link, const std::vector<TaskZone> & zones,
  const std::vector<TaskZoneLink> & existing_links, std::string * warning)
{
  if (link.type != "observation_to_pick") { if (warning) *warning = "unsupported task zone link type"; return false; }
  if (link.source_zone_id.empty() || link.target_zone_id.empty()) { if (warning) *warning = "Observation-to-Pick link source or target is unavailable"; return false; }
  if (link.source_zone_id == link.target_zone_id) { if (warning) *warning = "Observation-to-Pick link cannot target itself"; return false; }
  const TaskZone * source = find_task_zone_by_id(zones, link.source_zone_id);
  if (!source) { if (warning) *warning = "Observation-to-Pick link source is unavailable"; return false; }
  const TaskZone * target = find_task_zone_by_id(zones, link.target_zone_id);
  if (!target) { if (warning) *warning = "Observation-to-Pick link target is unavailable"; return false; }
  if (!is_camera_observation_zone(*source)) { if (warning) *warning = "Observation-to-Pick link source must be a Camera Observation Zone"; return false; }
  if (!is_pick_task_zone(*target)) { if (warning) *warning = "Observation-to-Pick link target must be a Pick Zone"; return false; }
  int duplicates = 0;
  for (const auto & existing : existing_links) {
    if (existing.enabled && existing.type == link.type && existing.source_zone_id == link.source_zone_id &&
      existing.target_zone_id == link.target_zone_id) ++duplicates;
  }
  if (duplicates > 1) { if (warning) *warning = "duplicate active Observation-to-Pick link"; return false; }
  return true;
}

std::vector<TaskZoneLink> set_observation_pick_link(
  std::vector<TaskZoneLink> links, const std::string & source_zone_id, const std::string & target_zone_id,
  const std::vector<TaskZone> & zones, std::string * warning)
{
  links.erase(std::remove_if(links.begin(), links.end(), [&](const TaskZoneLink & l){
    return l.type == "observation_to_pick" && l.source_zone_id == source_zone_id;
  }), links.end());
  if (target_zone_id.empty()) return links;
  TaskZoneLink link;
  link.id = "observation_to_pick_" + std::to_string(links.size() + 1);
  link.type = "observation_to_pick";
  link.source_zone_id = source_zone_id;
  link.target_zone_id = target_zone_id;
  link.enabled = true;
  link.status = "Linked — tracking and timing not yet configured";
  links.push_back(link);
  if (!validate_observation_to_pick_link(link, zones, links, warning)) links.pop_back();
  return links;
}

TaskZoneDefaults default_task_zone_dimensions() { return TaskZoneDefaults{}; }
TaskZoneDefaults default_pick_zone_dimensions() { return default_task_zone_dimensions(); }

bool validate_task_zone_dimensions(const TaskZone & zone, std::string * warning)
{
  const bool ok = std::isfinite(zone.dim_x) && std::isfinite(zone.dim_y) && std::isfinite(zone.dim_z) &&
    zone.dim_x > 0.0 && zone.dim_y > 0.0 && zone.dim_z > 0.0 &&
    zone.dim_x <= 100.0 && zone.dim_y <= 100.0 && zone.dim_z <= 100.0;
  if (!ok && warning) *warning = (zone.role == "place" || zone.type == "place") ? "Place Zone dimensions must be positive" : "Pick Zone dimensions must be positive";
  return ok;
}

static bool can_create_zone_for_robots(const std::vector<std::string> & robot_ids, const std::string & label, std::string * message)
{
  if (robot_ids.empty()) {
    if (message) *message = "Add or select a robot before creating a " + label;
    return false;
  }
  if (message) *message = robot_ids.size() == 1 ? robot_ids.front() : "Choose a robot for the " + label;
  return true;
}

bool can_create_pick_zone_for_robots(const std::vector<std::string> & robot_ids, std::string * message)
{
  return can_create_zone_for_robots(robot_ids, "Pick Zone", message);
}

bool can_create_place_zone_for_robots(const std::vector<std::string> & robot_ids, std::string * message)
{
  return can_create_zone_for_robots(robot_ids, "Place Zone", message);
}

ObservationZoneSuggestion suggest_robot_task_zone(
  const std::string & zone_type, const std::string & robot_id, const std::vector<TaskZone> & existing_zones,
  double world_x, double world_y, double surface_z, double yaw)
{
  ObservationZoneSuggestion out;
  const bool is_place = zone_type == "place";
  const std::string label = is_place ? "Place Zone" : "Pick Zone";
  if (robot_id.empty()) { out.messages.push_back(label + " robot is unavailable"); return out; }
  TaskZone z;
  for (int i = 1;; ++i) {
    z.id = zone_type + "_zone_" + std::to_string(i);
    if (std::none_of(existing_zones.begin(), existing_zones.end(), [&](const TaskZone & e){ return e.id == z.id; })) break;
  }
  const auto d = default_task_zone_dimensions();
  z.type = zone_type; z.role = zone_type; z.shape = "box"; z.parent_frame = "world"; z.robot_id = robot_id;
  z.x = world_x; z.y = world_y; z.z = surface_z; z.yaw = yaw; z.dim_x = d.width; z.dim_y = d.depth; z.dim_z = d.height;
  z.status = "Robot association: " + robot_id + " — reachability not yet validated";
  std::string warning;
  if (!validate_task_zone_dimensions(z, &warning)) { out.messages.push_back(warning); return out; }
  out.ok = true; out.zone = z; out.messages.push_back("Created " + label + " for " + robot_id);
  return out;
}

ObservationZoneSuggestion suggest_robot_pick_zone(
  const std::string & robot_id, const std::vector<TaskZone> & existing_zones,
  double world_x, double world_y, double surface_z, double yaw)
{
  return suggest_robot_task_zone("pick", robot_id, existing_zones, world_x, world_y, surface_z, yaw);
}

ObservationZoneSuggestion suggest_robot_place_zone(
  const std::string & robot_id, const std::vector<TaskZone> & existing_zones,
  double world_x, double world_y, double surface_z, double yaw)
{
  return suggest_robot_task_zone("place", robot_id, existing_zones, world_x, world_y, surface_z, yaw);
}

bool can_create_observation_zone_for_camera(
  const CameraPlacement * selected_camera, bool editable_scene, bool scene_writable)
{
  std::string warning;
  return editable_scene && scene_writable && selected_camera &&
    selected_camera->enabled && validate_camera_placement(*selected_camera, &warning);
}

ObservationZoneSuggestion suggest_camera_observation_zone(
  const CameraPlacement & camera, const std::vector<TaskZone> & existing_zones, double work_surface_z)
{
  ObservationZoneSuggestion out;
  if (!can_create_observation_zone_for_camera(&camera, true, true)) {
    out.messages.push_back("Observation zone camera is unavailable");
    return out;
  }
  const double cr = std::cos(camera.roll), sr = std::sin(camera.roll);
  const double cp = std::cos(camera.pitch), sp = std::sin(camera.pitch);
  const double cy = std::cos(camera.yaw), sy = std::sin(camera.yaw);
  const double dx = -(cy * sp * cr + sy * sr);
  const double dy = -(sy * sp * cr - cy * sr);
  const double dz = -(cp * cr);
  if (std::fabs(dz) < 1e-6 || ((work_surface_z - camera.z) / dz) <= 0.0) {
    out.messages.push_back("Camera view does not intersect the work surface");
    return out;
  }
  const double t = (work_surface_z - camera.z) / dz;
  double width = 0.6, depth = 0.4;
  if (camera.horizontal_fov_deg > 1.0 && camera.vertical_fov_deg > 1.0) {
    width = std::clamp(2.0 * t * std::tan(camera.horizontal_fov_deg * 3.14159265358979323846 / 360.0), 0.05, 5.0);
    depth = std::clamp(2.0 * t * std::tan(camera.vertical_fov_deg * 3.14159265358979323846 / 360.0), 0.05, 5.0);
  } else {
    out.messages.push_back("Observation zone uses default dimensions");
  }
  TaskZone z;
  z.id = "camera_observation_1";
  for (int i = 1;; ++i) {
    z.id = "camera_observation_" + std::to_string(i);
    if (std::none_of(existing_zones.begin(), existing_zones.end(), [&](const TaskZone & e){ return e.id == z.id; })) break;
  }
  z.type = "camera_observation";
  z.role = "camera_observation";
  z.shape = "box";
  z.parent_frame = "world";
  z.camera_id = camera.name;
  z.x = camera.x + t * dx; z.y = camera.y + t * dy; z.z = work_surface_z;
  z.yaw = camera.yaw; z.dim_x = width; z.dim_y = depth; z.dim_z = 0.02;
  z.status = "Created observation zone for " + camera.name;
  out.ok = true; out.zone = z; out.messages.push_back(z.status);
  return out;
}

bool load_robot_tool_pose_from_environment_yaml(
  const std::string & path, RobotMountConfig * robot_mount, ToolAttachmentConfig * tool_attachment,
  std::vector<std::string> * warnings)
{
  if (!robot_mount || !tool_attachment) {
    if (warnings) warnings->push_back("robot/tool pose load failed: null output pointer");
    return false;
  }

  *robot_mount = RobotMountConfig{};
  *tool_attachment = ToolAttachmentConfig{};
  bool ok = true;

  try {
    YAML::Node root = YAML::LoadFile(path);

    auto load_pose = [&](const YAML::Node & block, double * x, double * y, double * z, double * roll, double * pitch, double * yaw) {
      auto pose = block["pose"];
      if (!pose || !pose.IsMap()) return;
      auto xyz = pose["xyz"];
      if (xyz && xyz.IsSequence() && xyz.size() == 3) {
        *x = xyz[0].as<double>(*x);
        *y = xyz[1].as<double>(*y);
        *z = xyz[2].as<double>(*z);
      }
      auto rpy = pose["rpy"];
      if (rpy && rpy.IsSequence() && rpy.size() == 3) {
        *roll = rpy[0].as<double>(*roll);
        *pitch = rpy[1].as<double>(*pitch);
        *yaw = rpy[2].as<double>(*yaw);
      }
    };

    const auto rm = root["robot_mount"];
    if (rm && rm.IsMap()) {
      robot_mount->parent_link = rm["parent_link"].as<std::string>(robot_mount->parent_link);
      load_pose(
        rm, &robot_mount->x, &robot_mount->y, &robot_mount->z, &robot_mount->roll,
        &robot_mount->pitch, &robot_mount->yaw);
    }

    const auto ta = root["tool_attachment"];
    if (ta && ta.IsMap()) {
      tool_attachment->parent_link = ta["parent_link"].as<std::string>(tool_attachment->parent_link);
      tool_attachment->child_link = ta["child_link"].as<std::string>(tool_attachment->child_link);
      load_pose(
        ta, &tool_attachment->x, &tool_attachment->y, &tool_attachment->z, &tool_attachment->roll,
        &tool_attachment->pitch, &tool_attachment->yaw);
    }
  } catch (const std::exception & e) {
    if (warnings) warnings->push_back(std::string("robot/tool yaml parse warning: ") + e.what());
    return false;
  }

  ok = check_finite("robot_mount.x", robot_mount->x, warnings) &&
    check_finite("robot_mount.y", robot_mount->y, warnings) &&
    check_finite("robot_mount.z", robot_mount->z, warnings) &&
    check_finite("robot_mount.roll", robot_mount->roll, warnings) &&
    check_finite("robot_mount.pitch", robot_mount->pitch, warnings) &&
    check_finite("robot_mount.yaw", robot_mount->yaw, warnings) &&
    check_finite("tool_attachment.x", tool_attachment->x, warnings) &&
    check_finite("tool_attachment.y", tool_attachment->y, warnings) &&
    check_finite("tool_attachment.z", tool_attachment->z, warnings) &&
    check_finite("tool_attachment.roll", tool_attachment->roll, warnings) &&
    check_finite("tool_attachment.pitch", tool_attachment->pitch, warnings) &&
    check_finite("tool_attachment.yaw", tool_attachment->yaw, warnings) && ok;

  warn_if_suspicious_pose("robot_mount", robot_mount->x, robot_mount->y, robot_mount->z, warnings);
  warn_if_suspicious_pose("tool_attachment", tool_attachment->x, tool_attachment->y, tool_attachment->z, warnings);
  if (warnings && robot_mount->parent_link.empty()) warnings->push_back("robot_mount warning: empty parent_link");
  if (warnings && tool_attachment->parent_link.empty()) warnings->push_back("tool_attachment warning: empty parent_link");
  if (warnings && tool_attachment->child_link.empty()) warnings->push_back("tool_attachment warning: empty child_link");

  return ok;
}

PlacedObjectYamlWriteResult save_robot_tool_pose_to_environment_yaml(
  const std::string & path, const RobotMountConfig & robot_mount, const ToolAttachmentConfig & tool_attachment)
{
  PlacedObjectYamlWriteResult r;
  r.path_written = path;
  try {
    YAML::Node root;
    if (std::filesystem::exists(path)) {
      try {
        root = YAML::LoadFile(path);
      } catch (const std::exception & e) {
        r.warnings.push_back(std::string("existing environment yaml malformed; rewriting minimal root: ") + e.what());
        root = YAML::Node(YAML::NodeType::Map);
      }
    } else {
      root = YAML::Node(YAML::NodeType::Map);
    }

    YAML::Node rm;
    rm["parent_link"] = robot_mount.parent_link;
    append_pose_to_yaml(&rm, robot_mount.x, robot_mount.y, robot_mount.z, robot_mount.roll, robot_mount.pitch, robot_mount.yaw);
    root["robot_mount"] = rm;

    YAML::Node ta;
    ta["parent_link"] = tool_attachment.parent_link;
    ta["child_link"] = tool_attachment.child_link;
    append_pose_to_yaml(&ta, tool_attachment.x, tool_attachment.y, tool_attachment.z, tool_attachment.roll, tool_attachment.pitch, tool_attachment.yaw);
    root["tool_attachment"] = ta;

    std::ofstream out(path);
    out << root;
    r.ok = out.good();
    r.objects_saved = 2;
  } catch (const std::exception & e) {
    r.warnings.push_back(std::string("failed to save robot/tool pose: ") + e.what());
  }
  return r;
}

}  // namespace workcell_builder
