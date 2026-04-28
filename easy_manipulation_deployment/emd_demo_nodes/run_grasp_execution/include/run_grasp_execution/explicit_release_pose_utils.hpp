#pragma once

#include <array>
#include <cmath>
#include <fstream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace run_grasp_execution
{

struct ExplicitReleasePoseEntry
{
  std::string object_id;
  std::string normalized_object_id;
  std::string destination_id;
  std::string destination_name;
  std::string frame_id;
  geometry_msgs::msg::Pose pose;
  bool valid_pose{false};
};

struct ExplicitReleasePoseLoadResult
{
  bool loaded{false};
  std::vector<ExplicitReleasePoseEntry> entries;
  std::vector<std::string> warnings;
  std::string error;
};

inline std::string normalize_object_id(std::string value)
{
  while (!value.empty() && value.front() == '#') {
    value.erase(value.begin());
  }
  return value;
}

inline bool is_finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

inline std::optional<std::array<double, 3>> read_vec3(
  const boost::property_tree::ptree & node,
  const std::string & key)
{
  const auto child = node.get_child_optional(key);
  if (!child.has_value()) {
    return std::nullopt;
  }
  std::array<double, 3> out{};
  size_t i = 0;
  for (const auto & item : child.value()) {
    if (i >= out.size()) {
      return std::nullopt;
    }
    out[i++] = item.second.get_value<double>();
  }
  if (i != out.size()) {
    return std::nullopt;
  }
  return out;
}

inline std::optional<std::array<double, 4>> read_vec4(
  const boost::property_tree::ptree & node,
  const std::string & key)
{
  const auto child = node.get_child_optional(key);
  if (!child.has_value()) {
    return std::nullopt;
  }
  std::array<double, 4> out{};
  size_t i = 0;
  for (const auto & item : child.value()) {
    if (i >= out.size()) {
      return std::nullopt;
    }
    out[i++] = item.second.get_value<double>();
  }
  if (i != out.size()) {
    return std::nullopt;
  }
  return out;
}

inline ExplicitReleasePoseLoadResult load_explicit_release_pose_bridge_payload(const std::string & path)
{
  ExplicitReleasePoseLoadResult result;
  if (path.empty()) {
    result.error = "Bridge payload path is empty.";
    return result;
  }

  std::ifstream input(path);
  if (!input.good()) {
    result.error = "Cannot open bridge payload file: " + path;
    return result;
  }

  boost::property_tree::ptree root;
  try {
    boost::property_tree::read_json(path, root);
  } catch (const std::exception & ex) {
    result.error = std::string("Failed to parse bridge payload JSON: ") + ex.what();
    return result;
  }

  const auto targets = root.get_child_optional("grasp_task.grasp_targets");
  if (!targets.has_value()) {
    result.error = "Bridge payload missing grasp_task.grasp_targets.";
    return result;
  }

  for (const auto & node_item : targets.value()) {
    const auto & target = node_item.second;
    ExplicitReleasePoseEntry entry;
    entry.object_id = target.get<std::string>("object_id", "");
    entry.normalized_object_id = normalize_object_id(entry.object_id);
    entry.destination_id = target.get<std::string>("destination_id", "");
    entry.destination_name = target.get<std::string>("destination_name", "");

    const auto pose_node = target.get_child_optional("destination_pose");
    if (!pose_node.has_value()) {
      result.warnings.push_back("Missing destination_pose for object_id='" + entry.object_id + "'.");
      result.entries.push_back(entry);
      continue;
    }

    entry.frame_id = pose_node.value().get<std::string>("frame_id", "");
    const auto xyz = read_vec3(pose_node.value(), "xyz");
    const auto quat = read_vec4(pose_node.value(), "quaternion_xyzw");
    const auto rpy = read_vec3(pose_node.value(), "rpy");

    if (!xyz.has_value()) {
      result.warnings.push_back("Malformed destination_pose.xyz for object_id='" + entry.object_id + "'.");
      result.entries.push_back(entry);
      continue;
    }

    entry.pose.position.x = (*xyz)[0];
    entry.pose.position.y = (*xyz)[1];
    entry.pose.position.z = (*xyz)[2];

    if (quat.has_value()) {
      entry.pose.orientation.x = (*quat)[0];
      entry.pose.orientation.y = (*quat)[1];
      entry.pose.orientation.z = (*quat)[2];
      entry.pose.orientation.w = (*quat)[3];
    } else if (rpy.has_value()) {
      tf2::Quaternion q;
      q.setRPY((*rpy)[0], (*rpy)[1], (*rpy)[2]);
      q.normalize();
      entry.pose.orientation.x = q.x();
      entry.pose.orientation.y = q.y();
      entry.pose.orientation.z = q.z();
      entry.pose.orientation.w = q.w();
    } else {
      result.warnings.push_back("Missing destination_pose orientation for object_id='" + entry.object_id + "'.");
      result.entries.push_back(entry);
      continue;
    }

    entry.valid_pose = is_finite_pose(entry.pose);
    if (!entry.valid_pose) {
      result.warnings.push_back("Non-finite destination pose for object_id='" + entry.object_id + "'.");
    }

    result.entries.push_back(entry);
  }

  result.loaded = true;
  return result;
}

}  // namespace run_grasp_execution
