#include "rviz_pose_feedback_importer.hpp"

#include <cmath>
#include <fstream>
#include <limits>

#include <yaml-cpp/yaml.h>

namespace workcell_builder
{

namespace
{
bool parse_finite_double(const YAML::Node & node, const char * key, double & out)
{
  if (!node[key]) {
    return false;
  }
  try {
    out = node[key].as<double>();
  } catch (const YAML::Exception &) {
    return false;
  }
  return std::isfinite(out);
}

void add_large_coord_warnings(RvizPoseFeedbackObjectEntry & entry)
{
  if (std::abs(entry.x) > 100.0) entry.warnings.push_back("X coordinate magnitude exceeds 100.");
  if (std::abs(entry.y) > 100.0) entry.warnings.push_back("Y coordinate magnitude exceeds 100.");
  if (std::abs(entry.z) > 100.0) entry.warnings.push_back("Z coordinate magnitude exceeds 100.");
}
}  // namespace

bool RvizPoseFeedbackImportSummary::has_fatal_error() const
{
  return !errors.empty();
}

RvizPoseFeedbackImportSummary parse_rviz_pose_feedback_file(
  const std::string & scene_name,
  const std::string & feedback_yaml_path) noexcept
{
  RvizPoseFeedbackImportSummary out;
  out.scene_name = scene_name;

  std::ifstream input(feedback_yaml_path);
  if (!input.good()) {
    out.errors.push_back("Feedback file missing: " + feedback_yaml_path);
    return out;
  }

  YAML::Node root;
  try {
    root = YAML::Load(input);
  } catch (const YAML::Exception & ex) {
    out.errors.push_back(std::string("Malformed YAML: ") + ex.what());
    return out;
  }

  if (!root || !root.IsMap()) {
    out.errors.push_back("Malformed YAML: root is not a map.");
    return out;
  }

  if (root["scene_name"]) {
    try {
      out.scene_name = root["scene_name"].as<std::string>();
    } catch (const YAML::Exception &) {
      out.warnings.push_back("scene_name field is not a string; using provided scene name.");
    }
  }

  if (root["source"]) {
    try {
      out.source = root["source"].as<std::string>();
    } catch (const YAML::Exception &) {
      out.errors.push_back("source field exists but is not a string.");
    }
  }

  if (out.source != "rviz_interactive_marker_preview") {
    out.errors.push_back("Rejected feedback: source must be rviz_interactive_marker_preview.");
  }

  bool safe_flag = true;
  if (!root["safe_for_robot_motion"]) {
    out.errors.push_back("Rejected feedback: safe_for_robot_motion must be explicitly false.");
  } else {
    try {
      safe_flag = root["safe_for_robot_motion"].as<bool>();
    } catch (const YAML::Exception &) {
      out.errors.push_back("Rejected feedback: safe_for_robot_motion is not a boolean false.");
    }
  }
  out.safe_for_robot_motion = safe_flag;
  if (safe_flag) {
    out.errors.push_back("Rejected feedback: safe_for_robot_motion must be false.");
  }

  const YAML::Node objects = root["objects"];
  if (!objects || !objects.IsSequence()) {
    out.errors.push_back("Missing or invalid objects list.");
    return out;
  }

  for (std::size_t i = 0; i < objects.size(); ++i) {
    RvizPoseFeedbackObjectEntry entry;
    const YAML::Node object = objects[i];
    if (!object || !object.IsMap()) {
      entry.errors.push_back("Object entry is not a map.");
      out.entries.push_back(entry);
      continue;
    }

    try {
      entry.name = object["name"].as<std::string>();
    } catch (const YAML::Exception &) {
      entry.errors.push_back("Missing or invalid name.");
    }

    if (object["status"]) {
      try {
        entry.status = object["status"].as<std::string>();
      } catch (const YAML::Exception &) {
        entry.errors.push_back("Invalid status field.");
      }
    }

    if (object["original_mesh"]) {
      try {
        entry.original_mesh = object["original_mesh"].as<std::string>();
      } catch (const YAML::Exception &) {
        entry.errors.push_back("Invalid original_mesh field.");
      }
    }

    if (!parse_finite_double(object, "x", entry.x)) entry.errors.push_back("Missing or non-finite x.");
    if (!parse_finite_double(object, "y", entry.y)) entry.errors.push_back("Missing or non-finite y.");
    if (!parse_finite_double(object, "z", entry.z)) entry.errors.push_back("Missing or non-finite z.");
    if (!parse_finite_double(object, "roll", entry.roll)) entry.errors.push_back("Missing or non-finite roll.");
    if (!parse_finite_double(object, "pitch", entry.pitch)) entry.errors.push_back("Missing or non-finite pitch.");
    if (!parse_finite_double(object, "yaw", entry.yaw)) entry.errors.push_back("Missing or non-finite yaw.");

    add_large_coord_warnings(entry);
    entry.valid = entry.errors.empty();
    out.entries.push_back(entry);
  }

  return out;
}

}  // namespace workcell_builder
