#include "workcell_yaml_utils.hpp"
#include <algorithm>

namespace workcell_builder
{
static std::string to_lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return value;
}

static std::string canonicalize_scene3d_camera_mode(const std::string & raw_mode, std::string * legacy_source)
{
  const std::string raw_lower = to_lower_copy(raw_mode);
  if (legacy_source) legacy_source->clear();
  if (raw_lower.empty()) return "none";
  if (raw_lower == "none") return "none";
  if (raw_lower == "epd_optional") return "epd_optional";
  if (raw_lower == "snapshot_overlay") return "snapshot_overlay";

  if (raw_lower == "live_epd" || raw_lower == "live_epd_realsense" || raw_lower == "live") {
    if (legacy_source) *legacy_source = raw_mode;
    return "epd_optional";
  }
  if (raw_lower == "saved_snapshot" || raw_lower == "epd_replay") {
    if (legacy_source) *legacy_source = raw_mode;
    return "snapshot_overlay";
  }
  if (raw_lower == "manual_simulated" || raw_lower == "not_configured" || raw_lower == "disabled") {
    if (legacy_source) *legacy_source = raw_mode;
    return "none";
  }

  if (legacy_source) *legacy_source = raw_mode;
  return "none";
}

std::string yaml_scalar_or_empty(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) return "";
  try { return node.as<std::string>(); } catch (...) { return ""; }
}

std::string yaml_map_value_or_empty(const YAML::Node & node, const char * key)
{
  if (!node || !node.IsMap() || key == nullptr) return "";
  return yaml_scalar_or_empty(node[key]);
}

std::string yaml_name_from_node(const YAML::Node & node)
{
  if (!node) return "";
  if (node.IsScalar()) return yaml_scalar_or_empty(node);
  if (node.IsMap()) return yaml_map_value_or_empty(node, "name");
  return "";
}

std::string yaml_named_or_scalar(const YAML::Node & node, const char * key)
{
  if (!node || key == nullptr) return "";
  if (node.IsScalar()) return yaml_scalar_or_empty(node);
  if (node.IsMap()) {
    YAML::Node child;
    try { child = node[key]; } catch (...) { return ""; }
    const std::string direct = yaml_scalar_or_empty(child);
    if (!direct.empty()) return direct;
    if (child && child.IsMap()) return yaml_map_value_or_empty(child, "name");
  }
  return "";
}

bool yaml_read_string(const YAML::Node & node, std::string * out)
{
  if (out == nullptr || !node.IsDefined() || !node.IsScalar()) return false;
  try { *out = node.as<std::string>(); return true; } catch (...) { return false; }
}

bool yaml_read_double(const YAML::Node & node, double * out)
{
  if (out == nullptr || !node.IsDefined() || !node.IsScalar()) return false;
  try { *out = node.as<double>(); return true; } catch (...) { return false; }
}

bool yaml_read_bool(const YAML::Node & node, bool * out)
{
  if (out == nullptr || !node.IsDefined() || !node.IsScalar()) return false;
  try { *out = node.as<bool>(); return true; } catch (...) { return false; }
}

YAML::Node yaml_map_key(const YAML::Node & node, const char * key)
{
  if (key == nullptr) return YAML::Node(YAML::NodeType::Undefined);
  try {
    if (!node.IsDefined() || !node.IsMap()) return YAML::Node(YAML::NodeType::Undefined);
    const YAML::Node child = node[key];
    return child ? child : YAML::Node(YAML::NodeType::Undefined);
  } catch (...) { return YAML::Node(YAML::NodeType::Undefined); }
}

YAML::Node yaml_seq_index(const YAML::Node & node, std::size_t index)
{
  try {
    if (!node.IsDefined() || !node.IsSequence() || index >= node.size()) return YAML::Node(YAML::NodeType::Undefined);
    const YAML::Node child = node[index];
    return child ? child : YAML::Node(YAML::NodeType::Undefined);
  } catch (...) { return YAML::Node(YAML::NodeType::Undefined); }
}

YAML::Node get_map(const YAML::Node & node, const char * key)
{
  if (!node.IsDefined() || !node.IsMap() || key == nullptr) return YAML::Node();
  YAML::Node child;
  try { child = node[key]; } catch (...) { return YAML::Node(); }
  return (child && child.IsMap()) ? child : YAML::Node();
}
YAML::Node optional_map(const YAML::Node & node, const char * key) { return get_map(node, key); }

YAML::Node get_scalar(const YAML::Node & node, const char * key)
{
  if (!node.IsDefined() || !node.IsMap() || key == nullptr) return YAML::Node();
  YAML::Node child;
  try { child = node[key]; } catch (...) { return YAML::Node(); }
  return (child && child.IsScalar()) ? child : YAML::Node();
}
YAML::Node optional_scalar(const YAML::Node & node, const char * key) { return get_scalar(node, key); }

YAML::Node get_sequence(const YAML::Node & node, const char * key)
{
  if (!node.IsDefined() || !node.IsMap() || key == nullptr) return YAML::Node();
  YAML::Node child;
  try { child = node[key]; } catch (...) { return YAML::Node(); }
  return (child && child.IsSequence()) ? child : YAML::Node();
}

std::string get_optional_string(const YAML::Node & node, const char * key, const std::string & fallback)
{
  try {
    const YAML::Node scalar = get_scalar(node, key);
    if (!scalar) return fallback;
    std::string out;
    if (!yaml_read_string(scalar, &out)) return fallback;
    return out;
  } catch (...) {
    return fallback;
  }
}

std::optional<bool> bool_like(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) return std::nullopt;
  try {
    return node.as<bool>();
  } catch (...) {}
  try {
    std::string value = node.as<std::string>();
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    if (value == "1" || value == "true" || value == "yes" || value == "on" || value == "enabled") return true;
    if (value == "0" || value == "false" || value == "no" || value == "off" || value == "disabled") return false;
  } catch (...) {}
  return std::nullopt;
}

std::optional<bool> get_bool_like(const YAML::Node & node, const char * key)
{
  return bool_like(get_scalar(node, key));
}

PerceptionContractSummary parse_perception_contract_summary(const YAML::Node & task_or_root)
{
  PerceptionContractSummary out;
  const YAML::Node task = (task_or_root && task_or_root.IsMap() && task_or_root["task"]) ?
    task_or_root["task"] : task_or_root;
  if (!task || !task.IsMap()) {
    return out;
  }
  const YAML::Node perception = task["perception"];
  if (!perception) {
    return out;
  }
  if (!perception.IsMap()) {
    std::string scalar = yaml_scalar_or_empty(perception);
    std::string legacy;
    const std::string mode = canonicalize_scene3d_camera_mode(scalar, &legacy);
    if (mode == "none") {
      out.mode = "legacy_disabled";
      out.legacy_source_mode = legacy.empty() ? scalar : legacy;
      return out;
    }
    out.enabled = true;
    out.mode = mode;
    out.legacy_source_mode = legacy;
    return out;
  }
  if (perception.size() == 0) {
    return out;
  }
  out.enabled = true;
  const std::string raw_mode = yaml_map_value_or_empty(perception, "mode");
  if (raw_mode.empty()) {
    out.mode = "epd_optional";
    return out;
  }
  out.mode = canonicalize_scene3d_camera_mode(raw_mode, &out.legacy_source_mode);
  return out;
}
}
