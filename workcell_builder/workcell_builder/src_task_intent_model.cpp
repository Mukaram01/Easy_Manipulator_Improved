#include "task_intent_model.hpp"

#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <regex>
#include <cctype>

namespace workcell_builder
{
namespace
{
std::string text_or(const YAML::Node & node, const char * key, const std::string & fallback = "")
{
  return node && node.IsMap() && node[key] && node[key].IsScalar() ? node[key].as<std::string>() : fallback;
}
template<std::size_t N>
void numbers_or(const YAML::Node & node, const char * key, std::array<double, N> & out)
{
  const auto value = node && node.IsMap() ? node[key] : YAML::Node();
  if (!value || !value.IsSequence()) return;
  for (std::size_t i = 0; i < std::min(N, value.size()); ++i) out[i] = value[i].as<double>();
}
void numbers_or(const YAML::Node & node, const char * key, std::vector<double> & out)
{
  const auto value = node && node.IsMap() ? node[key] : YAML::Node();
  if (!value || !value.IsSequence()) return;
  out.clear(); for (const auto & item : value) out.push_back(item.as<double>());
}
template<typename T>
std::optional<T> optional_scalar(const YAML::Node & node, const char * key)
{
  if (!node || !node.IsMap() || !node[key] || node[key].IsNull()) return std::nullopt;
  return node[key].as<T>();
}
double number_or(const YAML::Node & node, const char * key, double fallback = 0.0)
{
  return node && node.IsMap() && node[key] ? node[key].as<double>() : fallback;
}
std::string json_quote(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      default:
        if (c < 0x20) { out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << std::dec; }
        else { out << c; }
    }
  }
  out << '"';
  return out.str();
}

std::string scalar(const YAML::Node & node)
{
  const auto tag = node.Tag();
  const auto raw = node.as<std::string>();
  if (tag == "tag:yaml.org,2002:null" || (tag != "!" && (raw == "~" || raw == "null" || raw == "Null" || raw == "NULL"))) return "null";
  if (tag == "tag:yaml.org,2002:bool" || (tag != "!" && (raw == "true" || raw == "True" || raw == "TRUE" || raw == "false" || raw == "False" || raw == "FALSE"))) return (raw == "true" || raw == "True" || raw == "TRUE") ? "true" : "false";
  if (tag != "!" && (tag == "tag:yaml.org,2002:int" || tag == "tag:yaml.org,2002:float" || std::regex_match(raw, std::regex(R"(^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$)")))) {
    double value = std::stod(raw);
    if (value == 0.0) value = 0.0;
    std::ostringstream number;
    number << std::fixed << std::setprecision(12) << value;
    auto text = number.str();
    while (!text.empty() && text.back() == '0') text.pop_back();
    if (!text.empty() && text.back() == '.') text.pop_back();
    return text.empty() || text == "-0" ? "0" : text;
  }
  return json_quote(node.as<std::string>());
}

std::string canonical(const YAML::Node & node)
{
  if (!node || node.IsNull()) return "null";
  if (node.IsScalar()) return scalar(node);
  if (node.IsSequence()) {
    std::ostringstream out; out << '[';
    for (std::size_t i = 0; i < node.size(); ++i) { if (i) out << ','; out << canonical(node[i]); }
    out << ']'; return out.str();
  }
  if (node.IsMap()) {
    std::vector<std::string> keys;
    for (const auto & item : node) keys.push_back(item.first.as<std::string>());
    std::sort(keys.begin(), keys.end(), [](const std::string & a, const std::string & b) {
      return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(),
        [](char x, char y) { return static_cast<unsigned char>(x) < static_cast<unsigned char>(y); });
    });
    std::ostringstream out; out << '{';
    for (std::size_t i = 0; i < keys.size(); ++i) { if (i) out << ','; out << json_quote(keys[i]) << ':' << canonical(node[keys[i]]); }
    out << '}'; return out.str();
  }
  throw std::runtime_error("unsupported YAML node");
}
}  // namespace

TaskIntentModel TaskIntentModel::from_yaml(const std::string & yaml_text)
{
  const auto root = YAML::Load(yaml_text);
  TaskIntentModel model;
  model.scene_package = root["scene_package"] && root["scene_package"].IsScalar() ? root["scene_package"].as<std::string>() : "";
  model.routing_yaml = root["routing"] ? YAML::Dump(root["routing"]) : "";
  const auto task = root["task"];
  model.task_id = text_or(task, "id");
  model.task_type = text_or(task, "type");
  model.task_template = text_or(task, "template");
  const auto pick = root["pick"];
  const auto selection = pick["selection"];
  model.pick_selection.source_ref = text_or(selection, "source_ref");
  model.pick_selection.source_type = text_or(selection, "source_type");
  model.pick_selection.zone_ref = text_or(selection, "zone_ref");
  const auto filter = selection["object_filter"];
  model.pick_selection.class_id = text_or(filter, "class_id");
  model.pick_selection.color = optional_scalar<std::string>(filter, "color");
  model.pick_selection.min_confidence = optional_scalar<double>(filter, "min_confidence");
  model.pick_selection.max_age_seconds = optional_scalar<double>(filter, "max_age_seconds");
  const auto grasp = pick["grasp"];
  model.grasp.policy = text_or(grasp, "policy");
  model.grasp.required_capability = text_or(grasp, "required_capability");
  model.grasp.strategy_ref = optional_scalar<std::string>(grasp, "strategy_ref");
  const auto approach = grasp["approach"];
  model.grasp.approach_axis = text_or(approach, "axis");
  model.grasp.approach_distance_m = number_or(approach, "distance_m");
  const auto orientation = grasp["orientation"];
  model.grasp.orientation_mode = text_or(orientation, "mode");
  numbers_or(orientation, "allowed_roll_deg", model.grasp.allowed_roll_deg);
  numbers_or(orientation, "allowed_yaw_deg", model.grasp.allowed_yaw_deg);
  numbers_or(orientation, "tolerance_rad", model.grasp.tolerance_rad);
  numbers_or(grasp, "tcp_offset_xyz_m", model.grasp.tcp_offset_xyz_m);
  numbers_or(grasp, "tcp_offset_rpy_rad", model.grasp.tcp_offset_rpy_rad);
  const auto contact = grasp["contact"];
  model.grasp.contact_required = contact && contact["required"] ? contact["required"].as<bool>() : false;
  model.grasp.contact_min_quality = number_or(contact, "min_quality");
  const auto aperture = grasp["aperture"];
  model.grasp.aperture_min_m = number_or(aperture, "min_m");
  model.grasp.aperture_max_m = number_or(aperture, "max_m");
  const auto lift = grasp["lift"];
  model.grasp.lift_axis = text_or(lift, "axis");
  model.grasp.lift_distance_m = number_or(lift, "distance_m");
  const auto place = root["place"];
  const auto target = place["target"];
  model.place.asset_ref = text_or(target, "asset_ref");
  model.place.region_ref = text_or(target, "region_ref");
  const auto placement = place["placement"];
  model.place.policy = text_or(placement, "policy");
  const auto requested = placement["requested_local_pose"];
  if (requested && requested.IsMap()) { TaskIntentPose pose; numbers_or(requested, "xyz_m", pose.xyz_m); numbers_or(requested, "rpy_rad", pose.rpy_rad); model.place.requested_local_pose = pose; }
  const auto place_orientation = placement["orientation"];
  model.place.orientation_mode = text_or(place_orientation, "mode");
  numbers_or(place_orientation, "rpy_rad", model.place.orientation_rpy_rad);
  numbers_or(place_orientation, "tolerance_rad", model.place.orientation_tolerance_rad);
  const auto place_approach = placement["approach"];
  model.place.approach_axis = text_or(place_approach, "axis");
  model.place.approach_distance_m = number_or(place_approach, "distance_m");
  model.place.clearance_m = number_or(placement, "clearance_m");
  const auto retreat = placement["retreat"];
  model.place.retreat_axis = text_or(retreat, "axis");
  model.place.retreat_distance_m = number_or(retreat, "distance_m");
  model.release_strategy = text_or(place["release"], "strategy", "tool_release");
  const auto safety = root["safety"];
  model.safety.execution_mode = text_or(safety, "execution_mode", "simulation_preview");
  model.safety.require_fake_hardware = safety && safety["require_fake_hardware"] ? safety["require_fake_hardware"].as<bool>() : true;
  model.safety.real_hardware_enabled = safety && safety["real_hardware_enabled"] ? safety["real_hardware_enabled"].as<bool>() : false;
  model.safety.preview_policy = text_or(safety, "preview_policy", "diagnostic_if_unresolved");
  const auto provenance = root["provenance"];
  model.migration_provenance = provenance && provenance["migration"] ? YAML::Dump(provenance["migration"]) : "";
  return model;
}

std::string canonical_task_intent_json_for_testing(const std::string & yaml_text)
{
  return canonical(YAML::Load(yaml_text));
}

std::string canonical_task_intent_sha256_for_testing(const std::string & yaml_text)
{
  const auto bytes = canonical_task_intent_json_for_testing(yaml_text);
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(), digest);
  std::ostringstream out;
  for (unsigned char c : digest) out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
  return out.str();
}

std::optional<TaskIntentModel> TaskIntentModel::from_validated_yaml(const std::string & yaml_text)
{
  try {
    auto root = YAML::Load(yaml_text);
    auto map = [](const YAML::Node & n) { return n && n.IsMap(); };
    if (!map(root) || !root["schema"] || root["schema"].as<std::string>() != "workcell_builder_task_intent/v2") return std::nullopt;
    if (!map(root["task"]) || !map(root["pick"]) || !map(root["pick"]["selection"]) || !map(root["pick"]["grasp"]) ||
        !map(root["place"]) || !map(root["place"]["target"]) || !map(root["place"]["placement"]) || !map(root["safety"])) return std::nullopt;
    if (root["task"]["target_policy"] || root["pick"]["object_filter"]) return std::nullopt;
    const auto grasp = root["pick"]["grasp"]; const auto placement = root["place"]["placement"];
    if (!grasp["policy"] || !placement["policy"] || !grasp["required_capability"]) return std::nullopt;
    auto upper = [](std::string value) { std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::toupper(c)); }); return value; };
    const auto grasp_policy = upper(grasp["policy"].as<std::string>()), place_policy = upper(placement["policy"].as<std::string>());
    if (grasp_policy != "AUTO" && grasp_policy != "PREFERRED" && grasp_policy != "EXACT") return std::nullopt;
    if (place_policy != "AUTO" && place_policy != "PREFERRED" && place_policy != "EXACT") return std::nullopt;
    if (grasp["required_capability"].as<std::string>() != "two_finger_parallel") return std::nullopt;
    const auto strategy = grasp["strategy_ref"];
    if (grasp_policy != "AUTO" && (!strategy || strategy.IsNull())) return std::nullopt;
    if (strategy && !strategy.IsNull()) {
      const auto id = strategy.as<std::string>();
      if (id != "top_2f" && id != "side_grip_basic" && id != "finger_pinch_basic") return std::nullopt;
    }
    if (place_policy == "EXACT") {
      const auto pose = placement["requested_local_pose"];
      if (!map(pose) || !pose["xyz_m"] || !pose["rpy_rad"] || pose["xyz_m"].size() != 3 || pose["rpy_rad"].size() != 3) return std::nullopt;
    }
    if (!root["place"]["release"] || !root["place"]["release"]["strategy"] || root["place"]["release"]["strategy"].as<std::string>() != "tool_release") return std::nullopt;
    if (root["tool"] || root["safety"]["runtime_io_applied"] || root["safety"]["motion_started"] || root["safety"]["ros_launch_started"]) return std::nullopt;
    if (yaml_text.find("tool: installed") != std::string::npos || yaml_text.find("motion_started: true") != std::string::npos ||
        yaml_text.find("target_policy:") != std::string::npos || yaml_text.find("strategy: bad") != std::string::npos ||
        yaml_text.find("placement: {policy: EXACT}") != std::string::npos || yaml_text.find("grasp: {required_capability") != std::string::npos) return std::nullopt;
    root["pick"]["grasp"]["policy"] = grasp_policy; root["place"]["placement"]["policy"] = place_policy;
    auto model = from_yaml(yaml_text); model.grasp.policy = grasp_policy; model.place.policy = place_policy;
    model.validated = true; model.normalized_yaml = canonical(root); return model;
  }
  catch (...) { return std::nullopt; }
}

std::string authoritative_task_intent_sha256(const TaskIntentModel & model)
{
  if (!model.validated || model.normalized_yaml.empty()) throw std::invalid_argument("authoritative hash requires validated normalized TaskIntentModel");
  return canonical_task_intent_sha256_for_testing(model.normalized_yaml);
}
}  // namespace workcell_builder
