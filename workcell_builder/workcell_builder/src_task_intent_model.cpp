#include "task_intent_model.hpp"

#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <regex>

namespace workcell_builder
{
namespace
{
std::string text_or(const YAML::Node & node, const char * key, const std::string & fallback = "")
{
  return node && node.IsMap() && node[key] && node[key].IsScalar() ? node[key].as<std::string>() : fallback;
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
  if (tag == "tag:yaml.org,2002:int" || tag == "tag:yaml.org,2002:float" || (tag != "!" && std::regex_match(raw, std::regex(R"(^[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$)")))) {
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
    std::sort(keys.begin(), keys.end());
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
  model.pick_selection.color = text_or(filter, "color");
  const auto grasp = pick["grasp"];
  model.grasp.policy = text_or(grasp, "policy", "AUTO");
  model.grasp.required_capability = text_or(grasp, "required_capability");
  model.grasp.strategy_ref = text_or(grasp, "strategy_ref");
  const auto place = root["place"];
  const auto target = place["target"];
  model.place.asset_ref = text_or(target, "asset_ref");
  model.place.region_ref = text_or(target, "region_ref");
  model.place.policy = text_or(place["placement"], "policy", "AUTO");
  model.release_strategy = text_or(place["release"], "strategy", "tool_release");
  const auto safety = root["safety"];
  model.safety.execution_mode = text_or(safety, "execution_mode", "simulation_preview");
  model.safety.require_fake_hardware = safety && safety["require_fake_hardware"] ? safety["require_fake_hardware"].as<bool>() : true;
  model.safety.real_hardware_enabled = safety && safety["real_hardware_enabled"] ? safety["real_hardware_enabled"].as<bool>() : false;
  model.safety.preview_policy = text_or(safety, "preview_policy", "diagnostic_if_unresolved");
  return model;
}

std::string canonical_task_intent_json(const std::string & yaml_text)
{
  return canonical(YAML::Load(yaml_text));
}

std::string canonical_task_intent_sha256(const std::string & yaml_text)
{
  const auto bytes = canonical_task_intent_json(yaml_text);
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(), digest);
  std::ostringstream out;
  for (unsigned char c : digest) out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c);
  return out.str();
}
}  // namespace workcell_builder
