#include "workcell_studio_id_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <regex>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;

namespace workcell_builder {
namespace {

std::string lowercase(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return text;
}

std::string read_string(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) return "";
  try {
    return node.as<std::string>("");
  } catch (...) {
    return "";
  }
}

void add_id_if_valid(const YAML::Node & node, std::set<std::string> * ids)
{
  const std::string id = read_string(node);
  if (!id.empty()) ids->insert(id);
}

}  // namespace

std::string workcell_studio_id_prefix_for_type(const std::string & type_or_category)
{
  const std::string text = lowercase(type_or_category);
  if (text.find("table") != std::string::npos) return "table";
  if (text.find("bin") != std::string::npos) return "bin";
  if (text.find("conveyor") != std::string::npos) return "conveyor";
  if (text.find("camera") != std::string::npos) return "camera";
  if (text.find("pick") != std::string::npos && text.find("zone") != std::string::npos) return "pick_zone";
  if (text.find("place") != std::string::npos && text.find("zone") != std::string::npos) return "place_zone";
  if (text.find("object") != std::string::npos) return "object";
  if (text.find("safety") != std::string::npos) return "safety_zone";
  if (text.find("fixture") != std::string::npos) return "fixture";
  return "object";
}

bool workcell_studio_is_valid_id(const std::string & id)
{
  static const std::regex kIdPattern("^[a-z][a-z0-9_]{1,62}$");
  return std::regex_match(id, kIdPattern);
}

std::set<std::string> workcell_studio_collect_layout_ids(const fs::path & layout_path)
{
  std::set<std::string> ids;
  if (!fs::exists(layout_path)) return ids;
  YAML::Node root;
  try {
    root = YAML::LoadFile(layout_path.string());
  } catch (...) {
    return ids;
  }
  YAML::Node placed = root["placed_assets"];
  if (placed && placed.IsSequence()) {
    for (const auto & item : placed) {
      if (!item.IsMap()) continue;
      add_id_if_valid(item["id"], &ids);
    }
  }
  YAML::Node items = root["items"];
  if (items && items.IsSequence()) {
    for (const auto & item : items) {
      if (!item.IsMap()) continue;
      add_id_if_valid(item["id"], &ids);
    }
  }
  return ids;
}

std::string workcell_studio_next_id(const std::string & type_or_category, const std::set<std::string> & reserved_ids)
{
  const std::string prefix = workcell_studio_id_prefix_for_type(type_or_category);
  for (int suffix = 1; suffix <= 99; ++suffix) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%s_%02d", prefix.c_str(), suffix);
    const std::string candidate(buffer);
    if (reserved_ids.find(candidate) == reserved_ids.end()) return candidate;
  }
  return prefix + "_99";
}

}  // namespace workcell_builder
