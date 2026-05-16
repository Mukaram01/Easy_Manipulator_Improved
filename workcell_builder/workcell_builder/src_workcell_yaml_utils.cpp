#include "workcell_yaml_utils.hpp"

namespace workcell_builder
{
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
    const std::string direct = yaml_scalar_or_empty(node[key]);
    if (!direct.empty()) return direct;
    const YAML::Node nested = node[key];
    if (nested.IsMap()) return yaml_map_value_or_empty(nested, "name");
  }
  return "";
}
}

