#pragma once

#include <yaml-cpp/yaml.h>
#include <string>

namespace workcell_builder
{
std::string yaml_scalar_or_empty(const YAML::Node & node);
std::string yaml_map_value_or_empty(const YAML::Node & node, const char * key);
std::string yaml_name_from_node(const YAML::Node & node);
std::string yaml_named_or_scalar(const YAML::Node & node, const char * key);
}

