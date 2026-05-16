#pragma once

#include <yaml-cpp/yaml.h>
#include <string>

namespace workcell_builder
{
std::string yaml_scalar_or_empty(const YAML::Node & node);
std::string yaml_map_value_or_empty(const YAML::Node & node, const char * key);
std::string yaml_name_from_node(const YAML::Node & node);
std::string yaml_named_or_scalar(const YAML::Node & node, const char * key);

bool yaml_read_string(const YAML::Node & node, std::string * out);
bool yaml_read_double(const YAML::Node & node, double * out);
bool yaml_read_bool(const YAML::Node & node, bool * out);

YAML::Node yaml_map_key(const YAML::Node & node, const char * key);
YAML::Node yaml_seq_index(const YAML::Node & node, std::size_t index);
}
