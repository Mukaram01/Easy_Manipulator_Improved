#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <optional>

namespace workcell_builder
{
struct PerceptionContractSummary
{
  bool enabled{false};
  std::string mode{"legacy_disabled"};
  std::string warning;
};

std::string yaml_scalar_or_empty(const YAML::Node & node);
std::string yaml_map_value_or_empty(const YAML::Node & node, const char * key);
std::string yaml_name_from_node(const YAML::Node & node);
std::string yaml_named_or_scalar(const YAML::Node & node, const char * key);

bool yaml_read_string(const YAML::Node & node, std::string * out);
bool yaml_read_double(const YAML::Node & node, double * out);
bool yaml_read_bool(const YAML::Node & node, bool * out);

YAML::Node yaml_map_key(const YAML::Node & node, const char * key);
YAML::Node yaml_seq_index(const YAML::Node & node, std::size_t index);

YAML::Node get_map(const YAML::Node & node, const char * key);
YAML::Node optional_map(const YAML::Node & node, const char * key);
YAML::Node get_scalar(const YAML::Node & node, const char * key);
YAML::Node optional_scalar(const YAML::Node & node, const char * key);
YAML::Node get_sequence(const YAML::Node & node, const char * key);
std::string get_optional_string(const YAML::Node & node, const char * key, const std::string & fallback = "");
std::optional<bool> get_bool_like(const YAML::Node & node, const char * key);
std::optional<bool> bool_like(const YAML::Node & node);
PerceptionContractSummary parse_perception_contract_summary(const YAML::Node & task_or_root);
}
