// Copyright 2021 ROS Industrial Consortium Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "emd/grasp_execution/context.hpp"
#include "emd/grasp_execution/exception.hpp"

#include "rcl_yaml_param_parser/parser.h"
#include "rcl_yaml_param_parser/types.h"
#include "rcutils/allocator.h"
#include "rcutils/types/string_array.h"

namespace {

const rcl_node_params_t * find_node_params(const rcl_params_t &params,
                                           const std::string &node_name) {
  for (size_t i = 0; i < params.num_nodes; ++i) {
    if (params.node_names[i] && node_name == params.node_names[i]) {
      return &params.params[i];
    }
  }
  return nullptr;
}

std::optional<size_t> find_param_index(const rcl_node_params_t &node_params,
                                       const std::string &param_name) {
  for (size_t i = 0; i < node_params.num_params; ++i) {
    if (node_params.parameter_names[i] &&
        param_name == node_params.parameter_names[i]) {
      return i;
    }
  }
  return std::nullopt;
}

const rcl_variant_t * get_param_variant(const rcl_node_params_t &node_params,
                                        const std::string &param_name) {
  auto idx = find_param_index(node_params, param_name);
  if (!idx) {
    return nullptr;
  }
  return &node_params.parameter_values[*idx];
}

std::optional<std::string> get_string_param(
    const rcl_node_params_t &node_params, const std::string &param_name) {
  const rcl_variant_t * variant = get_param_variant(node_params, param_name);
  if (!variant) {
    return std::nullopt;
  }
  if (!variant->string_value) {
    throw std::runtime_error("expected string parameter '" + param_name + "'");
  }
  return std::string(variant->string_value);
}

std::optional<double> get_double_param(const rcl_node_params_t &node_params,
                                       const std::string &param_name) {
  const rcl_variant_t * variant = get_param_variant(node_params, param_name);
  if (!variant) {
    return std::nullopt;
  }
  if (variant->double_value) {
    return *variant->double_value;
  }
  if (variant->integer_value) {
    return static_cast<double>(*variant->integer_value);
  }
  throw std::runtime_error("expected numeric parameter '" + param_name + "'");
}

std::optional<std::vector<std::string>> get_string_array_param(
    const rcl_node_params_t &node_params, const std::string &param_name) {
  const rcl_variant_t * variant = get_param_variant(node_params, param_name);
  if (!variant) {
    return std::nullopt;
  }
  if (!variant->string_array_value) {
    throw std::runtime_error("expected string array parameter '" + param_name +
                             "'");
  }
  const rcutils_string_array_t &array = *variant->string_array_value;
  std::vector<std::string> values;
  values.reserve(array.size);
  for (size_t i = 0; i < array.size; ++i) {
    if (!array.data[i]) {
      values.emplace_back();
      continue;
    }
    values.emplace_back(array.data[i]);
  }
  return values;
}

} // namespace

namespace grasp_execution {

/////////////////////////////////////////////////
void WorkcellContext::init_from_yaml(const std::string &path) {
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  std::unique_ptr<rcl_params_t, decltype(&rcl_yaml_node_struct_fini)> params(
      rcl_yaml_node_struct_init(allocator), rcl_yaml_node_struct_fini);
  if (!params) {
    throw ContextFileLoadingException(
        path, "failed to allocate yaml parameter structure");
  }
  if (!rcl_parse_yaml_file(path.c_str(), params.get())) {
    throw ContextFileLoadingException(path, "failed to parse yaml parameters");
  }

  const rcl_node_params_t *node_params = find_node_params(*params, "workcell");
  if (!node_params) {
    throw ContextLoadingException("workcell");
  }

  auto group_names = get_string_array_param(*node_params, "groups");
  if (!group_names || group_names->empty()) {
    throw ContextLoadingException("groups", "workcell");
  }

  for (const auto &group_name : *group_names) {
    this->init_group(group_name);

    std::string group_prefix = "groups." + group_name + ".";

    std::string prefix =
        get_string_param(*node_params, group_prefix + "prefix").value_or("");
    if (!prefix.empty() && prefix.back() != '_' && prefix.back() != '/') {
      prefix.push_back('_');
    }
    this->groups[group_name].prefix = prefix;

    auto executor_names = get_string_array_param(
        *node_params, group_prefix + "executors");
    if (!executor_names || executor_names->empty()) {
      this->load_execution_method(group_name, "default",
                                  "grasp_execution/DefaultExecutor", "");
    } else {
      for (const auto &execution_method : *executor_names) {
        std::string executor_prefix =
            group_prefix + "executors." + execution_method + ".";
        auto plugin =
            get_string_param(*node_params, executor_prefix + "plugin");
        if (!plugin) {
          throw ContextLoadingException("plugin",
                                        group_name + ".executors");
        }
        std::string controller =
            get_string_param(*node_params, executor_prefix + "controller")
                .value_or("");
        this->load_execution_method(group_name, execution_method, *plugin,
                                    controller);
      }
    }

    auto ee_names = get_string_array_param(
        *node_params, group_prefix + "end_effectors");
    if (!ee_names) {
      continue;
    }
    for (const auto &ee_name : *ee_names) {
      std::string ee_prefix =
          group_prefix + "end_effectors." + ee_name + ".";
      auto ee_brand = get_string_param(*node_params, ee_prefix + "brand");
      if (!ee_brand) {
        throw ContextLoadingException("brand",
                                      group_name + ".end_effectors." + ee_name);
      }
      auto ee_link = get_string_param(*node_params, ee_prefix + "link");
      if (!ee_link) {
        throw ContextLoadingException("link",
                                      group_name + ".end_effectors." +
                                          *ee_brand);
      }

      double ee_clearance =
          get_double_param(*node_params, ee_prefix + "clearance").value_or(
              0.0);
      std::string driver_prefix = ee_prefix + "driver.";
      auto driver_plugin =
          get_string_param(*node_params, driver_prefix + "plugin");
      if (!driver_plugin) {
        this->load_ee(group_name, ee_name, *ee_brand, *ee_link, ee_clearance,
                      "grasp_execution/DummyGripperDriverPlugin", "");
      } else {
        std::string driver_controller =
            get_string_param(*node_params, driver_prefix + "controller")
                .value_or("");
        this->load_ee(group_name, ee_name, *ee_brand, *ee_link, ee_clearance,
                      *driver_plugin, driver_controller);
      }
    }
  }
}
} // namespace grasp_execution
