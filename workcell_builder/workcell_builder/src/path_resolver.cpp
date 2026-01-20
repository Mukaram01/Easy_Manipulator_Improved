// Copyright 2024 Advanced Remanufacturing and Technology Centre
// Copyright 2024 ROS-Industrial Consortium Asia Pacific Team
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

#include "path_resolver.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <optional>
#include <string>

namespace fs = boost::filesystem;

namespace
{
std::optional<fs::path> g_workspace_root_override;
std::optional<fs::path> g_scenes_root_override;

fs::path resolve_share_subdir(const char * subdir)
{
  try {
    const auto share = ament_index_cpp::get_package_share_directory("workcell_builder");
    return fs::path(share) / subdir;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
  }
  return {};
}
}  // namespace

fs::path PathResolver::assets_root()
{
  return resolve_share_subdir("assets");
}

fs::path PathResolver::templates_root()
{
  return resolve_share_subdir("templates");
}

fs::path PathResolver::workspace_root()
{
  if (g_workspace_root_override.has_value()) {
    return g_workspace_root_override.value();
  }
  const char * env_root = std::getenv("WORKCELL_BUILDER_ROOT");
  if (env_root != nullptr && std::string(env_root).size() > 0) {
    return fs::path(env_root);
  }
  return {};
}

fs::path PathResolver::scenes_root()
{
  if (g_scenes_root_override.has_value()) {
    return g_scenes_root_override.value();
  }
  const fs::path workspace = workspace_root();
  if (workspace.empty()) {
    return {};
  }
  return workspace / "scenes";
}

void PathResolver::set_workspace_root_override(const fs::path & workspace_root)
{
  if (workspace_root.empty()) {
    g_workspace_root_override.reset();
  } else {
    g_workspace_root_override = workspace_root;
  }
}

void PathResolver::set_scenes_root_override(const fs::path & scenes_root)
{
  if (scenes_root.empty()) {
    g_scenes_root_override.reset();
  } else {
    g_scenes_root_override = scenes_root;
  }
}
