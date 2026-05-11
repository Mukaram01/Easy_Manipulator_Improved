#pragma once

#include <string>
#include <vector>

struct AssetCandidate
{
  std::string label;
  std::string description_package;
  std::string moveit_config_package;
  std::string urdf_or_xacro;
  std::string inferred_type;
  std::string status;
};

struct AssetDiscoveryReport
{
  std::vector<std::string> robot_paths;
  std::vector<std::string> end_effector_paths;
  std::vector<std::string> environment_paths;
  std::vector<AssetCandidate> robots;
  std::vector<AssetCandidate> end_effectors;
  std::vector<std::string> stl_meshes;
};

AssetDiscoveryReport discover_workcell_assets(const std::string & workspace_root, const std::string & repo_root);
