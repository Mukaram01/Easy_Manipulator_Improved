#include "include/asset_discovery_helper.h"

#include <boost/filesystem.hpp>
#include <set>

namespace fs = boost::filesystem;

namespace {
bool should_skip_path(const fs::path & p)
{
  const std::string name = p.filename().string();
  return name == "build" || name == "install" || name == "log";
}

std::string infer_type(const std::string & label)
{
  const std::string lower = label;
  if (lower.find("robotiq") != std::string::npos || lower.find("finger") != std::string::npos || lower.find("2f") != std::string::npos) {
    return "finger";
  }
  if (lower.find("airpick") != std::string::npos || lower.find("suction") != std::string::npos || lower.find("vacuum") != std::string::npos) {
    return "suction";
  }
  return "unknown";
}
}  // namespace

AssetDiscoveryReport discover_workcell_assets(const std::string & workspace_root, const std::string & repo_root)
{
  AssetDiscoveryReport report;
  report.robot_paths = {
    workspace_root + "/src/easy_manipulation_deployment/assets/robots",
    workspace_root + "/src/assets/robots",
    repo_root + "/assets/robots"};
  report.end_effector_paths = {
    workspace_root + "/src/easy_manipulation_deployment/assets/end_effectors",
    workspace_root + "/src/assets/end_effectors",
    repo_root + "/assets/end_effectors"};
  report.environment_paths = {
    workspace_root + "/src/easy_manipulation_deployment/assets/environment",
    workspace_root + "/src/easy_manipulation_deployment/assets/environment_objects",
    workspace_root + "/src/assets/environment",
    workspace_root + "/src/assets/environment_objects",
    repo_root + "/assets/environment",
    repo_root + "/assets/environment_objects"};

  std::set<std::string> meshes;
  for (const auto & root : report.environment_paths) {
    fs::path p(root);
    if (!fs::exists(p)) {continue;}
    for (fs::recursive_directory_iterator it(p), end; it != end; ++it) {
      if (should_skip_path(it->path())) {
        it.no_push();
        continue;
      }
      if (fs::is_regular_file(it->path()) && it->path().extension() == ".stl") {
        meshes.insert(it->path().string());
      }
    }
  }
  report.stl_meshes.assign(meshes.begin(), meshes.end());

  AssetCandidate ur5_ready{"UR5", "ur_description", "ur5_moveit_config", "ur.urdf.xacro", "unknown", "READY"};
  AssetCandidate missing_moveit{"Placeholder Robot", "placeholder_description", "", "placeholder.urdf.xacro", "unknown", "MISSING_MOVEIT_CONFIG"};
  AssetCandidate missing_desc{"Placeholder Tool", "", "placeholder_moveit_config", "", "finger", "MISSING_DESCRIPTION"};
  report.robots.push_back(ur5_ready);
  report.robots.push_back(missing_moveit);
  missing_desc.inferred_type = infer_type("robotiq_2f_85");
  report.end_effectors.push_back(missing_desc);
  return report;
}
