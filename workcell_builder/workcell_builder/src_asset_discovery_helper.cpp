#include "include/asset_discovery_helper.h"

#include <boost/filesystem.hpp>
#include <algorithm>
#include <set>
#include <map>
#include <cctype>

namespace fs = boost::filesystem;

namespace {

const std::vector<std::string> kEnvironmentPresets = {"table", "bin", "conveyor_placeholder", "fixture", "custom_stl"};
[[maybe_unused]] const char * kExternalAbsoluteStlWarning = "Absolute mesh path is outside workspace";

bool should_skip_path(const fs::path & p)
{
  const std::string name = p.filename().string();
  return name == "build" || name == "install" || name == "log";
}

std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

std::string infer_type(const std::string & label)
{
  const std::string lower = to_lower(label);
  if (lower.find("robotiq") != std::string::npos || lower.find("finger") != std::string::npos || lower.find("2f") != std::string::npos) {
    return "finger";
  }
  if (lower.find("airpick") != std::string::npos || lower.find("suction") != std::string::npos || lower.find("vacuum") != std::string::npos) {
    return "suction";
  }
  return "unknown";
}

std::string derive_urdf_hint(const fs::path & package_path)
{
  const fs::path urdf_dir = package_path / "urdf";
  if (!fs::exists(urdf_dir) || !fs::is_directory(urdf_dir)) {
    return "";
  }
  for (const auto & entry : fs::directory_iterator(urdf_dir)) {
    if (!fs::is_regular_file(entry.path())) {
      continue;
    }
    const std::string ext = entry.path().extension().string();
    if (ext == ".xacro" || ext == ".urdf") {
      return entry.path().filename().string();
    }
  }
  return "";
}

std::string normalize_mesh_path(const std::string & mesh_path, const std::string & workspace_root, const std::string & repo_root)
{
  const fs::path mesh(mesh_path);
  if (!mesh.is_absolute()) {
    return mesh.generic_string();
  }
  const fs::path ws(workspace_root);
  const fs::path repo(repo_root);
  if (!workspace_root.empty() && mesh.string().find(ws.string()) == 0) {
    return fs::relative(mesh, ws).generic_string();
  }
  if (!repo_root.empty() && mesh.string().find(repo.string()) == 0) {
    return fs::relative(mesh, repo).generic_string();
  }
  return mesh.generic_string();
}

void scan_asset_packages(const std::string & root, bool is_robot, std::vector<AssetCandidate> & out)
{
  fs::path p(root);
  if (!fs::exists(p) || !fs::is_directory(p)) {
    return;
  }

  std::map<std::string, std::string> descriptions;
  std::map<std::string, std::string> moveits;
  for (fs::recursive_directory_iterator it(p), end; it != end; ++it) {
    if (should_skip_path(it->path())) {
      it.no_push();
      continue;
    }
    if (!fs::is_directory(it->path())) {
      continue;
    }
    const std::string folder = it->path().filename().string();
    if (folder.size() > 12 && folder.substr(folder.size() - 12) == "_description") {
      descriptions[folder.substr(0, folder.size() - 12)] = it->path().string();
    }
    if (folder.size() > 14 && folder.substr(folder.size() - 14) == "_moveit_config") {
      moveits[folder.substr(0, folder.size() - 14)] = it->path().string();
    }
  }

  std::set<std::string> keys;
  for (const auto & kv : descriptions) { keys.insert(kv.first); }
  for (const auto & kv : moveits) { keys.insert(kv.first); }

  for (const auto & key : keys) {
    AssetCandidate c;
    c.label = key;
    c.description_package = descriptions.count(key) ? (key + "_description") : "";
    c.moveit_config_package = moveits.count(key) ? (key + "_moveit_config") : "";
    c.urdf_or_xacro = descriptions.count(key) ? derive_urdf_hint(fs::path(descriptions[key])) : "";
    c.inferred_type = is_robot ? "unknown" : infer_type(key);
    if (c.description_package.empty()) {
      c.status = "MISSING_DESCRIPTION";
    } else if (c.moveit_config_package.empty()) {
      c.status = "MISSING_MOVEIT_CONFIG";
    } else {
      c.status = "READY";
    }
    out.push_back(c);
  }
}
}  // namespace

AssetDiscoveryReport discover_workcell_assets(const std::string & workspace_root, const std::string & repo_root)
{
  AssetDiscoveryReport report;
  report.robot_paths = {
    workspace_root + "/src/easy_manipulation_deployment/assets/robots",
    workspace_root + "/src/assets/robots",
    repo_root + "/assets/robots",
    repo_root + "/workcell_builder/workcell_builder/assets/robots"};
  report.end_effector_paths = {
    workspace_root + "/src/easy_manipulation_deployment/assets/end_effectors",
    workspace_root + "/src/assets/end_effectors",
    repo_root + "/assets/end_effectors",
    repo_root + "/workcell_builder/workcell_builder/assets/end_effectors"};
  report.environment_paths = {
    workspace_root + "/src/easy_manipulation_deployment/assets/environment",
    workspace_root + "/src/easy_manipulation_deployment/assets/environment_objects",
    workspace_root + "/src/assets/environment",
    workspace_root + "/src/assets/environment_objects",
    repo_root + "/assets/environment",
    repo_root + "/assets/environment_objects",
    repo_root + "/workcell_builder/workcell_builder/assets/environment"};

  std::set<std::string> meshes;
  for (const auto & root : report.environment_paths) {
    fs::path p(root);
    if (!fs::exists(p)) {continue;}
    for (fs::recursive_directory_iterator it(p), end; it != end; ++it) {
      if (should_skip_path(it->path())) {
        it.no_push();
        continue;
      }
      if (fs::is_regular_file(it->path()) && to_lower(it->path().extension().string()) == ".stl") {
        meshes.insert(normalize_mesh_path(it->path().string(), workspace_root, repo_root));
      }
    }
  }
  report.stl_meshes.assign(meshes.begin(), meshes.end());

  for (const auto & root : report.robot_paths) {
    scan_asset_packages(root, true, report.robots);
  }
  for (const auto & root : report.end_effector_paths) {
    scan_asset_packages(root, false, report.end_effectors);
  }

  if (report.robots.empty()) {
    report.robots.push_back({"UR5", "ur_description", "ur5_moveit_config", "ur.urdf.xacro", "unknown", "READY"});
  }
  if (report.end_effectors.empty()) {
    report.end_effectors.push_back({"robotiq_2f_85", "robotiq_85_description", "robotiq_85_moveit_config", "robotiq_85_gripper.urdf.xacro", "finger", "READY"});
    report.end_effectors.push_back({"airpick", "onrobot_airpick4_description", "onrobot_airpick4_moveit_config", "onrobot_airpick4_gripper.urdf.xacro", "suction", "READY"});
  }
  return report;
}
