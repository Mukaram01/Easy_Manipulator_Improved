#include "workcell_studio_scene_browser.hpp"
#include "workcell_studio_canvas_model.hpp"
#include "workcell_yaml_utils.hpp"
#include "workcell_warning_once.hpp"

#include <yaml-cpp/yaml.h>

#include <set>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fs = boost::filesystem;

namespace workcell_builder {

namespace {
bool exists_file(const fs::path & p){ boost::system::error_code ec; return fs::exists(p, ec) && !ec; }

std::string authored_input_fingerprint(const fs::path & scene_dir)
{
  std::vector<fs::path> relative_paths;
  for (const char * relative : {
      "package.xml", "CMakeLists.txt", "environment.yaml", "environment_layout.yaml",
      "cell_definition.yaml", "scene_manifest.yaml", "layout/workcell_studio_layout.yaml"})
    relative_paths.emplace_back(relative);
  for (const char * relative : {"config", "launch", "urdf", "assets"}) {
    const fs::path root = scene_dir / relative;
    boost::system::error_code ec;
    if (!fs::is_directory(root, ec) || ec) continue;
    for (fs::recursive_directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec)) {
      if (fs::is_regular_file(it->path(), ec) && !ec)
        relative_paths.push_back(fs::relative(it->path(), scene_dir));
    }
  }
  std::sort(relative_paths.begin(), relative_paths.end(),
    [](const fs::path & lhs, const fs::path & rhs) { return lhs.generic_string() < rhs.generic_string(); });
  relative_paths.erase(std::unique(relative_paths.begin(), relative_paths.end()), relative_paths.end());
  std::uint64_t digest = 0xcbf29ce484222325ULL;
  const auto update = [&digest](const char * data, std::size_t size) {
    for (std::size_t index = 0; index < size; ++index) {
      digest ^= static_cast<unsigned char>(data[index]);
      digest *= 0x100000001b3ULL;
    }
  };
  for (const auto & relative : relative_paths) {
    const fs::path path = scene_dir / relative;
    if (!exists_file(path)) continue;
    const std::string relative_bytes = relative.generic_string();
    update(relative_bytes.data(), relative_bytes.size());
    update("\0", 1);
    std::ifstream file(path.string(), std::ios::binary);
    char buffer[8192];
    while (file.read(buffer, sizeof(buffer)) || file.gcount() > 0)
      update(buffer, static_cast<std::size_t>(file.gcount()));
    update("\0", 1);
  }
  std::ostringstream encoded;
  encoded << std::hex << std::setfill('0') << std::setw(16) << digest;
  return encoded.str();
}

std::filesystem::file_time_type latest_authored_input_time(const fs::path & scene_dir)
{
  auto latest = std::filesystem::file_time_type::min();
  const auto include_file = [&](const fs::path & path) {
    std::error_code ec;
    const std::filesystem::path std_path(path.string());
    if (std::filesystem::is_regular_file(std_path, ec) && !ec)
      latest = std::max(latest, std::filesystem::last_write_time(std_path, ec));
  };
  for (const char * relative : {
      "package.xml", "CMakeLists.txt", "environment.yaml", "environment_layout.yaml",
      "cell_definition.yaml", "scene_manifest.yaml", "layout/workcell_studio_layout.yaml"})
    include_file(scene_dir / relative);
  for (const char * relative : {"config", "launch", "urdf", "assets"}) {
    const fs::path root = scene_dir / relative;
    boost::system::error_code ec;
    if (!fs::is_directory(root, ec) || ec) continue;
    for (fs::recursive_directory_iterator it(root, ec), end; it != end && !ec; it.increment(ec))
      include_file(it->path());
  }
  return latest;
}

void inspect_acceptance_report(WorkcellStudioSceneInfo * s)
{
  const fs::path report = s->scene_dir / "acceptance/generated_scene_acceptance.json";
  s->has_acceptance_report_json = exists_file(report);
  if (!s->has_acceptance_report_json) return;
  try {
    const YAML::Node root = YAML::LoadFile(report.string());
    s->acceptance_status = root["status"].as<std::string>("");
    const std::string report_scene = root["scene_name"].as<std::string>("");
    const YAML::Node safety = root["safety_flags"];
    const bool safe = safety && safety.IsMap() &&
      safety["fake_hardware_first"].as<bool>(false) &&
      !safety["runtime_execution_enabled"].as<bool>(true) &&
      !safety["motion_command_sent"].as<bool>(true);
    const std::string accepted_fingerprint = root["authored_input_fingerprint"].as<std::string>("");
    if (!accepted_fingerprint.empty()) {
      s->acceptance_report_current = accepted_fingerprint == authored_input_fingerprint(s->scene_dir);
    } else {
      std::error_code ec;
      const auto report_time = std::filesystem::last_write_time(
        std::filesystem::path(report.string()), ec);
      s->acceptance_report_current = !ec && report_time >= latest_authored_input_time(s->scene_dir);
    }
    s->acceptance_report_passed = s->acceptance_status == "PASS" &&
      (report_scene.empty() || report_scene == s->scene_name) && safe;
  } catch (const std::exception &) {
    s->acceptance_status = "INVALID";
  }
}

fs::path canonical_or_absolute(const fs::path & p)
{
  boost::system::error_code ec;
  const fs::path canonical_path = fs::canonical(p, ec);
  if (!ec) {
    return canonical_path;
  }
  ec.clear();
  const fs::path weak = fs::weakly_canonical(p, ec);
  if (!ec) {
    return weak;
  }
  return fs::absolute(p);
}

bool is_valid_scene_package(const WorkcellStudioSceneInfo & s)
{
  return s.has_environment_yaml || s.has_scene_manifest_yaml ||
         (s.has_package_xml && s.has_launch_demo) || s.has_scene_urdf_xacro;
}

std::vector<fs::path> candidate_scene_roots(const fs::path & workspace_root)
{
  std::vector<fs::path> roots;
  roots.push_back(workspace_root / "src" / "easy_manipulation_deployment" / "scenes");
  roots.push_back(workspace_root / "src" / "scenes");
  boost::system::error_code ec;
  const fs::path emd = workspace_root / "src" / "easy_manipulation_deployment" / "scenes";
  const fs::path emd_canonical = fs::weakly_canonical(emd, ec);
  if (!ec) roots.push_back(emd_canonical);
  roots.push_back(fs::current_path() / "scenes");
  roots.push_back(workspace_root / "scenes");
  roots.push_back(fs::path(getenv("HOME") ? getenv("HOME") : "") / "scenes");
  std::set<std::string> seen;
  std::vector<fs::path> uniq;
  for (const auto & r : roots) {
    std::string k = canonical_or_absolute(r).string();
    if (seen.insert(k).second) uniq.push_back(r);
  }
  return uniq;
}

std::string compute_status(const WorkcellStudioSceneInfo & s)
{
  if (!s.has_environment_yaml) return "MISSING_ENVIRONMENT_YAML";
  if (!s.has_launch_demo && (s.has_static_preview_svg || s.has_static_preview_html)) return "PREVIEW_ONLY";
  if (!s.has_launch_demo) return "MISSING_LAUNCH";
  const bool scaffold = s.has_environment_yaml && !s.has_package_xml && !s.has_scene_urdf_xacro && !s.has_arm_hand_srdf_xacro;
  if (scaffold) return "SCAFFOLD_ONLY";
  if (s.parse_warning.empty() && s.has_package_xml && s.has_scene_urdf_xacro && s.has_arm_hand_srdf_xacro && s.has_task_recipe && s.acceptance_report_current && s.acceptance_report_passed) return "READY";
  if (s.acceptance_status == "BLOCKED") return "BLOCKED";
  if (!s.parse_warning.empty() || !s.has_task_recipe || !s.acceptance_report_current || !s.acceptance_report_passed) return "WARNINGS";
  return "BLOCKED";
}

void populate_readiness(WorkcellStudioSceneInfo * s)
{
  if (s == nullptr) return;
  s->readiness_reasons.clear();
  if (!s->has_environment_yaml) s->readiness_reasons.push_back("Missing environment.yaml");
  if (!s->has_launch_demo) s->readiness_reasons.push_back("Generate the scene package to create launch/demo.launch.py");
  if (!s->has_package_xml || !s->has_scene_urdf_xacro)
    s->readiness_reasons.push_back("Scene package outputs are incomplete");
  if (s->status == "WARNINGS") {
    if (!s->has_task_recipe && !s->has_task_intent) s->readiness_reasons.push_back("Task intent is not configured");
    if (!s->has_acceptance_report_json) s->readiness_reasons.push_back("Run validation to create a current offline report");
    else if (!s->acceptance_report_current) s->readiness_reasons.push_back("Scene changed since validation; run validation again");
    else if (!s->acceptance_report_passed) s->readiness_reasons.push_back("Offline validation did not pass");
    if (!s->parse_warning.empty()) s->readiness_reasons.push_back("Scene metadata contains a parse warning");
  }
  if (s->readiness_reasons.size() > 3U) s->readiness_reasons.resize(3U);
  s->fake_hardware_ready = s->status == "READY" && s->has_launch_demo && s->acceptance_report_passed;
}


std::string scalar_string(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) return "";
  return node.as<std::string>("");
}

void try_parse_manifest_launch_metadata(WorkcellStudioSceneInfo * s)
{
  if (s == nullptr || !s->has_scene_manifest_yaml) return;
  const fs::path manifest_yaml = s->scene_dir / "scene_manifest.yaml";
  try {
    const YAML::Node n = YAML::LoadFile(manifest_yaml.string());
    const std::string package_name = scalar_string(n["scene"]["package"]);
    const std::string launch_path = scalar_string(n["files"]["launch"]);
    if (!package_name.empty()) {
      s->launch_package = package_name;
      s->launch_metadata_present = true;
    }
    if (!launch_path.empty()) {
      const fs::path manifest_launch_path(launch_path);
      s->launch_file = manifest_launch_path.filename().string();
      s->launch_metadata_present = true;
      const fs::path resolved = manifest_launch_path.is_absolute() ? manifest_launch_path : (s->scene_dir / manifest_launch_path);
      s->launch_metadata_file_exists = exists_file(resolved);
      if (!s->launch_metadata_file_exists) {
        s->launch_metadata_warning = "scene_manifest.yaml launch file missing: " + resolved.string();
      }
    }
  } catch (const std::exception & e) {
    const std::string msg = std::string("Could not parse scene_manifest.yaml launch metadata: ") + e.what() +
      " (file: " + manifest_yaml.string() + ")";
    if (s->parse_warning.empty()) {
      s->parse_warning = msg;
    } else {
      s->parse_warning += "; " + msg;
    }
    log_warning_once_per_context_path_reason("scene_browser_load", manifest_yaml, "scene manifest YAML parse failed");
  }
}

void try_parse_env(WorkcellStudioSceneInfo * s)
{
  if (s == nullptr) return;
  const fs::path environment_yaml = s->scene_dir / "environment.yaml";
  try {
    YAML::Node n = YAML::LoadFile(environment_yaml.string());

    const std::string robot = yaml_name_from_node(n["robot"]);

    std::string gripper = yaml_name_from_node(n["end_effector"]);
    if (gripper.empty()) {
      // Legacy alias support.
      gripper = yaml_name_from_node(n["gripper"]);
    }

    std::size_t object_count = 0U;
    const YAML::Node object = n["object"];
    if (object && object.IsSequence()) object_count = object.size();
    if (object_count == 0U) {
      const YAML::Node objects = n["objects"];
      if (objects && objects.IsSequence()) object_count = objects.size();
    }

    if (!robot.empty()) s->robot_summary = robot;
    if (!gripper.empty()) s->gripper_summary = gripper;
    s->object_count = object_count;
  } catch (const std::exception & e) {
    s->parse_warning = std::string("Could not parse environment.yaml: ") + e.what() +
      " (file: " + environment_yaml.string() + ")";
    log_warning_once_per_context_path_reason("scene_browser_load", environment_yaml, "scene YAML parse failed");
  }
}
}

fs::path canonical_scene_identity(const fs::path & scene_dir)
{
  return canonical_or_absolute(scene_dir);
}

bool same_scene_identity(const WorkcellStudioSceneInfo & scene, const fs::path & scene_dir)
{
  const fs::path candidate_identity = scene.canonical_scene_dir.empty() ?
    canonical_scene_identity(scene.scene_dir) : canonical_scene_identity(scene.canonical_scene_dir);
  return candidate_identity == canonical_scene_identity(scene_dir);
}

int find_scene_by_identity(
  const WorkcellStudioSceneBrowserResult & scenes,
  const fs::path & scene_dir,
  const std::string & stable_scene_name)
{
  for (std::size_t i = 0; i < scenes.scenes.size(); ++i) {
    if (same_scene_identity(scenes.scenes[i], scene_dir)) return static_cast<int>(i);
  }
  if (!stable_scene_name.empty()) {
    int unique_match = -1;
    for (std::size_t i = 0; i < scenes.scenes.size(); ++i) {
      if (scenes.scenes[i].scene_name != stable_scene_name) continue;
      if (unique_match >= 0) return -1;  // Display/package names are not safe when ambiguous.
      unique_match = static_cast<int>(i);
    }
    return unique_match;
  }
  return -1;
}

WorkcellStudioSceneBrowserResult discover_workcell_studio_scenes(const fs::path & workspace_root)
{
  WorkcellStudioSceneBrowserResult out;
  out.scene_root = workspace_root;
  const auto roots = candidate_scene_roots(workspace_root);
  out.searched_roots = roots;

  boost::system::error_code ec;
  std::set<std::string> discovered_scene_dirs;
  for (const auto & scene_root : roots) {
    const bool exists = fs::exists(scene_root, ec) && !ec && fs::is_directory(scene_root, ec) && !ec;
    if (!exists) {
      ec.clear();
      continue;
    }

    out.scene_root = scene_root;
    out.root_exists = true;
    for (fs::directory_iterator it(scene_root, ec), end; it != end && !ec; it.increment(ec)) {
      if (!fs::is_directory(it->path(), ec) || ec) continue;
      WorkcellStudioSceneInfo s;
      s.scene_dir = it->path();
      s.canonical_scene_dir = canonical_scene_identity(s.scene_dir);
      const std::string scene_key = s.canonical_scene_dir.string();
      if (!discovered_scene_dirs.insert(scene_key).second) {
        continue;
      }
      s.scene_name = s.scene_dir.filename().string();
      s.has_environment_yaml = exists_file(s.scene_dir / "environment.yaml");
      s.has_package_xml = exists_file(s.scene_dir / "package.xml");
      s.has_launch_demo = exists_file(s.scene_dir / "launch" / "demo.launch.py");
      s.has_scene_urdf_xacro = exists_file(s.scene_dir / "urdf" / "scene.urdf.xacro") || exists_file(s.scene_dir / "urdf" / "environment.urdf.xacro");
      s.has_arm_hand_srdf_xacro = exists_file(s.scene_dir / "urdf" / "arm_hand.srdf.xacro");
      s.has_task_recipe = exists_file(s.scene_dir / "config" / "task_recipe.yaml");
      s.has_task_intent = exists_file(s.scene_dir / "config" / "workcell_builder_task_intent.yaml");
      s.has_smoke_report_json = exists_file(s.scene_dir / "smoke" / "offline_smoke_report.json");
      s.has_smoke_report_html = exists_file(s.scene_dir / "smoke" / "offline_smoke_report.html");
      inspect_acceptance_report(&s);
      s.has_static_preview_svg = exists_file(s.scene_dir / "preview" / "static_preview.svg");
      s.has_static_preview_html = exists_file(s.scene_dir / "preview" / "static_preview.html");
      s.has_scene_manifest_yaml = exists_file(s.scene_dir / "scene_manifest.yaml");
      if (s.has_scene_manifest_yaml) try_parse_manifest_launch_metadata(&s);
      if (!is_valid_scene_package(s)) {
        continue;
      }
      if (s.has_environment_yaml) try_parse_env(&s);
      const WorkcellStudioSceneMetadataSummary metadata =
        load_workcell_studio_scene_metadata_summary(s.scene_dir, s.scene_name);
      s.display_name = metadata.display_name;
      if (!metadata.robot.empty()) s.robot_summary = metadata.robot;
      if (!metadata.tool.empty()) s.gripper_summary = metadata.tool;
      s.task_summary = metadata.task;
      s.metadata_revision = metadata.revision;
      if (metadata.has_parse_warning && s.parse_warning.empty())
        s.parse_warning = "Canonical scene metadata contains invalid YAML";
      s.status = compute_status(s);
      populate_readiness(&s);
      out.scenes.push_back(s);
    }
    break;
  }
  return out;
}

}  // namespace workcell_builder
