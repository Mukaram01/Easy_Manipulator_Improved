#include "workcell_studio_scene_browser.hpp"
#include "workcell_yaml_utils.hpp"
#include "workcell_warning_once.hpp"

#include <yaml-cpp/yaml.h>

#include <set>
#include <cstdlib>
#include <iostream>

namespace fs = boost::filesystem;

namespace workcell_builder {

namespace {
bool exists_file(const fs::path & p){ boost::system::error_code ec; return fs::exists(p, ec) && !ec; }

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

bool is_directory(const fs::path & p)
{
  boost::system::error_code ec;
  return fs::exists(p, ec) && !ec && fs::is_directory(p, ec) && !ec;
}

fs::path normalize_workspace_root(const fs::path & input)
{
  const fs::path absolute = canonical_or_absolute(input);

  const auto looks_like_workspace = [](const fs::path & root) {
    return is_directory(root / "src" / "easy_manipulation_deployment" / "scenes");
  };

  if (looks_like_workspace(absolute)) return absolute;

  // Some call sites historically handed discovery either <workspace>/src or
  // the repository checkout itself.  Normalize both forms back to the workspace
  // before generating candidate roots so a refresh cannot drift onto src/scenes.
  if (absolute.filename() == "src" && looks_like_workspace(absolute.parent_path())) {
    return absolute.parent_path();
  }
  if (absolute.filename() == "easy_manipulation_deployment" &&
      absolute.parent_path().filename() == "src" &&
      looks_like_workspace(absolute.parent_path().parent_path())) {
    return absolute.parent_path().parent_path();
  }

  fs::path cursor = absolute;
  for (int depth = 0; depth < 4 && !cursor.empty(); ++depth) {
    if (looks_like_workspace(cursor)) return cursor;
    if (cursor == cursor.parent_path()) break;
    cursor = cursor.parent_path();
  }
  return absolute;
}

bool is_valid_scene_package(const WorkcellStudioSceneInfo & s)
{
  return s.has_environment_yaml || s.has_scene_manifest_yaml ||
         (s.has_package_xml && s.has_launch_demo) || s.has_scene_urdf_xacro;
}

std::vector<fs::path> candidate_scene_roots(const fs::path & workspace_root)
{
  const fs::path normalized_workspace = normalize_workspace_root(workspace_root);
  const fs::path canonical_repo_scenes =
    normalized_workspace / "src" / "easy_manipulation_deployment" / "scenes";

  // The repository scene tree is the authoring source of truth.  If it exists,
  // do not allow the compatibility alias at <workspace>/src/scenes to become a
  // competing root after Generate/Validate/Plan refreshes.
  if (is_directory(canonical_repo_scenes)) {
    return {canonical_or_absolute(canonical_repo_scenes)};
  }

  std::vector<fs::path> roots;
  roots.push_back(normalized_workspace / "src" / "scenes");
  roots.push_back(fs::current_path() / "scenes");
  roots.push_back(normalized_workspace / "scenes");
  roots.push_back(fs::path(getenv("HOME") ? getenv("HOME") : "") / "scenes");

  std::set<std::string> seen;
  std::vector<fs::path> uniq;
  for (const auto & r : roots) {
    const fs::path canonical = canonical_or_absolute(r);
    if (seen.insert(canonical.string()).second) uniq.push_back(canonical);
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
  if (s.parse_warning.empty() && s.has_package_xml && s.has_scene_urdf_xacro && s.has_arm_hand_srdf_xacro && s.has_task_recipe && s.has_smoke_report_json) return "READY";
  if (!s.parse_warning.empty() || !s.has_task_recipe || !s.has_smoke_report_json) return "WARNINGS";
  return "BLOCKED";
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
  out.scene_root = normalize_workspace_root(workspace_root);
  const auto roots = candidate_scene_roots(workspace_root);
  out.searched_roots = roots;

  boost::system::error_code ec;
  std::set<std::string> discovered_scene_dirs;
  for (const auto & candidate_root : roots) {
    const fs::path scene_root = canonical_or_absolute(candidate_root);
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
      s.scene_dir = canonical_scene_identity(it->path());
      s.canonical_scene_dir = s.scene_dir;
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
      s.has_static_preview_svg = exists_file(s.scene_dir / "preview" / "static_preview.svg");
      s.has_static_preview_html = exists_file(s.scene_dir / "preview" / "static_preview.html");
      s.has_scene_manifest_yaml = exists_file(s.scene_dir / "scene_manifest.yaml");
      if (s.has_scene_manifest_yaml) try_parse_manifest_launch_metadata(&s);
      if (!is_valid_scene_package(s)) {
        continue;
      }
      if (s.has_environment_yaml) try_parse_env(&s);
      s.status = compute_status(s);
      out.scenes.push_back(s);
    }
    break;
  }
  return out;
}

}  // namespace workcell_builder
