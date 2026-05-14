#include "workcell_studio_scene_browser.hpp"

#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;

namespace workcell_builder {

namespace {
bool exists_file(const fs::path & p){ boost::system::error_code ec; return fs::exists(p, ec) && !ec; }

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

void try_parse_env(WorkcellStudioSceneInfo * s)
{
  try {
    YAML::Node n = YAML::LoadFile((s->scene_dir / "environment.yaml").string());
    if (n["robot"] && n["robot"]["name"]) s->robot_summary = n["robot"]["name"].as<std::string>();
    if (n["end_effector"] && n["end_effector"]["name"]) s->gripper_summary = n["end_effector"]["name"].as<std::string>();
    if (n["object"] && n["object"].IsSequence()) s->object_count = n["object"].size();
  } catch (const std::exception &) {
    s->parse_warning = "Could not parse environment.yaml";
  }
}
}

WorkcellStudioSceneBrowserResult discover_workcell_studio_scenes(const fs::path & scene_root)
{
  WorkcellStudioSceneBrowserResult out;
  out.scene_root = scene_root;
  boost::system::error_code ec;
  out.root_exists = fs::exists(scene_root, ec) && !ec && fs::is_directory(scene_root, ec) && !ec;
  if (!out.root_exists) return out;

  for (fs::directory_iterator it(scene_root, ec), end; it != end && !ec; it.increment(ec)) {
    if (!fs::is_directory(it->path(), ec) || ec) continue;
    WorkcellStudioSceneInfo s;
    s.scene_dir = it->path();
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
    if (s.has_environment_yaml) try_parse_env(&s);
    s.status = compute_status(s);
    out.scenes.push_back(s);
  }
  return out;
}

}  // namespace workcell_builder
