#include "offline_smoke_check_model.hpp"

#include <fstream>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;
namespace workcell_builder
{
namespace
{
void add_check(OfflineSmokeCheckResult & r, const std::string & id, const std::string & category, const std::string & status,
               const std::string & message, const std::string & fix_hint = "", const fs::path & artifact = fs::path())
{
  r.checks.push_back({id, category, status, message, fix_hint, artifact.string()});
}
}

std::string offline_smoke_status_label(OfflineSmokeStatus status)
{
  switch (status) {
    case OfflineSmokeStatus::PASS: return "PASS";
    case OfflineSmokeStatus::WARNINGS: return "WARNINGS";
    case OfflineSmokeStatus::BLOCKED: return "BLOCKED";
    case OfflineSmokeStatus::PREVIEW_ONLY: return "PREVIEW_ONLY";
  }
  return "BLOCKED";
}

OfflineSmokeCheckResult run_offline_smoke_check(const Scene & scene, const fs::path & scene_dir)
{
  OfflineSmokeCheckResult r;
  r.scene_name = scene.name;
  r.scene_dir = scene_dir.string();
  const fs::path yaml_path = scene_dir / "environment.yaml";
  const bool scene_exists = fs::exists(scene_dir);
  add_check(r, "scene_dir_exists", "scene_yaml", scene_exists ? "PASS" : "BLOCKED", scene_exists ? "Scene directory exists" : "Missing scene directory", "Select or regenerate scene", scene_dir);
  if (!scene_exists) r.blockers.push_back("scene directory missing");

  const bool yaml_exists = fs::exists(yaml_path);
  add_check(r, "environment_yaml_exists", "scene_yaml", yaml_exists ? "PASS" : "BLOCKED", yaml_exists ? "environment.yaml found" : "environment.yaml missing", "Generate canonical files", yaml_path);
  if (!yaml_exists) r.blockers.push_back("environment.yaml missing");

  bool preview_only = false;
  if (yaml_exists) {
    try { auto n = YAML::LoadFile(yaml_path.string()); preview_only = (n["task"] && n["task"]["preview_only"] && n["task"]["preview_only"].as<bool>()); add_check(r, "environment_yaml_parse", "scene_yaml", "PASS", "environment.yaml parsed"); }
    catch (...) { add_check(r, "environment_yaml_parse", "scene_yaml", "BLOCKED", "environment.yaml parse failed", "Fix YAML syntax", yaml_path); r.blockers.push_back("environment.yaml parse failed"); }
  }

  const bool task_recipe = fs::exists(scene_dir / "config" / "task_recipe.yaml");
  add_check(r, "task_recipe", "task_grasp", task_recipe ? "PASS" : "WARNINGS", task_recipe ? "task_recipe.yaml exists" : "task_recipe.yaml missing (can be generated)", "Generate task recipe", scene_dir / "config" / "task_recipe.yaml");
  if (!task_recipe) r.warnings.push_back("task_recipe.yaml missing");
  const bool intent_yaml = fs::exists(scene_dir / "config" / "workcell_builder_task_intent.yaml");
  add_check(r, "task_intent", "task_grasp", intent_yaml ? "PASS" : "WARNINGS", intent_yaml ? "workcell_builder_task_intent.yaml exists" : "task intent yaml missing (can be generated)");

  const bool package_files = fs::exists(scene_dir / "package.xml") && fs::exists(scene_dir / "CMakeLists.txt") && fs::exists(scene_dir / "launch" / "demo.launch.py");
  add_check(r, "package_files", "package_files", package_files ? "PASS" : "BLOCKED", package_files ? "Generated package files found" : "Missing package files", "Generate full scene package");
  if (!package_files) r.blockers.push_back("package files missing");

  const bool rviz_optional = fs::exists(scene_dir / "demo.rviz");
  add_check(r, "demo_rviz_optional", "preview_artifacts", rviz_optional ? "PASS" : "WARNINGS", rviz_optional ? "demo.rviz found" : "demo.rviz missing (optional)");
  if (!rviz_optional) r.warnings.push_back("demo.rviz optional file missing");

  r.safety_flags = {
    {"fake_hardware_first", true},
    {"runtime_execution_enabled", false},
    {"motion_command_sent", false},
    {"moveit_service_called", false},
    {"real_hardware_default", false}
  };
  add_check(r, "safety_flags", "safety_flags", "PASS", "No robot motion / no live MoveIt service / fake hardware first.");

  r.build_command = "colcon build --symlink-install --packages-select " + scene.name;
  r.launch_command = "ros2 launch " + scene.name + " demo.launch.py use_fake_hardware:=true";
  add_check(r, "launch_command", "build_launch_commands", "PASS", r.launch_command);

  const fs::path preview = scene_dir / "preview" / "layout_preview.svg";
  add_check(r, "preview_artifact", "preview_artifacts", fs::exists(preview) ? "PASS" : "WARNINGS", fs::exists(preview) ? "preview artifact found" : "preview artifact missing (can be generated)", "Export preview", preview);

  r.status = !r.blockers.empty() ? OfflineSmokeStatus::BLOCKED : (!r.warnings.empty() ? OfflineSmokeStatus::WARNINGS : OfflineSmokeStatus::PASS);
  if (preview_only) r.status = OfflineSmokeStatus::PREVIEW_ONLY;
  r.next_action = r.status == OfflineSmokeStatus::BLOCKED ? "Fix blockers and re-run offline smoke check." : "Proceed to build and fake-hardware launch preview.";
  return r;
}

bool write_offline_smoke_report(const OfflineSmokeCheckResult & r, std::string * error)
{
  const fs::path smoke_dir = fs::path(r.scene_dir) / "smoke";
  boost::system::error_code ec;
  fs::create_directories(smoke_dir, ec);
  if (ec) { if (error) *error = ec.message(); return false; }
  std::ofstream j(smoke_dir / "offline_smoke_report.json");
  j << "{\n  \"scene_name\": \"" << r.scene_name << "\",\n  \"scene_path\": \"" << r.scene_dir << "\",\n  \"status\": \"" << offline_smoke_status_label(r.status) << "\",\n  \"next_action\": \"" << r.next_action << "\"\n}\n";
  std::ofstream h(smoke_dir / "offline_smoke_report.html");
  h << "<html><body><h1>Offline Smoke Report</h1><p>Status: " << offline_smoke_status_label(r.status) << "</p><p>Safety banner: no robot motion.</p></body></html>";
  std::ofstream t(smoke_dir / "offline_smoke_summary.txt");
  t << "scene=" << r.scene_name << "\nstatus=" << offline_smoke_status_label(r.status) << "\nblockers=" << r.blockers.size() << "\nwarnings=" << r.warnings.size() << "\nnext=" << r.next_action << "\n";
  return true;
}

}  // namespace workcell_builder
