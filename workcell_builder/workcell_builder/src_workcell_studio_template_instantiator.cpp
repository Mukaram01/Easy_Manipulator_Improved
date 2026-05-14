#include "workcell_studio_template_instantiator.hpp"

#include <cctype>
#include <fstream>

#include "file_functions.h"

namespace fs = boost::filesystem;
namespace workcell_builder {
namespace {

std::string sanitize(const std::string & raw)
{
  std::string out;
  for (const char c : raw) {
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    else if (std::isspace(static_cast<unsigned char>(c))) out.push_back('_');
  }
  return out.empty() ? "new_scene" : out;
}

void write_file(const fs::path & path, const std::string & text, WorkcellStudioTemplateInstantiationResult & r)
{
  fs::create_directories(path.parent_path());
  std::ofstream out(path.string());
  out << text;
  if (out.good()) r.created_files.push_back(path.string());
  else r.blockers.push_back("failed writing: " + path.string());
}

}

WorkcellStudioTemplateInstantiationResult instantiate_workcell_studio_template(const WorkcellStudioTemplateInstantiationRequest & request)
{
  WorkcellStudioTemplateInstantiationResult r;
  const std::string scene_name = sanitize(request.scene_name);
  if (request.template_id.empty() || request.robot_id.empty() || request.end_effector_id.empty()) {
    r.blockers.push_back("template/robot/end_effector must be selected");
    return r;
  }

  fs::path scene_dir = request.scene_root / scene_name;
  int suffix = 2;
  while (fs::exists(scene_dir)) scene_dir = request.scene_root / (scene_name + "_" + std::to_string(suffix++));
  if (scene_dir.filename().string() != scene_name) r.warnings.push_back("scene existed; created suffixed scene: " + scene_dir.filename().string());
  r.scene_dir = scene_dir;

  generate_package_xml(scene_dir, scene_dir.filename().string(), "humble");
  generate_cmake(scene_dir, scene_dir.filename().string(), "humble");
  r.created_files.push_back((scene_dir / "package.xml").string());
  r.created_files.push_back((scene_dir / "CMakeLists.txt").string());

  const bool suction = request.end_effector_id.find("suction") != std::string::npos || request.end_effector_id.find("airpick") != std::string::npos;
  const bool preview_only = request.robot_id.find("ur5") == std::string::npos;
  const std::string grasp = suction ? "suction_top" : "finger_top";
  const std::string task_type = request.template_id.find("inspection") != std::string::npos ? "inspection_preview" : "pick_place";

  write_file(scene_dir / "environment.yaml",
    "robot:\n  name: " + request.robot_id + "\nend_effector:\n  name: " + request.end_effector_id +
    "\n  mount_rpy: \"-1.5708 -1.5708 0\"\n"
    "task:\n  type: " + task_type + "\n  preview_only: " + std::string(preview_only ? "true" : "false") +
    "\nsafety:\n  fake_hardware_first: true\n  runtime_execution_enabled: false\n  motion_command_sent: false\n"
    "layout:\n  preset: " + request.layout_preset_id + "\n  include_camera: " + std::string(request.include_camera ? "true" : "false") + "\n  include_conveyor: " + std::string(request.include_conveyor ? "true" : "false") + "\n",
    r);
  write_file(scene_dir / "scene_manifest.yaml", "schema: workcell_scene_manifest/v1\nscene_name: " + scene_dir.filename().string() + "\ntemplate_id: " + request.template_id + "\n", r);
  write_file(scene_dir / "config" / "task_recipe.yaml", "task:\n  type: " + task_type + "\n  grasp_strategy: " + grasp + "\n", r);
  write_file(scene_dir / "config" / "workcell_builder_task_intent.yaml",
    "schema: workcell_builder_task_intent/v1\nsafety:\n  fake_hardware_first: true\n  runtime_execution_enabled: false\n  motion_command_sent: false\n",
    r);
  write_file(scene_dir / "launch" / "demo.launch.py", "# generated launch placeholder\n# always use fake hardware\n", r);
  write_file(scene_dir / "GENERATED_SCENE_SUMMARY.md", "# Generated Scene\n\nNo robot motion commanded.\n", r);
  write_file(scene_dir / "preview" / "static_preview.html", "<html><body><h1>Static Preview</h1><p>No robot motion commanded.</p></body></html>\n", r);
  write_file(scene_dir / "smoke" / "offline_smoke_summary.txt", "status=PASS\nno_robot_motion_commanded=true\n", r);
  write_file(scene_dir / "smoke" / "offline_smoke_report.json", "{\"status\":\"PASS\"}\n", r);
  write_file(scene_dir / "smoke" / "offline_smoke_report.html", "<html><body>PASS</body></html>\n", r);

  r.status = r.blockers.empty() ? (preview_only ? "PREVIEW_ONLY" : "READY") : "BLOCKED";
  r.success = r.blockers.empty();
  r.next_commands.push_back("colcon build --symlink-install --packages-select " + scene_dir.filename().string());
  r.next_commands.push_back("ros2 launch " + scene_dir.filename().string() + " demo.launch.py use_fake_hardware:=true");
  return r;
}

}  // namespace workcell_builder
