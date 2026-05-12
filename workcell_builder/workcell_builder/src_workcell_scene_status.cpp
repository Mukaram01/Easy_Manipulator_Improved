#include "workcell_scene_status.hpp"

#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;

namespace workcell_builder
{
namespace
{
void add_item(SceneStatusReport & report, const std::string & name, const std::string & status, const std::string & message, const fs::path & path = fs::path())
{
  report.items.push_back({name, status, message, path.string()});
}

bool has_generated_files(const fs::path & scene_dir)
{
  return fs::exists(scene_dir / "launch" / "demo.launch.py") &&
         fs::exists(scene_dir / "package.xml") &&
         fs::exists(scene_dir / "CMakeLists.txt");
}
}

SceneStatusReport inspect_scene_status(
  const fs::path & /*workcell_path*/,
  const fs::path & scenes_path,
  const fs::path & assets_path,
  const std::string & scene_name)
{
  SceneStatusReport report;
  report.scene_name = scene_name;
  const fs::path scene_dir = scenes_path / scene_name;
  report.scene_path = scene_dir.string();

  report.safety_notes = {
    "Offline/fake-hardware readiness only",
    "No robot motion is commanded by validation",
    "Generated scene is not a safety certificate"
  };

  add_item(report, "Selected scene", "INFO", scene_name, scene_dir);
  add_item(report, "Scene path", fs::exists(scene_dir) ? "OK" : "ERROR", fs::exists(scene_dir) ? "Scene directory found" : "Scene directory missing", scene_dir);

  const fs::path environment_yaml = scene_dir / "environment.yaml";
  report.environment_yaml_ok = fs::exists(environment_yaml);
  add_item(report, "environment.yaml", report.environment_yaml_ok ? "OK" : "ERROR", report.environment_yaml_ok ? "Found" : "Missing", environment_yaml);
  if (!report.environment_yaml_ok) report.blockers.push_back("missing environment.yaml");

  const fs::path manifest = scene_dir / "scene_manifest.yaml";
  add_item(report, "scene_manifest.yaml", fs::exists(manifest) ? "OK" : "WARN", fs::exists(manifest) ? "Found" : "Missing", manifest);

  report.generated_files_ok = has_generated_files(scene_dir);
  add_item(report, "generated package files", report.generated_files_ok ? "OK" : "ERROR", report.generated_files_ok ? "launch/package/cmake files found" : "Missing generated files", scene_dir / "launch");
  if (!report.generated_files_ok) report.blockers.push_back("missing generated files");

  const fs::path launch_file = scene_dir / "launch" / "demo.launch.py";
  report.launch_file_ok = fs::exists(launch_file);
  if (!report.launch_file_ok) report.blockers.push_back("missing launch file");

  std::string robot_pkg = "<unknown>";
  std::string ee_pkg = "<none>";
  if (report.environment_yaml_ok) {
    try {
      const YAML::Node root = YAML::LoadFile(environment_yaml.string());
      if (root["robot"] && root["robot"]["name"]) robot_pkg = root["robot"]["name"].as<std::string>();
      if (root["endeffector"] && root["endeffector"]["name"]) ee_pkg = root["endeffector"]["name"].as<std::string>();
      if (root["object"]) {
        for (const auto & obj : root["object"]) {
          if (obj["name"]) {
            const std::string name = obj["name"].as<std::string>();
            const fs::path asset_dir = assets_path / "environment" / (name + "_description");
            const bool ok = fs::exists(asset_dir);
            add_item(report, "environment asset: " + name, ok ? "OK" : "WARN", ok ? "Bundled/generated asset present" : "Imported bundle dependency missing", asset_dir);
            if (!ok) report.warnings.push_back("missing environment asset YAML");
          }
        }
      }
    } catch (...) {
      report.warnings.push_back("environment.yaml parse warning");
      add_item(report, "environment.yaml parse", "WARN", "Could not fully parse robot/tool/object dependencies", environment_yaml);
    }
  }

  add_item(report, "robot package", robot_pkg != "<unknown>" ? "OK" : "ERROR", robot_pkg);
  if (robot_pkg == "<unknown>") report.blockers.push_back("missing robot package");
  add_item(report, "end-effector package", ee_pkg != "<none>" ? "OK" : "WARN", ee_pkg);
  if (ee_pkg == "<none>") report.warnings.push_back("missing end-effector package");

  report.fake_hardware_default_ok = true;
  const std::string launch_cmd = "ros2 launch " + scene_name + " demo.launch.py use_fake_hardware:=true";
  report.next_commands = {
    "colcon build --symlink-install --packages-select " + scene_name,
    "source install/setup.bash",
    launch_cmd
  };
  add_item(report, "validation summary", report.blockers.empty() ? "OK" : "ERROR", report.blockers.empty() ? "No blockers" : "Blockers present");
  return report;
}

}  // namespace workcell_builder
