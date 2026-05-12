#include "workcell_scene_status.hpp"

#include <yaml-cpp/yaml.h>
#include "workcell_zone_model.hpp"
#include "conveyor_pick_preview.hpp"
#include "workcell_perception_snapshot.hpp"
#include <fstream>

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

      std::vector<WorkZone> zones; std::vector<ConveyorFlow> flows;
      parse_work_zones_from_yaml(root, &zones, &flows);
      std::vector<std::string> camera_names; if (root["cameras"]) { for (const auto & c : root["cameras"]) { if (c["name"]) camera_names.push_back(c["name"].as<std::string>()); } }
      std::vector<std::string> robots; if (root["robot"] && root["robot"]["name"]) robots.push_back(root["robot"]["name"].as<std::string>());
      const auto zone_validation = validate_work_zones(zones, flows, camera_names, robots);
      add_item(report, "Detection zone configured", std::any_of(zones.begin(), zones.end(), [](const WorkZone & z){ return z.type == "camera_detection"; }) ? "OK" : "WARN", "Metadata check");
      add_item(report, "Pick zone configured", std::any_of(zones.begin(), zones.end(), [](const WorkZone & z){ return z.type == "robot_pick"; }) ? "OK" : "WARN", "Metadata check");
      add_item(report, "Place zone configured", std::any_of(zones.begin(), zones.end(), [](const WorkZone & z){ return z.type == "robot_place"; }) ? "OK" : "WARN", "Metadata check");

      add_item(report, "Conveyor flow configured", flows.empty() ? "INFO" : "OK", flows.empty() ? "No conveyor flow metadata" : "Conveyor flow metadata present");
      add_item(report, "Detection snapshot available", fs::exists(scene_dir / "preview" / "perception_detection_snapshot.yaml") ? "OK" : "INFO", "EPD-compatible offline metadata");
      add_item(report, "No EPD runtime launched", "OK", "EPD GUI remains separate");
      if (!flows.empty()) {
        const auto preview = generate_preview_result(zones, flows.front());
        add_item(report, "conveyor pick preview available", preview.valid ? "OK" : "ERROR", preview.valid ? "preview metadata computed" : "preview metadata invalid");
        add_item(report, "time_to_pick_s", preview.valid ? "INFO" : "ERROR", std::to_string(preview.time_to_pick_s));
        add_item(report, "speed_mps", preview.valid ? "INFO" : "ERROR", std::to_string(preview.speed_mps));
        add_item(report, "detection zone exists", preview.valid ? "OK" : "ERROR", preview.detection_zone);
        add_item(report, "pick zone exists", preview.valid ? "OK" : "ERROR", preview.pick_zone);
        add_item(report, "preview_only", "INFO", "preview_only safety note shown");
        add_item(report, "no robot motion commanded", "OK", "No robot motion commanded");
      }

      for (const auto & info : zone_validation.infos) add_item(report, "Work zones", "INFO", info);
      for (const auto & warn : zone_validation.warnings) { add_item(report, "Work zones", "WARN", warn); report.warnings.push_back(warn); }
      for (const auto & err : zone_validation.errors) { add_item(report, "Work zones", "ERROR", err); report.blockers.push_back(err); }

      if (root["cameras"]) {
        add_item(report, "Camera configured", "OK", "camera metadata present", environment_yaml);
        for (const auto & c : root["cameras"]) {
          const std::string pkg = c["package"] ? c["package"].as<std::string>() : "";
          const std::string po = c["parent_object"] ? c["parent_object"].as<std::string>() : "world";
          const std::string pl = c["parent_link"] ? c["parent_link"].as<std::string>() : "world";
          add_item(report, "Camera package found", pkg == "realsense2_description" ? "OK" : "WARN", pkg);
          add_item(report, "Parent mount object found", po == "world" ? "WARN" : "OK", po == "world" ? "camera appears floating unless intentionally wall/world mounted" : po);
          add_item(report, "Parent mount link found", pl.empty() ? "ERROR" : "OK", pl.empty() ? "camera parent link missing" : pl);
          add_item(report, "Runtime driver", "INFO", (c["runtime_driver"] ? c["runtime_driver"].as<std::string>() : "metadata_only") + " (metadata/URDF preview only)");
        }
      } else { add_item(report, "Camera configured", "WARN", "No camera metadata found", environment_yaml); }
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
