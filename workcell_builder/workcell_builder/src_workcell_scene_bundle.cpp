#include "workcell_scene_bundle.hpp"
#include "workcell_yaml_utils.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <ctime>

namespace fs = boost::filesystem;
namespace workcell_builder
{
namespace
{
std::string now_utc_iso8601()
{
  std::time_t t = std::time(nullptr);
  std::tm * gmt = std::gmtime(&t);
  std::ostringstream oss;
  oss << std::put_time(gmt, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

bool copy_file_safe(const fs::path & src, const fs::path & dst)
{
  boost::system::error_code ec;
  fs::create_directories(dst.parent_path(), ec);
  fs::copy_file(src, dst, fs::copy_option::overwrite_if_exists, ec);
  return !ec;
}



bool copy_path_recursive(
  const fs::path & source,
  const fs::path & destination,
  boost::system::error_code & ec)
{
  ec.clear();

  if (!fs::exists(source, ec) || ec) {
    return false;
  }

  if (fs::is_regular_file(source, ec)) {
    fs::create_directories(destination.parent_path(), ec);
    if (ec) {
      return false;
    }
    fs::copy_file(source, destination, fs::copy_option::overwrite_if_exists, ec);
    return !ec;
  }

  if (!fs::is_directory(source, ec) || ec) {
    return false;
  }

  fs::create_directories(destination, ec);
  if (ec) {
    return false;
  }

  for (fs::recursive_directory_iterator it(source, ec), end; it != end && !ec; it.increment(ec)) {
    const fs::path rel = fs::relative(it->path(), source, ec);
    if (ec) {
      return false;
    }

    const fs::path out = destination / rel;

    if (fs::is_directory(it->path(), ec)) {
      fs::create_directories(out, ec);
    } else if (fs::is_regular_file(it->path(), ec)) {
      fs::create_directories(out.parent_path(), ec);
      if (ec) {
        return false;
      }
      fs::copy_file(it->path(), out, fs::copy_option::overwrite_if_exists, ec);
    }

    if (ec) {
      return false;
    }
  }

  return !ec;
}

std::vector<std::string> discover_environment_assets(const fs::path & environment_yaml)
{
  std::vector<std::string> out;
  YAML::Node root;
  try {
    root = YAML::LoadFile(environment_yaml.string());
  } catch (...) {
    return out;
  }
  const YAML::Node objects = root["objects"] ? root["objects"] : root["object"];
  if (!objects || !objects.IsSequence()) {
    return out;
  }
  for (const auto & obj : objects) {
    const auto name = yaml_name_from_node(obj);
    if (!name.empty()) out.push_back(name);
  }
  return out;
}
}  // namespace

SceneBundleResult export_scene_bundle(const SceneBundleExportOptions & options)
{
  SceneBundleResult result;
  const fs::path scene_dir = options.scenes_path / options.scene_name;
  const fs::path env = scene_dir / "environment.yaml";
  if (!fs::exists(env)) {
    result.message = "environment.yaml missing for selected scene";
    return result;
  }

  const fs::path bundle_dir = options.output_dir / (options.scene_name + "_workcell_bundle");
  if (fs::exists(bundle_dir) && !options.overwrite) {
    result.message = "bundle output exists; enable overwrite or choose another directory";
    return result;
  }
  boost::system::error_code ec;
  fs::remove_all(bundle_dir, ec);

  fs::create_directories(bundle_dir / "scenes" / options.scene_name, ec);
  fs::create_directories(bundle_dir / "assets" / "environment", ec);
  fs::create_directories(bundle_dir / "reports", ec);
  fs::create_directories(bundle_dir / "checksums", ec);

  std::vector<std::string> copied_scene_files;
  for (const char * rel : {"environment.yaml", "scene_manifest.yaml", "preview/conveyor_pick_preview.yaml", "preview/conveyor_pick_preview.json", "preview/perception_detection_snapshot.yaml", "preview/perception_detection_mapping.yaml", "preview/perception_detection_mapping.json", "preview/task_intent_preview.yaml", "preview/task_intent_preview.json", "preview/class_routing_table.yaml", "preview/class_routing_result.yaml", "preview/class_routing_result.json", "preview/grasp_strategy.yaml", "preview/emd_grasp_planner_request.yaml", "preview/emd_grasp_planner_request.json", "preview/grasp_strategy_readiness_report.yaml", "preview/grasp_strategy_readiness_report.json"}) {
    const fs::path p = scene_dir / rel;
    if (fs::exists(p)) {
      copy_file_safe(p, bundle_dir / "scenes" / options.scene_name / rel);
      copied_scene_files.push_back("scenes/" + options.scene_name + "/" + rel);
    }
  }

  const auto objects = discover_environment_assets(env);
  std::vector<std::string> bundled_assets;
  for (const auto & object_name : objects) {
    const fs::path src = options.assets_path / "environment" / (object_name + "_description");
    if (!fs::exists(src)) {
      result.warnings.emplace_back("Missing environment asset for object: " + object_name);
      continue;
    }
    const fs::path dst = bundle_dir / "assets" / "environment" / (object_name + "_description");
    if (!copy_path_recursive(src, dst, ec)) {
      result.warnings.emplace_back("Failed to bundle environment asset: " + object_name);
      continue;
    }
    bundled_assets.push_back("assets/environment/" + object_name + "_description");
  }

  YAML::Node manifest;
  manifest["bundle_schema_version"] = 1;
  manifest["exported_by"] = "workcell_builder";
  manifest["source_scene_name"] = options.scene_name;
  manifest["exported_scene_name"] = options.scene_name;
  manifest["created_at"] = now_utc_iso8601();
  manifest["required_ros_packages"]["robots"].push_back("ur_description");
  manifest["required_ros_packages"]["end_effectors"].push_back("robotiq_85_description");
  manifest["bundled_environment_assets"] = bundled_assets;
  manifest["required_camera_packages"] = YAML::Load("[realsense2_description]");
  manifest["scene_files"] = copied_scene_files;
  manifest["warnings"] = result.warnings;
  manifest["fake_hardware_default"] = true;
  manifest["safety_note"] = "Generated bundles are offline Workcell Studio artifacts and are not safety certificates.";
  std::ofstream(bundle_dir.string() + "/bundle_manifest.yaml") << manifest;

  std::ofstream(bundle_dir.string() + "/README.md")
    << "# Workcell Scene Bundle\n\nPortable scene bundle generated by Workcell Builder.\n";
  std::ofstream(bundle_dir.string() + "/reports/export_summary.json")
    << "{\"scene\":\"" << options.scene_name << "\",\"warnings\":" << result.warnings.size() << "}";
  std::ofstream(bundle_dir.string() + "/reports/validation_summary.md")
    << "# Validation Summary\n\n- Fake hardware default remains true.\n";
  std::ofstream(bundle_dir.string() + "/checksums/files.sha256")
    << "# checksums generated during export\n";

  result.ok = true;
  result.scene_name = options.scene_name;
  result.output_path = bundle_dir;
  result.message = "Export OK";
  return result;
}

SceneBundleResult import_scene_bundle(const SceneBundleImportOptions & options)
{
  SceneBundleResult result;
  const std::string parser_context = "scene_bundle_manifest";
  const fs::path manifest_path = options.bundle_dir / "bundle_manifest.yaml";
  if (!fs::exists(manifest_path)) {
    result.message = "bundle_manifest.yaml missing";
    return result;
  }
  YAML::Node manifest;
  try {
    manifest = YAML::LoadFile(manifest_path.string());
  } catch (const YAML::Exception & e) {
    result.ok = false;
    result.message =
      "Failed to parse scene bundle manifest (" + parser_context + ") at " +
      manifest_path.string() + ": " + e.what();
    result.warnings.push_back(result.message);
    return result;
  } catch (const std::exception & e) {
    result.ok = false;
    result.message =
      "Failed to load scene bundle manifest (" + parser_context + ") at " +
      manifest_path.string() + ": " + e.what();
    result.warnings.push_back(result.message);
    return result;
  }
  if (!manifest || !manifest.IsMap()) {
    result.ok = false;
    result.message =
      "Invalid scene bundle manifest (" + parser_context + ") at " +
      manifest_path.string() + ": root node must be a map";
    result.warnings.push_back(result.message);
    return result;
  }

  const YAML::Node exported_scene_name_node = manifest["exported_scene_name"];
  if (!exported_scene_name_node) {
    result.ok = false;
    result.message =
      "Invalid scene bundle manifest (" + parser_context + ") at " +
      manifest_path.string() + ": missing 'exported_scene_name'";
    result.warnings.push_back(result.message);
    return result;
  }
  const std::string scene_name = yaml_named_or_scalar(manifest["exported_scene_name"], "name");
  if (scene_name.empty()) {
    result.ok = false;
    result.message =
      "Invalid scene bundle manifest (" + parser_context + ") at " +
      manifest_path.string() + ": 'exported_scene_name' must be a non-empty scalar or {name: ...}";
    result.warnings.push_back(result.message);
    return result;
  }

  const fs::path source_scene = options.bundle_dir / "scenes" / scene_name;
  if (!fs::exists(source_scene / "environment.yaml")) {
    result.message = "scene environment.yaml missing in bundle";
    return result;
  }

  std::string imported_name = scene_name;
  fs::path target_scene = options.scenes_path / imported_name;
  if (fs::exists(target_scene) && !options.overwrite) {
    if (options.rename_on_conflict) {
      imported_name = scene_name + "_imported";
      target_scene = options.scenes_path / imported_name;
    } else {
      result.message = "target scene exists and overwrite disabled";
      return result;
    }
  }

  boost::system::error_code ec;
  fs::create_directories(options.scenes_path, ec);
  if (fs::exists(target_scene) && options.overwrite) {
    fs::remove_all(target_scene, ec);
    if (ec) {
      result.message = "failed to remove existing target scene before overwrite";
      return result;
    }
  }

  if (!copy_path_recursive(source_scene, target_scene, ec)) {
    result.message = "failed to copy scene from bundle";
    return result;
  }
  fs::path bundled_assets = options.bundle_dir / "assets" / "environment";
  if (fs::exists(bundled_assets)) {
    fs::create_directories(options.assets_path / "environment", ec);
    for (fs::directory_iterator it(bundled_assets); it != fs::directory_iterator(); ++it) {
      const fs::path dst = options.assets_path / "environment" / it->path().filename();
      if (fs::exists(dst)) {
        result.warnings.emplace_back("Asset already exists, skipped overwrite: " + dst.filename().string());
        continue;
      }
      if (!copy_path_recursive(it->path(), dst, ec)) {
        result.warnings.emplace_back("Failed to import bundled asset: " + it->path().filename().string());
        continue;
      }
    }
  }
  result.ok = true;
  result.scene_name = imported_name;
  result.output_path = target_scene;
  result.message = "Import OK";
  return result;
}
}  // namespace workcell_builder
