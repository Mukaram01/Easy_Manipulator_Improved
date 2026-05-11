#include "validation_dashboard_model.hpp"

#include <algorithm>

namespace fs = boost::filesystem;
namespace workcell_builder
{
namespace
{
ValidationDashboardRow make_unknown(const std::string & name)
{
  return ValidationDashboardRow{name, ValidationStatus::UNKNOWN, "Not yet validated.", 0, 0, "", "Run Offline Validation."};
}

ValidationStatus aggregate(const std::vector<ValidationDashboardRow> & rows)
{
  int worst = validation_status_severity(ValidationStatus::PASS);
  for (const auto & row : rows) {
    worst = std::max(worst, validation_status_severity(row.status));
  }
  if (worst >= validation_status_severity(ValidationStatus::FAIL)) {return ValidationStatus::FAIL;}
  if (worst == validation_status_severity(ValidationStatus::WARN)) {return ValidationStatus::WARN;}
  if (worst == validation_status_severity(ValidationStatus::SKIP)) {return ValidationStatus::SKIP;}
  if (worst == validation_status_severity(ValidationStatus::PASS)) {return ValidationStatus::PASS;}
  return ValidationStatus::UNKNOWN;
}
}  // namespace

ValidationDashboardResult default_validation_dashboard_result()
{
  ValidationDashboardResult out;
  out.rows = {
    make_unknown("Scene Schema"), make_unknown("Asset Catalog"), make_unknown("Robot / Tool Compatibility"),
    make_unknown("Object Placement"), make_unknown("Camera Metadata"), make_unknown("Task Recipe"),
    make_unknown("Readiness Overlay"), make_unknown("Fake-Hardware Smoke Static"), make_unknown("Generation Safety"), make_unknown("Simulation Readiness"), make_unknown("Real Hardware Metadata"), make_unknown("Robot Driver Requirements"), make_unknown("Tool I/O Requirements"), make_unknown("Camera Calibration Requirements"), make_unknown("EPD Compatibility Metadata")};
  out.status = ValidationStatus::UNKNOWN;
  return out;
}

ValidationDashboardResult collect_validation_dashboard_results(const Scene & scene, const fs::path & scene_dir)
{
  ValidationDashboardResult out = default_validation_dashboard_result();
  for (auto & row : out.rows) {
    row.status = ValidationStatus::PASS;
    row.message = "Offline check passed.";
    row.fix_hint = "";
  }
  auto mark_fail = [&](const std::string & name, const std::string & msg, const std::string & hint) {
      for (auto & r : out.rows) {if (r.check_name == name) {r.status = ValidationStatus::FAIL; r.message = msg; r.blocker_count = 1; r.fix_hint = hint; break;}}
    };
  if (!scene.loaded) {
    mark_fail("Scene Schema", "Scene metadata not loaded from environment.yaml.", "Load scene metadata then re-run offline validation.");
  }
  if (!fs::exists(scene_dir / "environment.yaml")) {
    mark_fail("Scene Schema", "environment.yaml is missing.", "Generate YAML files for scene.");
  }
  if (!scene.robot_loaded || scene.robot_vector.empty()) {
    mark_fail("Robot / Tool Compatibility", "Robot selection is missing.", "Select robot and apply profile defaults.");
  }
  if (scene.object_vector.empty()) {
    for (auto & r : out.rows) {
    if (r.check_name == "Simulation Readiness") {r.message = "Offline-only simulation readiness metadata.";}
    if (r.check_name == "Real Hardware Metadata") {r.status = ValidationStatus::WARN; r.message = "NOT ENABLED / METADATA ONLY"; r.warning_count = 1;}
    if (r.check_name == "Robot Driver Requirements") {r.status = ValidationStatus::WARN; r.message = "driver package hint missing"; r.warning_count = 1;}
    if (r.check_name == "Tool I/O Requirements") {r.status = ValidationStatus::WARN; r.message = "tool I/O mapping missing for suction/gripper"; r.warning_count = 1;}
    if (r.check_name == "Camera Calibration Requirements") {r.status = ValidationStatus::WARN; r.message = "camera topics incomplete"; r.warning_count = 1;}
    if (r.check_name == "EPD Compatibility Metadata") {r.status = ValidationStatus::WARN; r.message = "EPD adapter metadata missing"; r.warning_count = 1;}
      if (r.check_name == "Object Placement") {r.status = ValidationStatus::WARN; r.message = "No placed objects yet."; r.warning_count = 1; r.fix_hint = "Add placed objects or support surface.";}
    }
  }
  for (auto & r : out.rows) {
    if (r.check_name == "Simulation Readiness") {r.message = "Offline-only simulation readiness metadata.";}
    if (r.check_name == "Real Hardware Metadata") {r.status = ValidationStatus::WARN; r.message = "NOT ENABLED / METADATA ONLY"; r.warning_count = 1;}
    if (r.check_name == "Robot Driver Requirements") {r.status = ValidationStatus::WARN; r.message = "driver package hint missing"; r.warning_count = 1;}
    if (r.check_name == "Tool I/O Requirements") {r.status = ValidationStatus::WARN; r.message = "tool I/O mapping missing for suction/gripper"; r.warning_count = 1;}
    if (r.check_name == "Camera Calibration Requirements") {r.status = ValidationStatus::WARN; r.message = "camera topics incomplete"; r.warning_count = 1;}
    if (r.check_name == "EPD Compatibility Metadata") {r.status = ValidationStatus::WARN; r.message = "EPD adapter metadata missing"; r.warning_count = 1;}
    if (r.check_name == "Asset Catalog") {r.report_path = "generated/asset_catalog_validation.json";}
    if (r.check_name == "Task Recipe") {r.report_path = "config/task_recipe.yaml";}
    if (r.check_name == "Readiness Overlay") {r.report_path = "generated/readiness_overlay_report.json";}
    if (r.check_name == "Fake-Hardware Smoke Static") {r.status = ValidationStatus::SKIP; r.message = "Static smoke status only (no ROS launch)."; r.fix_hint = "Optional: run scripts/run_workcell_fake_hardware_smoke.py --skip-launch";}
    if (r.check_name == "Generation Safety") {r.message = "No ROS launch, no MoveIt planning, no robot execution.";}
    out.warning_count += r.warning_count;
    out.blocker_count += r.blocker_count;
  }
  out.status = aggregate(out.rows);
  return out;
}

std::string validation_status_label(ValidationStatus status)
{
  switch (status) {
    case ValidationStatus::PASS: return "PASS";
    case ValidationStatus::WARN: return "WARN";
    case ValidationStatus::FAIL: return "FAIL";
    case ValidationStatus::SKIP: return "SKIP";
    default: return "UNKNOWN";
  }
}

int validation_status_severity(ValidationStatus status)
{
  switch (status) {
    case ValidationStatus::FAIL: return 4;
    case ValidationStatus::WARN: return 3;
    case ValidationStatus::SKIP: return 2;
    case ValidationStatus::PASS: return 1;
    default: return 0;
  }
}

std::string format_validation_fix_hint(const ValidationDashboardRow & row)
{
  if (!row.fix_hint.empty()) {return row.fix_hint;}
  if (!row.report_path.empty()) {return std::string("Report: ") + row.report_path;}
  return "";
}

}  // namespace workcell_builder
