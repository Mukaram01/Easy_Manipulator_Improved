#pragma once

#include <boost/filesystem.hpp>
#include <string>
#include <vector>

#include "attributes/workcell.h"

namespace workcell_builder
{
enum class ValidationStatus
{
  PASS,
  WARN,
  FAIL,
  SKIP,
  PREVIEW_ONLY,
  UNKNOWN
};

struct ValidationDashboardRow
{
  std::string check_name;
  ValidationStatus status{ValidationStatus::UNKNOWN};
  std::string message;
  int warning_count{0};
  int blocker_count{0};
  std::string report_path;
  std::string fix_hint;
};

struct ValidationDashboardResult
{
  std::vector<ValidationDashboardRow> rows;
  int warning_count{0};
  int blocker_count{0};
  ValidationStatus status{ValidationStatus::UNKNOWN};
};

ValidationDashboardResult collect_validation_dashboard_results(
  const Scene & scene,
  const boost::filesystem::path & scene_dir);
ValidationDashboardResult default_validation_dashboard_result();
std::string validation_status_label(ValidationStatus status);
int validation_status_severity(ValidationStatus status);
std::string format_validation_fix_hint(const ValidationDashboardRow & row);

}  // namespace workcell_builder
