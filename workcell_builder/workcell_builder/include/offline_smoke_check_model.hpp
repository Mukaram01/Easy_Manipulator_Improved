#pragma once

#include <boost/filesystem.hpp>
#include <map>
#include <string>
#include <vector>

#include "attributes/workcell.h"

namespace workcell_builder
{

enum class OfflineSmokeStatus
{
  PASS,
  WARNINGS,
  BLOCKED,
  PREVIEW_ONLY
};

struct OfflineSmokeCheck
{
  std::string id;
  std::string category;
  std::string status;
  std::string message;
  std::string fix_hint;
  std::string artifact_path;
};

struct OfflineSmokeCheckResult
{
  OfflineSmokeStatus status{OfflineSmokeStatus::BLOCKED};
  std::string scene_name;
  std::string scene_dir;
  std::vector<OfflineSmokeCheck> checks;
  std::vector<std::string> blockers;
  std::vector<std::string> warnings;
  std::vector<std::string> generated_artifacts;
  std::string next_action;
  std::map<std::string, bool> safety_flags;
  std::string build_command;
  std::string launch_command;
};

OfflineSmokeCheckResult run_offline_smoke_check(const Scene & scene, const boost::filesystem::path & scene_dir);
std::string offline_smoke_status_label(OfflineSmokeStatus status);
bool write_offline_smoke_report(const OfflineSmokeCheckResult & result, std::string * error_message);

}  // namespace workcell_builder
