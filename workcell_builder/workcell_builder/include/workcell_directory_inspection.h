// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__WORKCELL_DIRECTORY_INSPECTION_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__WORKCELL_DIRECTORY_INSPECTION_H_

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <string>

namespace workcell_builder
{
namespace fs = boost::filesystem;

struct WorkcellRootInspection
{
  bool success{false};
  bool use_existing_root{false};
  fs::path selected_path;
  fs::path canonical_selected_path;
  fs::path workcell_root;
  std::string root_status_suffix;
  std::string error;
};

enum class WorkcellRootDirectoryProbeResult
{
  kMissing,
  kValidDirectory,
  kNonDirectoryCollision,
  kInspectionError,
};

inline WorkcellRootDirectoryProbeResult probe_workcell_root_directory(
  const fs::path & directory_path,
  fs::path * failing_path,
  boost::system::error_code * error)
{
  boost::system::error_code local_error;
  const auto path_status = fs::symlink_status(directory_path, local_error);
  if (local_error) {
    if (failing_path != nullptr) {
      *failing_path = directory_path;
    }
    if (error != nullptr) {
      *error = local_error;
    }
    return WorkcellRootDirectoryProbeResult::kInspectionError;
  }

  if (!fs::exists(path_status)) {
    return WorkcellRootDirectoryProbeResult::kMissing;
  }

  if (fs::is_symlink(path_status)) {
    const auto target_status = fs::status(directory_path, local_error);
    if (local_error) {
      if (failing_path != nullptr) {
        *failing_path = directory_path;
      }
      if (error != nullptr) {
        *error = local_error;
      }
      return WorkcellRootDirectoryProbeResult::kInspectionError;
    }

    if (fs::is_directory(target_status)) {
      return WorkcellRootDirectoryProbeResult::kValidDirectory;
    }
  } else if (fs::is_directory(path_status)) {
    return WorkcellRootDirectoryProbeResult::kValidDirectory;
  }

  if (failing_path != nullptr) {
    *failing_path = directory_path;
  }
  if (error != nullptr) {
    *error = boost::system::errc::make_error_code(boost::system::errc::not_a_directory);
  }
  return WorkcellRootDirectoryProbeResult::kNonDirectoryCollision;
}

inline bool has_workcell_root_directories(
  const fs::path & candidate_root,
  fs::path * failing_path,
  boost::system::error_code * error)
{
  auto clear_outputs = [&]() {
      if (failing_path != nullptr) {
        failing_path->clear();
      }
      if (error != nullptr) {
        error->clear();
      }
    };

  clear_outputs();

  const auto scenes_probe = probe_workcell_root_directory(candidate_root / "scenes", failing_path, error);
  if (scenes_probe == WorkcellRootDirectoryProbeResult::kValidDirectory) {
    return true;
  }
  if (scenes_probe == WorkcellRootDirectoryProbeResult::kNonDirectoryCollision ||
    scenes_probe == WorkcellRootDirectoryProbeResult::kInspectionError)
  {
    return false;
  }

  const auto assets_probe = probe_workcell_root_directory(candidate_root / "assets", failing_path, error);
  return assets_probe == WorkcellRootDirectoryProbeResult::kValidDirectory;
}

inline WorkcellRootInspection inspect_selected_workcell_path(const fs::path & selected_path)
{
  WorkcellRootInspection inspection;
  inspection.selected_path = selected_path;

  boost::system::error_code ec;
  if (!fs::exists(selected_path, ec)) {
    inspection.error = ec ?
      "Failed to inspect selected directory '" + selected_path.string() + "': " + ec.message() :
      "Selected path does not exist: '" + selected_path.string() + "'";
    return inspection;
  }
  if (ec) {
    inspection.error = "Failed to inspect selected directory '" + selected_path.string() + "': " +
      ec.message();
    return inspection;
  }

  if (!fs::is_directory(selected_path, ec)) {
    inspection.error = ec ?
      "Failed to inspect selected directory '" + selected_path.string() + "': " + ec.message() :
      "Selected path is not a directory: '" + selected_path.string() + "'";
    return inspection;
  }
  if (ec) {
    inspection.error = "Failed to inspect selected directory '" + selected_path.string() + "': " +
      ec.message();
    return inspection;
  }

  inspection.canonical_selected_path = fs::weakly_canonical(selected_path, ec);
  if (ec) {
    inspection.error = "Failed to canonicalize selected directory '" + selected_path.string() +
      "': " + ec.message();
    return inspection;
  }

  fs::path failing_path;
  bool base_has_root_dirs = has_workcell_root_directories(
    inspection.canonical_selected_path, &failing_path, &ec);
  const fs::path base_failing_path = failing_path;
  const boost::system::error_code base_probe_error = ec;

  const fs::path src_path = inspection.canonical_selected_path / "src";
  failing_path.clear();
  ec.clear();
  bool src_has_root_dirs = has_workcell_root_directories(src_path, &failing_path, &ec);
  const fs::path src_failing_path = failing_path;
  const boost::system::error_code src_probe_error = ec;

  if (base_probe_error && !src_has_root_dirs) {
    inspection.error = "Failed to inspect directory '" + base_failing_path.string() + "': " +
      base_probe_error.message();
    if (src_probe_error) {
      inspection.error += "; selected path/src was also unusable at '" +
        src_failing_path.string() + "': " + src_probe_error.message();
    }
    return inspection;
  }

  if (src_probe_error && !base_has_root_dirs) {
    inspection.error = "Failed to inspect directory '" + src_failing_path.string() + "': " +
      src_probe_error.message();
    if (base_probe_error) {
      inspection.error += "; selected path was also unusable at '" +
        base_failing_path.string() + "': " + base_probe_error.message();
    }
    return inspection;
  }

  inspection.success = true;
  if (base_has_root_dirs) {
    inspection.use_existing_root = true;
    inspection.workcell_root = inspection.canonical_selected_path;
    inspection.root_status_suffix = " (using selected path as workcell root)";
  } else if (src_has_root_dirs) {
    inspection.use_existing_root = true;
    inspection.workcell_root = src_path;
    inspection.root_status_suffix = " (using selected path/src as workcell root)";
  } else {
    inspection.workcell_root = src_path;
    inspection.root_status_suffix = " (created selected path/src as workcell root)";
  }

  return inspection;
}

}  // namespace workcell_builder

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__WORKCELL_DIRECTORY_INSPECTION_H_
