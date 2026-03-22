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

  auto probe_directory = [&](const fs::path & directory_path) {
      boost::system::error_code local_error;
      const bool path_exists = fs::exists(directory_path, local_error);
      if (local_error) {
        if (failing_path != nullptr) {
          *failing_path = directory_path;
        }
        if (error != nullptr) {
          *error = local_error;
        }
        return false;
      }

      if (!path_exists) {
        return false;
      }

      local_error.clear();
      const bool is_directory = fs::is_directory(directory_path, local_error);
      if (local_error || !is_directory) {
        if (!local_error && !is_directory) {
          local_error = boost::system::errc::make_error_code(boost::system::errc::not_a_directory);
        }
        if (failing_path != nullptr) {
          *failing_path = directory_path;
        }
        if (error != nullptr) {
          *error = local_error;
        }
        return false;
      }

      return true;
    };

  clear_outputs();
  return probe_directory(candidate_root / "scenes") ||
         probe_directory(candidate_root / "assets");
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
  if (ec) {
    inspection.error = "Failed to inspect directory '" + failing_path.string() + "': " + ec.message();
    return inspection;
  }

  const fs::path src_path = inspection.canonical_selected_path / "src";
  bool src_has_root_dirs = has_workcell_root_directories(src_path, &failing_path, &ec);
  if (ec) {
    inspection.error = "Failed to inspect directory '" + failing_path.string() + "': " + ec.message();
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
