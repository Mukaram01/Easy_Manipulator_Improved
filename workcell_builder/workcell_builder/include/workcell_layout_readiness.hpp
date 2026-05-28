#pragma once

#include <boost/filesystem.hpp>
#include <cstddef>

namespace workcell_builder {

enum class LayoutStateModel {
  NO_LAYOUT_FILE,
  EMPTY_LAYOUT,
  EDITABLE_LAYOUT_PRESENT,
  PREVIEW_ONLY_AVAILABLE,
  PREVIEW_UNAVAILABLE,
  PATH_MISMATCH,
  INVALID_LAYOUT_YAML
};

struct LayoutReadinessState {
  LayoutStateModel state{LayoutStateModel::NO_LAYOUT_FILE};
  std::size_t editable_item_count{0};
};

LayoutReadinessState derive_layout_state_model(
  const boost::filesystem::path & scene_dir,
  std::size_t preview_item_count,
  bool path_match);

bool save_layout_workflow_ready(
  const boost::filesystem::path & scene_dir,
  const LayoutReadinessState & layout_state);

}  // namespace workcell_builder
