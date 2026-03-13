#ifndef DEFAULT_ASSET_PATHS_H_
#define DEFAULT_ASSET_PATHS_H_

#include <boost/filesystem.hpp>
#include <exception>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fs = boost::filesystem;

inline fs::path get_workcell_builder_share_directory()
{
  try {
    return fs::path(ament_index_cpp::get_package_share_directory("workcell_builder"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("workcell_builder"), "%s", e.what());
    return fs::path();
  }
}

inline fs::path get_default_templates_directory()
{
  const fs::path share_path = get_workcell_builder_share_directory();
  return share_path.empty() ? fs::path() : share_path / "templates";
}

inline fs::path get_default_assets_directory()
{
  const fs::path share_path = get_workcell_builder_share_directory();
  return share_path.empty() ? fs::path() : share_path / "assets";
}

inline fs::path get_runtime_assets_directory(const fs::path & workcell_root)
{
  const fs::path workcell_assets = workcell_root / "assets";
  if (fs::exists(workcell_assets)) {
    return workcell_assets;
  }
  return get_default_assets_directory();
}

#endif  // DEFAULT_ASSET_PATHS_H_
