#ifndef PACKAGE_URI_UTILS_H_
#define PACKAGE_URI_UTILS_H_

#include <boost/filesystem.hpp>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace workcell_builder
{
namespace fs = boost::filesystem;

struct ResolvedPackageUri
{
  bool success = false;
  std::string original_uri;
  std::string package_name;
  std::string relative_path;
  fs::path package_share_path;
  fs::path resolved_path;
};

inline ResolvedPackageUri resolvePackageUriToPath(const std::string & uri)
{
  ResolvedPackageUri result;
  result.original_uri = uri;
  const std::string prefix = "package://";
  if (uri.rfind(prefix, 0) != 0) {
    result.success = true;
    result.resolved_path = fs::path(uri);
    return result;
  }

  const std::string package_and_path = uri.substr(prefix.size());
  const std::size_t first_slash = package_and_path.find('/');
  result.package_name = package_and_path.substr(0, first_slash);
  result.relative_path = first_slash == std::string::npos ? "" : package_and_path.substr(first_slash + 1);

  if (result.package_name.empty() || result.relative_path.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("workcell_builder"),
      "Failed to resolve package URI '%s': package='%s' relative_path='%s'",
      uri.c_str(), result.package_name.c_str(), result.relative_path.c_str());
    return result;
  }

  try {
    result.package_share_path = fs::path(ament_index_cpp::get_package_share_directory(result.package_name));
    result.resolved_path = result.package_share_path / result.relative_path;
    result.success = fs::exists(result.resolved_path);
    if (!result.success) {
      RCLCPP_ERROR(
        rclcpp::get_logger("workcell_builder"),
        "Failed to resolve package URI '%s': package='%s' relative_path='%s' share='%s'",
        uri.c_str(), result.package_name.c_str(), result.relative_path.c_str(),
        result.package_share_path.string().c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("workcell_builder"),
      "Failed to resolve package URI '%s': package='%s' relative_path='%s' error='%s'",
      uri.c_str(), result.package_name.c_str(), result.relative_path.c_str(), e.what());
  }
  return result;
}
}  // namespace workcell_builder

#endif  // PACKAGE_URI_UTILS_H_
