#pragma once

#include <map>
#include <string>
#include <vector>

namespace workcell_builder {

struct RobotHomeConfig
{
  std::string robot_profile;
  std::string source{"suggested"};
  std::vector<std::string> joint_order;
  std::map<std::string, double> joints;
  std::map<std::string, double> suggested_joints;
};

struct RobotHomeWriteResult
{
  bool ok{false};
  std::string detail;
  std::string backup_path;
};

double robot_home_degrees_to_radians(double degrees);
double robot_home_radians_to_degrees(double radians);
bool load_robot_home_from_environment_yaml(
  const std::string & path, RobotHomeConfig * config, std::string * detail = nullptr);
RobotHomeWriteResult save_robot_home_to_environment_yaml(
  const std::string & path, const RobotHomeConfig & config);

}  // namespace workcell_builder
