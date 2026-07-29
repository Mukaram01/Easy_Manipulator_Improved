#include "robot_home_yaml_io.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <algorithm>
#include <cctype>

#include <QDateTime>
#include <yaml-cpp/yaml.h>

namespace workcell_builder {
namespace {
constexpr double kPi = 3.14159265358979323846;

std::vector<std::string> ur_arm_joint_order()
{
  return {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
    "wrist_2_joint", "wrist_3_joint"};
}

std::map<std::string, double> ur5_suggestion()
{
  return {{"shoulder_pan_joint", 0.0}, {"shoulder_lift_joint", -kPi / 2.0},
    {"elbow_joint", kPi / 2.0}, {"wrist_1_joint", -kPi / 2.0},
    {"wrist_2_joint", -kPi / 2.0}, {"wrist_3_joint", 0.0}};
}

std::string robot_profile_name(const YAML::Node & robot)
{
  for (const char * key : {"model", "name", "type", "profile"}) {
    if (robot[key] && robot[key].IsScalar()) return robot[key].as<std::string>();
  }
  return {};
}
}  // namespace

double robot_home_degrees_to_radians(double degrees) { return degrees * kPi / 180.0; }
double robot_home_radians_to_degrees(double radians) { return radians * 180.0 / kPi; }

bool load_robot_home_from_environment_yaml(
  const std::string & path, RobotHomeConfig * config, std::string * detail)
{
  try {
    const YAML::Node yaml = YAML::LoadFile(path);
    const YAML::Node robot = yaml["robot"];
    if (!robot || !robot.IsMap()) throw std::runtime_error("robot profile is missing");
    config->robot_profile = robot_profile_name(robot);
    std::string normalized = config->robot_profile;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    const bool supported_ur_arm = normalized.find("ur3") != std::string::npos ||
      normalized.find("ur5") != std::string::npos || normalized.find("ur10") != std::string::npos;
    if (!supported_ur_arm) {
      throw std::runtime_error("active robot profile has no Robot Home joint definition: " + normalized);
    }
    config->joint_order = ur_arm_joint_order();
    config->suggested_joints = ur5_suggestion();
    config->joints = config->suggested_joints;
    const YAML::Node home = robot["home_joint_state"];
    if (home && home["source"]) config->source = home["source"].as<std::string>();
    const YAML::Node joints = home ? home["joints"] : YAML::Node();
    if (joints) {
      if (!joints.IsMap()) throw std::runtime_error("robot.home_joint_state.joints must be a map");
      std::set<std::string> actual;
      for (const auto & item : joints) actual.insert(item.first.as<std::string>());
      const std::set<std::string> expected(config->joint_order.begin(), config->joint_order.end());
      if (actual != expected) throw std::runtime_error("Robot Home joint set does not match active robot profile");
      for (const auto & name : config->joint_order) config->joints[name] = joints[name].as<double>();
    }
    for (const auto & entry : config->joints) {
      if (!std::isfinite(entry.second)) throw std::runtime_error("non-finite Robot Home value: " + entry.first);
    }
    return true;
  } catch (const std::exception & error) {
    if (detail) *detail = error.what();
    return false;
  }
}

RobotHomeWriteResult save_robot_home_to_environment_yaml(
  const std::string & path, const RobotHomeConfig & config)
{
  RobotHomeWriteResult result;
  try {
    RobotHomeConfig active;
    std::string load_error;
    if (!load_robot_home_from_environment_yaml(path, &active, &load_error)) throw std::runtime_error(load_error);
    if (config.joint_order != active.joint_order) throw std::runtime_error("Robot Home joint order does not match active robot profile");
    std::set<std::string> actual;
    for (const auto & entry : config.joints) {
      actual.insert(entry.first);
      if (!std::isfinite(entry.second)) throw std::runtime_error("non-finite Robot Home value: " + entry.first);
    }
    const std::set<std::string> expected(active.joint_order.begin(), active.joint_order.end());
    if (actual != expected) throw std::runtime_error("Robot Home joint set does not match active robot profile");

    YAML::Node yaml = YAML::LoadFile(path);
    YAML::Node joints(YAML::NodeType::Map);
    for (const auto & name : active.joint_order) joints[name] = config.joints.at(name);
    yaml["robot"]["home_joint_state"]["source"] = "user";
    yaml["robot"]["home_joint_state"]["joints"] = joints;
    result.backup_path = path + ".robot_home." +
      QDateTime::currentDateTimeUtc().toString("yyyyMMddHHmmsszzz").toStdString() + ".bak";
    std::filesystem::copy_file(path, result.backup_path, std::filesystem::copy_options::overwrite_existing);
    std::ofstream output(path);
    if (!output) throw std::runtime_error("could not open environment.yaml for writing");
    output << yaml << '\n';
    if (!output) throw std::runtime_error("could not write environment.yaml");
    result.ok = true;
    result.detail = path;
  } catch (const std::exception & error) {
    result.detail = error.what();
  }
  return result;
}
}  // namespace workcell_builder
