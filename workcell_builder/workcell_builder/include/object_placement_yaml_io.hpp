#pragma once

#include <string>
#include <vector>

#include "object_placement_model.hpp"

namespace workcell_builder
{

struct PlacedObjectYamlWriteResult
{
  bool ok{false};
  std::string path_written;
  std::vector<std::string> warnings;
  size_t objects_saved{0};
};

struct RobotToolPoseConfig
{
  double robot_base_xyz[3]{0.0, 0.0, 0.0};
  double robot_base_rpy[3]{0.0, 0.0, 0.0};
  double tool_attach_xyz[3]{0.0, 0.0, 0.0};
  double tool_attach_rpy[3]{0.0, 0.0, 0.0};
  std::string tool_link_id;
  std::string tool_joint_id;
};

std::vector<PlacedObject> load_placed_objects_from_environment_yaml(
  const std::string & environment_yaml_path,
  std::vector<std::string> * warnings = nullptr);

PlacedObjectYamlWriteResult save_placed_objects_to_environment_yaml(
  const std::string & environment_yaml_path,
  const std::vector<PlacedObject> & objects);

std::vector<CameraPlacement> load_camera_placements_from_environment_yaml(
  const std::string & environment_yaml_path,
  std::vector<std::string> * warnings = nullptr);

PlacedObjectYamlWriteResult save_camera_placements_to_environment_yaml(
  const std::string & environment_yaml_path,
  const std::vector<CameraPlacement> & cameras);

std::vector<TaskZone> load_task_zones_from_environment_yaml(
  const std::string & environment_yaml_path,
  std::vector<std::string> * warnings = nullptr);

PlacedObjectYamlWriteResult save_task_zones_to_environment_yaml(
  const std::string & environment_yaml_path,
  const std::vector<TaskZone> & task_zones);

RobotToolPoseConfig load_robot_tool_pose_from_environment_yaml(
  const std::string & environment_yaml_path,
  std::vector<std::string> * warnings = nullptr);

PlacedObjectYamlWriteResult save_robot_tool_pose_to_environment_yaml(
  const std::string & environment_yaml_path,
  const RobotToolPoseConfig & config);

}  // namespace workcell_builder
