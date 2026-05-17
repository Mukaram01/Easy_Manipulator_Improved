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

struct TaskZone
{
  std::string id;
  std::string type{"pick"};
  double x{0.0}, y{0.0}, z{0.0};
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  double size_x{0.2}, size_y{0.2}, size_z{0.2};
  std::string frame{"world"};
  std::string status;
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

}  // namespace workcell_builder
