#pragma once

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace workcell_builder
{

struct CameraPose { double x{0.0}, y{0.0}, z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0}; };

struct WorkcellCamera {
  std::string name;
  std::string package_name{"realsense2_description"};
  std::string model{"d435i"};
  std::string xacro_path{"urdf/_d435i.urdf.xacro"};
  std::string parent_object{"world"};
  std::string parent_link{"world"};
  CameraPose pose;
  std::string optical_frame{"camera_color_optical_frame"};
  std::string depth_frame{"camera_depth_optical_frame"};
  std::string role{"detection_camera"};
  std::string runtime_driver{"metadata_only"};
};

struct CameraValidationResult {
  bool ok{true};
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

WorkcellCamera default_realsense_camera(const std::string & model);
CameraValidationResult validate_camera_attachment(const WorkcellCamera & camera, const std::vector<std::string> & scene_objects, const std::vector<std::string> & parent_links);
void serialize_cameras_to_yaml(YAML::Emitter * out, const std::vector<WorkcellCamera> & cameras);
std::vector<WorkcellCamera> parse_cameras_from_yaml(const YAML::Node & root);
std::string generate_camera_xacro_include(const WorkcellCamera & camera);
std::string generate_camera_fixed_joint(const WorkcellCamera & camera);

}  // namespace workcell_builder
