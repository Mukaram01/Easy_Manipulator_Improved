#pragma once
#include <string>
#include <vector>

#include "attributes/workcell.h"
#include "camera_perception_profile.hpp"
#include "robot_tool_compatibility.hpp"

namespace workcell_builder
{
struct RobotDriverRequirement { bool real_driver_required{true}; std::string driver_package_hint; std::string controller_type_hint; bool ros2_control_required{true}; bool network_required{true}; bool calibration_required{false}; };
struct HardwareInterfaceRequirement { bool fake_hardware_first{true}; bool real_hardware_enabled{false}; bool runtime_execution_enabled{false}; bool motion_command_sent{false}; bool moveit_plan_service_called{false}; };
struct ToolIoRequirement { bool io_required{false}; std::string io_type{"unknown"}; std::string open_command_hint; std::string close_command_hint; std::string release_command_hint; bool real_hardware_io_mapping_required{false}; };
struct CameraCalibrationRequirement { bool calibration_required{true}; bool real_camera_driver_required{true}; std::string driver_package_hint; std::vector<std::string> expected_topics; };
struct EpdCompatibilityRequirement { std::string status{"EPD_METADATA_INCOMPLETE"}; std::string camera_frame; std::string rgb_topic; std::string depth_topic; std::string camera_info_topic; std::string pointcloud_topic; std::string expected_epd_input_hint; std::string adapter_metadata_file{"config/epd_adapter_metadata.json"}; std::vector<std::string> object_label_hints; };
struct DeploymentCompatibilityResult { bool simulation_ready{true}; bool real_hardware_ready{false}; std::string real_hardware_metadata_status{"REAL_HARDWARE_CONFIG_INCOMPLETE"}; std::string epd_metadata_status{"EPD_METADATA_INCOMPLETE"}; std::string status{"SIM_READY"}; RobotDriverRequirement robot_driver_requirement; ToolIoRequirement tool_io_requirement; CameraCalibrationRequirement camera_calibration_requirement; EpdCompatibilityRequirement epd_compatibility; HardwareInterfaceRequirement safety_flags; std::vector<std::string> warnings; std::vector<std::string> blockers; };
DeploymentCompatibilityResult evaluate_sim_to_real_compatibility(const Scene & scene, const std::string & config_root);
EpdCompatibilityRequirement evaluate_epd_metadata_compatibility(const CameraProfile & profile);
RobotDriverRequirement collect_driver_requirements(const RobotProfile & profile);
ToolIoRequirement collect_tool_io_requirements(const ToolProfile & profile);
CameraCalibrationRequirement collect_camera_calibration_requirements(const CameraProfile & profile);
std::string deployment_status_label(const DeploymentCompatibilityResult & result);
}

// status labels: SIM_READY REAL_HARDWARE_METADATA_READY REAL_HARDWARE_DRIVER_REQUIRED REAL_HARDWARE_CONFIG_INCOMPLETE EPD_METADATA_READY EPD_METADATA_INCOMPLETE UNSUPPORTED_ROBOT_FAMILY DELTA_ROBOT_METADATA_ONLY
