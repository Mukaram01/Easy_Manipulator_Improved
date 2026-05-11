#include "deployment_compatibility_profile.hpp"

namespace workcell_builder
{
RobotDriverRequirement collect_driver_requirements(const RobotProfile & p){ RobotDriverRequirement out; out.real_driver_required=p.real_driver_required; out.driver_package_hint=p.driver_package_hint; out.controller_type_hint=p.controller_type_hint; out.ros2_control_required=p.ros2_control_required; out.network_required=p.network_required; out.calibration_required=p.calibration_required; return out; }
ToolIoRequirement collect_tool_io_requirements(const ToolProfile & p){ ToolIoRequirement out; out.io_required=p.io_required; out.io_type=p.io_type; out.open_command_hint=p.open_command_hint; out.close_command_hint=p.close_command_hint; out.release_command_hint=p.release_command_hint; out.real_hardware_io_mapping_required=p.real_hardware_io_mapping_required; return out; }
CameraCalibrationRequirement collect_camera_calibration_requirements(const CameraProfile & p){ CameraCalibrationRequirement out; out.calibration_required=p.calibration_required; out.real_camera_driver_required=p.real_camera_driver_required; out.driver_package_hint=p.driver_package_hint; out.expected_topics={p.rgb_topic,p.depth_topic,p.camera_info_topic,p.pointcloud_topic}; return out; }
EpdCompatibilityRequirement evaluate_epd_metadata_compatibility(const CameraProfile & p){ EpdCompatibilityRequirement out; out.camera_frame=p.frame_id; out.rgb_topic=p.rgb_topic; out.depth_topic=p.depth_topic; out.camera_info_topic=p.camera_info_topic; out.pointcloud_topic=p.pointcloud_topic; out.expected_epd_input_hint=p.epd_input_hint; out.status=(p.epd_compatible=="true"||p.epd_compatible=="metadata_only")?"EPD_METADATA_READY":"EPD_METADATA_INCOMPLETE"; return out; }
DeploymentCompatibilityResult evaluate_sim_to_real_compatibility(const Scene &, const std::string &){ DeploymentCompatibilityResult out; out.status="REAL_HARDWARE_METADATA_READY"; out.real_hardware_metadata_status="REAL_HARDWARE_DRIVER_REQUIRED"; out.real_hardware_ready=false; return out; }
std::string deployment_status_label(const DeploymentCompatibilityResult & r){ return r.status; }
}
