#pragma once
#include <string>
#include <vector>

namespace workcell_builder
{
struct CameraProfile
{
  std::string camera_id{"realsense_d435i"};
  std::string label{"RealSense D435i"};
  std::string camera_type{"rgbd"};
  std::string frame_id{"camera_link"};
  std::string optical_frame_id{"camera_color_optical_frame"};
  std::string rgb_topic{"/camera/camera/color/image_raw"};
  std::string depth_topic{"/camera/camera/depth/image_rect_raw"};
  std::string camera_info_topic{"/camera/camera/color/camera_info"};
  std::string pointcloud_topic{"/camera/camera/depth/color/points"};
  std::vector<double> pose{0.0, 0.0, 1.2, 0.0, 0.0, 0.0};
  std::string mount_type{"fixed"};
  std::string perception_hint{"rgbd_object_detection"};
  std::string epd_input_hint{"external_epd_adapter"};
  bool calibration_required{true};
  bool real_camera_driver_required{true};
  std::string driver_package_hint{"realsense2_camera"};
  std::string epd_compatible{"metadata_only"};
  std::string deployment_notes{"EPD metadata export only"};
  std::vector<std::string> notes;
  std::vector<std::string> warnings;
};
using CameraPlacement = CameraProfile;
using PerceptionMetadata = CameraProfile;
struct CameraValidationResult { bool ok{true}; std::vector<std::string> warnings; std::vector<std::string> blockers; };

std::vector<CameraProfile> load_camera_profiles(const std::string & config_root);
CameraProfile default_realsense_d435i_profile();
CameraValidationResult validate_camera_placement(const CameraProfile & profile, bool strict);
void normalize_camera_topics(CameraProfile & profile);
std::string camera_status_label(const CameraValidationResult & result);
}  // namespace workcell_builder
