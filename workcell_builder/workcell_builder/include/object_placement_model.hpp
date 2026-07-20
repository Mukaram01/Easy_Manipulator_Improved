#pragma once

#include <string>
#include <vector>

namespace workcell_builder
{

struct PlacedObject
{
  std::string name;
  std::string source_type;  // asset_stl | generated_primitive | external_stl_warning
  std::string mesh_path;
  double x{0.0}, y{0.0}, z{0.0};
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  double scale_x{1.0}, scale_y{1.0}, scale_z{1.0};
  std::string parent_frame{"world"};
  std::string collision_mesh;
  bool collision_enabled{true};
  std::string status;
};

struct CameraPlacement
{
  std::string name{"camera_01"};
  std::string type{"realsense_d435i"};
  std::string source{"camera_asset"};
  std::string parent_frame{"world"};
  double x{0.0}, y{0.0}, z{0.0};
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  std::string link_frame{"camera_01_link"};
  std::string optical_frame{"camera_01_color_optical_frame"};
  std::string depth_frame{"camera_01_depth_optical_frame"};
  std::string color_topic{"/camera/color/image_raw"};
  std::string depth_topic{"/camera/depth/image_rect_raw"};
  std::string pointcloud_topic{"/camera/depth/color/points"};
  std::string camera_info_topic{"/camera/color/camera_info"};
  double horizontal_fov_deg{69.0};
  double vertical_fov_deg{42.0};
  double near_m{0.15};
  double far_m{1.5};
  bool enabled{true};
  std::string status;
};

struct TaskZone
{
  std::string id;
  std::string type;
  std::string role;
  std::string shape{"box"};
  std::string parent_frame{"world"};
  double x{0.0}, y{0.0}, z{0.0};
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  double dim_x{0.0}, dim_y{0.0}, dim_z{0.0};
  std::string frame_id;
  std::string camera_id;
  std::string support_surface_ref;
  std::string object_ref;
  std::string target_ref;
  std::vector<std::string> allowed_object_types;
  bool enabled{true};
  std::string status;
};

std::string sanitize_object_name(const std::string & name);
bool validate_placed_object(const PlacedObject & object, std::string * warning);
bool validate_camera_placement(const CameraPlacement & camera, std::string * warning);
std::string normalize_mesh_path_for_scene(const std::string & mesh_path);
std::string import_stl_to_asset_library(
  const std::string & stl_path,
  const std::string & repo_root,
  const std::string & managed_folder = "easy_manipulation_deployment/assets/environment/custom_meshes");
struct ObservationZoneSuggestion
{
  bool ok{false};
  TaskZone zone;
  std::vector<std::string> messages;
};

bool can_create_observation_zone_for_camera(
  const CameraPlacement * selected_camera, bool editable_scene, bool scene_writable);
ObservationZoneSuggestion suggest_camera_observation_zone(
  const CameraPlacement & camera, const std::vector<TaskZone> & existing_zones, double work_surface_z = 0.0);

std::string serialize_placed_objects_to_environment_yaml(const std::vector<PlacedObject> & objects);
std::vector<PlacedObject> parse_placed_objects_from_environment_yaml(const std::string & content);
bool save_environment_layout(const std::string & output_path, const std::vector<PlacedObject> & objects);
std::vector<PlacedObject> load_environment_layout(const std::string & input_path);

class ObjectPlacementModel
{
public:
  void add_object(const PlacedObject & object);
  bool duplicate_object(const std::string & name);
  bool remove_object(const std::string & name);
  bool update_object_pose(
    const std::string & name, double x, double y, double z, double roll, double pitch, double yaw,
    std::string * warning = nullptr);
  std::vector<PlacedObject> objects() const;

private:
  PlacedObject * find_object_by_name(const std::string & name);
  std::vector<PlacedObject> objects_;
};

}  // namespace workcell_builder
