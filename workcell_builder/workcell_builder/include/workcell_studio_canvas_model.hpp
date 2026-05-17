#pragma once
#include <boost/filesystem.hpp>
#include <string>
#include <vector>

namespace workcell_builder {
struct WorkcellStudioCanvasItem {
  std::string id; std::string type; std::string role; std::string label; std::string source_file;
  double x{0.0}, y{0.0}, z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0}, width{0.25}, depth{0.25}, height{0.25}, radius{0.0};
  std::string mesh_path;
  std::string mesh_type;
  double mesh_scale_x{1.0}, mesh_scale_y{1.0}, mesh_scale_z{1.0};
  double mesh_r{0.0}, mesh_p{0.0}, mesh_y{0.0};
  bool mesh_available{false};
  std::string mesh_load_warning;
  bool locked{false};
  std::vector<std::string> warnings;
};
struct WorkcellStudioCanvasModel {
  std::string scene_name, template_name, robot_summary, tool_summary, status;
  bool fake_hardware_first{true}, no_robot_motion{true}, has_warnings{false};
  std::string pick_source, grasp_strategy, place_target, release_strategy;
  std::vector<std::string> warnings; std::vector<WorkcellStudioCanvasItem> items;
};
WorkcellStudioCanvasModel build_workcell_studio_canvas_model(const boost::filesystem::path & scene_dir, const std::string & scene_name);
}
