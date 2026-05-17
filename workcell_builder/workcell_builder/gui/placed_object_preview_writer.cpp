#include "placed_object_preview_writer.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace workcell_builder
{
namespace fs = std::filesystem;

std::string PlacedObjectPreviewWriter::default_preview_root() { return "/tmp/workcell_builder_preview"; }

std::string PlacedObjectPreviewWriter::sanitize_scene_name(const std::string & scene_name)
{
  return sanitize_object_name(scene_name.empty() ? "default_scene" : scene_name);
}

MeshValidationResult PlacedObjectPreviewWriter::validate_mesh_path(const std::string & mesh_path, const std::string & repo_root) const
{
  MeshValidationResult result;
  if (mesh_path.empty()) {
    result.warnings.push_back("mesh path is empty");
    return result;
  }

  std::string lower = mesh_path;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  const bool good_ext = (lower.size() >= 4 && (lower.rfind(".stl") == lower.size()-4 || lower.rfind(".dae") == lower.size()-4 || lower.rfind(".obj") == lower.size()-4));
  if (!good_ext) {
    result.warnings.push_back("mesh extension should be .stl/.dae/.obj");
  }

  if (mesh_path.rfind("package://", 0) == 0) {
    result.valid_for_urdf = good_ext;
    return result;
  }

  fs::path p(mesh_path);
  if (p.is_absolute()) {
    result.warnings.push_back("absolute external path is discouraged for portable preview");
    result.valid_for_urdf = false;
    return result;
  }

  fs::path repo = fs::path(repo_root);
  fs::path resolved = fs::weakly_canonical(repo / p);
  if (resolved.string().find(repo.string()) != 0) {
    result.warnings.push_back("relative path resolves outside repository root");
    result.valid_for_urdf = false;
    return result;
  }
  if (!fs::exists(resolved)) {
    result.warnings.push_back("mesh file does not exist on disk");
  }
  result.valid_for_urdf = good_ext;
  return result;
}

bool PlacedObjectPreviewWriter::write_preview(const std::string & scene_name, const std::vector<PlacedObject> & objects, std::string * output_dir, std::vector<std::string> * warnings) const
{
  const std::string root = default_preview_root();
  const std::string safe_scene = sanitize_scene_name(scene_name);
  fs::path out_dir = fs::path(root) / safe_scene;
  fs::create_directories(out_dir);

  const std::string repo_root = fs::current_path().string();
  std::ostringstream yaml;
  yaml << "scene_name: " << safe_scene << "\nplaced_objects:\n";

  std::ostringstream xacro;
  xacro << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"workcell_builder_preview\">\n";
  xacro << "  <link name=\"world\"/>\n";

  for (const auto & o : objects) {
    const auto mesh_check = validate_mesh_path(o.mesh_path, repo_root);
    yaml << "  - name: " << o.name << "\n";
    yaml << "    source: " << o.source_type << "\n";
    yaml << "    mesh: " << o.mesh_path << "\n";
    yaml << "    pose: [" << o.x << ", " << o.y << ", " << o.z << ", " << o.roll << ", " << o.pitch << ", " << o.yaw << "]\n";
    for (const auto & w : mesh_check.warnings) {
      yaml << "    warning: \"" << w << "\"\n";
      if (warnings) warnings->push_back(o.name + ": " + w);
    }

    if (!mesh_check.valid_for_urdf) {
      continue;
    }
    const std::string lname = sanitize_object_name(o.name);
    xacro << "  <joint name=\"" << lname << "_joint\" type=\"fixed\">\n";
    xacro << "    <parent link=\"world\"/>\n";
    xacro << "    <child link=\"" << lname << "_link\"/>\n";
    xacro << "    <origin xyz=\"" << o.x << " " << o.y << " " << o.z << "\" rpy=\"" << o.roll << " " << o.pitch << " " << o.yaw << "\"/>\n";
    xacro << "  </joint>\n";
    xacro << "  <link name=\"" << lname << "_link\">\n";
    xacro << "    <visual><geometry><mesh filename=\"" << o.mesh_path << "\" scale=\"1 1 1\"/></geometry></visual>\n";
    xacro << "    <collision><geometry><mesh filename=\"" << o.mesh_path << "\" scale=\"1 1 1\"/></geometry></collision>\n";
    xacro << "  </link>\n";
  }
  xacro << "</robot>\n";

  std::ofstream(out_dir / "placed_objects_preview.yaml") << yaml.str();
  std::ofstream(out_dir / "placed_objects_preview.urdf.xacro") << xacro.str();

  std::ostringstream launch;
  launch << "from launch import LaunchDescription\n";
  launch << "from launch_ros.actions import Node\n";
  launch << "from launch.substitutions import Command\n";
  launch << "def generate_launch_description():\n";
  launch << "  xacro_path='" << (out_dir / "placed_objects_preview.urdf.xacro").string() << "'\n";
  launch << "  rviz_cfg='" << (out_dir / "workcell_builder_stl_preview.rviz").string() << "'\n";
  launch << "  return LaunchDescription([\n";
  launch << "    Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': Command(['xacro ', xacro_path])}]),\n";
  launch << "    Node(package='joint_state_publisher', executable='joint_state_publisher'),\n";
  launch << "    Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg]),\n";
  launch << "  ])\n";
  std::ofstream(out_dir / "preview_scene.launch.py") << launch.str();

  std::ofstream(out_dir / "README_PREVIEW.md")
    << "# Workcell Builder STL Preview\n\n"
    << "Visual-only offline preview. No MoveIt, controllers, trajectories, or real robot motion.\n\n"
    << "Run:\n\nros2 launch " << (out_dir / "preview_scene.launch.py").string() << "\n";

  std::ofstream(out_dir / "workcell_builder_stl_preview.rviz")
    << "Panels:\n- Class: rviz_common/Displays\nVisualization Manager:\n  Displays:\n    - Class: rviz_default_plugins/Grid\n      Name: Grid\n    - Class: rviz_default_plugins/TF\n      Name: TF\n    - Class: rviz_default_plugins/RobotModel\n      Name: RobotModel\n";

  if (output_dir) *output_dir = out_dir.string();
  return true;
}

}  // namespace workcell_builder
