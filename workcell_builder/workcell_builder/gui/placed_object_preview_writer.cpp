#include "placed_object_preview_writer.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace workcell_builder
{
namespace fs = std::filesystem;

std::string PlacedObjectPreviewWriter::default_preview_root() { return "/tmp/workcell_builder_preview"; }
std::string PlacedObjectPreviewWriter::interactive_preview_node_script() { return "scripts/workcell_builder_interactive_preview_node.py"; }

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
  if (!good_ext) result.warnings.push_back("mesh extension should be .stl/.dae/.obj");
  if (mesh_path.rfind("package://", 0) == 0) { result.valid_for_urdf = good_ext; return result; }
  fs::path p(mesh_path);
  if (p.is_absolute()) { result.warnings.push_back("absolute external path is discouraged for portable preview"); result.valid_for_urdf = false; return result; }
  fs::path resolved = fs::weakly_canonical(fs::path(repo_root) / p);
  if (resolved.string().find(repo_root) != 0) { result.warnings.push_back("relative path resolves outside repository root"); result.valid_for_urdf = false; return result; }
  if (!fs::exists(resolved)) result.warnings.push_back("mesh file does not exist on disk");
  result.valid_for_urdf = good_ext;
  return result;
}

bool PlacedObjectPreviewWriter::write_preview(const std::string & scene_name, const std::vector<PlacedObject> & objects, std::string * output_dir, std::vector<std::string> * warnings) const
{
  const std::string safe_scene = sanitize_scene_name(scene_name);
  fs::path out_dir = fs::path(default_preview_root()) / safe_scene;
  fs::create_directories(out_dir);

  std::ostringstream yaml;
  yaml << "scene_name: " << safe_scene << "\nplaced_objects:\n";
  std::ostringstream xacro;
  xacro << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"workcell_builder_preview\">\n  <link name=\"world\"/>\n";

  const std::string repo_root = fs::current_path().string();
  for (const auto & o : objects) {
    const auto mesh_check = validate_mesh_path(o.mesh_path, repo_root);
    yaml << "  - name: " << o.name << "\n    source: " << o.source_type << "\n    mesh: " << o.mesh_path << "\n";
    yaml << "    pose: [" << o.x << ", " << o.y << ", " << o.z << ", " << o.roll << ", " << o.pitch << ", " << o.yaw << "]\n";
    for (const auto & w : mesh_check.warnings) { yaml << "    warning: \"" << w << "\"\n"; if (warnings) warnings->push_back(o.name + ": " + w); }
    if (!mesh_check.valid_for_urdf) continue;
    const std::string lname = sanitize_object_name(o.name);
    xacro << "  <joint name=\"" << lname << "_joint\" type=\"fixed\">\n    <parent link=\"world\"/>\n    <child link=\"" << lname << "_link\"/>\n";
    xacro << "    <origin xyz=\"" << o.x << " " << o.y << " " << o.z << "\" rpy=\"" << o.roll << " " << o.pitch << " " << o.yaw << "\"/>\n  </joint>\n";
    xacro << "  <link name=\"" << lname << "_link\">\n    <visual><geometry><mesh filename=\"" << o.mesh_path << "\" scale=\"1 1 1\"/></geometry></visual>\n";
    xacro << "    <collision><geometry><mesh filename=\"" << o.mesh_path << "\" scale=\"1 1 1\"/></geometry></collision>\n  </link>\n";
  }
  xacro << "</robot>\n";

  std::ofstream(out_dir / "placed_objects_preview.yaml") << yaml.str();
  std::ofstream(out_dir / "placed_objects_preview.urdf.xacro") << xacro.str();

  std::ofstream(out_dir / "preview_scene.launch.py")
    << "from launch import LaunchDescription\nfrom launch_ros.actions import Node\nfrom launch.substitutions import Command\n"
    << "def generate_launch_description():\n"
    << "  xacro_path='" << (out_dir / "placed_objects_preview.urdf.xacro").string() << "'\n"
    << "  rviz_cfg='" << (out_dir / "workcell_builder_stl_preview.rviz").string() << "'\n"
    << "  return LaunchDescription([\n"
    << "    Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': Command(['xacro ', xacro_path])}]),\n"
    << "    Node(package='joint_state_publisher', executable='joint_state_publisher'),\n"
    << "    Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg]),\n"
    << "  ])\n";

  std::ofstream(out_dir / "workcell_builder_stl_preview.rviz") << "Panels:\n- Class: rviz_common/Displays\nVisualization Manager:\n  Displays:\n    - Class: rviz_default_plugins/Grid\n      Name: Grid\n    - Class: rviz_default_plugins/TF\n      Name: TF\n    - Class: rviz_default_plugins/RobotModel\n      Name: RobotModel\n";
  std::ofstream(out_dir / "workcell_builder_interactive_preview.rviz") << "Panels:\n- Class: rviz_common/Displays\nVisualization Manager:\n  Displays:\n    - Class: rviz_default_plugins/Grid\n      Name: Grid\n    - Class: rviz_default_plugins/TF\n      Name: TF\n    - Class: rviz_default_plugins/InteractiveMarkers\n      Name: InteractiveMarkers\n    - Class: rviz_default_plugins/RobotModel\n      Name: RobotModel\n";

  std::ofstream(out_dir / "interactive_preview.launch.py")
    << "from launch import LaunchDescription\nfrom launch_ros.actions import Node\n"
    << "def generate_launch_description():\n"
    << "  preview_dir='" << out_dir.string() << "'\n"
    << "  rviz_cfg='" << (out_dir / "workcell_builder_interactive_preview.rviz").string() << "'\n"
    << "  return LaunchDescription([\n"
    << "    Node(package='workcell_builder', executable='workcell_builder_interactive_preview_node.py', arguments=['--preview-dir', preview_dir]),\n"
    << "    Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg]),\n"
    << "  ])\n";

  std::ofstream(out_dir / "README_PREVIEW.md") << "# Workcell Builder STL Preview\n\nVisual-only offline preview. No MoveIt, controllers, trajectories, or real robot motion.\n\nRun:\n\nros2 launch " << (out_dir / "preview_scene.launch.py").string() << "\n";
  std::ofstream(out_dir / "README_INTERACTIVE_PREVIEW.md") << "# Workcell Builder Interactive RViz Preview\n\nOffline/visual-only interactive marker preview. No MoveIt, controllers, trajectory publishing, or real hardware.\n\nFeedback is written to placed_objects_feedback.yaml with safe_for_robot_motion: false.\n\nRun:\n\nros2 launch " << (out_dir / "interactive_preview.launch.py").string() << "\n";

  if (output_dir) *output_dir = out_dir.string();
  return true;
}

}  // namespace workcell_builder
