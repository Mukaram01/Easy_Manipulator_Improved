#include "placed_object_urdf_render.hpp"

#include <sstream>

namespace workcell_builder
{

std::string render_placed_object_urdf_snippet(const PlacedObject & object, const std::string & link_name)
{
  const std::string parent_frame = object.parent_frame.empty() ? "world" : object.parent_frame;
  const std::string collision_mesh = object.collision_mesh.empty() ? object.mesh_path : object.collision_mesh;

  std::ostringstream urdf;
  urdf << "  <joint name=\"" << link_name << "_joint\" type=\"fixed\">\n"
       << "    <parent link=\"" << parent_frame << "\"/>\n"
       << "    <child link=\"" << link_name << "_link\"/>\n"
       << "    <origin xyz=\"" << object.x << " " << object.y << " " << object.z
       << "\" rpy=\"" << object.roll << " " << object.pitch << " " << object.yaw << "\"/>\n"
       << "  </joint>\n"
       << "  <link name=\"" << link_name << "_link\">\n"
       << "    <visual><geometry><mesh filename=\"" << object.mesh_path << "\" scale=\""
       << object.scale_x << " " << object.scale_y << " " << object.scale_z << "\"/></geometry></visual>\n";
  if (object.collision_enabled && !collision_mesh.empty()) {
    urdf << "    <collision><geometry><mesh filename=\"" << collision_mesh << "\" scale=\""
         << object.scale_x << " " << object.scale_y << " " << object.scale_z << "\"/></geometry></collision>\n";
  }
  urdf << "  </link>\n";
  return urdf.str();
}

}  // namespace workcell_builder
