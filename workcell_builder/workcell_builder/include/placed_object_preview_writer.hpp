#pragma once

#include <string>
#include <vector>

#include "object_placement_model.hpp"

namespace workcell_builder
{

struct MeshValidationResult
{
  bool valid_for_urdf{false};
  std::vector<std::string> warnings;
};

class PlacedObjectPreviewWriter
{
public:
  static std::string default_preview_root();
  static std::string sanitize_scene_name(const std::string & scene_name);
  static std::string interactive_preview_node_script();

  bool write_preview(
    const std::string & scene_name,
    const std::vector<PlacedObject> & objects,
    std::string * output_dir,
    std::vector<std::string> * warnings) const;

private:
  MeshValidationResult validate_mesh_path(const std::string & mesh_path, const std::string & repo_root) const;
};

}  // namespace workcell_builder
