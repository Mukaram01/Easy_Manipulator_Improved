#ifndef WORKCELL_BUILDER__LAYOUT_ITEM_SERIALIZER_HPP_
#define WORKCELL_BUILDER__LAYOUT_ITEM_SERIALIZER_HPP_

#include <yaml-cpp/yaml.h>

#include <array>
#include <string>

namespace workcell_builder
{
struct LayoutItemSaveState
{
  std::string id, display_name, type, category, role, catalog_asset_id, mesh_path, source_package;
  std::array<double, 3> xyz{}, rpy{}, dimensions{};
  std::array<double, 3> mesh_scale{{1.0, 1.0, 1.0}};
};

inline bool is_scene_layer_token(const std::string & value)
{
  return value == "editable_layout" || value == "mesh_preview" ||
         value == "locked_generated_urdf_visual";
}

inline bool is_scene_relative_asset_path(const std::string & value)
{
  return value.rfind("assets/", 0) == 0 || value.rfind("./assets/", 0) == 0;
}

inline YAML::Node layout_string_scalar(const std::string & value)
{
  YAML::Node node(value);
  node.SetTag("tag:yaml.org,2002:str");
  return node;
}

inline YAML::Node layout_sequence3(const std::array<double, 3> & values)
{
  YAML::Node sequence(YAML::NodeType::Sequence);
  for (const double value : values) sequence.push_back(value);
  return sequence;
}

inline void update_layout_item_pose(YAML::Node item, const LayoutItemSaveState & state)
{
  YAML::Node pose = item["pose"];
  if (!pose || !pose.IsMap()) {pose = YAML::Node(YAML::NodeType::Map); item["pose"] = pose;}
  const bool scalar_xyz = pose["x"] || pose["y"] || pose["z"];
  const bool scalar_rpy = pose["roll"] || pose["pitch"] || pose["yaw"];
  if (scalar_xyz && !pose["xyz"]) {
    pose["x"] = state.xyz[0]; pose["y"] = state.xyz[1]; pose["z"] = state.xyz[2];
  } else {pose["xyz"] = layout_sequence3(state.xyz);}
  if (scalar_rpy && !pose["rpy"]) {
    pose["roll"] = state.rpy[0]; pose["pitch"] = state.rpy[1]; pose["yaw"] = state.rpy[2];
  } else {pose["rpy"] = layout_sequence3(state.rpy);}
}

inline void update_layout_item_dimensions(YAML::Node item, const LayoutItemSaveState & state)
{
  YAML::Node size = item["size"];
  if (size && size.IsMap() && !item["dimensions"]) {
    size["width"] = state.dimensions[0]; size["depth"] = state.dimensions[1];
    size["height"] = state.dimensions[2];
  } else {item["dimensions"] = layout_sequence3(state.dimensions);}
}

// Existing YAML is authored authority; Scene3D semantics/provenance are display-normalized.
inline YAML::Node serialize_layout_item(
  const LayoutItemSaveState & state, const YAML::Node & existing,
  const bool metadata_explicitly_edited = false)
{
  const bool existing_record = existing && existing.IsMap();
  YAML::Node item = existing_record ? YAML::Clone(existing) : YAML::Node(YAML::NodeType::Map);
  if (!existing_record) {
    item["id"] = layout_string_scalar(state.id);
    if (!state.display_name.empty()) item["display_name"] = layout_string_scalar(state.display_name);
    if (!state.type.empty()) item["type"] = state.type;
    if (!state.category.empty()) item["category"] = state.category;
    if (!state.role.empty()) item["role"] = state.role;
    if (!state.catalog_asset_id.empty()) item["catalog_asset_id"] = layout_string_scalar(state.catalog_asset_id);
    item["source"] = "layout/workcell_studio_layout.yaml";
    item["source_layer"] = "editable_layout";
    item["editable"] = true; item["locked"] = false;
    if (!state.mesh_path.empty() && !is_scene_layer_token(state.mesh_path)) {
      item["source_path"] = state.mesh_path; item["mesh_path"] = state.mesh_path;
      YAML::Node mesh(YAML::NodeType::Map); mesh["path"] = state.mesh_path;
      if (!is_scene_relative_asset_path(state.mesh_path) &&
        !state.source_package.empty() && !is_scene_layer_token(state.source_package))
        mesh["source_package"] = state.source_package;
      mesh["scale"] = layout_sequence3(state.mesh_scale); item["mesh"] = mesh;
    }
    item["scale"] = layout_sequence3(state.mesh_scale);
  } else if (metadata_explicitly_edited) {
    if (!state.display_name.empty()) item["display_name"] = layout_string_scalar(state.display_name);
    if (!state.role.empty()) item["role"] = state.role;
  }
  if (existing_record && !state.catalog_asset_id.empty() && !state.mesh_path.empty() &&
    !is_scene_layer_token(state.mesh_path))
  {
    item["source_path"] = state.mesh_path;
    item["mesh_path"] = state.mesh_path;
    YAML::Node mesh = item["mesh"];
    if (!mesh || !mesh.IsMap()) mesh = YAML::Node(YAML::NodeType::Map);
    mesh["path"] = state.mesh_path;
    if (is_scene_relative_asset_path(state.mesh_path)) {
      mesh.remove("source_package");
    } else if (!state.source_package.empty() && !is_scene_layer_token(state.source_package)) {
      mesh["source_package"] = state.source_package;
    }
    mesh["scale"] = layout_sequence3(state.mesh_scale);
    item["mesh"] = mesh;
  }
  update_layout_item_pose(item, state);
  update_layout_item_dimensions(item, state);
  return item;
}
}  // namespace workcell_builder

#endif
