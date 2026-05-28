#include "workcell_layout_readiness.hpp"
#include "workcell_yaml_utils.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <string>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;

namespace workcell_builder {
namespace {

bool file_exists(const fs::path & path)
{
  boost::system::error_code ec;
  return fs::exists(path, ec) && !ec;
}

struct BoolField
{
  bool defined{false};
  bool valid{false};
  bool value{false};
};

BoolField read_bool_field(const YAML::Node & node)
{
  BoolField field;
  field.defined = node.IsDefined();
  field.valid = yaml_read_bool(node, &field.value);
  return field;
}

bool layout_item_is_fallback_only(const YAML::Node & item)
{
  const YAML::Node source_layer = yaml_map_key(item, "source_layer");
  const YAML::Node visual_source = yaml_map_key(item, "visual_source");
  const YAML::Node active_visual_source = yaml_map_key(item, "active_visual_source");
  const YAML::Node provenance = yaml_map_key(item, "provenance");
  const YAML::Node status = yaml_map_key(item, "status");

  std::string value;
  const YAML::Node candidates[] = {source_layer, visual_source, active_visual_source, provenance, status};
  for (const YAML::Node & candidate : candidates) {
    if (!yaml_read_string(candidate, &value)) continue;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (value == "primitive_fallback" || value == "static_fallback" ||
        value == "legacy_static_fallback" || value == "fallback" ||
        value == "preview_only" || value == "generated_urdf_visual") {
      return true;
    }
  }
  return false;
}

bool layout_item_is_effectively_editable(const YAML::Node & item)
{
  if (!item || !item.IsMap()) return false;
  if (layout_item_is_fallback_only(item)) return false;

  const BoolField locked = read_bool_field(yaml_map_key(item, "locked"));
  if (locked.defined && (!locked.valid || locked.value)) return false;

  const BoolField editable = read_bool_field(yaml_map_key(item, "editable"));
  if (editable.defined) return editable.valid && editable.value;

  return !locked.defined || (locked.valid && !locked.value);
}

}  // namespace

LayoutReadinessState derive_layout_state_model(
  const fs::path & scene_dir,
  std::size_t preview_item_count,
  bool path_match)
{
  LayoutReadinessState result;
  if (!path_match) {
    result.state = LayoutStateModel::PATH_MISMATCH;
    return result;
  }

  const fs::path layout = scene_dir / "layout" / "workcell_studio_layout.yaml";
  if (!file_exists(layout)) {
    result.state = preview_item_count == 0 ? LayoutStateModel::PREVIEW_UNAVAILABLE : LayoutStateModel::NO_LAYOUT_FILE;
    return result;
  }

  try {
    const YAML::Node node = YAML::LoadFile(layout.string());
    const YAML::Node items = yaml_map_key(node, "items");
    if (!items || !items.IsSequence() || items.size() == 0) {
      result.state = preview_item_count == 0 ? LayoutStateModel::EMPTY_LAYOUT : LayoutStateModel::PREVIEW_ONLY_AVAILABLE;
      return result;
    }

    for (const auto & item : items) {
      if (layout_item_is_effectively_editable(item)) ++result.editable_item_count;
    }
    result.state = result.editable_item_count > 0 ?
      LayoutStateModel::EDITABLE_LAYOUT_PRESENT :
      (preview_item_count == 0 ? LayoutStateModel::EMPTY_LAYOUT : LayoutStateModel::PREVIEW_ONLY_AVAILABLE);
    return result;
  } catch (const YAML::Exception &) {
    result.state = LayoutStateModel::INVALID_LAYOUT_YAML;
  } catch (const std::exception &) {
    result.state = LayoutStateModel::INVALID_LAYOUT_YAML;
  } catch (...) {
    result.state = LayoutStateModel::INVALID_LAYOUT_YAML;
  }
  return result;
}

bool save_layout_workflow_ready(
  const fs::path & scene_dir,
  const LayoutReadinessState & layout_state)
{
  return layout_state.state == LayoutStateModel::EDITABLE_LAYOUT_PRESENT &&
    layout_state.editable_item_count > 0 &&
    file_exists(scene_dir / "environment_layout.yaml") &&
    file_exists(scene_dir / "layout" / "workcell_studio_layout.yaml") &&
    file_exists(scene_dir / "environment.yaml");
}

}  // namespace workcell_builder
