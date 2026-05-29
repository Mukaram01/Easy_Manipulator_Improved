#include "workcell_studio_canvas_model.hpp"
#include "workcell_yaml_utils.hpp"
#include "workcell_warning_once.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <set>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;
namespace workcell_builder {

struct YamlLoadStatus
{
  bool exists{false};
  bool loaded{false};
  bool parse_warning{false};
  std::string reason;
};

static YamlLoadStatus read_yaml(const fs::path & p, YAML::Node * out)
{
  YamlLoadStatus status;
  status.exists = fs::exists(p);
  if (!status.exists) return status;
  try {
    *out = YAML::LoadFile(p.string());
    status.loaded = true;
    return status;
  } catch (const YAML::Exception & e) {
    status.parse_warning = true;
    status.reason = e.what();
  } catch (const std::exception & e) {
    status.parse_warning = true;
    status.reason = e.what();
  } catch (...) {
    status.parse_warning = true;
    status.reason = "unknown exception";
  }
  workcell_builder::log_warning_once_per_context_path_reason(
    "task_metadata_summary_loader", p, "scene YAML parse warning: " + status.reason);
  return status;
}

static void add_mesh_candidate(const YAML::Node & node, std::vector<std::string> * out)
{
  if (!node.IsScalar()) return;
  const std::string text = node.as<std::string>("");
  if (text.empty()) return;
  if (text.rfind("package://", 0) == 0 || text.find(".stl") != std::string::npos || text.find("meshes/") != std::string::npos) out->push_back(text);
}

static bool mesh_node_disabled(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) return false;
  std::string value;
  if (!yaml_read_string(node, &value)) return false;
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return value == "none" || value == "disabled" || value == "false";
}

static std::string provenance_summary_text(const WorkcellStudioProvenanceStatus & status)
{
  return "Editable layout: " + std::to_string(status.editable_layout_count) + " items. Preview fallback: " +
    std::to_string(status.generated_or_legacy_preview_count + status.static_fallback_preview_count) +
    " items loaded from scene metadata.";
}

static std::vector<std::string> gather_mesh_candidates(const YAML::Node & env, const YAML::Node & manifest, const YAML::Node & layout_items, const std::string & item_id)
{
  std::vector<std::string> out;
  const auto scan_fields = [&out](const YAML::Node & n) {
    const YAML::Node mesh = yaml_map_key(n, "mesh");
    if (mesh && mesh.IsMap()) add_mesh_candidate(optional_scalar(mesh, "path"), &out);
    else add_mesh_candidate(mesh, &out);
    add_mesh_candidate(optional_scalar(n, "mesh_path"), &out);
    add_mesh_candidate(yaml_map_key(n, "visual_mesh"), &out);
    add_mesh_candidate(yaml_map_key(n, "collision_mesh"), &out);
    YAML::Node v = yaml_map_key(yaml_map_key(n, "visual"), "geometry");
    add_mesh_candidate(yaml_map_key(v, "filepath"), &out);
    add_mesh_candidate(yaml_map_key(v, "mesh"), &out);
  };
  scan_fields(env);
  scan_fields(manifest);
  if (layout_items.IsSequence()) {
    for (const auto & node : layout_items) {
      if (!node.IsMap()) continue;
      if (yaml_map_value_or_empty(node, "id") != item_id) continue;
      scan_fields(node);
      YAML::Node metadata = yaml_map_key(node, "metadata");
      scan_fields(metadata);
    }
  }
  return out;
}

static fs::path resolve_mesh_candidate(const std::string & c, const fs::path & scene_dir)
{
  fs::path path(c);
  if (c.rfind("package://", 0) == 0) {
    const std::string rest = c.substr(std::string("package://").size());
    path = scene_dir / "assets" / rest;
  } else if (!path.is_absolute()) {
    path = scene_dir / path;
  }
  return path;
}

static void probe_mesh_candidates(const fs::path & scene_dir, std::vector<fs::path> * visuals, std::vector<fs::path> * collisions)
{
  const std::vector<fs::path> roots = {
    scene_dir / "assets" / "environment_objects",
    scene_dir / "assets" / "robots",
    scene_dir / "assets" / "end_effectors"
  };
  for (const auto & root : roots) {
    if (!fs::exists(root)) continue;
    for (fs::recursive_directory_iterator it(root), end; it != end; ++it) {
      if (!fs::is_regular_file(it->path()) || it->path().extension() != ".stl") continue;
      const std::string p = it->path().generic_string();
      if (p.find("/meshes/visual/") != std::string::npos) visuals->push_back(it->path());
      else if (p.find("/meshes/collision/") != std::string::npos) collisions->push_back(it->path());
      else if (p.find("/assets/robots/") != std::string::npos || p.find("/assets/end_effectors/") != std::string::npos) visuals->push_back(it->path());
    }
  }
}

WorkcellStudioCanvasModel build_workcell_studio_canvas_model(const fs::path & scene_dir, const std::string & scene_name)
{
  try {
  WorkcellStudioCanvasModel m; m.scene_name = scene_name; m.status = "WARNINGS";
  std::string deterministic_fallback_reason;
  bool deterministic_fallback_layout = false;
  bool warned_incomplete_placement_metadata = false;
  const auto enable_deterministic_fallback = [&](const std::string & reason) {
    if (deterministic_fallback_layout) return;
    deterministic_fallback_layout = true;
    deterministic_fallback_reason = reason;
    workcell_builder::log_warning_once_per_context_path_reason("task_metadata_summary_loader", scene_dir / "layout" / "workcell_studio_layout.yaml", "downgraded to legacy mode");
  };
  const auto add_warning = [&m](const std::string & context, const std::string & detail) {
    m.warnings.push_back("layout/workcell_studio_layout.yaml [" + context + "]: " + detail);
    m.has_warnings = true;
  };
  const auto read_string_or_warn = [&add_warning](const YAML::Node & node, const std::string & context, const std::string & fallback) {
    std::string out;
    if (yaml_read_string(node, &out)) return out;
    if (node.IsDefined()) add_warning(context, "expected scalar string; using default");
    return fallback;
  };
  const auto read_double_or_warn = [&add_warning](const YAML::Node & node, const std::string & context, double fallback) {
    double out = fallback;
    if (yaml_read_double(node, &out)) return out;
    if (node.IsDefined()) add_warning(context, "expected scalar number; using default");
    return fallback;
  };

  YAML::Node env, manifest, task, layout;
  const fs::path env_path = scene_dir / "environment.yaml";
  const fs::path manifest_path = scene_dir / "scene_manifest.yaml";
  const fs::path task_path = scene_dir / "config" / "task_recipe.yaml";
  const fs::path layout_path = scene_dir / "layout" / "workcell_studio_layout.yaml";
  const YamlLoadStatus env_status = read_yaml(env_path, &env);
  const YamlLoadStatus manifest_status = read_yaml(manifest_path, &manifest);
  const YamlLoadStatus task_status = read_yaml(task_path, &task);
  const YamlLoadStatus layout_status = read_yaml(layout_path, &layout);
  const bool env_ok = env_status.loaded;
  const bool manifest_ok = manifest_status.loaded;
  const bool task_ok = task_status.loaded;
  const bool layout_ok = layout_status.loaded;
  if (!layout_ok) {
    if (layout_status.exists) enable_deterministic_fallback("layout/workcell_studio_layout.yaml is malformed");
    else enable_deterministic_fallback("layout/workcell_studio_layout.yaml is missing");
  }
  if (!env_ok) m.warnings.push_back(env_status.parse_warning ?
    ("environment.yaml parse warning (" + env_path.string() + "): " + env_status.reason + ". Validate YAML syntax (e.g., `yamllint`) and retry.") :
    "Malformed or missing environment.yaml");
  if (!manifest_ok) m.warnings.push_back(manifest_status.parse_warning ?
    ("scene_manifest.yaml parse warning (" + manifest_path.string() + "): " + manifest_status.reason + ". Validate YAML syntax (e.g., `yamllint`) and retry.") :
    "Missing scene_manifest.yaml");
  if (!task_ok) m.warnings.push_back(task_status.parse_warning ?
    ("task_recipe.yaml parse warning (" + task_path.string() + "): " + task_status.reason + ". Validate YAML syntax (e.g., `yamllint`) and retry.") :
    "Task intent missing");
  if (!layout_ok && layout_status.exists) {
    const std::string parse_reason = layout_status.parse_warning ? (": " + layout_status.reason) : "";
    m.warnings.push_back("Malformed layout/workcell_studio_layout.yaml (" + layout_path.string() + ")" + parse_reason +
                         "; falling back safely. Repair guidance: run YAML validation (e.g., `yamllint`) and fix syntax/indentation.");
  }

  m.template_name = manifest_ok ? yaml_map_value_or_empty(manifest, "template_name") : "";
  if (m.template_name.empty()) m.template_name = "unknown_template";
  m.robot_summary = env_ok ? yaml_named_or_scalar(env["robot"], "name") : "";
  if (m.robot_summary.empty()) m.robot_summary = "Robot";
  m.tool_summary = env_ok ? yaml_named_or_scalar(env["end_effector"], "name") : "";
  if (m.tool_summary.empty()) m.tool_summary = "Tool";
  m.pick_source = task_ok ? read_string_or_warn(yaml_map_key(task, "pick_source"), "task_recipe.pick_source", "Task intent missing") : "Task intent missing";
  m.grasp_strategy = task_ok ? read_string_or_warn(yaml_map_key(task, "grasp_strategy"), "task_recipe.grasp_strategy", "Generate task recipe to populate this panel") : "Generate task recipe to populate this panel";
  m.place_target = task_ok ? read_string_or_warn(yaml_map_key(task, "place_target"), "task_recipe.place_target", "Task intent missing") : "Task intent missing";
  m.release_strategy = task_ok ? read_string_or_warn(yaml_map_key(task, "release_strategy"), "task_recipe.release_strategy", "Generate task recipe to populate this panel") : "Generate task recipe to populate this panel";

  const auto push_default_item = [&m](const std::string & id, const std::string & type, const std::string & role,
                                       const std::string & label, const std::string & source_file,
                                       double x, double y, double z,
                                       double roll, double pitch, double yaw,
                                       double width, double depth, double height,
                                       double radius, bool locked) {
    WorkcellStudioCanvasItem item;
    item.id = id;
    item.type = type;
    item.role = role;
    item.label = label;
    item.source_file = source_file;
    item.x = x;
    item.y = y;
    item.z = z;
    item.roll = roll;
    item.pitch = pitch;
    item.yaw = yaw;
    item.width = width;
    item.depth = depth;
    item.height = height;
    item.radius = radius;
    item.locked = locked;
    item.provenance = WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
    m.items.push_back(item);
  };

  push_default_item("robot_base", "robot_base", "robot", "Robot Base", "environment.yaml", 0, 0, 0, 0, 0, 0, 0.35, 0.35, 0.35, 0, true);
  push_default_item("robot_reach", "reach", "reach", "Robot Reach", "environment.yaml", 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.2, true);
  push_default_item("table", "table", "table", "Table", "environment.yaml", 0.7, 0.0, 0, 0, 0, 0, 1.0, 0.6, 0.2, 0, false);
  push_default_item("conveyor", "conveyor", "conveyor", "Conveyor", "environment.yaml", -0.8, -0.3, 0, 0, 0, 0, 1.2, 0.3, 0.2, 0, false);
  push_default_item("camera", "camera", "camera", "Camera FOV", "environment.yaml", -0.2, 1.0, 1.2, 0, 0, -1.57, 0.18, 0.18, 0.2, 0, false);
  push_default_item("pick_zone", "zone", "pick_zone", "Pick Source", "config/task_recipe.yaml", 0.55, 0.0, 0, 0, 0, 0, 0.35, 0.35, 0.1, 0, false);
  push_default_item("place_zone", "zone", "place_zone", "Place Target", "config/task_recipe.yaml", 1.0, 0.1, 0, 0, 0, 0, 0.35, 0.35, 0.1, 0, false);
  push_default_item("bin_a", "bin", "bin", "Bin A", "environment.yaml", 1.3, -0.5, 0, 0, 0, 0, 0.32, 0.25, 0.2, 0, false);
  push_default_item("object_a", "object", "object", "Object", "environment.yaml", 0.6, 0.1, 0, 0, 0, 0, 0.08, 0.08, 0.08, 0, false);
  push_default_item("home_pose", "safety", "safety/home", "config/workcell_builder_task_intent.yaml", "config/workcell_builder_task_intent.yaml", -0.4, 0.5, 0, 0, 0, 0, 0.14, 0.14, 0.14, 0, true);

  const std::string schema_version = read_string_or_warn(yaml_map_key(layout, "schema_version"), "schema_version", "");
  YAML::Node layout_items = yaml_map_key(layout, "items");
  const bool schema_current = (schema_version == "workcell_studio_layout/v1");
  const bool schema_legacy = schema_version.empty() && layout_items.IsSequence();
  if (layout_ok && (schema_current || schema_legacy)) {
    bool incomplete_placement_metadata = false;
    if (!layout_items.IsDefined() || !layout_items.IsSequence()) {
      if (layout_items.IsDefined()) add_warning("items", "expected sequence; skipping malformed items");
    } else {
    for (const auto & node : layout_items) {
      if (!node.IsMap()) { add_warning("items[]", "expected map item; skipping"); continue; }
      const auto id = read_string_or_warn(yaml_map_key(node, "id"), "items[].id", "");
      if (id.empty()) continue;
      bool matched_existing = false;
      for (auto & item : m.items) {
        if (item.id == id) {
          matched_existing = true;
          item.provenance = WorkcellStudioItemProvenance::EditableLayout;
          YAML::Node pose = yaml_map_key(node, "pose");
          if (pose.IsDefined() && pose.IsMap()) {
            YAML::Node xyz = yaml_map_key(pose, "xyz");
            if (xyz.IsDefined() && xyz.IsSequence()) {
              if (xyz.size() < 3) incomplete_placement_metadata = true;
              item.x = read_double_or_warn(yaml_seq_index(xyz, 0), "items[].pose.xyz[0]", item.x);
              item.y = read_double_or_warn(yaml_seq_index(xyz, 1), "items[].pose.xyz[1]", item.y);
              item.z = read_double_or_warn(yaml_seq_index(xyz, 2), "items[].pose.xyz[2]", item.z);
            } else if (xyz.IsDefined()) { add_warning("items[].pose.xyz", "expected sequence; using defaults"); incomplete_placement_metadata = true; }
            else { incomplete_placement_metadata = true; }
            YAML::Node rpy = yaml_map_key(pose, "rpy");
            if (rpy.IsDefined() && rpy.IsSequence()) {
              if (rpy.size() < 3) incomplete_placement_metadata = true;
              item.roll = read_double_or_warn(yaml_seq_index(rpy, 0), "items[].pose.rpy[0]", item.roll);
              item.pitch = read_double_or_warn(yaml_seq_index(rpy, 1), "items[].pose.rpy[1]", item.pitch);
              item.yaw = read_double_or_warn(yaml_seq_index(rpy, 2), "items[].pose.rpy[2]", item.yaw);
            } else if (rpy.IsDefined()) { add_warning("items[].pose.rpy", "expected sequence; using defaults"); incomplete_placement_metadata = true; }
            else { incomplete_placement_metadata = true; }
          } else if (pose.IsDefined()) {
            add_warning("items[].pose", "expected map; using defaults");
            incomplete_placement_metadata = true;
          } else {
            incomplete_placement_metadata = true;
          }
        }
      }
      if (!matched_existing) {
        WorkcellStudioCanvasItem extra;
        extra.id = id;
        extra.type = read_string_or_warn(yaml_map_key(node, "type"), "items[].type", "object");
        extra.role = read_string_or_warn(yaml_map_key(node, "role"), "items[].role", "preview");
        const auto category = read_string_or_warn(yaml_map_key(node, "category"), "items[].category", "Custom / Imported");
        if (category == "Pick/Place Zones") extra.type = "zone";
        std::string label = read_string_or_warn(yaml_map_key(node, "display_name"), "items[].display_name", "");
        if (label.empty()) label = read_string_or_warn(yaml_map_key(node, "name"), "items[].name", id);
        extra.label = label.empty() ? id : label;
        std::string source_file = read_string_or_warn(yaml_map_key(node, "source_path"), "items[].source_path", "");
        if (source_file.empty()) source_file = read_string_or_warn(yaml_map_key(node, "source_layer"), "items[].source_layer", "layout/workcell_studio_layout.yaml");
        extra.source_file = source_file.empty() ? "layout/workcell_studio_layout.yaml" : source_file;
        extra.provenance = WorkcellStudioItemProvenance::GeneratedOrLegacyPreview;
        YAML::Node pose = yaml_map_key(node, "pose");
        if (pose.IsDefined() && pose.IsMap()) {
          YAML::Node xyz = yaml_map_key(pose, "xyz");
          if (xyz.IsDefined() && xyz.IsSequence()) {
            extra.x = read_double_or_warn(yaml_seq_index(xyz, 0), "items[].pose.xyz[0]", extra.x);
            extra.y = read_double_or_warn(yaml_seq_index(xyz, 1), "items[].pose.xyz[1]", extra.y);
            extra.z = read_double_or_warn(yaml_seq_index(xyz, 2), "items[].pose.xyz[2]", extra.z);
          } else if (xyz.IsDefined()) add_warning("items[].pose.xyz", "expected sequence; using defaults");
          YAML::Node rpy = yaml_map_key(pose, "rpy");
          if (rpy.IsDefined() && rpy.IsSequence()) {
            extra.roll = read_double_or_warn(yaml_seq_index(rpy, 0), "items[].pose.rpy[0]", extra.roll);
            extra.pitch = read_double_or_warn(yaml_seq_index(rpy, 1), "items[].pose.rpy[1]", extra.pitch);
            extra.yaw = read_double_or_warn(yaml_seq_index(rpy, 2), "items[].pose.rpy[2]", extra.yaw);
          } else if (rpy.IsDefined()) add_warning("items[].pose.rpy", "expected sequence; using defaults");
        } else if (pose.IsDefined()) add_warning("items[].pose", "expected map; using defaults");
        YAML::Node size = yaml_map_key(node, "size");
        if (size.IsDefined()) {
          if (size.IsMap()) {
            extra.width = read_double_or_warn(yaml_map_key(size, "width"), "items[].size.width", extra.width);
            extra.depth = read_double_or_warn(yaml_map_key(size, "depth"), "items[].size.depth", extra.depth);
            extra.height = read_double_or_warn(yaml_map_key(size, "height"), "items[].size.height", extra.height);
          } else if (size.IsSequence()) {
            extra.width = read_double_or_warn(yaml_seq_index(size, 0), "items[].size[0]", extra.width);
            extra.depth = read_double_or_warn(yaml_seq_index(size, 1), "items[].size[1]", extra.depth);
            extra.height = read_double_or_warn(yaml_seq_index(size, 2), "items[].size[2]", extra.height);
          } else {
            add_warning("items[].size", "expected map or sequence; using defaults");
          }
        }
        bool locked = false;
        if (yaml_read_bool(yaml_map_key(node, "locked"), &locked)) extra.locked = locked;
        else if (yaml_map_key(node, "locked").IsDefined()) add_warning("items[].locked", "expected scalar bool; using default");
        m.items.push_back(extra);
      }
    }
    }
    if (incomplete_placement_metadata) {
      warned_incomplete_placement_metadata = true;
      enable_deterministic_fallback("layout/workcell_studio_layout.yaml has incomplete placement metadata");
    }
  } else if (layout_ok) {
    enable_deterministic_fallback("layout/workcell_studio_layout.yaml has invalid or missing schema_version");
  }

  if (deterministic_fallback_layout) {
    for (auto & item : m.items) {
      item.provenance = WorkcellStudioItemProvenance::StaticFallbackPreview;
      if (item.id == "robot_base" || item.role == "robot") { item.x = -0.90; item.y = 0.0; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "table" || item.role == "table") { item.x = 0.0; item.y = 0.0; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "conveyor" || item.role == "conveyor") { item.x = -1.30; item.y = -0.60; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "pick_zone" || item.role == "pick_zone") { item.x = -0.35; item.y = 0.15; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "place_zone" || item.role == "place_zone") { item.x = 0.75; item.y = 0.15; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "bin_a" || item.role == "bin") { item.x = 1.10; item.y = -0.35; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "camera" || item.role == "camera") { item.x = -0.10; item.y = 0.95; item.z = 1.45; item.roll = 0.0; item.pitch = -0.40; item.yaw = -1.20; }
      else if (item.id == "home_pose" || item.role.find("safety") != std::string::npos) { item.x = 0.0; item.y = 0.95; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
      else if (item.id == "object_a" || item.role == "object") { item.x = -0.25; item.y = 0.10; item.z = 0.0; item.roll = 0.0; item.pitch = 0.0; item.yaw = 0.0; }
    }
    m.warnings.push_back("Using deterministic 3D fallback layout because layout metadata is incomplete.");
    if (!deterministic_fallback_reason.empty() && !warned_incomplete_placement_metadata) m.warnings.push_back("Fallback detail: " + deterministic_fallback_reason + ".");
  }

  std::vector<fs::path> probed_visuals;
  std::vector<fs::path> probed_collisions;
  probe_mesh_candidates(scene_dir, &probed_visuals, &probed_collisions);
  const auto choose_probed = [](const WorkcellStudioCanvasItem & item, const std::vector<fs::path> & pool)->fs::path {
    for (const auto & p : pool) {
      const std::string b = p.stem().string();
      if (b == item.id || b == item.type || b == item.role) return p;
    }
    return fs::path();
  };

  for (auto & item : m.items) {
    item.mesh_available = false;
    item.mesh_path.clear();
    item.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
    const auto candidates = gather_mesh_candidates(env, manifest, layout_items, item.id);
    fs::path visual;
    fs::path collision;
    for (const auto & c : candidates) {
      const fs::path resolved = resolve_mesh_candidate(c, scene_dir);
      if (!fs::exists(resolved)) continue;
      const auto text = resolved.generic_string();
      if (text.find("/visual/") != std::string::npos) visual = resolved;
      else if (text.find("/collision/") != std::string::npos) collision = resolved;
      else if (visual.empty()) visual = resolved;
    }
    if (visual.empty()) visual = choose_probed(item, probed_visuals);
    if (collision.empty()) collision = choose_probed(item, probed_collisions);

    if (!visual.empty()) {
      item.mesh_path = visual.generic_string();
      item.mesh_available = true;
      item.mesh_load_warning.clear();
    } else if (!collision.empty()) {
      item.mesh_path = collision.generic_string();
      item.mesh_available = true;
      item.mesh_load_warning = "Mesh preview fallback for " + item.id + ": visual mesh unavailable; using collision mesh";
      m.warnings.push_back(item.mesh_load_warning);
    } else {
      item.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
      m.warnings.push_back(item.mesh_load_warning);
    }

    if (layout_items.IsSequence()) {
      for (const auto & node : layout_items) {
        if (!node.IsMap() || yaml_map_value_or_empty(node, "id") != item.id) continue;
        const YAML::Node mesh = yaml_map_key(node, "mesh");
        if (!mesh || mesh.IsNull() || mesh_node_disabled(mesh)) {
          item.mesh_available = false;
          item.mesh_path.clear();
          item.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
        } else if (mesh.IsMap()) {
          item.has_mesh_metadata = true;
          const std::string path = get_optional_string(mesh, "path", "");
          if (path.empty()) {
            item.mesh_available = false;
            item.mesh_path.clear();
            item.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
          } else {
            const fs::path resolved_path = resolve_mesh_candidate(path, scene_dir);
            item.mesh_path = resolved_path.generic_string();
          }
          const YAML::Node scale = yaml_map_key(mesh, "scale");
          if (scale.IsSequence()) {
            item.mesh_scale_x = read_double_or_warn(yaml_seq_index(scale, 0), "items[].mesh.scale[0]", item.mesh_scale_x);
            item.mesh_scale_y = read_double_or_warn(yaml_seq_index(scale, 1), "items[].mesh.scale[1]", item.mesh_scale_y);
            item.mesh_scale_z = read_double_or_warn(yaml_seq_index(scale, 2), "items[].mesh.scale[2]", item.mesh_scale_z);
          }
          const YAML::Node rpy = yaml_map_key(mesh, "rpy");
          if (rpy.IsSequence()) {
            item.mesh_r = read_double_or_warn(yaml_seq_index(rpy, 0), "items[].mesh.rpy[0]", item.mesh_r);
            item.mesh_p = read_double_or_warn(yaml_seq_index(rpy, 1), "items[].mesh.rpy[1]", item.mesh_p);
            item.mesh_y = read_double_or_warn(yaml_seq_index(rpy, 2), "items[].mesh.rpy[2]", item.mesh_y);
          }
          const YAML::Node origin = yaml_map_key(mesh, "origin_offset");
          if (origin.IsSequence()) {
            item.has_origin_offset = true;
            item.origin_offset_x = read_double_or_warn(yaml_seq_index(origin, 0), "items[].mesh.origin_offset[0]", item.origin_offset_x);
            item.origin_offset_y = read_double_or_warn(yaml_seq_index(origin, 1), "items[].mesh.origin_offset[1]", item.origin_offset_y);
            item.origin_offset_z = read_double_or_warn(yaml_seq_index(origin, 2), "items[].mesh.origin_offset[2]", item.origin_offset_z);
          }
        }
      }
    }
  }

  std::stable_sort(m.items.begin(), m.items.end(), [](const WorkcellStudioCanvasItem & a, const WorkcellStudioCanvasItem & b) {
    return a.id < b.id;
  });
  for (const auto & item : m.items) {
    if (item.provenance == WorkcellStudioItemProvenance::EditableLayout) ++m.provenance_status.editable_layout_count;
    else if (item.provenance == WorkcellStudioItemProvenance::GeneratedOrLegacyPreview) ++m.provenance_status.generated_or_legacy_preview_count;
    else ++m.provenance_status.static_fallback_preview_count;
  }
  m.provenance_status.summary = provenance_summary_text(m.provenance_status);
  if (!m.warnings.empty()) { m.has_warnings = true; m.status = "WARNINGS"; { WorkcellStudioCanvasItem w; w.id="warning"; w.type="warning"; w.role="warning"; w.label="warning"; w.source_file="environment.yaml"; w.x=-1.2; w.y=1.2; w.width=0.1; w.depth=0.1; w.height=0.0; w.warnings=m.warnings; m.items.push_back(w); } }
  else { m.status = "READY"; }
  return m;
  } catch (const YAML::Exception & e) {
    workcell_builder::log_warning_once_per_context_path_reason("task_metadata_summary_loader", scene_dir / "layout" / "workcell_studio_layout.yaml", std::string("YAML parse exception in preview loader: ") + e.what());
  } catch (const std::exception & e) {
    workcell_builder::log_warning_once_per_context_path_reason("task_metadata_summary_loader", scene_dir / "layout" / "workcell_studio_layout.yaml", std::string("std exception in preview loader: ") + e.what());
  }
  WorkcellStudioCanvasModel fallback;
  fallback.scene_name = scene_name;
  fallback.status = "WARNINGS";
  fallback.has_warnings = true;
  fallback.warnings.push_back("mesh metadata missing or legacy; using primitive preview");
  fallback.provenance_status.summary = provenance_summary_text(fallback.provenance_status);
  return fallback;
}

static std::string lower_scalar_or_empty(const YAML::Node & node)
{
  std::string value;
  if (!yaml_read_string(node, &value)) return "";
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return value;
}

static bool layout_item_is_fallback_preview_entry(const YAML::Node & node)
{
  const std::array<std::string, 5> marker_keys = {"source_layer", "source_file", "source_path", "provenance", "preview_source"};
  for (const auto & key : marker_keys) {
    const std::string value = lower_scalar_or_empty(yaml_map_key(node, key));
    if (value.find("fallback") != std::string::npos || value.find("static_preview") != std::string::npos) return true;
  }
  return false;
}

static bool layout_item_is_effectively_editable(const YAML::Node & node)
{
  if (!node || !node.IsMap()) return false;

  bool locked = false;
  const YAML::Node locked_node = yaml_map_key(node, "locked");
  if (yaml_read_bool(locked_node, &locked) && locked) return false;
  if (locked_node.IsDefined() && !yaml_read_bool(locked_node, &locked)) return false;

  bool editable = false;
  const YAML::Node editable_node = yaml_map_key(node, "editable");
  if (yaml_read_bool(editable_node, &editable)) return editable;
  if (editable_node.IsDefined()) return false;

  if (layout_item_is_fallback_preview_entry(node)) return false;
  return true;
}


static YAML::Node ensure_map_child(YAML::Node parent, const char * key)
{
  if (!parent[key] || !parent[key].IsMap()) parent[key] = YAML::Node(YAML::NodeType::Map);
  return parent[key];
}

static YAML::Node ensure_sequence_child(YAML::Node parent, const char * key)
{
  if (!parent[key] || !parent[key].IsSequence()) parent[key] = YAML::Node(YAML::NodeType::Sequence);
  return parent[key];
}

static bool yaml_nodes_equal(const YAML::Node & lhs, const YAML::Node & rhs)
{
  return YAML::Dump(lhs) == YAML::Dump(rhs);
}

static bool layout_item_has_bootstrap_placeable_fields(const YAML::Node & item)
{
  if (!item || !item.IsMap()) return false;
  if (yaml_map_value_or_empty(item, "id").empty()) return false;
  const YAML::Node pose = yaml_map_key(item, "pose");
  const YAML::Node xyz = yaml_map_key(pose, "xyz");
  const YAML::Node rpy = yaml_map_key(pose, "rpy");
  const YAML::Node dimensions = yaml_map_key(item, "dimensions");
  if (!pose || !pose.IsMap() || !xyz || !xyz.IsSequence() || xyz.size() < 3) return false;
  if (!rpy || !rpy.IsSequence() || rpy.size() < 3) return false;
  if (!dimensions || !dimensions.IsSequence() || dimensions.size() < 3) return false;
  if (yaml_map_value_or_empty(item, "source").empty()) return false;
  bool editable = false;
  bool locked = true;
  if (!yaml_read_bool(yaml_map_key(item, "editable"), &editable) || !editable) return false;
  if (!yaml_read_bool(yaml_map_key(item, "locked"), &locked) || locked) return false;
  return layout_item_is_effectively_editable(item);
}

static YAML::Node bootstrap_environment_asset_from_layout_item(const YAML::Node & existing, const YAML::Node & layout_item)
{
  YAML::Node out = (existing && existing.IsMap()) ? YAML::Clone(existing) : YAML::Node(YAML::NodeType::Map);
  out["id"] = yaml_map_key(layout_item, "id");
  const YAML::Node type = yaml_map_key(layout_item, "type");
  if (type.IsDefined()) out["type"] = type;
  const YAML::Node category = yaml_map_key(layout_item, "category");
  if (category.IsDefined()) out["category"] = category;
  YAML::Node pose = ensure_map_child(out, "pose");
  pose["xyz"] = YAML::Clone(yaml_map_key(yaml_map_key(layout_item, "pose"), "xyz"));
  pose["rpy"] = YAML::Clone(yaml_map_key(yaml_map_key(layout_item, "pose"), "rpy"));
  out["dimensions"] = YAML::Clone(yaml_map_key(layout_item, "dimensions"));
  out["source"] = yaml_map_key(layout_item, "source");
  out["editable"] = yaml_map_key(layout_item, "editable");
  out["locked"] = yaml_map_key(layout_item, "locked");
  const YAML::Node mesh = yaml_map_key(layout_item, "mesh");
  if (mesh.IsDefined()) out["mesh"] = YAML::Clone(mesh);
  return out;
}

WorkcellStudioEditableLayoutInspection inspect_editable_layout_entries(const fs::path & scene_dir)
{
  WorkcellStudioEditableLayoutInspection out;
  const fs::path layout_path = scene_dir / "layout" / "workcell_studio_layout.yaml";
  out.exists = fs::exists(layout_path);
  YAML::Node layout;
  if (!read_yaml(layout_path, &layout).loaded) return out;
  const std::string schema_version = yaml_map_value_or_empty(layout, "schema_version");
  const YAML::Node items = yaml_map_key(layout, "items");
  const bool schema_current = (schema_version == "workcell_studio_layout/v1");
  const bool schema_legacy = schema_version.empty() && items.IsSequence();
  if (!schema_current && !schema_legacy) return out;
  out.valid = true;
  if (!items || !items.IsSequence()) return out;
  out.has_items_sequence = true;
  for (const auto & node : items) {
    if (!node || !node.IsMap()) continue;
    ++out.total_item_entries;
    if (layout_item_is_effectively_editable(node)) ++out.editable_item_count;
  }
  return out;
}

std::size_t count_editable_layout_entries(const fs::path & scene_dir)
{
  return inspect_editable_layout_entries(scene_dir).editable_item_count;
}

bool is_save_layout_workflow_ready(const fs::path & scene_dir)
{
  const WorkcellStudioEditableLayoutInspection layout = inspect_editable_layout_entries(scene_dir);
  return layout.exists && layout.valid && layout.editable_item_count > 0 &&
    fs::exists(scene_dir / "environment_layout.yaml") &&
    fs::exists(scene_dir / "environment.yaml");
}


WorkcellStudioEnvironmentLayoutBootstrapResult bootstrap_environment_layout_from_editable_layout(
  const fs::path & scene_dir, const std::string & scene_name, const YAML::Node & editable_layout)
{
  WorkcellStudioEnvironmentLayoutBootstrapResult result;
  const YAML::Node layout_items = yaml_map_key(editable_layout, "items");
  if (!layout_items || !layout_items.IsSequence()) {
    result.error = "editable layout has no items sequence";
    return result;
  }

  YAML::Node env_root(YAML::NodeType::Map);
  const fs::path env_layout_path = scene_dir / "environment_layout.yaml";
  result.created = !fs::exists(env_layout_path);
  if (!result.created) {
    try {
      env_root = YAML::LoadFile(env_layout_path.string());
    } catch (const std::exception & e) {
      result.error = std::string("failed to parse environment_layout.yaml: ") + e.what();
      return result;
    }
    if (!env_root || !env_root.IsMap()) env_root = YAML::Node(YAML::NodeType::Map);
  }

  YAML::Node before = YAML::Clone(env_root);
  env_root["schema_version"] = "environment_layout/v1";
  if (!env_root["scene_name"] || !env_root["scene_name"].IsScalar()) env_root["scene_name"] = scene_name;

  YAML::Node placed_assets = ensure_sequence_child(env_root, "placed_assets");

  std::set<std::string> layout_ids;
  std::vector<std::string> layout_id_order;
  YAML::Node bootstrap_by_id(YAML::NodeType::Map);
  for (const auto & layout_item : layout_items) {
    if (!layout_item_has_bootstrap_placeable_fields(layout_item)) continue;
    const std::string id = yaml_map_value_or_empty(layout_item, "id");
    if (layout_ids.insert(id).second) layout_id_order.push_back(id);
    bootstrap_by_id[id] = layout_item;
  }
  if (layout_ids.empty()) {
    result.error = "editable layout has no bootstrappable editable/placeable items";
    return result;
  }

  YAML::Node updated_placed_assets(YAML::NodeType::Sequence);
  std::set<std::string> emitted_ids;
  for (std::size_t i = 0; i < placed_assets.size(); ++i) {
    const YAML::Node existing = placed_assets[i];
    if (!existing || !existing.IsMap()) {
      updated_placed_assets.push_back(existing);
      continue;
    }
    const std::string id = yaml_map_value_or_empty(existing, "id");
    if (!id.empty() && layout_ids.count(id) != 0) {
      updated_placed_assets.push_back(bootstrap_environment_asset_from_layout_item(existing, bootstrap_by_id[id]));
      emitted_ids.insert(id);
    } else {
      updated_placed_assets.push_back(existing);
    }
  }
  for (const auto & id : layout_id_order) {
    if (emitted_ids.count(id) != 0) continue;
    updated_placed_assets.push_back(bootstrap_environment_asset_from_layout_item(YAML::Node(), bootstrap_by_id[id]));
  }
  env_root["placed_assets"] = updated_placed_assets;
  result.placed_assets_written = layout_ids.size();

  if (!result.created && yaml_nodes_equal(before, env_root)) {
    result.ok = true;
    result.wrote = false;
    return result;
  }

  boost::system::error_code ec;
  fs::create_directories(scene_dir, ec);
  if (ec) {
    result.error = std::string("failed to create scene directory: ") + ec.message();
    return result;
  }
  std::ofstream out(env_layout_path.string());
  if (!out.good()) {
    result.error = "failed to open environment_layout.yaml for write";
    return result;
  }
  out << env_root << "\n";
  out.close();
  if (!out.good()) {
    result.error = "failed to write environment_layout.yaml";
    return result;
  }
  result.ok = true;
  result.wrote = true;
  return result;
}

static bool has_safe_starter_layout_metadata(const WorkcellStudioCanvasItem & preview_item)
{
  if (!preview_item.has_mesh_metadata) return false;
  if (preview_item.mesh_path.empty()) return false;
  if (preview_item.source_file.empty()) return false;
  if (!preview_item.mesh_load_warning.empty()) return false;
  if (!preview_item.alignment_warning.empty()) return false;
  if (!preview_item.warnings.empty()) return false;
  return true;
}

WorkcellStudioStarterLayoutSummary build_starter_layout_entries_from_preview(const WorkcellStudioCanvasModel & model)
{
  WorkcellStudioStarterLayoutSummary summary;
  summary.total_preview_items = model.items.size();

  YAML::Node root(YAML::NodeType::Map);
  root["schema_version"] = "workcell_studio_layout/v1";
  root["scene_name"] = model.scene_name;
  YAML::Node items(YAML::NodeType::Sequence);
  for (const auto & preview_item : model.items) {
    if (preview_item.locked) {
      ++summary.skipped_locked_items;
      continue;
    }
    if (preview_item.provenance == WorkcellStudioItemProvenance::StaticFallbackPreview) {
      ++summary.skipped_static_fallback_items;
      continue;
    }
    if (!has_safe_starter_layout_metadata(preview_item)) {
      ++summary.skipped_unsafe_or_missing_metadata_items;
      continue;
    }
    YAML::Node item(YAML::NodeType::Map);
    item["id"] = preview_item.id;
    item["type"] = preview_item.type;
    item["category"] = preview_item.type;
    YAML::Node pose(YAML::NodeType::Map);
    pose["xyz"].push_back(preview_item.x);
    pose["xyz"].push_back(preview_item.y);
    pose["xyz"].push_back(preview_item.z);
    pose["rpy"].push_back(preview_item.roll);
    pose["rpy"].push_back(preview_item.pitch);
    pose["rpy"].push_back(preview_item.yaw);
    item["pose"] = pose;
    item["dimensions"].push_back(preview_item.width);
    item["dimensions"].push_back(preview_item.depth);
    item["dimensions"].push_back(preview_item.height);
    YAML::Node mesh(YAML::NodeType::Map);
    mesh["path"] = preview_item.mesh_path;
    mesh["source"] = preview_item.source_file;
    item["mesh"] = mesh;
    item["source"] = preview_item.source_file;
    item["editable"] = true;
    item["locked"] = false;
    items.push_back(item);
    ++summary.editable_items_created;
  }
  root["empty_layout_marker"] = summary.editable_items_created == 0;
  root["items"] = items;
  summary.layout = root;
  return summary;
}

}  // namespace workcell_builder
