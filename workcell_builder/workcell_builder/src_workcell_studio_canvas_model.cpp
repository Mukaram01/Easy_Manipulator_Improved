#include "workcell_studio_canvas_model.hpp"
#include "workcell_yaml_utils.hpp"
#include "workcell_warning_once.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <set>
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
  try {
    if (!node.IsDefined() || !node.IsScalar()) return;
    const std::string text = node.as<std::string>("");
    if (text.empty()) return;
    if (text.rfind("package://", 0) == 0 || text.find(".stl") != std::string::npos || text.find("meshes/") != std::string::npos) out->push_back(text);
  } catch (...) {
    return;
  }
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

static bool yaml_node_is_map(const YAML::Node & node)
{
  try { return node.IsDefined() && node.IsMap(); } catch (...) { return false; }
}

static bool yaml_node_is_sequence(const YAML::Node & node)
{
  try { return node.IsDefined() && node.IsSequence(); } catch (...) { return false; }
}

static bool yaml_node_is_defined(const YAML::Node & node)
{
  try { return node.IsDefined(); } catch (...) { return false; }
}

static std::vector<std::string> gather_mesh_candidates(const YAML::Node & env, const YAML::Node & manifest, const YAML::Node & layout_items, const std::string & item_id)
{
  std::vector<std::string> out;
  const auto scan_fields = [&out](const YAML::Node & n) {
    const YAML::Node mesh = yaml_map_key(n, "mesh");
    if (yaml_node_is_map(mesh)) add_mesh_candidate(optional_scalar(mesh, "path"), &out);
    else add_mesh_candidate(mesh, &out);
    add_mesh_candidate(optional_scalar(n, "mesh_path"), &out);
    add_mesh_candidate(yaml_map_key(n, "visual_mesh"), &out);
    add_mesh_candidate(yaml_map_key(n, "collision_mesh"), &out);
    const YAML::Node visual = yaml_map_key(n, "visual");
    const YAML::Node v = yaml_node_is_map(visual) ? yaml_map_key(visual, "geometry") : YAML::Node();
    add_mesh_candidate(yaml_map_key(v, "filepath"), &out);
    add_mesh_candidate(yaml_map_key(v, "mesh"), &out);
  };
  scan_fields(env);
  scan_fields(manifest);
  if (yaml_node_is_sequence(layout_items)) {
    for (const auto & node : layout_items) {
      if (!yaml_node_is_map(node)) continue;
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
        extra.provenance = WorkcellStudioItemProvenance::EditableLayout;
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
        if (yaml_node_is_defined(size)) {
          if (yaml_node_is_map(size)) {
            extra.width = read_double_or_warn(yaml_map_key(size, "width"), "items[].size.width", extra.width);
            extra.depth = read_double_or_warn(yaml_map_key(size, "depth"), "items[].size.depth", extra.depth);
            extra.height = read_double_or_warn(yaml_map_key(size, "height"), "items[].size.height", extra.height);
          } else if (yaml_node_is_sequence(size)) {
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
        if (!yaml_node_is_defined(mesh) || mesh_node_disabled(mesh)) {
          item.mesh_available = false;
          item.mesh_path.clear();
          item.mesh_load_warning = "mesh metadata missing or legacy; using primitive preview";
        } else if (yaml_node_is_map(mesh)) {
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
          if (yaml_node_is_sequence(scale)) {
            item.mesh_scale_x = read_double_or_warn(yaml_seq_index(scale, 0), "items[].mesh.scale[0]", item.mesh_scale_x);
            item.mesh_scale_y = read_double_or_warn(yaml_seq_index(scale, 1), "items[].mesh.scale[1]", item.mesh_scale_y);
            item.mesh_scale_z = read_double_or_warn(yaml_seq_index(scale, 2), "items[].mesh.scale[2]", item.mesh_scale_z);
          }
          const YAML::Node rpy = yaml_map_key(mesh, "rpy");
          if (yaml_node_is_sequence(rpy)) {
            item.mesh_r = read_double_or_warn(yaml_seq_index(rpy, 0), "items[].mesh.rpy[0]", item.mesh_r);
            item.mesh_p = read_double_or_warn(yaml_seq_index(rpy, 1), "items[].mesh.rpy[1]", item.mesh_p);
            item.mesh_y = read_double_or_warn(yaml_seq_index(rpy, 2), "items[].mesh.rpy[2]", item.mesh_y);
          }
          const YAML::Node origin = yaml_map_key(mesh, "origin_offset");
          if (yaml_node_is_sequence(origin)) {
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
    const std::string value = lower_scalar_or_empty(yaml_map_key(node, key.c_str()));
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


static double yaml_double_or_default(const YAML::Node & node, double fallback)
{
  double out = fallback;
  return yaml_read_double(node, &out) ? out : fallback;
}

static std::string canonical_item_id_from_node(const YAML::Node & node)
{
  std::string id = yaml_map_value_or_empty(node, "id");
  if (id.empty()) id = yaml_map_value_or_empty(node, "name");
  if (id.empty()) id = yaml_map_value_or_empty(node, "asset_id");
  return id;
}

static std::string canonical_item_type_from_node(const YAML::Node & node)
{
  std::string type = yaml_map_value_or_empty(node, "type");
  if (type.empty()) type = yaml_map_value_or_empty(node, "role");
  if (type.empty()) type = yaml_map_value_or_empty(node, "source");
  return type.empty() ? "object" : type;
}

static void copy_pose_from_canonical_node(const YAML::Node & source, YAML::Node * target)
{
  YAML::Node pose = yaml_map_key(source, "pose");
  YAML::Node xyz;
  YAML::Node rpy;
  if (yaml_node_is_map(pose)) {
    xyz = yaml_map_key(pose, "xyz");
    rpy = yaml_map_key(pose, "rpy");
  } else if (yaml_node_is_sequence(pose)) {
    xyz = pose;
  }

  YAML::Node out_pose(YAML::NodeType::Map);
  out_pose["xyz"].push_back(yaml_double_or_default(yaml_seq_index(xyz, 0), 0.0));
  out_pose["xyz"].push_back(yaml_double_or_default(yaml_seq_index(xyz, 1), 0.0));
  out_pose["xyz"].push_back(yaml_double_or_default(yaml_seq_index(xyz, 2), 0.0));
  out_pose["rpy"].push_back(yaml_double_or_default(yaml_seq_index(rpy, 0), 0.0));
  out_pose["rpy"].push_back(yaml_double_or_default(yaml_seq_index(rpy, 1), 0.0));
  out_pose["rpy"].push_back(yaml_double_or_default(yaml_seq_index(rpy, 2), 0.0));
  (*target)["pose"] = out_pose;
}

static void copy_dimensions_from_canonical_node(const YAML::Node & source, YAML::Node * target)
{
  try {
    YAML::Node size = yaml_map_key(source, "size");
    if (!yaml_node_is_defined(size)) size = yaml_map_key(source, "dimensions");
    if (yaml_node_is_map(size)) {
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_map_key(size, "width"), 0.25));
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_map_key(size, "depth"), 0.25));
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_map_key(size, "height"), 0.25));
    } else if (yaml_node_is_sequence(size)) {
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_seq_index(size, 0), 0.25));
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_seq_index(size, 1), 0.25));
      (*target)["dimensions"].push_back(yaml_double_or_default(yaml_seq_index(size, 2), 0.25));
    }
  } catch (const YAML::Exception &) {
  } catch (const std::exception &) {
  }
}

static bool append_canonical_layout_item(const YAML::Node & source, const std::string & source_name, YAML::Node * items)
{
  if (!yaml_node_is_map(source)) return false;
  const std::string id = canonical_item_id_from_node(source);
  if (id.empty()) return false;

  YAML::Node item(YAML::NodeType::Map);
  item["id"] = id;
  const std::string type = canonical_item_type_from_node(source);
  item["type"] = type;
  item["category"] = type;
  copy_pose_from_canonical_node(source, &item);
  copy_dimensions_from_canonical_node(source, &item);

  const YAML::Node mesh = yaml_map_key(source, "mesh");
  const std::string mesh_path = yaml_node_is_map(mesh) ? yaml_map_value_or_empty(mesh, "path") : yaml_scalar_or_empty(mesh);
  if (!mesh_path.empty()) {
    YAML::Node mesh_out(YAML::NodeType::Map);
    mesh_out["path"] = mesh_path;
    mesh_out["source"] = source_name;
    item["mesh"] = mesh_out;
  }
  const std::string collision_mesh = yaml_map_value_or_empty(source, "collision_mesh");
  if (!collision_mesh.empty() && !item["mesh"].IsDefined()) {
    YAML::Node mesh_out(YAML::NodeType::Map);
    mesh_out["path"] = collision_mesh;
    mesh_out["source"] = source_name;
    item["mesh"] = mesh_out;
  }
  item["source"] = source_name;
  item["editable"] = true;
  item["locked"] = false;
  items->push_back(item);
  return true;
}

WorkcellStudioStarterLayoutSummary bootstrap_editable_layout_from_trusted_canonical_yaml(const fs::path & scene_dir, const std::string & scene_name)
{
  WorkcellStudioStarterLayoutSummary summary;
  YAML::Node root(YAML::NodeType::Map);
  root["schema_version"] = "workcell_studio_layout/v1";
  root["scene_name"] = scene_name;
  YAML::Node items(YAML::NodeType::Sequence);

  YAML::Node env;
  const YamlLoadStatus env_status = read_yaml(scene_dir / "environment.yaml", &env);
  if (env_status.loaded) {
    const YAML::Node placed_objects = yaml_map_key(env, "placed_objects");
    if (yaml_node_is_sequence(placed_objects)) {
      for (const auto & node : placed_objects) {
        ++summary.total_preview_items;
        if (append_canonical_layout_item(node, "environment.yaml", &items)) ++summary.editable_items_created;
        else ++summary.skipped_unsafe_or_missing_metadata_items;
      }
    }
    const YAML::Node workspace = yaml_map_key(env, "workspace");
    const YAML::Node workspace_zones = yaml_node_is_map(workspace) ? yaml_map_key(workspace, "zones") : YAML::Node();
    if (yaml_node_is_sequence(workspace_zones)) {
      for (const auto & node : workspace_zones) {
        ++summary.total_preview_items;
        if (append_canonical_layout_item(node, "environment.yaml", &items)) ++summary.editable_items_created;
        else ++summary.skipped_unsafe_or_missing_metadata_items;
      }
    }
    const YAML::Node camera = yaml_map_key(env, "camera");
    bool camera_enabled = false;
    if (yaml_node_is_map(camera) && yaml_read_bool(yaml_map_key(camera, "enabled"), &camera_enabled) && camera_enabled) {
      ++summary.total_preview_items;
      YAML::Node camera_item = YAML::Clone(camera);
      if (!camera_item["id"]) camera_item["id"] = "camera";
      if (!camera_item["type"]) camera_item["type"] = "camera";
      if (append_canonical_layout_item(camera_item, "environment.yaml", &items)) ++summary.editable_items_created;
      else ++summary.skipped_unsafe_or_missing_metadata_items;
    }
  } else {
    ++summary.skipped_unsafe_or_missing_metadata_items;
  }

  if (summary.editable_items_created == 0) {
    const auto model = build_workcell_studio_canvas_model(scene_dir, scene_name);
    const auto preview_summary = build_starter_layout_entries_from_preview(model);
    summary.total_preview_items += preview_summary.total_preview_items;
    summary.skipped_locked_items += preview_summary.skipped_locked_items;
    summary.skipped_static_fallback_items += preview_summary.skipped_static_fallback_items;
    summary.skipped_unsafe_or_missing_metadata_items += preview_summary.skipped_unsafe_or_missing_metadata_items;
    if (preview_summary.editable_items_created > 0) {
      root = preview_summary.layout;
      summary.editable_items_created = preview_summary.editable_items_created;
      summary.layout = root;
      return summary;
    }
  }

  root["empty_layout_marker"] = summary.editable_items_created == 0;
  root["items"] = items;
  summary.layout = root;
  return summary;
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
