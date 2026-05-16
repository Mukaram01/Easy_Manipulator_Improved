#include "workcell_studio_canvas_model.hpp"
#include "workcell_yaml_utils.hpp"
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;
namespace workcell_builder {

static bool read_yaml(const fs::path & p, YAML::Node * out){ try{ if(!fs::exists(p)) return false; *out = YAML::LoadFile(p.string()); return true;}catch(...){return false;} }

WorkcellStudioCanvasModel build_workcell_studio_canvas_model(const fs::path & scene_dir, const std::string & scene_name)
{
  WorkcellStudioCanvasModel m; m.scene_name = scene_name; m.status = "WARNINGS";
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
  const bool env_ok = read_yaml(scene_dir / "environment.yaml", &env);
  const bool manifest_ok = read_yaml(scene_dir / "scene_manifest.yaml", &manifest);
  const bool task_ok = read_yaml(scene_dir / "config" / "task_recipe.yaml", &task);
  const bool layout_ok = read_yaml(scene_dir / "layout" / "workcell_studio_layout.yaml", &layout);
  if (!env_ok) m.warnings.push_back("Malformed or missing environment.yaml");
  if (!manifest_ok) m.warnings.push_back("Missing scene_manifest.yaml");
  if (!task_ok) m.warnings.push_back("Task intent missing");
  if (!layout_ok && fs::exists(scene_dir / "layout" / "workcell_studio_layout.yaml")) m.warnings.push_back("Malformed layout/workcell_studio_layout.yaml; falling back safely");

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

  m.items.push_back({"robot_base","robot_base","robot","Robot Base","environment.yaml",0,0,0,0,0,0,0.35,0.35,0.35,0,true,{}});
  m.items.push_back({"robot_reach","reach","reach","Robot Reach","environment.yaml",0,0,0,0,0,0,0,0,0,1.2,true,{}});
  m.items.push_back({"table","table","table","Table","environment.yaml",0.7,0.0,0,0,0,0,1.0,0.6,0.2,0,false,{}});
  m.items.push_back({"conveyor","conveyor","conveyor","Conveyor","environment.yaml",-0.8,-0.3,0,0,0,0,1.2,0.3,0.2,0,false,{}});
  m.items.push_back({"camera","camera","camera","Camera FOV","environment.yaml",-0.2,1.0,1.2,0,0,-1.57,0.18,0.18,0.2,0,false,{}});
  m.items.push_back({"pick_zone","zone","pick_zone","Pick Source","config/task_recipe.yaml",0.55,0.0,0,0,0,0,0.35,0.35,0.1,0,false,{}});
  m.items.push_back({"place_zone","zone","place_zone","Place Target","config/task_recipe.yaml",1.0,0.1,0,0,0,0,0.35,0.35,0.1,0,false,{}});
  m.items.push_back({"bin_a","bin","bin","Bin A","environment.yaml",1.3,-0.5,0,0,0,0,0.32,0.25,0.2,0,false,{}});
  m.items.push_back({"object_a","object","object","Object","environment.yaml",0.6,0.1,0,0,0,0,0.08,0.08,0.08,0,false,{}});
  m.items.push_back({"home_pose","safety","safety/home","config/workcell_builder_task_intent.yaml","config/workcell_builder_task_intent.yaml",-0.4,0.5,0,0,0,0,0.14,0.14,0.14,0,true,{}});

  const std::string schema_version = read_string_or_warn(yaml_map_key(layout, "schema_version"), "schema_version", "");
  YAML::Node layout_items = yaml_map_key(layout, "items");
  if (layout_ok && schema_version == "workcell_studio_layout/v1") {
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
          YAML::Node pose = yaml_map_key(node, "pose");
          if (pose.IsDefined() && pose.IsMap()) {
            YAML::Node xyz = yaml_map_key(pose, "xyz");
            if (xyz.IsDefined() && xyz.IsSequence()) {
              item.x = read_double_or_warn(yaml_seq_index(xyz, 0), "items[].pose.xyz[0]", item.x);
              item.y = read_double_or_warn(yaml_seq_index(xyz, 1), "items[].pose.xyz[1]", item.y);
              item.z = read_double_or_warn(yaml_seq_index(xyz, 2), "items[].pose.xyz[2]", item.z);
            } else if (xyz.IsDefined()) add_warning("items[].pose.xyz", "expected sequence; using defaults");
            YAML::Node rpy = yaml_map_key(pose, "rpy");
            if (rpy.IsDefined() && rpy.IsSequence()) {
              item.roll = read_double_or_warn(yaml_seq_index(rpy, 0), "items[].pose.rpy[0]", item.roll);
              item.pitch = read_double_or_warn(yaml_seq_index(rpy, 1), "items[].pose.rpy[1]", item.pitch);
              item.yaw = read_double_or_warn(yaml_seq_index(rpy, 2), "items[].pose.rpy[2]", item.yaw);
            } else if (rpy.IsDefined()) add_warning("items[].pose.rpy", "expected sequence; using defaults");
          } else if (pose.IsDefined()) {
            add_warning("items[].pose", "expected map; using defaults");
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
        extra.label = read_string_or_warn(yaml_map_key(node, "display_name"), "items[].display_name", id);
        extra.source_file = read_string_or_warn(yaml_map_key(node, "source_path"), "items[].source_path", "layout/workcell_studio_layout.yaml");
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
  }
  if (!m.warnings.empty()) { m.has_warnings = true; m.status = "WARNINGS"; { WorkcellStudioCanvasItem w; w.id="warning"; w.type="warning"; w.role="warning"; w.label="warning"; w.source_file="environment.yaml"; w.x=-1.2; w.y=1.2; w.width=0.1; w.depth=0.1; w.height=0.0; w.warnings=m.warnings; m.items.push_back(w); } }
  else { m.status = "READY"; }
  return m;
}
}
