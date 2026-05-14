#include "workcell_studio_canvas_model.hpp"
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;
namespace workcell_builder {

static bool read_yaml(const fs::path & p, YAML::Node * out){ try{ if(!fs::exists(p)) return false; *out = YAML::LoadFile(p.string()); return true;}catch(...){return false;} }

WorkcellStudioCanvasModel build_workcell_studio_canvas_model(const fs::path & scene_dir, const std::string & scene_name)
{
  WorkcellStudioCanvasModel m; m.scene_name = scene_name; m.status = "WARNINGS";
  YAML::Node env, manifest, task, layout;
  const bool env_ok = read_yaml(scene_dir / "environment.yaml", &env);
  const bool manifest_ok = read_yaml(scene_dir / "scene_manifest.yaml", &manifest);
  const bool task_ok = read_yaml(scene_dir / "config" / "task_recipe.yaml", &task);
  const bool layout_ok = read_yaml(scene_dir / "layout" / "workcell_studio_layout.yaml", &layout);
  if (!env_ok) m.warnings.push_back("Malformed or missing environment.yaml");
  if (!manifest_ok) m.warnings.push_back("Missing scene_manifest.yaml");
  if (!task_ok) m.warnings.push_back("Task intent missing");
  if (!layout_ok && fs::exists(scene_dir / "layout" / "workcell_studio_layout.yaml")) m.warnings.push_back("Malformed layout/workcell_studio_layout.yaml; falling back safely");

  m.template_name = manifest_ok && manifest["template_name"] ? manifest["template_name"].as<std::string>() : "unknown_template";
  m.robot_summary = env_ok && env["robot"] && env["robot"]["name"] ? env["robot"]["name"].as<std::string>() : "Robot";
  m.tool_summary = env_ok && env["end_effector"] && env["end_effector"]["name"] ? env["end_effector"]["name"].as<std::string>() : "Tool";
  m.pick_source = task_ok && task["pick_source"] ? task["pick_source"].as<std::string>() : "Task intent missing";
  m.grasp_strategy = task_ok && task["grasp_strategy"] ? task["grasp_strategy"].as<std::string>() : "Generate task recipe to populate this panel";
  m.place_target = task_ok && task["place_target"] ? task["place_target"].as<std::string>() : "Task intent missing";
  m.release_strategy = task_ok && task["release_strategy"] ? task["release_strategy"].as<std::string>() : "Generate task recipe to populate this panel";

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

  if (layout_ok && layout["schema_version"] && layout["schema_version"].as<std::string>() == "workcell_studio_layout/v1" && layout["items"] && layout["items"].IsSequence()) {
    for (const auto & node : layout["items"]) {
      if (!node["id"]) continue;
      const auto id = node["id"].as<std::string>();
      bool matched_existing = false;
      for (auto & item : m.items) {
        if (item.id == id) {
          matched_existing = true;
          if (node["pose"]) {
            auto p = node["pose"];
            if (p["xyz"] && p["xyz"].IsSequence() && p["xyz"].size() >= 3) { item.x = p["xyz"][0].as<double>(); item.y = p["xyz"][1].as<double>(); item.z = p["xyz"][2].as<double>(); }
            if (p["rpy"] && p["rpy"].IsSequence() && p["rpy"].size() >= 3) { item.roll = p["rpy"][0].as<double>(); item.pitch = p["rpy"][1].as<double>(); item.yaw = p["rpy"][2].as<double>(); }
          }
        }
      }
      if (!matched_existing) {
        WorkcellStudioCanvasItem extra;
        extra.id = id;
        extra.type = node["type"] ? node["type"].as<std::string>() : "object";
        extra.role = node["role"] ? node["role"].as<std::string>() : "preview";
        const auto category = node["category"] ? node["category"].as<std::string>() : "Custom / Imported";
        if (category == "Pick/Place Zones") extra.type = "zone";
        extra.label = node["display_name"] ? node["display_name"].as<std::string>() : id;
        extra.source_file = node["source_path"] ? node["source_path"].as<std::string>() : "layout/workcell_studio_layout.yaml";
        if (node["pose"]) {
          auto p = node["pose"];
          if (p["xyz"] && p["xyz"].IsSequence() && p["xyz"].size() >= 3) { extra.x = p["xyz"][0].as<double>(); extra.y = p["xyz"][1].as<double>(); extra.z = p["xyz"][2].as<double>(); }
          if (p["rpy"] && p["rpy"].IsSequence() && p["rpy"].size() >= 3) { extra.roll = p["rpy"][0].as<double>(); extra.pitch = p["rpy"][1].as<double>(); extra.yaw = p["rpy"][2].as<double>(); }
        }
        if (node["size"]) {
          if (node["size"].IsMap()) {
            if (node["size"]["width"]) extra.width = node["size"]["width"].as<double>();
            if (node["size"]["depth"]) extra.depth = node["size"]["depth"].as<double>();
            if (node["size"]["height"]) extra.height = node["size"]["height"].as<double>();
          } else if (node["size"].IsSequence() && node["size"].size() >= 3) {
            extra.width = node["size"][0].as<double>(); extra.depth = node["size"][1].as<double>(); extra.height = node["size"][2].as<double>();
          }
        }
        extra.locked = node["locked"] ? node["locked"].as<bool>() : false;
        m.items.push_back(extra);
      }
    }
  }
  if (!m.warnings.empty()) { m.has_warnings = true; m.status = "WARNINGS"; m.items.push_back({"warning","warning","warning","warning","environment.yaml",-1.2,1.2,0,0,0,0,0.1,0.1,0,m.warnings}); }
  else { m.status = "READY"; }
  return m;
}
}
