#pragma once
#include "inventory_compare.hh"
namespace workcell {
// Check observed lookup relationships; synthetic observations are tests only.
inline Json::Value CheckRendererIdentity(Json::Value r) {
  r["complete"]=false;r["visuals"]=Json::Value(Json::arrayValue);r["failures"]=Json::Value(Json::arrayValue);
  auto fail=[&](const std::string &condition,const std::string &path) {
    Json::Value f;f["condition"]=condition;f["path"]=path;r["failures"].append(f);
    if(!r.isMember("first_failure"))r["first_failure"]=f;
  };
  if(!r["scene_present"].asBool())fail("missing_scene","/scene_present");
  Json::UInt64 world=0,expectedWorld=0;
  if(!InventoryId(r["manager_world_id"],world)||!InventoryId(r["world_entity"],expectedWorld)||world!=expectedWorld)
    fail("world_identity_mismatch","/manager_world_id");
  std::set<Json::UInt64> expectedIds,rendererIds;
  for(const auto &v:r["expected_visuals"]) {
    Json::UInt64 id=0,link=0,model=0,rid=0;
    const auto path="/expected_visuals/"+std::to_string(v["visual_id"].isUInt64()?v["visual_id"].asUInt64():0);
    if(!InventoryId(v["visual_id"],id)||!InventoryId(v["link_id"],link)||!InventoryId(v["model_id"],model))fail("invalid_full_width_expected_id",path);
    if(!expectedIds.insert(id).second)fail("duplicate_expected_visual_id",path);
    if(v["link_matches"].asUInt()!=1)fail("ambiguous_link_owner",path+"/link_matches");
    if(v["model_matches"].asUInt()!=1)fail("ambiguous_model_owner",path+"/model_matches");
    if(!v["model_world_matches"].asBool())fail("model_world_mismatch",path+"/model_world_matches");
    for(const auto key:{"visual","link","model"})if(!v["lookups"][key]["found"].asBool())fail(std::string("missing_")+key,path+"/lookups/"+key);
    if(!v["visual_parent_is_link"].asBool())fail("visual_parent_mismatch",path+"/lookups/visual");
    if(!v["link_parent_is_model"].asBool())fail("link_parent_mismatch",path+"/lookups/link");
    if(!v["model_parent_is_scene_root"].asBool())fail("model_scene_root_mismatch",path+"/lookups/model");
    if(v["duplicate_visual_pointer"].asBool())fail("duplicate_visual_mapping",path);
    if(v["lookups"]["visual"]["found"].asBool()) {
      if(!InventoryId(v["lookups"]["visual"]["renderer_id"],rid)||!rendererIds.insert(rid).second)fail("invalid_or_duplicate_renderer_id",path);
      Json::Value item;item["visual_id"]=id;item["link_id"]=link;item["model_id"]=model;item["renderer_id"]=rid;r["visuals"].append(item);
    }
  }
  if(r["expected_visuals"].empty())fail("missing_expected_visual_inventory","/expected_visuals");
  unsigned index=0;
  for(const auto &v:r["scene_visuals"]) {
    const auto path="/scene_visuals/"+std::to_string(index++);
    if(!v["found"].asBool())fail("missing_scene_visual",path);
    else if(v["geometry_count"].asUInt()>0&&!v["mapped_expected_visual"].asBool())fail("unexpected_geometry_visual",path);
  }
  r["complete"]=r["failures"].empty();return r;
}
}
