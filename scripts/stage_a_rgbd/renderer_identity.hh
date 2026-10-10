// Lookup adapter only. Does not render, infer a step, or attest an MRT draw.
#pragma once
#include <ignition/gazebo/rendering/SceneManager.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#include <jsoncpp/json/json.h>
#include <set>
namespace workcell {
inline Json::Value RendererIdentity(ignition::gazebo::SceneManager &manager,
    const Json::Value &owner,const std::string &fingerprint) {
  Json::Value r;
  for(const auto key:{"session","world_entity","step","stamp_ns"})r[key]=owner[key];
  r["scene_fingerprint"]=fingerprint;
  r["lookup_source"]="GazeboSceneManager::VisualById";
  r["complete"]=false;r["visuals"]=Json::Value(Json::arrayValue);
  r["mrt_binding"]="BLOCKED_NOT_WITNESSED";
  r["timing_authority"]="CALLER_MUST_VERIFY_LIVE_RENDER_PHYSICS_ALIGNMENT";
  const auto scene=manager.Scene();
  if(!scene || manager.WorldId()!=owner["world_entity"].asUInt64())return r;
  r["scene_id"]=scene->Id();
  std::set<ignition::rendering::VisualPtr> mapped;
  for(const auto &v:owner["identity_inventory"]["visuals"]) {
    const auto id=v["id"].asUInt64(),link=v["parent"].asUInt64();
    Json::UInt64 model=0;
    for(const auto &l:owner["identity_inventory"]["links"])
      if(l["id"].asUInt64()==link)model=l["parent"].asUInt64();
    const auto visual=manager.VisualById(id),parent=manager.VisualById(link),root=manager.VisualById(model);
    if(!visual || !parent || !root || visual->Parent()!=parent || parent->Parent()!=root ||
       !mapped.insert(visual).second)return r;
    Json::Value item;item["visual_id"]=id;item["link_id"]=link;item["model_id"]=model;
    item["renderer_id"]=visual->Id();r["visuals"].append(item);
  }
  for(unsigned i=0;i<scene->VisualCount();++i) {
    const auto visual=scene->VisualByIndex(i);
    if(!visual || (visual->GeometryCount()>0 && !mapped.count(visual)))return r;
  }
  r["complete"]=true;return r;
}
}
