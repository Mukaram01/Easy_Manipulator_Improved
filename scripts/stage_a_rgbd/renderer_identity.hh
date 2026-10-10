// Lookup adapter only. Does not render, infer a step, or attest an MRT draw.
#pragma once
#include <ignition/gazebo/rendering/SceneManager.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>
#include "renderer_identity_check.hh"
namespace workcell {
inline Json::Value RendererNodeWitness(const ignition::rendering::NodePtr &node) {
  Json::Value r;r["found"]=static_cast<bool>(node);
  if(node) {
    r["renderer_id"]=Json::UInt64(node->Id());const auto parent=node->Parent();
    r["parent_found"]=static_cast<bool>(parent);
    if(parent)r["parent_renderer_id"]=Json::UInt64(parent->Id());
  }
  return r;
}
inline Json::Value RendererIdentity(ignition::gazebo::SceneManager &manager,
    const Json::Value &owner,const std::string &fingerprint) {
  Json::Value r;
  for(const auto key:{"session","world_entity","step","stamp_ns"})r[key]=owner[key];
  r["scene_fingerprint"]=fingerprint;r["lookup_source"]="GazeboSceneManager::VisualById";
  r["mrt_binding"]="BLOCKED_NOT_WITNESSED";
  r["timing_authority"]="CALLER_MUST_VERIFY_LIVE_RENDER_PHYSICS_ALIGNMENT";
  const auto scene=manager.Scene();r["scene_present"]=static_cast<bool>(scene);
  r["manager_world_id"]=Json::UInt64(manager.WorldId());
  if(scene){r["scene_id"]=Json::UInt64(scene->Id());r["scene_root_visual"]=RendererNodeWitness(scene->RootVisual());}
  r["expected_visuals"]=Json::Value(Json::arrayValue);r["scene_visuals"]=Json::Value(Json::arrayValue);
  std::set<ignition::rendering::VisualPtr> mapped;
  for(const auto &v:owner["identity_inventory"]["visuals"]) {
    const auto id=v["id"].asUInt64(),link=v["parent"].asUInt64();
    Json::UInt64 model=0;unsigned linkMatches=0,modelMatches=0;Json::UInt64 modelParent=0;
    for(const auto &l:owner["identity_inventory"]["links"])if(l["id"].asUInt64()==link){model=l["parent"].asUInt64();++linkMatches;}
    for(const auto &m:owner["identity_inventory"]["models"])if(m["id"].asUInt64()==model){modelParent=m["parent"].asUInt64();++modelMatches;}
    const auto visual=manager.VisualById(id),parent=manager.VisualById(link),root=manager.VisualById(model);
    Json::Value item;item["visual_id"]=id;item["link_id"]=link;item["model_id"]=model;
    item["expected_visual_parent_gazebo_id"]=link;item["expected_link_parent_gazebo_id"]=model;
    item["expected_model_parent_gazebo_id"]=owner["world_entity"];
    item["actual_model_parent_gazebo_id"]=modelParent;
    item["link_matches"]=linkMatches;item["model_matches"]=modelMatches;
    item["model_world_matches"]=modelParent==owner["world_entity"].asUInt64();
    item["lookups"]["visual"]=RendererNodeWitness(visual);
    item["lookups"]["link"]=RendererNodeWitness(parent);item["lookups"]["model"]=RendererNodeWitness(root);
    if(parent)item["expected_visual_parent_renderer_id"]=Json::UInt64(parent->Id());
    if(root)item["expected_link_parent_renderer_id"]=Json::UInt64(root->Id());
    item["visual_parent_is_link"]=visual&&parent&&visual->Parent()==parent;
    item["link_parent_is_model"]=parent&&root&&parent->Parent()==root;
    item["model_parent_is_scene_root"]=root&&scene&&root->Parent()==scene->RootVisual();
    item["duplicate_visual_pointer"]=visual&&!mapped.insert(visual).second;
    r["expected_visuals"].append(item);
  }
  if(scene)for(unsigned i=0;i<scene->VisualCount();++i) {
    const auto visual=scene->VisualByIndex(i);auto item=RendererNodeWitness(visual);item["index"]=i;
    if(visual){item["geometry_count"]=visual->GeometryCount();item["mapped_expected_visual"]=mapped.count(visual)!=0;}
    r["scene_visuals"].append(item);
  }
  return CheckRendererIdentity(r);
}
}
