#pragma once
// Diagnostic only: no classification can remove an entity or grant ownership.
#include "inventory_compare.hh"
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/LightType.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/World.hh>
namespace workcell {
inline Json::Value InventoryEcmDiagnostics(const ignition::gazebo::EntityComponentManager &ecm,
    const Json::Value &owner,const Json::Value &render) {
  namespace c=ignition::gazebo::components;
  std::set<Json::UInt64> requested;
  for(const auto &inv:{owner,render})for(const auto key:{"worlds","models","links","visuals","collisions"})
    for(const auto &row:inv[key]) {Json::UInt64 id=0;if(InventoryId(row["id"],id))requested.insert(id);}
  Json::Value out(Json::arrayValue);
  for(const auto entity:requested) {
    Json::Value entry;entry["entity_id"]=entity;std::set<Json::UInt64> visited;auto id=entity;
    while(id && visited.insert(id).second) {
      Json::Value node;node["id"]=id;node["exists"]=ecm.HasEntity(id);
      if(!ecm.HasEntity(id)){entry["parent_chain"].append(node);id=0;break;}
      std::set<ignition::gazebo::ComponentTypeId> types;
      for(const auto type:ecm.ComponentTypes(id))types.insert(type);
      for(const auto type:types) {
        Json::Value item;item["type_id"]=Json::UInt64(type);item["registered_name"]=c::Factory::Instance()->Name(type);
        node["components"].append(item);
      }
      const auto name=ecm.Component<c::Name>(id);if(name)node["diagnostic_name_not_identity"]=name->Data();
      const auto parent=ecm.Component<c::ParentEntity>(id);const auto parentId=parent?parent->Data():0;
      node["parent_id"]=Json::UInt64(parentId);
      node["has_visual"]=ecm.Component<c::Visual>(id)!=nullptr;
      node["has_collision"]=ecm.Component<c::Collision>(id)!=nullptr;
      node["has_link"]=ecm.Component<c::Link>(id)!=nullptr;
      node["has_model"]=ecm.Component<c::Model>(id)!=nullptr;
      node["has_world"]=ecm.Component<c::World>(id)!=nullptr;
      node["has_geometry"]=ecm.Component<c::Geometry>(id)!=nullptr;
      node["has_light"]=ecm.Component<c::Light>(id)!=nullptr;
      const auto lightType=ecm.Component<c::LightType>(id);if(lightType)node["light_type_component"]=lightType->Data();
      if(node["has_visual"].asBool() && !node["has_geometry"].asBool() && parent && ecm.Component<c::Light>(parentId))
        node["classification"]="visual_child_of_authoritative_Light_entity_without_Geometry";
      entry["parent_chain"].append(node);id=parentId;
    }
    if(id && visited.count(id))entry["cycle"]=true;
    out.append(entry);
  }
  return out;
}
}
