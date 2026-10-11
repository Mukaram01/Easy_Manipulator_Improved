// Read-only topology snapshot from the SAME ECM passed to OwnerRecord.
#pragma once
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Transparency.hh>
#include <sdf/Box.hh>
#include <jsoncpp/json/json.h>
namespace workcell {
inline Json::Value IdentityInventory(const ignition::gazebo::EntityComponentManager &ecm) {
  namespace c=ignition::gazebo::components;
  Json::Value result;
  auto row=[&](ignition::gazebo::Entity id,bool geometry,bool visual) {
    Json::Value r;r["id"]=Json::UInt64(id);
    const auto parent=ecm.Component<c::ParentEntity>(id);
    r["parent"]=Json::UInt64(parent ? parent->Data() : 0);
    if(geometry) {
      r["geometry"]["type"]="UNSUPPORTED_OR_MISSING";
      const auto g=ecm.Component<c::Geometry>(id);
      if(g && g->Data().Type()==sdf::GeometryType::BOX && g->Data().BoxShape()) {
        r["geometry"]["type"]="BOX";
        r["geometry"]["size"]=Json::Value(Json::arrayValue);
        const auto size=g->Data().BoxShape()->Size();
        for(int i=0;i<3;++i)r["geometry"]["size"].append(size[i]);
      }
    }
    if(visual) {
      const auto m=ecm.Component<c::Material>(id);
      const auto t=ecm.Component<c::Transparency>(id);
      r["opaque"]=m && t && t->Data()==0 && m->Data().Diffuse().A()==1 &&
        m->Data().Ambient().A()==1 && m->Data().ScriptUri().empty() &&
        m->Data().ScriptName().empty() && !m->Data().PbrMaterial();
    }
    return r;
  };
  // Single-component enumeration retains entities with missing required data.
  // A filtered multi-component Each would silently omit those failures.
#define WORKCELL_IDENTITIES(Component, key, geometry, visual) \
  result[key]=Json::Value(Json::arrayValue); \
  ecm.Each<c::Component>([&](const auto &id,const auto*){ \
    result[key].append(row(id,geometry,visual));return true;});
  WORKCELL_IDENTITIES(World,"worlds",false,false)
  WORKCELL_IDENTITIES(Model,"models",false,false)
  WORKCELL_IDENTITIES(Link,"links",false,false)
  WORKCELL_IDENTITIES(Visual,"visuals",true,true)
  WORKCELL_IDENTITIES(Collision,"collisions",true,false)
#undef WORKCELL_IDENTITIES
  result["complete"]=true; // enumeration complete; join validates every row
  return result;
}
}
