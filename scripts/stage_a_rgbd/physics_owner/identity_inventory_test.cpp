#include "identity_inventory.hh"
#include "../renderer_identity.hh"
#include <sdf/Geometry.hh>
#include <sdf/Material.hh>
#include <stdexcept>
#include <iostream>
#define check(condition) do {if(!(condition))throw std::runtime_error( \
    "identity inventory regression line " + std::to_string(__LINE__));} while(false)
int main() {
  namespace g=ignition::gazebo;namespace c=g::components;
  g::EntityComponentManager ecm;
  const auto world=ecm.CreateEntity(),model=ecm.CreateEntity(),link=ecm.CreateEntity();
  const auto visual=ecm.CreateEntity(),collision=ecm.CreateEntity();
  ecm.CreateComponent(world,c::World());ecm.CreateComponent(model,c::Model());
  ecm.CreateComponent(link,c::Link());ecm.CreateComponent(visual,c::Visual());
  ecm.CreateComponent(collision,c::Collision());
  for(const auto pair:{std::make_pair(model,world),{link,model},{visual,link},{collision,link}})
    ecm.CreateComponent(pair.first,c::ParentEntity(pair.second));
  sdf::Box box;box.SetSize({.025,.025,.025});sdf::Geometry geometry;
  geometry.SetType(sdf::GeometryType::BOX);geometry.SetBoxShape(box);
  ecm.CreateComponent(visual,c::Geometry(geometry));ecm.CreateComponent(collision,c::Geometry(geometry));
  sdf::Material material;material.SetDiffuse({1,0,0,1});material.SetAmbient({1,0,0,1});
  ecm.CreateComponent(visual,c::Material(material));ecm.CreateComponent(visual,c::Transparency(0));
  auto inventory=workcell::IdentityInventory(ecm);
  check(inventory["complete"].asBool() && inventory["visuals"].size()==1 && inventory["collisions"].size()==1);
  check(inventory["visuals"][0]["id"].asUInt64()==visual && inventory["visuals"][0]["parent"].asUInt64()==link);
  check(inventory["visuals"][0]["opaque"].asBool() && inventory["collisions"][0]["geometry"]["type"]=="BOX");
  // Missing parent/shape/material must remain in the snapshot, never disappear.
  const auto missing=ecm.CreateEntity();ecm.CreateComponent(missing,c::Visual());
  inventory=workcell::IdentityInventory(ecm);check(inventory["visuals"].size()==2);
  Json::Value row;for(const auto &v:inventory["visuals"])if(v["id"].asUInt64()==missing)row=v;
  check(row["id"].asUInt64()==missing && row["parent"].asUInt64()==0 && !row["opaque"].asBool());
  check(row["geometry"]["type"]=="UNSUPPORTED_OR_MISSING");
  ignition::gazebo::SceneManager manager;
  Json::Value owner;owner["world_entity"]=Json::UInt64(world);owner["identity_inventory"]=inventory;
  check(!workcell::RendererIdentity(manager,owner,"not-live")["complete"].asBool());
  // Actual ECM extraction output for reproducible CPU-only inspection.
  std::cout<<inventory<<std::endl;
}
