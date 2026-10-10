#pragma once
// Exact comparison, not a tolerance or permission. Original values are retained.
#include <jsoncpp/json/json.h>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <limits>
namespace workcell {
inline bool InventoryId(const Json::Value &v,Json::UInt64 &id) {
  if(v.type()!=Json::uintValue && v.type()!=Json::intValue)return false;
  if(v.type()==Json::intValue && v.asInt64()<0)return false;
  id=v.asUInt64();return true;
}
inline void InventoryDifference(Json::Value &r,const std::string &path,
    const Json::Value &a,const Json::Value &b,const std::string &reason) {
  Json::Value d;d["path"]=path;d["owner"]=a;d["render"]=b;d["reason"]=reason;
  d["owner_json_type"]=a.type();d["render_json_type"]=b.type();r["differences"].append(d);
}
inline void InventoryValue(const Json::Value &a,const Json::Value &b,
    const std::string &path,Json::Value &r) {
  Json::UInt64 ua=0,ub=0;
  if(InventoryId(a,ua) && InventoryId(b,ub) && ua==ub) {
    if(a.type()!=b.type()) {
      Json::Value d;d["path"]=path;d["owner_json_type"]=a.type();d["render_json_type"]=b.type();
      d["exact_uint64"]=ua;d["reason"]="equal_integer_value_different_signedness";r["representations"].append(d);
    }
    return;
  }
  if(a.type()!=b.type()) {InventoryDifference(r,path,a,b,"different_value_or_numeric_kind");return;}
  if(a.isObject()) {
    std::set<std::string> keys;for(const auto &k:a.getMemberNames())keys.insert(k);for(const auto &k:b.getMemberNames())keys.insert(k);
    for(const auto &k:keys) {
      if(!a.isMember(k)||!b.isMember(k))InventoryDifference(r,path+"/"+k,a[k],b[k],"missing_field");
      else InventoryValue(a[k],b[k],path+"/"+k,r);
    }
  } else if(a.isArray()) {
    if(a.size()!=b.size())InventoryDifference(r,path,a,b,"array_length");
    for(Json::ArrayIndex i=0;i<std::min(a.size(),b.size());++i)InventoryValue(a[i],b[i],path+"/"+std::to_string(i),r);
  } else if(a.isDouble() && (!std::isfinite(a.asDouble())||!std::isfinite(b.asDouble()))) {
    InventoryDifference(r,path,a,b,"nonfinite_number");
  } else if(a!=b)InventoryDifference(r,path,a,b,"different_value");
}
inline Json::Value CompareInventories(const Json::Value &owner,const Json::Value &render) {
  Json::Value r;r["owner_inventory"]=owner;r["render_inventory"]=render;
  for(const auto key:{"differences","representations","issues"})r[key]=Json::Value(Json::arrayValue);
  r["schema_valid"]=true;r["supported_geometry"]=true;
  using Rows=std::map<Json::UInt64,Json::Value>;
  using Groups=std::map<std::string,Rows>;
  auto issue=[&](const std::string &side,const std::string &path,const std::string &reason,bool geometry=false) {
    Json::Value d;d["side"]=side;d["path"]=path;d["reason"]=reason;r["issues"].append(d);
    r[geometry?"supported_geometry":"schema_valid"]=false;
  };
  const std::set<std::string> kinds={"worlds","models","links","visuals","collisions"};
  auto index=[&](const Json::Value &v,const std::string &side) {
    Groups groups;std::set<Json::UInt64> all;
    if(!v.isObject()){issue(side,"/","inventory_not_object");return groups;}
    if(!v["complete"].isBool() || !v["complete"].asBool())issue(side,"/complete","incomplete_inventory");
    for(const auto &key:v.getMemberNames())if(key!="complete"&&!kinds.count(key))issue(side,"/"+key,"unknown_inventory_field");
    for(const auto &kind:kinds) {
      if(!v[kind].isArray()){issue(side,"/"+kind,"missing_entity_array");continue;}
      for(const auto &row:v[kind]) {
        Json::UInt64 id=0,parent=0;
        if(!row.isObject() || !InventoryId(row["id"],id) || id==0) {issue(side,"/"+kind,"invalid_full_width_entity_id");continue;}
        const auto path="/"+kind+"/"+std::to_string(id);
        if(!all.insert(id).second)issue(side,path,"duplicate_entity_id_or_roles");
        if(!groups[kind].emplace(id,row).second)issue(side,path,"duplicate_entity_id");
        if(!InventoryId(row["parent"],parent))issue(side,path+"/parent","invalid_parent_id");
        const bool geometry=kind=="visuals"||kind=="collisions",visual=kind=="visuals";
        std::set<std::string> fields={"id","parent"};if(geometry)fields.insert("geometry");if(visual)fields.insert("opaque");
        for(const auto &k:row.getMemberNames())if(!fields.count(k))issue(side,path+"/"+k,"unknown_entity_field");
        for(const auto &k:fields)if(!row.isMember(k))issue(side,path+"/"+k,"missing_entity_field");
        if(geometry) {
          const auto &g=row["geometry"];bool box=g.isObject()&&g["type"]=="BOX"&&g["size"].isArray()&&g["size"].size()==3;
          if(box)for(const auto &x:g["size"])if(!x.isNumeric()||!std::isfinite(x.asDouble())||x.asDouble()<=0)box=false;
          if(!box)issue(side,path+"/geometry","unsupported_or_invalid_BOX_geometry",true);
          if(g.isObject())for(const auto &k:g.getMemberNames())if(k!="type"&&k!="size")issue(side,path+"/geometry/"+k,"unknown_geometry_field");
        }
        if(visual && (!row["opaque"].isBool()||!row["opaque"].asBool()))issue(side,path+"/opaque","unsupported_material",true);
      }
    }
    if(groups["worlds"].size()!=1)issue(side,"/worlds","requires_one_world");
    for(const auto &kind:kinds)for(const auto &[id,row]:groups[kind]) {
      Json::UInt64 parent=0;if(!InventoryId(row["parent"],parent))continue;
      const auto path="/"+kind+"/"+std::to_string(id);
      const auto parentKind=kind=="models"?"worlds":kind=="links"?"models":"links";
      if(kind=="worlds") {if(parent!=0)issue(side,path+"/parent","world_has_parent");}
      else if(!groups[parentKind].count(parent))issue(side,path+"/parent","missing_or_unsupported_parent_role");
    }
    for(const auto &[link,row]:groups["links"]) {
      unsigned visuals=0,collisions=0;
      for(const auto &[id,v]:groups["visuals"]) {Json::UInt64 p=0;if(InventoryId(v["parent"],p)&&p==link)++visuals;}
      for(const auto &[id,v]:groups["collisions"]) {Json::UInt64 p=0;if(InventoryId(v["parent"],p)&&p==link)++collisions;}
      if(visuals!=1||collisions!=1)issue(side,"/links/"+std::to_string(link),"ambiguous_or_incomplete_visual_collision_ownership");
    }
    return groups;
  };
  auto a=index(owner,"owner"),b=index(render,"render");
  InventoryValue(owner["complete"],render["complete"],"/complete",r);
  std::set<std::string> extraKeys;
  if(owner.isObject())for(const auto &k:owner.getMemberNames())if(k!="complete"&&!kinds.count(k))extraKeys.insert(k);
  if(render.isObject())for(const auto &k:render.getMemberNames())if(k!="complete"&&!kinds.count(k))extraKeys.insert(k);
  for(const auto &k:extraKeys) {
    if(!owner.isMember(k)||!render.isMember(k))InventoryDifference(r,"/"+k,owner[k],render[k],"missing_field");
    else InventoryValue(owner[k],render[k],"/"+k,r);
  }
  for(const auto &kind:kinds) {
    if(!owner.isMember(kind)||!render.isMember(kind))InventoryDifference(r,"/"+kind,owner[kind],render[kind],"missing_entity_array");
    // Invalid/duplicate IDs cannot be canonicalised losslessly: compare every
    // retained array element as well, and keep schema_valid false.
    if(!owner[kind].isArray()||!render[kind].isArray() ||
       a[kind].size()!=owner[kind].size()||b[kind].size()!=render[kind].size())
      InventoryValue(owner[kind],render[kind],"/"+kind,r);
    std::set<Json::UInt64> ids;for(const auto &[id,row]:a[kind])ids.insert(id);for(const auto &[id,row]:b[kind])ids.insert(id);
    for(const auto id:ids) {
      const auto path="/"+kind+"/"+std::to_string(id);
      if(!a[kind].count(id)||!b[kind].count(id))InventoryDifference(r,path,a[kind].count(id)?a[kind].at(id):Json::Value(),b[kind].count(id)?b[kind].at(id):Json::Value(),"missing_or_extra_entity");
      else InventoryValue(a[kind].at(id),b[kind].at(id),path,r);
    }
  }
  r["equivalent"]=r["differences"].empty();
  return r;
}
}
