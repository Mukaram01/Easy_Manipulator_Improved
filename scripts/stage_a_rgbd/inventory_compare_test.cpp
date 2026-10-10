// Pure CPU JsonCpp contract: no Gazebo server, graphics or physics engine.
#include "inventory_compare.hh"
#include <sstream>
#include <iostream>
#include <stdexcept>
#define check(x) do { if(!(x)) throw std::runtime_error("inventory comparison line " + std::to_string(__LINE__)); } while(false)
Json::Value fixture() {
  Json::Value r;std::istringstream in(R"({"complete":true,"worlds":[{"id":1,"parent":0}],"models":[{"id":2,"parent":1}],"links":[{"id":3,"parent":2}],"visuals":[{"id":4,"parent":3,"geometry":{"type":"BOX","size":[0.025,0.025,0.025]},"opaque":true}],"collisions":[{"id":5,"parent":3,"geometry":{"type":"BOX","size":[0.025,0.025,0.025]}}]})");in>>r;return r;
}
int main() {
  auto a=fixture();for(auto key:{"worlds","models","links","visuals","collisions"})
    for(auto &row:a[key])for(auto field:{"id","parent"})row[field]=Json::UInt64(row[field].asUInt64());
  Json::StreamWriterBuilder w;w["precision"]=17;
  Json::Value b;std::istringstream in(Json::writeString(w,a));in>>b;
  check(a!=b); // Reproduce the exact raw-equality defect across JSON round trip.
  auto r=workcell::CompareInventories(a,b);
  check(r["equivalent"].asBool() && r["schema_valid"].asBool() && r["supported_geometry"].asBool());
  check(!r["representations"].empty() && r["differences"].empty());
  check(r["owner_inventory"]==a && r["render_inventory"]==b);
  a["visuals"][0]["id"]=Json::UInt64(18446744073709551615ull);
  b["visuals"][0]["id"]=Json::UInt64(18446744073709551615ull);
  check(workcell::CompareInventories(a,b)["equivalent"].asBool());
  b["visuals"][0]["id"]=Json::UInt64(18446744073709551614ull);
  r=workcell::CompareInventories(a,b);check(!r["equivalent"].asBool());
  check(r["differences"].size()==2);
  b=a;b["visuals"].append(a["visuals"][0]);
  check(!workcell::CompareInventories(a,b)["schema_valid"].asBool());
  b=a;b["visuals"][0]["id"]=-1;
  check(!workcell::CompareInventories(a,b)["schema_valid"].asBool());
  b=a;b["visuals"][0]["id"]=4.0;
  check(!workcell::CompareInventories(a,b)["schema_valid"].asBool());
  b=a;b["visuals"][0]["parent"]=Json::UInt64(999);
  check(!workcell::CompareInventories(a,b)["schema_valid"].asBool());
  a=fixture();a["visuals"][0]["geometry"]["type"]="UNSUPPORTED_OR_MISSING";b=a;
  r=workcell::CompareInventories(a,b);check(r["equivalent"].asBool() && !r["supported_geometry"].asBool());
  a=fixture();b=a;b["visuals"][0]["geometry"]["size"][0]=.026;
  r=workcell::CompareInventories(a,b);check(!r["equivalent"].asBool());
  check(r["differences"][0]["path"]=="/visuals/4/geometry/size/0");
  a=fixture();b=a;b["visuals"]=Json::Value(Json::arrayValue);
  check(!workcell::CompareInventories(a,b)["equivalent"].asBool());
  a=fixture();Json::Value extra;extra["id"]=Json::UInt64(13);extra["parent"]=Json::UInt64(12);
  extra["geometry"]["type"]="UNSUPPORTED_OR_MISSING";extra["opaque"]=false;
  a["visuals"].append(extra);b=a;r=workcell::CompareInventories(a,b);
  check(r["equivalent"].asBool() && !r["schema_valid"].asBool() && !r["supported_geometry"].asBool());
  check(r["owner_inventory"]["visuals"].size()==2);
  a=fixture();extra=a["visuals"][0];extra["id"]=6;a["visuals"].append(extra);b=a;
  check(!workcell::CompareInventories(a,b)["schema_valid"].asBool());
  a=fixture();b=a;b["collisions"][0]["geometry"]["size"][0]=std::numeric_limits<double>::infinity();
  check(!workcell::CompareInventories(a,b)["supported_geometry"].asBool());
  std::cout<<"PASS lossless round trip, signed/unsigned, full uint64, exact fields, missing/extra, duplicate, ambiguous ownership, unsupported geometry\n";
}
