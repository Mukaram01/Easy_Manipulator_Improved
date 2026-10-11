// CPU-only tests of recorded observations, never a live-renderer substitute.
#include "renderer_identity_check.hh"
#include <stdexcept>
#include <iostream>
#define check(x) do {if(!(x))throw std::runtime_error("renderer identity line "+std::to_string(__LINE__));}while(false)
Json::Value observations() {
  Json::Value r;r["scene_present"]=true;r["manager_world_id"]=Json::UInt64(1);r["world_entity"]=Json::UInt64(1);
  r["expected_visuals"]=Json::Value(Json::arrayValue);r["scene_visuals"]=Json::Value(Json::arrayValue);
  Json::Value v;v["visual_id"]=Json::UInt64(4294967300ull);v["link_id"]=Json::UInt64(4294967301ull);v["model_id"]=Json::UInt64(4294967302ull);
  v["link_matches"]=1;v["model_matches"]=1;v["model_world_matches"]=true;
  for(auto key:{"visual","link","model"})v["lookups"][key]["found"]=true;
  v["lookups"]["visual"]["renderer_id"]=22;
  v["visual_parent_is_link"]=true;v["link_parent_is_model"]=true;v["model_parent_is_scene_root"]=true;v["duplicate_visual_pointer"]=false;
  r["expected_visuals"].append(v);Json::Value s;s["found"]=true;s["geometry_count"]=1;s["mapped_expected_visual"]=true;r["scene_visuals"].append(s);return r;
}
int main() {
  auto r=workcell::CheckRendererIdentity(observations());check(r["complete"].asBool());
  check(r["visuals"][0]["visual_id"].asUInt64()==4294967300ull);
  for(auto key:{"visual","link","model"}) {
    auto a=observations();a["expected_visuals"][0]["lookups"][key]["found"]=false;
    r=workcell::CheckRendererIdentity(a);check(!r["complete"].asBool());
    check(r["first_failure"]["condition"]==std::string("missing_")+key);
    check(r["expected_visuals"]==a["expected_visuals"]);
  }
  for(auto key:{"visual_parent_is_link","link_parent_is_model","model_parent_is_scene_root","model_world_matches"}) {
    auto a=observations();a["expected_visuals"][0][key]=false;
    check(!workcell::CheckRendererIdentity(a)["complete"].asBool());
  }
  for(auto key:{"scene_present","world_mismatch","unexpected_geometry","duplicate","ambiguous_link","ambiguous_model","null_scene_visual","duplicate_pointer"}) {
    auto a=observations();std::string k=key;
    if(k=="scene_present")a["scene_present"]=false;
    if(k=="world_mismatch")a["manager_world_id"]=2;
    if(k=="unexpected_geometry")a["scene_visuals"][0]["mapped_expected_visual"]=false;
    if(k=="duplicate")a["expected_visuals"].append(a["expected_visuals"][0]);
    if(k=="ambiguous_link")a["expected_visuals"][0]["link_matches"]=2;
    if(k=="ambiguous_model")a["expected_visuals"][0]["model_matches"]=2;
    if(k=="null_scene_visual")a["scene_visuals"][0]["found"]=false;
    if(k=="duplicate_pointer")a["expected_visuals"][0]["duplicate_visual_pointer"]=true;
    r=workcell::CheckRendererIdentity(a);check(!r["complete"].asBool()&&!r["failures"].empty());
  }
  std::cout<<"PASS full-width IDs, complete mapping, missing lookups, parent/world mismatch, duplicate/ambiguous mapping, unexpected geometry, retained observations\n";
}
