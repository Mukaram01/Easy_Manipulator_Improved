#pragma once
#include <jsoncpp/json/json.h>
#include <string>
#include <cmath>
inline std::string CheckMaterialContract(const Json::Value &r) {
  if(!r["ecm_material_present"].asBool() || !r["ecm_transparency_present"].asBool())return "missing_ecm_material_or_transparency";
  if(!r["ecm_transparency"].isNumeric() || r["ecm_transparency"].asDouble()!=0)return "nonopaque_ecm_transparency";
  for(const auto key:{"diffuse","ambient"}) {
    const auto &c=r[key];if(!c.isArray() || c.size()!=4)return "missing_ecm_colour";
    for(const auto &v:c)if(!v.isNumeric() || !std::isfinite(v.asDouble()) || v.asDouble()<0 || v.asDouble()>1)return "invalid_ecm_colour";
    if(c[3].asDouble()!=1)return "nonopaque_ecm_alpha";
  }
  if(!r["script_uri"].isString() || !r["script_uri"].asString().empty() ||
     !r["script_name"].isString() || !r["script_name"].asString().empty() || r["pbr"].asBool())return "unsupported_ecm_material_profile";
  if(!r["geometry_material"]["present"].asBool())return "missing_geometry_material";
  for(const auto key:{"visual_material","geometry_material"}) {
    const auto &m=r[key];if(!m["present"].asBool())continue;
    if(!m["transparency"].isNumeric() || m["transparency"].asDouble()!=0)return "nonopaque_native_material";
    const auto &c=m["diffuse"];if(!c.isArray() || c.size()!=4)return "missing_native_colour";
    for(unsigned i=0;i<4;++i)if(!c[i].isNumeric() || c[i].asDouble()!=r["diffuse"][i].asDouble())return "native_ecm_colour_disagreement";
  }
  if(!r["hlms_pbs"].asBool() || !r["native_opaque_blend"].asBool() || !r["alpha_test_disabled"].asBool())return "unsupported_original_hlms_state";
  return "";
}
