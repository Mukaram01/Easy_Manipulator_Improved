// Synthetic CPU contract tests. Not runtime material evidence.
#include "material_contract.hh"
#include <cassert>
Json::Value fixture() {
  Json::Value r;r["ecm_material_present"]=r["ecm_transparency_present"]=true;
  r["visual_material"]["present"]=false;
  r["ecm_transparency"]=0.;r["script_uri"]="";r["script_name"]="";r["pbr"]=false;
  for(double x:{.72,.44,.16,1.})r["diffuse"].append(x);
  for(double x:{0.,0.,0.,1.})r["ambient"].append(x);
  r["geometry_material"]["present"]=true;r["geometry_material"]["transparency"]=0.;
  r["geometry_material"]["diffuse"]=r["diffuse"];
  r["hlms_pbs"]=r["native_opaque_blend"]=r["alpha_test_disabled"]=true;
  return r;
}
int main() {
  const auto good=fixture();assert(CheckMaterialContract(good).empty());
  for(const auto key:{"ecm_material_present","ecm_transparency_present","hlms_pbs","native_opaque_blend","alpha_test_disabled"}) {
    auto r=good;r[key]=false;assert(!CheckMaterialContract(r).empty());
  }
  auto r=good;r["ecm_transparency"]=.1;assert(!CheckMaterialContract(r).empty());
  r=good;r["diffuse"][3]=.5;assert(!CheckMaterialContract(r).empty());
  r=good;r["diffuse"][0]=Json::nullValue;assert(!CheckMaterialContract(r).empty());
  r=good;r["diffuse"][0]=1.1;assert(!CheckMaterialContract(r).empty());
  r=good;r["geometry_material"]["present"]=false;assert(!CheckMaterialContract(r).empty());
  r=good;r["geometry_material"]["transparency"]=.1;assert(!CheckMaterialContract(r).empty());
  r=good;r["geometry_material"]["diffuse"][0]=.71;assert(!CheckMaterialContract(r).empty());
  r=good;r["script_uri"]="texture";assert(!CheckMaterialContract(r).empty());
  r=good;r["pbr"]=true;assert(!CheckMaterialContract(r).empty());
  r=good;r["visual_material"]["present"]=true;r["visual_material"]["transparency"]=.1;
  assert(!CheckMaterialContract(r).empty());
}
