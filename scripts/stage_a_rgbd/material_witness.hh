#pragma once
#include "material_contract.hh"
#include <OgreHlms.h>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Transparency.hh>
// Read exact live ECM identity and original Item state BEFORE replacing its material.
inline Json::Value MaterialWitness(const ignition::gazebo::EntityComponentManager &ecm,
    Json::UInt64 visualId,const ignition::rendering::VisualPtr &visual) {
  Json::Value r;r["visual_id"]=visualId;
  const auto m=ecm.Component<ignition::gazebo::components::Material>(visualId);
  const auto t=ecm.Component<ignition::gazebo::components::Transparency>(visualId);
  r["ecm_material_present"]=m!=nullptr;r["ecm_transparency_present"]=t!=nullptr;
  if(t)r["ecm_transparency"]=t->Data();
  auto colour=[](const ignition::math::Color &c) {
    Json::Value v(Json::arrayValue);for(const auto x:{c.R(),c.G(),c.B(),c.A()})v.append(x);return v;
  };
  if(m) {
    r["diffuse"]=colour(m->Data().Diffuse());r["ambient"]=colour(m->Data().Ambient());
    r["script_uri"]=m->Data().ScriptUri();r["script_name"]=m->Data().ScriptName();r["pbr"]=m->Data().PbrMaterial()!=nullptr;
  }
  auto material=[&](const ignition::rendering::MaterialPtr &mat) {
    Json::Value v;v["present"]=static_cast<bool>(mat);
    if(mat){v["transparency"]=mat->Transparency();v["diffuse"]=colour(mat->Diffuse());v["material_id"]=Json::UInt64(mat->Id());}
    return v;
  };
  if(visual) {
    r["renderer_id"]=Json::UInt64(visual->Id());r["visual_material"]=material(visual->Material());
    r["geometry_count"]=visual->GeometryCount();
    const auto geometry=visual->GeometryCount()==1?visual->GeometryByIndex(0):nullptr;
    r["geometry_material"]=material(geometry?geometry->Material():nullptr);
    const auto native=std::dynamic_pointer_cast<ignition::rendering::Ogre2Geometry>(geometry);
    const auto item=native?dynamic_cast<Ogre::Item*>(native->OgreObject()):nullptr;
    if(item) {
      r["item_id"]=Json::UInt64(item->getId());r["subitem_count"]=item->getNumSubItems();
      if(item->getNumSubItems()==1) {
        const auto sub=item->getSubItem(0);const auto db=sub->getDatablock();
        r["legacy_ogre_material_present"]=!sub->getMaterial().isNull();
        r["datablock_present"]=db!=nullptr;
        if(db) {
          std::ostringstream address;address<<static_cast<const void*>(db);r["datablock_identity"]=address.str();
          const auto name=db->getNameStr();r["datablock_name"]=name?Json::Value(*name):Json::Value();
          const auto creator=db->getCreator();r["hlms_backed"]=creator!=nullptr;
          if(creator){r["hlms_type"]=int(creator->getType());r["hlms_type_name"]=creator->getTypeNameStr();}
          r["hlms_pbs"]=creator&&creator->getType()==Ogre::HLMS_PBS;
          const auto blend=db->getBlendblock();r["blendblock_present"]=blend!=nullptr;
          if(blend) {
            r["blend_source"]=int(blend->mSourceBlendFactor);r["blend_destination"]=int(blend->mDestBlendFactor);
            r["blend_transparent_flags"]=blend->mIsTransparent;r["separate_alpha_blend"]=blend->mSeparateBlend;
            r["native_opaque_blend"]=blend->mSourceBlendFactor==Ogre::SBF_ONE && blend->mDestBlendFactor==Ogre::SBF_ZERO &&
              !blend->mSeparateBlend && blend->mIsTransparent==0;
          }
          r["alpha_test"]=int(db->getAlphaTest());r["alpha_test_disabled"]=db->getAlphaTest()==Ogre::CMPF_ALWAYS_PASS;
        }
      }
    }
  }
  r["failure"]=CheckMaterialContract(r);
  r["colour_source"]="live_ECM_Material_Diffuse_exact_Gazebo_visual_id";
  return r;
}
