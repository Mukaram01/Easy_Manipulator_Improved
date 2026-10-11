#pragma once
#include <tinyxml2.h>
#include <jsoncpp/json/json.h>
#include <sstream>
#include <cmath>

namespace stage_a {
// Collision definitions only: dynamic world poses never enter this record.
inline Json::Value assetGeometry(const tinyxml2::XMLElement* world, int64_t stamp) {
  Json::Value out;
  out["source"]="live_generate_world_sdf";
  out["capture_stamp_ns"]=Json::Int64(stamp);
  out["models"]=Json::Value(Json::arrayValue);
  out["complete"]=world && world->Attribute("name") && !world->FirstChildElement("include");
  if(!world) return out;
  out["world"]=world->Attribute("name")?world->Attribute("name"):"";
  for(auto model=world->FirstChildElement("model");model;model=model->NextSiblingElement("model")) {
    Json::Value item;
    item["name"]=model->Attribute("name")?model->Attribute("name"):"";
    const auto stat=model->FirstChildElement("static");
    const std::string value=stat && stat->GetText()?stat->GetText():"false";
    item["static"]=(value=="true" || value=="1");
    if(value!="true" && value!="false" && value!="0" && value!="1") out["complete"]=false;
    if(!item["static"].asBool()) {
      auto link=model->FirstChildElement("link");
      auto collision=link?link->FirstChildElement("collision"):nullptr;
      auto geometry=collision?collision->FirstChildElement("geometry"):nullptr;
      auto box=geometry?geometry->FirstChildElement("box"):nullptr;
      auto size=box?box->FirstChildElement("size"):nullptr;
      const bool rigid=link && !link->NextSiblingElement("link") && collision &&
        !collision->NextSiblingElement("collision") && !model->FirstChildElement("model") &&
        !model->FirstChildElement("include") && !model->FirstChildElement("joint") &&
        !model->FirstChildElement("plugin") && geometry && geometry->FirstChildElement()==box &&
        box && !box->NextSiblingElement();
      item["supported"]=false;
      if(rigid && size && size->GetText() && link->Attribute("name") && collision->Attribute("name")) {
        std::istringstream input(size->GetText());
        double x,y,z;std::string extra;
        if(input>>x>>y>>z && !(input>>extra) && std::isfinite(x) && std::isfinite(y) &&
            std::isfinite(z) && x>0 && y>0 && z>0) {
          item["supported"]=true;item["link"]=link->Attribute("name");
          item["collision"]=collision->Attribute("name");
          item["dimensions_m"]=Json::Value(Json::arrayValue);
          for(double v:{x,y,z}) item["dimensions_m"].append(v);
        }
      }
    }
    out["models"].append(item);
  }
  return out;
}
}
