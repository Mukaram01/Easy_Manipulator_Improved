// Opt-in prerequisite probe only. No Gazebo world, camera render, or physics.
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/Scene.hh>
#include <jsoncpp/json/json.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

int main(int argc,char **argv) {
  if(argc!=3 || std::filesystem::exists(argv[2])) {
    std::cerr<<"Usage: stage_a_render_identity_probe ENGINE NEW_REPORT.json\n";
    return 2;
  }
  Json::Value r;
  r["schema"]="workcell_render_identity_capability/v1";
  r["requested_engine"]=argv[1];r["decision"]="BLOCKED";
  r["contact_authority"]=false;r["execution_goals"]=0;
  r["moveit_started"]=false;r["segmentation_camera_created"]=false;
  r["scope"]="standalone empty-scene API capability; no frame or association qualification";
  auto engine=ignition::rendering::engine(argv[1]);
  if(!engine)r["reason"]="RENDER_ENGINE_UNAVAILABLE";
  else {
    r["engine_name"]=engine->Name();
    auto scene=engine->CreateScene("stage_a_identity_capability_only");
    if(!scene)r["reason"]="RENDER_SCENE_UNAVAILABLE";
    else {
      auto camera=scene->CreateSegmentationCamera("identity_capability_only");
      r["segmentation_camera_created"]=static_cast<bool>(camera);
      r["reason"]=camera?"CAPABILITY_AVAILABLE_BUT_ALIGNMENT_AND_IDENTITY_UNQUALIFIED":
                          "SEGMENTATION_CAMERA_UNSUPPORTED";
      engine->DestroyScene(scene);
    }
  }
  // Record actual loaded files, not library names inferred from package versions.
  std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> paths;
  while(std::getline(maps,line)) {
    auto start=line.find('/');if(start==std::string::npos)continue;
    auto path=line.substr(start);
    if(path.find(".so")!=std::string::npos &&
       (path.find("rendering")!=std::string::npos || path.find("Ogre")!=std::string::npos))
      paths.insert(path);
  }
  r["loaded_library_paths"]=Json::Value(Json::arrayValue);
  for(const auto &path:paths)r["loaded_library_paths"].append(path);
  std::ofstream output(argv[2]);if(!output)return 2;
  output<<r<<'\n';output.close();
  if(!output)return 2;
  // Successful API creation alone must never report qualified contact evidence.
  return 2;
}
