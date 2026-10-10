// Opt-in rendering prerequisite probe. No Gazebo world, physics or authority.
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/DepthCamera.hh>
#include <ignition/rendering/SegmentationCamera.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Image.hh>
#include <jsoncpp/json/json.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

#ifdef STAGE_A_OGRE2_IMAGES
#include "render_triplet.hh"
#endif

int main(int argc,char **argv) {
  if((argc!=3 && argc!=4) || (argc==4 && std::string(argv[3])!="--images") || std::filesystem::exists(argv[2])) {
    std::cerr<<"Usage: stage_a_render_identity_probe ENGINE NEW_REPORT.json [--images]\n";
    return 2;
  }
  Json::Value r;bool production=false;
  r["schema"]="workcell_render_identity_capability/v1";
  r["requested_engine"]=argv[1];r["decision"]="BLOCKED";
  r["contact_authority"]=false;r["execution_goals"]=0;
  r["moveit_started"]=false;r["segmentation_camera_created"]=false;
  r["scope"]=argc==4?"static native render fixture; no Gazebo or physical association":"standalone empty-scene API capability; no frame or association qualification";
  #ifdef STAGE_A_OGRE2_IMAGES
  // Ogre 1 and Ogre-Next export overlapping symbols. Never load Ogre 1 into
  // the native-readback executable linked to Ogre-Next.
  if(std::string(argv[1])!="ogre2") {
    r["reason"]="RENDER_ENGINE_PROFILE_UNSUPPORTED";
    std::ofstream out(argv[2]);out<<r<<'\n';return 2;
  }
#else
  if(argc==4) {
    r["reason"]="IMAGE_PROBE_REQUIRES_ISOLATED_OGRE2_TARGET";
    std::ofstream out(argv[2]);out<<r<<'\n';return 2;
  }
#endif
  auto engine=ignition::rendering::engine(argv[1]);
  if(!engine)r["reason"]="RENDER_ENGINE_UNAVAILABLE";
  else {
    r["engine_name"]=engine->Name();
    auto scene=engine->CreateScene("stage_a_identity_capability_only");
    if(!scene)r["reason"]="RENDER_SCENE_UNAVAILABLE";
    else {
      #ifdef STAGE_A_OGRE2_IMAGES
      if(argc==4)production=images(scene,argv[2],r);
      else
#endif
      {
      auto camera=scene->CreateSegmentationCamera("identity_capability_only");
      r["segmentation_camera_created"]=static_cast<bool>(camera);
      r["reason"]=camera?"CAPABILITY_AVAILABLE_BUT_ALIGNMENT_AND_IDENTITY_UNQUALIFIED":
                          "SEGMENTATION_CAMERA_UNSUPPORTED";
      }
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
  return production?0:2;
}
