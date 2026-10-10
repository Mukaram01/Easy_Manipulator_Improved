// Original fixture geometry and acceptance; compositor shared with live opt-in capture.
#include "fragment_mrt.hh"

int main(int argc,char **argv){
  if(argc!=2)return 2;
  for(const auto &suffix:{"",".rgb8",".ids.u32"})
    if(std::filesystem::exists(std::string(argv[1])+suffix))return 2;
  static_assert(sizeof(unsigned)==4,"uint32 readback required");
  const std::string path=argv[1];Json::Value r;r["contact_authority"]=false;r["execution_goals"]=0;
  r["session"]=std::to_string(getpid())+":"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  r["schema"]="workcell_same_fragment_fixture/v1";r["decision"]="BLOCKED";
  bool ok=false;
  try{
    auto engine=ignition::rendering::engine("ogre2");if(!engine)throw std::runtime_error("missing Ogre2");
    GLint major=0,minor=0;glGetIntegerv(GL_MAJOR_VERSION,&major);glGetIntegerv(GL_MINOR_VERSION,&minor);
    if(major<4||(major==4&&minor<5))throw std::runtime_error("GL4.5 required for typed clear/attachment validation");
    auto scene=engine->CreateScene("same_fragment_only");
    auto native=std::dynamic_pointer_cast<ignition::rendering::Ogre2Scene>(scene);
    if(!native)throw std::runtime_error("missing native scene");auto sm=native->OgreSceneManager();
    auto camera=sm->createCamera("same_fragment_camera");camera->setNearClipDistance(.1);camera->setFarClipDistance(10);
    camera->setAspectRatio(1);camera->setFOVy(Ogre::Radian(1.0471975511965976));
    // Ogre 2.2 createCamera already attaches to the dynamic root (public contract).
    if(camera->getParentSceneNode()!=sm->getRootSceneNode(Ogre::SCENE_DYNAMIC))
      throw std::runtime_error("unexpected camera parent");
    auto &pm=Ogre::HighLevelGpuProgramManager::getSingleton();
    for(auto stage:{Ogre::GPT_VERTEX_PROGRAM,Ogre::GPT_FRAGMENT_PROGRAM}){
      auto p=pm.createProgram(stage==Ogre::GPT_VERTEX_PROGRAM?"fragment_vs":"fragment_fs","General","glsl",stage);
      p->setSource(stage==Ogre::GPT_VERTEX_PROGRAM?vs:fs);p->load();if(p->hasCompileError())throw std::runtime_error("shader compile error");
    }
    r["vertex_shader"]=vs;r["fragment_shader"]=fs;
    // Near cube is submitted first; farther cube overlaps it but must not replace its pixels.
    const unsigned ids[2]={16777217u,4000000022u};
    for(int i=0;i<2;++i){
      auto name="fragment_cube_"+std::to_string(i);auto mat=Ogre::MaterialManager::getSingleton().create(name,"General");
      auto pass=mat->getTechnique(0)->getPass(0);pass->setVertexProgram("fragment_vs");pass->setFragmentProgram("fragment_fs");
      pass->getVertexProgramParameters()->setNamedAutoConstant("worldViewProj",Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
      pass->getFragmentProgramParameters()->setNamedConstant("baseColour",i?Ogre::Vector3(0,1,0):Ogre::Vector3(1,0,0));
      pass->getFragmentProgramParameters()->setNamedConstant("entityId",ids[i]);
      Ogre::HlmsMacroblock macro;macro.mDepthCheck=true;macro.mDepthWrite=true;
      pass->setMacroblock(macro);Ogre::HlmsBlendblock blend;
      blend.mSourceBlendFactor=Ogre::SBF_ONE;blend.mDestBlendFactor=Ogre::SBF_ZERO;
      pass->setBlendblock(blend);
      mat->load();auto entity=sm->createEntity(Ogre::SceneManager::PT_CUBE);entity->setMaterialName(name);
      auto node=sm->getRootSceneNode()->createChildSceneNode();node->setScale(.01,.01,.01);
      node->setPosition(i?.4:-.2,0,i?-3.0:-2.0);node->attachObject(entity);
      Json::Value v;v["id"]=ids[i];v["native_entity"]=std::to_string(entity->getId());v["name"]=name;
      v["material"]=name;v["relationship"]="native fixture only; no Gazebo collision mapping";
      r["inventory"].append(v);
    }
    auto actualEntities=sm->getMovableObjectIterator("Entity");unsigned actualCount=0;
    while(actualEntities.hasMoreElements()){auto e=actualEntities.getNext();++actualCount;
      bool found=false;for(const auto &v:r["inventory"])found|=v["native_entity"].asString()==std::to_string(e->getId());
      if(!found)throw std::runtime_error("untracked fixture entity");
    }
    if(actualCount!=2)throw std::runtime_error("incomplete fixture entity inventory");
    r["inventory_complete"]=true;r["camera_id"]=Json::UInt64(camera->getId());
    auto labels=CaptureFragmentMrt(scene,sm,camera,r,path);
    std::set<unsigned> visible;unsigned counts[2]={};
    for(size_t p=0;p<labels.size();++p){visible.insert(labels[p]);
      if(labels[p]==ids[0])++counts[0];if(labels[p]==ids[1])++counts[1];
    }
    r["center_id"]=labels[128*256+128];r["visible_counts"].append(counts[0]);r["visible_counts"].append(counts[1]);
    ok=visible==std::set<unsigned>{0,ids[0],ids[1]}&&counts[0]>0&&counts[1]>0&&labels[128*256+128]==ids[0];
    r["fixture_result"]=ok?"PASS":"BLOCKED";r["physical_mapping"]="BLOCKED_NO_GAZEBO_INVENTORY";
    engine->DestroyScene(scene);
  }catch(const std::exception &e){r["failure_reason"]=e.what();ok=false;}
  std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> loaded;
  while(std::getline(maps,line)){auto p=line.find('/');if(p!=std::string::npos&&line.find(".so",p)!=std::string::npos)loaded.insert(line.substr(p));}
  for(const auto &p:loaded)r["loaded_library_paths"].append(p);
  std::ofstream output(path);output<<r<<'\n';return ok&&output?0:2;
}
