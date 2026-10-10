// Disposable Gazebo render owner. No ROS/EPD/planning/controller interfaces.
#include "fragment_mrt.hh"
#include "renderer_identity.hh"
#include "physics_owner/identity_inventory.hh"
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/rendering/ogre2/Ogre2Geometry.hh>
#include <ignition/rendering/ogre2/Ogre2Visual.hh>
#include <ignition/rendering/Material.hh>
#include <OgreItem.h>
#include <OgreSubItem.h>
#include <thread>

namespace g=ignition::gazebo;
namespace rd=ignition::rendering;
const char *liveFs=R"(#version 330 core
uniform vec3 baseColour;
uniform uint entityId;
layout(location=0) out vec4 rgb;
layout(location=1) out uint identity;
void main(){ rgb=vec4(baseColour,1.0);identity=entityId; }
)";

struct LiveCapture final:g::System,g::ISystemPostUpdate {
  g::RenderUtil render;
  std::string ownerPath,path,session;
  Json::Value record;
  bool initialized=false,done=false,ok=false;
  unsigned epoch=0;
  std::thread::id renderThread;
  LiveCapture(std::string owner,std::string output,std::string run):ownerPath(owner),path(output),session(run) {
    record["session"]=session;record["schema"]="workcell_live_fragment/v1";
    record["contact_authority"]=false;record["execution_goals"]=0;
    record["timing_authority"]="BLOCKED";record["render_owner"]="GazeboRenderUtil";
    record["rgb_profile"]="opaque_unlit_diffuse_MRT";
    record["stock_rgb_equivalence"]="NOT_CLAIMED_DISPOSABLE_UNLIT_RGB_PATH";
    record["geometry_source"]="SceneManager::VisualById::GeometryByIndex::OgreObject";
  }
  void PostUpdate(const g::UpdateInfo &info,const g::EntityComponentManager &ecm) override {
    if(done)return;
    try {
      if(!initialized) {
        renderThread=std::this_thread::get_id();
        render.SetEngineName("ogre2");render.SetSceneName("live_fragment_"+session);
        render.SetHeadlessRendering(true);render.SetEnableSensors(false);render.Init();initialized=true;
      }
      if(renderThread!=std::this_thread::get_id())throw std::runtime_error("render owner thread changed");
      render.UpdateFromECM(info,ecm);render.Update();
      Json::Value event;event["epoch"]=++epoch;event["step"]=Json::UInt64(info.iterations);
      event["stamp_ns"]=Json::Int64(info.simTime.count());
      event["renderutil_stamp_ns"]=Json::Int64(render.SimTime().count());
      event["boundary"]="PostUpdate_after_UpdateFromECM_and_Update";
      record["scene_updates"].append(event);
      if(info.iterations!=2)return;
      done=true;
      Json::Value owner;std::ifstream input(ownerPath);std::string line;
      while(std::getline(input,line)) {Json::Value candidate;std::istringstream stream(line);stream>>candidate;
        if(candidate["step"].asUInt64()==info.iterations) {
          if(!owner.isNull())throw std::runtime_error("duplicate owner step");owner=candidate;
        }
      }
      if(owner.isNull() || !owner["complete"].asBool() || owner["session"]!=session ||
         owner["stamp_ns"].asInt64()!=info.simTime.count())throw std::runtime_error("missing/stale owner record");
      const auto inventory=workcell::IdentityInventory(ecm);
      record["owner_inventory_equal"]=inventory==owner["identity_inventory"];
      if(!record["owner_inventory_equal"].asBool())throw std::runtime_error("owner/render ECM inventory changed");
      // Fingerprint is calculated over retained records by the strict CPU validator.
      auto mapping=workcell::RendererIdentity(render.SceneManager(),owner,"");
      if(!mapping["complete"].asBool())throw std::runtime_error("incomplete live SceneManager map");
      record["renderer"]=mapping;
      auto scene=render.Scene();auto native=std::dynamic_pointer_cast<rd::Ogre2Scene>(scene);
      if(!native)throw std::runtime_error("unsupported native renderer");auto sm=native->OgreSceneManager();
      GLint major=0,minor=0;glGetIntegerv(GL_MAJOR_VERSION,&major);glGetIntegerv(GL_MINOR_VERSION,&minor);
      if(major<4||(major==4&&minor<5))throw std::runtime_error("GL4.5 typed clear API required");
      const auto world=inventory["worlds"];
      if(world.size()!=1 || world[0]["id"]!=owner["world_entity"])throw std::runtime_error("world identity mismatch");
      // Controlled profile: no subset silently selected from a larger world.
      if(inventory["visuals"].size()!=2 || inventory["collisions"].size()!=2 ||
         inventory["links"].size()!=2 || inventory["models"].size()!=2 || owner["shapes"].size()!=2)
        throw std::runtime_error("unsupported/partial two-cube inventory");
      auto &pm=Ogre::HighLevelGpuProgramManager::getSingleton();
      for(auto stage:{Ogre::GPT_VERTEX_PROGRAM,Ogre::GPT_FRAGMENT_PROGRAM}) {
        auto program=pm.createProgram(stage==Ogre::GPT_VERTEX_PROGRAM?"live_vs":"live_fs","General","glsl",stage);
        program->setSource(stage==Ogre::GPT_VERTEX_PROGRAM?vs:liveFs);program->load();
        if(program->hasCompileError())throw std::runtime_error("live shader compile error");
      }
      record["vertex_shader"]=vs;record["fragment_shader"]=liveFs;
      std::set<Ogre::Item*> items;std::set<unsigned> ids;
      std::vector<std::pair<Ogre::Item*,Ogre::MaterialPtr>> bound;
      for(const auto &entry:mapping["visuals"]) {
        const auto visualId=entry["visual_id"].asUInt64(),linkId=entry["link_id"].asUInt64();
        Json::Value visualRow,collision,shape;unsigned collisions=0,visuals=0,shapes=0;
        for(const auto &v:inventory["visuals"])if(v["parent"].asUInt64()==linkId){visualRow=v;++visuals;}
        for(const auto &c:inventory["collisions"])if(c["parent"].asUInt64()==linkId){collision=c;++collisions;}
        for(const auto &s:owner["shapes"])if(s["collision_id"]==collision["id"]){shape=s;++shapes;}
        if(visuals!=1 || collisions!=1 || shapes!=1 || !visualRow["opaque"].asBool() ||
           visualRow["geometry"]["type"]!="BOX" || collision["geometry"]["type"]!="BOX" ||
           shape["shape_type"]!="BoxShape" || shape["shape_node_identity"].asString().empty())
          throw std::runtime_error("ambiguous/unsupported collision ShapeNode ownership");
        auto visual=render.SceneManager().VisualById(visualId);
        if(!visual || visual->GeometryCount()!=1)throw std::runtime_error("unsupported visual geometry inventory");
        auto geometry=std::dynamic_pointer_cast<rd::Ogre2Geometry>(visual->GeometryByIndex(0));
        auto node=std::dynamic_pointer_cast<rd::Ogre2Visual>(visual);
        auto item=geometry?dynamic_cast<Ogre::Item*>(geometry->OgreObject()):nullptr;
        if(!item || !node || geometry->Parent()!=visual || item->getParentSceneNode()!=node->Node() ||
           !items.insert(item).second || item->getNumSubItems()!=1)
          throw std::runtime_error("missing/duplicate/unattached live Ogre Item");
        const unsigned id=visual->Id();
        if(id==0 || id==4294967295u || !ids.insert(id).second)throw std::runtime_error("duplicate/invalid native uint32 ID");
        const auto material=visual->Material();
        if(!material || material->Transparency()!=0)throw std::runtime_error("unsupported native material");
        const auto colour=material->Diffuse();
        // Same live Ogre Item and geometry; only this disposable unlit material changes.
        auto mat=Ogre::MaterialManager::getSingleton().create("live_mrt_"+std::to_string(id),"General");
        auto pass=mat->getTechnique(0)->getPass(0);pass->setVertexProgram("live_vs");pass->setFragmentProgram("live_fs");
        pass->getVertexProgramParameters()->setNamedAutoConstant("worldViewProj",Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
        pass->getFragmentProgramParameters()->setNamedConstant("baseColour",Ogre::Vector3(colour.R(),colour.G(),colour.B()));
        pass->getFragmentProgramParameters()->setNamedConstant("entityId",id);
        Ogre::HlmsMacroblock macro;macro.mDepthCheck=true;macro.mDepthWrite=true;pass->setMacroblock(macro);
        Ogre::HlmsBlendblock blend;blend.mSourceBlendFactor=Ogre::SBF_ONE;blend.mDestBlendFactor=Ogre::SBF_ZERO;pass->setBlendblock(blend);
        mat->load();item->setMaterial(mat);bound.emplace_back(item,mat);
        Json::Value b=entry;b["collision_id"]=collision["id"];b["physics_shape_id"]=shape["physics_shape_id"];
        b["shape_node_identity"]=shape["shape_node_identity"];b["item_id"]=Json::UInt64(item->getId());
        b["geometry_id"]=visual->GeometryByIndex(0)->Id();b["original_item_reused"]=true;
        record["draw_bindings"].append(b);
        Json::Value v;v["id"]=id;v["native_entity"]=std::to_string(item->getId());record["inventory"].append(v);
      }
      auto actual=sm->getMovableObjectIterator("Item");unsigned count=0;
      while(actual.hasMoreElements()){++count;if(!items.count(dynamic_cast<Ogre::Item*>(actual.getNext())))throw std::runtime_error("unexpected renderer Item");}
      if(count!=items.size())throw std::runtime_error("incomplete Item inventory");
      if(sm->getMovableObjectIterator("Entity").hasMoreElements())throw std::runtime_error("unexpected legacy renderer geometry");
      record["inventory_complete"]=true;
      auto camera=sm->createCamera("live_mrt_camera");
      if(camera->getParentSceneNode()!=sm->getRootSceneNode(Ogre::SCENE_DYNAMIC))throw std::runtime_error("unexpected camera attachment");
      camera->setPosition(.40,-.217,.35);camera->setNearClipDistance(.01);camera->setFarClipDistance(2);
      camera->setAspectRatio(1);camera->setFOVy(Ogre::Radian(1.0471975511965976));
      record["camera_id"]=Json::UInt64(camera->getId());
      Json::Value acquisition;acquisition["session"]=session;acquisition["world_entity"]=world[0]["id"];
      acquisition["step"]=Json::UInt64(info.iterations);acquisition["stamp_ns"]=Json::Int64(info.simTime.count());
      acquisition["scene_id"]=scene->Id();acquisition["frame"]=1;acquisition["update_epoch"]=epoch;
      acquisition["phase"]="PostUpdate_blocking_capture_no_subsequent_server_iteration";record["acquisition"]=acquisition;
      const auto labels=CaptureFragmentMrt(scene,sm,camera,record,path);
      for(const auto &[item,mat]:bound)if(item->getSubItem(0)->getMaterial()!=mat)throw std::runtime_error("material changed during acquisition");
      std::set<unsigned> visible;
      for(auto id:labels){visible.insert(id);const auto key=std::to_string(id);record["pixel_counts"][key]=record["pixel_counts"].get(key,0).asUInt()+1;}
      ids.insert(0);if(visible!=ids)throw std::runtime_error("missing/unexpected visible IDs");
      record["native_result"]="PASS_TESTED_VISUAL_DRAW_PRODUCTION";ok=true;
    }catch(const std::exception &e){done=true;ok=false;record["failure_reason"]=e.what();}
    if(done) {
      // Capture while the live Server/physics plugin still exists, before teardown.
      std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> paths;
      while(std::getline(maps,line)){auto pos=line.find('/');if(pos!=std::string::npos&&line.find(".so",pos)!=std::string::npos)paths.insert(line.substr(pos));}
      for(const auto &p:paths)record["loaded_library_paths"].append(p);
      record["loaded_library_witness_phase"]="live_PostUpdate_before_Server_destruction";
    }
  }
};

int main(int argc,char **argv) {
  // world, owner trace, output prefix, session; caller must use a fresh directory.
  if(argc!=5)return 2;
  const std::string path=argv[3];
  for(const auto &s:{"",".rgb8",".ids.u32"})if(std::filesystem::exists(path+s))return 2;
  auto capture=std::make_shared<LiveCapture>(argv[2],path,argv[4]);
  try {
    g::ServerConfig config;if(!config.SetSdfFile(argv[1]))throw std::runtime_error("invalid disposable SDF");
    g::Server server(config);if(server.AddSystem(capture)!=std::optional<bool>(true))throw std::runtime_error("capture owner not installed");
    if(!server.Run(true,2,false))throw std::runtime_error("bounded Gazebo run failed");
  }catch(const std::exception &e){capture->ok=false;capture->record["failure_reason"]=e.what();}
  std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> loaded;
  while(std::getline(maps,line)){auto pos=line.find('/');if(pos!=std::string::npos&&line.find(".so",pos)!=std::string::npos)loaded.insert(line.substr(pos));}
  for(const auto &p:loaded)capture->record["loaded_library_paths"].append(p);
  std::ofstream output(path);output<<capture->record<<'\n';return capture->ok && output?0:2;
}
